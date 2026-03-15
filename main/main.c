/*
 * HDMI Color Bars + EDID dump — 640x480@24MHz
 * Olimex ESP32-P4-PC — ESP-IDF 5.5.x
 *
 * Ejemplo minimo de salida de video HDMI con el LT8912B.
 * Verificado en HP 23xi y TV 24W_LCD_TV.
 *
 * Patron extraido de SamCoupe/components/sim_display (produccion).
 * Valores clave (del sdkconfig del SamCoupe):
 *   PCLK    = 24 MHz
 *   LANE    = 400 Mbps
 *   DDS     = 0x10, 0x22, 0x22
 *   VFP/VBP = 1 / 17  (no los CEA estandar 10/33)
 *
 * Al arrancar vuelca el EDID del monitor por el log — util para
 * diagnostico cuando la imagen no aparece en monitores del usuario.
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_ldo_regulator.h"
#include "esp_cache.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_mipi_dsi.h"
#include "nvs_flash.h"



static const char *TAG = "colorbars";

/* ------------------------------------------------------------------ *
 *  Timings 640x480@60Hz — CEA-861 VIC1                               *
 * ------------------------------------------------------------------ */
#define HACT   640
#define VACT   480
#define HS      96
#define HFP     16
#define HBP     48
#define VS       2
#define VFP     10
#define VBP     33
#define HTOTAL  (HACT + HS + HFP + HBP)   /* 800  */
#define VTOTAL  (VACT + VS + VFP + VBP)    /* 525 — CEA-861 standard */
#define PCLK_MHZ  24
#define LANES      2
/* Valores exactos de produccion SamCoupe/sdkconfig:
 *   DSI_LANE_MBPS=400, PCLK=24MHz, DDS=0x22,0x22,0x10 */
#define LANE_MBPS  400

/* DDS freq word — valores de produccion SamCoupe (NO calcular, usar los del sdkconfig) */
#define DDS_B0  0x10
#define DDS_B1  0x22
#define DDS_B2  0x22

/* I2C */
#define I2C_PORT   1
#define SDA_GPIO   7
#define SCL_GPIO   8
#define HPD_GPIO   15

/* LT8912B I2C addresses */
#define ADDR_MAIN     0x48
#define ADDR_CEC_DSI  0x49
#define ADDR_AUDIO    0x4A

/* ------------------------------------------------------------------ *
 *  Globals                                                             *
 * ------------------------------------------------------------------ */
static i2c_master_bus_handle_t s_bus       = NULL;
static i2c_master_dev_handle_t s_dev_main  = NULL;
static i2c_master_dev_handle_t s_dev_cec   = NULL;
static i2c_master_dev_handle_t s_dev_audio = NULL;
static esp_lcd_panel_handle_t  s_panel     = NULL;
static void *s_fb[2] = {NULL, NULL};

/* ------------------------------------------------------------------ *
 *  I2C helpers                                                         *
 * ------------------------------------------------------------------ */
static esp_err_t lt_write(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(dev, buf, 2, 100);
}

static esp_err_t lt_read(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(dev, &reg, 1, val, 1, 100);
}



/* ------------------------------------------------------------------ *
 *  LT8912B init sequence — kernel Linux + SamCoupe production         *
 * ------------------------------------------------------------------ */
static esp_err_t lt8912b_init_config(void)
{
    i2c_master_dev_handle_t m = s_dev_main;
    /* Digital clock en */
    lt_write(m, 0x08, 0xFF); lt_write(m, 0x09, 0xFF);
    lt_write(m, 0x0A, 0xFF); lt_write(m, 0x0B, 0x7C);
    lt_write(m, 0x0C, 0xFF); lt_write(m, 0x42, 0x04);
    /* Tx Analog — 0xB1 kernel confirmed */
    lt_write(m, 0x31, 0xB1); lt_write(m, 0x32, 0xB1);
    lt_write(m, 0x33, 0x0E); /* HDMI TX output enable */
    lt_write(m, 0x37, 0x00); lt_write(m, 0x38, 0x22); lt_write(m, 0x60, 0x82);
    /* Cbus Analog */
    lt_write(m, 0x39, 0x45); lt_write(m, 0x3A, 0x00); lt_write(m, 0x3B, 0x00);
    /* HDMI PLL Analog */
    lt_write(m, 0x44, 0x31); lt_write(m, 0x55, 0x44);
    lt_write(m, 0x57, 0x01); lt_write(m, 0x5A, 0x02);
    /* MIPI Analog */
    lt_write(m, 0x3E, 0xD6); lt_write(m, 0x3F, 0xD4); lt_write(m, 0x41, 0x3C);
    /* HDMI mode */
    lt_write(m, 0xB2, 0x01);
    return ESP_OK;
}

static esp_err_t lt8912b_mipi_basic(void)
{
    i2c_master_dev_handle_t d = s_dev_cec;
    uint8_t settle = (VACT <= 600) ? 0x04 : (VACT == 1080) ? 0x0A : 0x08;
    ESP_LOGI(TAG, "settle=0x%02x", settle);
    lt_write(d, 0x10, 0x01);   /* term_en */
    lt_write(d, 0x11, settle); /* settle */
    lt_write(d, 0x12, 0x04);   /* trail */
    lt_write(d, 0x13, 0x02);   /* 2 lanes */
    lt_write(d, 0x14, 0x00);
    lt_write(d, 0x15, 0x00);   /* no swap */
    lt_write(d, 0x1A, 0x03);
    lt_write(d, 0x1B, 0x03);
    return ESP_OK;
}

static esp_err_t lt8912b_video_timing(void)
{
    i2c_master_dev_handle_t d = s_dev_cec;
    i2c_master_dev_handle_t m = s_dev_main;
    ESP_LOGI(TAG, "Video: %dx%d htotal=%d vtotal=%d pclk=%dMHz",
             HACT, VACT, HTOTAL, VTOTAL, PCLK_MHZ);
    lt_write(d, 0x18, HS);
    lt_write(d, 0x19, VS);
    lt_write(d, 0x1C, HACT & 0xFF); lt_write(d, 0x1D, HACT >> 8);
    lt_write(d, 0x2F, 0x0C);
    lt_write(d, 0x34, HTOTAL & 0xFF); lt_write(d, 0x35, HTOTAL >> 8);
    lt_write(d, 0x36, VTOTAL & 0xFF); lt_write(d, 0x37, VTOTAL >> 8);
    lt_write(d, 0x38, VBP & 0xFF); lt_write(d, 0x39, VBP >> 8);
    lt_write(d, 0x3A, VFP & 0xFF); lt_write(d, 0x3B, VFP >> 8);
    lt_write(d, 0x3C, HBP & 0xFF); lt_write(d, 0x3D, HBP >> 8);
    lt_write(d, 0x3E, HFP & 0xFF); lt_write(d, 0x3F, HFP >> 8);
    /* 640x480: polaridades negativas (CEA-861 VIC1) */
    lt_write(m, 0xAB, 0x00);
    /* HDMI mode */
    lt_write(m, 0xB2, 0x01);
    return ESP_OK;
}

static esp_err_t lt8912b_avi_infoframe(void)
{
    /* VIC=1 para 640x480@60Hz, 4:3 */
    i2c_master_dev_handle_t a = s_dev_audio;
    uint8_t vic = 1;
    uint8_t aspect = 1;  /* 4:3 */
    uint8_t pb2 = (aspect << 4) + 0x08;
    uint8_t pb4 = vic;
    uint8_t pb0 = (((pb2 + pb4) <= 0x5F) ? (0x5F - pb2 - pb4) : (0x15F - pb2 - pb4));
    ESP_LOGI(TAG, "AVI: VIC=%d pb0=%02X pb2=%02X pb4=%02X", vic, pb0, pb2, pb4);
    lt_write(a, 0x3C, 0x41);
    lt_write(a, 0x43, pb0);
    lt_write(a, 0x44, 0x10);  /* RGB */
    lt_write(a, 0x45, pb2);
    lt_write(a, 0x46, 0x00);
    lt_write(a, 0x47, pb4);
    return ESP_OK;
}

static esp_err_t lt8912b_dds_config(void)
{
    i2c_master_dev_handle_t d = s_dev_cec;
    /* DDS freq word calculado por pclk */
    /* DDS valores exactos de produccion SamCoupe para 640x480@24MHz */
    ESP_LOGI(TAG, "DDS: [0x%02X,0x%02X,0x%02X]", DDS_B0, DDS_B1, DDS_B2);
    lt_write(d, 0x4E, DDS_B0);
    lt_write(d, 0x4F, DDS_B1);
    lt_write(d, 0x50, DDS_B2);
    lt_write(d, 0x51, 0x80);
    lt_write(d, 0x1F, 0x5E); lt_write(d, 0x20, 0x01); lt_write(d, 0x21, 0x2C);
    lt_write(d, 0x22, 0x01); lt_write(d, 0x23, 0xFA); lt_write(d, 0x24, 0x00);
    lt_write(d, 0x25, 0xC8); lt_write(d, 0x26, 0x00); lt_write(d, 0x27, 0x5E);
    lt_write(d, 0x28, 0x01); lt_write(d, 0x29, 0x2C); lt_write(d, 0x2A, 0x01);
    lt_write(d, 0x2B, 0xFA); lt_write(d, 0x2C, 0x00); lt_write(d, 0x2D, 0xC8);
    lt_write(d, 0x2E, 0x00); lt_write(d, 0x42, 0x64); lt_write(d, 0x43, 0x00);
    lt_write(d, 0x44, 0x04); lt_write(d, 0x45, 0x00); lt_write(d, 0x46, 0x59);
    lt_write(d, 0x47, 0x00); lt_write(d, 0x48, 0xF2); lt_write(d, 0x49, 0x06);
    lt_write(d, 0x4A, 0x00); lt_write(d, 0x4B, 0x72); lt_write(d, 0x4C, 0x45);
    lt_write(d, 0x4D, 0x00); lt_write(d, 0x52, 0x08); lt_write(d, 0x53, 0x00);
    lt_write(d, 0x54, 0xB2); lt_write(d, 0x55, 0x00); lt_write(d, 0x56, 0xE4);
    lt_write(d, 0x57, 0x0D); lt_write(d, 0x58, 0x00); lt_write(d, 0x59, 0xE4);
    lt_write(d, 0x5A, 0x8A); lt_write(d, 0x5B, 0x00); lt_write(d, 0x5C, 0x34);
    lt_write(d, 0x1E, 0x4F);
    lt_write(d, 0x51, 0x00);  /* commit */
    return ESP_OK;
}

static esp_err_t lt8912b_rxlogicres(void)
{
    lt_write(s_dev_main, 0x03, 0x7F);
    vTaskDelay(pdMS_TO_TICKS(10));
    lt_write(s_dev_main, 0x03, 0xFF);
    return ESP_OK;
}

static esp_err_t lt8912b_lvds_config(void)
{
    i2c_master_dev_handle_t m = s_dev_main;
    lt_write(m, 0x44, 0x30); lt_write(m, 0x51, 0x05);
    lt_write(m, 0x50, 0x24); lt_write(m, 0x51, 0x2D);
    lt_write(m, 0x52, 0x04); lt_write(m, 0x69, 0x0E);
    lt_write(m, 0x69, 0x8E); lt_write(m, 0x6A, 0x00);
    lt_write(m, 0x6C, 0xB8); lt_write(m, 0x6B, 0x51);
    lt_write(m, 0x04, 0xFB); lt_write(m, 0x04, 0xFF);
    lt_write(m, 0x7F, 0x00); lt_write(m, 0xA8, 0x13);
    lt_write(m, 0x02, 0xF7); lt_write(m, 0x02, 0xFF);
    lt_write(m, 0x03, 0xCF); lt_write(m, 0x03, 0xFF);
    return ESP_OK;
}

static void lt8912b_status(void)
{
    uint8_t hs_l=0, hs_h=0, vs_l=0, vs_h=0;
    uint8_t c0=0, c1=0, c6=0, c9=0;
    lt_read(s_dev_main, 0x9C, &hs_l); lt_read(s_dev_main, 0x9D, &hs_h);
    lt_read(s_dev_main, 0x9E, &vs_l); lt_read(s_dev_main, 0x9F, &vs_h);
    lt_read(s_dev_main, 0xC0, &c0);   lt_read(s_dev_main, 0xC1, &c1);
    lt_read(s_dev_main, 0xC6, &c6);   lt_read(s_dev_main, 0xC9, &c9);
    ESP_LOGI(TAG, "MIPI sync: Hsync=0x%02X%02X Vsync=0x%02X%02X%s",
             hs_h, hs_l, vs_h, vs_l,
             (hs_l||hs_h||vs_l||vs_h) ? " — DSI activo" : " — SIN SEÑAL DSI");
    ESP_LOGI(TAG, "Status: C0=%02X C1=%02X C6=%02X C9=%02X  HPD=%d CLK=%d PLL=%d",
             c0, c1, c6, c9,
             !!(c1&0x80), !!(c1&0x40), !!(c6&0x80));
}

/* ------------------------------------------------------------------ *
 *  Color bars RGB888 directas al framebuffer                          *
 * ------------------------------------------------------------------ */
static const uint8_t k_rgb[10][3] = {
    {255,0,0},{255,128,0},{255,255,0},{0,255,0},{0,255,255},
    {0,0,255},{255,0,255},{128,0,255},{255,255,255},{0,0,0},
};

static void fill_color_bars(uint8_t *buf)
{
    const int N = 10;
    int bw = HACT / N;
    for (int y = 0; y < VACT; y++) {
        for (int x = 0; x < HACT; x++) {
            int b = x / bw; if (b >= N) b = N-1;
            uint8_t *p = buf + (y * HACT + x) * 3;
            p[0] = k_rgb[b][0];
            p[1] = k_rgb[b][1];
            p[2] = k_rgb[b][2];
        }
    }
}

/* ------------------------------------------------------------------ *
 *  EDID read y dump                                                    *
 * ------------------------------------------------------------------ */
static void read_edid(void)
{
    i2c_device_config_t ddc_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = 0x50,
        .scl_speed_hz    = 100000,
    };
    i2c_master_dev_handle_t ddc = NULL;
    if (i2c_master_bus_add_device(s_bus, &ddc_cfg, &ddc) != ESP_OK) {
        ESP_LOGW(TAG, "No se pudo crear device DDC 0x50");
        return;
    }
    uint8_t edid[128] = {0};
    uint8_t reg0 = 0x00;
    esp_err_t err = i2c_master_transmit_receive(ddc, &reg0, 1, edid, 128, pdMS_TO_TICKS(300));
    i2c_master_bus_rm_device(ddc);

    if (err != ESP_OK) { ESP_LOGW(TAG, "EDID error: %s", esp_err_to_name(err)); return; }
    static const uint8_t hdr[8] = {0,0xff,0xff,0xff,0xff,0xff,0xff,0};
    if (memcmp(edid, hdr, 8) != 0) { ESP_LOGW(TAG, "EDID cabecera invalida"); return; }

    char mfr[4] = {
        '@' + ((edid[8]>>2) & 0x1f),
        '@' + (((edid[8]&3)<<3) | (edid[9]>>5)),
        '@' + (edid[9] & 0x1f), 0
    };
    ESP_LOGI(TAG, "=== EDID === %s v%d.%d  %dcmx%dcm",
             mfr, edid[18], edid[19], edid[21], edid[22]);
    ESP_LOGI(TAG, "Est.timings: 0x%02x 0x%02x 0x%02x",edid[35],edid[36],edid[37]);
    if (edid[35]&0x20) ESP_LOGI(TAG, "  640x480@60 soportado");
    if (edid[35]&0x01) ESP_LOGI(TAG, "  800x600@60 soportado");
    if (edid[36]&0x08) ESP_LOGI(TAG, "  1024x768@60 soportado");

    for (int i = 0; i < 4; i++) {
        const uint8_t *d = edid + 54 + i*18;
        uint16_t pclk = d[0] | (d[1]<<8);
        if (!pclk) {
            if (d[3]==0xfc) { char n[14]={0}; memcpy(n,d+5,13); ESP_LOGI(TAG,"Nombre: %s",n); }
            if (d[3]==0xff) { char s[14]={0}; memcpy(s,d+5,13); ESP_LOGI(TAG,"Serie:  %s",s); }
            if (d[3]==0xfd) ESP_LOGI(TAG,"Rango: V=%d-%dHz H=%d-%dKHz pclk_max=%dMHz",
                             d[5],d[6],d[7],d[8],d[9]*10);
            continue;
        }
        uint16_t hact=d[2]|((d[4]&0xf0)<<4), hblnk=d[3]|((d[4]&0x0f)<<8);
        uint16_t vact=d[5]|((d[7]&0xf0)<<4), vblnk=d[6]|((d[7]&0x0f)<<8);
        uint16_t hfp=d[8]|((d[11]&0xc0)<<2), hpw=d[9]|((d[11]&0x30)<<4);
        uint16_t vfp=((d[10]>>4)&0xf)|((d[11]&0x0c)<<2), vpw=(d[10]&0xf)|((d[11]&0x03)<<4);
        uint32_t pkhz=(uint32_t)pclk*10;
        uint32_t ht=hact+hblnk, vt=vact+vblnk;
        ESP_LOGI(TAG,"DTD[%d]: %dx%d@%luHz pclk=%luKHz hfp=%d hpw=%d hbp=%d vfp=%d vpw=%d vbp=%d",
                 i,hact,vact,(unsigned long)(pkhz*1000/ht/vt),(unsigned long)pkhz,
                 hfp,hpw,(int)(hblnk-hfp-hpw),vfp,vpw,(int)(vblnk-vfp-vpw));
    }
    ESP_LOGI(TAG,"--- hex ---");
    for (int r=0;r<8;r++) {
        ESP_LOGI(TAG,"[%02x] %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
                 r*16,edid[r*16],edid[r*16+1],edid[r*16+2],edid[r*16+3],
                 edid[r*16+4],edid[r*16+5],edid[r*16+6],edid[r*16+7],
                 edid[r*16+8],edid[r*16+9],edid[r*16+10],edid[r*16+11],
                 edid[r*16+12],edid[r*16+13],edid[r*16+14],edid[r*16+15]);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ------------------------------------------------------------------ */

void app_main(void)
{
    ESP_LOGI(TAG, "Color bars 640x480@%dMHz lane=%dMbps DVI-mode", PCLK_MHZ, LANE_MBPS);

    /* NVS — no estrictamente necesario aqui pero buena practica */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase(); nvs_flash_init();
    }

    /* ---- Step 0: LDO3 → MIPI DPHY power ---- */
    {
        esp_ldo_channel_handle_t ldo = NULL;
        esp_ldo_channel_config_t lcfg = { .chan_id = 3, .voltage_mv = 2500 };
        if (esp_ldo_acquire_channel(&lcfg, &ldo) == ESP_OK)
            ESP_LOGI(TAG, "LDO3 @ 2500mV OK");
        else
            ESP_LOGW(TAG, "LDO3 failed — external supply?");
    }

    /* ---- Step 1: DSI bus ---- */
    ESP_LOGI(TAG, "Step 1: DSI bus @ %d Mbps/lane", LANE_MBPS);
    esp_lcd_dsi_bus_handle_t dsi_bus = NULL;
    esp_lcd_dsi_bus_config_t bus_cfg = {
        .bus_id             = 0,
        .num_data_lanes     = LANES,
        .phy_clk_src        = MIPI_DSI_PHY_CLK_SRC_DEFAULT,
        .lane_bit_rate_mbps = LANE_MBPS,
    };
    ESP_ERROR_CHECK(esp_lcd_new_dsi_bus(&bus_cfg, &dsi_bus));



    /* ---- Step 3: I2C bus + LT8912B probe ---- */
    ESP_LOGI(TAG, "Step 3: I2C + LT8912B");
    i2c_master_bus_config_t i2c_cfg = {
        .i2c_port            = I2C_PORT,
        .sda_io_num          = SDA_GPIO,
        .scl_io_num          = SCL_GPIO,
        .clk_source          = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt   = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_cfg, &s_bus));

    /* Scan */
    ESP_LOGI(TAG, "I2C scan:");
    for (uint8_t a = 0x08; a <= 0x77; a++) {
        if (i2c_master_probe(s_bus, a, 20) == ESP_OK)
            ESP_LOGI(TAG, "  0x%02X", a);
    }

    i2c_device_config_t dev_cfg = { .scl_speed_hz = 400000, .device_address = ADDR_MAIN };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev_main));
    dev_cfg.device_address = ADDR_CEC_DSI;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev_cec));
    dev_cfg.device_address = ADDR_AUDIO;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev_audio));

    /* Verificar chip ID */
    uint8_t id_h=0, id_l=0;
    lt_read(s_dev_main, 0x00, &id_h);
    lt_read(s_dev_main, 0x01, &id_l);
    ESP_LOGI(TAG, "LT8912B ID: 0x%02X%02X %s", id_h, id_l,
             (id_h==0x12 && id_l==0xB2) ? "OK" : "MISMATCH");

    /* HPD */
    gpio_config_t hpd_cfg = {
        .pin_bit_mask = (1ULL << HPD_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&hpd_cfg);

    /* EDID — leer antes de init para diagnostico */
    vTaskDelay(pdMS_TO_TICKS(200));
    read_edid();

    /* ---- Step 4: DPI panel ---- */
    ESP_LOGI(TAG, "Step 4: DPI panel %dx%d @ %dMHz", HACT, VACT, PCLK_MHZ);
    esp_lcd_dpi_panel_config_t dpi_cfg = {
        .dpi_clk_src         = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
        .dpi_clock_freq_mhz  = PCLK_MHZ,
        .pixel_format        = LCD_COLOR_PIXEL_FORMAT_RGB888,
        .num_fbs             = 2,
        .video_timing = {
            .h_size            = HACT,
            .v_size            = VACT,
            .hsync_pulse_width = HS,
            .hsync_back_porch  = HBP,
            .hsync_front_porch = HFP,
            .vsync_pulse_width = VS,
            .vsync_back_porch  = VBP,
            .vsync_front_porch = VFP,
        },
        .flags.disable_lp = 1,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_dpi(dsi_bus, &dpi_cfg, &s_panel));

    /* ---- Step 5: Framebuffers, barras, panel enable ---- */
    ESP_LOGI(TAG, "Step 5: Framebuffers + panel enable");
    size_t fb_size = HACT * VACT * 3;
    if (esp_lcd_dpi_panel_get_frame_buffer(s_panel, 2, &s_fb[0], &s_fb[1]) != ESP_OK) {
        esp_lcd_dpi_panel_get_frame_buffer(s_panel, 1, &s_fb[0]);
        s_fb[1] = NULL;
    }
    ESP_LOGI(TAG, "FB: fb0=%p fb1=%p (%u KB)", s_fb[0], s_fb[1], (unsigned)(fb_size/1024));

    /* Dibujar barras en ambos buffers */
    for (int i = 0; i < 2; i++) {
        if (!s_fb[i]) continue;
        fill_color_bars((uint8_t *)s_fb[i]);
        esp_cache_msync(s_fb[i], fb_size,
            ESP_CACHE_MSYNC_FLAG_DIR_C2M |
            ESP_CACHE_MSYNC_FLAG_TYPE_DATA |
            ESP_CACHE_MSYNC_FLAG_UNALIGNED);
    }

    /* Panel enable */
    esp_lcd_panel_init(s_panel);

    /* ---- Step 6: LT8912B full init (post DPI) ---- */
    ESP_LOGI(TAG, "Step 6: LT8912B post-DPI init");
    lt8912b_init_config();
    lt8912b_mipi_basic();
    lt8912b_video_timing();
    lt8912b_avi_infoframe();
    lt8912b_dds_config();
    lt8912b_rxlogicres();
    lt8912b_lvds_config();

    vTaskDelay(pdMS_TO_TICKS(200));
    lt8912b_status();

    ESP_LOGI(TAG, "Init completo — barras en pantalla");

    /* Monitorizar HPD — si el monitor se desconecta y reconecta,
     * reinicializar el LT8912B para que vuelva a sincronizar */
    bool was_connected = true;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));

        uint8_t c1 = 0;
        lt_read(s_dev_main, 0xC1, &c1);
        bool connected = !!(c1 & 0x80);

        if (connected && !was_connected) {
            ESP_LOGI(TAG, "HPD: monitor reconectado — reinicializando LT8912B");
            lt8912b_init_config();
            lt8912b_mipi_basic();
            lt8912b_video_timing();
            lt8912b_avi_infoframe();
            lt8912b_dds_config();
            lt8912b_rxlogicres();
            lt8912b_lvds_config();
            vTaskDelay(pdMS_TO_TICKS(200));
            /* Re-leer EDID del nuevo monitor */
            read_edid();
        } else if (!connected && was_connected) {
            ESP_LOGW(TAG, "HPD: monitor desconectado");
        }

        was_connected = connected;

        if (connected) {
            lt8912b_status();
        }
    }
}
