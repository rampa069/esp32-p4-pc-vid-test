/*
 * HDMI Color Bars + EDID auto-mode
 * Olimex ESP32-P4-PC — ESP-IDF 5.5.x
 *
 * Lee el EDID del monitor y selecciona automáticamente el mejor modo
 * compatible con el PHY del ESP32-P4 (pclk <= 62 MHz con 2 lanes a
 * 1500 Mbps máx, mínimo ~20 MHz para que el PHY arranque).
 *
 * Si no hay EDID o ningún modo encaja, cae a 640x480@60Hz (CEA VIC1).
 *
 * Al arrancar vuelca el EDID completo por el log — herramienta de
 * diagnóstico para cuando el usuario reporta que no ve imagen.
 */

#include <string.h>
#include <stdlib.h>
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
 *  Limits for the ESP32-P4 MIPI DSI PHY                               *
 *  2 data lanes, max 1500 Mbps/lane                                   *
 *  lane_mbps = pclk_mhz * 24 / 2  =>  max pclk = 125 MHz             *
 *  PHY min lane rate ~80 Mbps  =>  min pclk ~ 7 MHz (use 20 as safe) *
 * ------------------------------------------------------------------ */
#define PHY_MAX_PCLK_MHZ   90   /* 90MHz * 24/2 = 1080 Mbps/lane, safe for rev v1.0 */
#define PHY_MIN_PCLK_MHZ   20

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
 *  Video timing — filled from EDID or fallback                        *
 * ------------------------------------------------------------------ */
typedef struct {
    uint16_t hact, htotal, hs, hfp, hbp;
    uint16_t vact, vtotal, vs, vfp, vbp;
    uint32_t pclk_khz;
    uint8_t  vic;          /* CEA VIC, 0 if unknown */
    uint8_t  aspect_16_9;  /* 1 = 16:9, 0 = 4:3 */
} video_timing_t;

/* Fallback: 640x480@60Hz CEA VIC1 */
static const video_timing_t k_fallback = {
    .hact=640, .hs=96, .hfp=16, .hbp=48, .htotal=800,
    .vact=480, .vs=2,  .vfp=10, .vbp=33, .vtotal=525,
    .pclk_khz=25175, .vic=1, .aspect_16_9=0,
};

static video_timing_t s_timing;  /* chosen timing */

/* ------------------------------------------------------------------ *
 *  Globals                                                             *
 * ------------------------------------------------------------------ */
static i2c_master_bus_handle_t  s_bus       = NULL;
static i2c_master_dev_handle_t  s_dev_main  = NULL;
static i2c_master_dev_handle_t  s_dev_cec   = NULL;
static i2c_master_dev_handle_t  s_dev_audio = NULL;
static esp_lcd_dsi_bus_handle_t s_dsi_bus   = NULL;
static esp_lcd_panel_handle_t   s_panel     = NULL;
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
 *  EDID — read 128 bytes from DDC (I2C 0x50)                         *
 * ------------------------------------------------------------------ */
static bool edid_read(uint8_t *buf)
{
    i2c_device_config_t ddc_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = 0x50,
        .scl_speed_hz    = 100000,
    };
    i2c_master_dev_handle_t ddc = NULL;
    if (i2c_master_bus_add_device(s_bus, &ddc_cfg, &ddc) != ESP_OK)
        return false;

    uint8_t reg0 = 0x00;
    bool ok = (i2c_master_transmit_receive(ddc, &reg0, 1, buf, 128,
                                           pdMS_TO_TICKS(300)) == ESP_OK);
    i2c_master_bus_rm_device(ddc);

    if (!ok) return false;
    static const uint8_t hdr[8] = {0,0xff,0xff,0xff,0xff,0xff,0xff,0};
    return (memcmp(buf, hdr, 8) == 0);
}

/* ------------------------------------------------------------------ *
 *  EDID — dump to log                                                  *
 * ------------------------------------------------------------------ */
static void edid_dump(const uint8_t *e)
{
    char mfr[4] = {
        '@' + ((e[8]>>2) & 0x1f),
        '@' + (((e[8]&3)<<3) | (e[9]>>5)),
        '@' + (e[9] & 0x1f), 0
    };
    ESP_LOGI(TAG, "=== EDID === %s v%d.%d  %dcmx%dcm",
             mfr, e[18], e[19], e[21], e[22]);
    ESP_LOGI(TAG, "Est.timings: 0x%02x 0x%02x 0x%02x", e[35], e[36], e[37]);
    if (e[35]&0x20) ESP_LOGI(TAG, "  640x480@60  soportado");
    if (e[35]&0x01) ESP_LOGI(TAG, "  800x600@60  soportado");
    if (e[36]&0x08) ESP_LOGI(TAG, "  1024x768@60 soportado");

    for (int i = 0; i < 4; i++) {
        const uint8_t *d = e + 54 + i*18;
        uint16_t pclk = d[0] | (d[1]<<8);
        if (!pclk) {
            if (d[3]==0xfc) { char n[14]={0}; memcpy(n,d+5,13);
                for(int j=12;j>=0;j--){if(n[j]=='\n'||n[j]==' ')n[j]=0;else break;}
                ESP_LOGI(TAG,"Nombre: %s",n); }
            if (d[3]==0xff) { char s[14]={0}; memcpy(s,d+5,13);
                for(int j=12;j>=0;j--){if(s[j]=='\n'||s[j]==' ')s[j]=0;else break;}
                ESP_LOGI(TAG,"Serie:  %s",s); }
            if (d[3]==0xfd) ESP_LOGI(TAG,"Rango: V=%d-%dHz H=%d-%dKHz pclk_max=%dMHz",
                             d[5],d[6],d[7],d[8],d[9]*10);
            continue;
        }
        uint16_t hact  = d[2]|((d[4]&0xf0)<<4), hblnk=d[3]|((d[4]&0x0f)<<8);
        uint16_t vact  = d[5]|((d[7]&0xf0)<<4), vblnk=d[6]|((d[7]&0x0f)<<8);
        uint16_t hfp   = d[8]|((d[11]&0xc0)<<2), hpw=d[9]|((d[11]&0x30)<<4);
        uint16_t vfp   = ((d[10]>>4)&0xf)|((d[11]&0x0c)<<2);
        uint16_t vpw   = (d[10]&0xf)|((d[11]&0x03)<<4);
        uint32_t pkhz  = (uint32_t)pclk*10;
        uint32_t ht    = hact+hblnk, vt=vact+vblnk;
        uint32_t fps   = (ht&&vt) ? (pkhz*1000/ht/vt) : 0;
        ESP_LOGI(TAG,"DTD[%d]: %dx%d@%luHz pclk=%luKHz hfp=%d hpw=%d hbp=%d vfp=%d vpw=%d vbp=%d",
                 i,hact,vact,(unsigned long)fps,(unsigned long)pkhz,
                 hfp,hpw,(int)(hblnk-hfp-hpw),vfp,vpw,(int)(vblnk-vfp-vpw));
    }

    ESP_LOGI(TAG,"--- hex ---");
    for (int r=0;r<8;r++) {
        ESP_LOGI(TAG,"[%02x] %02x %02x %02x %02x %02x %02x %02x %02x"
                 " %02x %02x %02x %02x %02x %02x %02x %02x", r*16,
                 e[r*16],e[r*16+1],e[r*16+2],e[r*16+3],
                 e[r*16+4],e[r*16+5],e[r*16+6],e[r*16+7],
                 e[r*16+8],e[r*16+9],e[r*16+10],e[r*16+11],
                 e[r*16+12],e[r*16+13],e[r*16+14],e[r*16+15]);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

/* ------------------------------------------------------------------ *
 *  EDID — select best timing compatible with ESP32-P4 PHY             *
 *                                                                      *
 *  Priority:                                                           *
 *   1. DTD entries (higher resolution = better, if pclk fits PHY)     *
 *   2. Known CEA modes from established timings                        *
 *   3. Fallback 640x480                                                *
 * ------------------------------------------------------------------ */
static void edid_select_mode(const uint8_t *e)
{
    video_timing_t best = {0};
    uint32_t best_pixels = 0;

    /* --- Scan DTD blocks --- */
    for (int i = 0; i < 4; i++) {
        const uint8_t *d = e + 54 + i*18;
        uint16_t pclk_10khz = d[0] | (d[1]<<8);
        if (!pclk_10khz) continue;

        uint16_t hact  = d[2]|((d[4]&0xf0)<<4);
        uint16_t hblnk = d[3]|((d[4]&0x0f)<<8);
        uint16_t vact  = d[5]|((d[7]&0xf0)<<4);
        uint16_t vblnk = d[6]|((d[7]&0x0f)<<8);
        uint16_t hfp   = d[8]|((d[11]&0xc0)<<2);
        uint16_t hpw   = d[9]|((d[11]&0x30)<<4);
        uint16_t vfp   = ((d[10]>>4)&0x0f)|((d[11]&0x0c)<<2);
        uint16_t vpw   = (d[10]&0x0f)|((d[11]&0x03)<<4);
        uint16_t hbp   = hblnk - hfp - hpw;
        uint16_t vbp   = vblnk - vfp - vpw;
        uint32_t pclk_khz = (uint32_t)pclk_10khz * 10;
        uint32_t pclk_mhz = pclk_khz / 1000;

        /* Check PHY limits */
        if (pclk_mhz < PHY_MIN_PCLK_MHZ || pclk_mhz > PHY_MAX_PCLK_MHZ) {
            ESP_LOGI(TAG, "  DTD[%d] %dx%d pclk=%luMHz — fuera de rango PHY, saltando",
                     i, hact, vact, (unsigned long)pclk_mhz);
            continue;
        }

        /* Skip interlaced/half-frame modes: vact not multiple of 8,
         * or suspicious aspect (1920x540 is 1080i half-frame) */
        if (vact & 7) {
            ESP_LOGI(TAG, "  DTD[%d] %dx%d — vact no múltiplo de 8, posible entrelazado, saltando",
                     i, hact, vact);
            continue;
        }
        /* 1920x540 specifically: looks like 1080i/2, skip */
        if (hact == 1920 && vact == 540) {
            ESP_LOGI(TAG, "  DTD[%d] 1920x540 — modo entrelazado 1080i, saltando", i);
            continue;
        }

        uint32_t pixels = (uint32_t)hact * vact;
        if (pixels > best_pixels) {
            best_pixels = pixels;
            best.hact=hact; best.htotal=hact+hblnk;
            best.hs=hpw;    best.hfp=hfp; best.hbp=hbp;
            best.vact=vact; best.vtotal=vact+vblnk;
            best.vs=vpw;    best.vfp=vfp; best.vbp=vbp;
            best.pclk_khz=pclk_khz;
            /* Aspect from image size: width/height > 1.4 => 16:9 */
        best.aspect_16_9 = (hact * 10 / (vact ? vact : 1) > 14) ? 1 : 0;
            best.vic = 0;  /* unknown CEA VIC for custom modes */
            ESP_LOGI(TAG, "  DTD[%d] %dx%d@%luMHz — candidato",
                     i, hact, vact, (unsigned long)pclk_mhz);
        }
    }

    /* --- Established timings fallback (CEA known modes) ---
     * Only used if no DTD fitted the PHY */
    if (best_pixels == 0) {
        /* 1024x768@60Hz: pclk=65MHz -> over limit, skip */
        /* 800x600@60Hz:  pclk=40MHz -> fits */
        if (e[35] & 0x01) {  /* 800x600@60Hz */
            ESP_LOGI(TAG, "  Usando established timing: 800x600@60Hz");
            best = (video_timing_t){
                .hact=800,.hs=128,.hfp=40,.hbp=88,.htotal=1056,
                .vact=600,.vs=4,  .vfp=1, .vbp=23,.vtotal=628,
                .pclk_khz=40000,.vic=0,.aspect_16_9=0
            };
            best_pixels = 800*600;
        } else if (e[35] & 0x20) {  /* 640x480@60Hz */
            ESP_LOGI(TAG, "  Usando established timing: 640x480@60Hz");
            best = k_fallback;
            best_pixels = 640*480;
        }
    }

    if (best_pixels == 0) {
        ESP_LOGW(TAG, "Ningun modo EDID compatible — usando fallback 640x480");
        s_timing = k_fallback;
    } else {
        s_timing = best;
        /* Assign VIC and aspect for known CEA modes */
        if (s_timing.hact==640  && s_timing.vact==480)  { s_timing.vic=1;  s_timing.aspect_16_9=0; }
        if (s_timing.hact==720  && s_timing.vact==480)  { s_timing.vic=2;  s_timing.aspect_16_9=0; }
        if (s_timing.hact==1280 && s_timing.vact==720)  { s_timing.vic=4;  s_timing.aspect_16_9=1; }
        if (s_timing.hact==1920 && s_timing.vact==1080) { s_timing.vic=16; s_timing.aspect_16_9=1; }
        if (s_timing.hact==800  && s_timing.vact==600)  { s_timing.vic=0;  s_timing.aspect_16_9=0; }
    }

    ESP_LOGI(TAG, "Modo seleccionado: %dx%d pclk=%luKHz htotal=%d vtotal=%d VIC=%d",
             s_timing.hact, s_timing.vact, (unsigned long)s_timing.pclk_khz,
             s_timing.htotal, s_timing.vtotal, s_timing.vic);
}

/* ------------------------------------------------------------------ *
 *  LT8912B init                                                        *
 * ------------------------------------------------------------------ */
static esp_err_t lt8912b_init_config(void)
{
    i2c_master_dev_handle_t m = s_dev_main;
    lt_write(m, 0x08, 0xFF); lt_write(m, 0x09, 0xFF);
    lt_write(m, 0x0A, 0xFF); lt_write(m, 0x0B, 0x7C);
    lt_write(m, 0x0C, 0xFF); lt_write(m, 0x42, 0x04);
    lt_write(m, 0x31, 0xB1); lt_write(m, 0x32, 0xB1);
    lt_write(m, 0x33, 0x0E);
    lt_write(m, 0x37, 0x00); lt_write(m, 0x38, 0x22); lt_write(m, 0x60, 0x82);
    lt_write(m, 0x39, 0x45); lt_write(m, 0x3A, 0x00); lt_write(m, 0x3B, 0x00);
    lt_write(m, 0x44, 0x31); lt_write(m, 0x55, 0x44);
    lt_write(m, 0x57, 0x01); lt_write(m, 0x5A, 0x02);
    lt_write(m, 0x3E, 0xD6); lt_write(m, 0x3F, 0xD4); lt_write(m, 0x41, 0x3C);
    lt_write(m, 0xB2, 0x01);  /* HDMI mode */
    return ESP_OK;
}

static esp_err_t lt8912b_mipi_basic(void)
{
    i2c_master_dev_handle_t d = s_dev_cec;
    uint8_t settle = (s_timing.vact <= 600) ? 0x04 :
                     (s_timing.vact == 1080) ? 0x0A : 0x08;
    ESP_LOGI(TAG, "settle=0x%02x", settle);
    lt_write(d, 0x10, 0x01);
    lt_write(d, 0x11, settle);
    lt_write(d, 0x12, 0x04);
    lt_write(d, 0x13, 0x02);  /* 2 lanes */
    lt_write(d, 0x14, 0x00);
    lt_write(d, 0x15, 0x00);
    lt_write(d, 0x1A, 0x03);
    lt_write(d, 0x1B, 0x03);
    return ESP_OK;
}

static esp_err_t lt8912b_video_timing(void)
{
    i2c_master_dev_handle_t d = s_dev_cec;
    i2c_master_dev_handle_t m = s_dev_main;
    const video_timing_t *t = &s_timing;

    ESP_LOGI(TAG, "Video: %dx%d htotal=%d vtotal=%d pclk=%luKHz",
             t->hact, t->vact, t->htotal, t->vtotal, (unsigned long)t->pclk_khz);

    lt_write(d, 0x18, t->hs  & 0xFF);
    lt_write(d, 0x19, t->vs  & 0xFF);
    lt_write(d, 0x1C, t->hact & 0xFF); lt_write(d, 0x1D, t->hact >> 8);
    lt_write(d, 0x2F, 0x0C);
    lt_write(d, 0x34, t->htotal & 0xFF); lt_write(d, 0x35, t->htotal >> 8);
    lt_write(d, 0x36, t->vtotal & 0xFF); lt_write(d, 0x37, t->vtotal >> 8);
    lt_write(d, 0x38, t->vbp & 0xFF); lt_write(d, 0x39, t->vbp >> 8);
    lt_write(d, 0x3A, t->vfp & 0xFF); lt_write(d, 0x3B, t->vfp >> 8);
    lt_write(d, 0x3C, t->hbp & 0xFF); lt_write(d, 0x3D, t->hbp >> 8);
    lt_write(d, 0x3E, t->hfp & 0xFF); lt_write(d, 0x3F, t->hfp >> 8);
    lt_write(m, 0xAB, 0x00);   /* polarities: both negative for 640x480 */
    lt_write(m, 0xB2, 0x01);   /* HDMI mode */
    return ESP_OK;
}

static esp_err_t lt8912b_avi_infoframe(void)
{
    i2c_master_dev_handle_t a = s_dev_audio;
    uint8_t vic    = s_timing.vic;
    uint8_t aspect = s_timing.aspect_16_9 ? 2 : 1;  /* 2=16:9, 1=4:3 */
    uint8_t pb2    = (uint8_t)((aspect << 4) + 0x08);
    uint8_t pb4    = vic;
    uint8_t sum    = pb2 + pb4;
    uint8_t pb0    = (sum <= 0x5F) ? (0x5F - sum) : (0xFF - sum + 0x60);
    ESP_LOGI(TAG, "AVI: VIC=%d aspect=%s pb0=%02X pb2=%02X pb4=%02X",
             vic, s_timing.aspect_16_9?"16:9":"4:3", pb0, pb2, pb4);
    lt_write(a, 0x3C, 0x41);
    lt_write(a, 0x43, pb0);
    lt_write(a, 0x44, 0x10);
    lt_write(a, 0x45, pb2);
    lt_write(a, 0x46, 0x00);
    lt_write(a, 0x47, pb4);
    return ESP_OK;
}

static esp_err_t lt8912b_dds_config(void)
{
    i2c_master_dev_handle_t d = s_dev_cec;

    /* DDS freq word — from production SamCoupe for 24MHz pixel clock.
     * Formula: pclk_mhz * 0x16C16. For other clocks we use same base
     * table but override the frequency word. */
    /* Use pclk in units of 0.25MHz for better precision:
     * DDS = pclk_quarter * 0x16C16 / 4
     * For 74.25MHz: quarter=297, dds = 297*0x16C16/4 = 0x6B1A63
     * For 24MHz:    quarter=96,  dds = 96 *0x16C16/4 = 0x238E26 */
    uint32_t pclk_mhz = s_timing.pclk_khz / 1000;
    uint32_t pclk_q   = (s_timing.pclk_khz + 124) / 250;  /* round to nearest 0.25MHz */
    uint32_t dds      = (pclk_q * 0x16C16u) / 4;
    ESP_LOGI(TAG, "DDS: pclk=%luMHz word=0x%06lX [0x%02X,0x%02X,0x%02X]",
             (unsigned long)pclk_mhz, (unsigned long)dds,
             (uint8_t)(dds&0xFF),(uint8_t)((dds>>8)&0xFF),(uint8_t)((dds>>16)&0xFF));

    lt_write(d, 0x4E, dds & 0xFF);
    lt_write(d, 0x4F, (dds >>  8) & 0xFF);
    lt_write(d, 0x50, (dds >> 16) & 0xFF);
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
    lt_write(d, 0x51, 0x00);
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
    uint8_t h0=0,h1=0,v0=0,v1=0,c0=0,c1=0,c6=0;
    lt_read(s_dev_main, 0x9C, &h0); lt_read(s_dev_main, 0x9D, &h1);
    lt_read(s_dev_main, 0x9E, &v0); lt_read(s_dev_main, 0x9F, &v1);
    lt_read(s_dev_main, 0xC0, &c0); lt_read(s_dev_main, 0xC1, &c1);
    lt_read(s_dev_main, 0xC6, &c6);
    ESP_LOGI(TAG, "MIPI sync: Hsync=0x%02X%02X Vsync=0x%02X%02X%s",
             h1,h0,v1,v0, (h0||h1||v0||v1)?" — DSI activo":" — SIN señal");
    ESP_LOGI(TAG, "Status: C0=%02X C1=%02X C6=%02X  HPD=%d CLK=%d PLL=%d",
             c0,c1,c6, !!(c1&0x80), !!(c1&0x40), !!(c6&0x80));
}

/* ------------------------------------------------------------------ *
 *  Color bars — RGB888 direct to framebuffer                          *
 * ------------------------------------------------------------------ */
static const uint8_t k_rgb[10][3] = {
    {255,0,0},{255,128,0},{255,255,0},{0,255,0},{0,255,255},
    {0,0,255},{255,0,255},{128,0,255},{255,255,255},{0,0,0},
};

static void fill_color_bars(uint8_t *buf, int w, int h)
{
    const int N = 10;
    int bw = w / N;
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            int b = x / bw; if (b >= N) b = N-1;
            uint8_t *p = buf + (y * w + x) * 3;
            p[0] = k_rgb[b][0];
            p[1] = k_rgb[b][1];
            p[2] = k_rgb[b][2];
        }
    }
}

/* ------------------------------------------------------------------ *
 *  app_main                                                            *
 * ------------------------------------------------------------------ */
void app_main(void)
{
    ESP_LOGI(TAG, "HDMI colorbars — EDID auto-mode");

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase(); nvs_flash_init();
    }

    /* ---- LDO3 → MIPI DPHY ---- */
    {
        esp_ldo_channel_handle_t ldo = NULL;
        esp_ldo_channel_config_t lcfg = {.chan_id=3, .voltage_mv=2500};
        if (esp_ldo_acquire_channel(&lcfg, &ldo) == ESP_OK)
            ESP_LOGI(TAG, "LDO3 @ 2500mV OK");
    }

    /* ---- I2C init ---- */
    {
        i2c_master_bus_config_t i2c_cfg = {
            .i2c_port=I2C_PORT, .sda_io_num=SDA_GPIO, .scl_io_num=SCL_GPIO,
            .clk_source=I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt=7,
            .flags.enable_internal_pullup=true,
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_cfg, &s_bus));

        i2c_device_config_t dev = {.scl_speed_hz=400000};
        dev.device_address = ADDR_MAIN;
        ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus, &dev, &s_dev_main));
        dev.device_address = ADDR_CEC_DSI;
        ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus, &dev, &s_dev_cec));
        dev.device_address = ADDR_AUDIO;
        ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus, &dev, &s_dev_audio));
    }

    /* ---- LT8912B probe ---- */
    {
        uint8_t id_h=0, id_l=0;
        lt_read(s_dev_main, 0x00, &id_h);
        lt_read(s_dev_main, 0x01, &id_l);
        ESP_LOGI(TAG, "LT8912B ID: 0x%02X%02X %s",
                 id_h, id_l, (id_h==0x12&&id_l==0xB2)?"OK":"MISMATCH");
    }

    /* ---- Read EDID and select mode ---- */
    uint8_t edid[128] = {0};
    vTaskDelay(pdMS_TO_TICKS(200));
    if (edid_read(edid)) {
        edid_dump(edid);
        edid_select_mode(edid);
    } else {
        ESP_LOGW(TAG, "No se pudo leer EDID — usando fallback 640x480");
        s_timing = k_fallback;
    }

    /* ---- DSI bus — lane rate from selected timing ---- */
    uint32_t pclk_mhz  = s_timing.pclk_khz / 1000;
    /* lane_mbps = pclk * bpp / lanes, rounded up to nearest 50, min 400 */
    uint32_t lane_mbps  = (pclk_mhz * 24 / 2);
    /* round up to multiple of 50, keep above 400 */
    lane_mbps = ((lane_mbps + 49) / 50) * 50;
    if (lane_mbps < 400) lane_mbps = 400;
    ESP_LOGI(TAG, "DSI: pclk=%luMHz lane=%luMbps", (unsigned long)pclk_mhz, (unsigned long)lane_mbps);

    {
        esp_lcd_dsi_bus_config_t bus_cfg = {
            .bus_id=0, .num_data_lanes=2,
            .phy_clk_src=MIPI_DSI_PHY_CLK_SRC_DEFAULT,
            .lane_bit_rate_mbps=lane_mbps,
        };
        ESP_ERROR_CHECK(esp_lcd_new_dsi_bus(&bus_cfg, &s_dsi_bus));
    }

    /* ---- DPI panel ---- */
    ESP_LOGI(TAG, "DPI: %dx%d @ %.3fMHz", s_timing.hact, s_timing.vact,
             (float)s_timing.pclk_khz / 1000.0f);
    {
        /* dpi_clock_freq_mhz is float — use exact KHz value for precision
         * e.g. 74250KHz = 74.25MHz, avoids 0.3% error that causes flicker */
        float dpi_clk_exact = (float)s_timing.pclk_khz / 1000.0f;
        esp_lcd_dpi_panel_config_t dpi_cfg = {
            .dpi_clk_src        = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
            .dpi_clock_freq_mhz = dpi_clk_exact,
            .pixel_format       = LCD_COLOR_PIXEL_FORMAT_RGB888,
            .num_fbs            = 2,
            .video_timing = {
                .h_size            = s_timing.hact,
                .v_size            = s_timing.vact,
                .hsync_pulse_width = s_timing.hs,
                .hsync_back_porch  = s_timing.hbp,
                .hsync_front_porch = s_timing.hfp,
                .vsync_pulse_width = s_timing.vs,
                .vsync_back_porch  = s_timing.vbp,
                .vsync_front_porch = s_timing.vfp,
            },
            .flags.disable_lp = 1,
        };
        ESP_ERROR_CHECK(esp_lcd_new_panel_dpi(s_dsi_bus, &dpi_cfg, &s_panel));
    }

    /* ---- Framebuffers + color bars ---- */
    size_t fb_size = (size_t)s_timing.hact * s_timing.vact * 3;
    if (esp_lcd_dpi_panel_get_frame_buffer(s_panel, 2, &s_fb[0], &s_fb[1]) != ESP_OK) {
        esp_lcd_dpi_panel_get_frame_buffer(s_panel, 1, &s_fb[0]);
        s_fb[1] = NULL;
    }
    ESP_LOGI(TAG, "FB: fb0=%p fb1=%p (%u KB)", s_fb[0], s_fb[1], (unsigned)(fb_size/1024));

    for (int i = 0; i < 2; i++) {
        if (!s_fb[i]) continue;
        fill_color_bars((uint8_t*)s_fb[i], s_timing.hact, s_timing.vact);
        esp_cache_msync(s_fb[i], fb_size,
            ESP_CACHE_MSYNC_FLAG_DIR_C2M |
            ESP_CACHE_MSYNC_FLAG_TYPE_DATA |
            ESP_CACHE_MSYNC_FLAG_UNALIGNED);
    }
    esp_lcd_panel_init(s_panel);

    /* ---- LT8912B full init (after DPI is running) ---- */
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

    /* ---- Hot-plug monitor ---- */
    bool was_connected = true;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        uint8_t c1 = 0;
        lt_read(s_dev_main, 0xC1, &c1);
        bool connected = !!(c1 & 0x80);
        if (connected && !was_connected) {
            ESP_LOGI(TAG, "HPD: monitor reconectado — reinicializando");
            /* Re-read EDID and reinit with potentially different mode */
            if (edid_read(edid)) {
                edid_dump(edid);
                edid_select_mode(edid);
            }
            lt8912b_init_config();
            lt8912b_mipi_basic();
            lt8912b_video_timing();
            lt8912b_avi_infoframe();
            lt8912b_dds_config();
            lt8912b_rxlogicres();
            lt8912b_lvds_config();
            vTaskDelay(pdMS_TO_TICKS(200));
        } else if (!connected && was_connected) {
            ESP_LOGW(TAG, "HPD: monitor desconectado");
        }
        was_connected = connected;
        if (connected) lt8912b_status();
    }
}
