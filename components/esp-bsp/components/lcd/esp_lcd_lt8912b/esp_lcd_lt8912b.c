/*
 * LT8912B MIPI-DSI to HDMI bridge driver
 *
 * Init sequence ported from Linux kernel driver:
 *   drivers/gpu/drm/bridge/lontium-lt8912b.c
 *
 * Key differences vs. original Espressif driver:
 *  - 0x31/0x32 = 0xb1  (Espressif had 0xe1 — bug fixed in kernel commit 051ad27)
 *  - 0x33 = 0x0e from the start (HDMI TX enabled immediately, not deferred)
 *  - 0xb2 = 0x00 in init, set to 0x01 (HDMI mode) during video_setup
 *  - term_en (0x10) and settle (0x11) written in video_setup alongside timings
 *  - mipi_basic_config writes only {0x12,0x14,0x15,0x1a,0x1b} — no settle here
 *  - lane count (0x13) written directly, not inside mipi_basic_config
 *  - DDS: 0x4e=0xff, 0x4f=0x56, 0x50=0x69 (Linux reference values)
 *  - rxlogicres reset uses usleep 10-20ms (not vTaskDelay 10ms each)
 *  - EDID read via DDC over I2C (AVI I2C address 0x50 = DDC)
 */

#include <string.h>
#include <stdlib.h>
#include <sys/cdefs.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_commands.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_lcd_lt8912b.h"

static const char *TAG = "lt8912b";

/* ------------------------------------------------------------------ *
 *  Internal types                                                      *
 * ------------------------------------------------------------------ */
static esp_err_t panel_lt8912b_del(esp_lcd_panel_t *panel);
static esp_err_t panel_lt8912b_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_lt8912b_init(esp_lcd_panel_t *panel);
static esp_err_t panel_lt8912b_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t panel_lt8912b_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_lt8912b_disp_on_off(esp_lcd_panel_t *panel, bool off);
static esp_err_t panel_lt8912b_sleep(esp_lcd_panel_t *panel, bool sleep);

typedef struct {
    esp_lcd_panel_lt8912b_io_t io;
    esp_lcd_panel_lt8912b_video_timing_t video_timing;
    int reset_gpio_num;
    bool reset_level;
    uint8_t lane_num;
    esp_err_t (*del)(esp_lcd_panel_t *panel);
    esp_err_t (*init)(esp_lcd_panel_t *panel);
} lt8912b_panel_t;

typedef struct {
    uint8_t cmd;
    uint8_t data;
} lt8912b_reg_t;

/* ------------------------------------------------------------------ *
 *  Low-level I2C helpers                                               *
 * ------------------------------------------------------------------ */
static esp_err_t _reg_write(esp_lcd_panel_io_handle_t io, uint8_t reg, uint8_t val)
{
    return esp_lcd_panel_io_tx_param(io, reg, &val, 1);
}

static esp_err_t _reg_read(esp_lcd_panel_io_handle_t io, uint8_t reg, uint8_t *val)
{
    return esp_lcd_panel_io_rx_param(io, reg, val, 1);
}

static esp_err_t _reg_write_array(esp_lcd_panel_io_handle_t io,
                                  const lt8912b_reg_t *cmds, uint16_t n)
{
    for (uint16_t i = 0; i < n; i++) {
        ESP_RETURN_ON_ERROR(_reg_write(io, cmds[i].cmd, cmds[i].data),
                            TAG, "write reg 0x%02x failed", cmds[i].cmd);
    }
    return ESP_OK;
}

/* ------------------------------------------------------------------ *
 *  Init sequence tables — from Linux kernel lontium-lt8912b.c         *
 * ------------------------------------------------------------------ */

/* lt8912_write_init_config — I2C_MAIN (0x48) */
static const lt8912b_reg_t cmd_init_config[] = {
    /* Digital clock en */
    {0x08, 0xff},
    {0x09, 0xff},
    {0x0a, 0xff},
    {0x0b, 0x7c},
    {0x0c, 0xff},
    {0x42, 0x04},
    /* Tx Analog — 0xb1 confirmed correct by Linux commit 051ad27 */
    {0x31, 0xb1},
    {0x32, 0xb1},
    {0x33, 0x0e},   /* HDMI TX enable from the start */
    {0x37, 0x00},
    {0x38, 0x22},
    {0x60, 0x82},
    /* Cbus Analog */
    {0x39, 0x45},
    {0x3a, 0x00},
    {0x3b, 0x00},
    /* HDMI PLL Analog */
    {0x44, 0x31},
    {0x55, 0x44},
    {0x57, 0x01},
    {0x5a, 0x02},
    /* MIPI Analog */
    {0x3e, 0xd6},
    {0x3f, 0xd4},
    {0x41, 0x3c},
    {0xb2, 0x00},   /* DVI mode initially; set to 0x01 (HDMI) in video_setup */
};

/* lt8912_write_mipi_basic_config — I2C_CEC_DSI (0x49) */
/* NOTE: Linux does NOT write 0x10 (term_en) or 0x11 (settle) here */
static const lt8912b_reg_t cmd_mipi_basic[] = {
    {0x12, 0x04},
    {0x14, 0x00},
    {0x15, 0x00},
    {0x1a, 0x03},
    {0x1b, 0x03},
};

/* lt8912_write_dds_config — I2C_CEC_DSI (0x49) */
static const lt8912b_reg_t cmd_dds_config[] = {
    {0x4e, 0xff},
    {0x4f, 0x56},
    {0x50, 0x69},
    {0x51, 0x80},
    {0x1f, 0x5e},
    {0x20, 0x01},
    {0x21, 0x2c},
    {0x22, 0x01},
    {0x23, 0xfa},
    {0x24, 0x00},
    {0x25, 0xc8},
    {0x26, 0x00},
    {0x27, 0x5e},
    {0x28, 0x01},
    {0x29, 0x2c},
    {0x2a, 0x01},
    {0x2b, 0xfa},
    {0x2c, 0x00},
    {0x2d, 0xc8},
    {0x2e, 0x00},
    {0x42, 0x64},
    {0x43, 0x00},
    {0x44, 0x04},
    {0x45, 0x00},
    {0x46, 0x59},
    {0x47, 0x00},
    {0x48, 0xf2},
    {0x49, 0x06},
    {0x4a, 0x00},
    {0x4b, 0x72},
    {0x4c, 0x45},
    {0x4d, 0x00},
    {0x52, 0x08},
    {0x53, 0x00},
    {0x54, 0xb2},
    {0x55, 0x00},
    {0x56, 0xe4},
    {0x57, 0x0d},
    {0x58, 0x00},
    {0x59, 0xe4},
    {0x5a, 0x8a},
    {0x5b, 0x00},
    {0x5c, 0x34},
    {0x1e, 0x4f},
    {0x51, 0x00},
};

/* lt8912_write_lvds_config — I2C_MAIN (0x48) */
static const lt8912b_reg_t cmd_lvds_config[] = {
    {0x44, 0x30},
    {0x51, 0x05},
    {0x50, 0x24},
    {0x51, 0x2d},
    {0x52, 0x04},
    {0x69, 0x0e},
    {0x69, 0x8e},
    {0x6a, 0x00},
    {0x6c, 0xb8},
    {0x6b, 0x51},
    {0x04, 0xfb},
    {0x04, 0xff},
    {0x7f, 0x00},
    {0xa8, 0x13},
    {0x02, 0xf7},
    {0x02, 0xff},
    {0x03, 0xcf},
    {0x03, 0xff},
};

/* ------------------------------------------------------------------ *
 *  lt8912_video_setup — port of Linux lt8912_video_setup()            *
 *  Writes timing registers AND settle/term_en together                *
 * ------------------------------------------------------------------ */
static esp_err_t _lt8912b_video_setup(lt8912b_panel_t *lt)
{
    esp_lcd_panel_io_handle_t io_main    = lt->io.main;
    esp_lcd_panel_io_handle_t io_cec_dsi = lt->io.cec_dsi;
    const esp_lcd_panel_lt8912b_video_timing_t *t = &lt->video_timing;

    uint16_t hactive = t->hact;
    uint16_t htotal  = t->htotal;
    uint16_t hpw     = t->hs;
    uint16_t hfp     = t->hfp;
    uint16_t hbp     = t->hbp;

    uint16_t vactive = t->vact;
    uint16_t vtotal  = t->vtotal;
    uint16_t vpw     = t->vs;
    uint16_t vfp     = t->vfp;
    uint16_t vbp     = t->vbp;

    /* Settle time — from Linux kernel, keyed by vertical resolution */
    uint8_t settle = 0x08;
    if (vactive <= 600)       settle = 0x04;
    else if (vactive == 1080) settle = 0x0a;

    ESP_LOGI(TAG, "video_setup: %dx%d htotal=%d vtotal=%d settle=0x%02x",
             hactive, vactive, htotal, vtotal, settle);

    /* settle depends on lane bit rate (T_HS-SETTLE in D-PHY spec).
     * Formula approx: settle = ceil(155ns / T_bit) where T_bit = 1/lane_mbps
     * At 300 Mbps: T_bit=3.33ns, 155ns/3.33ns ~ 47 -> but LT8912B uses
     * internal units. Linux values (keyed by vactive) work as proxy:
     * vactive<=600 (low res, low rate) -> 0x04 */
    ESP_LOGI(TAG, "settle=0x%02x (vactive=%d)", settle, vactive);
    ESP_RETURN_ON_ERROR(_reg_write(io_cec_dsi, 0x10, 0x01), TAG, "term_en");
    ESP_RETURN_ON_ERROR(_reg_write(io_cec_dsi, 0x11, settle), TAG, "settle");

    ESP_RETURN_ON_ERROR(_reg_write(io_cec_dsi, 0x18, hpw & 0xff), TAG, "hpw");
    ESP_RETURN_ON_ERROR(_reg_write(io_cec_dsi, 0x19, vpw & 0xff), TAG, "vpw");
    ESP_RETURN_ON_ERROR(_reg_write(io_cec_dsi, 0x1c, hactive & 0xff), TAG, "hact_lo");
    ESP_RETURN_ON_ERROR(_reg_write(io_cec_dsi, 0x1d, hactive >> 8),   TAG, "hact_hi");
    ESP_RETURN_ON_ERROR(_reg_write(io_cec_dsi, 0x2f, 0x0c), TAG, "fifo");
    ESP_RETURN_ON_ERROR(_reg_write(io_cec_dsi, 0x34, htotal & 0xff), TAG, "htotal_lo");
    ESP_RETURN_ON_ERROR(_reg_write(io_cec_dsi, 0x35, htotal >> 8),   TAG, "htotal_hi");
    ESP_RETURN_ON_ERROR(_reg_write(io_cec_dsi, 0x36, vtotal & 0xff), TAG, "vtotal_lo");
    ESP_RETURN_ON_ERROR(_reg_write(io_cec_dsi, 0x37, vtotal >> 8),   TAG, "vtotal_hi");
    ESP_RETURN_ON_ERROR(_reg_write(io_cec_dsi, 0x38, vbp & 0xff), TAG, "vbp_lo");
    ESP_RETURN_ON_ERROR(_reg_write(io_cec_dsi, 0x39, vbp >> 8),   TAG, "vbp_hi");
    ESP_RETURN_ON_ERROR(_reg_write(io_cec_dsi, 0x3a, vfp & 0xff), TAG, "vfp_lo");
    ESP_RETURN_ON_ERROR(_reg_write(io_cec_dsi, 0x3b, vfp >> 8),   TAG, "vfp_hi");
    ESP_RETURN_ON_ERROR(_reg_write(io_cec_dsi, 0x3c, hbp & 0xff), TAG, "hbp_lo");
    ESP_RETURN_ON_ERROR(_reg_write(io_cec_dsi, 0x3d, hbp >> 8),   TAG, "hbp_hi");
    ESP_RETURN_ON_ERROR(_reg_write(io_cec_dsi, 0x3e, hfp & 0xff), TAG, "hfp_lo");
    ESP_RETURN_ON_ERROR(_reg_write(io_cec_dsi, 0x3f, hfp >> 8),   TAG, "hfp_hi");

    /* Sync polarity — reg 0xab bits 0 (vsync) and 1 (hsync)
     * I2C rx_param is unreliable with disable_control_phase — write directly */
    uint8_t ab = 0;
    if (t->v_polarity) ab |= (1 << 0);
    if (t->h_polarity) ab |= (1 << 1);
    ESP_RETURN_ON_ERROR(_reg_write(io_main, 0xab, ab), TAG, "0xab polarity");
    ESP_LOGI(TAG, "0xab polarity: vsync=%d hsync=%d -> 0x%02x",
             t->v_polarity, t->h_polarity, ab);

    /* HDMI mode — reg 0xb2 bit0: 1=HDMI, 0=DVI */
    ESP_RETURN_ON_ERROR(_reg_write(io_main, 0xb2, 0x01), TAG, "0xb2 hdmi");

    return ESP_OK;
}

/* ------------------------------------------------------------------ *
 *  lt8912_write_rxlogicres_config                                      *
 * ------------------------------------------------------------------ */
static esp_err_t _lt8912b_rxlogicres(esp_lcd_panel_io_handle_t io_main)
{
    ESP_RETURN_ON_ERROR(_reg_write(io_main, 0x03, 0x7f), TAG, "rxlogicres assert");
    vTaskDelay(pdMS_TO_TICKS(15));
    ESP_RETURN_ON_ERROR(_reg_write(io_main, 0x03, 0xff), TAG, "rxlogicres deassert");
    return ESP_OK;
}

/* ------------------------------------------------------------------ *
 *  EDID read via DDC (I2C address 0x50 on the AVI bus)                *
 *  Returns the raw 128-byte EDID block in buf.                        *
 * ------------------------------------------------------------------ */
esp_err_t esp_lcd_panel_lt8912b_read_edid(esp_lcd_panel_t *panel, uint8_t *buf, size_t buf_len)
{
    if (!panel || !buf || buf_len < 128) {
        return ESP_ERR_INVALID_ARG;
    }

    lt8912b_panel_t *lt = (lt8912b_panel_t *)panel->user_data;
    esp_lcd_panel_io_handle_t io_avi = lt->io.avi;

    /* The EDID DDC address is 0x50 — we re-use the AVI I2C handle
     * which is on the same bus. However the Espressif I2C panel IO
     * is locked to its configured address. We read EDID by reading
     * 128 bytes starting at register 0x00 from the AVI IO handle.
     * Note: LT8912B routes DDC to its AVI I2C address (0x4A) internally
     * for EDID passthrough. Some boards expose DDC directly.
     *
     * We read 128 bytes: registers 0x00..0x7f via rx_param bursts. */
    ESP_LOGI(TAG, "Reading EDID...");
    for (int i = 0; i < 128 && i < (int)buf_len; i++) {
        esp_err_t err = esp_lcd_panel_io_rx_param(io_avi, i, &buf[i], 1);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "EDID read error at byte %d: %s", i, esp_err_to_name(err));
            memset(buf + i, 0xff, buf_len - i);
            return err;
        }
    }
    return ESP_OK;
}

/* ------------------------------------------------------------------ *
 *  HPD check                                                           *
 * ------------------------------------------------------------------ */
static bool _lt8912b_get_hpd(lt8912b_panel_t *lt)
{
    uint8_t val = 0;
    esp_lcd_panel_io_rx_param(lt->io.main, 0xc1, &val, 1);
    return (val & 0x80) != 0;
}

bool esp_lcd_panel_lt8912b_is_ready(esp_lcd_panel_t *panel)
{
    lt8912b_panel_t *lt = (lt8912b_panel_t *)panel->user_data;
    return _lt8912b_get_hpd(lt);
}

/* ------------------------------------------------------------------ *
 *  MIPI input detection (diagnostic only)                              *
 * ------------------------------------------------------------------ */
static void _lt8912b_detect_mipi(lt8912b_panel_t *lt)
{
    uint8_t h0, h1, v0, v1;
    esp_lcd_panel_io_rx_param(lt->io.cec_dsi, 0x9c, &h0, 1);
    esp_lcd_panel_io_rx_param(lt->io.cec_dsi, 0x9d, &h1, 1);
    esp_lcd_panel_io_rx_param(lt->io.cec_dsi, 0x9e, &v0, 1);
    esp_lcd_panel_io_rx_param(lt->io.cec_dsi, 0x9f, &v1, 1);
    ESP_LOGI(TAG, "Detected MIPI input. H sync: 0x%02x 0x%02x, V sync: 0x%02x 0x%02x",
             h0, h1, v0, v1);
}

/* ------------------------------------------------------------------ *
 *  panel_lt8912b_init — full Linux init sequence                       *
 * ------------------------------------------------------------------ */
static esp_err_t panel_lt8912b_init(esp_lcd_panel_t *panel)
{
    lt8912b_panel_t *lt = (lt8912b_panel_t *)panel->user_data;
    esp_lcd_panel_io_handle_t io_main    = lt->io.main;
    esp_lcd_panel_io_handle_t io_cec_dsi = lt->io.cec_dsi;

    /* Wait for LT8912B to be ready after power-on / reset */
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Probe: try to read a register before writing to confirm I2C works */
    {
        uint8_t probe = 0;
        esp_err_t probe_err = _reg_read(io_main, 0x00, &probe);
        ESP_LOGI(TAG, "I2C probe reg[0x00] = 0x%02x  err=%s", probe, esp_err_to_name(probe_err));
    }

    /* Step 1: lt8912_write_init_config (MAIN) */
    ESP_RETURN_ON_ERROR(
        _reg_write_array(io_main, cmd_init_config, sizeof(cmd_init_config) / sizeof(cmd_init_config[0])),
        TAG, "init_config failed");

    /* Step 2: lane count (MAIN 0x13 via CEC — actually CEC reg 0x13) */
    /* NOTE: In Linux this is: regmap_write(CEC_DSI, 0x13, lanes & 3) */
    ESP_RETURN_ON_ERROR(_reg_write(io_cec_dsi, 0x13, lt->lane_num & 3),
                        TAG, "lane count failed");

    /* Step 3: lt8912_write_mipi_basic_config (CEC_DSI) */
    /* No term_en/settle here — Linux writes those in video_setup */
    ESP_RETURN_ON_ERROR(
        _reg_write_array(io_cec_dsi, cmd_mipi_basic, sizeof(cmd_mipi_basic) / sizeof(cmd_mipi_basic[0])),
        TAG, "mipi_basic failed");

    /* Step 4: lt8912_video_setup (timing + settle + polarity + HDMI mode) */
    ESP_RETURN_ON_ERROR(_lt8912b_video_setup(lt), TAG, "video_setup failed");

    /* Step 5: lt8912_write_dds_config (CEC_DSI) */
    ESP_RETURN_ON_ERROR(
        _reg_write_array(io_cec_dsi, cmd_dds_config, sizeof(cmd_dds_config) / sizeof(cmd_dds_config[0])),
        TAG, "dds_config failed");

    /* Step 6: lt8912_write_rxlogicres_config (MAIN) */
    ESP_RETURN_ON_ERROR(_lt8912b_rxlogicres(io_main), TAG, "rxlogicres failed");

    /* Step 7: lt8912_write_lvds_config (MAIN) */
    ESP_RETURN_ON_ERROR(
        _reg_write_array(io_main, cmd_lvds_config, sizeof(cmd_lvds_config) / sizeof(cmd_lvds_config[0])),
        TAG, "lvds_config failed");

    /* Diagnostic: read MIPI sync counters after full init */
    vTaskDelay(pdMS_TO_TICKS(50));
    _lt8912b_detect_mipi(lt);

    /* Log HPD status */
    bool hpd = _lt8912b_get_hpd(lt);
    ESP_LOGI(TAG, "HPD: %s", hpd ? "HIGH (monitor connected)" : "LOW (no monitor or HPD pin NC)");

    /* Step 8: init the underlying MIPI DPI panel */
    ESP_RETURN_ON_ERROR(lt->init(panel), TAG, "init MIPI DPI panel failed");

    return ESP_OK;
}

/* ------------------------------------------------------------------ *
 *  Panel create / delete / reset                                       *
 * ------------------------------------------------------------------ */
esp_err_t esp_lcd_new_panel_lt8912b(const esp_lcd_panel_lt8912b_io_t *io,
                                     const esp_lcd_panel_dev_config_t *panel_dev_config,
                                     esp_lcd_panel_handle_t *ret_panel)
{
    ESP_RETURN_ON_FALSE(io && io->main && io->cec_dsi && io->avi &&
                        panel_dev_config && ret_panel,
                        ESP_ERR_INVALID_ARG, TAG, "invalid argument");

    lt8912b_vendor_config_t *vendor_config =
        (lt8912b_vendor_config_t *)panel_dev_config->vendor_config;
    ESP_RETURN_ON_FALSE(vendor_config && vendor_config->mipi_config.dpi_config &&
                        vendor_config->mipi_config.dsi_bus,
                        ESP_ERR_INVALID_ARG, TAG, "invalid vendor config");

    esp_err_t ret = ESP_OK;
    lt8912b_panel_t *lt = (lt8912b_panel_t *)calloc(1, sizeof(lt8912b_panel_t));
    ESP_RETURN_ON_FALSE(lt, ESP_ERR_NO_MEM, TAG, "no mem for lt8912b panel");

    /* GPIO reset */
    if (panel_dev_config->reset_gpio_num >= 0) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "reset gpio config failed");
    }

    memcpy(&lt->io, io, sizeof(esp_lcd_panel_lt8912b_io_t));
    memcpy(&lt->video_timing, &vendor_config->video_timing,
           sizeof(esp_lcd_panel_lt8912b_video_timing_t));
    lt->lane_num        = vendor_config->mipi_config.lane_num;
    lt->reset_gpio_num  = panel_dev_config->reset_gpio_num;
    lt->reset_level     = panel_dev_config->flags.reset_active_high;

    /* Create the underlying MIPI DPI panel */
    ESP_GOTO_ON_ERROR(
        esp_lcd_new_panel_dpi(vendor_config->mipi_config.dsi_bus,
                              vendor_config->mipi_config.dpi_config,
                              ret_panel),
        err, TAG, "create MIPI DPI panel failed");

    /* Wrap the DPI panel with our init */
    lt->del  = (*ret_panel)->del;
    lt->init = (*ret_panel)->init;

    (*ret_panel)->user_data    = lt;
    (*ret_panel)->del          = panel_lt8912b_del;
    (*ret_panel)->reset        = panel_lt8912b_reset;
    (*ret_panel)->init         = panel_lt8912b_init;
    (*ret_panel)->invert_color = panel_lt8912b_invert_color;
    (*ret_panel)->mirror       = panel_lt8912b_mirror;
    (*ret_panel)->disp_on_off  = panel_lt8912b_disp_on_off;

    return ESP_OK;

err:
    if (lt) {
        if (panel_dev_config->reset_gpio_num >= 0) {
            gpio_reset_pin(panel_dev_config->reset_gpio_num);
        }
        free(lt);
    }
    return ret;
}

static esp_err_t panel_lt8912b_del(esp_lcd_panel_t *panel)
{
    lt8912b_panel_t *lt = (lt8912b_panel_t *)panel->user_data;
    if (lt->reset_gpio_num >= 0) {
        gpio_reset_pin(lt->reset_gpio_num);
    }
    lt->del(panel);
    free(lt);
    return ESP_OK;
}

static esp_err_t panel_lt8912b_reset(esp_lcd_panel_t *panel)
{
    lt8912b_panel_t *lt = (lt8912b_panel_t *)panel->user_data;
    if (lt->reset_gpio_num >= 0) {
        gpio_set_level(lt->reset_gpio_num, lt->reset_level);
        vTaskDelay(pdMS_TO_TICKS(20));
        gpio_set_level(lt->reset_gpio_num, !lt->reset_level);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    return ESP_OK;
}

static esp_err_t panel_lt8912b_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t panel_lt8912b_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    ESP_LOGW(TAG, "Mirror is not supported in LT8912B driver. Please use SW rotation.");
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t panel_lt8912b_disp_on_off(esp_lcd_panel_t *panel, bool on_off)
{
    return ESP_OK;
}

static esp_err_t panel_lt8912b_sleep(esp_lcd_panel_t *panel, bool sleep)
{
    return ESP_OK;
}
