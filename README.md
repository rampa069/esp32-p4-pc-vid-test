# esp32-p4-pc-vid-test

HDMI video test for the **Olimex ESP32-P4-PC** board.

Displays color bars at 640×480@60Hz via the LT8912B MIPI-DSI→HDMI bridge
and dumps the monitor's EDID to the serial log at boot.

Tested on 4 different monitors. Hot-plug detection: if you reconnect the
HDMI cable the LT8912B reinitializes automatically and the new monitor's
EDID is printed.

---

## Hardware

| Item | Value |
|------|-------|
| Board | Olimex ESP32-P4-PC |
| SoC | ESP32-P4 rev v1.0 |
| Bridge | Lontium LT8912B (MIPI-DSI → HDMI) |
| Resolution | 640×480 @ 60 Hz |
| Pixel clock | 24 MHz |
| DSI lane rate | 400 Mbps × 2 lanes |
| Color format | RGB888 (24-bit) |
| I2C | Port 1, SDA=GPIO7, SCL=GPIO8 |
| HPD | GPIO15 |

---

## What it does

1. Powers the MIPI D-PHY via internal LDO3 (2500 mV).
2. Initialises the MIPI-DSI bus and the DPI panel (frame buffers in PSRAM).
3. Reads the monitor's **EDID** via DDC (I2C 0x50) and prints:
   - Manufacturer, model name, serial number
   - Supported established timings (640×480, 800×600, 1024×768…)
   - All Detailed Timing Descriptors (DTD) with full parameters
   - Raw 128-byte hex dump
4. Initialises the LT8912B with the Linux-kernel register sequence.
5. Writes **10 colour bars** (RGB888) directly to both DPI frame buffers
   using `esp_cache_msync` for correct PSRAM flush.
6. Monitors HPD every 2 s — on cable reconnect, reinitialises LT8912B
   and re-reads the new monitor's EDID.

The EDID dump is the primary diagnostic tool: if the image does not
appear on a user's monitor, ask them to paste the serial log so you can
inspect the EDID.

---

## Key parameters (main.c)

```c
#define PCLK_MHZ   24      // pixel clock
#define LANE_MBPS  400     // DSI lane bit rate
#define VFP        10      // vertical front porch (CEA-861 standard)
#define VBP        33      // vertical back porch  (CEA-861 standard)
// DDS frequency word (LT8912B reg 0x4E/4F/50)
#define DDS_B0  0x10
#define DDS_B1  0x22
#define DDS_B2  0x22
```

These values follow the CEA-861 VIC1 standard timing for 640×480@60Hz
and have been verified working on multiple monitors.

---

## Build & flash

### Requirements

- ESP-IDF **v5.5.x** (tested with v5.5.3)
- Olimex ESP32-P4-PC or compatible board with LT8912B

### Setup ESP-IDF (first time only)

```bash
git clone https://github.com/espressif/esp-idf -b "release/v5.5"
cd esp-idf
./install.sh
```

### Build

```bash
# Activate IDF environment (every new terminal)
. ~/esp/esp-idf/export.sh

cd esp32-p4-pc-vid-test
idf.py build
```

### Flash & monitor

```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

---

## Serial output (example)

```
I (1533) colorbars: LT8912B ID: 0x12B2 OK
I (1745) colorbars: === EDID === HWP v1.3  51cmx29cm
I (1745) colorbars:   640x480@60 soportado
I (1754) colorbars: DTD[0]: 1920x1080@60Hz pclk=148500KHz ...
I (1768) colorbars: Nombre: HP 23xi
I (1772) colorbars: Serie:  3CM4240CTN
I (1779) colorbars: [00] 00 ff ff ff ff ff ff 00 22 f0 32 30 ...
...
I (2082) colorbars: MIPI sync: Hsync=0x6831 Vsync=0x01F4 — DSI activo
I (2082) colorbars: Status: C0=20 C1=FC C6=01 C9=00  HPD=1 CLK=1 PLL=0
I (2083) colorbars: Init completo — barras en pantalla
```

`HPD=1 CLK=1` confirms the monitor is connected and the DSI clock is
running. `PLL=0` (LT8912B HDMI PLL lock bit) is normal for this
configuration — the image is displayed correctly regardless.

---

## LT8912B register sequence

Ported from the Linux kernel driver:
`drivers/gpu/drm/bridge/lontium-lt8912b.c`

Key differences from the original Espressif BSP driver:
- `0x31/0x32 = 0xB1` (kernel), not `0xE1`
- `0x33 = 0x0E` in init (HDMI TX output enabled from the start)
- `0xB2 = 0x00` in init, set to `0x01` (HDMI mode) in video_setup
- DDS `0x4E/4F/50` values from production sdkconfig, not calculated
- `rxlogicres` + `lvds_config` resets at end (kernel sequence)
- `esp_cache_msync` for correct PSRAM→DPI frame buffer flush

---

## License

The project code in `main/` is provided as-is for reference.
The `components/esp-bsp/` directory contains Espressif components
under their respective licences (Apache 2.0).
