# SO101 ESP32 firmware

Firmware for the Waveshare ESP32-S3-LCD-1.3 platform (`So101-Platform/`).

## Arduino_GFX display patch (ESP32 core 4.x)

The ST7789 display uses **Arduino_GFX** (`GFX_Library_for_Arduino`). ESP32 Arduino core **4.x** (ESP-IDF 6.x) breaks the stock databus drivers — SPI clock setup and bus init must be patched.

After installing or updating the library, run:

```bash
./patches/apply.sh
```

This patches:

- `Arduino_ESP32SPI.cpp` — `spiFrequencyToClockDiv(spi, …)` after `_spi` init; IDF 6 `spi_ll_*` bus clock on ESP32-S3
- `Arduino_ESP32SPIDMA.cpp` — remove obsolete `spiFrequencyToClockDiv` call
- `Arduino_ESP32LCD8.cpp`, `Arduino_ESP32LCD16.cpp`, `Arduino_ESP32RGBPanel.cpp` — `(gpio_num_t)` casts for IDF 6 (compiled via umbrella header even for SPI displays)
- `Arduino_ESP32RGBPanel.cpp` — `LCD_COLOR_FMT_RGB565` in/out color format (replaces removed `bits_per_pixel` field)

Requires **GFX_Library_for_Arduino** (or a symlinked `Arduino_GFX`) under `~/Arduino/libraries`.

## Build

Open `So101-Platform/So101-Platform.ino` in Arduino IDE:

- Board: **ESP32S3 Dev Module** (or Waveshare ESP32-S3-LCD-1.3)
- ESP32 core: **4.x**
- Libraries: Arduino_GFX, NimBLE / ESP32 BLE (as used by the sketch)
