#!/usr/bin/env python3
"""Apply Arduino_GFX fixes for ESP32 Arduino core 4.x (ESP-IDF 6.x) + Waveshare S3 LCD."""

from __future__ import annotations

import os
import sys
from pathlib import Path


def gfx_lib_dir(lib_dir: Path) -> Path:
    for name in ("GFX_Library_for_Arduino", "Arduino_GFX"):
        candidate = lib_dir / name
        if candidate.is_dir():
            return candidate
    raise FileNotFoundError(
        f"Arduino_GFX not found under {lib_dir} "
        "(expected GFX_Library_for_Arduino or Arduino_GFX)"
    )


def patch_spi(path: Path) -> list[str]:
    text = path.read_text()
    notes: list[str] = []

    if '#include "hal/spi_ll.h"' not in text:
        marker = "#if defined(ESP32) && (CONFIG_IDF_TARGET_ESP32"
        idx = text.find(marker)
        if idx < 0:
            raise RuntimeError(f"{path}: ESP32 guard not found")
        end = text.find("\n", idx)
        text = (
            text[:end]
            + "\n\n#if CONFIG_IDF_TARGET_ESP32S3\n#include \"hal/spi_ll.h\"\n#endif"
            + text[end:]
        )
        notes.append("added hal/spi_ll.h include")
    else:
        notes.append("spi_ll include already present")

    old_div = """  if (!_div)
  {
    // Fix for ESP32 Arduino core 3.3.6+ compatibility
    // Ref: https://github.com/espressif/arduino-esp32/pull/12265
    // Changed: spiFrequencyToClockDiv(freq) -> spiFrequencyToClockDiv(spi, freq)
#if defined(ESP_ARDUINO_VERSION) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 3, 6))
    _div = spiFrequencyToClockDiv(&_spi_bus_array[_spi_num], _speed);
#else
    _div = spiFrequencyToClockDiv(_speed);
#endif
  }

  // set pin mode"""

    anchor = """  _spi = &_spi_bus_array[_spi_num];

#if !CONFIG_DISABLE_HAL_LOCKS"""

    relocated = """  _spi = &_spi_bus_array[_spi_num];

  if (!_div)
  {
    // Fix for ESP32 Arduino core 3.3.6+ compatibility
    // Ref: https://github.com/espressif/arduino-esp32/pull/12265
    // Changed: spiFrequencyToClockDiv(freq) -> spiFrequencyToClockDiv(spi, freq)
#if defined(ESP_ARDUINO_VERSION) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 3, 6))
    _div = spiFrequencyToClockDiv(_spi, _speed);
#else
    _div = spiFrequencyToClockDiv(_speed);
#endif
  }

#if !CONFIG_DISABLE_HAL_LOCKS"""

    if "_div = spiFrequencyToClockDiv(_spi, _speed)" in text:
        notes.append("_div relocation already present")
    elif old_div in text and anchor in text:
        text = text.replace(old_div, "  // set pin mode", 1)
        text = text.replace(anchor, relocated, 1)
        notes.append("relocated _div calculation after _spi init")
    else:
        raise RuntimeError(f"{path}: unexpected _div block layout")

    old_s3 = """#elif CONFIG_IDF_TARGET_ESP32S3
  if (_spi_num == FSPI)
  {
    periph_ll_reset(PERIPH_SPI2_MODULE);
    periph_ll_enable_clk_clear_rst(PERIPH_SPI2_MODULE);
  }
  else if (_spi_num == HSPI)
  {
    periph_ll_reset(PERIPH_SPI3_MODULE);
    periph_ll_enable_clk_clear_rst(PERIPH_SPI3_MODULE);
  }"""

    new_s3 = """#elif CONFIG_IDF_TARGET_ESP32S3
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
  if (_spi_num == FSPI)
  {
    PERIPH_RCC_ATOMIC()
    {
      spi_ll_enable_bus_clock(SPI2_HOST, true);
      spi_ll_reset_register(SPI2_HOST);
      spi_ll_enable_clock(SPI2_HOST, true);
    }
  }
  else if (_spi_num == HSPI)
  {
    PERIPH_RCC_ATOMIC()
    {
      spi_ll_enable_bus_clock(SPI3_HOST, true);
      spi_ll_reset_register(SPI3_HOST);
      spi_ll_enable_clock(SPI3_HOST, true);
    }
  }
#else
  if (_spi_num == FSPI)
  {
    periph_ll_reset(PERIPH_SPI2_MODULE);
    periph_ll_enable_clk_clear_rst(PERIPH_SPI2_MODULE);
  }
  else if (_spi_num == HSPI)
  {
    periph_ll_reset(PERIPH_SPI3_MODULE);
    periph_ll_enable_clk_clear_rst(PERIPH_SPI3_MODULE);
  }
#endif"""

    if "spi_ll_enable_bus_clock(SPI2_HOST" in text and "#elif CONFIG_IDF_TARGET_ESP32S3" in text:
        if old_s3 not in text:
            notes.append("IDF6 S3 SPI init already present")
        else:
            text = text.replace(old_s3, new_s3, 1)
            notes.append("added IDF6 S3 SPI bus init")
    elif old_s3 in text:
        text = text.replace(old_s3, new_s3, 1)
        notes.append("added IDF6 S3 SPI bus init")
    else:
        raise RuntimeError(f"{path}: unexpected ESP32S3 SPI init block")

    path.write_text(text)
    return notes


def patch_gpio_num_casts(path: Path) -> list[str]:
    """ESP-IDF 6: gpio_num_t is enum — int8_t pins need explicit casts."""
    import re

    text = path.read_text()
    notes: list[str] = []

    if path.name == "Arduino_ESP32LCD8.cpp":
        if "(gpio_num_t)_dc" in text:
            return ["gpio_num_t casts already present"]
        replacements = [
            (".dc_gpio_num = _dc,", ".dc_gpio_num = (gpio_num_t)_dc,"),
            (".wr_gpio_num = _wr,", ".wr_gpio_num = (gpio_num_t)_wr,"),
            (".cs_gpio_num = _cs,", ".cs_gpio_num = (gpio_num_t)_cs,"),
            (
                ".data_gpio_nums = {\n"
                "          _d0, _d1, _d2, _d3, _d4, _d5, _d6, _d7},",
                ".data_gpio_nums = {\n"
                "          (gpio_num_t)_d0, (gpio_num_t)_d1, (gpio_num_t)_d2, (gpio_num_t)_d3, "
                "(gpio_num_t)_d4, (gpio_num_t)_d5, (gpio_num_t)_d6, (gpio_num_t)_d7},",
            ),
            (
                ".data_gpio_nums = {_d0, _d1, _d2, _d3, _d4, _d5, _d6, _d7},",
                ".data_gpio_nums = {(gpio_num_t)_d0, (gpio_num_t)_d1, (gpio_num_t)_d2, (gpio_num_t)_d3, "
                "(gpio_num_t)_d4, (gpio_num_t)_d5, (gpio_num_t)_d6, (gpio_num_t)_d7},",
            ),
        ]
        for old, new in replacements:
            if old in text:
                text = text.replace(old, new)
                notes.append("gpio struct initializer")
    elif path.name == "Arduino_ESP32LCD16.cpp":
        if "(gpio_num_t)_dc" in text:
            return ["gpio_num_t casts already present"]
        replacements = [
            (".dc_gpio_num = _dc,", ".dc_gpio_num = (gpio_num_t)_dc,"),
            (".wr_gpio_num = _wr,", ".wr_gpio_num = (gpio_num_t)_wr,"),
            (
                ".data_gpio_nums = {\n"
                "          _d0, _d1, _d2, _d3, _d4, _d5, _d6, _d7,\n"
                "          _d8, _d9, _d10, _d11, _d12, _d13, _d14, _d15},",
                ".data_gpio_nums = {\n"
                "          (gpio_num_t)_d0, (gpio_num_t)_d1, (gpio_num_t)_d2, (gpio_num_t)_d3, "
                "(gpio_num_t)_d4, (gpio_num_t)_d5, (gpio_num_t)_d6, (gpio_num_t)_d7,\n"
                "          (gpio_num_t)_d8, (gpio_num_t)_d9, (gpio_num_t)_d10, (gpio_num_t)_d11, "
                "(gpio_num_t)_d12, (gpio_num_t)_d13, (gpio_num_t)_d14, (gpio_num_t)_d15},",
            ),
        ]
        for old, new in replacements:
            if old in text:
                text = text.replace(old, new)
                notes.append("gpio struct initializer")
    elif path.name == "Arduino_ESP32RGBPanel.cpp":
        if "LCD_COLOR_FMT_RGB565" not in text:
            old_rgb_cfg = """      .data_width = 16, // RGB565 in parallel mode, thus 16 bits in width
#if (!defined(ESP_ARDUINO_VERSION_MAJOR)) || (ESP_ARDUINO_VERSION_MAJOR < 3)
#else
      .bits_per_pixel = 16,
      .num_fbs = 1,
      .bounce_buffer_size_px = _bounce_buffer_size_px,
#endif
      .sram_trans_align = 8,
      .psram_trans_align = 64,"""
            new_rgb_cfg = """      .data_width = 16, // RGB565 in parallel mode, thus 16 bits in width
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
      .in_color_format = LCD_COLOR_FMT_RGB565,
      .out_color_format = LCD_COLOR_FMT_RGB565,
      .num_fbs = 1,
      .bounce_buffer_size_px = _bounce_buffer_size_px,
#elif (!defined(ESP_ARDUINO_VERSION_MAJOR)) || (ESP_ARDUINO_VERSION_MAJOR < 3)
#else
      .bits_per_pixel = 16,
      .num_fbs = 1,
      .bounce_buffer_size_px = _bounce_buffer_size_px,
#endif
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(6, 0, 0)
      .sram_trans_align = 8,
      .psram_trans_align = 64,
#endif"""
            if old_rgb_cfg not in text:
                raise RuntimeError(f"{path}: unexpected rgb panel config block")
            text = text.replace(old_rgb_cfg, new_rgb_cfg, 1)
            notes.append("IDF6 rgb panel config (color format fields)")
        else:
            notes.append("IDF6 rgb panel config already present")

        if ".data_gpio_nums = {0}," in text:
            text = text.replace(
                ".data_gpio_nums = {0},",
                ".data_gpio_nums = {(gpio_num_t)0},",
                1,
            )
            notes.append("cast data_gpio_nums placeholder zero")

        if "(gpio_num_t)_hsync" not in text:
            replacements = [
                (".hsync_gpio_num = _hsync,", ".hsync_gpio_num = (gpio_num_t)_hsync,"),
                (".vsync_gpio_num = _vsync,", ".vsync_gpio_num = (gpio_num_t)_vsync,"),
                (".de_gpio_num = _de,", ".de_gpio_num = (gpio_num_t)_de,"),
                (".pclk_gpio_num = _pclk,", ".pclk_gpio_num = (gpio_num_t)_pclk,"),
            ]
            for old, new in replacements:
                if old in text:
                    text = text.replace(old, new)
                    notes.append("rgb sync gpio")
            text, n = re.subn(
                r"(panel_config\.data_gpio_nums\[\d+\] = )(_[a-z0-9]+);",
                r"\1(gpio_num_t)\2;",
                text,
            )
            if n:
                notes.append(f"cast {n} RGB data_gpio_nums assignments")
        else:
            notes.append("gpio_num_t casts already present")
    else:
        return ["skipped (no gpio_num_t rules)"]

    if not notes:
        raise RuntimeError(f"{path}: expected gpio_num_t patches but found no matches")

    path.write_text(text)
    return notes


def patch_spidma(path: Path) -> list[str]:
    text = path.read_text()
    old = """  // Fix for ESP32 Arduino core 3.3.6+ compatibility
  // Ref: https://github.com/espressif/arduino-esp32/pull/12265
  // Note: _div is not used in DMA mode (speed is passed directly to ESP-IDF driver),
  // so we skip the call entirely for 3.3.6+ to avoid the changed function signature.
#if !defined(ESP_ARDUINO_VERSION) || (ESP_ARDUINO_VERSION < ESP_ARDUINO_VERSION_VAL(3, 3, 6))
  if (!_div)
  {
    _div = spiFrequencyToClockDiv(_speed);
  }
#endif

  // set pin mode"""

    if old not in text:
        if "spiFrequencyToClockDiv" not in text:
            return ["legacy _div block already removed"]
        raise RuntimeError(f"{path}: unexpected SPIDMA _div block")

    path.write_text(text.replace(old, "  // set pin mode", 1))
    return ["removed legacy SPIDMA _div block"]


def main() -> int:
    lib_dir = Path(os.environ.get("ARDUINO_LIBRARIES_DIR", Path.home() / "Arduino" / "libraries"))
    gfx = gfx_lib_dir(lib_dir)
    print(f"==> Arduino_GFX ESP32 4.x patches")
    print(f"==> libraries: {lib_dir}")
    print(f"==> gfx lib:   {gfx}")

    spi = gfx / "src/databus/Arduino_ESP32SPI.cpp"
    spidma = gfx / "src/databus/Arduino_ESP32SPIDMA.cpp"

    for note in patch_spi(spi):
        print(f"    Arduino_ESP32SPI.cpp: {note}")
    for note in patch_spidma(spidma):
        print(f"    Arduino_ESP32SPIDMA.cpp: {note}")

    databus = gfx / "src/databus"
    for fname in (
        "Arduino_ESP32LCD8.cpp",
        "Arduino_ESP32LCD16.cpp",
        "Arduino_ESP32RGBPanel.cpp",
    ):
        for note in patch_gpio_num_casts(databus / fname):
            print(f"    {fname}: {note}")

    print("==> done")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
