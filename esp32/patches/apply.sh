#!/usr/bin/env bash
# Apply Arduino_GFX patches for Waveshare ESP32-S3-LCD-1.3 on ESP32 core 4.x.
#
# Usage:
#   ./apply.sh
#
# Environment:
#   ARDUINO_LIBRARIES_DIR   Default: ~/Arduino/libraries

set -euo pipefail
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/apply.py"
