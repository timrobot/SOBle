#!/usr/bin/env bash
# Wrapper for detect_atags.py (systemd user service / manual runs).
set -euo pipefail

RASPI_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$RASPI_DIR"

export SERIAL_PORT="${SERIAL_PORT:-/dev/ttyUSB0}"
export PYTHONUNBUFFERED="${PYTHONUNBUFFERED:-1}"

if [[ -x "$RASPI_DIR/.venv/bin/python3" ]]; then
  PYTHON="$RASPI_DIR/.venv/bin/python3"
elif command -v python3 >/dev/null 2>&1; then
  PYTHON="python3"
else
  echo "run_detect_atags.sh: python3 not found" >&2
  exit 1
fi

exec "$PYTHON" "$RASPI_DIR/detect_atags.py"
