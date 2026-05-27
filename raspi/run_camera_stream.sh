#!/usr/bin/env bash
# Stream Pi camera over RTP/UDP (see camera_stream.py).
#
# Usage:
#   RTP_HOST=192.168.1.100 RTP_PORT=5000 ./run_camera_stream.sh

set -euo pipefail
[ -n "${BASH_VERSION:-}" ] || exec bash "$0" "$@"

RASPI_DIR=$(cd "$(dirname "$0")" && pwd)
PYTHON=${PYTHON:-python3}

exec "$PYTHON" "$RASPI_DIR/camera_stream.py"
