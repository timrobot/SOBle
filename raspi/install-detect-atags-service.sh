#!/usr/bin/env bash
# Install detect_atags.py as a systemd *user* service (starts at graphical login).
set -euo pipefail

RASPI_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNIT_NAME="detect-atags.service"
USER_UNIT_DIR="${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user"
TEMPLATE="$RASPI_DIR/detect-atags.service.in"
DEST="$USER_UNIT_DIR/$UNIT_NAME"

if [[ ! -f "$TEMPLATE" ]]; then
  echo "Missing template: $TEMPLATE" >&2
  exit 1
fi

chmod +x "$RASPI_DIR/run_detect_atags.sh"

mkdir -p "$USER_UNIT_DIR"
sed "s|@REPO_DIR@|${RASPI_DIR}|g" "$TEMPLATE" >"$DEST"

systemctl --user daemon-reload
systemctl --user enable "$UNIT_NAME"

if systemctl --user start "$UNIT_NAME"; then
  echo "Installed and started $UNIT_NAME"
else
  echo "Installed $UNIT_NAME but start failed — check logs below" >&2
fi

echo
echo "Status:  systemctl --user status $UNIT_NAME"
echo "Logs:    journalctl --user -u $UNIT_NAME -f"
echo "Stop:    systemctl --user stop $UNIT_NAME"
echo "Disable: systemctl --user disable $UNIT_NAME"
