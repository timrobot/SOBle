#!/usr/bin/env bash
# Stop the detect_atags systemd user service (see install-detect-atags-service.sh).
set -euo pipefail

UNIT_NAME="detect-atags.service"
DISABLE=0

usage() {
  cat <<EOF
Usage: $(basename "$0") [--disable]

Stop $UNIT_NAME.

  --disable   Also disable autostart on login (systemctl --user disable).
EOF
}

log() { printf '%s\n' "$*"; }
warn() { printf 'WARNING: %s\n' "$*" >&2; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --disable)
      DISABLE=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

if ! systemctl --user --quiet is-active "$UNIT_NAME"; then
  log "$UNIT_NAME is not running."
else
  systemctl --user stop "$UNIT_NAME"
  log "Stopped $UNIT_NAME"
fi

if [[ $DISABLE -eq 1 ]]; then
  if systemctl --user is-enabled "$UNIT_NAME" >/dev/null 2>&1; then
    systemctl --user disable "$UNIT_NAME"
    log "Disabled $UNIT_NAME"
  else
    log "$UNIT_NAME was not enabled."
  fi
fi

log
log "Status:  systemctl --user status $UNIT_NAME"
log "Start:   systemctl --user start $UNIT_NAME"
log "Logs:    journalctl --user -u $UNIT_NAME -f"
