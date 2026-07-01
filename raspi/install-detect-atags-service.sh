#!/usr/bin/env bash
# Full Pi setup for detect_atags.py (see assembly guide) + systemd user autostart.
set -euo pipefail

RASPI_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNIT_NAME="detect-atags.service"
USER_UNIT_DIR="${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user"
TEMPLATE="$RASPI_DIR/detect-atags.service.in"
DEST="$USER_UNIT_DIR/$UNIT_NAME"

NEEDS_REBOOT=0
NEEDS_RELOGIN=0

APT_PACKAGES=(
  python3-picamera2
  python3-libcamera
  python3-opencv
  python3-apriltag
  python3-serial
  python3-numpy
  python3-gi
  gstreamer1.0-tools
  gstreamer1.0-plugins-base
  gstreamer1.0-plugins-good
  gstreamer1.0-plugins-bad
  gstreamer1.0-libcamera
)

log() { printf '%s\n' "$*"; }
warn() { printf 'WARNING: %s\n' "$*" >&2; }

require_pi() {
  if [[ -f /proc/device-tree/model ]]; then
    if grep -aq Raspberry /proc/device-tree/model 2>/dev/null; then
      return 0
    fi
  fi
  warn "This does not look like a Raspberry Pi (/proc/device-tree/model)."
  warn "Continuing anyway — config paths may be missing on non-Pi systems."
}

find_boot_config() {
  local path
  for path in /boot/firmware/config.txt /boot/config.txt; do
    if [[ -f $path ]]; then
      printf '%s' "$path"
      return 0
    fi
  done
  return 1
}

# Append line to $file if no existing line matches $pattern (extended regex).
ensure_boot_config_line() {
  local file="$1"
  local pattern="$2"
  local line="$3"
  local label="$4"

  if sudo grep -qE "$pattern" "$file"; then
    log "  OK: $label"
    return 0
  fi

  log "  Adding: $line"
  printf '%s\n' "$line" | sudo tee -a "$file" >/dev/null
  NEEDS_REBOOT=1
}

configure_camera() {
  local config_txt
  if ! config_txt="$(find_boot_config)"; then
    warn "Boot config not found (/boot/firmware/config.txt or /boot/config.txt) — skipping camera overlay setup."
    return 0
  fi

  log "Camera config: $config_txt"

  if sudo grep -qE '^[[:space:]]*camera_auto_detect=0' "$config_txt"; then
    log "  OK: camera_auto_detect=0"
  elif sudo grep -qE '^[[:space:]]*camera_auto_detect=' "$config_txt"; then
    log "  Setting camera_auto_detect=0"
    sudo sed -i -E 's/^[[:space:]]*camera_auto_detect=.*/camera_auto_detect=0/' "$config_txt"
    NEEDS_REBOOT=1
  else
    log "  Adding: camera_auto_detect=0"
    printf '%s\n' 'camera_auto_detect=0' | sudo tee -a "$config_txt" >/dev/null
    NEEDS_REBOOT=1
  fi

  ensure_boot_config_line "$config_txt" 'dtoverlay=imx708' 'dtoverlay=imx708' 'dtoverlay=imx708'

  if sudo grep -qE 'dtoverlay=vc4-kms-v3d,cma-128' "$config_txt"; then
    log "  OK: dtoverlay=vc4-kms-v3d,cma-128"
  elif sudo grep -qE 'dtoverlay=vc4-kms-v3d' "$config_txt"; then
    log "  Updating vc4-kms-v3d overlay to include cma-128"
    sudo sed -i -E 's/^([[:space:]]*dtoverlay=vc4-kms-v3d)(,cma-[0-9]+)?(.*)$/\1,cma-128\3/' "$config_txt"
    NEEDS_REBOOT=1
  else
    log "  Adding: dtoverlay=vc4-kms-v3d,cma-128"
    printf '%s\n' 'dtoverlay=vc4-kms-v3d,cma-128' | sudo tee -a "$config_txt" >/dev/null
    NEEDS_REBOOT=1
  fi
}

configure_h264_codec() {
  local codec_conf="/etc/modprobe.d/bcm2835-codec.conf"
  local modules_file="/etc/modules"

  log "H.264 encoder: $codec_conf"
  if [[ ! -f $codec_conf ]]; then
    log "  Creating $codec_conf"
    printf '%s\n' 'options bcm2835-codec' | sudo tee "$codec_conf" >/dev/null
    NEEDS_REBOOT=1
  elif ! sudo grep -qE '^[[:space:]]*options[[:space:]]+bcm2835-codec' "$codec_conf"; then
    log "  Adding: options bcm2835-codec"
    printf '%s\n' 'options bcm2835-codec' | sudo tee -a "$codec_conf" >/dev/null
    NEEDS_REBOOT=1
  else
    log "  OK: options bcm2835-codec"
  fi

  log "Kernel module: $modules_file"
  if [[ ! -f $modules_file ]]; then
    log "  Creating $modules_file"
    printf '%s\n' 'bcm2835-codec' | sudo tee "$modules_file" >/dev/null
    NEEDS_REBOOT=1
  elif ! sudo grep -qE '^[[:space:]]*bcm2835-codec([[:space:]]|$)' "$modules_file"; then
    log "  Adding: bcm2835-codec"
    printf '%s\n' 'bcm2835-codec' | sudo tee -a "$modules_file" >/dev/null
    NEEDS_REBOOT=1
  else
    log "  OK: bcm2835-codec"
  fi
}

install_packages() {
  log "Installing apt packages..."
  sudo apt-get update
  sudo apt-get install -y "${APT_PACKAGES[@]}"
}

configure_groups() {
  local group
  for group in video dialout; do
    if id -nG "$USER" | tr ' ' '\n' | grep -qx "$group"; then
      log "  OK: $USER is in group $group"
    else
      log "  Adding $USER to group $group"
      sudo usermod -aG "$group" "$USER"
      NEEDS_RELOGIN=1
    fi
  done
}

verify_gstreamer() {
  local elem
  for elem in libcamerasrc v4l2h264enc; do
    if gst-inspect-1.0 "$elem" >/dev/null 2>&1; then
      log "  OK: gst-inspect-1.0 $elem"
    else
      warn "gst-inspect-1.0 $elem failed — reboot after install if overlays/modules were just changed."
    fi
  done
}

install_systemd_service() {
  if [[ ! -f $TEMPLATE ]]; then
    echo "Missing template: $TEMPLATE" >&2
    exit 1
  fi

  chmod +x "$RASPI_DIR/run_detect_atags.sh" "$RASPI_DIR/run_camera_stream.sh"

  mkdir -p "$USER_UNIT_DIR"
  sed "s|@REPO_DIR@|${RASPI_DIR}|g" "$TEMPLATE" >"$DEST"

  systemctl --user daemon-reload
  systemctl --user enable "$UNIT_NAME"

  if systemctl --user start "$UNIT_NAME"; then
    log "Installed and started $UNIT_NAME"
  else
    warn "Installed $UNIT_NAME but start failed — check logs below"
  fi
}

main() {
  require_pi

  log "=== SO101 detect_atags Pi setup ==="
  log

  install_packages
  log

  configure_camera
  log

  configure_h264_codec
  log

  configure_groups
  log

  log "Verifying GStreamer elements..."
  verify_gstreamer
  log

  install_systemd_service

  log
  log "=== Done ==="
  if [[ $NEEDS_REBOOT -eq 1 ]]; then
    warn "Reboot required for boot config / kernel module changes."
  fi
  if [[ $NEEDS_RELOGIN -eq 1 ]]; then
    warn "Log out and back in (or reboot) for video/dialout group membership."
  fi
  if [[ $NEEDS_REBOOT -eq 0 && $NEEDS_RELOGIN -eq 0 ]]; then
    log "No reboot or re-login required."
  fi

  log
  log "Status:  systemctl --user status $UNIT_NAME"
  log "Logs:    journalctl --user -u $UNIT_NAME -f"
  log "Stop:    $RASPI_DIR/stop-detect-atags-service.sh"
  log "Disable: $RASPI_DIR/stop-detect-atags-service.sh --disable"
}

main "$@"
