# Raspberry Pi — AprilTag detection & camera stream

The Pi camera finds **tag16h5** tags and streams them to the robot over USB. Host software in the **SOBle** repo (`pip install soble`) reads tags and can request an RTP camera stream over BLE—see that repo for APIs and examples.

## Pi setup

Run once on the Pi (idempotent — safe if some steps were already done manually):

```bash
cd ~/SO101Base/SOBle/raspi   # adjust path to your clone
chmod +x install-detect-atags-service.sh run_detect_atags.sh
./install-detect-atags-service.sh
```

The installer:

1. Installs apt packages (camera, AprilTags, serial, GStreamer)
2. Ensures `/boot/firmware/config.txt` (or `/boot/config.txt`) has imx708 + KMS overlays
3. Ensures `bcm2835-codec` modprobe + `/etc/modules` entry for hardware H.264
4. Adds your user to `video` and `dialout`
5. Installs and enables the `detect-atags` systemd user service

Reboot if prompted (boot config / codec module). Log out and back in if prompted (`video` / `dialout`).

### Manual reference

<details>
<summary>What the installer configures (if you prefer to edit by hand)</summary>

#### 1. Camera (`/boot/firmware/config.txt`)

```ini
camera_auto_detect=0
dtoverlay=imx708
dtoverlay=vc4-kms-v3d,cma-128
```

Reboot after editing.

#### 2. Hardware H.264 encoder

`/etc/modprobe.d/bcm2835-codec.conf`:

```
options bcm2835-codec
```

`/etc/modules` — add:

```
bcm2835-codec
```

Reboot after editing.

#### 3. Packages

```bash
sudo apt-get update
sudo apt-get install -y \
  python3-picamera2 python3-libcamera \
  python3-opencv python3-apriltag python3-serial python3-numpy \
  python3-gi gstreamer1.0-tools \
  gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad gstreamer1.0-libcamera

sudo usermod -aG video,dialout "$USER"
```

Log out and back in (or reboot).

</details>

| Component | Packages |
|-----------|----------|
| Pi camera / AprilTags | `python3-picamera2`, `python3-libcamera`, `python3-opencv`, `python3-apriltag` |
| USB serial to ESP32 | `python3-serial` |
| RTP send (`camera_stream.py`) | `python3-gi`, `gstreamer1.0-*`, `gstreamer1.0-libcamera` |

---

## AprilTag detection (`detect_atags.py`)

Forwards tag detections to the ESP32 over USB. Used with `viz-apriltags.py` on a laptop.

### Run manually

```bash
cd ~/SO101Base/SOBle/raspi
chmod +x run_detect_atags.sh
./run_detect_atags.sh
```

Optional serial device:

```bash
SERIAL_PORT=/dev/ttyACM0 ./run_detect_atags.sh
```

### Autostart at login

Included in `./install-detect-atags-service.sh` (see [Pi setup](#pi-setup)). To install only the service after manual setup, run that script again — it is idempotent.

```bash
journalctl --user -u detect-atags -f
```

### Stop autostart

```bash
systemctl --user stop detect-atags
systemctl --user disable detect-atags
```

---

## RTP/UDP camera stream (`camera_stream.py`)

Sends **720p30 H.264** over RTP/UDP via GStreamer (`libcamerasrc` → `v4l2h264enc` → `rtph264pay` → `udpsink`). Target: Pi Zero 2 W with libcamera.

**Cannot run at the same time as `detect_atags.py`** — both need the camera. `detect_atags.py` starts/stops the stream when it receives a BLE-forwarded `CMD_STREAM` (`'A'`) from the host.

### Run manually (Pi)

```bash
cd ~/SO101Base/SOBle/raspi
chmod +x run_camera_stream.sh
RTP_HOST=192.168.1.100 RTP_PORT=5000 ./run_camera_stream.sh
```

Environment variables: `RTP_HOST`, `RTP_PORT` (default broadcast `255.255.255.255:5000`), `STREAM_WIDTH`, `STREAM_HEIGHT`, `STREAM_FPS`, `STREAM_BITRATE`.

### Receive on a laptop (GStreamer CLI)

```bash
gst-launch-1.0 -v udpsrc port=5000 \
  caps='application/x-rtp,media=video,encoding-name=H264,payload=96' \
  ! rtph264depay ! h264parse ! avdec_h264 ! autovideosink sync=false
```

### Receive via SOBle (BLE → Pi → RTP)

On the host (needs GStreamer receive deps — same `python3-gi` + plugin packages as above, plus a decoder such as `gstreamer1.0-plugins-good` / `gstreamer1.0-libav`):

```bash
pip install "soble @ git+https://github.com/timrobot/SOBle.git@master"
cd SOBle/examples
python open-camera-stream.py
```

The host sends **its own LAN IP and UDP port** (default `5000`) to the Pi; the Pi streams H.264 to that address.

---

## Troubleshooting

- **Serial permission** — user in `dialout`; check `SERIAL_PORT` (`/dev/ttyACM0` default).
- **Camera** — user in `video`; reboot after install; only one process may use the camera.
- **Stream won't start** — confirm WiFi on the Pi (`nmcli`); ESP32 USB serial bridge must forward `CMD_STREAM` to `detect_atags.py`.
- **GStreamer** — `gst-inspect-1.0 libcamerasrc` and `gst-inspect-1.0 v4l2h264enc` should succeed on the Pi; `bcm2835-codec` loaded (`lsmod | grep bcm2835`).

---

## Quick start: lead–follow (laptop)

See the **SOBle** repository README.

```bash
pip install "soble @ git+https://github.com/timrobot/SOBle.git@master"
python -m pip show soble   # verify install
git clone https://github.com/timrobot/SOBle.git && cd SOBle/examples
python lead-follow.py
```

Leader arm on USB; robot BLE name `Capybara`; joint mapping in `examples/angular_config.json`.

---

## Quick start: view AprilTags (laptop)

1. Start detection on the Pi (`./run_detect_atags.sh` or autostart service).
2. Hold a printed **tag16h5** in front of the Pi camera.
3. On the host:

```bash
pip install "soble @ git+https://github.com/timrobot/SOBle.git@master"
git clone https://github.com/timrobot/SOBle.git && cd SOBle/examples
python viz-apriltags.py
```

You should see tag outlines and bitmap overlays when tags are detected.
