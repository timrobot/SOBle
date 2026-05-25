# Raspberry Pi — AprilTag detection

The Pi camera finds **tag16h5** tags and streams them to the robot over USB. Host software in the **`SOBle`** git repo (`pip install soble`) reads those tags over BLE—see that repo for APIs, examples, and teleop.

## What the SO101 Platform is

A small mobile robot with:

- **Two drive wheels** (differential drive)
- **SO-101 arm** on top (six joints)
- **Wheel encoders** (left and right)
- **IMU** (orientation quaternion)
- **AprilTag detection** on the Pi camera (this directory)

## Pi setup

```bash
sudo apt-get update
sudo apt-get install -y python3-picamera2 python3-opencv python3-apriltag python3-serial python3-numpy
sudo usermod -aG video,dialout "$USER"
```

Log out and back in. Clone the repo to e.g. `~/SO101Base`.

### Run manually

```bash
cd ~/SO101Base/raspi
chmod +x run_detect_atags.sh
./run_detect_atags.sh
```

Optional port: `SERIAL_PORT=/dev/ttyACM0 ./run_detect_atags.sh`

### Autostart at login

```bash
cd ~/SO101Base/raspi
chmod +x install-detect-atags-service.sh run_detect_atags.sh
./install-detect-atags-service.sh
journalctl --user -u detect-atags -f
```

### Stop autostart

```bash
systemctl --user stop detect-atags
systemctl --user disable detect-atags
```

### Pi troubleshooting

- **Serial permission** — add user to `dialout`, check `SERIAL_PORT`.
- **Camera** — add user to `video`, reboot after install, ensure no other app uses the camera.

---

## Quick start: lead–follow (on your laptop)

See the **SOBle** repository README. Summary:

```bash
pip install "soble @ git+https://github.com/timrobot/SOBle.git@master"
python -m pip show soble   # verify install
git clone https://github.com/timrobot/SOBle.git && cd SOBle/examples
python lead-follow.py
```

Leader arm on USB (pass serial port to the example script), robot BLE name `Capybara`; example joint mapping is `examples/angular_config.json` in the SOBle repo.

---

## Quick start: view AprilTags (on your laptop)

1. Start detection on the Pi (above).
2. Hold a printed **tag16h5** in front of the Pi camera.
3. On the host:

```bash
pip install "soble @ git+https://github.com/timrobot/SOBle.git@master"
git clone https://github.com/timrobot/SOBle.git && cd SOBle/examples
python viz_apriltags.py /dev/ttyACM0
```

You should see tag outlines and bitmap overlays on the 1280×720 view when tags are detected.
