# soble — SO101 host BLE API

Python package for host-side control and teleop of the SO101 platform over BLE, plus USB **leader** arm mapping.

Install name on PyPI-style indexes: **`soble`** (import: `from soble import SO101Platform, SO101Leader`).

---

## Install

### From git

Repo: [timrobot/SOBle](https://github.com/timrobot/SOBle) (`master`). The repository root **is** the `soble` package.

```bash
pip install "soble @ git+https://github.com/timrobot/SOBle.git@master"
```

### Editable install (development)

```bash
git clone https://github.com/timrobot/SOBle.git
cd SOBle
pip install -e .
```

---

## Package layout

```
pyproject.toml            # pip metadata (repo root)
soble/                    # importable package
    __init__.py
    so101_platform.py
    so101_leader.py
examples/
  angular_config.json     # example joint mapping (used by example scripts)
  lead-follow.py
  viz_apriltags.py
```

---

## Quick usage

```python
from soble import SO101Leader, SO101Platform

config = {
    "leader": {
        "J1": [675, 3270],
        "J2": [1801, 350],
        "J3": [553, 2544],
        "J4": [105, 2408],
        "J5": [2330, 6144],
        "J6": [2046, 3261],
    },
    "follower": {
        "J1": [671, 3387],
        "J2": [831, 3180],
        "J3": [861, 3064],
        "J4": [913, 3202],
        "J5": [78, 3906],
        "J6": [1974, 3417],
    },
}
leader_limits, follower_limits = SO101Leader.limits_from_config(config)

leader = SO101Leader("/dev/ttyACM0", leader_limits, follower_limits)
platform = SO101Platform("Capybara")

leader.start()
platform.start()

positions = leader.getPositions()  # 6 ints, 0..4095
platform.setSO101Position(positions)
platform.setLeftRightMotors(0, 0)

platform.stop()
leader.stop()
```

Optional: load the same structure from a JSON file (e.g. `config.json`):

```python
leader_limits, follower_limits = SO101Leader.load_config("config.json")
```

The repo’s example scripts use `examples/angular_config.json`.

---

## Examples

```bash
pip install -e .   # if developing from a clone

cd examples
python lead-follow.py
python viz_apriltags.py /dev/ttyACM0
```

| Script | What it does |
|--------|----------------|
| `examples/lead-follow.py` | Leader → follower joint mirror; wheels at 0 |
| `examples/viz_apriltags.py` | WASD drive, leader mirror, tag overlay |

AprilTag overlays need the robot’s Pi camera running tag detection and forwarding over BLE (separate Pi setup on the robot).

---

## What the robot provides

- **Two wheels** — `setLeftRightMotors(left, right)`
- **Six arm joints** — `setSO101Position(joints)`
- **Encoders** — `getEncoders()`
- **IMU** — `getIMUQuaternion()`
- **AprilTags** (from Pi camera via robot) — `getApriltagTags()`

Default BLE device name substring: **`Capybara`**.

---

## Technical reference — `SO101Platform`

Module: `soble.so101_platform`. BLE I/O runs in a background **process**. Call `start()` before getters/setters and `stop()` when done.

### Lifecycle

| Method | Returns | Notes |
|--------|---------|--------|
| `SO101Platform(device_name: str, *, reconnect_delay_s=0.25, log_state=True)` | — | Does not connect until `start()`. |
| `start()` | `None` | Scan by name, connect, subscribe @ 25 Hz, TX loop. |
| `stop()` | `None` | Stop worker; clear tag state. |
| `running` (property) | `bool` | BLE process alive. |
| `last_notify_age_s()` | `float \| None` | Seconds since last state notify; `None` if none yet. |

### Commands (host → robot)

| Method | Arguments | Range / type |
|--------|-----------|----------------|
| `setLeftRightMotors(left, right)` | `left: int`, `right: int` | Each **−125 … 125** (clamped). |
| `setSO101Position(joints)` | `joints: list[int]` | **6** values, each **0 … 4095** (12-bit). Order **J1 … J6**. |

### State (robot → host)

| Method | Returns | Range / type |
|--------|-----------|----------------|
| `getEncoders()` | `tuple[int, int]` | `(left, right)`, each **0 … 4095**. |
| `getIMUQuaternion()` | `tuple[float, float, float, float]` | Unit quaternion **(w, x, y, z)**. |
| `getApriltagTags()` | `list[tuple[int, list[tuple[float, float]]]] \| None` | `None` before first notify. Up to **10** tags: `(tag_id, corners_px)`. |

### AprilTags — corners and image

| Item | Value |
|------|--------|
| Image size | **1280 × 720** pixels |
| Tag family | **tag16h5** |
| Corner order | **lb, rb, rt, lt** — each corner `(x, y)` float pixels |
| `x` range | **0 … 1280** |
| `y` range | **0 … 720** |
| Max tags | **10** |

**Example** — two tag16h5 tags in view (`None` until the first state notify; `[]` when connected but none detected):

```python
>>> platform.getApriltagTags()
[
    (0, [
        (612.4, 388.2),   # lb
        (667.8, 388.2),   # rb
        (667.8, 331.6),   # rt
        (612.4, 331.6),   # lt
    ]),
    (3, [
        (118.4, 142.3),
        (182.0, 140.0),
        (180.2, 198.1),
        (120.0, 200.5),
    ]),
]
```

Each list entry is `(tag_id, corners_px)` with **four** `(x, y)` pixel pairs in **lb → rb → rt → lt** order.

Reconnects if no state notify for **0.3 s** after the first good packet.

---

## Technical reference — `SO101Leader`

Module: `soble.so101_leader`. Leader USB serial runs in a background **process**.

| Method | Returns | Notes |
|--------|---------|--------|
| `SO101Leader.limits_from_config(cfg: dict)` | `tuple[list[JointLimits], list[JointLimits]]` | `cfg` has `"leader"` / `"follower"` keys **J1…J6**, each `[min, max]`. |
| `SO101Leader.load_config(path: str \| Path)` | `tuple[list[JointLimits], list[JointLimits]]` | Same layout as a JSON file (e.g. `config.json`). |
| `SO101Leader(port, leader_limits, follower_limits, baud=1000000)` | — | **`port` required** (e.g. `/dev/ttyACM0`). |
| `start()` / `stop()` | `None` | Start/stop SYNC_READ loop ~100 Hz. |
| `getPositions()` | `list[int]` | **6** follower-mapped joint raws **0 … 4095**, or **`[]`** if not ready. |
| `status_line()` | `str` | Debug: leader vs follower raw per joint. |

---

## Troubleshooting (host)

- **Lead-follow idle** — wrong leader port; `getPositions()` empty until first good read.
- **BLE reconnect loop** — robot off or out of range; BLE name must match `SO101Platform("…")`.
- **No tags in viewer** — Pi detector not running; tag16h5 in camera view.
- **`ModuleNotFoundError: soble`** — run `pip install` from git or `pip install -e .` in this repo.
