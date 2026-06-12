"""BLE interface to so101base.ino (RobotState notify + RobotCommand write)."""

from __future__ import annotations

import asyncio
import ipaddress
import math
import multiprocessing as mp
import platform
import queue
import socket
import struct
import threading
import time
from collections.abc import Callable
from typing import Optional

import cv2
import numpy as np
from bleak import BleakClient, BleakScanner

from soble import camera_stream as host_camera_stream

SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331d914"
CHAR_UUID = "beb5483e-36e1-4688-b7f2-e6a6a6d74324"

CMD_ACTUATOR_LEN = 13  # cmd + left + right + arm[9] + enabled — matches RobotCommand on ESP32
CMD_RASPI_LEN = 12  # cmd + raspi union (forwarded as 7 bytes on ESP32)
CMD_ACTUATORS = ord("0")
CMD_TAG16H5 = ord("1")
CMD_TAG25H9 = ord("2")
CMD_TAG36H11 = ord("3")
CMD_STREAM = ord("A")
CMD_WIFIUSER = ord("U")
CMD_WIFIPASS = ord("P")
WIFI_CRED_MAX = 128  # matches ESP32 So101-Platform.ino
ARM_MOTOR_COUNT = 6  # 6 x 12-bit positions in armPos[9]
ARM_ENABLE_MASK = 0x3F  # bit0=J1 .. bit5=J6 — all arm joints engaged
ARM_CENTER_RAW = 2048  # mid of 0..4095 — default when leader not connected
STATE_LEN = 201
TAG_INFO_LEN = 18
MAX_TAGS = 10
TAG_BLOB_LEN = MAX_TAGS * TAG_INFO_LEN

TX_INTERVAL = 0.04  # 25 Hz — matches BLE_TX_INTERVAL on ESP32
DEFAULT_RECONNECT_DELAY_S = 0.25
STALE_NOTIFY_S = 0.3  # reconnect if no RobotState notify within this window

# Must match raspi/detect_atags.py (origin 640,360 and * 25 packing)
TAG_ORIGIN_X = 640.0
TAG_ORIGIN_Y = 360.0
TAG_CORNER_SCALE = 25.0

AprilTagList = list[tuple[int, tuple[int, ...]]]


def _is_lan_ipv4(ip: str) -> bool:
    try:
        addr = ipaddress.IPv4Address(ip)
    except ValueError:
        return False
    return not addr.is_loopback and (addr.is_private or addr.is_link_local)


def _ipv4_from_interface(ifname: str) -> str | None:
    """Linux/macOS: IPv4 bound to a named interface."""
    if platform.system() == "Windows":
        return None
    try:
        import fcntl
    except ImportError:
        return None
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        ifreq = struct.pack("256s", ifname[:15].encode())
        res = fcntl.ioctl(sock.fileno(), 0x8915, ifreq)  # SIOCGIFADDR
        return socket.inet_ntoa(res[20:24])
    except OSError:
        return None
    finally:
        sock.close()


def get_lan_ip() -> str:
    """Return this computer's IPv4 on the local network (Wi‑Fi/Ethernet), not the public WAN address.

    Uses the default-route interface when possible; otherwise scans non-loopback interfaces.
    """
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            # No packets need to reach the internet; picks the LAN-facing interface.
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            if _is_lan_ipv4(ip):
                return ip
    except OSError:
        pass

    if platform.system() != "Windows":
        prefer = ("wlan", "wlp", "wifi", "en", "eth")
        ranked: list[tuple[int, str]] = []
        for _, ifname in socket.if_nameindex():
            name = ifname.decode() if isinstance(ifname, bytes) else ifname
            if name == "lo" or name.startswith(("docker", "br-", "veth", "virbr")):
                continue
            ip = _ipv4_from_interface(name)
            if ip and _is_lan_ipv4(ip):
                rank = next(
                    (i for i, prefix in enumerate(prefer) if name.startswith(prefix)),
                    len(prefer),
                )
                ranked.append((rank, ip))
        if ranked:
            ranked.sort(key=lambda item: item[0])
            return ranked[0][1]

    raise RuntimeError(
        "Could not determine LAN IPv4; pass host= to enableCameraStreamMode()"
    )


def _pack_arm12(joints: list[int]) -> bytes:
    packed = bytearray(9)
    byteidx = 0
    insert2 = True
    for v in joints:
        v &= 0x0FFF
        if insert2:
            packed[byteidx] = v & 0xFF
            byteidx += 1
            packed[byteidx] = (v >> 8) & 0x0F
        else:
            packed[byteidx] = ((v & 0x0F) << 4) | packed[byteidx]
            byteidx += 1
            packed[byteidx] = (v >> 4) & 0xFF
            byteidx += 1
        insert2 = not insert2
    return bytes(packed)


def _unpack_arm12(packed: bytes) -> list[int]:
    out: list[int] = []
    byteidx = 0
    extract2 = True
    b = 0
    for _ in range(ARM_MOTOR_COUNT):
        a = packed[byteidx]
        byteidx += 1
        if extract2:
            b = packed[byteidx]
            byteidx += 1
            out.append(((b & 0x0F) << 8) | a)
            b >>= 4
        else:
            out.append((a << 4) | b)
        extract2 = not extract2
    return out


def _unpack_enc12(packed: bytes) -> tuple[int, int]:
    left = packed[0] | ((packed[1] & 0x0F) << 8)
    right = ((packed[1] >> 4) & 0x0F) | (packed[2] << 4)
    return left & 0x0FFF, right & 0x0FFF


def _pack_robot_command(
    left: int, right: int, arm_packed: bytes, *, arm_disabled: bool = False
) -> bytes:
    """BLE teleop: cmd='0' + left + right + arm[9] + enabled (bit0=J1 .. bit5=J6)."""
    if len(arm_packed) != 9:
        arm_packed = _pack_arm12([ARM_CENTER_RAW] * ARM_MOTOR_COUNT)
    left = max(-125, min(125, int(left)))
    right = max(-125, min(125, int(right)))
    enabled = 0 if arm_disabled else ARM_ENABLE_MASK
    return struct.pack("<Bbb", CMD_ACTUATORS, left, right) + arm_packed + bytes([enabled])


def _pack_raspi_ble_command(cmd: int, ip: str = "0.0.0.0", port: int = 0) -> bytes:
    """BLE → ESP32 → USB → Pi (detect_atags.py). cmd is '1'/'2'/'3'/'A'."""
    ip_le = int(ipaddress.IPv4Address(ip)).to_bytes(4, "little")
    body = ip_le + struct.pack("<H", int(port) & 0xFFFF) + b"\x00" * 5
    return struct.pack("<B", cmd & 0xFF) + body


def _pack_wifi_user(ssid: str) -> bytes:
    """BLE → ESP32: 'U' + SSID bytes (no NUL). Max 127 UTF-8 bytes."""
    raw = ssid.encode("utf-8")
    if not raw or len(raw) > WIFI_CRED_MAX - 1:
        raise ValueError(f"SSID must be 1..{WIFI_CRED_MAX - 1} UTF-8 bytes")
    return bytes([CMD_WIFIUSER]) + raw


def _pack_wifi_pass(password: str) -> bytes:
    """BLE → ESP32: 'P' + password bytes (no NUL). Max 127 UTF-8 bytes."""
    raw = password.encode("utf-8")
    if not raw or len(raw) > WIFI_CRED_MAX - 1:
        raise ValueError(f"password must be 1..{WIFI_CRED_MAX - 1} UTF-8 bytes")
    return bytes([CMD_WIFIPASS]) + raw


def _unpack_robot_state(data: bytes) -> dict | None:
    if len(data) < STATE_LEN:
        return None
    enc_l, enc_r = _unpack_enc12(data[0:3])
    arm = _unpack_arm12(data[3:12])
    quat = struct.unpack("<hhhh", data[12:20])
    ntags = min(data[20] & 0x1F, 10)
    raspi_alive = (data[20] & 0x80) != 0
    wifi_connected = (data[20] & 0x40) != 0
    tags: AprilTagList = []
    off = 21
    for _ in range(ntags):
        if off + TAG_INFO_LEN > len(data):
            break
        chunk = data[off : off + TAG_INFO_LEN]
        tag_id = struct.unpack_from("<H", chunk, 0)[0]
        corners = struct.unpack_from("<8h", chunk, 2)
        tags.append((tag_id, corners))
        off += TAG_INFO_LEN
    return {
        "enc": (enc_l, enc_r),
        "arm": arm,
        "quat": quat,
        "ntags": ntags,
        "tags": tags,
        "raspi": raspi_alive,
        "wifi": wifi_connected,
    }


def _quat_to_rph_deg(qw: float, qx: float, qy: float, qz: float) -> tuple[float, float, float]:
    """Roll, pitch, heading (deg) — same convention as So101-Platform.ino Madgwick output."""
    roll = math.degrees(
        math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    )
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(90.0, sinp)
    else:
        pitch = math.degrees(math.asin(sinp))
    heading = math.degrees(
        math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    )
    return roll, pitch, heading


def _format_state_line(state: dict) -> str:
    enc_l, enc_r = state["enc"]
    arm = state["arm"]
    qw, qx, qy, qz = (v / 1000.0 for v in state["quat"])
    roll, pitch, heading = _quat_to_rph_deg(qw, qx, qy, qz)
    parts = [
        f"wheel_raw L={enc_l} R={enc_r}",
        f"arm_raw={' '.join(str(a) for a in arm)}",
        f"roll={roll:6.1f} pitch={pitch:6.1f} heading={heading:6.1f}",
    ]
    if state["ntags"] > 0:
        parts.append(f"ntags={state['ntags']}")
    return " | ".join(parts)


def _publish_state(
    state: dict,
    got_state: mp.Value,
    last_notify: mp.Value,
    enc: mp.Array,
    quat: mp.Array,
    ntags: mp.Value,
    tag_blob: mp.Array,
    raspi: mp.Value,
    wifi: mp.Value,
    lock: mp.Lock,
) -> None:
    qw, qx, qy, qz = (v / 1000.0 for v in state["quat"])
    enc_l, enc_r = state["enc"]
    n = min(state["ntags"], MAX_TAGS)
    with lock:
        enc[0] = enc_l
        enc[1] = enc_r
        quat[0], quat[1], quat[2], quat[3] = qw, qx, qy, qz
        ntags.value = n
        raspi.value = bool(state["raspi"])
        wifi.value = bool(state["wifi"])
        for i in range(TAG_BLOB_LEN):
            tag_blob[i] = 0
        off = 0
        for tag_id, corners in state["tags"][:n]:
            chunk = struct.pack("<H8h", tag_id, *corners)
            for j, byte in enumerate(chunk):
                tag_blob[off + j] = byte
            off += TAG_INFO_LEN
        got_state.value = True
        last_notify.value = time.monotonic()


def _clear_state(
    got_state: mp.Value,
    last_notify: mp.Value,
    ntags: mp.Value,
    raspi: mp.Value,
    wifi: mp.Value,
    lock: mp.Lock,
) -> None:
    with lock:
        got_state.value = False
        last_notify.value = 0.0
        ntags.value = 0
        raspi.value = False
        wifi.value = False

def _corners_to_pixels(corners: tuple[int, ...], tag_corner_scale: float, tag_origin: tuple[float, float]) -> list[tuple[float, float]]:
    """Undo Pi packing: pixel = corner / scale + origin (detect_atags.py inverse)."""
    out: list[tuple[float, float]] = []
    for i in range(4):
        x = corners[i * 2] / tag_corner_scale + tag_origin[0]
        y = corners[i * 2 + 1] / tag_corner_scale + tag_origin[1]
        out.append((x, y))
    return out

def _unpack_tags_from_shared(ntags: mp.Value, tag_blob: mp.Array, lock: mp.Lock, tag_corner_scale: float, tag_origin: tuple[float, float]) -> AprilTagList:
    with lock:
        n = min(int(ntags.value), MAX_TAGS)
        blob = bytes(tag_blob[: TAG_BLOB_LEN])
    tags: AprilTagList = []
    for i in range(n):
        off = i * TAG_INFO_LEN
        chunk = blob[off : off + TAG_INFO_LEN]
        tag_id = struct.unpack_from("<H", chunk, 0)[0]
        corners = struct.unpack_from("<8h", chunk, 2)
        tags.append((tag_id, _corners_to_pixels(corners, tag_corner_scale, tag_origin)))
    return tags

def _estimate_tag_pose(corners: tuple[int, ...], tag_size: float, camera_params: tuple[float, float, int, int]) -> tuple[bool, np.ndarray, np.ndarray]:
    half_width = tag_size / 2.0
    object_points = np.array([
        [-half_width, -half_width, 0],
        [half_width, -half_width, 0],
        [half_width, half_width, 0],
        [-half_width, half_width, 0],
    ], dtype=np.float32)
    image_points = np.array(corners, dtype=np.float32)
    K = np.array([
        [camera_params[0], 0, camera_params[2]],
        [0, camera_params[0], camera_params[3]],
        [0, 0, 1],
    ], dtype=np.float32)
    dist = np.zeros(5)  # assume no distortion unless calibrated
    success, rvec, tvec = cv2.solvePnP(
        object_points,
        image_points,
        K,
        dist,
        flags=cv2.SOLVEPNP_ITERATIVE
    )
    R, _ = cv2.Rodrigues(rvec)
    return success, R, tvec


async def _find_device_address(device_name: str) -> Optional[str]:
    dev = await BleakScanner.find_device_by_filter(
        lambda d, _: bool(d.name and device_name.lower() in d.name.lower())
    )
    if dev is None:
        return None
    print(f"Found {dev.name} ({dev.address})", flush=True)
    return dev.address


async def _ble_session(
    dev_path: str,
    log_state: bool,
    stop: mp.Event,
    left_cmd: mp.Value,
    right_cmd: mp.Value,
    arm_packed: mp.Array,
    pending_ble: mp.Array,
    pending_ble_valid: mp.Value,
    arm_disabled: mp.Value,
    got_state: mp.Value,
    last_notify: mp.Value,
    enc: mp.Array,
    quat: mp.Array,
    ntags: mp.Value,
    tag_blob: mp.Array,
    raspi: mp.Value,
    wifi: mp.Value,
    ble_write_queue: mp.Queue,
    lock: mp.Lock,
) -> bool:
    def on_notify(_handle: int, data: bytearray) -> None:
        state = _unpack_robot_state(bytes(data))
        if state is None:
            return
        _publish_state(
            state,
            got_state,
            last_notify,
            enc,
            quat,
            ntags,
            tag_blob,
            raspi,
            wifi,
            lock,
        )
        if log_state:
            print(_format_state_line(state), flush=True)

    def on_disconnect(_client: BleakClient) -> None:
        disconnect_event.set()

    disconnect_event = asyncio.Event()
    disconnect_event.clear()
    print(f"Connecting to {dev_path}...", flush=True)
    async with BleakClient(
        dev_path, disconnected_callback=on_disconnect, timeout=20.0
    ) as client:
        try:
            await client.exchange_mtu(512)
        except Exception as mtu_exc:
            print(
                f"MTU exchange skipped: {type(mtu_exc).__name__}: {mtu_exc!r}",
                flush=True,
            )
        print("Subscribing to notify...", flush=True)
        await client.start_notify(CHAR_UUID, on_notify)
        print("Connected.", flush=True)

        last_sent = 0.0
        try:
            while not stop.is_set() and not disconnect_event.is_set():
                now = time.monotonic()
                with lock:
                    got = bool(got_state.value)
                    sn = float(last_notify.value)
                    left = int(left_cmd.value)
                    right = int(right_cmd.value)
                    arm = bytes(arm_packed[:])
                if got and sn > 0.0 and (now - sn) > STALE_NOTIFY_S:
                    print("No notify — reconnecting...", flush=True)
                    break

                try:
                    queued = ble_write_queue.get_nowait()
                except queue.Empty:
                    queued = None
                if queued is not None:
                    await client.write_gatt_char(CHAR_UUID, queued, response=True)
                    last_sent = time.monotonic()
                    continue

                if now - last_sent >= TX_INTERVAL:
                    with lock:
                        if pending_ble_valid.value:
                            n = (
                                CMD_ACTUATOR_LEN
                                if pending_ble[0] == CMD_ACTUATORS
                                else CMD_RASPI_LEN
                            )
                            payload = bytes(pending_ble[:n])
                            pending_ble_valid.value = False
                        else:
                            payload = _pack_robot_command(
                                left, right, arm, arm_disabled=bool(arm_disabled.value)
                            )
                    assert len(payload) in (CMD_ACTUATOR_LEN, CMD_RASPI_LEN)
                    await client.write_gatt_char(CHAR_UUID, payload, response=True)
                    last_sent = time.monotonic()
                    continue

                remaining = TX_INTERVAL - (now - last_sent)
                if remaining > 0.001:
                    await asyncio.sleep(remaining)
                else:
                    await asyncio.sleep(0)
        finally:
            try:
                await client.stop_notify(CHAR_UUID)
            except Exception:
                pass
    return disconnect_event.is_set()


async def _ble_main(
    device_name: str,
    reconnect_delay_s: float,
    log_state: bool,
    stop: mp.Event,
    left_cmd: mp.Value,
    right_cmd: mp.Value,
    arm_packed: mp.Array,
    pending_ble: mp.Array,
    pending_ble_valid: mp.Value,
    arm_disabled: mp.Value,
    got_state: mp.Value,
    last_notify: mp.Value,
    enc: mp.Array,
    quat: mp.Array,
    ntags: mp.Value,
    tag_blob: mp.Array,
    raspi: mp.Value,
    wifi: mp.Value,
    ble_write_queue: mp.Queue,
    lock: mp.Lock,
) -> None:
    while not stop.is_set():
        print(f"Scanning for {device_name!r}...", flush=True)
        dev_path = await _find_device_address(device_name)
        if dev_path is None:
            print(
                f"No device with name containing {device_name!r}; "
                f"retry in {reconnect_delay_s:.0f}s",
                flush=True,
            )
            await asyncio.sleep(reconnect_delay_s)
            continue

        try:
            disconnected = await _ble_session(
                dev_path,
                log_state,
                stop,
                left_cmd,
                right_cmd,
                arm_packed,
                pending_ble,
                pending_ble_valid,
                arm_disabled,
                got_state,
                last_notify,
                enc,
                quat,
                ntags,
                tag_blob,
                raspi,
                wifi,
                ble_write_queue,
                lock,
            )
        except Exception as exc:
            detail = f"{type(exc).__name__}: {exc!r}"
            cause = exc.__cause__
            if cause is not None:
                detail += f" (caused by {type(cause).__name__}: {cause!r})"
            print(f"BLE error: {detail}", flush=True)
            disconnected = False
        if disconnected:
            print("Disconnected.", flush=True)

        _clear_state(got_state, last_notify, ntags, raspi, wifi, lock)
        if stop.is_set():
            break
        print(f"Reconnecting in {reconnect_delay_s:.0f}s...", flush=True)
        await asyncio.sleep(reconnect_delay_s)


def _ble_worker(
    device_name: str,
    reconnect_delay_s: float,
    log_state: bool,
    stop: mp.Event,
    left_cmd: mp.Value,
    right_cmd: mp.Value,
    arm_packed: mp.Array,
    pending_ble: mp.Array,
    pending_ble_valid: mp.Value,
    arm_disabled: mp.Value,
    got_state: mp.Value,
    last_notify: mp.Value,
    enc: mp.Array,
    quat: mp.Array,
    ntags: mp.Value,
    tag_blob: mp.Array,
    raspi: mp.Value,
    wifi: mp.Value,
    ble_write_queue: mp.Queue,
    lock: mp.Lock,
) -> None:
    asyncio.run(
        _ble_main(
            device_name,
            reconnect_delay_s,
            log_state,
            stop,
            left_cmd,
            right_cmd,
            arm_packed,
            pending_ble,
            pending_ble_valid,
            arm_disabled,
            got_state,
            last_notify,
            enc,
            quat,
            ntags,
            tag_blob,
            raspi,
            wifi,
            ble_write_queue,
            lock,
        )
    )


class SO101Platform:
    """BLE link in a child process; parent polls getters and updates command setters."""

    def __init__(
        self,
        device_name: str,
        *,
        reconnect_delay_s: float = DEFAULT_RECONNECT_DELAY_S,
        log_state: bool = True,
    ) -> None:
        self._device_name = device_name
        self._reconnect_delay_s = reconnect_delay_s
        self._log_state = log_state
        self._tag_origin_x = TAG_ORIGIN_X
        self._tag_origin_y = TAG_ORIGIN_Y
        self._tag_corner_scale = TAG_CORNER_SCALE

        self._lock = mp.Lock()
        self._stop = mp.Event()
        self._left_cmd = mp.Value("i", 0)
        self._right_cmd = mp.Value("i", 0)
        self._arm_packed = mp.Array(
            "B", _pack_arm12([ARM_CENTER_RAW] * ARM_MOTOR_COUNT)
        )
        self._pending_ble = mp.Array("B", CMD_ACTUATOR_LEN)
        self._pending_ble_valid = mp.Value("b", False)
        self._arm_disabled = mp.Value("b", False)
        self._got_state = mp.Value("b", False)
        self._last_notify = mp.Value("d", 0.0)
        self._enc = mp.Array("i", 2)
        self._quat = mp.Array("d", 4)
        self._quat[0] = 1.0
        self._ntags = mp.Value("i", 0)
        self._tag_blob = mp.Array("B", TAG_BLOB_LEN)
        self._raspi = mp.Value("b", False)
        self._wifi = mp.Value("b", False)
        self._ble_write_queue: mp.Queue[bytes] = mp.Queue()
        self._proc: mp.Process | None = None

        self._stream_proc: mp.Process | None = None
        self._stream_stop = mp.Event()
        self._frame_buf = mp.Array(
            "B", host_camera_stream.STREAM_FRAME_BYTES
        )
        self._frame_ready = mp.Event()
        self._frame_seq = mp.Value("Q", 0)
        self._frame_lock = mp.Lock()
        self._stream_dispatch_stop = threading.Event()
        self._stream_dispatch_thread: threading.Thread | None = None
        self._on_frame_callback: Callable[[np.ndarray], None] | None = None

    @property
    def running(self) -> bool:
        return self._proc is not None and self._proc.is_alive()

    def last_notify_age_s(self) -> float | None:
        with self._lock:
            if not self._got_state.value or self._last_notify.value <= 0.0:
                return None
            return time.monotonic() - float(self._last_notify.value)

    def getEncoders(self) -> tuple[int, int]:
        with self._lock:
            return int(self._enc[0]), int(self._enc[1])

    def getIMUQuaternion(self) -> tuple[float, float, float, float]:
        """Unit quaternion (w, x, y, z)."""
        with self._lock:
            return (
                float(self._quat[0]),
                float(self._quat[1]),
                float(self._quat[2]),
                float(self._quat[3]),
            )

    def getRaspiAlive(self) -> bool:
        """True if ESP32 has received serial from the Pi recently."""
        with self._lock:
            return bool(self._raspi.value)

    def getWifiConnected(self) -> bool:
        """True if the Pi reported WiFi connected (0xFF serial ack)."""
        with self._lock:
            return bool(self._wifi.value)

    def getApriltagTags(self, estimate_tag_pose=False, camera_params=[1270, 1270, 640, 360], tag_size=3) -> AprilTagList:
        """Tag list from last notify (empty list if no state received yet).
        If estimate_tag_pose is True, the tag list will be estimated from the tag corners.

        Args:
            estimate_tag_pose: Whether to estimate the tag pose from the tag corners.
            camera_params: A tuple of (focal_length, focal_length, image_width, image_height).
            tag_size: The size of the tag (cm or inches or meters)

        Returns:
            A list of tuples, each containing a tag id and a list of tag corners. [tag_id, [corner1, corner2, corner3, corner4]]
            If estimate_tag_pose is True, the tag list will be estimated from the tag corners: 
            [tag_id, [lb, rb, rt, lt], R, tvec]
        """
        with self._lock:
            if not self._got_state.value:
                return []
        if not estimate_tag_pose:
            return _unpack_tags_from_shared(
                self._ntags,
                self._tag_blob,
                self._lock,
                self._tag_corner_scale,
                (self._tag_origin_x, self._tag_origin_y),
            )
        else:
            poses = []
            tags = _unpack_tags_from_shared(
                self._ntags,
                self._tag_blob,
                self._lock,
                self._tag_corner_scale,
                (self._tag_origin_x, self._tag_origin_y),
            )
            for (tag_id, corners) in tags:
                success, R, tvec = _estimate_tag_pose(corners, tag_size, camera_params)
                if success:
                    poses.append((tag_id, corners, R, tvec))
            return poses

    def setSO101Position(self, joints: list[int]) -> None:
        packed = _pack_arm12(joints)
        with self._lock:
            for i, byte in enumerate(packed):
                self._arm_packed[i] = byte

    def setLeftRightMotors(self, left: int, right: int) -> None:
        with self._lock:
            self._left_cmd.value = max(-125, min(125, int(left)))
            self._right_cmd.value = max(-125, min(125, int(right)))

    def disable(self) -> None:
        """Release arm joints (torque off) so they can be moved by hand."""
        with self._lock:
            self._arm_disabled.value = True

    def enable(self) -> None:
        """Drive arm joint positions from setSO101Position (default after construction)."""
        with self._lock:
            self._arm_disabled.value = False

    def isArmEngaged(self) -> bool:
        """True unless disable() was called."""
        with self._lock:
            return not bool(self._arm_disabled.value)

    def _queue_ble_command(self, payload: bytes) -> None:
        if len(payload) not in (CMD_ACTUATOR_LEN, CMD_RASPI_LEN):
            raise ValueError(
                f"BLE command must be {CMD_ACTUATOR_LEN} or {CMD_RASPI_LEN} bytes, "
                f"got {len(payload)}"
            )
        with self._lock:
            for i in range(CMD_ACTUATOR_LEN):
                self._pending_ble[i] = 0
            for i, byte in enumerate(payload):
                self._pending_ble[i] = byte
            self._pending_ble_valid.value = True

    def connectToWifi(self, ssid: str, password: str) -> None:
        """Send SSID and password to the Pi via ESP32 (BLE 'U' then 'P' packets)."""
        if not self.running:
            raise RuntimeError("Call start() before connectToWifi()")
        self._ble_write_queue.put(_pack_wifi_user(ssid))
        self._ble_write_queue.put(_pack_wifi_pass(password))

    def _stop_camera_stream_receiver(self) -> None:
        self._stream_dispatch_stop.set()
        if self._stream_dispatch_thread is not None:
            self._stream_dispatch_thread.join(timeout=2.0)
            self._stream_dispatch_thread = None
        self._stream_dispatch_stop.clear()
        self._on_frame_callback = None

        self._stream_stop.set()
        host_camera_stream.stop_receive_stream(self._stream_proc, self._stream_stop)
        self._stream_proc = None
        self._stream_stop.clear()
        self._frame_ready.clear()
        with self._frame_lock:
            self._frame_seq.value = 0

    def _stream_frame_dispatch_loop(self) -> None:
        last_seq = 0
        shape = (
            host_camera_stream.STREAM_HEIGHT,
            host_camera_stream.STREAM_WIDTH,
            host_camera_stream.STREAM_CHANNELS,
        )
        nbytes = host_camera_stream.STREAM_FRAME_BYTES
        while not self._stream_dispatch_stop.is_set():
            if not self._frame_ready.wait(timeout=0.05):
                continue
            with self._frame_lock:
                seq = int(self._frame_seq.value)
            if seq == last_seq:
                continue
            last_seq = seq
            cb = self._on_frame_callback
            if cb is None:
                continue
            frame = (
                np.frombuffer(self._frame_buf, dtype=np.uint8, count=nbytes)
                .reshape(shape)
                .copy()
            )
            cb(frame)

    def setTagDetectionMode(self, family: str) -> None:
        """Forward tag-detection mode to the Pi ('tag16h5', 'tag25h9', 'tag36h11'). Stops host RTP receiver."""
        self._stop_camera_stream_receiver()
        fam = family.lower().replace("-", "").replace("_", "")
        cmd_by_family = {
            "tag16h5": CMD_TAG16H5,
            "tag25h9": CMD_TAG25H9,
            "tag36h11": CMD_TAG36H11,
        }
        if fam not in cmd_by_family:
            raise ValueError(f"Unknown tag family {family!r}")
        self._queue_ble_command(_pack_raspi_ble_command(cmd_by_family[fam]))

    def enableCameraStreamMode(
        self,
        host: str | None = None,
        port: int = 5000,
        onFrameCallback: Callable[[np.ndarray], None] | None = None,
    ) -> str:
        """Forward RTP stream command to the Pi and receive H.264 on this host (UDP port).

        Args:
            host: Destination IP for the Pi RTP sender; default is this PC's LAN IP.
            port: UDP port (default 5000).
            onFrameCallback: Called on a background thread for each BGR frame (1280×720).

        Returns the host IP sent to the Pi.
        """
        if not self.getWifiConnected():
            raise RuntimeError("WiFi not connected, cannot enable camera stream mode")

        self._stop_camera_stream_receiver()

        if host is None:
            host = get_lan_ip()

        self._on_frame_callback = onFrameCallback
        self._stream_proc = host_camera_stream.run_receive_stream(
            port,
            self._frame_buf,
            self._frame_ready,
            self._frame_seq,
            self._frame_lock,
            self._stream_stop,
        )
        if onFrameCallback is not None:
            self._stream_dispatch_stop.clear()
            self._stream_dispatch_thread = threading.Thread(
                target=self._stream_frame_dispatch_loop,
                name="so101-camera-frame-dispatch",
                daemon=True,
            )
            self._stream_dispatch_thread.start()

        self._queue_ble_command(_pack_raspi_ble_command(CMD_STREAM, host, port))
        return host

    def start(self) -> None:
        if self._proc is not None and self._proc.is_alive():
            return
        self._stop.clear()
        self._proc = mp.Process(
            target=_ble_worker,
            args=(
                self._device_name,
                self._reconnect_delay_s,
                self._log_state,
                self._stop,
                self._left_cmd,
                self._right_cmd,
                self._arm_packed,
                self._pending_ble,
                self._pending_ble_valid,
                self._arm_disabled,
                self._got_state,
                self._last_notify,
                self._enc,
                self._quat,
                self._ntags,
                self._tag_blob,
                self._raspi,
                self._wifi,
                self._ble_write_queue,
                self._lock,
            ),
            daemon=True,
        )
        self._proc.start()

    def stop(self) -> None:
        self._stop.set()
        self._stop_camera_stream_receiver()
        if self._proc is not None:
            self._proc.join(timeout=3.0)
            self._proc = None
        _clear_state(
            self._got_state,
            self._last_notify,
            self._ntags,
            self._raspi,
            self._wifi,
            self._lock,
        )
