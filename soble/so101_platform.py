"""BLE interface to so101base.ino (RobotState notify + RobotCommand write)."""

from __future__ import annotations

import asyncio
import multiprocessing as mp
import struct
import time
from typing import Optional

from bleak import BleakClient, BleakScanner

SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331d914"
CHAR_UUID = "beb5483e-36e1-4688-b7f2-e6a6a6d74324"

CMD_LEN = 11
ARM_MOTOR_COUNT = 6  # 6 x 12-bit positions in armPos[9]
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


def _pack_robot_command(left: int, right: int, arm_packed: bytes) -> bytes:
    if len(arm_packed) != 9:
        arm_packed = bytes(9)
    return struct.pack("<bb", left, right) + arm_packed


def _unpack_robot_state(data: bytes) -> dict | None:
    if len(data) < STATE_LEN:
        return None
    enc_l, enc_r = _unpack_enc12(data[0:3])
    arm = _unpack_arm12(data[3:12])
    quat = struct.unpack("<hhhh", data[12:20])
    ntags = min(data[20], 10)
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
    }


def _format_state_line(state: dict) -> str:
    enc_l, enc_r = state["enc"]
    arm = state["arm"]
    qw, qx, qy, qz = (v / 1000.0 for v in state["quat"])
    parts = [
        f"enc_raw L={enc_l} R={enc_r}",
        f"arm_raw={' '.join(str(a) for a in arm)}",
        f"quat_wxyz=({qw:.4f},{qx:.4f},{qy:.4f},{qz:.4f})",
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


def _clear_state(got_state: mp.Value, last_notify: mp.Value, ntags: mp.Value, lock: mp.Lock) -> None:
    with lock:
        got_state.value = False
        last_notify.value = 0.0
        ntags.value = 0

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
    got_state: mp.Value,
    last_notify: mp.Value,
    enc: mp.Array,
    quat: mp.Array,
    ntags: mp.Value,
    tag_blob: mp.Array,
    lock: mp.Lock,
) -> bool:
    def on_notify(_handle: int, data: bytearray) -> None:
        state = _unpack_robot_state(bytes(data))
        if state is None:
            return
        _publish_state(state, got_state, last_notify, enc, quat, ntags, tag_blob, lock)
        if log_state:
            print(_format_state_line(state), flush=True)

    def on_disconnect(_client: BleakClient) -> None:
        disconnect_event.set()

    disconnect_event = asyncio.Event()
    disconnect_event.clear()
    print(f"Connecting to {dev_path}...", flush=True)
    async with BleakClient(dev_path, disconnected_callback=on_disconnect) as client:
        try:
            await client.exchange_mtu(512)
        except Exception:
            pass
        await client.start_notify(CHAR_UUID, on_notify)
        print("Connected.", flush=True)

        try:
            while not stop.is_set() and not disconnect_event.is_set():
                with lock:
                    got = bool(got_state.value)
                    sn = float(last_notify.value)
                    left = int(left_cmd.value)
                    right = int(right_cmd.value)
                    arm = bytes(arm_packed[:])
                if got and sn > 0.0 and (time.monotonic() - sn) > STALE_NOTIFY_S:
                    print("No notify — reconnecting...", flush=True)
                    break
                payload = _pack_robot_command(left, right, arm)
                assert len(payload) == CMD_LEN
                await client.write_gatt_char(CHAR_UUID, payload, response=True)
                await asyncio.sleep(TX_INTERVAL)
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
    got_state: mp.Value,
    last_notify: mp.Value,
    enc: mp.Array,
    quat: mp.Array,
    ntags: mp.Value,
    tag_blob: mp.Array,
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
                got_state,
                last_notify,
                enc,
                quat,
                ntags,
                tag_blob,
                lock,
            )
        except Exception as exc:
            print(f"BLE error: {exc}", flush=True)
            disconnected = False
        if disconnected:
            print("Disconnected.", flush=True)

        _clear_state(got_state, last_notify, ntags, lock)
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
    got_state: mp.Value,
    last_notify: mp.Value,
    enc: mp.Array,
    quat: mp.Array,
    ntags: mp.Value,
    tag_blob: mp.Array,
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
            got_state,
            last_notify,
            enc,
            quat,
            ntags,
            tag_blob,
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
        self._arm_packed = mp.Array("B", 9)
        self._got_state = mp.Value("b", False)
        self._last_notify = mp.Value("d", 0.0)
        self._enc = mp.Array("i", 2)
        self._quat = mp.Array("d", 4)
        self._quat[0] = 1.0
        self._ntags = mp.Value("i", 0)
        self._tag_blob = mp.Array("B", TAG_BLOB_LEN)
        self._proc: mp.Process | None = None

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

    def getApriltagTags(self) -> AprilTagList | None:
        """Tag list from last notify, or None if no state received yet."""
        with self._lock:
            if not self._got_state.value:
                return None
        return _unpack_tags_from_shared(
            self._ntags,
            self._tag_blob,
            self._lock,
            self._tag_corner_scale,
            (self._tag_origin_x, self._tag_origin_y),
        )

    def setSO101Position(self, joints: list[int]) -> None:
        packed = _pack_arm12(joints)
        with self._lock:
            for i, byte in enumerate(packed):
                self._arm_packed[i] = byte

    def setLeftRightMotors(self, left: int, right: int) -> None:
        with self._lock:
            self._left_cmd.value = max(-125, min(125, int(left)))
            self._right_cmd.value = max(-125, min(125, int(right)))

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
                self._got_state,
                self._last_notify,
                self._enc,
                self._quat,
                self._ntags,
                self._tag_blob,
                self._lock,
            ),
            daemon=True,
        )
        self._proc.start()

    def stop(self) -> None:
        self._stop.set()
        if self._proc is not None:
            self._proc.join(timeout=3.0)
            self._proc = None
        _clear_state(self._got_state, self._last_notify, self._ntags, self._lock)
