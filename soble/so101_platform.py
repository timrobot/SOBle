"""BLE interface to so101base.ino (RobotState notify + RobotCommand write).
"""

from __future__ import annotations

import asyncio
import multiprocessing as mp
import platform
import struct
import time

import cv2
import numpy as np
from bleak import BleakClient, BleakScanner
from bleak.backends.device import BLEDevice

from soble.sts_protocol import (
    ARM_JOINT_COUNT,
    LEG_JOINT_COUNT,
    parse_arm_joints,
    parse_leg_joints,
)

# BLE worker must use spawn on Linux — fork inherits parent threads/state and breaks asyncio/BlueZ.
MP_CTX = mp.get_context("spawn")

SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331d914"
CHAR_UUID = "beb5483e-36e1-4688-b7f2-e6a6a6d74324"

CMD_ACTUATOR_LEN = 28  # cmd + 4 wheels + arm[9] + leg[12] + enables — matches RobotCommand
CMD_RASPI_LEN = 12  # cmd + raspi union (forwarded as 7 bytes on ESP32)
CMD_ACTUATORS = ord("0")
CMD_TAG16H5 = ord("1")
CMD_TAG25H9 = ord("2")
CMD_TAG36H11 = ord("3")
ARM_ENABLE_MASK = 0x3F  # bit0=J1 .. bit5=J6 — all arm joints engaged
LEG_ENABLE_MASK = 0xFF  # bit0=J11 .. bit7=J18 — all leg joints engaged
ARM_PACKED_LEN = 9
LEG_PACKED_LEN = 12
ARM_CENTER_RAW = 2048  # mid of 0..4095 — default when leader not connected
STATE_LEN = 216  # wheelEnc[6] + arm[9] + leg[12] + quat[8] + ntags + tags[180]


async def _prepare_ble_link(
    client: BleakClient,
    char_uuid: str = CHAR_UUID,
    *,
    need_notify_len: int = STATE_LEN,
) -> None:
    """Negotiate ATT MTU for robot notifies; warn only if payload looks too small."""
    _ = client.services
    # BleakClient.exchange_mtu() is not implemented on the BlueZ backend (Linux).
    # Use backend._acquire_mtu() instead — AcquireWrite or AcquireNotify via BlueZ,
    # which sets _mtu_size so client.mtu_size is not the default 23.
    backend = client._backend
    acquire_mtu = getattr(backend, "_acquire_mtu", None)
    if acquire_mtu is not None:
        try:
            await acquire_mtu()
        except Exception as exc:
            print(
                f"MTU acquire skipped: {type(exc).__name__}: {exc!r}",
                flush=True,
            )

    char = client.services.get_characteristic(char_uuid)
    if char is None:
        raise RuntimeError(f"GATT characteristic not found: {char_uuid}")

    link_mtu = int(client.mtu_size)
    prop_payload = int(char.max_write_without_response_size)
    effective_payload = max(prop_payload, link_mtu - 3)
    if effective_payload < need_notify_len:
        print(
            f"Warning: BLE payload limit {effective_payload} < robot notify "
            f"{need_notify_len}; state may be truncated",
            flush=True,
        )
TAG_INFO_LEN = 18
MAX_TAGS = 10
TAG_BLOB_LEN = MAX_TAGS * TAG_INFO_LEN

TX_INTERVAL = 0.04  # 25 Hz — matches BLE_TX_INTERVAL on ESP32
DEFAULT_RECONNECT_DELAY_S = 0.25
STALE_NOTIFY_S = 0.3  # reconnect if no RobotState notify within this window


def _gatt_write_with_response() -> bool:
    """Use ATT write-with-response except on Linux, where it blocks BlueZ notify delivery."""
    return platform.system() == "Windows" or platform.system() == "Darwin"

# Must match raspi/detect_atags.py (origin 640,360 and * 25 packing)
TAG_ORIGIN_X = 640.0
TAG_ORIGIN_Y = 360.0
TAG_CORNER_SCALE = 25.0

AprilTagList = list[tuple[int, tuple[int, ...]]]


def _pack_u12(joints: list[int], packed_len: int) -> bytes:
    packed = bytearray(packed_len)
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


def _unpack_u12(packed: bytes, count: int) -> list[int]:
    out: list[int] = []
    byteidx = 0
    extract2 = True
    b = 0
    for _ in range(count):
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


def _pack_arm12(joints: list[int]) -> bytes:
    return _pack_u12(joints, ARM_PACKED_LEN)


def _unpack_arm12(packed: bytes) -> list[int]:
    return _unpack_u12(packed, ARM_JOINT_COUNT)


def _pack_leg12(joints: list[int]) -> bytes:
    return _pack_u12(joints, LEG_PACKED_LEN)


def _unpack_leg12(packed: bytes) -> list[int]:
    return _unpack_u12(packed, LEG_JOINT_COUNT)


def _pack_wheel_enc12(values: list[int]) -> bytes:
    """Pack four 12-bit wheel encoders into 6 bytes."""
    if len(values) != 4:
        raise ValueError(f"expected 4 wheel encoders, got {len(values)}")
    return _pack_u12([int(v) & 0x0FFF for v in values], 6)


def _unpack_enc12(packed: bytes) -> tuple[int, int, int, int]:
    """Unpack four 12-bit wheel encoders from 6 bytes (left1, right1, left2, right2)."""
    vals = _unpack_u12(packed[:6], 4)
    return vals[0], vals[1], vals[2], vals[3]


def _pack_robot_command(
    left: int,
    right: int,
    arm_packed: bytes,
    *,
    left2: int = 0,
    right2: int = 0,
    leg_packed: bytes | None = None,
    arm_disabled: bool = False,
    leg_disabled: bool = False,
) -> bytes:
    """BLE teleop matching ESP32 RobotCommand (28 bytes)."""
    if len(arm_packed) != ARM_PACKED_LEN:
        arm_packed = _pack_arm12([ARM_CENTER_RAW] * ARM_JOINT_COUNT)
    if leg_packed is None or len(leg_packed) != LEG_PACKED_LEN:
        leg_packed = _pack_leg12([ARM_CENTER_RAW] * LEG_JOINT_COUNT)
    left = max(-125, min(125, int(left)))
    right = max(-125, min(125, int(right)))
    left2 = max(-125, min(125, int(left2)))
    right2 = max(-125, min(125, int(right2)))
    arm_enabled = 0 if arm_disabled else ARM_ENABLE_MASK
    leg_enabled = 0 if leg_disabled else LEG_ENABLE_MASK
    return (
        struct.pack("<Bbbbb", CMD_ACTUATORS, left, right, left2, right2)
        + arm_packed
        + leg_packed
        + bytes([arm_enabled, leg_enabled])
    )


def _pack_raspi_ble_command(cmd: int) -> bytes:
    """BLE → ESP32 → USB → Pi (detect_atags.py). cmd is tag family '1'/'2'/'3'."""
    return struct.pack("<B", cmd & 0xFF) + b"\x00" * (CMD_RASPI_LEN - 1)


def _unpack_robot_state(data: bytes) -> dict | None:
    if len(data) < STATE_LEN:
        return None
    enc = _unpack_enc12(data[0:6])
    arm = _unpack_arm12(data[6:15])
    leg = _unpack_leg12(data[15:27])
    quat = struct.unpack("<hhhh", data[27:35])
    ntags = min(data[35] & 0x1F, 10)
    raspi_alive = (data[35] & 0x80) != 0
    tags: AprilTagList = []
    off = 36
    for _ in range(ntags):
        if off + TAG_INFO_LEN > len(data):
            break
        chunk = data[off : off + TAG_INFO_LEN]
        tag_id = struct.unpack_from("<H", chunk, 0)[0]
        corners = struct.unpack_from("<8h", chunk, 2)
        tags.append((tag_id, corners))
        off += TAG_INFO_LEN
    return {
        "enc": enc,
        "arm": arm,
        "leg": leg,
        "quat": quat,
        "ntags": ntags,
        "tags": tags,
        "raspi": raspi_alive,
    }


def _quat_to_rph_deg(qw: float, qx: float, qy: float, qz: float) -> tuple[float, float, float]:
    """Roll, pitch, heading (deg) — same convention as So101-Platform.ino Madgwick output."""
    roll = float(np.degrees(
        np.arctan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    ))
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = float(np.copysign(90.0, sinp))
    else:
        pitch = float(np.degrees(np.arcsin(sinp)))
    heading = float(np.degrees(
        np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    ))
    return roll, pitch, heading


def _quat_conjugate(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64)
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)


def _quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    q1 = np.asarray(q1, dtype=np.float64)
    q2 = np.asarray(q2, dtype=np.float64)
    return np.array(
        [
            q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3],
            q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2],
            q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1],
            q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0],
        ],
        dtype=np.float64,
    )


def _quat_normalize(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64)
    norm = float(np.linalg.norm(q))
    if norm == 0.0:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    return q / norm


def _format_state_line(state: dict) -> str:
    enc = state["enc"]
    arm = state["arm"]
    qw, qx, qy, qz = (v / 1000.0 for v in state["quat"])
    roll, pitch, heading = _quat_to_rph_deg(qw, qx, qy, qz)
    leg = state.get("leg", [])
    parts = [
        f"wheel_raw={enc}",
        f"arm_raw={' '.join(str(a) for a in arm)}",
        f"leg_raw={' '.join(str(a) for a in leg)}",
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
    arm_raw: mp.Array,
    leg_raw: mp.Array,
    quat: mp.Array,
    quat0: mp.Array,
    quat0_valid: mp.Value,
    ntags: mp.Value,
    tag_blob: mp.Array,
    raspi: mp.Value,
    lock: mp.Lock,
) -> None:
    qw, qx, qy, qz = (v / 1000.0 for v in state["quat"])
    enc_vals = state["enc"]
    n = min(state["ntags"], MAX_TAGS)
    with lock:
        for i, v in enumerate(enc_vals):
            enc[i] = int(v)
        for i, v in enumerate(state["arm"]):
            arm_raw[i] = v
        for i, v in enumerate(state["leg"]):
            leg_raw[i] = v

        current_q = np.array([float(qw), float(qx), float(qy), float(qz)], dtype=np.float64)
        if not bool(quat0_valid.value):
            quat0[0], quat0[1], quat0[2], quat0[3] = current_q
            quat0_valid.value = True
            calibrated_q = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        else:
            base_q = np.array(
                [float(quat0[0]), float(quat0[1]), float(quat0[2]), float(quat0[3])],
                dtype=np.float64,
            )
            calibrated_q = _quat_normalize(
                _quat_multiply(current_q, _quat_conjugate(base_q))
            )

        quat[0], quat[1], quat[2], quat[3] = calibrated_q
        ntags.value = n
        raspi.value = bool(state["raspi"])
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
    quat0: mp.Array,
    quat0_valid: mp.Value,
    lock: mp.Lock,
) -> None:
    with lock:
        got_state.value = False
        last_notify.value = 0.0
        ntags.value = 0
        raspi.value = False
        quat0_valid.value = False
        quat0[0], quat0[1], quat0[2], quat0[3] = 0.0, 0.0, 0.0, 0.0

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


async def _find_device(device_name: str) -> BLEDevice | None:
    dev = await BleakScanner.find_device_by_filter(
        lambda d, _: bool(d.name and device_name.lower() in d.name.lower())
    )
    if dev is None:
        return None
    print(f"Found {dev.name} ({dev.address})", flush=True)
    return dev


async def _discover_named_ble_devices_async(*, timeout: float = 5.0) -> list[str]:
    """Return sorted unique advertised names from a BLE scan.

    Prefers devices advertising ``SERVICE_UUID``. If none do, falls back to all
    named advertisers so the UI still has something to pick.
    """
    found = await BleakScanner.discover(timeout=timeout, return_adv=True)
    service = SERVICE_UUID.lower()
    with_service: set[str] = set()
    named: set[str] = set()
    for device, adv in found.values():
        name = (device.name or getattr(adv, "local_name", None) or "").strip()
        if not name:
            continue
        named.add(name)
        uuids = [str(u).lower() for u in (adv.service_uuids or [])]
        if service in uuids:
            with_service.add(name)
    return sorted(with_service if with_service else named)


def discover_named_ble_devices(*, timeout: float = 5.0) -> list[str]:
    """Blocking BLE name scan for UI pickers (does not connect).

    ``SO101Platform`` still requires an explicit ``device_name``; use a name
    returned here when constructing the platform link.

    Must be called from a thread that is not already running an asyncio loop
    (e.g. a background scanner thread in the 3D viewer).
    """
    return asyncio.run(_discover_named_ble_devices_async(timeout=timeout))


async def _ble_session(
    device: BLEDevice,
    log_state: bool,
    stop: mp.Event,
    left_cmd: mp.Value,
    right_cmd: mp.Value,
    left2_cmd: mp.Value,
    right2_cmd: mp.Value,
    arm_packed: mp.Array,
    leg_packed: mp.Array,
    pending_ble: mp.Array,
    pending_ble_valid: mp.Value,
    arm_positions_valid: mp.Value,
    leg_positions_valid: mp.Value,
    got_state: mp.Value,
    last_notify: mp.Value,
    enc: mp.Array,
    arm_raw: mp.Array,
    leg_raw: mp.Array,
    quat: mp.Array,
    quat0: mp.Array,
    quat0_valid: mp.Value,
    ntags: mp.Value,
    tag_blob: mp.Array,
    raspi: mp.Value,
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
            arm_raw,
            leg_raw,
            quat,
            quat0,
            quat0_valid,
            ntags,
            tag_blob,
            raspi,
            lock,
        )
        if log_state:
            print(_format_state_line(state), flush=True)

    def on_disconnect(_client: BleakClient) -> None:
        disconnect_event.set()

    disconnect_event = asyncio.Event()
    disconnect_event.clear()
    print(f"Connecting to {device.address}...", flush=True)
    async with BleakClient(
        device, disconnected_callback=on_disconnect, timeout=20.0
    ) as client:
        await _prepare_ble_link(client)
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
                    left2 = int(left2_cmd.value)
                    right2 = int(right2_cmd.value)
                    arm = bytes(arm_packed[:])
                    leg = bytes(leg_packed[:])
                if got and sn > 0.0 and (now - sn) > STALE_NOTIFY_S:
                    print("No notify — reconnecting...", flush=True)
                    break

                if pending_ble_valid.value or now - last_sent >= TX_INTERVAL:
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
                            arm_off = not bool(arm_positions_valid.value)
                            leg_off = not bool(leg_positions_valid.value)
                            payload = _pack_robot_command(
                                left,
                                right,
                                arm,
                                left2=left2,
                                right2=right2,
                                leg_packed=leg,
                                arm_disabled=arm_off,
                                leg_disabled=leg_off,
                            )
                    assert len(payload) in (CMD_ACTUATOR_LEN, CMD_RASPI_LEN)
                    await client.write_gatt_char(
                        CHAR_UUID, payload, response=_gatt_write_with_response()
                    )
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
    left2_cmd: mp.Value,
    right2_cmd: mp.Value,
    arm_packed: mp.Array,
    leg_packed: mp.Array,
    pending_ble: mp.Array,
    pending_ble_valid: mp.Value,
    arm_positions_valid: mp.Value,
    leg_positions_valid: mp.Value,
    got_state: mp.Value,
    last_notify: mp.Value,
    enc: mp.Array,
    arm_raw: mp.Array,
    leg_raw: mp.Array,
    quat: mp.Array,
    quat0: mp.Array,
    quat0_valid: mp.Value,
    ntags: mp.Value,
    tag_blob: mp.Array,
    raspi: mp.Value,
    lock: mp.Lock,
) -> None:
    while not stop.is_set():
        print(f"Scanning for {device_name!r}...", flush=True)
        device = await _find_device(device_name)
        if device is None:
            print(
                f"No device with name containing {device_name!r}; "
                f"retry in {reconnect_delay_s:.0f}s",
                flush=True,
            )
            await asyncio.sleep(reconnect_delay_s)
            continue

        try:
            disconnected = await _ble_session(
                device,
                log_state,
                stop,
                left_cmd,
                right_cmd,
                left2_cmd,
                right2_cmd,
                arm_packed,
                leg_packed,
                pending_ble,
                pending_ble_valid,
                arm_positions_valid,
                leg_positions_valid,
                got_state,
                last_notify,
                enc,
                arm_raw,
                leg_raw,
                quat,
                quat0,
                quat0_valid,
                ntags,
                tag_blob,
                raspi,
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

        _clear_state(
            got_state, last_notify, ntags, raspi, quat0, quat0_valid, lock
        )
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
    left2_cmd: mp.Value,
    right2_cmd: mp.Value,
    arm_packed: mp.Array,
    leg_packed: mp.Array,
    pending_ble: mp.Array,
    pending_ble_valid: mp.Value,
    arm_positions_valid: mp.Value,
    leg_positions_valid: mp.Value,
    got_state: mp.Value,
    last_notify: mp.Value,
    enc: mp.Array,
    arm_raw: mp.Array,
    leg_raw: mp.Array,
    quat: mp.Array,
    quat0: mp.Array,
    quat0_valid: mp.Value,
    ntags: mp.Value,
    tag_blob: mp.Array,
    raspi: mp.Value,
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
            left2_cmd,
            right2_cmd,
            arm_packed,
            leg_packed,
            pending_ble,
            pending_ble_valid,
            arm_positions_valid,
            leg_positions_valid,
            got_state,
            last_notify,
            enc,
            arm_raw,
            leg_raw,
            quat,
            quat0,
            quat0_valid,
            ntags,
            tag_blob,
            raspi,
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
        autostart: bool = True,
    ) -> None:
        self._device_name = device_name
        self._reconnect_delay_s = reconnect_delay_s
        self._log_state = log_state
        self._closed = False
        self._tag_origin_x = TAG_ORIGIN_X
        self._tag_origin_y = TAG_ORIGIN_Y
        self._tag_corner_scale = TAG_CORNER_SCALE

        self._lock = MP_CTX.Lock()
        self._stop = MP_CTX.Event()
        self._left_cmd = MP_CTX.Value("i", 0)
        self._right_cmd = MP_CTX.Value("i", 0)
        self._left2_cmd = MP_CTX.Value("i", 0)
        self._right2_cmd = MP_CTX.Value("i", 0)
        self._arm_packed = MP_CTX.Array(
            "B", _pack_arm12([ARM_CENTER_RAW] * ARM_JOINT_COUNT)
        )
        self._leg_packed = MP_CTX.Array(
            "B", _pack_leg12([ARM_CENTER_RAW] * LEG_JOINT_COUNT)
        )
        self._pending_ble = MP_CTX.Array("B", CMD_ACTUATOR_LEN)
        self._pending_ble_valid = MP_CTX.Value("b", False)
        self._arm_positions_valid = MP_CTX.Value("b", False)
        self._leg_positions_valid = MP_CTX.Value("b", False)
        self._got_state = MP_CTX.Value("b", False)
        self._last_notify = MP_CTX.Value("d", 0.0)
        self._enc = MP_CTX.Array("i", 4)
        self._arm_raw = MP_CTX.Array("i", ARM_JOINT_COUNT)
        self._leg_raw = MP_CTX.Array("i", LEG_JOINT_COUNT)
        self._quat = MP_CTX.Array("d", 4)
        self._quat[0] = 1.0
        self._quat0 = MP_CTX.Array("d", 4)
        self._quat0_valid = MP_CTX.Value("b", False)
        self._ntags = MP_CTX.Value("i", 0)
        self._tag_blob = MP_CTX.Array("B", TAG_BLOB_LEN)
        self._raspi = MP_CTX.Value("b", False)
        self._proc: mp.Process | None = None

        if autostart:
            self.start()

    @property
    def running(self) -> bool:
        return self._proc is not None and self._proc.is_alive()

    def last_notify_age_s(self) -> float | None:
        with self._lock:
            if not self._got_state.value or self._last_notify.value <= 0.0:
                return None
            return time.monotonic() - float(self._last_notify.value)

    def wheelEncoders(self) -> tuple[int, int, int, int]:
        """(left1, right1, left2, right2), each 0..4095."""
        with self._lock:
            return (
                int(self._enc[0]),
                int(self._enc[1]),
                int(self._enc[2]),
                int(self._enc[3]),
            )

    def getArmPositions(self) -> list[int]:
        """Six joint raw encoder values (J1..J6, 0..4095) from last robot notify."""
        with self._lock:
            if not self._got_state.value:
                return []
            return [int(self._arm_raw[i]) for i in range(ARM_JOINT_COUNT)]

    def getLegPositions(self) -> list[int]:
        """Eight leg joint raw encoder values (J11..J18, 0..4095) from last notify."""
        with self._lock:
            if not self._got_state.value:
                return []
            return [int(self._leg_raw[i]) for i in range(LEG_JOINT_COUNT)]

    # Singular aliases matching the public API name.
    def imuQuaternion(self) -> tuple[float, float, float, float]:
        """Unit quaternion (w, x, y, z)."""
        with self._lock:
            return (
                float(self._quat[0]),
                float(self._quat[1]),
                float(self._quat[2]),
                float(self._quat[3]),
            )

    def imuRotation(self) -> tuple[float, float, float]:
        """Roll, pitch, heading in degrees (from onboard Madgwick quaternion)."""
        return _quat_to_rph_deg(*self.imuQuaternion())

    def raspiAlive(self) -> bool:
        """True if ESP32 has received serial from the Pi recently."""
        with self._lock:
            return bool(self._raspi.value)


    def detectApriltags(
        self,
        estimate_tag_pose=False,
        camera_params=(940.48, 940.48, 640, 360),
        tag_size=3,
    ) -> AprilTagList:
        """Tag list from last notify (empty list if no state received yet).
        If estimate_tag_pose is True, the tag list will be estimated from the tag corners.

        Args:
            estimate_tag_pose: Whether to estimate the tag pose from the tag corners.
            camera_params: ``(fx, fy, cx, cy)`` intrinsics; default matches IMX708 (~940.48 px focal length, 640×360 principal point).
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

    def setArmPositions(
        self, joints: list[int] | tuple[int, ...] | np.ndarray
    ) -> None:
        """Drive arm joints, or pass ``[]`` to release torque."""
        parsed = parse_arm_joints(joints)
        with self._lock:
            if parsed is None:
                self._arm_positions_valid.value = False
                return
            packed = _pack_arm12(parsed)
            for i, byte in enumerate(packed):
                self._arm_packed[i] = byte
            self._arm_positions_valid.value = True

    def setLegPositions(
        self, joints: list[int] | tuple[int, ...] | np.ndarray
    ) -> None:
        """Drive leg joints (J11..J18), or pass ``[]`` to release torque."""
        parsed = parse_leg_joints(joints)
        with self._lock:
            if parsed is None:
                self._leg_positions_valid.value = False
                return
            packed = _pack_leg12(parsed)
            for i, byte in enumerate(packed):
                self._leg_packed[i] = byte
            self._leg_positions_valid.value = True

    def drive(self, left: int, right: int, left2: int = 0, right2: int = 0) -> None:
        """Tank / omni wheel speeds. ``left2``/``right2`` default 0 for two-wheel mode."""
        with self._lock:
            self._left_cmd.value = max(-125, min(125, int(left)))
            self._right_cmd.value = max(-125, min(125, int(right)))
            self._left2_cmd.value = max(-125, min(125, int(left2)))
            self._right2_cmd.value = max(-125, min(125, int(right2)))

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

    def setTagFamily(self, family: str) -> None:
        """Forward tag family to the Pi ('tag16h5', 'tag25h9', 'tag36h11')."""
        fam = family.lower().replace("-", "").replace("_", "")
        cmd_by_family = {
            "tag16h5": CMD_TAG16H5,
            "tag25h9": CMD_TAG25H9,
            "tag36h11": CMD_TAG36H11,
        }
        if fam not in cmd_by_family:
            raise ValueError(f"Unknown tag family {family!r}")
        self._queue_ble_command(_pack_raspi_ble_command(cmd_by_family[fam]))

    def start(self) -> None:
        """Start (or restart) the BLE worker. Called automatically from ``__init__``."""
        if self._proc is not None and self._proc.is_alive():
            return
        self._closed = False
        self._stop.clear()
        self._proc = MP_CTX.Process(
            target=_ble_worker,
            args=(
                self._device_name,
                self._reconnect_delay_s,
                self._log_state,
                self._stop,
                self._left_cmd,
                self._right_cmd,
                self._left2_cmd,
                self._right2_cmd,
                self._arm_packed,
                self._leg_packed,
                self._pending_ble,
                self._pending_ble_valid,
                self._arm_positions_valid,
                self._leg_positions_valid,
                self._got_state,
                self._last_notify,
                self._enc,
                self._arm_raw,
                self._leg_raw,
                self._quat,
                self._quat0,
                self._quat0_valid,
                self._ntags,
                self._tag_blob,
                self._raspi,
                self._lock,
            ),
            daemon=True,
        )
        self._proc.start()

    def stop(self) -> None:
        """Stop BLE worker. Called automatically from ``__del__``."""
        if self._closed:
            return
        self._closed = True
        self._stop.set()
        if self._proc is not None:
            self._proc.join(timeout=3.0)
            self._proc = None
        _clear_state(
            self._got_state,
            self._last_notify,
            self._ntags,
            self._raspi,
            self._quat0,
            self._quat0_valid,
            self._lock,
        )

    def __del__(self) -> None:
        try:
            self.stop()
        except Exception:
            pass


def _main_test() -> int:
    import argparse

    parser = argparse.ArgumentParser(
        description="SO101Platform BLE test (multiprocess, cmd='0' @ 25 Hz)"
    )
    parser.add_argument("-n", "--name", default="Capybara")
    parser.add_argument("-d", "--duration", type=float, default=15.0)
    parser.add_argument("--quiet", action="store_true", help="Suppress state lines")
    args = parser.parse_args()

    print(
        f"Starting SO101Platform — drive(0,0) @ {1.0 / TX_INTERVAL:.0f} Hz "
        f"for {args.duration:.0f}s (write response={_gatt_write_with_response()})",
        flush=True,
    )
    plat = SO101Platform(args.name, log_state=False)
    plat.drive(0, 0)

    t0 = time.monotonic()
    end = t0 + args.duration
    notify_count = 0
    last_sn = 0.0
    stale_reported = False

    while time.monotonic() < end:
        sn = float(plat._last_notify.value)
        got = bool(plat._got_state.value)
        if got and sn > last_sn:
            notify_count += 1
            last_sn = sn
            if not args.quiet and (
                notify_count <= 3 or notify_count % 50 == 0
            ):
                print(f"notify #{notify_count}", flush=True)

        if got and sn > 0.0:
            age = time.monotonic() - sn
            if age > STALE_NOTIFY_S and not stale_reported:
                print(f"WARN: no notify for {age:.2f}s", flush=True)
                stale_reported = True
        if not plat.running:
            print("WARN: BLE worker died", flush=True)
            break
        time.sleep(0.01)

    span = max(last_sn - t0, 0.0) if notify_count > 0 else 0.0
    notify_hz = notify_count / span if span > 0 else 0.0
    write_hz = 1.0 / TX_INTERVAL
    write_count = int(args.duration * write_hz)
    print(
        f"notifies={notify_count} ({notify_hz:.1f} Hz) "
        f"writes≈{write_count} ({write_hz:.1f} Hz)",
        flush=True,
    )

    ok = notify_count >= int(args.duration * 8)
    plat._stop.set()
    if plat._proc is not None:
        plat._proc.join(timeout=2.0)

    if ok:
        print("PASS", flush=True)
        return 0
    print("FAIL", flush=True)
    return 1


if __name__ == "__main__":
    raise SystemExit(_main_test())
