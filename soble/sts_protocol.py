"""FeeTech STS serial protocol and SO-101 arm helpers."""

from __future__ import annotations

import sys
import time
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    import serial

ARM_JOINT_COUNT = 6

ARM_JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)

DEFAULT_RAW_LIMITS = np.tile([0, 4095], (ARM_JOINT_COUNT, 1)).astype(np.int32)

# FeeTech STS/SMS serial protocol (see examples/config-sts.py)
STS_BAUDRATE = 1_000_000
STS_BROADCAST_ID = 0xFE  # 254 — single motor on bus
STS_REG_ID = 5
STS_REG_LOCK = 55
STS_REG_TORQUE_ENABLE = 40
STS_REG_GOAL_POSITION = 0x2A
STS_REG_PRESENT_POSITION = 0x38
STS_INST_SYNC_READ = 0x82
STS_INST_SYNC_WRITE = 0x83
STS_STATUS_FRAME_LEN = 8
STS_PRESENT_POSITION_DATA_LEN = 2
STS_CENTER_RAW = 2048
STS_MIN_MOTOR_ID = 1
STS_MAX_MOTOR_ID = 253
STS_WRITE_DATA = 3


def parse_arm_joints(
    joints: list[int] | tuple[int, ...] | np.ndarray,
) -> list[int] | None:
    """Return six joint values, or None if joints is empty (disable torque)."""
    if isinstance(joints, np.ndarray):
        flat = joints.reshape(-1)
        if flat.size == 0:
            return None
        if flat.size != ARM_JOINT_COUNT:
            raise ValueError(f"expected {ARM_JOINT_COUNT} joint values, got {flat.size}")
        raw = flat.tolist()
    elif isinstance(joints, tuple):
        raw = list(joints)
    elif isinstance(joints, list):
        raw = joints
    else:
        raise TypeError(
            "joints must be a list, tuple, or numpy.ndarray of 6 numbers (or [] to disable)"
        )

    if len(raw) == 0:
        return None
    if len(raw) != ARM_JOINT_COUNT:
        raise ValueError(f"expected {ARM_JOINT_COUNT} joint values, got {len(raw)}")

    out: list[int] = []
    for i, v in enumerate(raw):
        try:
            iv = int(v)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"joint[{i}] is not a number: {v!r}") from exc
        if not 0 <= iv <= 4095:
            raise ValueError(f"joint[{i}]={iv} out of range 0..4095")
        out.append(iv)
    return out


def _sts_checksum(packet: list[int]) -> int:
    """FeeTech checksum: ~ (ID + Length + Cmd + Params) & 0xFF."""
    return (~sum(packet[2:])) & 0xFF


def _sts_sync_write(ser: serial.Serial, packet: bytes) -> None:
    ser.reset_input_buffer()
    ser.write(packet)
    ser.flush()


def _build_sync_write_byte(ids: list[int], reg: int, values: list[int]) -> bytes:
    packet = [
        0xFF,
        0xFF,
        0xFE,
        4 + 2 * len(ids),
        STS_INST_SYNC_WRITE,
        reg,
        1,
    ]
    for motor_id, value in zip(ids, values):
        packet.extend([motor_id, value & 0xFF])
    packet.append(_sts_checksum(packet))
    return bytes(packet)


def _build_sync_write_position(ids: list[int], positions: list[int]) -> bytes:
    packet = [
        0xFF,
        0xFF,
        0xFE,
        4 + 5 * len(ids),
        STS_INST_SYNC_WRITE,
        STS_REG_GOAL_POSITION,
        4,
    ]
    for motor_id, pos in zip(ids, positions):
        raw = int(pos) & 0xFFF
        packet.extend([motor_id, raw & 0xFF, (raw >> 8) & 0xFF, 0, 0])
    packet.append(_sts_checksum(packet))
    return bytes(packet)


def _build_sync_read_packet(
    ids: list[int],
    *,
    reg: int = STS_REG_PRESENT_POSITION,
    data_len: int = STS_PRESENT_POSITION_DATA_LEN,
) -> bytes:
    packet = [
        0xFF,
        0xFF,
        0xFE,
        len(ids) + 4,
        STS_INST_SYNC_READ,
        reg,
        data_len,
        *ids,
    ]
    packet.append(_sts_checksum(packet))
    return bytes(packet)


def _parse_sts_status_frame(frame: bytes) -> int | None:
    if len(frame) < STS_STATUS_FRAME_LEN or frame[0] != 0xFF or frame[1] != 0xFF:
        return None
    # Validate checksum using the full frame minus trailing checksum byte.
    if _sts_checksum(list(frame[:-1])) != frame[-1]:
        return None
    return (frame[6] << 8) | frame[5]


def _find_sts_position(rx_buf: bytes, motor_id: int) -> int | None:
    for i in range(len(rx_buf) - STS_STATUS_FRAME_LEN + 1):
        if (
            rx_buf[i] == 0xFF
            and rx_buf[i + 1] == 0xFF
            and rx_buf[i + 2] == motor_id
        ):
            return _parse_sts_status_frame(rx_buf[i : i + STS_STATUS_FRAME_LEN])
    return None


def sts_sync_read_positions(
    ser: serial.Serial, ids: list[int]
) -> dict[int, int | None]:
    """SYNC_READ present position for each motor ID (``None`` if frame not found)."""
    expected_rx = len(ids) * STS_STATUS_FRAME_LEN
    ser.reset_input_buffer()
    ser.write(_build_sync_read_packet(ids))
    ser.flush()
    rx = ser.read(expected_rx)
    return {motor_id: _find_sts_position(rx, motor_id) for motor_id in ids}


def sts_sync_enable_torque(
    ser: serial.Serial, ids: list[int], enabled: bool
) -> None:
    """SYNC_WRITE Torque_Enable (1 = holding, 0 = backdrivable) for each motor ID."""
    value = 1 if enabled else 0
    _sts_sync_write(
        ser,
        _build_sync_write_byte(ids, STS_REG_TORQUE_ENABLE, [value] * len(ids)),
    )


def _build_sync_write_u16(ids: list[int], reg: int, values: list[int]) -> bytes:
    packet = [
        0xFF,
        0xFF,
        0xFE,
        4 + 3 * len(ids),
        STS_INST_SYNC_WRITE,
        reg,
        2,
    ]
    for motor_id, value in zip(ids, values):
        raw = int(value) & 0xFFFF
        packet.extend([motor_id, raw & 0xFF, (raw >> 8) & 0xFF])
    packet.append(_sts_checksum(packet))
    return bytes(packet)


sts_sync_write = _sts_sync_write
build_sts_sync_write_byte = _build_sync_write_byte
build_sts_sync_write_u16 = _build_sync_write_u16


def _sts_write_reg_byte(
    ser: serial.Serial, motor_id: int, register: int, value: int
) -> None:
    """Write a 1-byte value to an STS register."""
    length = 4  # instruction + register + 1 data byte
    packet = [0xFF, 0xFF, motor_id, length, STS_WRITE_DATA, register, value]
    packet.append(_sts_checksum(packet))
    ser.write(bytes(packet))
    time.sleep(0.05)


def write_sts_servo_id(
    comport: str,
    target_id: int,
    *,
    current_id: int = STS_BROADCAST_ID,
    baudrate: int = STS_BAUDRATE,
) -> tuple[bool, str]:
    """Flash a new STS motor ID over serial (one motor on the bus at a time).

    Uses broadcast ID 254 by default to address the only connected servo.
    Returns ``(success, message)`` for UI feedback.
    """
    import serial

    if not STS_MIN_MOTOR_ID <= target_id <= STS_MAX_MOTOR_ID:
        return (
            False,
            f"Target ID must be {STS_MIN_MOTOR_ID}-{STS_MAX_MOTOR_ID}, got {target_id}.",
        )

    try:
        with serial.Serial(comport, baudrate, timeout=1) as ser:
            _sts_write_reg_byte(ser, current_id, STS_REG_LOCK, 0)
            _sts_write_reg_byte(ser, current_id, STS_REG_ID, target_id)
            _sts_write_reg_byte(ser, target_id, STS_REG_LOCK, 1)
    except serial.SerialException as exc:
        return False, f"Could not open {comport}: {exc}"
    except Exception as exc:
        return False, f"Flash failed: {exc}"

    return (
        True,
        f"Servo ID set to {target_id}. Power-cycle the motor for the change to take effect.",
    )


def center_sts_servo(
    comport: str,
    motor_id: int,
    *,
    position: int = STS_CENTER_RAW,
    baudrate: int = STS_BAUDRATE,
    settle_s: float = 1.5,
) -> tuple[bool, str]:
    """Enable torque, move one STS servo to ``position``, then release torque.

    Use broadcast ID 254 when only one motor is on the bus.
    Returns ``(success, message)`` for UI feedback.
    """
    import serial

    if motor_id != STS_BROADCAST_ID and not (
        STS_MIN_MOTOR_ID <= motor_id <= STS_MAX_MOTOR_ID
    ):
        return (
            False,
            f"Motor ID must be {STS_MIN_MOTOR_ID}-{STS_MAX_MOTOR_ID} or broadcast, "
            f"got {motor_id}.",
        )
    if not 0 <= position <= 4095:
        return False, f"Position must be 0..4095, got {position}."

    ids = [motor_id]
    try:
        with serial.Serial(comport, baudrate, timeout=1) as ser:
            sts_sync_enable_torque(ser, ids, True)
            time.sleep(0.02)
            _sts_sync_write(ser, _build_sync_write_position(ids, [position]))
            time.sleep(settle_s)
            sts_sync_enable_torque(ser, ids, False)
    except serial.SerialException as exc:
        return False, f"Could not open {comport}: {exc}"
    except Exception as exc:
        return False, f"Center failed: {exc}"

    return True, f"Servo driven to {position}, torque released."


def _is_sts_serial_device(device: str) -> bool:
    """Return True if ``device`` looks like an OS serial port for STS adapters."""
    if sys.platform == "win32":
        return device.upper().startswith("COM")
    if sys.platform == "darwin":
        return device.startswith("/dev/tty.")
    return (
        device.startswith("/dev/ttyACM")
        or device.startswith("/dev/ttyUSB")
        or device.startswith("/dev/tty.usbmodem")
    )


def scan_sts_serial_ports() -> list[str]:
    """Return sorted serial port names that may host an STS bus on this OS."""
    import serial.tools.list_ports

    found: list[str] = []
    for info in serial.tools.list_ports.comports():
        device = info.device
        if _is_sts_serial_device(device):
            found.append(device)
    return sorted(set(found))


def vector_map(
    vector: np.ndarray,
    range1: np.ndarray,
    range2: np.ndarray,
    clamp: bool = True,
) -> np.ndarray:
    """Linearly map ``vector`` from ``range1`` endpoints to ``range2`` (both N×2)."""
    assert range1.shape[1] == range2.shape[1] == 2
    assert vector.shape[0] == range1.shape[0] == range2.shape[0]
    denom = range1[:, 1] - range1[:, 0]
    mapped = range2[:, 0] + (vector - range1[:, 0]) * (range2[:, 1] - range2[:, 0]) / denom
    if clamp:
        lo = np.minimum(range2[:, 0], range2[:, 1])
        hi = np.maximum(range2[:, 0], range2[:, 1])
        mapped = np.clip(mapped, lo, hi)
    return mapped