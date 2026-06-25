"""Arm joint list validation shared by leader and platform."""

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