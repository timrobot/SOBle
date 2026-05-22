"""Host BLE API and leader-arm teleop for the SO101 platform."""

from soble.so101_leader import (
    ARM_MOTOR_COUNT,
    DEFAULT_BAUD,
    JOINT_KEYS,
    JointLimits,
    SO101Leader,
)
from soble.so101_platform import SO101Platform

__all__ = [
    "ARM_MOTOR_COUNT",
    "DEFAULT_BAUD",
    "JOINT_KEYS",
    "JointLimits",
    "SO101Leader",
    "SO101Platform",
]

__version__ = "0.1.0"
