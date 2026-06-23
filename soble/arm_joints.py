"""Arm joint list validation shared by leader and platform."""

from __future__ import annotations

import numpy as np

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