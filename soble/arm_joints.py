"""Arm joint list validation shared by leader and platform."""

from __future__ import annotations

import numpy as np

ARM_JOINT_COUNT = 6


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
