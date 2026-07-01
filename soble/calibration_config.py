"""SO-101 calibration config: typed joint and teleop settings."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Literal

import numpy as np

from soble.sts_protocol import ARM_JOINT_COUNT, ARM_JOINT_NAMES

MotorId = Literal[1, 2, 3, 4, 5, 6]
WRAP_OFFSET = 4096


@dataclass
class JointConfig:
    id: MotorId
    type: str = "ST3215"
    zero_offset: int = 0
    min_limit: int = 0
    max_limit: int = 4095
    direction: int = 1

    @classmethod
    def from_limits(cls, lo: int, hi: int, motor_id: int) -> JointConfig:
        lo_i = int(lo)
        hi_i = int(hi)
        return cls(
            id=motor_id,  # type: ignore[arg-type]
            zero_offset=(lo_i + hi_i) // 2,
            min_limit=lo_i,
            max_limit=hi_i,
            direction=1,
        )

    @classmethod
    def _from_json(cls, data: dict) -> JointConfig:
        return cls(
            id=int(data["id"]),  # type: ignore[arg-type]
            type=str(data.get("type", "ST3215")),
            zero_offset=int(data["zero_offset"]),
            min_limit=int(data["min_limit"]),
            max_limit=int(data["max_limit"]),
            direction=int(data.get("direction", 1)),
        )

    def apply_limits(self, lo: int, hi: int) -> None:
        lo_i = int(lo)
        hi_i = int(hi)
        self.min_limit = lo_i
        self.max_limit = hi_i
        self.zero_offset = (lo_i + hi_i) // 2
        self.direction = 1

    def _to_json(self) -> dict:
        return {
            "id": self.id,
            "type": self.type,
            "zero_offset": self.zero_offset,
            "min_limit": self.min_limit,
            "max_limit": self.max_limit,
            "direction": self.direction,
        }


@dataclass
class ScalingFactors:
    shoulder_pan: float = 1.0
    shoulder_lift: float = 1.0
    elbow_flex: float = 1.0
    wrist_flex: float = 1.0
    wrist_roll: float = 1.0
    gripper: float = 1.0

    @classmethod
    def defaults(cls) -> ScalingFactors:
        return cls()

    @classmethod
    def _from_json(cls, data: dict) -> ScalingFactors:
        return cls(
            **{name: float(data.get(name, 1.0)) for name in ARM_JOINT_NAMES}
        )

    def _to_json(self) -> dict:
        return {name: float(getattr(self, name)) for name in ARM_JOINT_NAMES}


@dataclass
class ArmConfig:
    shoulder_pan: JointConfig
    shoulder_lift: JointConfig
    elbow_flex: JointConfig
    wrist_flex: JointConfig
    wrist_roll: JointConfig
    gripper: JointConfig

    @classmethod
    def _from_json(cls, joints: dict) -> ArmConfig:
        return cls(
            **{
                name: JointConfig._from_json(joints[name])
                for name in ARM_JOINT_NAMES
            }
        )

    @classmethod
    def from_limits_array(cls, limits: np.ndarray) -> ArmConfig:
        return cls(
            **{
                ARM_JOINT_NAMES[index]: JointConfig.from_limits(
                    limits[index, 0],
                    limits[index, 1],
                    index + 1,
                )
                for index in range(ARM_JOINT_COUNT)
            }
        )

    def apply_limits_array(self, limits: np.ndarray) -> None:
        for index, name in enumerate(ARM_JOINT_NAMES):
            getattr(self, name).apply_limits(limits[index, 0], limits[index, 1])

    def to_limits_array(self) -> np.ndarray:
        rows: list[list[int]] = []
        for name in ARM_JOINT_NAMES:
            joint = getattr(self, name)
            lo = int(joint.min_limit)
            hi = int(joint.max_limit)
            if hi < lo:
                hi += WRAP_OFFSET
            rows.append([lo, hi])
        return np.asarray(rows, dtype=np.int32)

    def _to_json(self) -> dict:
        return {name: getattr(self, name)._to_json() for name in ARM_JOINT_NAMES}


@dataclass
class CalibrationConfig:
    type: str = "direct_proportional"
    scaling_factors: ScalingFactors = field(default_factory=ScalingFactors.defaults)
    leader: ArmConfig | None = None
    follower: ArmConfig | None = None

    @classmethod
    def resolve(cls, config: CalibrationConfig | str | Path) -> CalibrationConfig:
        if isinstance(config, CalibrationConfig):
            return config
        return cls.from_path(config)

    @classmethod
    def _from_json(cls, cfg: dict) -> CalibrationConfig:
        if "so101_leader" not in cfg or "so101_follower" not in cfg:
            raise ValueError(
                'config must include "so101_leader" and "so101_follower" '
                'with "joints" (min_limit/max_limit per joint)'
            )
        teleop = cfg.get("teleop_mapping", {})
        return cls(
            type=str(teleop.get("type", "direct_proportional")),
            scaling_factors=ScalingFactors._from_json(
                teleop.get("scaling_factors", {})
            ),
            leader=ArmConfig._from_json(cfg["so101_leader"]["joints"]),
            follower=ArmConfig._from_json(cfg["so101_follower"]["joints"]),
        )

    @classmethod
    def from_limits(
        cls,
        *,
        leader_limits: np.ndarray | None = None,
        follower_limits: np.ndarray | None = None,
    ) -> CalibrationConfig:
        return cls(
            leader=(
                ArmConfig.from_limits_array(leader_limits)
                if leader_limits is not None
                else None
            ),
            follower=(
                ArmConfig.from_limits_array(follower_limits)
                if follower_limits is not None
                else None
            ),
        )

    @classmethod
    def from_path(cls, path: str | Path) -> CalibrationConfig:
        with Path(path).open(encoding="utf-8") as f:
            return cls._from_json(json.load(f))

    def limits_arrays(self) -> tuple[np.ndarray, np.ndarray]:
        if self.leader is None or self.follower is None:
            raise ValueError("leader and follower must both be set")
        return self.leader.to_limits_array(), self.follower.to_limits_array()

    def patch_arm_limits(self, arm: str, raw_limits: np.ndarray) -> None:
        if arm not in ("leader", "follower"):
            raise ValueError(f"arm must be 'leader' or 'follower', got {arm!r}")
        arm_cfg = self.leader if arm == "leader" else self.follower
        if arm_cfg is None:
            raise ValueError(f"{arm} is not set on this config")
        arm_cfg.apply_limits_array(raw_limits)

    def _to_json(self) -> dict:
        cfg: dict = {
            "teleop_mapping": {
                "type": self.type,
                "scaling_factors": self.scaling_factors._to_json(),
            }
        }
        if self.leader is not None:
            cfg["so101_leader"] = {"joints": self.leader._to_json()}
        if self.follower is not None:
            cfg["so101_follower"] = {"joints": self.follower._to_json()}
        return cfg

    def save(self, path: str | Path) -> None:
        with Path(path).open("w", encoding="utf-8") as f:
            json.dump(self._to_json(), f, indent=2)
            f.write("\n")
