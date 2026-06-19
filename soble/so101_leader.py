"""SO-101 leader arm: Feetech SYNC_READ in a child process."""

from __future__ import annotations

import json
import multiprocessing as mp
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import serial

from soble.arm_joints import ARM_JOINT_COUNT, parse_arm_joints

MOTOR_IDS = [1, 2, 3, 4, 5, 6]
JOINT_KEYS = ["J1", "J2", "J3", "J4", "J5", "J6"]

INST_SYNC_READ = 0x82
INST_SYNC_WRITE = 0x83
REG_PRESENT_POSITION = 0x38
REG_TORQUE_ENABLE = 40
DATA_LEN = 2
RESP_FRAME_LEN = 8
EXPECTED_RX = ARM_JOINT_COUNT * RESP_FRAME_LEN

WRAP_OFFSET = 4096
BELOW_MIN_MARGIN = 100
POLL_INTERVAL = 0.01  # 100 Hz

DEFAULT_BAUD = 1000000


@dataclass
class JointLimits:
    min_val: int
    max_val: int

    @property
    def range(self) -> int:
        return self.max_val - self.min_val


def _limits_to_tuples(limits: list[JointLimits]) -> list[tuple[int, int]]:
    return [(lim.min_val, lim.max_val) for lim in limits]


def _normalize_limit_pair(min_val: int, max_val: int) -> JointLimits:
    if max_val < min_val:
        max_val += WRAP_OFFSET
    return JointLimits(min_val, max_val)


def _limits_from_j1_config(cfg: dict) -> tuple[list[JointLimits], list[JointLimits]]:
    leader: list[JointLimits] = []
    follower: list[JointLimits] = []
    for key in JOINT_KEYS:
        l_min, l_max = cfg["leader"][key]
        f_min, f_max = cfg["follower"][key]
        leader.append(_normalize_limit_pair(int(l_min), int(l_max)))
        follower.append(_normalize_limit_pair(int(f_min), int(f_max)))
    return leader, follower


def _limits_from_so101_arm(arm_cfg: dict) -> list[JointLimits]:
    joints = arm_cfg.get("joints")
    if not isinstance(joints, dict):
        raise ValueError('so101 arm config must have a "joints" object')

    by_id: dict[int, JointLimits] = {}
    for name, joint in joints.items():
        if not isinstance(joint, dict):
            raise ValueError(f'joint {name!r} must be an object')
        if "id" not in joint or "min_limit" not in joint or "max_limit" not in joint:
            raise ValueError(
                f'joint {name!r} must include "id", "min_limit", and "max_limit"'
            )
        motor_id = int(joint["id"])
        if motor_id not in MOTOR_IDS:
            raise ValueError(f'joint {name!r} has invalid id {motor_id}')
        if motor_id in by_id:
            raise ValueError(f'duplicate motor id {motor_id} in joints')
        by_id[motor_id] = _normalize_limit_pair(
            int(joint["min_limit"]),
            int(joint["max_limit"]),
        )

    if len(by_id) != ARM_JOINT_COUNT:
        raise ValueError(
            f"expected {ARM_JOINT_COUNT} joints with ids 1..{ARM_JOINT_COUNT}, "
            f"got {len(by_id)}"
        )
    return [by_id[motor_id] for motor_id in MOTOR_IDS]


def _limits_from_so101_config(cfg: dict) -> tuple[list[JointLimits], list[JointLimits]]:
    return (
        _limits_from_so101_arm(cfg["so101_leader"]),
        _limits_from_so101_arm(cfg["so101_follower"]),
    )


def _limits_from_tuples(tuples: list[tuple[int, int]]) -> list[JointLimits]:
    return [JointLimits(a, b) for a, b in tuples]


def _build_sync_write_byte(ids: list[int], reg: int, values: list[int]) -> bytes:
    packet = [
        0xFF,
        0xFF,
        0xFE,
        4 + 2 * len(ids),
        INST_SYNC_WRITE,
        reg,
        1,
    ]
    for mid, val in zip(ids, values):
        packet.extend([mid, val & 0xFF])
    packet.append((~sum(packet[2:])) & 0xFF)
    return bytes(packet)


def _set_arm_torque(ser: serial.Serial, enabled: bool) -> None:
    """STS Torque_Enable: 0 = backdrivable, 1 = holding."""
    values = [1 if enabled else 0] * len(MOTOR_IDS)
    pkt = _build_sync_write_byte(MOTOR_IDS, REG_TORQUE_ENABLE, values)
    ser.reset_input_buffer()
    ser.write(pkt)
    ser.flush()


def _so101_leader_worker(
    port: str,
    baud: int,
    leader_lim_tuples: list[tuple[int, int]],
    follower_lim_tuples: list[tuple[int, int]],
    follower_out: mp.Array,
    valid: mp.Value,
    arm_disabled: mp.Value,
    stop: mp.Event,
    poll_interval_s: float,
) -> None:
    reader = _SO101LeaderReader(
        _limits_from_tuples(leader_lim_tuples),
        _limits_from_tuples(follower_lim_tuples),
    )

    try:
        ser = serial.Serial(port, baud, timeout=0.08)
    except serial.SerialException as e:
        print(f"Leader serial error: {e}", file=sys.stderr)
        return

    ser.reset_input_buffer()
    print(f"Leader arm {port} @ {baud}", flush=True)

    torque_engaged = not bool(arm_disabled.value)
    _set_arm_torque(ser, torque_engaged)
    try:
        while not stop.is_set():
            want_engaged = not bool(arm_disabled.value)
            if want_engaged != torque_engaged:
                _set_arm_torque(ser, want_engaged)
                torque_engaged = want_engaged

            leader_raw = reader.sync_read(ser)
            follower_raws = reader.compute_follower_raws(leader_raw)
            if follower_raws is not None:
                with valid.get_lock():
                    for i, raw in enumerate(follower_raws):
                        follower_out[i] = raw
                    valid.value = True
            time.sleep(poll_interval_s)
    except Exception as e:
        print(f"Leader process error: {e}", file=sys.stderr)
    finally:
        if torque_engaged:
            _set_arm_torque(ser, False)
        ser.close()


class _SO101LeaderReader:
    """Serial SYNC_READ + angular mapping (runs only in the child process)."""

    def __init__(
        self,
        leader_limits: list[JointLimits],
        follower_limits: list[JointLimits],
    ) -> None:
        self._leader_limits = leader_limits
        self._follower_limits = follower_limits

    @staticmethod
    def _build_sync_read_packet() -> bytes:
        packet = [
            0xFF,
            0xFF,
            0xFE,
            len(MOTOR_IDS) + 4,
            INST_SYNC_READ,
            REG_PRESENT_POSITION,
            DATA_LEN,
            *MOTOR_IDS,
        ]
        packet.append(~sum(packet[2:]) & 0xFF)
        return bytes(packet)

    @staticmethod
    def _parse_status_frame(frame: bytes) -> int | None:
        if len(frame) < RESP_FRAME_LEN or frame[0] != 0xFF or frame[1] != 0xFF:
            return None
        if (~sum(frame[2:-1]) & 0xFF) != frame[-1]:
            return None
        return (frame[6] << 8) | frame[5]

    def _find_position_for_id(self, rx_buf: bytes, motor_id: int) -> int | None:
        for i in range(len(rx_buf) - RESP_FRAME_LEN + 1):
            if rx_buf[i] == 0xFF and rx_buf[i + 1] == 0xFF and rx_buf[i + 2] == motor_id:
                return self._parse_status_frame(rx_buf[i : i + RESP_FRAME_LEN])
        return None

    def sync_read(self, ser: serial.Serial) -> dict[int, int]:
        ser.reset_input_buffer()
        ser.write(self._build_sync_read_packet())
        ser.flush()
        rx = ser.read(EXPECTED_RX)
        positions: dict[int, int] = {}
        for m_id in MOTOR_IDS:
            pos = self._find_position_for_id(rx, m_id)
            if pos is not None:
                positions[m_id] = pos
        return positions

    @staticmethod
    def _normalize_leader_angle(raw: int, limits: JointLimits) -> int:
        if raw < limits.min_val - BELOW_MIN_MARGIN:
            raw += WRAP_OFFSET
        return raw

    @staticmethod
    def _map_to_follower(
        leader_angle: int,
        leader_lim: JointLimits,
        follower_lim: JointLimits,
    ) -> int:
        l_range = leader_lim.range
        if l_range <= 0:
            return follower_lim.min_val
        ratio = follower_lim.range / l_range
        mapped = (leader_angle - leader_lim.min_val) * ratio + follower_lim.min_val
        return int(round(mapped)) & 0x0FFF

    def compute_follower_raws(self, leader_raw: dict[int, int]) -> list[int] | None:
        raws: list[int] = []
        for i, m_id in enumerate(MOTOR_IDS):
            raw = leader_raw.get(m_id)
            if raw is None:
                return None
            norm = self._normalize_leader_angle(raw, self._leader_limits[i])
            raws.append(
                self._map_to_follower(norm, self._leader_limits[i], self._follower_limits[i])
            )
        return raws


class SO101Leader:
    """Leader reader in a background process; parent polls via getArmPositions()."""

    def __init__(
        self,
        port: str,
        leader_limits: list[JointLimits] | None = None,
        follower_limits: list[JointLimits] | None = None,
        *,
        config_path: str | Path | None = None,
        baud: int = DEFAULT_BAUD,
        poll_interval_s: float = POLL_INTERVAL,
    ) -> None:
        self._port = port
        self._baud = baud
        self._poll_interval_s = poll_interval_s
        self._closed = False

        if config_path is not None:
            if leader_limits is not None or follower_limits is not None:
                raise ValueError("pass config_path or explicit limits, not both")
            self._apply_limits_from_path(config_path)
        elif leader_limits is not None and follower_limits is not None:
            self._leader_limits = leader_limits
            self._follower_limits = follower_limits
        elif leader_limits is not None or follower_limits is not None:
            raise TypeError("leader_limits and follower_limits must both be provided")
        else:
            self._leader_limits = []
            self._follower_limits = []

        self._lock = mp.Lock()
        self._follower_raws = mp.Array("i", ARM_JOINT_COUNT)
        self._valid = mp.Value("b", False)
        self._arm_disabled = mp.Value("b", True)  # backdrivable by default
        self._stop = mp.Event()
        self._proc: mp.Process | None = None

        if len(self._leader_limits) == ARM_JOINT_COUNT:
            self.start()

    @staticmethod
    def limits_from_config(cfg: dict) -> tuple[list[JointLimits], list[JointLimits]]:
        """Build leader/follower limit lists from a config dict (see README)."""
        if "so101_leader" in cfg and "so101_follower" in cfg:
            return _limits_from_so101_config(cfg)
        if "leader" in cfg and "follower" in cfg:
            return _limits_from_j1_config(cfg)
        raise ValueError(
            'config must include "leader"/"follower" (J1..J6) or '
            '"so101_leader"/"so101_follower" (joints with min_limit/max_limit)'
        )

    def _apply_limits_from_path(self, path: str | Path) -> None:
        with Path(path).open(encoding="utf-8") as f:
            self._leader_limits, self._follower_limits = self.limits_from_config(
                json.load(f)
            )

    def load_config(self, path: str | Path) -> None:
        """Load leader/follower joint limits from a JSON file (see README)."""
        running = self._proc is not None and self._proc.is_alive()
        if running:
            self.stop()
        self._apply_limits_from_path(path)
        self.start()

    def start(self) -> None:
        """Start (or restart) the serial reader. Called automatically from ``__init__``."""
        if self._proc is not None and self._proc.is_alive():
            return
        self._closed = False
        self._stop.clear()
        self._proc = mp.Process(
            target=_so101_leader_worker,
            args=(
                self._port,
                self._baud,
                _limits_to_tuples(self._leader_limits),
                _limits_to_tuples(self._follower_limits),
                self._follower_raws,
                self._valid,
                self._arm_disabled,
                self._stop,
                self._poll_interval_s,
            ),
            daemon=True,
        )
        self._proc.start()

    def setArmPositions(
        self, joints: list[int] | tuple[int, ...] | np.ndarray
    ) -> None:
        """Engage leader torque, or pass ``[]`` to release (backdrivable)."""
        parsed = parse_arm_joints(joints)
        with self._lock:
            self._arm_disabled.value = parsed is None

    def getArmPositions(self) -> list[int]:
        """Latest mapped follower joint raws (6 ints, J1..J6), or [] if none yet."""
        with self._lock:
            if not self._valid.value:
                return []
            return list(self._follower_raws[:])

    def stop(self) -> None:
        """Stop the serial reader. Called automatically from ``__del__``."""
        if self._closed:
            return
        self._closed = True
        self._stop.set()
        if self._proc is not None:
            self._proc.join(timeout=2.0)
            self._proc = None

    def __del__(self) -> None:
        try:
            self.stop()
        except Exception:
            pass
