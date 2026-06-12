"""SO-101 leader arm: Feetech SYNC_READ in a child process."""

from __future__ import annotations

import json
import multiprocessing as mp
import sys
import time
from dataclasses import dataclass
from pathlib import Path
import serial

MOTOR_IDS = [1, 2, 3, 4, 5, 6]
JOINT_KEYS = ["J1", "J2", "J3", "J4", "J5", "J6"]
ARM_MOTOR_COUNT = 6

INST_SYNC_READ = 0x82
INST_SYNC_WRITE = 0x83
REG_PRESENT_POSITION = 0x38
REG_TORQUE_ENABLE = 40
DATA_LEN = 2
RESP_FRAME_LEN = 8
EXPECTED_RX = ARM_MOTOR_COUNT * RESP_FRAME_LEN

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
    leader_out: mp.Array,
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
                    for i, m_id in enumerate(MOTOR_IDS):
                        leader_out[i] = leader_raw.get(m_id, -1)
                        follower_out[i] = follower_raws[i]
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
    """Leader reader in a background process; parent polls via getPositions()."""

    def __init__(
        self,
        port: str,
        leader_limits: list[JointLimits],
        follower_limits: list[JointLimits],
        baud: int = DEFAULT_BAUD,
        *,
        poll_interval_s: float = POLL_INTERVAL,
    ) -> None:
        self._port = port
        self._baud = baud
        self._leader_limits = leader_limits
        self._follower_limits = follower_limits
        self._poll_interval_s = poll_interval_s

        self._lock = mp.Lock()
        self._follower_raws = mp.Array("i", ARM_MOTOR_COUNT)
        self._leader_raws = mp.Array("i", ARM_MOTOR_COUNT)
        self._valid = mp.Value("b", False)
        self._arm_disabled = mp.Value("b", True)  # backdrivable by default
        self._stop = mp.Event()
        self._proc: mp.Process | None = None

    @staticmethod
    def limits_from_config(cfg: dict) -> tuple[list[JointLimits], list[JointLimits]]:
        """Build leader/follower limit lists from a config dict (see README)."""
        leader: list[JointLimits] = []
        follower: list[JointLimits] = []
        for key in JOINT_KEYS:
            l_min, l_max = cfg["leader"][key]
            f_min, f_max = cfg["follower"][key]
            if l_max < l_min:
                l_max += WRAP_OFFSET
            if f_max < f_min:
                f_max += WRAP_OFFSET
            leader.append(JointLimits(l_min, l_max))
            follower.append(JointLimits(f_min, f_max))
        return leader, follower

    @staticmethod
    def load_config(path: str | Path) -> tuple[list[JointLimits], list[JointLimits]]:
        with Path(path).open(encoding="utf-8") as f:
            return SO101Leader.limits_from_config(json.load(f))

    def start(self) -> None:
        if self._proc is not None and self._proc.is_alive():
            return
        self._stop.clear()
        self._proc = mp.Process(
            target=_so101_leader_worker,
            args=(
                self._port,
                self._baud,
                _limits_to_tuples(self._leader_limits),
                _limits_to_tuples(self._follower_limits),
                self._follower_raws,
                self._leader_raws,
                self._valid,
                self._arm_disabled,
                self._stop,
                self._poll_interval_s,
            ),
            daemon=True,
        )
        self._proc.start()

    def disable(self) -> None:
        """Release leader arm torque so joints can be moved by hand (default)."""
        with self._lock:
            self._arm_disabled.value = True

    def enable(self) -> None:
        """Engage leader arm torque (hold current position)."""
        with self._lock:
            self._arm_disabled.value = False

    def isArmEngaged(self) -> bool:
        """False by default until enable() is called."""
        with self._lock:
            return not bool(self._arm_disabled.value)

    def getPositions(self) -> list[int]:
        """Latest mapped follower joint raws (6 ints, J1..J6), or [] if none yet."""
        with self._lock:
            if not self._valid.value:
                return []
            return list(self._follower_raws[:])

    def status_line(self) -> str:
        """HUD line with leader (L) and follower (F) raw values."""
        with self._lock:
            if not self._valid.value:
                return ""
            leader = list(self._leader_raws[:])
            follower = list(self._follower_raws[:])
        parts = [
            f"J{m_id} L={leader[i]} F={follower[i]}" for i, m_id in enumerate(MOTOR_IDS)
        ]
        return " | ".join(parts)

    def stop(self) -> None:
        self._stop.set()
        if self._proc is not None:
            self._proc.join(timeout=2.0)
            self._proc = None
