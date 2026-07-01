"""SO-101 leader arm: Feetech SYNC_READ in a child process."""

from __future__ import annotations

import multiprocessing as mp
import sys
import time
from pathlib import Path

import numpy as np
import serial

from soble.sts_protocol import (
    ARM_JOINT_COUNT,
    DEFAULT_RAW_LIMITS,
    parse_arm_joints,
    sts_sync_enable_torque,
    sts_sync_read_positions,
    vector_map,
)
from soble.calibration_config import CalibrationConfig

MOTOR_IDS = [1, 2, 3, 4, 5, 6]

BELOW_MIN_MARGIN = 100
POLL_INTERVAL = 0.01  # 100 Hz

DEFAULT_BAUD = 1000000


def _so101_leader_worker(
    port: str,
    baud: int,
    leader_out: mp.Array,
    valid: mp.Value,
    arm_disabled: mp.Value,
    stop: mp.Event,
    poll_interval_s: float,
) -> None:
    try:
        ser = serial.Serial(port, baud, timeout=0.08)
    except serial.SerialException as e:
        print(f"Leader serial error: {e}", file=sys.stderr)
        return

    ser.reset_input_buffer()
    print(f"Leader arm {port} @ {baud}", flush=True)

    torque_engaged = not bool(arm_disabled.value)
    sts_sync_enable_torque(ser, MOTOR_IDS, torque_engaged)
    try:
        while not stop.is_set():
            want_engaged = not bool(arm_disabled.value)
            if want_engaged != torque_engaged:
                sts_sync_enable_torque(ser, MOTOR_IDS, want_engaged)
                torque_engaged = want_engaged

            positions = sts_sync_read_positions(ser, MOTOR_IDS)
            if all(positions.get(m_id) is not None for m_id in MOTOR_IDS):
                with valid.get_lock():
                    for i, m_id in enumerate(MOTOR_IDS):
                        leader_out[i] = int(positions[m_id])  # type: ignore[arg-type]
                    valid.value = True
            time.sleep(poll_interval_s)
    except Exception as e:
        print(f"Leader process error: {e}", file=sys.stderr)
    finally:
        if torque_engaged:
            sts_sync_enable_torque(ser, MOTOR_IDS, False)
        ser.close()


class SO101Leader:
    """Leader reader in a background process; parent polls via getArmPositions() / getMappedPositions()."""

    def __init__(
        self,
        port: str,
        *,
        config: CalibrationConfig | str | Path | None = None,
        baud: int = DEFAULT_BAUD,
        poll_interval_s: float = POLL_INTERVAL,
    ) -> None:
        self._port = port
        self._baud = baud
        self._poll_interval_s = poll_interval_s
        self._closed = False
        self._config: CalibrationConfig | None = None
        self._leader_raw_limits = DEFAULT_RAW_LIMITS.copy()
        self._follower_raw_limits = DEFAULT_RAW_LIMITS.copy()

        if config is not None:
            self._apply_config(CalibrationConfig.resolve(config))

        self._lock = mp.Lock()
        self._leader_raws = mp.Array("i", ARM_JOINT_COUNT)
        self._valid = mp.Value("b", False)
        self._arm_disabled = mp.Value("b", True)  # backdrivable by default
        self._stop_event = mp.Event()
        self._proc: mp.Process | None = None

        self._start()

    def _apply_config(self, config: CalibrationConfig) -> None:
        self._config = config
        self._leader_raw_limits, self._follower_raw_limits = config.limits_arrays()

    def load_config(self, config: CalibrationConfig | str | Path) -> None:
        """Load leader/follower joint limits from a calibration config (mapping only)."""
        with self._lock:
            self._apply_config(CalibrationConfig.resolve(config))

    @property
    def config(self) -> CalibrationConfig | None:
        return self._config

    def _start(self) -> None:
        """Start the serial reader (called from ``__init__``)."""
        if self._proc is not None and self._proc.is_alive():
            return
        self._closed = False
        self._stop_event.clear()
        self._proc = mp.Process(
            target=_so101_leader_worker,
            args=(
                self._port,
                self._baud,
                self._leader_raws,
                self._valid,
                self._arm_disabled,
                self._stop_event,
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
        """Latest leader joint raws (6 ints, J1..J6, 0..4095), or [] if none yet."""
        with self._lock:
            if not self._valid.value:
                return []
            return list(self._leader_raws[:])

    def getMappedPositions(self) -> list[int]:
        """Follower-mapped joint raws (6 ints, J1..J6), or [] if not ready."""
        with self._lock:
            if not self._valid.value:
                return []
            leader_raws = np.asarray(self._leader_raws[:], dtype=np.float64)
        mapped = vector_map(
            leader_raws, self._leader_raw_limits, self._follower_raw_limits
        )
        return (np.round(mapped).astype(int) & 0x0FFF).tolist()

    def _stop(self) -> None:
        """Stop the serial reader (called from ``__del__``)."""
        if self._closed:
            return
        self._closed = True
        self._stop_event.set()
        if self._proc is not None:
            self._proc.join(timeout=2.0)
            self._proc = None

    def __del__(self) -> None:
        try:
            self._stop()
        except Exception:
            pass
