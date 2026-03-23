"""
MT6835 Calibration Manager for NLC and User Auto-Calibration.

NLC encoding (confirmed):
  - 256 entries, 6-bit two's complement (-32 to +31)
  - Packed MSB-first: AAAAAABB BBBBCCCC CCDDDDDD
  - Indexed by raw magnetic angle (before ZERO_POS subtraction)
  - 1 NLC LSB = 360 / 2^18 = 0.001373 deg (nominal, under investigation)
  - Chip internally removes the DC component of the table
"""

import datetime
import glob
import os
import time
from typing import List, Optional, Callable

import numpy as np
from PyQt5.QtCore import QObject, QTimer, pyqtSignal
from PyQt5.QtWidgets import QApplication

"""Data directory: one level up from the control/ folder."""
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(os.path.dirname(_SCRIPT_DIR), "data")
NLC_DIR = os.path.join(DATA_DIR, "nlc")


class CalibrationManager(QObject):
    """Orchestrates MT6835 calibration workflows and data collection."""

    calibration_progress = pyqtSignal(int, str)
    calibration_status = pyqtSignal(str)
    calibration_finished = pyqtSignal(bool, str)

    NLC_POINTS = 256
    NLC_BYTES = 192
    NLC_LSB_DEGREES = 360.0 / (1 << 18)
    USTEPS_PER_REV = 12800

    def __init__(self, serial_sender: Callable, console_updater: Callable):
        super().__init__()
        self.serial_sender = serial_sender
        self.console_updater = console_updater

        self.calibration_in_progress = False
        self.calibration_type: Optional[str] = None

        self.user_cal_timer = QTimer()
        self.user_cal_timer.timeout.connect(self._check_user_cal_status)
        self.user_cal_timeout_counter = 0

        self.step_timer = QTimer()
        self.step_timer.timeout.connect(self._step_process_logic)

        self.step_idx = 0
        self.target_points = 256
        self.steps_per_segment = 50
        self.state = "IDLE"
        self.wait_counter = 0
        self.sample_interval_counter = 0

        self.data_lir: List[float] = []
        self.data_mt: List[float] = []
        self.temp_lir_samples: List[float] = []
        self.temp_mt_samples: List[float] = []

        self.latest_hex_table = ""
        self.all_nlc_variants = {}
        self.lir_bit_depth = 23
        self.zero_pos_raw = 0

        self.SAMPLES_PER_POINT = 12
        self.SETTLE_TIME_MS = 400
        self.SAMPLE_INTERVAL_MS = 50

    # ================================================================
    # User auto-calibration
    # ================================================================

    def start_user_calibration(self, rpm: int = 60):
        if self.calibration_in_progress:
            return
        self.calibration_in_progress = True
        self.calibration_type = "user"
        self.console_updater(f"=== User Auto-Cal @ {rpm} RPM ===")
        try:
            self.serial_sender(f"SET_SPEED {rpm}")
            time.sleep(0.2)
            self.serial_sender(f"SET_AUTOCAL_RPM {rpm}")
            time.sleep(0.2)
            self.console_updater("Spinning up motor (5s)...")
            time.sleep(5)
            self.serial_sender("MOVE_CW")
            time.sleep(2)
            self.serial_sender("MT6835_CAL_ENABLE")
            time.sleep(0.1)
            self.user_cal_timeout_counter = 0
            self.user_cal_timer.start(1000)
        except Exception as e:
            self.console_updater(f"Error: {e}")
            self.cancel_calibration()

    def _check_user_cal_status(self):
        if not self.calibration_in_progress:
            return
        self.serial_sender("STATUS")
        self.user_cal_timeout_counter += 1
        if self.user_cal_timeout_counter > 100:
            self.console_updater("ERROR: User Cal timeout")
            self.cancel_calibration()

    def handle_user_cal_running(self):
        self.console_updater("User Cal: Running...")

    def handle_user_cal_success(self):
        self.user_cal_timer.stop()
        self.console_updater("User Cal: SUCCESS! Waiting 6.5s for EEPROM...")
        for _ in range(65):
            QApplication.processEvents()
            time.sleep(0.1)
        self.serial_sender("MT6835_CAL_DISABLE")
        self.serial_sender("STOP_MOTOR")
        self.console_updater("DONE. Power cycle MT6835 now.")
        self.calibration_finished.emit(True, "user")
        self.calibration_in_progress = False
        self.calibration_type = None

    def handle_user_cal_error(self):
        self.console_updater("User Cal: FAILED!")
        self.cancel_calibration()

    def cancel_calibration(self):
        if self.calibration_in_progress:
            self.console_updater("Cancelling...")
            self.user_cal_timer.stop()
            self.step_timer.stop()
            self.serial_sender("MT6835_CAL_DISABLE")
            self.serial_sender("STOP_MOTOR")
            self.calibration_in_progress = False
            self.calibration_type = None
            self.state = "IDLE"

    # ================================================================
    # Stepped data collection
    # ================================================================

    def start_data_collection(self, points: int):
        """Collect measurement points for analysis or NLC generation."""
        if self.calibration_in_progress:
            return
        self.calibration_in_progress = True
        self.calibration_type = "collect"
        self.target_points = points
        self._start_stepped_process()

    def _start_stepped_process(self):
        est_sec = self.target_points * (
            self.SETTLE_TIME_MS / 1000.0
            + self.SAMPLES_PER_POINT * self.SAMPLE_INTERVAL_MS / 1000.0
        )
        self.console_updater(
            f"=== Collecting {self.target_points} pts (~{est_sec / 60:.1f} min) ==="
        )
        self.console_updater("Reading ZERO_POS...")
        QTimer.singleShot(200, self._read_zero_pos)

    def _read_zero_pos(self):
        self.serial_sender("MT6835_READ_ZERO")
        time.sleep(0.3)
        QTimer.singleShot(500, self._configure_stepped_move)

    def handle_zero_pos_response(self, zero_pos_raw: int):
        self.zero_pos_raw = zero_pos_raw
        self.console_updater(
            f"ZERO_POS={zero_pos_raw} ({zero_pos_raw * 360.0 / 4096:.2f} deg)"
        )

    def _configure_stepped_move(self):
        self.steps_per_segment = self.USTEPS_PER_REV // self.target_points
        self.console_updater(f"Config: 1 RPM, {self.steps_per_segment} usteps/segment")
        self.serial_sender("SET_SPEED 1")
        time.sleep(0.2)
        self.serial_sender(f"SET_STEPS {self.steps_per_segment}")
        time.sleep(0.2)
        self.serial_sender("TMC2225_MS_32")
        time.sleep(0.5)

        self.step_idx = 0
        self.data_lir = []
        self.data_mt = []
        self.temp_lir_samples = []
        self.temp_mt_samples = []
        self.wait_counter = 0
        self.sample_interval_counter = 0
        self.skip_first_point = True  # discard first point (no motor approach)
        self.state = "SETTLE"
        self.console_updater("Collecting...")
        self.step_timer.start(50)

    def _step_process_logic(self):
        if not self.calibration_in_progress:
            return

        if self.state == "SAMPLE_WAIT":
            self.sample_interval_counter += 1
            if self.sample_interval_counter >= self.SAMPLE_INTERVAL_MS / 50:
                self.sample_interval_counter = 0
                self.state = "SAMPLE_READ"

        elif self.state == "SAMPLE_READ":
            self.serial_sender("STATUS")
            self.state = "SAMPLE_RECEIVE"
            self.wait_counter = 0

        elif self.state == "SAMPLE_RECEIVE":
            self.wait_counter += 1
            if self.wait_counter > 20:
                self.state = "SAMPLE_READ"

        elif self.state == "CHECK_COMPLETE":
            if len(self.temp_lir_samples) >= self.SAMPLES_PER_POINT:
                self._store_point()
            else:
                self.sample_interval_counter = 0
                self.state = "SAMPLE_WAIT"

        elif self.state == "MOVE":
            if len(self.data_lir) >= self.target_points:
                self._finish_stepped_process()
                return
            self.serial_sender("MOVE_CW_STEPS")
            self.wait_counter = 0
            self.state = "SETTLE"

        elif self.state == "SETTLE":
            self.wait_counter += 1
            if self.wait_counter >= self.SETTLE_TIME_MS / 50:
                self.step_idx += 1
                self.calibration_progress.emit(
                    int((len(self.data_lir) / self.target_points) * 100),
                    self.calibration_type,
                )
                self.temp_lir_samples = []
                self.temp_mt_samples = []
                self.sample_interval_counter = 0
                self.state = "SAMPLE_WAIT"

    def _store_point(self):
        avg_lir = float(np.median(self.temp_lir_samples))
        avg_mt = float(np.median(self.temp_mt_samples))

        # Discard the first point — collected at starting position before
        # any stepped approach, often contains stale/transient readings.
        if self.skip_first_point:
            self.skip_first_point = False
            self.temp_lir_samples = []
            self.temp_mt_samples = []
            self.state = "MOVE"
            return

        self.data_lir.append(avg_lir)
        self.data_mt.append(avg_mt)

        n = len(self.data_lir)
        if n % 32 == 0:
            self.console_updater(
                f"  Pt {n}/{self.target_points}  LIR={avg_lir:.4f}  MT={avg_mt:.4f}"
            )
        self.temp_lir_samples = []
        self.temp_mt_samples = []
        self.state = "MOVE"

    def record_nlc_data_point(self, lir_deg: float, mt_deg: float):
        if self.calibration_in_progress and self.state == "SAMPLE_RECEIVE":
            if self.calibration_type in ["nlc", "collect"]:
                self.temp_lir_samples.append(lir_deg)
                self.temp_mt_samples.append(mt_deg)
                self.state = "CHECK_COMPLETE"

    def _finish_stepped_process(self):
        self.step_timer.stop()
        self.serial_sender("STOP_MOTOR")
        self.console_updater(f"Done. {len(self.data_lir)} points collected.")

        data_dir = DATA_DIR
        os.makedirs(data_dir, exist_ok=True)
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_name = f"collected_data_{ts}.csv"

        try:
            csv_path = os.path.join(data_dir, csv_name)
            with open(csv_path, "w") as f:
                f.write("LIR_deg,MT_deg\n")
                for l_val, m_val in zip(self.data_lir, self.data_mt):
                    f.write(f"{l_val:.6f},{m_val:.6f}\n")
            self.console_updater(f"Saved: {csv_name}")
            self.calibration_finished.emit(True, self.calibration_type)
        except Exception as e:
            self.console_updater(f"Error: {e}")
            self.calibration_finished.emit(False, self.calibration_type)
        finally:
            self.calibration_in_progress = False
            self.calibration_type = None
            self.state = "IDLE"

    # ================================================================
    # NLC generation from collected data
    # ================================================================

    def generate_nlc_from_last_collection(self):
        """Compute an NLC table from the most recent data_lir/data_mt arrays.

        Called from the 'Generate NLC' button. Uses the last collected data
        without requiring a new collection.
        """
        if not self.data_lir or not self.data_mt:
            self.console_updater("No data to generate NLC from.")
            return

        ref = np.array(self.data_lir)
        meas = np.array(self.data_mt)

        error = meas - ref
        error[error > 180] -= 360
        error[error < -180] += 360

        self.console_updater(
            f"=== Generate NLC from {len(ref)} pts ===\n"
            f"  Error: P2P={np.ptp(error):.5f}  "
            f"RMS={np.sqrt(np.mean(error**2)):.5f}  "
            f"Mean={np.mean(error):+.5f} deg"
        )

        # Map NLC grid to user angle space via ZERO_POS.
        zero_pos_deg = self.zero_pos_raw * 360.0 / 4096.0
        nlc_raw_angles = np.linspace(0, 360.0, 256, endpoint=False)
        nlc_user_angles = (nlc_raw_angles - zero_pos_deg) % 360.0

        ref_ext = np.concatenate([ref - 360, ref, ref + 360])
        err_ext = np.concatenate([error, error, error])
        sort_idx = np.argsort(ref_ext)
        error_at_nlc = np.interp(nlc_user_angles, ref_ext[sort_idx], err_ext[sort_idx])

        # Remove DC — chip does this internally.
        error_ac = error_at_nlc - np.mean(error_at_nlc)

        correction_lsb = -error_ac / self.NLC_LSB_DEGREES
        nlc_signed = np.clip(np.round(correction_lsb).astype(int), -32, 31)

        n_sat = np.sum((nlc_signed == -32) | (nlc_signed == 31))
        chip_corr = (nlc_signed - np.mean(nlc_signed)) * self.NLC_LSB_DEGREES
        residual = error_ac + chip_corr

        self.console_updater(
            f"  NLC: [{np.min(nlc_signed)}..{np.max(nlc_signed)}]  "
            f"sat={n_sat}/256  ZERO_POS={self.zero_pos_raw}\n"
            f"  Predicted: {np.ptp(error_ac):.5f} -> {np.ptp(residual):.5f} deg "
            f"({(1 - np.ptp(residual) / np.ptp(error_ac)) * 100:.0f}% reduction)"
        )

        # Save hex file
        hex_data = self._pack_nlc_msb_first(nlc_signed).hex().upper()
        nlc_dir = NLC_DIR
        os.makedirs(nlc_dir, exist_ok=True)
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        fname = f"nlc_table_{ts}.hex"
        with open(os.path.join(nlc_dir, fname), "w") as f:
            f.write(hex_data)

        self.latest_hex_table = hex_data
        self.all_nlc_variants = {"nlc_table": hex_data}
        self.console_updater(f"  Saved: {fname}  (click 'Upload NLC' to load)")

    # ================================================================
    # NLC hex file management
    # ================================================================

    def get_available_nlc_hex_files(self):
        nlc_dir = NLC_DIR
        if not os.path.isdir(nlc_dir):
            return []
        files = glob.glob(os.path.join(nlc_dir, "*.hex"))
        files.sort(key=os.path.getmtime, reverse=True)
        return [os.path.basename(f) for f in files]

    def load_nlc_hex_file(self, filename):
        filepath = os.path.join(NLC_DIR, filename)
        with open(filepath, "r") as f:
            return f.read().strip()

    # ================================================================
    # NLC packing — MSB-first: AAAAAABB BBBBCCCC CCDDDDDD
    # ================================================================

    def _pack_nlc_msb_first(self, values: np.ndarray) -> bytearray:
        packed = bytearray()
        for i in range(0, 256, 4):
            v = [int(values[i + j]) & 0x3F for j in range(4)]
            packed.append(((v[0]) << 2) | ((v[1] >> 4) & 0x03))
            packed.append(((v[1] & 0x0F) << 4) | ((v[2] >> 2) & 0x0F))
            packed.append(((v[2] & 0x03) << 6) | (v[3]))
        return packed
