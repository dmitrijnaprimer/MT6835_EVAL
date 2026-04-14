"""
MT6835 calibration manager for NLC and user auto-calibration.

Handles two calibration workflows:
- User auto-calibration: spins the motor, enables CAL_EN, polls until done.
- Stepped data collection: moves the shaft in discrete steps, collects
  LIR + MT6835 angle pairs at each position.

Also provides NLC table generation from collected data.

NLC encoding (confirmed by testing):
  - 256 entries, 6-bit two's complement (-32 to +31)
  - Packed MSB-first: AAAAAABB BBBBCCCC CCDDDDDD
  - Indexed by raw magnetic angle (before ZERO_POS subtraction)
  - 1 NLC LSB = 360 / 2^18 = 0.001373 deg
  - Chip internally removes the DC component of the table
"""

import datetime
import glob
import os
import time
from typing import Callable, List, Optional

import numpy as np
from PyQt5.QtCore import QObject, QTimer, pyqtSignal

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(os.path.dirname(_SCRIPT_DIR), "data")
NLC_DIR = os.path.join(DATA_DIR, "nlc")


class CalibrationManager(QObject):
    """Orchestrates MT6835 calibration workflows and data collection.

    :param serial_sender: Callable that sends a command string to the board.
    :param console_updater: Callable that logs a message to the GUI console.
    """

    calibration_progress = pyqtSignal(int, str)
    """Signal(percent, cal_type): emitted during collection to update progress bars."""

    calibration_status = pyqtSignal(str)
    """Signal(message): emitted for status text updates."""

    calibration_finished = pyqtSignal(bool, str)
    """Signal(success, cal_type): emitted when a calibration or collection completes."""

    NLC_POINTS = 256
    NLC_BYTES = 192
    NLC_LSB_DEGREES = 360.0 / (1 << 18)
    USTEPS_PER_REV = 12800

    def __init__(self, serial_sender: Callable, console_updater: Callable):
        """Initialize the calibration manager.

        :param serial_sender: Function to send a UART command string.
        :param console_updater: Function to append a line to the console.
        """
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
        """Start the MT6835 internal auto-calibration procedure.

        Non-blocking: sends commands via QTimer delays so the GUI stays responsive.

        :param rpm: Motor speed for calibration (25-200 RPM).
        """
        if self.calibration_in_progress:
            return
        self.calibration_in_progress = True
        self.calibration_type = "user"
        self._ucal_rpm = rpm
        self.console_updater(f"=== User Auto-Cal @ {rpm} RPM ===")
        self.serial_sender(f"SET_SPEED {rpm}")
        QTimer.singleShot(200, self._ucal_step2)

    def _ucal_step2(self):
        """Configure autocal frequency and start motor."""
        self.serial_sender(f"SET_AUTOCAL_RPM {self._ucal_rpm}")
        QTimer.singleShot(200, self._ucal_step3)

    def _ucal_step3(self):
        """Start motor and wait 5s for stable speed."""
        self.serial_sender("MOVE_CW")
        self.console_updater("Motor started, waiting 5s for stable speed...")
        QTimer.singleShot(5000, self._ucal_step4)

    def _ucal_step4(self):
        """Assert CAL_EN and start polling for completion."""
        self.serial_sender("MT6835_CAL_ENABLE")
        self.console_updater("CAL_EN asserted, polling for completion...")
        self.user_cal_timeout_counter = 0
        self.user_cal_timer.start(2000)

    def _check_user_cal_status(self):
        """Timer callback: read cal status register to check progress."""
        if not self.calibration_in_progress:
            self.user_cal_timer.stop()
            return
        self.serial_sender("STATUS")
        self.user_cal_timeout_counter += 1
        if self.user_cal_timeout_counter > 60:
            self.console_updater("ERROR: User Cal timeout (2 min)")
            self.cancel_calibration()

    def handle_user_cal_running(self):
        """Called when STATUS reports USER_CAL = Running."""
        self.console_updater("User Cal: Running...")

    def handle_user_cal_success(self):
        """Called when STATUS reports USER_CAL = OK."""
        self.user_cal_timer.stop()
        self.calibration_in_progress = False
        self.calibration_type = None
        self.console_updater("User Cal: SUCCESS! Waiting 6.5s for EEPROM...")
        QTimer.singleShot(6500, self._ucal_finish)

    def _ucal_finish(self):
        """Disable CAL_EN and stop motor after EEPROM write completes."""
        self.serial_sender("MT6835_CAL_DISABLE")
        self.serial_sender("STOP_MOTOR")
        self.console_updater("DONE. Power cycle MT6835 now.")
        self.calibration_finished.emit(True, "user")

    def handle_user_cal_error(self):
        """Called when STATUS reports USER_CAL = Failed."""
        self.console_updater("User Cal: FAILED!")
        self.cancel_calibration()

    def cancel_calibration(self):
        """Abort any running calibration or collection. Stops motor."""
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
        """Begin a stepped data collection across one full revolution.

        :param points: Number of measurement positions (e.g. 256).
        """
        if self.calibration_in_progress:
            return
        self.calibration_in_progress = True
        self.calibration_type = "collect"
        self.target_points = points
        self._start_stepped_process()

    def _start_stepped_process(self):
        """Read ZERO_POS, then start the collection state machine."""
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
        """Request ZERO_POS from the chip."""
        self.serial_sender("MT6835_READ_ZERO")
        time.sleep(0.3)
        QTimer.singleShot(500, self._configure_stepped_move)

    def handle_zero_pos_response(self, zero_pos_raw: int):
        """Store the ZERO_POS value received from the board.

        :param zero_pos_raw: 12-bit ZERO_POS register value (0-4095).
        """
        self.zero_pos_raw = zero_pos_raw
        self.console_updater(
            f"ZERO_POS={zero_pos_raw} ({zero_pos_raw * 360.0 / 4096:.2f} deg)"
        )

    def _configure_stepped_move(self):
        """Configure motor and start the collection state machine."""
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
        # Start by sampling at Home position (≈0°), then move+sample
        self.state = "SAMPLE_WAIT"
        self.console_updater("Collecting...")
        self.step_timer.start(50)

    def _step_process_logic(self):
        """50ms timer callback driving the collection state machine."""
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
        """Compute median of samples and store."""
        avg_lir = float(np.median(self.temp_lir_samples))
        avg_mt = float(np.median(self.temp_mt_samples))

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
        """Called by the GUI when a STATUS response contains both encoder angles.

        :param lir_deg: LIR angle in degrees.
        :param mt_deg: MT6835 angle in degrees.
        """
        if self.calibration_in_progress and self.state == "SAMPLE_RECEIVE":
            if self.calibration_type in ("nlc", "collect"):
                self.temp_lir_samples.append(lir_deg)
                self.temp_mt_samples.append(mt_deg)
                self.state = "CHECK_COMPLETE"

    def _finish_stepped_process(self):
        """Stop motor, save CSV with ZERO_POS header, emit completion."""
        self.step_timer.stop()
        self.serial_sender("STOP_MOTOR")
        self.console_updater(f"Done. {len(self.data_lir)} points collected.")

        os.makedirs(DATA_DIR, exist_ok=True)
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_name = f"collected_data_{ts}.csv"

        try:
            csv_path = os.path.join(DATA_DIR, csv_name)
            with open(csv_path, "w") as f:
                f.write(f"# ZERO_POS={self.zero_pos_raw}\n")
                f.write("LIR_deg,MT_deg\n")
                for l_val, m_val in zip(self.data_lir, self.data_mt):
                    f.write(f"{l_val:.6f},{m_val:.6f}\n")
            self.console_updater(f"Saved: {csv_name}")
            self.calibration_finished.emit(True, self.calibration_type)
        except Exception as e:
            self.console_updater(f"Error saving: {e}")
            self.calibration_finished.emit(False, self.calibration_type)
        finally:
            self.calibration_in_progress = False
            self.calibration_type = None
            self.state = "IDLE"

    # ================================================================
    # NLC generation from collected data
    # ================================================================

    def generate_nlc_from_last_collection(self):
        """Compute an NLC correction table from the last collected data.

        Uses direct assignment: measurement point i → NLC grid point i.
        This works because collection starts at Home (≈0°) and step size
        matches the NLC grid spacing (12800/256 = 50 µsteps = 1.406°).
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

        # Direct assignment: error[i] → NLC[i]
        # DC removal (chip does this internally, but we do it for prediction)
        error_ac = error - np.mean(error)

        correction_lsb = -error_ac / self.NLC_LSB_DEGREES
        nlc_signed = np.clip(np.round(correction_lsb).astype(int), -32, 31)

        # Account for ZERO_POS: NLC is indexed by physical angle
        zero_pos_shift = int(round(self.zero_pos_raw * 256 / 4096))
        nlc_shifted = np.roll(nlc_signed, zero_pos_shift)

        n_sat = int(np.sum((nlc_shifted == -32) | (nlc_shifted == 31)))
        chip_corr = (nlc_shifted - np.mean(nlc_shifted)) * self.NLC_LSB_DEGREES
        # Predict residual on unshifted error
        residual = error_ac + np.roll(chip_corr, -zero_pos_shift)

        self.console_updater(
            f"  NLC: [{np.min(nlc_shifted)}..{np.max(nlc_shifted)}]  "
            f"sat={n_sat}/256  ZERO_POS={self.zero_pos_raw} (shift={zero_pos_shift})\n"
            f"  Predicted: {np.ptp(error_ac):.5f} -> {np.ptp(residual):.5f} deg "
            f"({(1 - np.ptp(residual) / np.ptp(error_ac)) * 100:.0f}% reduction)"
        )

        hex_data = self._pack_nlc_msb_first(nlc_shifted).hex().upper()
        os.makedirs(NLC_DIR, exist_ok=True)
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        fname = f"nlc_table_{ts}.hex"
        with open(os.path.join(NLC_DIR, fname), "w") as f:
            f.write(hex_data)

        self.latest_hex_table = hex_data
        self.all_nlc_variants = {"nlc_table": hex_data}
        self.console_updater(f"  Saved: {fname}  (click 'Upload NLC' to load)")

    # ================================================================
    # NLC hex file management
    # ================================================================

    def get_available_nlc_hex_files(self):
        """Return list of .hex filenames in the NLC data directory.

        :return: Filenames sorted by modification time, most recent first.
        """
        if not os.path.isdir(NLC_DIR):
            return []
        files = glob.glob(os.path.join(NLC_DIR, "*.hex"))
        files.sort(key=os.path.getmtime, reverse=True)
        return [os.path.basename(f) for f in files]

    def load_nlc_hex_file(self, filename):
        """Load a hex string from a .hex file.

        :param filename: Filename (not full path).
        :return: 384-character uppercase hex string.
        """
        filepath = os.path.join(NLC_DIR, filename)
        with open(filepath, "r") as f:
            return f.read().strip()

    # ================================================================
    # NLC packing — MSB-first: AAAAAABB BBBBCCCC CCDDDDDD
    # ================================================================

    @staticmethod
    def _pack_nlc_msb_first(values: np.ndarray) -> bytearray:
        """Pack 256 signed 6-bit values into 192 bytes (MSB-first).

        :param values: Array of 256 integers in range [-32, 31].
        :return: 192-byte packed table.
        """
        packed = bytearray()
        for i in range(0, 256, 4):
            v = [int(values[i + j]) & 0x3F for j in range(4)]
            packed.append(((v[0]) << 2) | ((v[1] >> 4) & 0x03))
            packed.append(((v[1] & 0x0F) << 4) | ((v[2] >> 2) & 0x0F))
            packed.append(((v[2] & 0x03) << 6) | (v[3]))
        return packed
