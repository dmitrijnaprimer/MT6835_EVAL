"""Encoder evaluation GUI for MT6835 motor control.

Provides serial communication with the eval board, motor control,
MT6835 configuration (HYST, BW, NLC, ZERO_POS), stepped data
collection, and a visualization window.
"""

import sys
import time

import serial
import serial.tools.list_ports
from PyQt5.QtCore import QThread, QTimer, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QProgressBar,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from calibration import CalibrationManager
from visualize import VisualizationWindow

FONT_MAIN = "font-family: 'Segoe UI','Consolas',monospace; font-size: 12px;"
FONT_CONSOLE = "font-family: 'Consolas','Courier New',monospace; font-size: 11px;"
BTN_WIDTH = 120

QUIET_COMMANDS = {
    "STATUS",
    "MOVE_CW_STEPS",
    "MOVE_CCW_STEPS",
    "SET_STEPS",
    "SET_SPEED",
    "TMC2225_MS_32",
}

LIR_BIT_DEPTH = 23
USTEPS_PER_REV = 12800
DEG_PER_USTEP = 360.0 / USTEPS_PER_REV

"""Movement presets: (label, microsteps). All are exact multiples of one microstep."""
MOVE_PRESETS = [
    ("Continuous", 0),
    ("0.028° (1 µst)", 1),
    ("0.113° (4 µst)", 4),
    ("0.225° (8 µst)", 8),
    ("0.450° (16 µst)", 16),
    ("0.900° (32 µst)", 32),
    ("1.406° (50 µst)", 50),
    ("1.800° (64 µst)", 64),
    ("2.813° (100 µst)", 100),
    ("5.625° (200 µst)", 200),
    ("11.25° (400 µst)", 400),
    ("22.50° (800 µst)", 800),
    ("45.00° (1600 µst)", 1600),
    ("90.00° (3200 µst)", 3200),
    ("180.0° (6400 µst)", 6400),
    ("360.0° (12800 µst)", 12800),
]


class SerialReader(QThread):
    """Background thread for reading UART data line-by-line.

    Emits ``data_received(str)`` for each complete line,
    ``connection_status_changed(bool)`` on connect/disconnect,
    and ``error_occurred(str)`` on errors.
    """

    data_received = pyqtSignal(str)
    connection_status_changed = pyqtSignal(bool)
    error_occurred = pyqtSignal(str)

    def __init__(self, port_name, baudrate=115200):
        """Open a serial reader on the given port.

        :param port_name: COM port string (e.g. ``COM3``).
        :param baudrate: Baud rate (default 115200).
        """
        super().__init__()
        self.port_name = port_name
        self.baudrate = baudrate
        self.running = False
        self.serial_port = None
        self.buffer = ""

    def run(self):
        """Thread entry point. Opens port and reads until stopped."""
        try:
            self.serial_port = serial.Serial(
                self.port_name,
                self.baudrate,
                timeout=0.1,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                rtscts=False,
                dsrdtr=False,
            )
            self.running = True
            self.connection_status_changed.emit(True)
            while self.running:
                try:
                    if self.serial_port.in_waiting > 0:
                        raw = self.serial_port.read(self.serial_port.in_waiting)
                        self.buffer += raw.decode("utf-8", errors="ignore")
                        while "\n" in self.buffer:
                            line, self.buffer = self.buffer.split("\n", 1)
                            line = line.strip()
                            if line:
                                self.data_received.emit(line)
                except Exception as e:
                    self.error_occurred.emit(f"Read error: {e}")
                    break
                self.msleep(5)
        except Exception as e:
            self.error_occurred.emit(f"Connection error: {e}")
        finally:
            self.running = False
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
            self.connection_status_changed.emit(False)

    def stop(self):
        """Signal the thread to stop and close the serial port."""
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.reset_input_buffer()
                self.serial_port.reset_output_buffer()
                self.serial_port.close()
            except Exception:
                pass


def _btn(text, callback, width=BTN_WIDTH):
    """Create a fixed-width QPushButton connected to a callback."""
    b = QPushButton(text)
    b.setFixedWidth(width)
    b.clicked.connect(callback)
    return b


def _get_int(combo):
    """Extract the leading integer from a combo box text like ``50 RPM``."""
    return int(combo.currentText().split()[0])


class EncoderEvaluationGUI(QMainWindow):
    """Main application window for MT6835 encoder evaluation.

    Manages serial connection, motor control, MT6835 register access,
    data collection, NLC table upload, and visualization.
    """

    MT6835_BITS = 21
    MT6835_COUNTS = 2**21

    HYST_LABELS = [
        "HYST 0.022°",
        "HYST 0.044°",
        "HYST 0.088°",
        "HYST 0.176°",
        "HYST OFF",
        "HYST 0.003°",
        "HYST 0.006°",
        "HYST 0.011°",
    ]

    BW_LABELS = [
        "BW Baseline",
        "BW x2",
        "BW x4",
        "BW x8",
        "BW x16",
        "BW x32",
        "BW x64",
        "BW x128",
    ]

    def __init__(self):
        super().__init__()
        self.setWindowTitle("MT6835_EVAL_01")
        self.setGeometry(100, 50, 560, 820)
        self.setStyleSheet(FONT_MAIN)

        self.comms_connected = False
        self.homed = False
        self.tmc_enabled = False
        self.last_lir_raw = 0
        self.last_mt_raw = 0

        self.serial_reader_thread = None
        self.calibration_manager = None
        self.visualization_window = None
        self.command_history = []
        self.history_index = -1
        self.hide_status_messages = True
        self._suppress_combo_send = False

        self._build_ui()
        self._set_controls_enabled(False)

    # ================================================================
    # UI
    # ================================================================

    def _build_ui(self):
        """Assemble the main window layout from group boxes."""
        c = QWidget()
        self.setCentralWidget(c)
        root = QVBoxLayout(c)
        root.setSpacing(4)
        root.setContentsMargins(5, 5, 5, 5)
        root.addWidget(self._build_connection())
        root.addWidget(self._build_status())
        root.addWidget(self._build_motor())
        root.addWidget(self._build_mt6835())
        root.addWidget(self._build_analysis())
        root.addWidget(self._build_console(), 1)

    def _build_connection(self):
        g = QGroupBox("Connection")
        lay = QHBoxLayout(g)
        lay.setSpacing(4)
        self.com_port_combo = QComboBox()
        self.com_port_combo.setMinimumWidth(100)
        self._refresh_com_ports()
        lay.addWidget(QLabel("Port:"))
        lay.addWidget(self.com_port_combo, 1)
        lay.addWidget(_btn("Refresh", self._refresh_com_ports))
        self.connect_btn = _btn("Connect", self._on_connect)
        self.disconnect_btn = _btn("Disconnect", self._on_disconnect)
        lay.addWidget(self.connect_btn)
        lay.addWidget(self.disconnect_btn)
        return g

    def _build_status(self):
        g = QGroupBox("Status")
        lay = QGridLayout(g)
        lay.setSpacing(2)
        lay.setColumnStretch(1, 1)
        lay.setColumnStretch(3, 1)
        self.st_comms = QLabel("Disconnected")
        self.st_homed = QLabel("No")
        self.st_tmc = QLabel("---")
        self.st_lir_pos = QLabel("---")
        self.st_mt_pos = QLabel("---")
        self.st_mt_raw = QLabel("---")
        self.st_mt_zero = QLabel("---")
        r = 0
        lay.addWidget(QLabel("Comms:"), r, 0)
        lay.addWidget(self.st_comms, r, 1)
        lay.addWidget(QLabel("LIR-DA237T:"), r, 2)
        lay.addWidget(self.st_lir_pos, r, 3)
        r += 1
        lay.addWidget(QLabel("Homed:"), r, 0)
        lay.addWidget(self.st_homed, r, 1)
        lay.addWidget(QLabel("MT6835:"), r, 2)
        lay.addWidget(self.st_mt_pos, r, 3)
        r += 1
        lay.addWidget(QLabel("TMC2225:"), r, 0)
        lay.addWidget(self.st_tmc, r, 1)
        lay.addWidget(QLabel("MT6835 RAW:"), r, 2)
        lay.addWidget(self.st_mt_raw, r, 3)
        r += 1
        lay.addWidget(QLabel(""), r, 0)
        lay.addWidget(QLabel(""), r, 1)
        lay.addWidget(QLabel("MT6835 ZERO_POS:"), r, 2)
        lay.addWidget(self.st_mt_zero, r, 3)
        return g

    def _build_motor(self):
        g = QGroupBox("Motor Control")
        lay = QGridLayout(g)
        lay.setSpacing(3)

        self.stop_btn = _btn("STOP", self._on_stop)
        self.home_btn = _btn("Home", self._on_home)
        self.speed_combo = QComboBox()
        self.speed_combo.setFixedWidth(BTN_WIDTH)
        self.speed_combo.addItems([
            "1 RPM",
            "5 RPM",
            "10 RPM",
            "25 RPM",
            "30 RPM",
            "40 RPM",
            "50 RPM",
            "60 RPM",
            "75 RPM",
            "100 RPM",
            "150 RPM",
            "200 RPM",
        ])
        self.speed_combo.setCurrentText("50 RPM")
        self.speed_combo.currentTextChanged.connect(self._on_speed_changed)
        self.refresh_status_btn = _btn("Refresh", self._request_status)
        lay.addWidget(self.stop_btn, 0, 0)
        lay.addWidget(self.home_btn, 0, 1)
        lay.addWidget(self.speed_combo, 0, 2)
        lay.addWidget(self.refresh_status_btn, 0, 3)

        # Movement: single combo for distance, CW/CCW buttons
        self.move_combo = QComboBox()
        self.move_combo.setMinimumWidth(200)
        for label, _usteps in MOVE_PRESETS:
            self.move_combo.addItem(label)
        self.move_combo.setCurrentIndex(13)  # 90°

        self.move_cw_btn = _btn("Move CW", self._on_move_cw)
        self.move_ccw_btn = _btn("Move CCW", self._on_move_ccw)

        lay.addWidget(self.move_combo, 1, 0, 1, 2)
        lay.addWidget(self.move_cw_btn, 1, 2)
        lay.addWidget(self.move_ccw_btn, 1, 3)
        return g

    def _build_mt6835(self):
        g = QGroupBox("MT6835")
        lay = QGridLayout(g)
        lay.setSpacing(3)

        self.set_zero_btn = _btn("Set Zero Pos", self._on_set_zero)
        self.ucal_rpm = QComboBox()
        self.ucal_rpm.setFixedWidth(BTN_WIDTH)
        self.ucal_rpm.addItems([
            "25 RPM",
            "30 RPM",
            "40 RPM",
            "50 RPM",
            "60 RPM",
            "75 RPM",
            "100 RPM",
            "150 RPM",
            "200 RPM",
        ])
        self.ucal_rpm.setCurrentText("50 RPM")
        self.ucal_btn = _btn("User Auto-Cal", self._on_user_cal)
        self.mt_progress = QProgressBar()
        self.mt_progress.setFixedWidth(BTN_WIDTH)
        self.mt_progress.setValue(0)
        lay.addWidget(self.set_zero_btn, 0, 0)
        lay.addWidget(self.ucal_rpm, 0, 1)
        lay.addWidget(self.ucal_btn, 0, 2)
        lay.addWidget(self.mt_progress, 0, 3)

        # HYST and BW
        self.hyst_combo = QComboBox()
        self.hyst_combo.setFixedWidth(BTN_WIDTH)
        self.hyst_combo.addItems(self.HYST_LABELS)
        self.hyst_combo.setCurrentIndex(7)
        self.hyst_combo.currentIndexChanged.connect(self._on_hyst_changed)

        self.bw_combo = QComboBox()
        self.bw_combo.setFixedWidth(BTN_WIDTH)
        self.bw_combo.addItems(self.BW_LABELS)
        self.bw_combo.setCurrentIndex(5)
        self.bw_combo.currentIndexChanged.connect(self._on_bw_changed)

        self.nlc_upload_btn = _btn("Upload NLC", self._on_nlc_upload)
        self.nlc_en_btn = _btn("Enable NLC", lambda: self._send("MT6835_ENABLE_NLC"))
        self.nlc_dis_btn = _btn("Disable NLC", lambda: self._send("MT6835_DISABLE_NLC"))
        self.nlc_clear_btn = _btn("Clear NLC", self._on_nlc_clear)
        self.nlc_prog_btn = _btn("Program EEPROM", self._on_program_eeprom)
        self.read_regs_btn = _btn("Read Registers", self._on_read_registers)

        lay.addWidget(self.hyst_combo, 1, 0)
        lay.addWidget(self.bw_combo, 1, 1)
        lay.addWidget(self.nlc_upload_btn, 1, 2)
        lay.addWidget(self.nlc_en_btn, 1, 3)
        lay.addWidget(self.nlc_dis_btn, 2, 0)
        lay.addWidget(self.nlc_clear_btn, 2, 1)
        lay.addWidget(self.nlc_prog_btn, 2, 2)
        lay.addWidget(self.read_regs_btn, 2, 3)
        return g

    def _build_analysis(self):
        g = QGroupBox("Analysis")
        lay = QGridLayout(g)
        lay.setSpacing(3)
        self.collect_pts = QComboBox()
        self.collect_pts.setFixedWidth(BTN_WIDTH)
        self.collect_pts.addItems([
            "64 pts",
            "100 pts",
            "128 pts",
            "200 pts",
            "256 pts",
            "320 pts",
            "400 pts",
            "512 pts",
            "640 pts",
            "800 pts",
            "1280 pts",
            "1600 pts",
        ])
        self.collect_pts.setCurrentText("256 pts")
        self.collect_btn = _btn("Collect Data", self._on_collect)
        self.gen_nlc_btn = _btn("Generate NLC", self._on_generate_nlc)
        self.viz_btn = _btn("Visualization", self._on_visualize)
        self.analysis_progress = QProgressBar()
        self.analysis_progress.setFixedWidth(BTN_WIDTH)
        self.analysis_progress.setValue(0)
        lay.addWidget(self.collect_pts, 0, 0)
        lay.addWidget(self.collect_btn, 0, 1)
        lay.addWidget(self.gen_nlc_btn, 0, 2)
        lay.addWidget(self.analysis_progress, 0, 3)
        lay.addWidget(self.viz_btn, 1, 0)
        return g

    def _build_console(self):
        g = QGroupBox("Console")
        lay = QVBoxLayout(g)
        self.console = QPlainTextEdit()
        self.console.setReadOnly(True)
        self.console.setMaximumBlockCount(500)
        self.console.setMinimumHeight(120)
        self.console.setStyleSheet(FONT_CONSOLE)
        lay.addWidget(self.console)
        cl = QHBoxLayout()
        self.hide_status_cb = QCheckBox("Hide auto-commands")
        self.hide_status_cb.setChecked(True)
        self.hide_status_cb.stateChanged.connect(
            lambda s: setattr(self, "hide_status_messages", s == 2)
        )
        cl.addWidget(self.hide_status_cb)
        cl.addStretch()
        cl.addWidget(_btn("Clear", self.console.clear))
        lay.addLayout(cl)
        self.cmd_input = QLineEdit()
        self.cmd_input.returnPressed.connect(self._on_cmd_entered)
        lay.addWidget(self.cmd_input)
        return g

    # ================================================================
    # Connection
    # ================================================================

    def _refresh_com_ports(self):
        self.com_port_combo.clear()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.com_port_combo.addItems(ports if ports else ["No ports found"])

    def _log(self, t):
        self.console.appendPlainText(t)

    def _set_controls_enabled(self, on):
        self.connect_btn.setEnabled(not on)
        self.disconnect_btn.setEnabled(on)
        self.com_port_combo.setEnabled(not on)
        for w in [
            self.stop_btn,
            self.home_btn,
            self.refresh_status_btn,
            self.speed_combo,
            self.move_combo,
            self.move_cw_btn,
            self.move_ccw_btn,
            self.set_zero_btn,
            self.ucal_btn,
            self.ucal_rpm,
            self.hyst_combo,
            self.bw_combo,
            self.nlc_upload_btn,
            self.nlc_en_btn,
            self.nlc_dis_btn,
            self.nlc_clear_btn,
            self.nlc_prog_btn,
            self.read_regs_btn,
            self.collect_pts,
            self.collect_btn,
            self.gen_nlc_btn,
            self.viz_btn,
            self.cmd_input,
        ]:
            w.setEnabled(on)

    def _reset_status(self):
        self.homed = self.tmc_enabled = False
        for w in [
            self.st_tmc,
            self.st_lir_pos,
            self.st_mt_pos,
            self.st_mt_raw,
            self.st_mt_zero,
        ]:
            w.setText("---")
        self.st_homed.setText("No")

    def _on_connect(self):
        if self.serial_reader_thread and self.serial_reader_thread.isRunning():
            return
        port = self.com_port_combo.currentText()
        if not port or "No ports" in port:
            QMessageBox.warning(self, "Error", "Select a valid COM port.")
            return
        self.serial_reader_thread = SerialReader(port, 115200)
        self.serial_reader_thread.data_received.connect(self._handle_serial_data)
        self.serial_reader_thread.connection_status_changed.connect(
            self._connection_changed
        )
        self.serial_reader_thread.error_occurred.connect(self._serial_error)
        self.serial_reader_thread.start()
        self.calibration_manager = CalibrationManager(self._send, self._log)
        self.calibration_manager.calibration_progress.connect(self._cal_progress)
        self.calibration_manager.calibration_finished.connect(self._cal_finished)

    def _on_disconnect(self):
        if self.calibration_manager:
            self.calibration_manager.cancel_calibration()
        if self.serial_reader_thread:
            self.serial_reader_thread.stop()
            self.serial_reader_thread.wait(2000)
            self.serial_reader_thread = None
        self._connection_changed(False)

    def _connection_changed(self, connected):
        self.comms_connected = connected
        if connected:
            self.st_comms.setText("Connected")
            self._set_controls_enabled(True)
            self._suppress_combo_send = True
            self._send("CONNECT")
            self._send(f"LIR_BITS {LIR_BIT_DEPTH}")
            self._send(f"SET_SPEED {_get_int(self.speed_combo)}")
            self._suppress_combo_send = False
            QTimer.singleShot(300, self._request_status)
        else:
            self.st_comms.setText("Disconnected")
            self._set_controls_enabled(False)
            self._reset_status()

    def _serial_error(self, msg):
        self._log(f"Serial Error: {msg}")
        self._on_disconnect()

    # ================================================================
    # Serial
    # ================================================================

    def _send(self, cmd):
        """Send a command string over UART. Logs unless suppressed."""
        if not (
            self.serial_reader_thread
            and self.serial_reader_thread.isRunning()
            and self.serial_reader_thread.serial_port
            and self.serial_reader_thread.serial_port.is_open
        ):
            return
        try:
            self.serial_reader_thread.serial_port.write((cmd + "\n").encode())
            cmd_word = cmd.split()[0] if cmd else ""
            if not (self.hide_status_messages and cmd_word in QUIET_COMMANDS):
                self._log(f"> {cmd}")
            time.sleep(0.02)
        except Exception as e:
            self._serial_error(str(e))

    def _request_status(self):
        self._send("STATUS")

    def _delayed_status(self, delay_ms=500):
        QTimer.singleShot(delay_ms, self._request_status)

    def _on_cmd_entered(self):
        cmd = self.cmd_input.text().strip()
        if cmd:
            self.command_history.append(cmd)
            self.history_index = len(self.command_history)
            self._send(cmd)
            self.cmd_input.clear()

    # ================================================================
    # Motor
    # ================================================================

    def _on_stop(self):
        if self.calibration_manager:
            self.calibration_manager.cancel_calibration()
        self._send("STOP_MOTOR")
        self._delayed_status()

    def _on_home(self):
        self._send("HOME")

    def _on_speed_changed(self, t):
        if self._suppress_combo_send:
            return
        try:
            self._send(f"SET_SPEED {int(t.split()[0])}")
        except Exception:
            pass

    def _get_move_usteps(self):
        """Return microsteps for the selected move preset (0 = continuous)."""
        idx = self.move_combo.currentIndex()
        if 0 <= idx < len(MOVE_PRESETS):
            return MOVE_PRESETS[idx][1]
        return 0

    def _on_move_cw(self):
        self._send(f"SET_SPEED {_get_int(self.speed_combo)}")
        time.sleep(0.05)
        usteps = self._get_move_usteps()
        if usteps == 0:
            self._send("MOVE_CW")
        else:
            self._send(f"SET_STEPS {usteps}")
            time.sleep(0.02)
            self._send("MOVE_CW_STEPS")
            self._delayed_status(1000)

    def _on_move_ccw(self):
        self._send(f"SET_SPEED {_get_int(self.speed_combo)}")
        time.sleep(0.05)
        usteps = self._get_move_usteps()
        if usteps == 0:
            self._send("MOVE_CCW")
        else:
            self._send(f"SET_STEPS {usteps}")
            time.sleep(0.02)
            self._send("MOVE_CCW_STEPS")
            self._delayed_status(1000)

    # ================================================================
    # MT6835
    # ================================================================

    def _on_set_zero(self):
        if (
            QMessageBox.question(
                self,
                "Set MT6835 ZERO_POS",
                "Set current position as MT6835 zero?\n\n"
                "This invalidates any existing NLC calibration.",
                QMessageBox.Yes | QMessageBox.No,
            )
            == QMessageBox.Yes
        ):
            self._send("SET_ZERO_MT6835")
            self._delayed_status()

    def _on_user_cal(self):
        if self.calibration_manager:
            self.calibration_manager.start_user_calibration(_get_int(self.ucal_rpm))

    def _on_hyst_changed(self, index):
        if not self._suppress_combo_send:
            self._send(f"MT6835_SET_HYST {index}")

    def _on_bw_changed(self, index):
        if not self._suppress_combo_send:
            self._send(f"MT6835_SET_BW {index}")

    def _on_read_registers(self):
        self._send("MT6835_READ_REGISTERS")

    def _on_nlc_upload(self):
        if not self.calibration_manager:
            return
        variants = getattr(self.calibration_manager, "all_nlc_variants", {})
        memory_names = [f"[memory] {k}" for k in sorted(variants.keys())]
        hex_files = self.calibration_manager.get_available_nlc_hex_files()
        all_choices = memory_names + hex_files
        if not all_choices:
            self._log("No NLC tables in data/nlc/.")
            return
        choice, ok = QInputDialog.getItem(
            self,
            "Select NLC Table",
            "Choose NLC table to upload:",
            all_choices,
            0,
            False,
        )
        if not ok or not choice:
            return
        try:
            if choice.startswith("[memory] "):
                hex_data = variants[choice.replace("[memory] ", "")]
            else:
                hex_data = self.calibration_manager.load_nlc_hex_file(choice)
            if len(hex_data) != 384:
                self._log(f"ERR: Need 384 hex chars, got {len(hex_data)}")
                return
            self._log(f"Uploading: {choice}")
            self.calibration_manager.latest_hex_table = hex_data
            self._send(f"LOAD_NLC {hex_data}")
            self._delayed_status(8000)
        except Exception as e:
            self._log(f"Error: {e}")

    def _on_nlc_clear(self):
        if (
            QMessageBox.question(
                self,
                "Clear NLC",
                "Erase NLC from MT6835 EEPROM?",
                QMessageBox.Yes | QMessageBox.No,
            )
            == QMessageBox.Yes
        ):
            self._send("MT6835_CLEAR_NLC")
            self._delayed_status(8000)

    def _on_program_eeprom(self):
        if (
            QMessageBox.question(
                self,
                "Program EEPROM",
                "Program MT6835 registers to EEPROM?\nTakes ~6 seconds.",
                QMessageBox.Yes | QMessageBox.No,
            )
            == QMessageBox.Yes
        ):
            self._send("MT6835_PROGRAM_EEPROM")
            self._delayed_status(8000)

    # ================================================================
    # Analysis
    # ================================================================

    def _on_collect(self):
        if self.calibration_manager:
            self.calibration_manager.lir_bit_depth = LIR_BIT_DEPTH
            self.calibration_manager.start_data_collection(_get_int(self.collect_pts))

    def _on_generate_nlc(self):
        if not self.calibration_manager:
            return
        if not self.calibration_manager.data_lir:
            self._log("No collected data. Run 'Collect Data' first.")
            return
        self.calibration_manager.generate_nlc_from_last_collection()

    def _cal_progress(self, pct, cal_type):
        self.analysis_progress.setValue(pct)

    def _cal_finished(self, success, cal_type):
        self.analysis_progress.setValue(100 if success else 0)
        self._delayed_status()

    def _on_visualize(self):
        self.visualization_window = VisualizationWindow()
        self.visualization_window.exec_()

    # ================================================================
    # Serial data routing
    # ================================================================

    HYST_DECODE = {
        0: "0.022°",
        1: "0.044°",
        2: "0.088°",
        3: "0.176°",
        4: "OFF",
        5: "0.003°",
        6: "0.006°",
        7: "0.011°",
    }
    BW_DECODE = {
        0: "Baseline",
        1: "x2",
        2: "x4",
        3: "x8",
        4: "x16",
        5: "x32",
        6: "x64",
        7: "x128",
    }

    def _handle_serial_data(self, text):
        """Route incoming serial lines to the appropriate handler."""
        text = text.strip()
        if not text:
            return

        if text.startswith("NLC_DUMP:"):
            dump = text.split(":", 1)[1].strip()
            if self.calibration_manager and self.calibration_manager.latest_hex_table:
                self._log(
                    f"< NLC verify: {'MATCH' if dump == self.calibration_manager.latest_hex_table else 'MISMATCH'}"
                )
            else:
                self._log(f"< NLC dump ({len(dump) // 2} bytes)")
            return

        if text.startswith("NLC_RAW_DUMP:"):
            dump = text.split(":", 1)[1].strip()
            nonzero = sum(1 for i in range(0, len(dump), 2) if dump[i : i + 2] != "00")
            self._log(
                f"< NLC chip: {nonzero} non-zero bytes"
                if nonzero
                else "< NLC chip: empty"
            )
            return

        if text.startswith("MT_REG_DUMP:"):
            self._decode_register_dump(text.split(":", 1)[1].strip())
            return

        if text.startswith("OK:ZERO_POS="):
            try:
                zp = int(text.split("=")[1].split()[0])
                if self.calibration_manager:
                    self.calibration_manager.handle_zero_pos_response(zp)
            except Exception:
                pass
            self._log(f"< {text}")
            return

        if text.startswith("OK:Homed") or text.startswith("OK:MT6835 zero"):
            self._log(f"< {text}")
            self._delayed_status()
            return

        if "STATUS_" in text:
            self._parse_status(text)
            if not self.hide_status_messages:
                self._log(f"< {text[:120]}")
            return

        if (
            text.startswith("OK:")
            and self.hide_status_messages
            and self.calibration_manager
            and self.calibration_manager.calibration_in_progress
        ):
            return

        self._log(f"< {text}" if text.startswith(("OK:", "ERR:", "INFO:")) else text)

    def _decode_register_dump(self, raw):
        """Parse MT6835 register dump and display with human-readable decode. and display MT6835 registers in a human-readable format."""
        regs = {}
        for pair in raw.split(","):
            if "=" in pair:
                addr_s, val_s = pair.strip().split("=", 1)
                try:
                    regs[int(addr_s, 16)] = int(val_s, 16)
                except ValueError:
                    continue

        self._log("< MT6835 Register Dump:")

        for addr in sorted(regs.keys()):
            val = regs[addr]
            decode = ""

            if addr == 0x001:
                decode = f"USER_ID={val}"
            elif addr == 0x009:
                zp_high = val
                zp_low = regs.get(0x00A, 0)
                zp = (zp_high << 4) | ((zp_low >> 4) & 0x0F)
                decode = f"ZERO_POS={zp} ({zp * 360.0 / 4096:.3f}°)"
            elif addr == 0x00A:
                z_edge = (val >> 3) & 1
                z_wid = val & 0x07
                decode = f"Z_EDGE={z_edge} Z_WID={z_wid}"
            elif addr == 0x00C:
                nlc_en = (val >> 5) & 1
                pwm_fq = (val >> 4) & 1
                decode = f"NLC_EN={nlc_en} PWM_FQ={'497' if pwm_fq else '994'}Hz"
            elif addr == 0x00D:
                rot_dir = (val >> 3) & 1
                hyst = val & 0x07
                decode = f"ROT_DIR={'CW' if rot_dir else 'CCW'} HYST={self.HYST_DECODE.get(hyst, '?')}"
            elif addr == 0x00E:
                gpio_ds = (val >> 7) & 1
                acf = (val >> 4) & 0x07
                decode = f"GPIO_DS={gpio_ds} AUTOCAL_FREQ={acf}"
            elif addr == 0x011:
                bw = val & 0x07
                decode = f"BW={self.BW_DECODE.get(bw, '?')}"

            line = f"  0x{addr:03X} = 0x{val:02X}"
            if decode:
                line += f"  ({decode})"
            self._log(line)

    def _parse_status(self, text):
        """Parse a STATUS response and update all GUI fields."""
        try:
            lir_deg = mt_deg = None
            for part in text.split(","):
                if ":" not in part:
                    continue
                key, val = part.split(":", 1)
                key, val = key.strip(), val.strip()

                if key == "STATUS_LIR-DA237T_POS":
                    self.last_lir_raw = int(val)
                    lir_deg = (self.last_lir_raw / (2**LIR_BIT_DEPTH)) * 360.0
                    self.st_lir_pos.setText(f"{lir_deg:.6f}°")
                elif key == "STATUS_MT6835_POS":
                    self.last_mt_raw = int(val)
                    mt_deg = (self.last_mt_raw / self.MT6835_COUNTS) * 360.0
                    self.st_mt_pos.setText(f"{mt_deg:.6f}°  ({self.last_mt_raw})")
                elif key == "STATUS_MT6835_RAW":
                    raw_val = int(val)
                    raw_deg = (raw_val / self.MT6835_COUNTS) * 360.0
                    self.st_mt_raw.setText(f"{raw_deg:.6f}°  ({raw_val})")
                elif key == "STATUS_MT6835_ZERO_POS":
                    zp = int(val)
                    self.st_mt_zero.setText(f"{zp * 360.0 / 4096:.3f}°  ({zp})")
                elif key == "STATUS_TMC2225_EN":
                    self.tmc_enabled = val.lower() == "true"
                    self.st_tmc.setText("Enabled" if self.tmc_enabled else "Disabled")
                elif key == "STATUS_HOME":
                    self.homed = val.lower() == "true"
                    self.st_homed.setText("Yes" if self.homed else "No")
                elif key == "STATUS_MT6835_HYST":
                    idx = int(val)
                    if 0 <= idx <= 7 and self.hyst_combo.currentIndex() != idx:
                        self._suppress_combo_send = True
                        self.hyst_combo.setCurrentIndex(idx)
                        self._suppress_combo_send = False
                elif key == "STATUS_MT6835_BW":
                    idx = int(val)
                    if 0 <= idx <= 7 and self.bw_combo.currentIndex() != idx:
                        self._suppress_combo_send = True
                        self.bw_combo.setCurrentIndex(idx)
                        self._suppress_combo_send = False
                elif key == "STATUS_MT6835_USER_CAL":
                    if (
                        self.calibration_manager
                        and self.calibration_manager.calibration_in_progress
                        and self.calibration_manager.calibration_type == "user"
                    ):
                        v = val.upper().strip()
                        if v == "RUNNING":
                            self.calibration_manager.handle_user_cal_running()
                        elif v == "OK":
                            self.calibration_manager.handle_user_cal_success()
                        elif v in ("FAILED", "ERROR"):
                            self.calibration_manager.handle_user_cal_error()

            if lir_deg is not None and mt_deg is not None and self.calibration_manager:
                self.calibration_manager.record_nlc_data_point(lir_deg, mt_deg)
        except Exception as e:
            self._log(f"Parse error: {e}")

    def keyPressEvent(self, event):
        """Handle Up/Down arrow keys for command history."""
        if event.key() == 16777235 and self.history_index > 0:
            self.history_index -= 1
            self.cmd_input.setText(self.command_history[self.history_index])
        elif event.key() == 16777237:
            if self.history_index < len(self.command_history) - 1:
                self.history_index += 1
                self.cmd_input.setText(self.command_history[self.history_index])
            else:
                self.history_index = len(self.command_history)
                self.cmd_input.clear()
        else:
            super().keyPressEvent(event)


def main():
    """Application entry point."""
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = EncoderEvaluationGUI()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
