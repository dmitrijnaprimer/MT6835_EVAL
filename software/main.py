"""MT6835 encoder evaluation GUI."""

import sys
import time

import serial
import serial.tools.list_ports
from PyQt5.QtCore import Qt, QThread, QTimer, pyqtSignal
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
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from calibration import CalibrationManager
from visualize import VisualizationWindow

FONT_MAIN = "font-family: 'Segoe UI', sans-serif; font-size: 12px;"
FONT_CONSOLE = "font-family: 'Consolas', 'Courier New', monospace; font-size: 11px;"
BTN_WIDTH = 120

# Commands suppressed from console when "Hide auto-commands" is checked.
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

# (label, microsteps) — 0 means continuous rotation.
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
    "BW ×2",
    "BW ×4",
    "BW ×8",
    "BW ×16",
    "BW ×32",
    "BW ×64",
    "BW ×128",
]

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
    1: "×2",
    2: "×4",
    3: "×8",
    4: "×16",
    5: "×32",
    6: "×64",
    7: "×128",
}


# ====================================================================
# Serial reader thread
# ====================================================================


class SerialReader(QThread):
    """Background thread that reads UART lines and emits them as signals."""

    data_received = pyqtSignal(str)
    connection_status_changed = pyqtSignal(bool)
    error_occurred = pyqtSignal(str)

    def __init__(self, port_name, baudrate=115200):
        super().__init__()
        self.port_name = port_name
        self.baudrate = baudrate
        self.running = False
        self.serial_port = None
        self.buffer = ""

    def run(self):
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
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.reset_input_buffer()
                self.serial_port.reset_output_buffer()
                self.serial_port.close()
            except Exception:
                pass


# ====================================================================
# Helpers
# ====================================================================


def _btn(text, callback, width=BTN_WIDTH):
    """Fixed-width button wired to *callback*."""
    b = QPushButton(text)
    b.setFixedWidth(width)
    b.clicked.connect(callback)
    return b


def _combo(items, current=None, width=BTN_WIDTH):
    """Fixed-width combo box with centered text."""
    c = QComboBox()
    c.setFixedWidth(width)
    c.setEditable(False)
    c.addItems(items)
    # Center-align the displayed text and dropdown items.
    c.setStyleSheet(
        "QComboBox { text-align: center; }"
        "QComboBox QAbstractItemView { text-align: center; }"
    )
    for i in range(c.count()):
        c.setItemData(i, Qt.AlignCenter, Qt.TextAlignmentRole)
    if current:
        c.setCurrentText(current)
    return c


def _get_int(combo):
    """Leading integer from combo text like '50 RPM'."""
    return int(combo.currentText().split()[0])


# ====================================================================
# Main window
# ====================================================================


class EncoderEvaluationGUI(QMainWindow):
    """MT6835 encoder evaluation application."""

    MT6835_COUNTS = 2**21

    def __init__(self):
        super().__init__()
        self.setWindowTitle("MT6835 Encoder Evaluation")
        self.setGeometry(100, 30, 560, 960)
        self.setStyleSheet(FONT_MAIN)

        self.comms_connected = False
        self.homed = False
        self.tmc_enabled = False
        self.nlc_enabled = False
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

    # ----------------------------------------------------------------
    # UI construction
    # ----------------------------------------------------------------

    def _build_ui(self):
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
        lay.setSpacing(3)
        lay.setColumnStretch(1, 1)
        lay.setColumnStretch(3, 1)
        self.st_comms = QLabel("Disconnected")
        self.st_homed = QLabel("---")
        self.st_tmc = QLabel("---")
        self.st_lir_pos = QLabel("---")
        self.st_mt_pos = QLabel("---")
        self.st_mt_raw = QLabel("---")
        self.st_mt_zero = QLabel("---")
        r = 0
        lay.addWidget(QLabel("Comms:"), r, 0)
        lay.addWidget(self.st_comms, r, 1)
        lay.addWidget(QLabel("LIR:"), r, 2)
        lay.addWidget(self.st_lir_pos, r, 3)
        r += 1
        lay.addWidget(QLabel("Motor:"), r, 0)
        lay.addWidget(self.st_tmc, r, 1)
        lay.addWidget(QLabel("MT6835:"), r, 2)
        lay.addWidget(self.st_mt_pos, r, 3)
        r += 1
        lay.addWidget(QLabel("Homed:"), r, 0)
        lay.addWidget(self.st_homed, r, 1)
        lay.addWidget(QLabel("MT RAW:"), r, 2)
        lay.addWidget(self.st_mt_raw, r, 3)
        r += 1
        lay.addWidget(QLabel(""), r, 0)
        lay.addWidget(QLabel(""), r, 1)
        lay.addWidget(QLabel("ZERO_POS:"), r, 2)
        lay.addWidget(self.st_mt_zero, r, 3)
        return g

    def _build_motor(self):
        g = QGroupBox("Motor")
        lay = QGridLayout(g)
        lay.setSpacing(3)
        self.stop_btn = _btn("STOP", self._on_stop)
        self.home_btn = _btn("Home", self._on_home)
        self.speed_combo = _combo(
            [
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
            ],
            "50 RPM",
        )
        self.speed_combo.currentTextChanged.connect(self._on_speed_changed)
        self.refresh_btn = _btn("Refresh", self._request_status)
        lay.addWidget(self.stop_btn, 0, 0)
        lay.addWidget(self.home_btn, 0, 1)
        lay.addWidget(self.speed_combo, 0, 2)
        lay.addWidget(self.refresh_btn, 0, 3)

        self.move_combo = QComboBox()
        self.move_combo.setMinimumWidth(240)
        self.move_combo.setStyleSheet(
            "QComboBox { text-align: center; }"
            "QComboBox QAbstractItemView { text-align: center; }"
        )
        for label, _ in MOVE_PRESETS:
            self.move_combo.addItem(label)
        for i in range(self.move_combo.count()):
            self.move_combo.setItemData(i, Qt.AlignCenter, Qt.TextAlignmentRole)
        self.move_combo.setCurrentIndex(13)
        self.move_cw_btn = _btn("CW", self._on_move_cw)
        self.move_ccw_btn = _btn("CCW", self._on_move_ccw)
        lay.addWidget(self.move_combo, 1, 0, 1, 2)
        lay.addWidget(self.move_cw_btn, 1, 2)
        lay.addWidget(self.move_ccw_btn, 1, 3)
        return g

    def _build_mt6835(self):
        g = QGroupBox("MT6835")
        lay = QGridLayout(g)
        lay.setSpacing(3)

        self.set_zero_btn = _btn("Set Zero Pos", self._on_set_zero)
        self.ucal_rpm = _combo(
            [
                "25 RPM",
                "30 RPM",
                "40 RPM",
                "50 RPM",
                "60 RPM",
                "75 RPM",
                "100 RPM",
                "150 RPM",
                "200 RPM",
            ],
            "50 RPM",
        )
        self.ucal_btn = _btn("Auto-Cal", self._on_user_cal)
        self.prog_btn = _btn("Program EEPROM", self._on_program_eeprom)
        lay.addWidget(self.set_zero_btn, 0, 0)
        lay.addWidget(self.ucal_rpm, 0, 1)
        lay.addWidget(self.ucal_btn, 0, 2)
        lay.addWidget(self.prog_btn, 0, 3)

        self.hyst_combo = _combo(HYST_LABELS, HYST_LABELS[7])
        self.hyst_combo.currentIndexChanged.connect(self._on_hyst_changed)
        self.bw_combo = _combo(BW_LABELS, BW_LABELS[5])
        self.bw_combo.currentIndexChanged.connect(self._on_bw_changed)
        self.nlc_upload_btn = _btn("Upload NLC", self._on_nlc_upload)
        self.nlc_toggle_btn = _btn("Enable NLC", self._on_nlc_toggle)
        lay.addWidget(self.hyst_combo, 1, 0)
        lay.addWidget(self.bw_combo, 1, 1)
        lay.addWidget(self.nlc_upload_btn, 1, 2)
        lay.addWidget(self.nlc_toggle_btn, 1, 3)

        self.nlc_clear_btn = _btn("Clear NLC", self._on_nlc_clear)
        lay.addWidget(self.nlc_clear_btn, 2, 0)
        return g

    def _build_analysis(self):
        g = QGroupBox("Analysis")
        lay = QGridLayout(g)
        lay.setSpacing(3)
        self.collect_pts = _combo(
            [
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
            ],
            "256 pts",
        )
        self.collect_btn = _btn("Collect Data", self._on_collect)
        self.gen_nlc_btn = _btn("Generate NLC", self._on_generate_nlc)
        self.viz_btn = _btn("Visualization", self._on_visualize)
        lay.addWidget(self.collect_pts, 0, 0)
        lay.addWidget(self.collect_btn, 0, 1)
        lay.addWidget(self.gen_nlc_btn, 0, 2)
        lay.addWidget(self.viz_btn, 0, 3)
        return g

    def _build_console(self):
        g = QGroupBox("Console")
        lay = QVBoxLayout(g)
        self.console = QPlainTextEdit()
        self.console.setReadOnly(True)
        self.console.setMaximumBlockCount(500)
        self.console.setMinimumHeight(200)
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
        self.cmd_input.setPlaceholderText("Type command and press Enter...")
        self.cmd_input.returnPressed.connect(self._on_cmd_entered)
        lay.addWidget(self.cmd_input)
        return g

    # ----------------------------------------------------------------
    # Connection
    # ----------------------------------------------------------------

    def _refresh_com_ports(self):
        self.com_port_combo.clear()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.com_port_combo.addItems(ports if ports else ["No ports found"])

    def _log(self, msg):
        self.console.appendPlainText(msg)

    def _set_controls_enabled(self, on):
        self.connect_btn.setEnabled(not on)
        self.disconnect_btn.setEnabled(on)
        self.com_port_combo.setEnabled(not on)
        for w in [
            self.stop_btn,
            self.home_btn,
            self.refresh_btn,
            self.speed_combo,
            self.move_combo,
            self.move_cw_btn,
            self.move_ccw_btn,
            self.set_zero_btn,
            self.ucal_btn,
            self.ucal_rpm,
            self.prog_btn,
            self.hyst_combo,
            self.bw_combo,
            self.nlc_upload_btn,
            self.nlc_toggle_btn,
            self.nlc_clear_btn,
            self.collect_pts,
            self.collect_btn,
            self.gen_nlc_btn,
            self.viz_btn,
            self.cmd_input,
        ]:
            w.setEnabled(on)

    def _reset_status(self):
        self.homed = self.tmc_enabled = self.nlc_enabled = False
        for w in [
            self.st_tmc,
            self.st_lir_pos,
            self.st_mt_pos,
            self.st_mt_raw,
            self.st_mt_zero,
            self.st_homed,
        ]:
            w.setText("---")
        self.nlc_toggle_btn.setText("Enable NLC")

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
        self._log(f"Serial error: {msg}")
        self._on_disconnect()

    # ----------------------------------------------------------------
    # Serial I/O
    # ----------------------------------------------------------------

    def _send(self, cmd):
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

    def _delayed_status(self, ms=500):
        QTimer.singleShot(ms, self._request_status)

    def _on_cmd_entered(self):
        cmd = self.cmd_input.text().strip()
        if cmd:
            self.command_history.append(cmd)
            self.history_index = len(self.command_history)
            self._send(cmd)
            self.cmd_input.clear()

    # ----------------------------------------------------------------
    # Motor actions
    # ----------------------------------------------------------------

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
        idx = self.move_combo.currentIndex()
        return MOVE_PRESETS[idx][1] if 0 <= idx < len(MOVE_PRESETS) else 0

    def _on_move_cw(self):
        self._send(f"SET_SPEED {_get_int(self.speed_combo)}")
        time.sleep(0.05)
        us = self._get_move_usteps()
        if us == 0:
            self._send("MOVE_CW")
        else:
            self._send(f"SET_STEPS {us}")
            time.sleep(0.02)
            self._send("MOVE_CW_STEPS")
            self._delayed_status(1000)

    def _on_move_ccw(self):
        self._send(f"SET_SPEED {_get_int(self.speed_combo)}")
        time.sleep(0.05)
        us = self._get_move_usteps()
        if us == 0:
            self._send("MOVE_CCW")
        else:
            self._send(f"SET_STEPS {us}")
            time.sleep(0.02)
            self._send("MOVE_CCW_STEPS")
            self._delayed_status(1000)

    # ----------------------------------------------------------------
    # MT6835 actions
    # ----------------------------------------------------------------

    def _on_set_zero(self):
        if (
            QMessageBox.question(
                self,
                "Set ZERO_POS",
                "Set current position as MT6835 zero?\n\n"
                "Invalidates existing NLC calibration.",
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

    def _on_program_eeprom(self):
        if (
            QMessageBox.question(
                self,
                "Program EEPROM",
                "Write all registers to EEPROM?\nTakes ~6 seconds.",
                QMessageBox.Yes | QMessageBox.No,
            )
            == QMessageBox.Yes
        ):
            self._send("MT6835_PROGRAM_EEPROM")
            self._delayed_status(8000)

    def _on_nlc_toggle(self):
        if self.nlc_enabled:
            self._send("MT6835_DISABLE_NLC")
        else:
            self._send("MT6835_ENABLE_NLC")
        self._delayed_status()

    def _on_nlc_upload(self):
        if not self.calibration_manager:
            return
        variants = getattr(self.calibration_manager, "all_nlc_variants", {})
        memory_names = [f"[memory] {k}" for k in sorted(variants.keys())]
        hex_files = self.calibration_manager.get_available_nlc_hex_files()
        choices = memory_names + hex_files
        if not choices:
            self._log("No NLC tables found in data/nlc/.")
            return
        choice, ok = QInputDialog.getItem(
            self, "Upload NLC", "Select table:", choices, 0, False
        )
        if not ok or not choice:
            return
        try:
            if choice.startswith("[memory] "):
                hex_data = variants[choice.replace("[memory] ", "")]
            else:
                hex_data = self.calibration_manager.load_nlc_hex_file(choice)
            if len(hex_data) != 384:
                self._log(f"Error: expected 384 hex chars, got {len(hex_data)}")
                return
            self._log(f"Uploading {choice}...")
            self.calibration_manager.latest_hex_table = hex_data
            self._send(f"LOAD_NLC {hex_data}")
            self._delayed_status(8000)
        except Exception as e:
            self._log(f"Upload error: {e}")

    def _on_nlc_clear(self):
        if (
            QMessageBox.question(
                self,
                "Clear NLC",
                "Erase NLC table and program EEPROM?",
                QMessageBox.Yes | QMessageBox.No,
            )
            == QMessageBox.Yes
        ):
            self._send("MT6835_CLEAR_NLC")
            self._delayed_status(8000)

    # ----------------------------------------------------------------
    # Analysis
    # ----------------------------------------------------------------

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
        # Progress shown in console messages, no progress bar needed.
        pass

    def _cal_finished(self, success, cal_type):
        self._delayed_status()

    def _on_visualize(self):
        self.visualization_window = VisualizationWindow()
        self.visualization_window.exec_()

    # ----------------------------------------------------------------
    # Incoming serial data
    # ----------------------------------------------------------------

    def _handle_serial_data(self, text):
        text = text.strip()
        if not text:
            return

        # NLC verify readback.
        if text.startswith("NLC_DUMP:"):
            dump = text.split(":", 1)[1].strip()
            if self.calibration_manager and self.calibration_manager.latest_hex_table:
                match = dump == self.calibration_manager.latest_hex_table
                self._log(f"NLC verify: {'MATCH' if match else 'MISMATCH'}")
            return

        # NLC chip dump.
        if text.startswith("NLC_RAW_DUMP:"):
            dump = text.split(":", 1)[1].strip()
            nz = sum(1 for i in range(0, len(dump), 2) if dump[i : i + 2] != "00")
            self._log(f"NLC chip: {'empty' if nz == 0 else f'{nz} non-zero bytes'}")
            return

        # Register dump.
        if text.startswith("MT_REG_DUMP:"):
            self._decode_register_dump(text.split(":", 1)[1].strip())
            return

        # ZERO_POS response (from MT6835_READ_ZERO).
        if text.startswith("OK:ZERO_POS="):
            try:
                zp = int(text.split("=")[1].split()[0])
                if self.calibration_manager:
                    self.calibration_manager.handle_zero_pos_response(zp)
            except Exception:
                pass
            return

        # NLC enable/disable confirmation — update toggle button.
        if "NLC enabled" in text or "NLC disabled" in text:
            self.nlc_enabled = "enabled" in text
            self.nlc_toggle_btn.setText(
                "Disable NLC" if self.nlc_enabled else "Enable NLC"
            )
            self._log(text)
            return

        # NLC programmed — mark as enabled.
        if text.startswith("OK:NLC programmed"):
            self.nlc_enabled = True
            self.nlc_toggle_btn.setText("Disable NLC")
            self._log(text)
            return

        # Homed or zero-set confirmation — refresh status.
        if text.startswith("OK:Homed") or text.startswith("OK:MT6835 zero"):
            self._log(text)
            self._delayed_status()
            return

        # Structured status telemetry.
        if "STATUS_" in text:
            self._parse_status(text)
            return

        # Suppress OK responses from auto-commands during calibration.
        if (
            text.startswith("OK:")
            and self.hide_status_messages
            and self.calibration_manager
            and self.calibration_manager.calibration_in_progress
        ):
            return

        # Everything else goes to console.
        self._log(text)

    def _decode_register_dump(self, raw):
        """Parse register dump into human-readable lines."""
        regs = {}
        for pair in raw.split(","):
            if "=" in pair:
                a, v = pair.strip().split("=", 1)
                try:
                    regs[int(a, 16)] = int(v, 16)
                except ValueError:
                    continue

        self._log("MT6835 registers:")
        for addr in sorted(regs.keys()):
            val = regs[addr]
            info = ""
            if addr == 0x009:
                zp = (val << 4) | ((regs.get(0x00A, 0) >> 4) & 0x0F)
                info = f"ZERO_POS={zp} ({zp * 360.0 / 4096:.3f}°)"
            elif addr == 0x00C:
                info = f"NLC_EN={(val >> 5) & 1}"
            elif addr == 0x00D:
                info = f"HYST={HYST_DECODE.get(val & 0x07, '?')}"
            elif addr == 0x00E:
                info = f"AUTOCAL_FREQ={(val >> 4) & 0x07}"
            elif addr == 0x011:
                info = f"BW={BW_DECODE.get(val & 0x07, '?')}"
            line = f"  0x{addr:03X}=0x{val:02X}"
            if info:
                line += f"  {info}"
            self._log(line)

    def _parse_status(self, text):
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
                    self.st_mt_pos.setText(f"{mt_deg:.6f}°")
                elif key == "STATUS_MT6835_RAW":
                    raw_deg = (int(val) / self.MT6835_COUNTS) * 360.0
                    self.st_mt_raw.setText(f"{raw_deg:.6f}°")
                elif key == "STATUS_MT6835_ZERO_POS":
                    zp = int(val)
                    self.st_mt_zero.setText(f"{zp * 360.0 / 4096:.3f}° ({zp})")
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

    # ----------------------------------------------------------------
    # Keyboard
    # ----------------------------------------------------------------

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Up and self.history_index > 0:
            self.history_index -= 1
            self.cmd_input.setText(self.command_history[self.history_index])
        elif event.key() == Qt.Key_Down:
            if self.history_index < len(self.command_history) - 1:
                self.history_index += 1
                self.cmd_input.setText(self.command_history[self.history_index])
            else:
                self.history_index = len(self.command_history)
                self.cmd_input.clear()
        else:
            super().keyPressEvent(event)


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = EncoderEvaluationGUI()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
