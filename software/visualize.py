"""
Visualization window for encoder linearity assessment.

Side-panel layout: file list and options on the left, plots and stats
on the right. Error plot plus optional NLC correction overlay showing
the correction values and where they saturate.
"""

import csv
import glob
import hashlib
import os
from typing import List

import numpy as np

"""Data directory: one level up from the control/ folder."""
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(os.path.dirname(_SCRIPT_DIR), "data")
NLC_DIR = os.path.join(DATA_DIR, "nlc")
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
from matplotlib.ticker import FuncFormatter
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QCheckBox,
    QDialog,
    QDoubleSpinBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QPushButton,
    QSplitter,
    QVBoxLayout,
    QWidget,
)


PLOT_COLORS = [
    "#1f77b4",
    "#d62728",
    "#2ca02c",
    "#ff7f0e",
    "#9467bd",
    "#8c564b",
    "#e377c2",
    "#7f7f7f",
    "#bcbd22",
    "#17becf",
]


def _color_for_filename(filename: str) -> str:
    h = int(hashlib.md5(filename.encode()).hexdigest(), 16)
    return PLOT_COLORS[h % len(PLOT_COLORS)]


def deg_to_dms_str(deg: float) -> str:
    sign = "-" if deg < 0 else ""
    deg = abs(deg)
    d = int(deg)
    remainder = (deg - d) * 60
    m = int(remainder)
    s = (remainder - m) * 60
    if d > 0:
        return f"{sign}{d}d {m:02d}' {s:05.2f}\""
    elif m > 0:
        return f"{sign}{m}' {s:05.2f}\""
    else:
        return f'{sign}{s:.2f}"'


def deg_to_dms_tick(deg: float) -> str:
    sign = "-" if deg < 0 else ""
    deg = abs(deg)
    if deg < 1 / 3600:
        return f'{sign}{deg * 3600:.2f}"'
    elif deg < 1 / 60:
        return f'{sign}{deg * 3600:.1f}"'
    elif deg < 1:
        m = int(deg * 60)
        s = (deg * 60 - m) * 60
        return f"{sign}{m}'{s:.0f}\""
    else:
        d = int(deg)
        m = int((deg - d) * 60)
        return f"{sign}{d}d{m:02d}'"


class VisualizationWindow(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MT6835 Non-Linearity Assessment")
        self.setGeometry(50, 50, 1400, 900)

        root_layout = QHBoxLayout(self)
        root_layout.setContentsMargins(4, 4, 4, 4)
        splitter = QSplitter(Qt.Horizontal)
        root_layout.addWidget(splitter)

        # Left panel
        left = QWidget()
        left_lay = QVBoxLayout(left)
        left_lay.setContentsMargins(0, 0, 0, 0)
        left_lay.setSpacing(4)

        file_group = QGroupBox("Data Files")
        file_lay = QVBoxLayout(file_group)
        btn_row = QHBoxLayout()
        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.clicked.connect(self.refresh_sources)
        btn_row.addWidget(self.refresh_btn)
        btn_row.addStretch()
        file_lay.addLayout(btn_row)
        self.file_list = QListWidget()
        self.refresh_sources()
        self.file_list.itemChanged.connect(self._on_check_changed)
        file_lay.addWidget(self.file_list)
        left_lay.addWidget(file_group, 1)

        opt_group = QGroupBox("Options")
        opt_lay = QVBoxLayout(opt_group)
        self.demean_cb = QCheckBox("De-mean (linearity only)")
        self.demean_cb.setChecked(False)
        self.demean_cb.stateChanged.connect(lambda _: self._trigger_replot())
        opt_lay.addWidget(self.demean_cb)

        self.show_nlc_cb = QCheckBox("Show NLC correction overlay")
        self.show_nlc_cb.setChecked(False)
        self.show_nlc_cb.stateChanged.connect(lambda _: self._trigger_replot())
        opt_lay.addWidget(self.show_nlc_cb)

        lsb_row = QHBoxLayout()
        lsb_row.addWidget(QLabel("NLC LSB:"))
        self.nlc_lsb_spin = QDoubleSpinBox()
        self.nlc_lsb_spin.setRange(0.0005, 0.01)
        self.nlc_lsb_spin.setDecimals(5)
        self.nlc_lsb_spin.setSingleStep(0.0001)
        self.nlc_lsb_spin.setValue(360.0 / (1 << 18))
        self.nlc_lsb_spin.setSuffix("\u00b0")
        self.nlc_lsb_spin.valueChanged.connect(lambda _: self._trigger_replot())
        lsb_row.addWidget(self.nlc_lsb_spin)
        opt_lay.addLayout(lsb_row)

        left_lay.addWidget(opt_group)

        stats_group = QGroupBox("Statistics")
        stats_lay = QVBoxLayout(stats_group)
        self.stats_label = QLabel("Check CSV files to plot.")
        self.stats_label.setStyleSheet(
            "font-family: 'Consolas','Courier New',monospace; font-size: 10px;"
        )
        self.stats_label.setWordWrap(True)
        stats_lay.addWidget(self.stats_label)
        left_lay.addWidget(stats_group)

        splitter.addWidget(left)

        # Right panel
        right = QWidget()
        right_lay = QVBoxLayout(right)
        right_lay.setContentsMargins(0, 0, 0, 0)
        self.figure = Figure(figsize=(12, 7), dpi=100)
        self.canvas = FigureCanvas(self.figure)
        self.toolbar = NavigationToolbar(self.canvas, self)
        right_lay.addWidget(self.toolbar)
        right_lay.addWidget(self.canvas, 1)
        splitter.addWidget(right)

        splitter.setSizes([300, 1100])

        self.replot_timer = QTimer()
        self.replot_timer.setSingleShot(True)
        self.replot_timer.timeout.connect(self._replot_checked)

    def _trigger_replot(self):
        self.replot_timer.start(100)

    def refresh_sources(self):
        self.file_list.blockSignals(True)
        self.file_list.clear()
        search_paths = [
            DATA_DIR,
            NLC_DIR,
        ]
        all_files = []
        for sp in search_paths:
            for f in glob.glob(os.path.join(sp, "*.csv")):
                all_files.append(f)
        all_files.sort(key=os.path.getmtime, reverse=True)

        data_root = DATA_DIR
        for f in all_files:
            rel = os.path.relpath(f, data_root)
            item = QListWidgetItem(rel)
            item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
            item.setCheckState(Qt.Unchecked)
            self.file_list.addItem(item)

        if not all_files:
            item = QListWidgetItem("No CSV files found")
            item.setFlags(item.flags() & ~Qt.ItemIsUserCheckable)
            self.file_list.addItem(item)
        self.file_list.blockSignals(False)

    def _on_check_changed(self, item):
        self.replot_timer.start(300)

    def _get_checked_files(self):
        checked = []
        for i in range(self.file_list.count()):
            item = self.file_list.item(i)
            if item.checkState() == Qt.Checked:
                checked.append(item.text())
        return checked

    def _replot_checked(self):
        filenames = self._get_checked_files()
        if not filenames:
            self.figure.clear()
            self.canvas.draw()
            self.stats_label.setText("Check CSV files to plot.")
            return

        data_root = DATA_DIR
        datasets = []
        for fn in filenames:
            filepath = os.path.join(data_root, fn)
            try:
                ref, meas = self._load_csv(filepath)
                if len(ref) < 2:
                    continue
                error = self._compute_error(ref, meas)
                datasets.append((fn, ref, error))
            except Exception as e:
                self.stats_label.setText(f"Error loading {fn}: {e}")
                return

        if not datasets:
            return

        demean = self.demean_cb.isChecked()
        show_nlc = self.show_nlc_cb.isChecked()

        if demean:
            datasets = [(fn, ref, err - np.mean(err)) for fn, ref, err in datasets]

        # Stats
        stats_lines = []
        for fn, ref, error in datasets:
            stats_lines.append(self._format_stats(error, os.path.basename(fn)))
        self.stats_label.setText("\n".join(stats_lines))

        # Plot
        self.figure.clear()
        if show_nlc:
            ax_err = self.figure.add_subplot(211)
            ax_nlc = self.figure.add_subplot(212, sharex=ax_err)
        else:
            ax_err = self.figure.add_subplot(111)
            ax_nlc = None

        ax_err.axhline(0, color="black", linestyle="-", linewidth=0.5, alpha=0.5)

        max_ref = 360
        for fn, ref, error in datasets:
            color = _color_for_filename(os.path.basename(fn))
            p2p = np.ptp(error)
            short = os.path.basename(fn)
            if len(short) > 40:
                short = short[:37] + "..."
            label = f"{short}  P2P={p2p:.5f}\u00b0"
            ax_err.plot(
                ref,
                error,
                ".-",
                color=color,
                linewidth=0.8,
                markersize=2,
                alpha=0.7,
                label=label,
            )
            if len(ref) > 0 and max(ref) > max_ref:
                max_ref = max(ref)

        ylabel = "Error (de-meaned) [deg]" if demean else "Error [deg]"
        ax_err.set_ylabel(ylabel, fontsize=10)
        if ax_nlc is None:
            ax_err.set_xlabel("LIR-DA237T Reference [deg]", fontsize=10)

        if len(datasets) == 1:
            fn, ref, error = datasets[0]
            p2p = np.ptp(error)
            rms = np.sqrt(np.mean(error**2))
            ax_err.set_title(
                f"{os.path.basename(fn)}\n"
                f"P2P: {p2p:.5f}\u00b0 = {deg_to_dms_str(p2p)}   |   "
                f"RMS: {rms:.5f}\u00b0 = {deg_to_dms_str(rms)}",
                fontsize=11,
            )
        else:
            ax_err.set_title(f"Comparison: {len(datasets)} datasets", fontsize=11)

        ax_err.legend(fontsize=7, loc="upper right")
        ax_err.grid(True, linestyle=":", alpha=0.5)
        ax_err.set_xlim(0, max_ref)
        self._add_dms_axis(ax_err)

        # NLC correction overlay
        if ax_nlc is not None:
            nlc_lsb = self.nlc_lsb_spin.value()
            nlc_grid = np.linspace(0, 360.0, 256, endpoint=False)

            for fn, ref, error in datasets:
                color = _color_for_filename(os.path.basename(fn))
                short = os.path.basename(fn)
                if len(short) > 30:
                    short = short[:27] + "..."

                # De-mean for NLC computation (chip removes DC)
                err_ac = error - np.mean(error)

                # Interpolate error onto NLC 256-point grid
                ref_ext = np.concatenate([ref - 360, ref, ref + 360])
                err_ext = np.concatenate([err_ac, err_ac, err_ac])
                sort_idx = np.argsort(ref_ext)
                err_on_grid = np.interp(nlc_grid, ref_ext[sort_idx], err_ext[sort_idx])

                # Compute ideal (unclipped) and actual (clipped) correction
                ideal_lsb = -err_on_grid / nlc_lsb
                clipped_lsb = np.clip(np.round(ideal_lsb).astype(int), -32, 31)
                n_sat = np.sum((clipped_lsb == -32) | (clipped_lsb == 31))

                ax_nlc.plot(
                    nlc_grid,
                    ideal_lsb,
                    "-",
                    color=color,
                    linewidth=0.6,
                    alpha=0.4,
                    label=f"{short} ideal",
                )
                ax_nlc.step(
                    nlc_grid,
                    clipped_lsb,
                    "-",
                    color=color,
                    linewidth=1.0,
                    alpha=0.8,
                    where="mid",
                    label=f"{short} clipped (sat={n_sat}/256)",
                )

            # Saturation limits
            ax_nlc.axhline(31, color="red", linestyle="--", linewidth=0.7, alpha=0.5)
            ax_nlc.axhline(-32, color="red", linestyle="--", linewidth=0.7, alpha=0.5)
            ax_nlc.fill_between([0, max_ref], 31, 40, color="red", alpha=0.05)
            ax_nlc.fill_between([0, max_ref], -32, -40, color="red", alpha=0.05)

            ax_nlc.set_xlabel("Reference Angle [deg]", fontsize=10)
            ax_nlc.set_ylabel("NLC correction [LSBs]", fontsize=10)
            ax_nlc.set_title(
                f"NLC correction at LSB={nlc_lsb:.5f}\u00b0  "
                f"(range: \u00b1{32 * nlc_lsb:.4f}\u00b0)",
                fontsize=10,
            )
            ax_nlc.legend(fontsize=7)
            ax_nlc.grid(True, linestyle=":", alpha=0.5)
            ax_nlc.set_ylim(-40, 40)

        self.figure.tight_layout()
        self.canvas.draw()

    def _load_csv(self, filepath):
        lir_values: List[float] = []
        mt_values: List[float] = []
        with open(filepath, "r") as f:
            reader = csv.reader(f)
            next(reader)
            for row in reader:
                if len(row) >= 2:
                    try:
                        lir_values.append(float(row[0]))
                        mt_values.append(float(row[1]))
                    except ValueError:
                        continue
        return np.array(lir_values), np.array(mt_values)

    def _compute_error(self, ref, meas):
        error = meas - ref
        error[error > 180] -= 360
        error[error < -180] += 360
        return error

    def _format_stats(self, error, label=""):
        p2p = np.ptp(error)
        rms = np.sqrt(np.mean(error**2))
        mean_err = np.mean(error)
        prefix = f"[{label}]\n" if label else ""
        return (
            f"{prefix}  N={len(error)}  "
            f"P2P={p2p:.5f}\u00b0 ({deg_to_dms_str(p2p)})\n"
            f"  RMS={rms:.5f}\u00b0  Mean={mean_err:+.5f}\u00b0\n"
        )

    def _add_dms_axis(self, ax):
        def fmt_deg(x, pos):
            if abs(x) < 0.001:
                return f"{x:.5f}"
            elif abs(x) < 0.1:
                return f"{x:.4f}"
            else:
                return f"{x:.3f}"

        ax.yaxis.set_major_formatter(FuncFormatter(fmt_deg))
        ax2 = ax.twinx()
        ax2.set_ylim(ax.get_ylim())
        ax2.yaxis.set_major_formatter(FuncFormatter(lambda x, p: deg_to_dms_tick(x)))
        ax2.set_ylabel("Error [DMS]", fontsize=10)
