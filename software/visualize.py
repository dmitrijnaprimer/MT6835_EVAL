"""
Visualization window for encoder linearity assessment.

Side-panel layout: file list and options on the left, plot and stats on the
right. When two datasets are checked, a second subplot shows their
point-by-point difference (useful for before/after NLC comparisons).
"""

import glob
import os
from typing import List

import numpy as np
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
from matplotlib.ticker import FuncFormatter
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QCheckBox,
    QDialog,
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

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(os.path.dirname(_SCRIPT_DIR), "data")
NLC_DIR = os.path.join(DATA_DIR, "nlc")

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


def deg_to_dms_str(deg: float) -> str:
    """Convert decimal degrees to a human-readable DMS string.

    :param deg: Angle in degrees.
    :return: String like ``12' 34.56"`` or ``5.67"``.
    """
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
    """Short DMS format suitable for axis tick labels.

    :param deg: Angle in degrees.
    :return: Compact string like ``3'12"`` or ``1d05'``.
    """
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
    """Modal dialog for plotting encoder error data from CSV files.

    Supports multi-file overlay, optional de-mean, and a difference subplot
    when exactly two datasets are selected.
    """

    def __init__(self):
        """Create the visualization window with side-panel layout."""
        super().__init__()
        self.setWindowTitle("MT6835 Non-Linearity Assessment")
        self.setGeometry(50, 50, 1400, 900)

        root_layout = QHBoxLayout(self)
        root_layout.setContentsMargins(4, 4, 4, 4)
        splitter = QSplitter(Qt.Horizontal)
        root_layout.addWidget(splitter)

        # ---- Left panel: file list + options + stats ----
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
        self.demean_cb = QCheckBox("De-mean (remove DC offset)")
        self.demean_cb.setChecked(False)
        self.demean_cb.stateChanged.connect(lambda _: self._trigger_replot())
        opt_lay.addWidget(self.demean_cb)
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

        # ---- Right panel: matplotlib plot ----
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
        """Schedule a debounced replot."""
        self.replot_timer.start(100)

    def refresh_sources(self):
        """Scan the data directory for CSV files and populate the file list."""
        self.file_list.blockSignals(True)
        self.file_list.clear()
        all_files = []
        for sp in (DATA_DIR, NLC_DIR):
            for f in glob.glob(os.path.join(sp, "*.csv")):
                all_files.append(f)
        all_files.sort(key=os.path.getmtime, reverse=True)

        for f in all_files:
            rel = os.path.relpath(f, DATA_DIR)
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
        """Checkbox toggled — schedule replot."""
        self.replot_timer.start(300)

    def _get_checked_files(self):
        """Return list of checked filenames (relative to DATA_DIR)."""
        checked = []
        for i in range(self.file_list.count()):
            item = self.file_list.item(i)
            if item.checkState() == Qt.Checked:
                checked.append(item.text())
        return checked

    def _replot_checked(self):
        """Load checked CSVs, compute errors, and draw the plot."""
        filenames = self._get_checked_files()
        if not filenames:
            self.figure.clear()
            self.canvas.draw()
            self.stats_label.setText("Check CSV files to plot.")
            return

        datasets = []
        for fn in filenames:
            filepath = os.path.join(DATA_DIR, fn)
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
        if demean:
            datasets = [(fn, ref, err - np.mean(err)) for fn, ref, err in datasets]

        # Build stats text.
        stats_lines = []
        for fn, ref, error in datasets:
            stats_lines.append(self._format_stats(error, os.path.basename(fn)))
        self.stats_label.setText("\n".join(stats_lines))

        # Plot.
        self.figure.clear()
        show_diff = len(datasets) == 2

        if show_diff:
            ax_err = self.figure.add_subplot(211)
            ax_diff = self.figure.add_subplot(212, sharex=ax_err)
        else:
            ax_err = self.figure.add_subplot(111)
            ax_diff = None

        ax_err.axhline(0, color="black", linestyle="-", linewidth=0.5, alpha=0.5)
        max_ref = 360

        for idx, (fn, ref, error) in enumerate(datasets):
            color = PLOT_COLORS[idx % len(PLOT_COLORS)]
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
        if ax_diff is None:
            ax_err.set_xlabel("LIR Reference [deg]", fontsize=10)

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
        elif show_diff:
            p2p_a = np.ptp(datasets[0][2])
            p2p_b = np.ptp(datasets[1][2])
            if p2p_a > 0:
                change = ((p2p_b - p2p_a) / p2p_a) * 100
                ax_err.set_title(
                    f"P2P: {p2p_a:.5f}\u00b0 \u2192 {p2p_b:.5f}\u00b0 ({change:+.1f}%)",
                    fontsize=11,
                )
            else:
                ax_err.set_title("Comparison", fontsize=11)
        else:
            ax_err.set_title(f"Comparison: {len(datasets)} datasets", fontsize=11)

        ax_err.legend(fontsize=7, loc="upper right")
        ax_err.grid(True, linestyle=":", alpha=0.5)
        ax_err.set_xlim(0, max_ref)
        self._add_dms_axis(ax_err)

        # Difference subplot (only for exactly 2 datasets).
        if ax_diff is not None:
            fn_a, ref_a, err_a = datasets[0]
            fn_b, ref_b, err_b = datasets[1]

            # Interpolate B onto A's reference grid.
            ref_b_ext = np.concatenate([ref_b - 360, ref_b, ref_b + 360])
            err_b_ext = np.concatenate([err_b, err_b, err_b])
            si = np.argsort(ref_b_ext)
            err_b_interp = np.interp(ref_a, ref_b_ext[si], err_b_ext[si])
            diff = err_b_interp - err_a

            ax_diff.axhline(0, color="black", linestyle="-", linewidth=0.5, alpha=0.5)
            ax_diff.plot(
                ref_a,
                diff,
                ".-",
                color="#7f7f7f",
                linewidth=0.8,
                markersize=2,
                alpha=0.8,
            )
            ax_diff.fill_between(ref_a, diff, alpha=0.15, color="#7f7f7f")

            ax_diff.set_xlabel("LIR Reference [deg]", fontsize=10)
            ax_diff.set_ylabel("Difference [deg]", fontsize=10)
            ax_diff.set_title(
                f"Difference (B \u2212 A)  "
                f"Mean={np.mean(diff):+.5f}\u00b0  "
                f"P2P={np.ptp(diff):.5f}\u00b0",
                fontsize=10,
            )
            ax_diff.grid(True, linestyle=":", alpha=0.5)
            self._add_dms_axis(ax_diff)

        self.figure.tight_layout()
        self.canvas.draw()

    # ================================================================
    # Helpers
    # ================================================================

    def _load_csv(self, filepath):
        """Load a two-column CSV file (LIR_deg, MT_deg).

        Skips comment lines (``#``) and the header row.

        :param filepath: Full path to CSV.
        :return: Tuple of (lir_array, mt_array) as numpy arrays.
        """
        lir_values: List[float] = []
        mt_values: List[float] = []
        with open(filepath, "r") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#") or line.startswith("LIR"):
                    continue
                parts = line.split(",")
                if len(parts) >= 2:
                    try:
                        lir_values.append(float(parts[0]))
                        mt_values.append(float(parts[1]))
                    except ValueError:
                        continue
        return np.array(lir_values), np.array(mt_values)

    @staticmethod
    def _compute_error(ref, meas):
        """Compute angle error with wrap-around handling.

        :param ref: Reference angles in degrees.
        :param meas: Measured angles in degrees.
        :return: Error array (meas - ref), wrapped to [-180, +180].
        """
        error = meas - ref
        error[error > 180] -= 360
        error[error < -180] += 360
        return error

    @staticmethod
    def _format_stats(error, label=""):
        """Format error statistics as a multi-line string.

        :param error: Error array in degrees.
        :param label: Optional label prefix.
        :return: Formatted string.
        """
        p2p = np.ptp(error)
        rms = np.sqrt(np.mean(error**2))
        mean_err = np.mean(error)
        prefix = f"[{label}]\n" if label else ""
        return (
            f"{prefix}  N={len(error)}  "
            f"P2P={p2p:.5f}\u00b0 ({deg_to_dms_str(p2p)})\n"
            f"  RMS={rms:.5f}\u00b0  Mean={mean_err:+.5f}\u00b0\n"
        )

    @staticmethod
    def _add_dms_axis(ax):
        """Add a secondary Y-axis with DMS-formatted tick labels.

        :param ax: Matplotlib axes to add the twin axis to.
        """

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
