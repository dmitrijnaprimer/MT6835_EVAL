"""Encoder linearity visualization.

Side-panel with file selector and statistics. Plot shows de-meaned error
(linearity only, DC offset removed) for one or more datasets overlaid.
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

COLORS = [
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


def _dms(deg):
    """Degrees to human-readable DMS string."""
    s = "-" if deg < 0 else ""
    deg = abs(deg)
    d, rem = int(deg), (deg - int(deg)) * 60
    m, sec = int(rem), (rem - int(rem)) * 60
    if d:
        return f"{s}{d}d {m:02d}' {sec:05.2f}\""
    if m:
        return f"{s}{m}' {sec:05.2f}\""
    return f'{s}{sec:.2f}"'


def _dms_tick(deg):
    """Short DMS for axis ticks."""
    s = "-" if deg < 0 else ""
    deg = abs(deg)
    if deg < 1 / 3600:
        return f'{s}{deg * 3600:.2f}"'
    if deg < 1 / 60:
        return f'{s}{deg * 3600:.1f}"'
    if deg < 1:
        m = int(deg * 60)
        return f"{s}{m}'{(deg * 60 - m) * 60:.0f}\""
    d = int(deg)
    return f"{s}{d}d{int((deg - d) * 60):02d}'"


class VisualizationWindow(QDialog):
    """Error plot dialog — always de-meaned, multi-file overlay."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Linearity Assessment")
        self.setGeometry(50, 50, 1400, 900)

        root = QHBoxLayout(self)
        root.setContentsMargins(4, 4, 4, 4)
        sp = QSplitter(Qt.Horizontal)
        root.addWidget(sp)

        # -- Left panel --
        left = QWidget()
        ll = QVBoxLayout(left)
        ll.setContentsMargins(0, 0, 0, 0)
        ll.setSpacing(4)

        fg = QGroupBox("Data Files")
        fl = QVBoxLayout(fg)
        row = QHBoxLayout()
        self._refresh_btn = QPushButton("Refresh")
        self._refresh_btn.clicked.connect(self._refresh)
        row.addWidget(self._refresh_btn)
        row.addStretch()
        fl.addLayout(row)
        self._file_list = QListWidget()
        self._refresh()
        self._file_list.itemChanged.connect(lambda _: self._schedule())
        fl.addWidget(self._file_list)
        ll.addWidget(fg, 1)

        sg = QGroupBox("Statistics")
        sl = QVBoxLayout(sg)
        self._stats = QLabel("Select files to plot.")
        self._stats.setStyleSheet(
            "font-family: 'Consolas', monospace; font-size: 10px;"
        )
        self._stats.setWordWrap(True)
        sl.addWidget(self._stats)
        ll.addWidget(sg)
        sp.addWidget(left)

        # -- Right panel --
        right = QWidget()
        rl = QVBoxLayout(right)
        rl.setContentsMargins(0, 0, 0, 0)
        self._fig = Figure(figsize=(12, 7), dpi=100)
        self._canvas = FigureCanvas(self._fig)
        self._toolbar = NavigationToolbar(self._canvas, self)
        rl.addWidget(self._toolbar)
        rl.addWidget(self._canvas, 1)
        sp.addWidget(right)
        sp.setSizes([280, 1120])

        self._timer = QTimer()
        self._timer.setSingleShot(True)
        self._timer.timeout.connect(self._replot)

    # ---- File management ----

    def _schedule(self):
        self._timer.start(200)

    def _refresh(self):
        self._file_list.blockSignals(True)
        self._file_list.clear()
        files = []
        for d in (DATA_DIR, NLC_DIR):
            files.extend(glob.glob(os.path.join(d, "*.csv")))
        files.sort(key=os.path.getmtime, reverse=True)
        for f in files:
            item = QListWidgetItem(os.path.relpath(f, DATA_DIR))
            item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
            item.setCheckState(Qt.Unchecked)
            self._file_list.addItem(item)
        if not files:
            item = QListWidgetItem("No CSV files found")
            item.setFlags(item.flags() & ~Qt.ItemIsUserCheckable)
            self._file_list.addItem(item)
        self._file_list.blockSignals(False)

    def _checked(self):
        return [
            self._file_list.item(i).text()
            for i in range(self._file_list.count())
            if self._file_list.item(i).checkState() == Qt.Checked
        ]

    # ---- Plot ----

    def _replot(self):
        names = self._checked()
        if not names:
            self._fig.clear()
            self._canvas.draw()
            self._stats.setText("Select files to plot.")
            return

        # Load and compute de-meaned error.
        sets = []
        for n in names:
            try:
                ref, meas = self._load(os.path.join(DATA_DIR, n))
                if len(ref) < 2:
                    continue
                err = meas - ref
                err[err > 180] -= 360
                err[err < -180] += 360
                err -= np.mean(err)  # Always de-mean.
                sets.append((n, ref, err))
            except Exception as e:
                self._stats.setText(f"Error: {e}")
                return
        if not sets:
            return

        # Statistics.
        lines = []
        for n, _, e in sets:
            p = np.ptp(e)
            rms = np.sqrt(np.mean(e**2))
            lines.append(
                f"[{os.path.basename(n)}]\n  P2P={p:.5f}° ({_dms(p)})  RMS={rms:.5f}°"
            )
        self._stats.setText("\n".join(lines))

        # Draw.
        self._fig.clear()
        ax = self._fig.add_subplot(111)
        ax.axhline(0, color="k", lw=0.5, alpha=0.5)
        mx = 360

        for i, (n, ref, err) in enumerate(sets):
            c = COLORS[i % len(COLORS)]
            short = os.path.basename(n)
            if len(short) > 40:
                short = short[:37] + "…"
            ax.plot(
                ref,
                err,
                ".-",
                color=c,
                lw=0.8,
                ms=2,
                alpha=0.7,
                label=f"{short}  P2P={np.ptp(err):.5f}°",
            )
            if len(ref) and max(ref) > mx:
                mx = max(ref)

        ax.set_xlabel("LIR reference [°]")
        ax.set_ylabel("Error (de-meaned) [°]")

        if len(sets) == 1:
            p = np.ptp(sets[0][2])
            rms = np.sqrt(np.mean(sets[0][2] ** 2))
            ax.set_title(
                f"{os.path.basename(sets[0][0])}\n"
                f"P2P: {p:.5f}° = {_dms(p)}  |  RMS: {rms:.5f}°"
            )
        else:
            ax.set_title(f"Comparison: {len(sets)} datasets")

        ax.legend(fontsize=7, loc="upper right")
        ax.grid(True, ls=":", alpha=0.5)
        ax.set_xlim(0, mx)
        self._dms_axis(ax)
        self._fig.tight_layout()
        self._canvas.draw()

    # ---- Helpers ----

    @staticmethod
    def _load(path):
        """Load two-column CSV, skip comments and header."""
        lir: List[float] = []
        mt: List[float] = []
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line or line[0] in "#L":
                    continue
                p = line.split(",")
                if len(p) >= 2:
                    try:
                        lir.append(float(p[0]))
                        mt.append(float(p[1]))
                    except ValueError:
                        pass
        return np.array(lir), np.array(mt)

    @staticmethod
    def _dms_axis(ax):
        """Add secondary Y-axis with DMS tick labels."""
        ax.yaxis.set_major_formatter(
            FuncFormatter(
                lambda x, _: (
                    f"{x:.5f}"
                    if abs(x) < 0.001
                    else f"{x:.4f}"
                    if abs(x) < 0.1
                    else f"{x:.3f}"
                )
            )
        )
        ax2 = ax.twinx()
        ax2.set_ylim(ax.get_ylim())
        ax2.yaxis.set_major_formatter(FuncFormatter(lambda x, _: _dms_tick(x)))
        ax2.set_ylabel("[DMS]", fontsize=9)
