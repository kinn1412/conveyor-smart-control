"""Tuning GUI for the conveyor actuator (spec §12).

python tools/gui.py
"""
import collections
import csv
import glob
import json
import os
import re
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import numpy as np
import pyqtgraph as pg
import serial.tools.list_ports
from PySide6.QtCore import QObject, Qt, QTimer, Signal
from PySide6.QtGui import QColor, QFont, QKeySequence, QShortcut
from PySide6.QtWidgets import (QApplication, QButtonGroup, QCheckBox, QComboBox, QDoubleSpinBox,
                               QFileDialog, QFormLayout, QGridLayout, QGroupBox, QHBoxLayout,
                               QInputDialog, QLabel, QListWidget, QListWidgetItem, QMainWindow,
                               QPlainTextEdit, QPushButton, QRadioButton, QScrollArea, QSpinBox,
                               QStackedWidget, QTabWidget, QVBoxLayout, QWidget)

import analyze
import proto as P
from host import CSV_HEADER, Link, LinkError, telem_row

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RUNS_DIR = os.path.join(ROOT, "runs")
PROFILES_DIR = os.path.join(ROOT, "tools", "profiles")
ACTIVE_FILE = os.path.join(PROFILES_DIR, "active.txt")
BLUE, RED, GREEN, GRAY, TEXT = "#1565C0", "#C62828", "#2E7D32", "#757575", "#212121"
# Run comparison needs distinct lines; blue first so a single run keeps the theme color
PALETTE = ["#1565C0", "#E65100", "#6A1B9A", "#00838F", "#5D4037", "#AD1457"]
TELEM_HZ = 100
BUF_S = 60  # seconds of telemetry kept for the plots
GAIN_KEYS = ("kp_v", "ki_v", "kp_p", "u_dz")
LIMIT_KEYS = ("v_max", "a_max")
NO_PROFILE = "(không)"
METRIC_NAMES = {"target_mms": "v_target [mm/s]", "t_r_s": "t_r (10-90 %) [s]",
                "t_s_s": "t_s (2 %) [s]", "POT_pct": "POT [%]", "e_ss_mms": "e_ss [mm/s]",
                "track_max_mm": "Tracking max [mm]", "track_rms_mm": "Tracking RMS [mm]",
                "final_err_mm": "Final error [mm]", "move_time_s": "Move time [s]"}

STYLE = f"""
QWidget {{ background: white; color: {TEXT}; font-family: "Segoe UI"; font-size: 10pt; }}
QGroupBox {{ border: 1px solid {BLUE}; border-radius: 4px; margin-top: 14px; padding: 10px 8px 8px 8px; }}
QGroupBox::title {{ subcontrol-origin: margin; left: 10px; padding: 0 4px; color: {BLUE}; font-weight: 600; }}
QPushButton {{ border: 1px solid {BLUE}; border-radius: 3px; padding: 4px 12px; color: {BLUE}; }}
QPushButton:hover {{ background: #E8F0FB; }}
QPushButton:disabled {{ border-color: #BDBDBD; color: #9E9E9E; }}
QPushButton[primary="true"] {{ background: {BLUE}; color: white; }}
QPushButton[primary="true"]:disabled {{ background: #BDBDBD; border-color: #BDBDBD; }}
QPushButton#estop {{ background: {RED}; border-color: {RED}; color: white; font-weight: 700; padding: 6px 20px; }}
QTabWidget::pane {{ border: 1px solid {BLUE}; top: -1px; }}
QTabBar::tab {{ padding: 6px 18px; border: 1px solid #BDBDBD; border-bottom: none; margin-right: 2px; }}
QTabBar::tab:selected {{ border-color: {BLUE}; color: {BLUE}; font-weight: 600; }}
QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox, QPlainTextEdit, QListWidget {{
    border: 1px solid #9FB7DA; border-radius: 3px; padding: 2px 4px; }}
QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus {{ border-color: {BLUE}; }}
QScrollArea {{ border: none; }}
QLabel#value {{ font-family: "Consolas"; }}
"""


def config_defaults():
    """Initial Config values: the firmware boots with the config.h defaults."""
    with open(os.path.join(ROOT, "App", "config.h"), encoding="utf-8") as f:
        txt = f.read()

    def num(name):
        return float(re.search(rf"#define {name}\s+\(?(-?[\d.]+)", txt).group(1))

    return {"kp_v": num("KP_V"), "ki_v": num("KI_V"), "kp_p": num("KP_P"),
            "u_dz": num("U_DZ_PERMILLE"), "v_max": num("V_MAX_DEFAULT_MMS"),
            "a_max": num("A_MAX_DEFAULT_MMS2"), "telem_hz": 0.0,
            "V_MAX": num("V_MAX_MMS"), "BRINGUP": num("BRINGUP_DUTY_MAX_PERMILLE"),
            "A_MIN": num("A_MAX_MIN_MMS2"), "A_MAX": num("A_MAX_MAX_MMS2"),
            "TS": num("CTRL_TS_S")}


class Bridge(QObject):
    """Carries results from worker / reader threads to the UI thread."""
    reply = Signal(object)        # list of (cmd, args, status or exception, data)
    event = Signal(int, bytes)


def button(text, primary=False):
    b = QPushButton(text)
    if primary:
        b.setProperty("primary", True)
    return b


def value_label(text="-"):
    lab = QLabel(text)
    lab.setObjectName("value")
    lab.setTextInteractionFlags(Qt.TextSelectableByMouse)
    return lab


def dspin(lo, hi, step, decimals, suffix=""):
    s = QDoubleSpinBox()
    s.setRange(lo, hi)
    s.setSingleStep(step)
    s.setDecimals(decimals)
    s.setSuffix(suffix)
    return s


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Conveyor Tuning")
        self.resize(1400, 900)
        self.cfg = config_defaults()
        self.link = None
        self.pool = ThreadPoolExecutor(max_workers=1)  # commands, in order, off the UI thread
        self.bridge = Bridge()
        self.bridge.reply.connect(self.on_reply)
        self.bridge.event.connect(self.on_event)
        self.telem_q = collections.deque()             # filled by the reader thread
        self.buf = collections.deque(maxlen=BUF_S * TELEM_HZ)
        self.last = None                               # last telemetry dict
        self.state = None
        self.prev_t = None
        self.lost_samples = 0
        self.rx_times = collections.deque(maxlen=TELEM_HZ * 2)
        self.rec = None                                # run being recorded (REQ-G07)
        self.run_cache = {}                            # path -> analyze.load() result
        self.cmp_curves = []
        self.comparing = False

        tabs = QTabWidget()
        tabs.addTab(self.build_general(), "Tổng quan")
        tabs.addTab(self.build_dashboard(), "Dashboard")
        root = QWidget()
        lay = QVBoxLayout(root)
        lay.addLayout(self.build_top_bar())
        lay.addWidget(tabs)
        self.setCentralWidget(root)

        QShortcut(QKeySequence(Qt.Key_Escape), self, self.estop, context=Qt.ApplicationShortcut)
        self.timer = QTimer(self)
        self.timer.setTimerType(Qt.PreciseTimer)  # the default coarse timer rounds to ~16 ms steps
        self.timer.timeout.connect(self.tick)
        self.timer.start(33)  # ~30 frames/s (REQ-G02)
        self.refresh_ports()
        self.load_profiles()
        self.load_runs()
        self.update_controls()

    # ---------- layout ----------

    def build_top_bar(self):
        self.port = QComboBox()
        self.port.setEditable(True)  # COMx or socket://host:port
        self.port.setMinimumWidth(170)
        self.btn_refresh = button("Làm mới")
        self.btn_refresh.clicked.connect(self.refresh_ports)
        self.btn_connect = button("Kết nối", primary=True)
        self.btn_connect.clicked.connect(self.toggle_connect)
        self.lab_link = QLabel("Chưa kết nối")
        self.lab_state = QLabel("-")
        self.lab_state.setMinimumWidth(90)
        self.lab_state.setAlignment(Qt.AlignCenter)
        self.lab_fault = QLabel("-")
        self.lab_rec = QLabel("")
        self.btn_estop = button("ESTOP (Esc)")
        self.btn_estop.setObjectName("estop")
        self.btn_estop.clicked.connect(self.estop)

        bar = QHBoxLayout()
        for w in (QLabel("Cổng"), self.port, self.btn_refresh, self.btn_connect):
            bar.addWidget(w)
        bar.addSpacing(24)
        for name, w in (("Link", self.lab_link), ("State", self.lab_state),
                        ("Fault", self.lab_fault)):
            bar.addWidget(QLabel(name))
            bar.addWidget(w)
            bar.addSpacing(16)
        bar.addWidget(self.lab_rec)
        bar.addStretch(1)
        bar.addWidget(self.btn_estop)
        return bar

    def build_general(self):
        # State
        self.btn_enable = button("ENABLE", primary=True)
        self.btn_disable = button("DISABLE")
        self.btn_stop = button("STOP")
        self.btn_clear = button("CLEAR_FAULT")
        self.btn_enable.clicked.connect(lambda: self.send([(P.ENABLE, (1,))]))
        self.btn_disable.clicked.connect(lambda: self.send([(P.ENABLE, (0,))]))
        self.btn_stop.clicked.connect(lambda: self.send([(P.STOP, ())]))
        self.btn_clear.clicked.connect(lambda: self.send([(P.CLEAR_FAULT, ())]))
        g_state = QGroupBox("State")
        h = QHBoxLayout(g_state)
        for b in (self.btn_enable, self.btn_disable, self.btn_stop, self.btn_clear):
            h.addWidget(b)

        # Status (telemetry)
        g_status = QGroupBox("Status (telemetry)")
        form = QFormLayout(g_status)
        self.st = {}
        for key, name in (("state", "State"), ("fault", "Fault"), ("pos", "pos [mm]"),
                          ("vel", "vel [mm/s]"), ("vref", "v_ref [mm/s]"),
                          ("pref", "pos_ref [mm]"), ("duty", "duty [‰]"),
                          ("missed", "missed_cycles"), ("t", "t_ms"),
                          ("rate", "Telemetry")):
            self.st[key] = value_label()
            form.addRow(name, self.st[key])

        # Config: last values sent (initial values from config.h)
        g_cfg = QGroupBox("Config (giá trị gửi gần nhất, khởi tạo từ config.h)")
        form = QFormLayout(g_cfg)
        self.cf = {}
        for key, name in (("kp_v", "Kp_v [‰/(mm/s)]"), ("ki_v", "Ki_v [‰/mm]"),
                          ("kp_p", "Kp_p [1/s]"), ("u_dz", "u_dz [‰]"),
                          ("v_max", "v_max [mm/s]"), ("a_max", "a_max [mm/s²]"),
                          ("telem_hz", "Telemetry [Hz]")):
            self.cf[key] = value_label()
            form.addRow(name, self.cf[key])
        self.lab_active = value_label()
        form.addRow("Profile", self.lab_active)

        # Diagnostics
        g_diag = QGroupBox("Diagnostics")
        grid = QGridLayout(g_diag)
        self.btn_diag = button("Đọc GET_DIAG")
        self.btn_diag.clicked.connect(lambda: self.send([(P.GET_DIAG, ())]))
        self.lab_version = value_label()
        grid.addWidget(self.btn_diag, 0, 0)
        grid.addWidget(QLabel("PROTO_VERSION"), 0, 1)
        grid.addWidget(self.lab_version, 0, 2)
        self.dg = {}
        for i, key in enumerate(P.DIAG_FIELDS):
            self.dg[key] = value_label()
            grid.addWidget(QLabel(key), 1 + i // 2, (i % 2) * 2)
            grid.addWidget(self.dg[key], 1 + i // 2, (i % 2) * 2 + 1)

        # Servo
        g_servo = QGroupBox("Servo")
        h2 = QHBoxLayout(g_servo)
        self.servo_deg = dspin(-90, 90, 5, 1, " °")
        self.btn_servo = button("Gửi SERVO_SET")
        self.btn_servo.clicked.connect(
            lambda: self.send([(P.SERVO_SET, (int(round(self.servo_deg.value() * 10)),))]))
        h2.addWidget(QLabel("Góc"))
        h2.addWidget(self.servo_deg)
        h2.addWidget(self.btn_servo)
        h2.addStretch(1)

        # Log
        g_log = QGroupBox("Log")
        self.log_box = QPlainTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setMaximumBlockCount(500)
        QVBoxLayout(g_log).addWidget(self.log_box)

        left = QVBoxLayout()
        for g in (g_state, g_status, g_cfg):
            left.addWidget(g)
        left.addStretch(1)
        right = QVBoxLayout()
        right.addWidget(g_diag)
        right.addWidget(g_servo)
        right.addWidget(g_log, 1)
        w = QWidget()
        h3 = QHBoxLayout(w)
        h3.addLayout(left, 1)
        h3.addLayout(right, 1)
        return w

    def build_setpoint(self):
        c = self.cfg
        self.v_set = dspin(-c["V_MAX"], c["V_MAX"], 5, 0, " mm/s")
        self.v_set.setValue(50)
        self.btn_run_v = button("Run", primary=True)
        self.btn_run_v.clicked.connect(lambda: self.run_motion(P.SET_VEL, int(self.v_set.value())))
        self.p_set = dspin(-1000, 1000, 10, 1, " mm")
        self.p_set.setValue(200)
        self.btn_move = button("Move", primary=True)
        self.btn_move.clicked.connect(self.move_test)
        self.d_set = QSpinBox()
        self.d_set.setSingleStep(10)
        self.d_set.setSuffix(" ‰")
        self.unlock = QCheckBox(f"Mở khóa > {int(c['BRINGUP'])} ‰")
        self.unlock.toggled.connect(self.update_duty_range)
        self.update_duty_range()
        self.btn_run_d = button("Run", primary=True)
        self.btn_run_d.clicked.connect(lambda: self.run_motion(P.OPENLOOP_DUTY, self.d_set.value()))
        self.hold = dspin(0.5, 30, 0.5, 1, " s")
        self.hold.setValue(3.0)
        self.btn_step_v = button("Step test")
        self.btn_step_v.clicked.connect(
            lambda: self.step_test("vel", P.SET_VEL, int(self.v_set.value())))
        self.btn_step_d = button("Step test")
        self.btn_step_d.clicked.connect(
            lambda: self.step_test("openloop", P.OPENLOOP_DUTY, self.d_set.value()))

        self.pages = QStackedWidget()
        self.stop_buttons = []
        pages = ((("Setpoint v", self.v_set),), (("Quãng đường", self.p_set),),
                 (("Duty", self.d_set), ("", self.unlock)))
        runs = (self.btn_run_v, self.btn_move, self.btn_run_d)
        steps = (self.btn_step_v, None, self.btn_step_d)
        for rows, run, step in zip(pages, runs, steps):
            page = QWidget()
            form = QFormLayout(page)
            for name, w in rows:
                form.addRow(name, w)
            stop = button("Stop")
            stop.clicked.connect(lambda: self.send([(P.STOP, ())]))
            self.stop_buttons.append(stop)
            hb = QHBoxLayout()
            hb.addWidget(run)
            hb.addWidget(stop)
            form.addRow(hb)
            if step:
                hs = QHBoxLayout()
                hs.addWidget(QLabel("Giữ"))
                hs.addWidget(self.hold if step is self.btn_step_v else self.make_hold_mirror())
                hs.addWidget(step)
                form.addRow(hs)
            else:
                form.addRow(QLabel("Move được ghi lại thành run"))
            self.pages.addWidget(page)
        g = QGroupBox("Setpoint")
        QVBoxLayout(g).addWidget(self.pages)
        return g

    def make_hold_mirror(self):
        """Second hold spin box (open-loop page) kept in sync with the first."""
        s = dspin(0.5, 30, 0.5, 1, " s")
        s.setValue(self.hold.value())
        s.valueChanged.connect(self.hold.setValue)
        self.hold.valueChanged.connect(s.setValue)
        return s

    def build_tuning(self):
        c = self.cfg
        g = QGroupBox("Tuning")
        form = QFormLayout(g)
        self.t_kpv = dspin(0, 100, 0.1, 3)
        self.t_kiv = dspin(0, 2000, 1, 2)
        self.t_kpp = dspin(0, 50, 0.1, 3)
        self.t_udz = dspin(0, 1000, 5, 1)
        self.t_vmax = dspin(1, c["V_MAX"], 5, 0)
        self.t_amax = dspin(c["A_MIN"], c["A_MAX"], 10, 0)
        self.tune = dict(zip(GAIN_KEYS + LIMIT_KEYS, (self.t_kpv, self.t_kiv, self.t_kpp,
                                                      self.t_udz, self.t_vmax, self.t_amax)))
        for key, w in self.tune.items():
            w.setValue(c[key])
        form.addRow("Kp_v [‰/(mm/s)]", self.t_kpv)
        form.addRow("Ki_v [‰/mm]", self.t_kiv)
        form.addRow("Kp_p [1/s]", self.t_kpp)
        form.addRow("u_dz [‰]", self.t_udz)
        self.btn_gains = button("Apply gains", primary=True)
        self.btn_gains.clicked.connect(lambda: self.send([self.gains_cmd()]))
        form.addRow(self.btn_gains)
        form.addRow("v_max [mm/s]", self.t_vmax)
        form.addRow("a_max [mm/s²]", self.t_amax)
        self.btn_limits = button("Apply limits", primary=True)
        self.btn_limits.clicked.connect(lambda: self.send([self.limits_cmd()]))
        form.addRow(self.btn_limits)
        form.addRow(QLabel("Chỉ gửi được ở IDLE / READY"))
        return g

    def build_calculator(self):
        """REQ-G05: spec §5.2 pole cancellation."""
        g = QGroupBox("Gain calculator (§5.2)")
        form = QFormLayout(g)
        self.k_K = dspin(0.001, 10, 0.01, 4)
        self.k_K.setValue(0.24)
        self.k_tau = dspin(0.001, 5, 0.005, 3, " s")
        self.k_tau.setValue(0.05)
        self.k_tcl = dspin(0.01, 5, 0.01, 3, " s")
        self.k_tcl.setValue(0.1)
        form.addRow("K [mm/s per ‰]", self.k_K)
        form.addRow("τ [s]", self.k_tau)
        form.addRow("τ_cl [s]", self.k_tcl)
        self.calc_out = value_label()
        self.calc_warn = QLabel("")
        self.calc_warn.setStyleSheet(f"color: {RED};")
        self.calc_warn.setWordWrap(True)
        form.addRow(self.calc_out)
        form.addRow(self.calc_warn)
        btn = button("Chép sang Tuning")
        btn.clicked.connect(self.copy_calc)
        form.addRow(btn)
        for s in (self.k_K, self.k_tau, self.k_tcl):
            s.valueChanged.connect(self.update_calc)
        self.update_calc()
        return g

    def build_profiles(self):
        """REQ-G09: gain profiles on the PC, the active one is sent on Connect."""
        g = QGroupBox("Profile")
        v = QVBoxLayout(g)
        self.prof = QComboBox()
        self.prof.currentTextChanged.connect(self.set_active_profile)
        h = QHBoxLayout()
        for text, fn in (("Lưu…", self.save_profile), ("Nạp", self.load_profile),
                         ("Xóa", self.delete_profile)):
            b = button(text)
            b.clicked.connect(fn)
            h.addWidget(b)
        v.addWidget(self.prof)
        v.addLayout(h)
        v.addWidget(QLabel("Profile đang chọn được gửi khi Kết nối"))
        return g

    def build_dashboard(self):
        g_mode = QGroupBox("Control mode")
        self.mode = QButtonGroup(self)
        hm = QHBoxLayout(g_mode)
        for i, name in enumerate(("Velocity", "Position", "Open-loop")):
            r = QRadioButton(name)
            self.mode.addButton(r, i)
            hm.addWidget(r)
        self.mode.button(0).setChecked(True)
        g_set = self.build_setpoint()
        self.mode.idToggled.connect(lambda i, on: on and self.on_mode(i))

        controls = QWidget()
        cl = QVBoxLayout(controls)
        for g in (g_mode, g_set, self.build_tuning(), self.build_calculator(),
                  self.build_profiles()):
            cl.addWidget(g)
        cl.addStretch(1)
        left = QScrollArea()
        left.setWidget(controls)
        left.setWidgetResizable(True)
        left.setFixedWidth(360)
        left.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        # Plots
        self.win_s = QSpinBox()
        self.win_s.setRange(2, BUF_S)
        self.win_s.setValue(10)
        self.win_s.setSuffix(" s")
        self.pause = QCheckBox("Pause")
        btn_clear = button("Xóa")
        btn_clear.clicked.connect(self.buf.clear)
        self.lab_view = QLabel("Live")
        self.lab_view.setStyleSheet(f"color: {BLUE}; font-weight: 600;")
        ph = QHBoxLayout()
        for w in (self.lab_view, QLabel("   Cửa sổ"), self.win_s, self.pause, btn_clear):
            ph.addWidget(w)
        ph.addStretch(1)

        pg.setConfigOptions(background="w", foreground="#424242", antialias=True)
        self.glw = pg.GraphicsLayoutWidget()
        meas = pg.mkPen(BLUE, width=2)
        ref = pg.mkPen(GRAY, width=1.5, style=Qt.DashLine)
        self.plots = []
        for row, ylab in enumerate(("v [mm/s]", "pos [mm]", "e_v = v_ref − v [mm/s]", "duty [‰]")):
            p = self.glw.addPlot(row=row, col=0)
            p.showGrid(x=True, y=True, alpha=0.15)
            p.setLabel("left", ylab)
            if self.plots:
                p.setXLink(self.plots[0])
            self.plots.append(p)
        self.plots[-1].setLabel("bottom", "t [s] (0 = mẫu mới nhất)")
        self.legends = [self.plots[0].addLegend(offset=(-10, 5)),
                        self.plots[1].addLegend(offset=(-10, 5))]
        self.c_v = self.plots[0].plot(pen=meas, name="v")
        self.c_vref = self.plots[0].plot(pen=ref, name="v_ref")
        self.c_p = self.plots[1].plot(pen=meas, name="pos")
        self.c_pref = self.plots[1].plot(pen=ref, name="pos_ref")
        self.c_e = self.plots[2].plot(pen=meas)
        self.c_d = self.plots[3].plot(pen=meas)
        self.live_curves = (self.c_v, self.c_vref, self.c_p, self.c_pref, self.c_e, self.c_d)
        self.plots[2].addLine(y=0, pen=pg.mkPen(GRAY, width=1))
        for y in (-1000, 1000):
            self.plots[3].addLine(y=y, pen=pg.mkPen(GRAY, width=1, style=Qt.DotLine))

        # Metrics (REQ-G06) and runs (REQ-G07)
        g_met = QGroupBox("Metrics (run gần nhất)")
        self.met_grid = QGridLayout(g_met)
        self.met_title = QLabel("Chưa có run")
        self.met_grid.addWidget(self.met_title, 0, 0, 1, 3)
        self.met_rows = []
        g_runs = QGroupBox("Runs (tick để so sánh)")
        vr = QVBoxLayout(g_runs)
        self.runs = QListWidget()
        self.runs.itemChanged.connect(lambda _: self.draw_compare())
        vr.addWidget(self.runs)
        hr = QHBoxLayout()
        for text, fn in (("Nạp CSV…", self.open_csv), ("Bỏ chọn hết", self.uncheck_runs),
                         ("Bỏ khỏi danh sách", self.remove_run)):
            b = button(text)
            b.clicked.connect(fn)
            hr.addWidget(b)
        hr.addStretch(1)
        vr.addLayout(hr)
        bottom = QHBoxLayout()
        bottom.addWidget(g_met, 2)
        bottom.addWidget(g_runs, 3)
        bw = QWidget()
        bw.setLayout(bottom)
        bw.setFixedHeight(230)

        right = QVBoxLayout()
        right.addLayout(ph)
        right.addWidget(self.glw, 1)
        right.addWidget(bw)
        w = QWidget()
        h = QHBoxLayout(w)
        h.addWidget(left)
        h.addLayout(right, 1)
        return w

    # ---------- connection ----------

    def refresh_ports(self):
        cur = self.port.currentText()
        self.port.clear()
        self.port.addItems([p.device for p in serial.tools.list_ports.comports()])
        if cur:
            self.port.setCurrentText(cur)

    def toggle_connect(self):
        if self.link:
            self.disconnect_link()
            return
        port = self.port.currentText().strip()
        try:
            self.link = Link(port, verbose=False, on_telem=self.telem_q.append,
                             on_event=lambda c, p: self.bridge.event.emit(c, bytes(p)))
        except Exception as e:  # serial.SerialException, bad URL
            self.log(f"Không mở được {port}: {e}")
            self.link = None
            return
        self.buf.clear()
        self.prev_t = None
        self.lost_samples = 0
        self.link.heartbeat_on = True  # REQ-G01: watchdog fed while connected
        self.log(f"Kết nối {port}")
        cmds = [(P.PING, ()), (P.SET_TELEM, (TELEM_HZ,))]
        prof = self.read_profile(self.prof.currentText())
        if prof:  # REQ-G09
            self.set_tuning(prof)
            cmds += [self.gains_cmd(), self.limits_cmd()]
            self.log(f"Gửi profile '{self.prof.currentText()}'")
        self.send(cmds)
        self.update_controls()

    def disconnect_link(self, wait=False):
        """REQ-G01: STOP, ENABLE 0, SET_TELEM 0, then close."""
        link, self.link = self.link, None
        if not link:
            return
        self.rec = None

        def job():
            for cmd, args in ((P.STOP, ()), (P.ENABLE, (0,)), (P.SET_TELEM, (0,))):
                try:
                    link.request(cmd, P.pack(cmd, *args))
                except LinkError:
                    break
            link.close()

        fut = self.pool.submit(job)
        if wait:
            fut.result(timeout=3)
        self.state = None
        self.last = None
        self.log("Ngắt kết nối")
        self.update_controls()

    def closeEvent(self, ev):
        self.disconnect_link(wait=True)
        self.pool.shutdown(wait=False)
        super().closeEvent(ev)

    # ---------- commands ----------

    def gains_cmd(self):
        return (P.SET_GAINS, tuple(self.tune[k].value() for k in GAIN_KEYS))

    def limits_cmd(self):
        return (P.SET_LIMITS, tuple(int(self.tune[k].value()) for k in LIMIT_KEYS))

    def send(self, cmds):
        """Runs [(cmd, args), ...] in order on the worker thread; stops at the first non-OK."""
        link = self.link
        if not link:
            return

        def job():
            out = []
            for cmd, args in cmds:
                try:
                    st, data, _ = link.request(cmd, P.pack(cmd, *args))
                except LinkError as e:
                    out.append((cmd, args, e, b""))
                    break
                out.append((cmd, args, st, bytes(data)))
                if st != 0:
                    break
            self.bridge.reply.emit(out)

        self.pool.submit(job)

    def run_motion(self, cmd, value):
        """Run from the Dashboard; ENABLE first when the actuator is IDLE."""
        if self.state == 4:
            self.log("Đang FAULT: bấm CLEAR_FAULT trước")
            return
        seq = [(P.ENABLE, (1,))] if self.state == 1 else []
        self.send(seq + [(cmd, (value,))])

    def estop(self):
        """Own thread: must not wait behind queued commands (REQ-G04)."""
        link = self.link
        if not link:
            return

        def job():
            try:
                link.request(P.ESTOP)
            except LinkError as e:
                self.bridge.reply.emit([(P.ESTOP, (), e, b"")])

        threading.Thread(target=job, daemon=True).start()
        self.rec = None
        self.log("ESTOP")

    def on_reply(self, results):
        for cmd, args, st, data in results:
            name = P.CMD_NAME[cmd]
            if isinstance(st, Exception):
                self.log(f"{name}: {st}")
                self.abort_run(cmd)
                continue
            if st != 0:  # REQ-G03
                self.log(f"{name}{list(args) if args else ''}: {P.STATUS[st]}")
                self.abort_run(cmd)
                continue
            if cmd == P.PING:
                self.lab_version.setText(str(data[0]))
                if data[0] != P.PROTO_VERSION:
                    self.log(f"PROTO_VERSION board = {data[0]}, tool = {P.PROTO_VERSION}")
            elif cmd == P.GET_DIAG:
                for k, v in P.unpack_diag(data).items():
                    self.dg[k].setText(str(v))
            elif cmd == P.SET_GAINS:
                self.cfg.update(zip(GAIN_KEYS, args))
            elif cmd == P.SET_LIMITS:
                self.cfg.update(zip(LIMIT_KEYS, args))
            elif cmd == P.SET_TELEM:
                self.cfg["telem_hz"] = args[0]
            if cmd not in (P.PING, P.GET_STATUS, P.GET_DIAG, P.SET_TELEM):
                self.log(f"{name}{list(args) if args else ''}: OK")

    def on_event(self, cmd, pl):
        if cmd == P.EVT_FAULT:
            self.log(f"EVT_FAULT {P.fault_names(pl[0])}")
            return
        err = int.from_bytes(pl[1:5], "little", signed=True)
        self.log(f"EVT_MOVE_DONE seq={pl[0]} final error {err / 1000:.3f} mm")
        rec = self.rec
        if rec and rec["mode"] == "move":
            QTimer.singleShot(500, lambda: self.finish_run(rec))

    def log(self, text):
        t = time.strftime("%H:%M:%S") + f".{int(time.time() * 1000) % 1000:03d}"
        self.log_box.appendPlainText(f"{t}  {text}")

    # ---------- recorded runs (REQ-G07) ----------

    def start_run(self, mode, setpoint, cmd, value):
        if self.rec:
            self.log("Đang ghi một run khác")
            return None
        if self.state == 4:
            self.log("Đang FAULT: bấm CLEAR_FAULT trước")
            return None
        if self.state == 1:
            self.send([(P.ENABLE, (1,))])
        rec = {"mode": mode, "setpoint": setpoint, "rows": [], "cmd": cmd,
               "start": time.strftime("%Y%m%d-%H%M%S"),
               "gains": {k: self.cfg[k] for k in GAIN_KEYS},
               "limits": {k: self.cfg[k] for k in LIMIT_KEYS}}
        self.rec = rec
        # 0.5 s of baseline before the command, like host.py scenarios
        QTimer.singleShot(500, lambda: self.rec is rec and self.send([(cmd, (value,))]))
        return rec

    def step_test(self, mode, cmd, value):
        rec = self.start_run(mode, value, cmd, value)
        if not rec:
            return
        hold = self.hold.value()
        QTimer.singleShot(int((0.5 + hold) * 1000),
                          lambda: self.rec is rec and self.send([(P.STOP, ())]))
        QTimer.singleShot(int((1.5 + hold) * 1000), lambda: self.finish_run(rec))

    def move_test(self):
        mm = self.p_set.value()
        rec = self.start_run("move", mm, P.MOVE_REL, int(round(mm * 1000)))
        if not rec:
            return
        v, a = max(self.cfg["v_max"], 1), max(self.cfg["a_max"], 1)
        t_est = abs(mm) / v + v / a
        QTimer.singleShot(int((0.5 + 2 * t_est + 3) * 1000),
                          lambda: self.finish_run(rec, timeout=True))

    def abort_run(self, cmd):
        if self.rec and cmd in (P.ENABLE, self.rec["cmd"]):
            self.log("Run bị hủy")
            self.rec = None

    def finish_run(self, rec, timeout=False):
        if self.rec is not rec:
            return
        self.rec = None
        if timeout:
            self.log("Move: không nhận được EVT_MOVE_DONE, lưu run tới thời điểm này")
        if not rec["rows"]:
            self.log("Run không có telemetry")
            return
        os.makedirs(RUNS_DIR, exist_ok=True)
        base = os.path.join(RUNS_DIR, f"{rec['start']}_{rec['mode']}_{rec['setpoint']:g}")
        with open(base + ".csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(CSV_HEADER)
            w.writerows(rec["rows"])
        unit = {"vel": "mm/s", "move": "mm", "openloop": "permille"}[rec["mode"]]
        meta = {k: rec[k] for k in ("mode", "setpoint", "gains", "limits", "start")}
        meta["unit"] = unit
        with open(base + ".json", "w", encoding="utf-8") as f:
            json.dump(meta, f, indent=2)
        self.log(f"Lưu run {os.path.basename(base)}.csv")
        self.add_run(base + ".csv", check=False, front=True)
        self.show_metrics(base + ".csv", rec["mode"])

    def show_metrics(self, path, mode):
        for w in self.met_rows:
            w.deleteLater()
        self.met_rows = []
        self.met_title.setText(os.path.basename(path))
        if mode == "openloop":
            self.met_title.setText(os.path.basename(path) + ": open-loop, dùng analyze.py --fit")
            return
        try:
            m = analyze.metrics(self.load_run(path))
        except Exception as e:  # too short, no step found, ...
            self.met_title.setText(f"{os.path.basename(path)}: không tính được ({e})")
            return
        for i, (k, val) in enumerate(m.items(), start=1):
            ok = analyze.passes(k, val)
            cells = [QLabel(METRIC_NAMES.get(k, k)), value_label("-" if val != val else f"{val:.4g}"),
                     QLabel("" if ok is None else ("PASS" if ok else "FAIL"))]
            if ok is not None:
                cells[2].setStyleSheet(f"color: {GREEN if ok else RED}; font-weight: 600;")
            for col, wdg in enumerate(cells):
                self.met_grid.addWidget(wdg, i, col)
                self.met_rows.append(wdg)

    # ---------- run list and comparison (9.6) ----------

    def load_run(self, path):
        if path not in self.run_cache:
            self.run_cache[path] = analyze.load(path)
        return self.run_cache[path]

    def run_label(self, path):
        meta_path = os.path.splitext(path)[0] + ".json"
        if not os.path.exists(meta_path):
            return os.path.basename(path)
        with open(meta_path, encoding="utf-8") as f:
            m = json.load(f)
        g = m.get("gains", {})
        return (f"{m['start'][9:11]}:{m['start'][11:13]}:{m['start'][13:15]}  {m['mode']} "
                f"{m['setpoint']:g} {m['unit']}   Kp_v={g.get('kp_v', 0):g} Ki_v={g.get('ki_v', 0):g}"
                f" Kp_p={g.get('kp_p', 0):g} u_dz={g.get('u_dz', 0):g}")

    def add_run(self, path, check=False, front=False):
        item = QListWidgetItem(self.run_label(path))
        item.setData(Qt.UserRole, path)
        item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
        item.setCheckState(Qt.Checked if check else Qt.Unchecked)
        item.setToolTip(path)
        if front:
            self.runs.insertItem(0, item)
        else:
            self.runs.addItem(item)

    def load_runs(self):
        for path in sorted(glob.glob(os.path.join(RUNS_DIR, "*.csv")), reverse=True):
            self.add_run(path)

    def open_csv(self):
        path, _ = QFileDialog.getOpenFileName(self, "Nạp CSV", ROOT, "CSV (*.csv)")
        if path:
            self.add_run(path, check=True, front=True)

    def uncheck_runs(self):
        for i in range(self.runs.count()):
            self.runs.item(i).setCheckState(Qt.Unchecked)

    def remove_run(self):
        for item in self.runs.selectedItems():
            self.runs.takeItem(self.runs.row(item))
        self.draw_compare()

    def checked_runs(self):
        return [self.runs.item(i) for i in range(self.runs.count())
                if self.runs.item(i).checkState() == Qt.Checked]

    def draw_compare(self):
        for plot, c in self.cmp_curves:
            plot.removeItem(c)
        self.cmp_curves = []
        items = self.checked_runs()
        self.comparing = bool(items)
        for lg in self.legends:
            lg.setVisible(not self.comparing)
        self.runs.blockSignals(True)  # setForeground would emit itemChanged -> draw_compare again
        for i in range(self.runs.count()):
            item = self.runs.item(i)
            item.setForeground(QColor(PALETTE[items.index(item) % len(PALETTE)]
                                      if item in items else TEXT))
        self.runs.blockSignals(False)
        if not self.comparing:
            self.lab_view.setText("Live")
            self.plots[-1].setLabel("bottom", "t [s] (0 = mẫu mới nhất)")
            self.on_mode(self.mode.checkedId())
            return
        self.lab_view.setText(f"So sánh {len(items)} run")
        self.plots[2].setLabel("left", "e = ref − đo (v: mm/s, move: mm)")
        self.plots[-1].setLabel("bottom", "t [s] (0 = bắt đầu step)")
        for c in self.live_curves:
            c.setData([], [])
        for i, item in enumerate(items):
            color = PALETTE[i % len(PALETTE)]
            try:
                d = self.load_run(item.data(Qt.UserRole))
            except Exception as e:
                self.log(f"Không đọc được {item.toolTip()}: {e}")
                continue
            i0 = analyze.step_start(d)
            x = d["t"] - d["t"][i0]
            p0 = d["pos_mm"][i0]
            e = (d["pos_ref_mm"] - d["pos_mm"]) if analyze.is_move(d) else (d["vel_ref_mms"] - d["vel_mms"])
            solid, dash = pg.mkPen(color, width=2), pg.mkPen(color, width=1, style=Qt.DashLine)
            for plot, y, pen in ((0, d["vel_mms"], solid), (0, d["vel_ref_mms"], dash),
                                 (1, d["pos_mm"] - p0, solid), (1, d["pos_ref_mm"] - p0, dash),
                                 (2, e, solid), (3, d["duty"], solid)):
                self.cmp_curves.append((self.plots[plot], self.plots[plot].plot(x, y, pen=pen)))
        self.plots[0].enableAutoRange(x=True)

    # ---------- gain calculator and profiles ----------

    def update_calc(self):
        g = analyze.pi_from_model(self.k_K.value(), self.k_tau.value(), self.k_tcl.value(),
                                  self.cfg["TS"])
        self.calc = g
        self.calc_out.setText(f"Kp_v = {g['kp_v']:.3f}\nKi_v = {g['ki_v']:.2f}\n"
                              f"Kp_p <= {g['kp_p_max']:.3f}\nt_s ~ {g['t_s']:.3f} s")
        self.calc_warn.setText("" if g["ts_ok"] else
                               f"Ts = {self.cfg['TS'] * 1000:g} ms > τ_cl/10: tăng τ_cl")

    def copy_calc(self):
        self.t_kpv.setValue(self.calc["kp_v"])
        self.t_kiv.setValue(self.calc["ki_v"])
        self.t_kpp.setValue(self.calc["kp_p_max"])
        self.log("Đã chép gain từ calculator sang Tuning (bấm Apply gains để gửi)")

    def set_tuning(self, values):
        for k, w in self.tune.items():
            if k in values:
                w.setValue(values[k])

    def read_profile(self, name):
        path = os.path.join(PROFILES_DIR, name + ".json")
        if name == NO_PROFILE or not os.path.exists(path):
            return None
        with open(path, encoding="utf-8") as f:
            return json.load(f)

    def load_profiles(self, select=None):
        names = sorted(os.path.splitext(os.path.basename(p))[0]
                       for p in glob.glob(os.path.join(PROFILES_DIR, "*.json")))
        if select is None and os.path.exists(ACTIVE_FILE):
            with open(ACTIVE_FILE, encoding="utf-8") as f:
                select = f.read().strip()
        self.prof.blockSignals(True)
        self.prof.clear()
        self.prof.addItems([NO_PROFILE] + names)
        self.prof.setCurrentText(select if select in names else NO_PROFILE)
        self.prof.blockSignals(False)
        self.lab_active.setText(self.prof.currentText())

    def set_active_profile(self, name):
        os.makedirs(PROFILES_DIR, exist_ok=True)
        with open(ACTIVE_FILE, "w", encoding="utf-8") as f:
            f.write(name)
        self.lab_active.setText(name)

    def save_profile(self):
        cur = self.prof.currentText()
        name, ok = QInputDialog.getText(self, "Lưu profile", "Tên profile:",
                                        text="" if cur == NO_PROFILE else cur)
        name = name.strip()
        if not ok or not name or name == NO_PROFILE or re.search(r'[\\/:*?"<>|]', name):
            return
        os.makedirs(PROFILES_DIR, exist_ok=True)
        with open(os.path.join(PROFILES_DIR, name + ".json"), "w", encoding="utf-8") as f:
            json.dump({k: w.value() for k, w in self.tune.items()}, f, indent=2)
        self.load_profiles(select=name)
        self.set_active_profile(name)
        self.log(f"Lưu profile '{name}'")

    def load_profile(self):
        prof = self.read_profile(self.prof.currentText())
        if prof:
            self.set_tuning(prof)
            self.log(f"Nạp profile '{self.prof.currentText()}' vào Tuning (bấm Apply để gửi)")

    def delete_profile(self):
        name = self.prof.currentText()
        if name == NO_PROFILE:
            return
        os.remove(os.path.join(PROFILES_DIR, name + ".json"))
        self.load_profiles(select=NO_PROFILE)
        self.set_active_profile(NO_PROFILE)
        self.log(f"Xóa profile '{name}'")

    # ---------- periodic update ----------

    def tick(self):
        while self.telem_q:
            t = self.telem_q.popleft()
            if self.prev_t is not None and t["t_ms"] - self.prev_t > 15:
                self.lost_samples += round((t["t_ms"] - self.prev_t) / 10) - 1
            self.prev_t = t["t_ms"]
            self.rx_times.append(time.perf_counter())
            self.last = t
            self.buf.append((t["t_ms"], t["pos_um"] / 1000, t["pos_ref_um"] / 1000,
                             t["vel_dmms"] / 10, t["vel_ref_dmms"] / 10, t["duty"]))
            if self.rec:
                self.rec["rows"].append(telem_row(t))
        self.update_status()
        if not self.pause.isChecked() and not self.comparing:
            self.update_plots()

    def update_status(self):
        link = self.link
        if link:
            age = time.perf_counter() - link.last_rx
            if age > 0.5:  # REQ-G01
                self.lab_link.setText("Mất kết nối")
                self.lab_link.setStyleSheet(f"color: {RED}; font-weight: 600;")
                self.state = None
            else:
                ping = f"{link.ping_ms:.1f} ms" if link.ping_ms is not None else "-"
                self.lab_link.setText(f"OK, {ping}")
                self.lab_link.setStyleSheet(f"color: {BLUE};")
        else:
            self.lab_link.setText("Chưa kết nối")
            self.lab_link.setStyleSheet("")
        self.lab_rec.setText(f"Đang ghi run ({self.rec['mode']})" if self.rec else "")
        self.lab_rec.setStyleSheet(f"color: {BLUE}; font-weight: 600;")

        t = self.last
        if link and t and time.perf_counter() - link.last_rx <= 0.5:
            self.state = t["state"]
            faults = P.fault_names(t["fault_bits"])
            name = P.STATES[t["state"]] if t["state"] < len(P.STATES) else str(t["state"])
            fault_on = t["state"] == 4
            self.lab_state.setText(name)
            self.lab_state.setStyleSheet(
                f"border: 1px solid {RED if fault_on else BLUE}; border-radius: 3px; padding: 2px 8px;"
                f"color: {'white' if fault_on else BLUE}; background: {RED if fault_on else 'white'};"
                "font-weight: 600;")
            self.lab_fault.setText(faults)
            self.lab_fault.setStyleSheet(f"color: {RED};" if t["fault_bits"] else "")
            now = time.perf_counter()
            rate = sum(1 for x in self.rx_times if now - x <= 1.0)
            for key, val in (("state", name), ("fault", faults),
                             ("pos", f"{t['pos_um'] / 1000:.3f}"),
                             ("vel", f"{t['vel_dmms'] / 10:.1f}"),
                             ("vref", f"{t['vel_ref_dmms'] / 10:.1f}"),
                             ("pref", f"{t['pos_ref_um'] / 1000:.3f}"),
                             ("duty", str(t["duty"])), ("missed", str(t["missed_cycles"])),
                             ("t", str(t["t_ms"])),
                             ("rate", f"{rate} Hz, mất {self.lost_samples} mẫu")):
                self.st[key].setText(val)
        else:
            self.lab_state.setText("-")
            self.lab_state.setStyleSheet("")
            self.lab_fault.setText("-")
            self.lab_fault.setStyleSheet("")
        for key, lab in self.cf.items():
            lab.setText(f"{self.cfg[key]:g}")
        self.update_controls()

    def update_controls(self):
        s = self.state if self.link else None
        on = s is not None
        idle = self.rec is None
        self.btn_connect.setText("Ngắt kết nối" if self.link else "Kết nối")
        self.port.setEnabled(self.link is None)
        self.btn_estop.setEnabled(self.link is not None)
        self.btn_enable.setEnabled(on and P.allowed(P.ENABLE, s))
        self.btn_disable.setEnabled(on and P.allowed(P.ENABLE, s))
        self.btn_stop.setEnabled(on)
        self.btn_clear.setEnabled(on and P.allowed(P.CLEAR_FAULT, s))
        self.btn_diag.setEnabled(on)
        self.btn_servo.setEnabled(on and P.allowed(P.SERVO_SET, s))
        # IDLE counts too: an ENABLE is sent first
        self.btn_run_v.setEnabled(on and idle and s in (1, 2, 3))
        self.btn_run_d.setEnabled(on and idle and s in (1, 2, 3))
        self.btn_move.setEnabled(on and idle and s in (1, 2))
        self.btn_step_v.setEnabled(on and idle and s in (1, 2))  # steps start from rest
        self.btn_step_d.setEnabled(on and idle and s in (1, 2))
        for b in self.stop_buttons:
            b.setEnabled(on)
        self.btn_gains.setEnabled(on and P.allowed(P.SET_GAINS, s))
        self.btn_limits.setEnabled(on and P.allowed(P.SET_LIMITS, s))

    def on_mode(self, i):
        self.pages.setCurrentIndex(i)
        self.plots[2].setLabel("left", "e_p = pos_ref − pos [mm]" if i == 1
                               else "e_v = v_ref − v [mm/s]")

    def update_duty_range(self):
        lim = 1000 if self.unlock.isChecked() else int(self.cfg["BRINGUP"])  # REQ-G04
        self.d_set.setRange(-lim, lim)

    def update_plots(self):
        n = self.win_s.value() * TELEM_HZ
        if not self.buf:
            for c in self.live_curves:
                c.setData([], [])
            return
        a = np.array(list(self.buf)[-n:], dtype=float)
        x = (a[:, 0] - a[-1, 0]) / 1000.0
        self.c_v.setData(x, a[:, 3])
        self.c_vref.setData(x, a[:, 4])
        self.c_p.setData(x, a[:, 1])
        self.c_pref.setData(x, a[:, 2])
        e = a[:, 2] - a[:, 1] if self.mode.checkedId() == 1 else a[:, 4] - a[:, 3]
        self.c_e.setData(x, e)
        self.c_d.setData(x, a[:, 5])
        self.plots[0].setXRange(-self.win_s.value(), 0, padding=0)


def main():
    app = QApplication(sys.argv)
    app.setStyleSheet(STYLE)
    app.setFont(QFont("Segoe UI", 10))
    w = MainWindow()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
