#!/usr/bin/env python3
"""
Live CPAP pressure / patient-flow plotter over USB VCP.

Expects CSV lines from firmware:
  T,<ms>,<P_sensor>,<patient_flow_slm>,<pid_rpm_cmd>,<mech_rpm>,<P_mask>
  R,<ms>,<rpm_cmd>,<rpm_act>,<pressure_cmh2o>   — on manual blower speed change

Usage:
  pip install pyserial matplotlib
  python cpap_telemetry_plot.py --port COM13
  python cpap_telemetry_plot.py --port COM13 --monitor   # text-only check
  python cpap_telemetry_plot.py --list                   # show COM ports
"""

from __future__ import annotations

import argparse
import collections
import re
import sys
import threading
import time
import tkinter as tk
from datetime import datetime
from tkinter import ttk

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
from matplotlib.ticker import MultipleLocator
import serial
import serial.tools.list_ports

TELEM_RE = re.compile(
    rb"^T,(\d+),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)"
    rb"(?:,(-?\d+))?(?:,(-?\d+))?(?:,([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?))?\s*$"
)
BENCH_RE = re.compile(rb"^R,(\d+),(-?\d+),(-?\d+),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*$")
PHYSIO_RE = re.compile(
    rb"^PH,(\d+),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),"
    rb"([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),(\d+),"
    rb"([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*$"
)
# Firmware S-line has grown past the leak/breath block (severity, session P,
# APAP events, …). Trailing fields are optional so a longer line still parses.
STATUS_RE = re.compile(
    rb"^S,(\d+),(\d+),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),"
    rb"([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),(\d+),(\d+),(\d+)"
    rb"(?:,(\d+),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?))?"
    rb"(?:,(\d+),(\d+),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),"
    rb"([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),(\d+),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),(\d+))?"
    rb"(?:,(\d+),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),(\d+))?"
    rb"(?:,(\d+),(-?\d+),(-?\d+),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),"
    rb"([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),(\d+))?"
    rb"(?:,([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),"
    rb"([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?))?"
    rb"(?:,(\d+),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?))?"
    rb"(?:,.*)?\s*$"
)

APAP_STATE_NAMES = {
    0: "Idle",
    1: "Monitor",
    2: "NoBreath",
    3: "FOT",
    4: "FOT",
}

THERAPY_STATE_NAMES = {
    0: "Idle",
    1: "Ramp",
    2: "Running",
    3: "Stopping",
    4: "Fault",
}

THERAPY_MODE_NAMES = {
    0: "CPAP",
    1: "Autoset",
    2: "Autoset for her",
}

DASH = "—"


def format_fot_impedance(z: float) -> str:
    if z >= 900.0:
        return "HIGH (blocked)"
    if z <= 0.0:
        return "—"
    return f"{z:.2f} cmH2O/slm"


def format_leak_status(st: dict[str, float | int]) -> str:
    line1, line2 = format_leak_status_lines(st)
    return line1 if not line2 else f"{line1}\n{line2}"


def format_leak_status_lines(st: dict[str, float | int]) -> tuple[str, str]:
    if "leak_algo" not in st:
        leak = float(st.get("leak", 0.0))
        return (f"Leak (therapy): {leak:.1f} slm  [detail requires updated firmware]", "")

    algo = int(st.get("leak_algo", 0))
    algo_name = "Vivo table×ratio" if algo else "Simple √P LPF"
    breath = "Insp" if int(st.get("leak_breath_insp", 0)) else "Exp/idle"
    if "leak_sev" in st:
        high = {2: "HIGH", 1: "small"}.get(int(st.get("leak_sev", 0)), "ok")
    else:
        high = "HIGH" if float(st.get("leak_uninten", 0.0)) > 24.0 else "ok"
    line1 = (
        f"Leak [{algo_name}]  Reported={float(st.get('leak_reported', 0.0)):.1f} slm  "
        f"Uninten={float(st.get('leak_uninten', 0.0)):.1f} slm  "
        f"VentRef={float(st.get('leak_vent', 0.0)):.1f} slm  "
        f"FlowLeak={float(st.get('leak_flow', 0.0)):.1f} slm"
    )
    line2 = (
        f"MeanRatio={int(st.get('leak_mean_ratio', 0))}  "
        f"Ratio10s={int(st.get('leak_mean_ratio_10s', 0))}  "
        f"RestVol={float(st.get('leak_rest_vol', 0.0)):.0f} mL  "
        f"Phase={breath}  Alarm={high}"
    )
    return line1, line2


def fot_impedance(
    st: dict[str, float | int], ph: dict[str, float | int]
) -> tuple[float, str]:
    """Airway impedance from the APAP probe when it runs, else the physio FOT probe.

    The physio probe runs in every mode, so CPAP sessions still get an impedance.
    """
    z = float(st.get("fot_z", 0.0))
    if z > 0.0:
        return z, "APAP FOT"
    p_pp = float(ph.get("p_pp", 0.0)) or float(st.get("physio_p_pp", 0.0))
    q_pp = float(ph.get("q_pp", 0.0)) or float(st.get("physio_q_pp", 0.0))
    if p_pp <= 0.0:
        return 0.0, DASH
    src = int(ph.get("fot_src", 0) or st.get("physio_fot_src", 0))
    label = "APAP FOT" if src == 2 else "physio FOT"
    if q_pp < 0.05:
        return 999.0, label
    return p_pp / q_pp, label


THERAPY_METRICS = (
    ("mode", "Mode"),
    ("state", "State"),
    ("target", "Target"),
    ("setpoint", "Setpoint"),
    ("measured", "Measured"),
    ("range", "Range"),
    ("epr", "EPR"),
    ("maskoff", "Mask off"),
    ("fault", "Fault"),
)

BREATH_METRICS = (
    ("rate", "Rate"),
    ("vt", "Vt"),
    ("insp", "Insp vol"),
    ("exp", "Exp vol"),
    ("mv", "Min vent"),
    ("phase", "Phase"),
    ("phase_vol", "Phase vol"),
    ("source", "Source"),
)

LEAK_METRICS = (
    ("algo", "Algorithm"),
    ("reported", "Reported"),
    ("uninten", "Unintent."),
    ("vent", "Vent ref"),
    ("flow", "Flow leak"),
    ("ratio", "Mean ratio"),
    ("ratio10", "Ratio 10 s"),
    ("restvol", "Rest vol"),
    ("alarm", "Alarm"),
)

AIRWAY_METRICS = (
    ("z", "Z_eff"),
    ("zsrc", "Z source"),
    ("r_eff", "R_eff"),
    ("c_proxy", "C_proxy"),
    ("p_pp", "dP p-p"),
    ("q_pp", "dQ p-p"),
    ("q_thr", "OSA thr"),
    ("apap_st", "APAP state"),
    ("verdict", "Airway"),
    ("treating", "Response"),
)

SIGNAL_METRICS = (
    ("p_sensor", "P sensor"),
    ("p_mask", "P mask"),
    ("flow", "Q patient"),
    ("rpm_cmd", "PID RPM cmd"),
    ("rpm_mech", "RPM mech"),
    ("rate_hz", "Telem rate"),
)


def therapy_metric_values(st: dict[str, float | int]) -> dict[str, str]:
    if not st:
        return {}
    mode = int(st.get("mode", 0))
    if int(st.get("epr", 0)):
        phase = {0: "--", 1: "inhale", 2: "exhale"}.get(int(st.get("epr_phase", 0)), "?")
        epr = f"L{int(st.get('epr_level', 0))} -{float(st.get('epr_relief', 0.0)):.1f} {phase}"
    else:
        epr = "Off"
    fault = int(st.get("fault", 0))
    return {
        "mode": THERAPY_MODE_NAMES.get(mode, str(mode)),
        "state": THERAPY_STATE_NAMES.get(int(st.get("state", -1)), str(st.get("state", "?"))),
        "target": f"{float(st.get('target', 0.0)):.1f} cmH2O",
        "setpoint": f"{float(st.get('setpoint', 0.0)):.1f} cmH2O",
        "measured": f"{float(st.get('measured', 0.0)):.2f} cmH2O",
        "range": (
            DASH
            if mode == 0
            else f"{float(st.get('pressure_min', 0.0)):.1f}-{float(st.get('pressure_max', 0.0)):.1f}"
        ),
        "epr": epr,
        "maskoff": "ACTIVE" if int(st.get("maskoff", 0)) else "no",
        "fault": "none" if fault == 0 else f"0x{fault:02X}",
    }


def breath_metric_values(
    st: dict[str, float | int],
    local: dict[str, float | int | bool],
    ph: dict[str, float | int],
) -> dict[str, str]:
    fw_rate = float(st.get("breath_rate", 0.0))
    fw_insp = float(st.get("breath_insp_vol", 0.0))
    fw_exp = float(st.get("breath_exp_vol", 0.0))
    use_fw = fw_rate > 0.0 or fw_insp > 0.0 or fw_exp > 0.0

    rate = fw_rate if use_fw else float(local.get("breath_rate_bpm", 0.0))
    insp = fw_insp if use_fw else float(local.get("last_insp_vol_ml", 0.0))
    exp = fw_exp if use_fw else float(local.get("last_exp_vol_ml", 0.0))
    phase_vol = (
        float(st.get("breath_phase_vol", 0.0)) if use_fw else float(local.get("phase_vol_ml", 0.0))
    )
    insp_phase = (
        bool(int(st.get("leak_breath_insp", 0)))
        if "leak_breath_insp" in st
        else bool(local.get("insp", False))
    )
    vt = float(ph.get("vt_ml", 0.0)) or insp
    rr = float(ph.get("rr_bpm", 0.0)) or rate
    mv = rr * vt / 1000.0

    return {
        "rate": f"{rate:.1f} bpm" if rate > 0.0 else DASH,
        "vt": f"{vt:.0f} mL" if vt > 0.0 else DASH,
        "insp": f"{insp:.0f} mL" if insp > 0.0 else DASH,
        "exp": f"{exp:.0f} mL" if exp > 0.0 else DASH,
        "mv": f"{mv:.1f} L/min" if mv > 0.0 else DASH,
        "phase": "Inspiration" if insp_phase else "Expiration",
        "phase_vol": f"{abs(phase_vol):.0f} mL",
        "source": "firmware" if use_fw else "PC detector",
    }


def leak_metric_values(st: dict[str, float | int]) -> dict[str, str]:
    if not st:
        return {}
    if "leak_algo" not in st:
        return {
            "algo": "legacy fw",
            "reported": f"{float(st.get('leak', 0.0)):.1f} slm",
        }
    uninten = float(st.get("leak_uninten", 0.0))
    if "leak_sev" in st:
        alarm = {2: "HIGH LEAK", 1: "small"}.get(int(st.get("leak_sev", 0)), "ok")
    else:
        alarm = "HIGH LEAK" if uninten > 24.0 else "ok"
    return {
        "algo": "Vivo table" if int(st.get("leak_algo", 0)) else "sqrt(P) LPF",
        "reported": f"{float(st.get('leak_reported', 0.0)):.1f} slm",
        "uninten": f"{uninten:.1f} slm",
        "vent": f"{float(st.get('leak_vent', 0.0)):.1f} slm",
        "flow": f"{float(st.get('leak_flow', 0.0)):.1f} slm",
        "ratio": f"{int(st.get('leak_mean_ratio', 0))}",
        "ratio10": f"{int(st.get('leak_mean_ratio_10s', 0))}",
        "restvol": f"{float(st.get('leak_rest_vol', 0.0)):.0f} mL",
        "alarm": alarm,
    }


def airway_metric_values(
    st: dict[str, float | int], ph: dict[str, float | int]
) -> dict[str, str]:
    z, z_src = fot_impedance(st, ph)
    r_eff = float(ph.get("r_eff", 0.0)) or float(st.get("physio_r_eff", 0.0))
    c_proxy = float(ph.get("c_proxy", 0.0)) or float(st.get("physio_c_proxy", 0.0))
    p_pp = (
        float(st.get("fot_p_pp", 0.0))
        or float(ph.get("p_pp", 0.0))
        or float(st.get("physio_p_pp", 0.0))
    )
    q_pp = (
        float(st.get("fot_q_pp", 0.0))
        or float(ph.get("q_pp", 0.0))
        or float(st.get("physio_q_pp", 0.0))
    )
    q_thr = float(st.get("fot_q_thr", 0.0))
    fot_src = int(ph.get("fot_src", 0) or st.get("physio_fot_src", 0))

    if "apap_st" in st:
        apap_st = APAP_STATE_NAMES.get(int(st.get("apap_st", 0)), str(st.get("apap_st")))
    else:
        apap_st = DASH

    if z <= 0.0:
        if int(st.get("state", 0)) in (1, 2):
            verdict = "waiting for FOT"
        else:
            verdict = "no probe yet"
    elif int(st.get("fot_osa", 0)):
        verdict = "OBSTRUCTED"
    elif int(st.get("fot_osc", 0)):
        verdict = "APAP probe"
    elif fot_src == 1:
        verdict = "physio FOT"
    elif "fot_osa" in st:
        verdict = "patent (last FOT)"
    else:
        verdict = {0: "held", 1: "probing", 2: "APAP probe"}.get(fot_src, "probing")

    return {
        "z": format_fot_impedance(z),
        "zsrc": z_src,
        "r_eff": f"{r_eff:.2f} cmH2O/slm" if r_eff > 0.0 else DASH,
        "c_proxy": f"{c_proxy:.0f} mL/cmH2O" if c_proxy > 0.0 else DASH,
        "p_pp": f"{p_pp:.2f} cmH2O" if p_pp > 0.0 else DASH,
        "q_pp": f"{q_pp:.2f} slm" if q_pp > 0.0 else DASH,
        "q_thr": f"{q_thr:.1f} slm" if q_thr > 0.0 else DASH,
        "apap_st": apap_st,
        "verdict": verdict,
        "treating": "raising P" if int(st.get("fot_treating", 0)) else "holding",
    }


def signal_metric_values(
    times: list[float],
    pressure: list[float],
    pressure_mask: list[float],
    flow: list[float],
    pid_rpm_cmd: list[int | None],
    mech_rpm: list[int | None],
) -> dict[str, str]:
    if not times:
        return {}

    def last_rpm(series: list[int | None]) -> str:
        for value in reversed(series):
            if value is not None:
                return f"{value} rpm"
        return DASH

    rate = DASH
    if len(times) > 1 and times[-1] > times[0]:
        rate = f"{(len(times) - 1) / (times[-1] - times[0]):.0f} Hz"

    return {
        "p_sensor": f"{pressure[-1]:.2f} cmH2O" if pressure else DASH,
        "p_mask": f"{pressure_mask[-1]:.2f} cmH2O" if pressure_mask else DASH,
        "flow": f"{flow[-1]:+.2f} slm" if flow else DASH,
        "rpm_cmd": last_rpm(pid_rpm_cmd),
        "rpm_mech": last_rpm(mech_rpm),
        "rate_hz": rate,
    }


class MetricPanel(ttk.LabelFrame):
    """Titled block of name/value readouts laid out as compact label pairs."""

    def __init__(
        self,
        master: tk.Misc,
        title: str,
        metrics: tuple[tuple[str, str], ...],
        pair_columns: int = 1,
        value_width: int = 15,
    ) -> None:
        super().__init__(master, text=title, padding=(6, 2))
        self._vars: dict[str, tk.StringVar] = {}
        name_font = ("TkDefaultFont", 8)
        value_font = ("TkDefaultFont", 8, "bold")
        for index, (key, caption) in enumerate(metrics):
            row, col = divmod(index, pair_columns)
            var = tk.StringVar(value=DASH)
            self._vars[key] = var
            ttk.Label(self, text=caption, font=name_font, anchor="e").grid(
                row=row, column=col * 2, sticky="e", padx=(0, 4)
            )
            ttk.Label(
                self, textvariable=var, font=value_font, anchor="w", width=value_width
            ).grid(row=row, column=(col * 2) + 1, sticky="w", padx=(0, 8))

    def update_values(self, values: dict[str, str]) -> None:
        for key, var in self._vars.items():
            var.set(values.get(key, DASH))


def configure_time_grid(ax: plt.Axes) -> None:
    """Major vertical grid every 5 s; thinner minor lines every 1 s."""
    ax.xaxis.set_major_locator(MultipleLocator(5))
    ax.xaxis.set_minor_locator(MultipleLocator(1))
    ax.grid(True, which="major", alpha=0.35)
    ax.grid(True, which="minor", axis="x", alpha=0.15, linewidth=0.6)


class BreathTracker:
    """Patient-flow breath detector for GUI metrics (matches firmware thresholds)."""

    INSP_ENTER_SLM = 2.0
    EXP_ENTER_SLM = -1.5
    INSP_CONFIRM_S = 0.080
    EXP_CONFIRM_S = 0.050

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.insp = False
        self.insp_confirm_s = 0.0
        self.exp_confirm_s = 0.0
        self.phase_vol_ml = 0.0
        self.last_insp_vol_ml = 0.0
        self.last_exp_vol_ml = 0.0
        self.breath_rate_bpm = 0.0
        self.prev_ms: int | None = None
        self.prev_insp_ms: int | None = None

    def update(self, flow_slm: float, ms: int) -> None:
        if self.prev_ms is None:
            self.prev_ms = ms
            return

        dt_s = (ms - self.prev_ms) / 1000.0
        self.prev_ms = ms
        if dt_s <= 0.0 or dt_s > 0.25:
            return

        want_insp = flow_slm >= self.INSP_ENTER_SLM
        want_exp = flow_slm <= self.EXP_ENTER_SLM

        if not self.insp:
            if want_insp:
                self.insp_confirm_s += dt_s
                self.exp_confirm_s = 0.0
                if self.insp_confirm_s >= self.INSP_CONFIRM_S:
                    self.last_exp_vol_ml = abs(self.phase_vol_ml)
                    self.phase_vol_ml = 0.0
                    if self.prev_insp_ms is not None:
                        dt_breath = (ms - self.prev_insp_ms) / 1000.0
                        if 1.0 <= dt_breath <= 20.0:
                            bpm = 60.0 / dt_breath
                            if self.breath_rate_bpm <= 0.0:
                                self.breath_rate_bpm = bpm
                            else:
                                self.breath_rate_bpm = 0.6 * self.breath_rate_bpm + 0.4 * bpm
                    self.prev_insp_ms = ms
                    self.insp = True
            else:
                self.insp_confirm_s = 0.0
        else:
            if want_exp:
                self.exp_confirm_s += dt_s
                self.insp_confirm_s = 0.0
                if self.exp_confirm_s >= self.EXP_CONFIRM_S:
                    self.last_insp_vol_ml = max(self.phase_vol_ml, 0.0)
                    self.phase_vol_ml = 0.0
                    self.insp = False
            else:
                self.exp_confirm_s = 0.0

        self.phase_vol_ml += flow_slm * (1000.0 / 60.0) * dt_s

    def snapshot(self) -> dict[str, float | int | bool]:
        return {
            "breath_rate_bpm": self.breath_rate_bpm,
            "last_insp_vol_ml": self.last_insp_vol_ml,
            "last_exp_vol_ml": self.last_exp_vol_ml,
            "phase_vol_ml": abs(self.phase_vol_ml),
            "insp": self.insp,
        }


def list_serial_ports(verbose: bool = False) -> list[str]:
    ports = []
    for p in serial.tools.list_ports.comports():
        ports.append(p.device)
        if verbose:
            print(f"{p.device}\t{p.description}\t{p.hwid}")
    return ports


def pick_cpap_serial_port() -> str | None:
    """Prefer STM32 USB-CDC over ST-Link UART when both are present."""
    ranked: list[tuple[int, str, str]] = []
    for p in serial.tools.list_ports.comports():
        desc = f"{p.description} {p.hwid}".lower()
        score = 0
        if "stmicroelectronics" in desc or "stm32" in desc:
            score += 20
        if "virtual com" in desc or "usb serial device" in desc:
            score += 10
        if "st-link" in desc or "stlink" in desc:
            score -= 30  # usually debug UART, not CDC telemetry
        if "bluetooth" in desc:
            score -= 50
        ranked.append((score, p.device, p.description))
    if not ranked:
        return None
    ranked.sort(key=lambda x: (-x[0], x[1]))
    best = ranked[0]
    print(f"Using port: {best[1]}  ({best[2]})  score={best[0]}")
    if best[0] < 0:
        print("Warning: this looks like ST-Link UART, not USB-CDC. Try --list.")
    return best[1]


class TelemetryReader:
    def __init__(
        self,
        port: str,
        baud: int,
        window_s: float,
        debug: bool = False,
        enable_on_open: bool = True,
        record_path: str | None = None,
    ) -> None:
        self.port = port
        self.baud = baud
        self.window_s = window_s
        self.debug = debug
        self.enable_on_open = enable_on_open
        self._lock = threading.Lock()
        self._write_lock = threading.Lock()
        self._t0_ms: int | None = None
        self._times: collections.deque[float] = collections.deque()
        self._pressure: collections.deque[float] = collections.deque()
        self._pressure_mask: collections.deque[float] = collections.deque()
        self._flow: collections.deque[float] = collections.deque()
        self._pid_rpm_cmd: collections.deque[int | None] = collections.deque()
        self._mech_rpm: collections.deque[int | None] = collections.deque()
        self._bench_times: collections.deque[float] = collections.deque()
        self._bench_rpm_cmd: collections.deque[int] = collections.deque()
        self._bench_rpm_act: collections.deque[int] = collections.deque()
        self._last_line = ""
        self._last_other = ""
        self._last_bench_line = ""
        self._last_ack = ""
        self._last_err = ""
        self._last_edf = ""
        self._last_apap_events = ""
        self._last_status: dict[str, float | int] = {}
        self._last_physio: dict[str, float | int] = {}
        self._samples = 0
        self._bench_events = 0
        self._bytes_in = 0
        self._lines_in = 0
        self._other_lines = 0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._ser: serial.Serial | None = None
        self._opened_at = 0.0
        self._record_path = record_path
        self._record_file = None
        # The reader thread writes to _record_file while the GUI thread may be
        # opening or closing it; guard both sides.
        self._record_lock = threading.Lock()
        self._echo_bench_lines = False
        self._breath = BreathTracker()

    def start(self) -> None:
        if self._record_path:
            self._record_file = open(self._record_path, "w", encoding="utf-8")
            self._record_file.write("# raw CPAP telemetry\n")
        # Avoid DTR/RTS reset quirks on STM32 VCP: open without toggling lines.
        self._ser = serial.Serial()
        self._ser.port = self.port
        self._ser.baudrate = self.baud
        self._ser.timeout = 0.1
        self._ser.dsrdtr = False
        self._ser.rtscts = False
        try:
            self._ser.setDTR(False)
            self._ser.setRTS(False)
        except Exception:
            pass
        self._ser.open()
        try:
            self._ser.setDTR(False)
            self._ser.setRTS(False)
        except Exception:
            pass
        self._opened_at = time.monotonic()
        # After flash/replug, CDC can take >0.4s to stream. Don't wipe early.
        time.sleep(1.0)
        if self.enable_on_open:
            self._ser.write(b"telem on\r\n")
            self._ser.flush()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def record_start(self, path: str) -> str:
        """Begin (or restart) capture to `path`. Returns a status string.

        Appends rather than truncates: a bench run is often several protocol
        cycles with the capture toggled around each, and silently discarding
        the earlier ones would be the wrong default."""
        with self._record_lock:
            if self._record_file is not None:
                self._record_file.close()
            self._record_file = open(path, "a", encoding="utf-8")
            self._record_file.write(
                f"# raw CPAP telemetry — capture started {datetime.now():%Y-%m-%d %H:%M:%S}\n"
            )
            self._record_file.flush()
            self._record_path = path
        return f"Recording to {path}"

    def record_stop(self) -> str:
        with self._record_lock:
            if self._record_file is None:
                return "Not recording"
            path = self._record_path
            self._record_file.close()
            self._record_file = None
        return f"Recording stopped ({path})"

    def is_recording(self) -> bool:
        with self._record_lock:
            return self._record_file is not None

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._ser is not None and self._ser.is_open:
            self._ser.close()
        with self._record_lock:
            if self._record_file is not None:
                self._record_file.close()
                self._record_file = None

    def send_command(self, command: str) -> bool:
        line = command.strip()
        if not line:
            return False
        with self._write_lock:
            if self._ser is None or not self._ser.is_open:
                with self._lock:
                    self._last_err = "serial closed"
                return False
            self._ser.write((line + "\r\n").encode("ascii", errors="ignore"))
            self._ser.flush()
        return True

    def snapshot(
        self,
    ) -> tuple[
        list[float],
        list[float],
        list[float],
        list[float],
        list[int | None],
        list[int | None],
        list[float],
        list[int],
        list[int],
        str,
        int,
        int,
        int,
        str,
        float,
        int,
    ]:
        with self._lock:
            return (
                list(self._times),
                list(self._pressure),
                list(self._pressure_mask),
                list(self._flow),
                list(self._pid_rpm_cmd),
                list(self._mech_rpm),
                list(self._bench_times),
                list(self._bench_rpm_cmd),
                list(self._bench_rpm_act),
                self._last_line,
                self._samples,
                self._bytes_in,
                self._lines_in,
                self._last_other,
                time.monotonic() - self._opened_at,
                self._bench_events,
            )

    def control_snapshot(self) -> tuple[str, str, dict[str, float | int]]:
        with self._lock:
            return self._last_ack, self._last_err, dict(self._last_status)

    def edf_apap_snapshot(self) -> tuple[str, str]:
        with self._lock:
            return self._last_edf, self._last_apap_events

    def breath_snapshot(self) -> dict[str, float | int | bool]:
        with self._lock:
            return self._breath.snapshot()

    def physio_snapshot(self) -> dict[str, float | int]:
        with self._lock:
            return dict(self._last_physio)

    def _time_s(self, ms: int) -> float:
        if self._t0_ms is None:
            self._t0_ms = ms
        return (ms - self._t0_ms) / 1000.0

    def _trim_window(self) -> None:
        while self._times and (self._times[-1] - self._times[0]) > self.window_s:
            self._times.popleft()
            self._pressure.popleft()
            self._pressure_mask.popleft()
            self._flow.popleft()
            self._pid_rpm_cmd.popleft()
            self._mech_rpm.popleft()
        while self._bench_times and self._times and self._bench_times[0] < self._times[0]:
            self._bench_times.popleft()
            self._bench_rpm_cmd.popleft()
            self._bench_rpm_act.popleft()

    def _append_sample(
        self,
        ms: int,
        pressure: float,
        flow: float,
        pid_rpm_cmd: int | None,
        mech_rpm: int | None,
        pressure_mask: float | None = None,
    ) -> None:
        t_s = self._time_s(ms)

        with self._lock:
            self._times.append(t_s)
            self._pressure.append(pressure)
            if pressure_mask is not None:
                self._pressure_mask.append(pressure_mask)
            elif self._pressure_mask:
                self._pressure_mask.append(self._pressure_mask[-1])
            else:
                self._pressure_mask.append(pressure)
            self._flow.append(flow)
            self._pid_rpm_cmd.append(pid_rpm_cmd)
            self._mech_rpm.append(mech_rpm)
            self._breath.update(flow, ms)
            self._samples += 1
            parts = [f"T,{ms},{pressure:.2f},{flow:.2f}"]
            if pid_rpm_cmd is not None:
                parts.append(str(pid_rpm_cmd))
                if mech_rpm is not None:
                    parts.append(str(mech_rpm))
            if pressure_mask is not None:
                parts.append(f"{pressure_mask:.2f}")
            self._last_line = ",".join(parts)
            self._trim_window()

    def _append_bench(self, ms: int, rpm_cmd: int, rpm_act: int, pressure: float) -> None:
        t_s = self._time_s(ms)

        with self._lock:
            self._bench_times.append(t_s)
            self._bench_rpm_cmd.append(rpm_cmd)
            self._bench_rpm_act.append(rpm_act)
            self._bench_events += 1
            bench_line = f"R,{ms},{rpm_cmd},{rpm_act},{pressure:.2f}"
            self._last_line = bench_line
            self._last_bench_line = bench_line
            self._trim_window()

    def _handle_line(self, line: bytes) -> None:
        with self._lock:
            self._lines_in += 1

        if self.debug:
            print(line.decode("utf-8", errors="replace"))

        with self._record_lock:
            if self._record_file is not None:
                self._record_file.write(line.decode("utf-8", errors="replace") + "\n")
                self._record_file.flush()

        m = TELEM_RE.match(line)
        if m:
            ms = int(m.group(1))
            pressure = float(m.group(2))
            flow = float(m.group(3))
            pid_rpm_cmd = int(m.group(4)) if m.group(4) is not None else None
            mech_rpm = int(m.group(5)) if m.group(5) is not None else None
            pressure_mask = float(m.group(6)) if m.group(6) is not None else None
            self._append_sample(ms, pressure, flow, pid_rpm_cmd, mech_rpm, pressure_mask)
            return

        m = BENCH_RE.match(line)
        if m:
            ms = int(m.group(1))
            rpm_cmd = int(m.group(2))
            rpm_act = int(m.group(3))
            pressure = float(m.group(4))
            self._append_bench(ms, rpm_cmd, rpm_act, pressure)
            return

        if line.startswith(b"PH,"):
            parts = line.decode("utf-8", errors="replace").strip().split(",")
            if len(parts) >= 9:
                try:
                    p_pp = float(parts[7])
                    q_pp = float(parts[8])
                    r_eff = float(parts[3])
                    c_proxy = float(parts[4])
                    with self._lock:
                        prev = self._last_physio
                        if p_pp <= 0.0:
                            p_pp = float(prev.get("p_pp", 0.0))
                            q_pp = float(prev.get("q_pp", 0.0))
                        if r_eff <= 0.0:
                            r_eff = float(prev.get("r_eff", 0.0))
                        if c_proxy <= 0.0:
                            c_proxy = float(prev.get("c_proxy", 0.0))
                        self._last_physio = {
                            "ms": int(float(parts[1])),
                            "vt_ml": float(parts[2]),
                            "r_eff": r_eff,
                            "c_proxy": c_proxy,
                            "rr_bpm": float(parts[5]),
                            "fot_src": int(float(parts[6])),
                            "p_pp": p_pp,
                            "q_pp": q_pp,
                        }
                        self._last_line = line.decode("utf-8", errors="replace")
                    return
                except ValueError:
                    pass

        m = STATUS_RE.match(line)
        if m:
            with self._lock:
                self._last_status = {
                    "ms": int(m.group(1)),
                    "state": int(m.group(2)),
                    "target": float(m.group(3)),
                    "setpoint": float(m.group(4)),
                    "measured": float(m.group(5)),
                    "leak": float(m.group(6)),
                    "epr": int(m.group(7)),
                    "maskoff": int(m.group(8)),
                    "fault": int(m.group(9)),
                }
                if m.group(10) is not None:
                    self._last_status["mode"] = int(m.group(10))
                    self._last_status["pressure_min"] = float(m.group(11))
                    self._last_status["pressure_max"] = float(m.group(12))
                if m.group(13) is not None:
                    self._last_status["apap_st"] = int(m.group(13))
                    self._last_status["fot_osc"] = int(m.group(14))
                    self._last_status["fot_p_pp"] = float(m.group(15))
                    self._last_status["fot_q_pp"] = float(m.group(16))
                    self._last_status["fot_z"] = float(m.group(17))
                    self._last_status["fot_osa"] = int(m.group(18))
                    self._last_status["fot_q_thr"] = float(m.group(19))
                    self._last_status["fot_treating"] = int(m.group(20))
                if m.group(21) is not None:
                    self._last_status["epr_level"] = int(m.group(21))
                    self._last_status["epr_relief"] = float(m.group(22))
                    self._last_status["epr_phase"] = int(m.group(23))
                if m.group(24) is not None:
                    self._last_status["leak_algo"] = int(m.group(24))
                    self._last_status["leak_mean_ratio"] = int(m.group(25))
                    self._last_status["leak_mean_ratio_10s"] = int(m.group(26))
                    self._last_status["leak_vent"] = float(m.group(27))
                    self._last_status["leak_uninten"] = float(m.group(28))
                    self._last_status["leak_flow"] = float(m.group(29))
                    self._last_status["leak_reported"] = float(m.group(30))
                    self._last_status["leak_rest_vol"] = float(m.group(31))
                    self._last_status["leak_breath_insp"] = int(m.group(32))
                if m.group(33) is not None:
                    self._last_status["breath_rate"] = float(m.group(33))
                    self._last_status["breath_insp_vol"] = float(m.group(34))
                    self._last_status["breath_exp_vol"] = float(m.group(35))
                    self._last_status["breath_phase_vol"] = float(m.group(36))
                if m.group(37) is not None:
                    self._last_status["leak_sev"] = int(m.group(37))
                    self._last_status["leak_pct"] = float(m.group(38))
                # Trailing physio FOT: src, R_eff, C_proxy, dP, dQ (indices 76-80).
                parts = line.decode("utf-8", errors="replace").strip().split(",")
                if len(parts) >= 81:
                    try:
                        fot_src = int(float(parts[76]))
                        r_eff = float(parts[77])
                        c_proxy = float(parts[78])
                        p_pp = float(parts[79])
                        q_pp = float(parts[80])
                    except ValueError:
                        fot_src = 0
                        r_eff = c_proxy = p_pp = q_pp = 0.0
                    else:
                        self._last_status["physio_fot_src"] = fot_src
                        self._last_status["physio_r_eff"] = r_eff
                        self._last_status["physio_c_proxy"] = c_proxy
                        self._last_status["physio_p_pp"] = p_pp
                        self._last_status["physio_q_pp"] = q_pp
                        prev = self._last_physio
                        if p_pp <= 0.0:
                            p_pp = float(prev.get("p_pp", 0.0))
                            q_pp = float(prev.get("q_pp", 0.0))
                        if r_eff <= 0.0:
                            r_eff = float(prev.get("r_eff", 0.0))
                        if c_proxy <= 0.0:
                            c_proxy = float(prev.get("c_proxy", 0.0))
                        if fot_src == 0:
                            fot_src = int(prev.get("fot_src", 0))
                        self._last_physio = {
                            "ms": int(self._last_status.get("ms", 0)),
                            "vt_ml": float(prev.get("vt_ml", 0.0)),
                            "r_eff": r_eff,
                            "c_proxy": c_proxy,
                            "rr_bpm": float(prev.get("rr_bpm", 0.0)),
                            "fot_src": fot_src,
                            "p_pp": p_pp,
                            "q_pp": q_pp,
                        }
                self._last_line = line.decode("utf-8", errors="replace")
            return

        if line.startswith(b"# ack "):
            with self._lock:
                self._last_ack = line.decode("utf-8", errors="replace")
                self._last_err = ""
                self._last_line = self._last_ack
            return

        if line.startswith(b"# err "):
            with self._lock:
                self._last_err = line.decode("utf-8", errors="replace")
                self._last_line = self._last_err
            return

        if line.startswith(b"# EDF "):
            with self._lock:
                self._last_edf = line.decode("utf-8", errors="replace")
                self._last_line = self._last_edf
            return

        if line.startswith(b"# APAP "):
            with self._lock:
                self._last_apap_events = line.decode("utf-8", errors="replace")
                self._last_line = self._last_apap_events
            return

        with self._lock:
            self._other_lines += 1
            text = line.decode("utf-8", errors="replace")
            if text:
                self._last_other = text

    def _run(self) -> None:
        assert self._ser is not None
        buf = bytearray()
        while not self._stop.is_set():
            chunk = self._ser.read(512)
            if not chunk:
                continue
            with self._lock:
                self._bytes_in += len(chunk)
            buf.extend(chunk)
            while True:
                nl = buf.find(b"\n")
                if nl < 0:
                    break
                line = bytes(buf[:nl]).strip(b"\r")
                del buf[: nl + 1]
                if line:
                    self._handle_line(line)


def run_monitor(reader: TelemetryReader, duration: float, record_path: str | None) -> int:
    reader._echo_bench_lines = True
    print(f"Monitoring {reader.port} for {duration:.0f}s (Ctrl+C to stop early)...")
    if record_path:
        print(f"Recording raw T/R lines to: {record_path}")
    print("Expect lines like: T,12345,8.12,3.45,17500,17420,7.95  or  R,12345,17500,17480,7.50")
    print("Or debug text like: [0.001] I [SYS] debug log online")
    print("-" * 60)
    t_end = time.monotonic() + duration
    try:
        while time.monotonic() < t_end:
            (
                _t,
                _p,
                _pm,
                _f,
                _rpm,
                _mech,
                _bt,
                _bc,
                _ba,
                last,
                samples,
                nbytes,
                nlines,
                other,
                elapsed,
                bench,
            ) = reader.snapshot()
            if reader._echo_bench_lines and reader._last_bench_line:
                print(f"\n{reader._last_bench_line}")
                reader._last_bench_line = ""
            status = (
                f"[{elapsed:5.1f}s] bytes={nbytes:6d} lines={nlines:4d} "
                f"telem={samples:4d} bench={bench:3d} other={reader._other_lines}"
            )
            if last:
                status += f"  last={last}"
            elif other:
                status += f"  last_other={other[:60]}"
            print(status, end="\r", flush=True)
            time.sleep(0.25)
    except KeyboardInterrupt:
        print()

    (
        _t,
        _p,
        _pm,
        _f,
        _rpm,
        _mech,
        _bt,
        _bc,
        _ba,
        last,
        samples,
        nbytes,
        nlines,
        other,
        elapsed,
        bench,
    ) = reader.snapshot()
    print(
        f"Done in {elapsed:.1f}s: {nbytes} bytes, {nlines} lines, "
        f"{samples} telem, {bench} bench"
    )
    if nbytes == 0:
        print("\nNo data received. Check:")
        print("  1. Correct COM port? STM32 boards often expose TWO ports")
        print("     (ST-Link UART vs USB-CDC). Use --list for descriptions.")
        print("  2. Firmware rebuilt/flashed with telemetry_stream.c?")
        print("  3. Close other serial terminals (PuTTY, Keil debug, etc.)")
        print("  4. Replug USB after flashing")
        return 1
    if samples == 0 and reader._other_lines > 0:
        print("\nSerial works but no T,... lines yet.")
        print("  - g_telem_enabled should be 1 (default after latest firmware)")
        print("  - Sent 'telem on' on connect; try again after reset")
        if other:
            print(f"  - Last non-telem line: {other}")
        return 2
    if samples > 0:
        print(f"Telemetry OK. Last line: {last}")
        return 0
    return 2


def main() -> int:
    parser = argparse.ArgumentParser(description="CPAP live telemetry plotter")
    parser.add_argument("--port", help="Serial port (e.g. COM13)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--window",
        type=float,
        default=30.0,
        help="Rolling plot window in seconds (default 30)",
    )
    parser.add_argument("--list", action="store_true", help="List serial ports and exit")
    parser.add_argument(
        "--monitor",
        action="store_true",
        help="Text-only monitor (no plot) to verify serial data",
    )
    parser.add_argument(
        "--monitor-time",
        type=float,
        default=15.0,
        help="Seconds for --monitor mode (default 15)",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Print every received line to the console",
    )
    parser.add_argument(
        "--no-enable",
        action="store_true",
        help="Do not send 'telem on' when opening the port",
    )
    parser.add_argument(
        "--record",
        metavar="FILE",
        help="Write raw T/R lines to FILE (works with --monitor or plot mode)",
    )
    args = parser.parse_args()

    if args.list:
        if not list_serial_ports(verbose=True):
            print("No serial ports found.")
        return 0

    port = args.port
    if port is None:
        port = pick_cpap_serial_port()
        if port is None:
            print("No serial ports found. Use --port COMx", file=sys.stderr)
            return 1
    else:
        print(f"Using port: {port}")

    reader = TelemetryReader(
        port,
        args.baud,
        args.window,
        debug=args.debug,
        enable_on_open=not args.no_enable,
        record_path=args.record,
    )
    try:
        reader.start()
    except serial.SerialException as exc:
        print(f"Cannot open {port}: {exc}", file=sys.stderr)
        print("Close other programs using this COM port and try again.", file=sys.stderr)
        return 1

    if args.monitor:
        try:
            return run_monitor(reader, args.monitor_time, args.record)
        finally:
            reader.stop()

    root = tk.Tk()
    root.title(f"CPAP Telemetry and Therapy Control - {port}")
    root.geometry("1100x900")
    root.minsize(1000, 720)

    notebook = ttk.Notebook(root)
    notebook.pack(fill=tk.BOTH, expand=True)

    plot_tab = ttk.Frame(notebook)
    control_tab = ttk.Frame(notebook)
    foc_tab = ttk.Frame(notebook, padding=12)
    apap_edf_tab = ttk.Frame(notebook)
    notebook.add(plot_tab, text="Plots")
    notebook.add(control_tab, text="Therapy Controls")
    notebook.add(foc_tab, text="FOC Tuning")
    notebook.add(apap_edf_tab, text="APAP / EDF")

    control_canvas = tk.Canvas(control_tab, highlightthickness=0, borderwidth=0)
    control_scrollbar = ttk.Scrollbar(control_tab, orient=tk.VERTICAL, command=control_canvas.yview)
    control_inner = ttk.Frame(control_canvas, padding=12)
    control_inner_id = control_canvas.create_window((0, 0), window=control_inner, anchor="nw")

    def _on_control_inner_configure(_event: object = None) -> None:
        control_canvas.configure(scrollregion=control_canvas.bbox("all"))

    def _on_control_canvas_configure(event: tk.Event) -> None:
        control_canvas.itemconfigure(control_inner_id, width=event.width)

    def _on_control_mousewheel(event: tk.Event) -> None:
        control_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

    control_inner.bind("<Configure>", _on_control_inner_configure)
    control_canvas.bind("<Configure>", _on_control_canvas_configure)
    control_canvas.configure(yscrollcommand=control_scrollbar.set)
    control_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    control_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
    control_canvas.bind("<Enter>", lambda _e: control_canvas.bind_all("<MouseWheel>", _on_control_mousewheel))
    control_canvas.bind("<Leave>", lambda _e: control_canvas.unbind_all("<MouseWheel>"))

    fig = Figure(figsize=(10, 6.2), dpi=100)
    fig.subplots_adjust(left=0.07, right=0.96, top=0.94, bottom=0.08, hspace=0.28)
    ax_p = fig.add_subplot(2, 1, 1)
    ax_f = fig.add_subplot(2, 1, 2, sharex=ax_p)
    fig.suptitle(f"CPAP telemetry — {port}")
    ax_p.set_ylabel("Pressure (cmH2O)")
    ax_p.set_ylim(0, 25)
    ax_p.set_xlim(0, args.window)
    configure_time_grid(ax_p)
    ax_rpm = ax_p.twinx()
    ax_rpm.set_ylabel("RPM")
    ax_rpm.set_ylim(0, 22000)
    ax_f.set_ylabel("Patient flow (slm SFM)")
    ax_f.set_xlabel("Time (s)")
    ax_f.set_ylim(-30, 30)
    ax_f.set_xlim(0, args.window)
    configure_time_grid(ax_f)

    line_p, = ax_p.plot([], [], "b-", linewidth=1.2, label="P_sensor")
    line_p_mask, = ax_p.plot([], [], color="teal", linewidth=1.2, label="P_mask (est.)")
    line_pid_rpm, = ax_rpm.plot(
        [], [], color="darkorange", linewidth=1.0, drawstyle="steps-post", label="PID RPM cmd"
    )
    line_mech_rpm, = ax_rpm.plot(
        [], [], color="crimson", linewidth=1.2, label="RPM mech"
    )
    line_rpm_cmd, = ax_rpm.plot(
        [], [], "x", color="saddlebrown", markersize=4, linestyle="none", label="bench RPM cmd"
    )
    line_rpm_act, = ax_rpm.plot(
        [], [], "o", color="mediumpurple", markersize=4, linestyle="none", label="bench RPM act"
    )
    line_f, = ax_f.plot([], [], "g-", linewidth=1.2, label="Q_patient")
    ax_p.legend(loc="upper left")
    ax_rpm.legend(loc="upper right")
    ax_f.legend(loc="upper right")
    waiting = ax_p.text(
        0.5,
        0.5,
        "Waiting for T,... telemetry lines",
        transform=ax_p.transAxes,
        ha="center",
        va="center",
        fontsize=11,
        color="gray",
    )

    plot_status = ttk.Frame(plot_tab)
    serial_plot_var = tk.StringVar(value="Serial: —")
    metrics_visible_var = tk.BooleanVar(value=True)

    status_poll_counter = [0]

    canvas = FigureCanvasTkAgg(fig, master=plot_tab)
    toolbar = NavigationToolbar2Tk(canvas, plot_tab, pack_toolbar=False)
    toolbar.update()

    metrics_row = ttk.Frame(plot_status)
    therapy_panel = MetricPanel(metrics_row, "Therapy", THERAPY_METRICS, value_width=14)
    breath_panel = MetricPanel(metrics_row, "Breathing", BREATH_METRICS, value_width=11)
    leak_panel = MetricPanel(metrics_row, "Leak", LEAK_METRICS, value_width=11)
    airway_panel = MetricPanel(metrics_row, "Airway mechanics / FOT", AIRWAY_METRICS, value_width=15)
    signal_panel = MetricPanel(metrics_row, "Live signals", SIGNAL_METRICS, value_width=11)
    metric_panels = (therapy_panel, breath_panel, leak_panel, airway_panel, signal_panel)
    for column, panel in enumerate(metric_panels):
        panel.grid(row=0, column=column, sticky="nsw", padx=(0, 6))
    metrics_row.pack(side=tk.TOP, fill=tk.X, pady=(0, 2))

    footer = ttk.Frame(plot_status)
    ttk.Checkbutton(footer, text="Show measured parameters", variable=metrics_visible_var).pack(
        side=tk.RIGHT
    )
    ttk.Label(
        footer,
        textvariable=serial_plot_var,
        anchor="w",
        justify=tk.LEFT,
        font=("TkDefaultFont", 8),
    ).pack(side=tk.LEFT, fill=tk.X, expand=True)
    footer.pack(side=tk.TOP, fill=tk.X)

    def on_metrics_visibility() -> None:
        if metrics_visible_var.get():
            metrics_row.pack(side=tk.TOP, fill=tk.X, pady=(0, 2), before=footer)
        else:
            metrics_row.pack_forget()

    metrics_visible_var.trace_add("write", lambda *_: on_metrics_visibility())

    canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
    plot_status.pack(side=tk.BOTTOM, fill=tk.X, padx=4, pady=(0, 2))
    toolbar.pack(side=tk.BOTTOM, fill=tk.X)

    def refresh_metric_panels() -> None:
        _, _, st = reader.control_snapshot()
        physio = reader.physio_snapshot()
        therapy_panel.update_values(therapy_metric_values(st))
        breath_panel.update_values(breath_metric_values(st, reader.breath_snapshot(), physio))
        leak_panel.update_values(leak_metric_values(st))
        airway_panel.update_values(airway_metric_values(st, physio))
        leak_live_var.set(format_leak_status(st))

    cmd_status = tk.StringVar(value="Ready")
    ack_status = tk.StringVar(value="")
    live_status = tk.StringVar(value="No therapy status yet")
    mode_var = tk.StringVar(value="CPAP")
    pressure_var = tk.StringVar(value="8.0")
    apap_min_var = tk.StringVar(value="5.0")
    apap_max_var = tk.StringVar(value="20.0")
    ramp_var = tk.StringVar(value="0")
    epr_var = tk.BooleanVar(value=False)
    epr_type_var = tk.StringVar(value="fulltime")
    epr_level_var = tk.IntVar(value=1)
    maskoff_var = tk.BooleanVar(value=True)
    smart_start_var = tk.BooleanVar(value=False)
    smart_stop_var = tk.BooleanVar(value=False)
    tube_mode_var = tk.StringVar(value="Auto")
    tube_temp_var = tk.StringVar(value="27")
    climate_var = tk.StringVar(value="Auto")
    humidity_var = tk.StringVar(value="4")
    airplane_var = tk.BooleanVar(value=False)
    loop_enable_var = tk.BooleanVar(value=True)
    pid_enable_var = tk.BooleanVar(value=True)
    rpm_floor_var = tk.BooleanVar(value=False)
    cpap_plateau_var = tk.BooleanVar(value=False)
    cpap_hybrid_var = tk.BooleanVar(value=False)
    physio_fot_var = tk.BooleanVar(value=True)
    # Bench switch: freeze the integral trim so the baseline stops wandering
    # with breathing. Sends immediately rather than riding on Apply, so it can
    # be flipped mid-breath while watching the trace.
    integ_hold_var = tk.BooleanVar(value=False)
    tubecomp_enable_var = tk.BooleanVar(value=True)
    manual_rpm_var = tk.StringVar(value="12000")
    manual_ramp_ms_var = tk.StringVar(value="5000")
    leak_vivo_var = tk.BooleanVar(value=True)
    leak_pause_lpf_var = tk.BooleanVar(value=False)
    leak_standard_ratio_var = tk.StringVar(value="1000")
    leak_patient_lpf_var = tk.StringVar(value="2.0")
    leak_high_threshold_var = tk.StringVar(value="24.0")
    leak_simple_vent_k_var = tk.StringVar(value="8.7")
    leak_simple_tau_var = tk.StringVar(value="10.0")
    leak_live_var = tk.StringVar(value="Leak monitor: waiting for status…")
    flow_units_var = tk.StringVar(value="SLM")  # SLM (SFM3300) or BTPS
    blower_vars: dict[str, tk.StringVar | tk.BooleanVar] = {
        "kp_low": tk.StringVar(value="1800"),
        "ki_low": tk.StringVar(value="4500"),
        "kd_low": tk.StringVar(value="2.0"),
        "kp_mid": tk.StringVar(value="1200"),
        "ki_mid": tk.StringVar(value="4200"),
        "kd_mid": tk.StringVar(value="1.5"),
        "kp_high": tk.StringVar(value="1000"),
        "ki_high": tk.StringVar(value="3900"),
        "kd_high": tk.StringVar(value="0.8"),
        "ff_enable": tk.BooleanVar(value=False),
        "ff_k": tk.StringVar(value="5933"),   # 2026-10-22 refit: TKFan BA7060H24B, no box
        "flow_ff_enable": tk.BooleanVar(value=False),
        "dip_ff_enable": tk.BooleanVar(value=True),
        "physics_ff_enable": tk.BooleanVar(value=True),
        "physics_ff_c_fan": tk.StringVar(value="0.0008"),   # 2026-10-05 ~150 slm element set
        "physics_ff_c_fan_exp": tk.StringVar(value="0.0001"),   # 2026-10-22 TKFan: expiratory side (sweep)
        "physics_ff_flow_lpf_hz": tk.StringVar(value="4"),
        "physics_ff_flow_lpf_inhale_hz": tk.StringVar(value="3"),
        "physics_ff_rise": tk.StringVar(value="20000"),   # 2026-10-02 (was 6000)
        "physics_ff_fall": tk.StringVar(value="12000"),   # 2026-10-05 (was 4000)
        "physics_ff_fall_exhale": tk.StringVar(value="8000"),
        "physics_ff_flow_lpf_exhale_hz": tk.StringVar(value="3"),
        "physics_ff_exp_tail_flow_slm": tk.StringVar(value="6"),  # 2026-10-06 firmware default
        "physics_ff_inhale_rise_boost": tk.StringVar(value="2"),
        "under_p_gain": tk.StringVar(value="5"),   # 2026-10-02 (was 3.5)
        "ff_offset": tk.StringVar(value="628"),   # 2026-10-22 TKFan refit with ff_k 5933 (slope 0)
        # Inspiratory onset lead. Fires on dQ/dt so it leads the flow
        # magnitude term. (An older thresh of 80 slm/s was above what a 25 slm
        # bench breath produces, ~40 slm/s peak; firmware default is now 20.)
        "physics_ff_onset_thresh": tk.StringVar(value="20"),
        "physics_ff_onset_k": tk.StringVar(value="20"),
        "physics_ff_onset_max": tk.StringVar(value="2500"),
        "physics_ff_onset_dqdt_tau": tk.StringVar(value="0.03"),
        "physics_ff_onset_bypass_slew": tk.BooleanVar(value=True),
        # Dip FF lead, gated on dP_sensor/dt (firmware default 1.5 cmH2O/s;
        # a bench breath falls at ~1.6). err_min 0.30 keeps it off noise.
        "dip_ff_thresh": tk.StringVar(value="1.5"),
        "dip_ff_k": tk.StringVar(value="400"),
        "dip_ff_max": tk.StringVar(value="1500"),
        "dip_ff_err_min": tk.StringVar(value="0.30"),
        # Expiratory RPM floors — clear to let the loop pull pressure down
        # during expiration instead of waiting for the bump to bleed off.
        "ff_ss_floor_enable": tk.BooleanVar(value=False),
        "exhale_integ_clamp_enable": tk.BooleanVar(value=False),
        # Floor 4 — pins RPM at the resting-vent-flow speed through the whole
        # breath, and scales with sqrt K, so it binds hardest on expiration.
        "vent_floor_enable": tk.BooleanVar(value=False),
        # Freeze-only-slews-up branch in physicsFfStep(). Firmware default is
        # ON; suspected cause of the second expiratory pressure hump.
        "physics_ff_exp_tail_hold_enable": tk.BooleanVar(value=True),
        # Measured cause of the second expiratory hump: zeroes a negative
        # integrator mid-expiration, discarding the correction holding
        # pressure down. Firmware default is now OFF.
        "exp_neg_i_clamp_enable": tk.BooleanVar(value=False),
        # Expiratory rise FF - mirror of dip FF, brakes a rising pressure.
        # Off by default; the piece missing from expiratory rejection.
        "rise_ff_enable": tk.BooleanVar(value=True),
        # Expiratory flow FF, phase-lead compensated. k and lead derived from
        # measurement (22.8 rpm/slm, excess leads flow by 200 ms).
        "exp_flow_ff_enable": tk.BooleanVar(value=False),  # 2026-10-06 firmware default off
        # Low-pass the error feeding the integrator, so it stops chasing the
        # breath cycle (the cause of the end-inspiratory spike) while still
        # trimming steady state. Replaces the deadband, which deadlocks.
        "integ_err_lpf_enable": tk.BooleanVar(value=True),
        "integ_err_lpf_tau": tk.StringVar(value="15"),
        "integ_err_lpf_tau_start": tk.StringVar(value="1.0"),
        "integ_err_lpf_warmup": tk.StringVar(value="45"),
        "integ_err_lpf_band": tk.StringVar(value="0.15"),
        "exp_flow_ff_k": tk.StringVar(value="23"),
        "exp_flow_ff_lead": tk.StringVar(value="0.20"),
        "exp_flow_ff_max": tk.StringVar(value="800"),
        "rise_ff_k": tk.StringVar(value="400"),
        "rise_ff_thresh": tk.StringVar(value="1.5"),
        "rise_ff_max": tk.StringVar(value="1500"),
        "rise_ff_err_min": tk.StringVar(value="0.30"),
        # Integrator deadband is dormant in firmware unless this is set.
        "integ_deadband_enable": tk.BooleanVar(value=False),
        # Tube compensation (sent as "tubecomp set R_lam/R_turb", not blower
        # set). Firmware's calibrated values - NOT the physics-FF hose basis
        # 0.002 / 0.00022, which is a separate, frozen parameter.
        "tube_R_lam": tk.StringVar(value="0.00353"),
        "tube_R_turb": tk.StringVar(value="0.000118"),
        "flow_ff_k": tk.StringVar(value="58"),
        "flow_ff_max": tk.StringVar(value="3000"),
        "flow_ff_deadband": tk.StringVar(value="8"),   # 2026-10-05 ~150 slm element (was 3)
        "flow_ff_rise": tk.StringVar(value="10000"),
        "flow_ff_fall": tk.StringVar(value="7000"),
        "over_p_gain": tk.StringVar(value="4.0"),   # 2026-10-02 evening 6 -> 4 (delay-calibrated twin + bench + breathing)
        # Over-pressure threshold (arms the over_p_gain boost and the brake):
        # rest value, value while breathing, and how long after the last real
        # breath the breathing value holds (firmware 2026-10-01).
        "over_p_thresh": tk.StringVar(value="0.30"),
        "over_p_thresh_exp": tk.StringVar(value="0.12"),
        "over_p_exp_window": tk.StringVar(value="10"),
        "under_p_thresh": tk.StringVar(value="0.08"),
        # Post-hand-over speed ramp, rpm/s (0 = off). Firmware 2026-10-01.
        "start_ramp": tk.StringVar(value="12000"),
        "over_brake": tk.StringVar(value="6000"),
        "slew_down": tk.StringVar(value="18"),
        "slew_up": tk.StringVar(value="18"),
        "out_alpha": tk.StringVar(value="1.0"),
        "integ_deadband": tk.StringVar(value="1.2"),
        "integ_max": tk.StringVar(value="4000"),
        "meas_alpha": tk.StringVar(value="0.45"),
        "deriv_alpha": tk.StringVar(value="0.25"),
        # Pressure-loop command cap = BLOWER_IPC_MAX_SPEED_RPM. The therapy
        # ceiling is safety safe_rpm_max (38500), not this.
        "rpm_max": tk.StringVar(value="38500"),   # 2026-10-05 (was 43000 = drive limit): therapy runs at the 38500 safety ceiling; Apply mid-therapy must not lift it
    }
    foc_vars: dict[str, tk.StringVar] = {
        "speed_kp": tk.StringVar(value="1637"),   # 4000 tripped DP_FAULT in therapy, see drive_parameters.h
        "speed_ki": tk.StringVar(value="1024"),  # drive_parameters.h since 2026-09-25
        "torque_kp": tk.StringVar(value="2168"),
        "torque_ki": tk.StringVar(value="806"),
        "flux_kp": tk.StringVar(value="2168"),
        "flux_ki": tk.StringVar(value="806"),
        # Speed-PI current limit, mA (IQMAX_A boot default; CM4 clamps
        # 500-1800). Runtime since 2026-10-01 - see drive_parameters.h.
        "iq_max_ma": tk.StringVar(value="1445"),
        # Speed-dependent boost on top (CM4 boot defaults, blower_ipc_cm4.c):
        # boost current up to lo rpm, tapering to iq_max by hi rpm. 0 = off.
        "iq_boost_ma": tk.StringVar(value="1800"),   # 2026-10-02 (was 1650)
        "iq_boost_lo_rpm": tk.StringVar(value="30000"),   # TKFan: ~88 % modulation under load
        "iq_boost_hi_rpm": tk.StringVar(value="31500"),
    }

    state_names = THERAPY_STATE_NAMES
    therapy_mode_names = THERAPY_MODE_NAMES
    therapy_mode_commands = {
        "CPAP": "cpap",
        "Autoset": "autoset",
        "Autoset for her": "autoset_her",
    }

    def send(command: str) -> None:
        if reader.send_command(command):
            cmd_status.set(f"Sent: {command}")
        else:
            cmd_status.set(f"Send failed: {command}")

    def ramp_minutes() -> int:
        text = ramp_var.get()
        if text.startswith("Auto"):
            return 5
        try:
            return int(text)
        except ValueError:
            return 0

    FF_PHYSICS_DISPLACE = ("ff_enable", "flow_ff_enable")
    _ff_mutex_sync = False
    legacy_ff_checkbuttons: list[ttk.Checkbutton] = []
    physics_ff_checkbutton: ttk.Checkbutton | None = None
    dip_ff_checkbutton: ttk.Checkbutton | None = None
    _plateau_gui_snap: dict[str, object] | None = None
    _hybrid_gui_snap: dict[str, object] | None = None
    _profile_gui_sync = False

    PLATEAU_GUI_PRESET: dict[str, object] = {
        "ff_enable": True,
        "flow_ff_enable": False,
        "dip_ff_enable": False,
        "physics_ff_enable": False,
        "under_p_gain": "1.0",
        "over_p_gain": "1.0",
        "out_alpha": "0.40",
        "meas_alpha": "0.70",
        "deriv_alpha": "0.20",
        "integ_deadband": "1.5",
    }
    HYBRID_GUI_PRESET: dict[str, object] = {
        "ff_enable": False,
        "flow_ff_enable": False,
        "dip_ff_enable": False,
        "physics_ff_enable": True,
        "under_p_gain": "1.6",
        "over_p_gain": "1.0",
        "out_alpha": "0.50",
        "meas_alpha": "0.55",
        "deriv_alpha": "0.20",
        "integ_deadband": "1.5",
    }

    def profile_locks_ff() -> bool:
        return cpap_plateau_var.get() or cpap_hybrid_var.get()

    def sync_ff_checkbox_states() -> None:
        physics_on = blower_vars["physics_ff_enable"].get()
        locked = profile_locks_ff()
        legacy_blocks_physics = any(blower_vars[k].get() for k in FF_PHYSICS_DISPLACE)
        for cb in legacy_ff_checkbuttons:
            cb.configure(state="disabled" if (physics_on or locked) else "normal")
        if physics_ff_checkbutton is not None:
            physics_ff_checkbutton.configure(
                state="disabled" if (legacy_blocks_physics or locked) else "normal"
            )
        if dip_ff_checkbutton is not None:
            dip_ff_checkbutton.configure(state="disabled" if locked else "normal")

    def apply_gui_profile_preset(
        enable: bool,
        preset: dict[str, object],
        snap_attr: str,
    ) -> None:
        nonlocal _plateau_gui_snap, _hybrid_gui_snap
        snap = _plateau_gui_snap if snap_attr == "plateau" else _hybrid_gui_snap
        if enable:
            if snap is None:
                snap = {k: blower_vars[k].get() for k in preset}
                if snap_attr == "plateau":
                    _plateau_gui_snap = snap
                else:
                    _hybrid_gui_snap = snap
            for key, val in preset.items():
                blower_vars[key].set(val)  # type: ignore[arg-type]
        elif snap is not None:
            for key, val in snap.items():
                blower_vars[key].set(val)  # type: ignore[arg-type]
            if snap_attr == "plateau":
                _plateau_gui_snap = None
            else:
                _hybrid_gui_snap = None
        sync_ff_checkbox_states()

    def on_cpap_plateau_toggle() -> None:
        nonlocal _profile_gui_sync
        if _profile_gui_sync:
            return
        if cpap_plateau_var.get() and cpap_hybrid_var.get():
            _profile_gui_sync = True
            cpap_hybrid_var.set(False)
            apply_gui_profile_preset(False, HYBRID_GUI_PRESET, "hybrid")
            _profile_gui_sync = False
            send("blower set cpap_hybrid_enable 0")
        apply_gui_profile_preset(cpap_plateau_var.get(), PLATEAU_GUI_PRESET, "plateau")
        send(f"blower set cpap_plateau_enable {1 if cpap_plateau_var.get() else 0}")

    def on_cpap_hybrid_toggle() -> None:
        nonlocal _profile_gui_sync
        if _profile_gui_sync:
            return
        if cpap_hybrid_var.get() and cpap_plateau_var.get():
            _profile_gui_sync = True
            cpap_plateau_var.set(False)
            apply_gui_profile_preset(False, PLATEAU_GUI_PRESET, "plateau")
            _profile_gui_sync = False
            send("blower set cpap_plateau_enable 0")
        apply_gui_profile_preset(cpap_hybrid_var.get(), HYBRID_GUI_PRESET, "hybrid")
        send(f"blower set cpap_hybrid_enable {1 if cpap_hybrid_var.get() else 0}")

    def on_physio_fot_toggle() -> None:
        send(f"physio set fot_enable {1 if physio_fot_var.get() else 0}")

    def on_integ_hold_toggle() -> None:
        send(f"blower set integ_hold {1 if integ_hold_var.get() else 0}")

    def on_physics_ff_toggle(*_args: object) -> None:
        nonlocal _ff_mutex_sync
        if _ff_mutex_sync or profile_locks_ff():
            return
        if blower_vars["physics_ff_enable"].get():
            _ff_mutex_sync = True
            for key in FF_PHYSICS_DISPLACE:
                blower_vars[key].set(False)
            _ff_mutex_sync = False
        sync_ff_checkbox_states()

    def on_legacy_ff_toggle(*_args: object) -> None:
        nonlocal _ff_mutex_sync
        if _ff_mutex_sync or profile_locks_ff():
            return
        if any(blower_vars[k].get() for k in FF_PHYSICS_DISPLACE):
            _ff_mutex_sync = True
            blower_vars["physics_ff_enable"].set(False)
            _ff_mutex_sync = False
        sync_ff_checkbox_states()

    def apply_loop_options(request_status: bool = True) -> None:
        # Outer-loop enable maps to BlowerCtrl_SetEnable(). Enabling it while
        # therapy is IDLE spins the blower to the last setpoint and looks like
        # "therapy start" — only allow enable when RAMP/RUNNING; always allow off.
        want_loop = bool(loop_enable_var.get())
        _, _, st = reader.control_snapshot()
        therapy_state = int(st.get("state", 0)) if st else 0
        therapy_active = therapy_state in (1, 2)  # RAMP, RUNNING
        if want_loop and therapy_active:
            send("blower set loop_enable 1")
        elif not want_loop:
            send("blower set loop_enable 0")
        # else: checkbox on but therapy idle → skip (use Start Therapy instead)

        send(f"blower set pid_enable {1 if pid_enable_var.get() else 0}")
        send(f"blower set rpm_floor_enable {1 if rpm_floor_var.get() else 0}")
        # The other two expiratory floors live in the same panel. Also sent by
        # Apply Blower Tuning (they are in blower_vars); resending here is
        # idempotent and keeps either button doing what the panel implies.
        send(
            "blower set ff_ss_floor_enable "
            f"{1 if blower_vars['ff_ss_floor_enable'].get() else 0}"
        )
        send(
            "blower set exhale_integ_clamp_enable "
            f"{1 if blower_vars['exhale_integ_clamp_enable'].get() else 0}"
        )
        send(
            "blower set vent_floor_enable "
            f"{1 if blower_vars['vent_floor_enable'].get() else 0}"
        )
        send(
            "blower set physics_ff_exp_tail_hold_enable "
            f"{1 if blower_vars['physics_ff_exp_tail_hold_enable'].get() else 0}"
        )
        send(
            "blower set exp_neg_i_clamp_enable "
            f"{1 if blower_vars['exp_neg_i_clamp_enable'].get() else 0}"
        )
        send(
            "blower set rise_ff_enable "
            f"{1 if blower_vars['rise_ff_enable'].get() else 0}"
        )
        send(
            "blower set exp_flow_ff_enable "
            f"{1 if blower_vars['exp_flow_ff_enable'].get() else 0}"
        )
        send(
            "blower set integ_err_lpf_enable "
            f"{1 if blower_vars['integ_err_lpf_enable'].get() else 0}"
        )
        send(f"physio set fot_enable {1 if physio_fot_var.get() else 0}")
        if cpap_hybrid_var.get():
            apply_gui_profile_preset(True, HYBRID_GUI_PRESET, "hybrid")
            send("blower set cpap_plateau_enable 0")
            send("blower set cpap_hybrid_enable 1")
        elif cpap_plateau_var.get():
            apply_gui_profile_preset(True, PLATEAU_GUI_PRESET, "plateau")
            send("blower set cpap_hybrid_enable 0")
            send("blower set cpap_plateau_enable 1")
        else:
            send("blower set cpap_hybrid_enable 0")
            send("blower set cpap_plateau_enable 0")
            physics_on = blower_vars["physics_ff_enable"].get()
            send(f"blower set physics_ff_enable {1 if physics_on else 0}")
            if physics_on:
                send("blower set ff_enable 0")
                send("blower set flow_ff_enable 0")
            else:
                send(f"blower set ff_enable {1 if blower_vars['ff_enable'].get() else 0}")
                send(f"blower set flow_ff_enable {1 if blower_vars['flow_ff_enable'].get() else 0}")
            send(f"blower set dip_ff_enable {1 if blower_vars['dip_ff_enable'].get() else 0}")
        if request_status:
            send("therapy status")
        if want_loop and not therapy_active:
            cmd_status.set(
                "Loop options applied (outer loop not enabled — use Start Therapy)"
            )

    def manual_ramp_ms() -> int:
        try:
            ramp_ms = int(float(manual_ramp_ms_var.get()))
        except ValueError:
            cmd_status.set("Invalid manual ramp duration (ms)")
            return -1
        return max(0, min(65535, ramp_ms))

    def manual_target_rpm() -> int | None:
        try:
            rpm = int(float(manual_rpm_var.get()))
        except ValueError:
            cmd_status.set("Invalid manual target RPM")
            return None
        return max(0, min(45000, rpm))

    def start_manual_blower() -> None:
        rpm = manual_target_rpm()
        if rpm is None:
            return
        ramp_ms = manual_ramp_ms()
        if ramp_ms < 0:
            return
        send("therapy stop")
        send("blower set loop_enable 0")
        send(f"blower manual start {rpm} {ramp_ms}")

    def update_manual_blower_speed() -> None:
        rpm = manual_target_rpm()
        if rpm is None:
            return
        ramp_ms = manual_ramp_ms()
        if ramp_ms < 0:
            return
        send(f"blower manual speed {rpm} {ramp_ms}")

    def stop_manual_blower() -> None:
        send("blower manual stop")

    def apply_therapy_config(request_status: bool = True) -> None:
        """Therapy mode/pressure/ramp/EPR/mask-off + loop enables only."""
        mode = mode_var.get()
        if mode not in therapy_mode_commands:
            cmd_status.set("Invalid therapy mode")
            return
        send(f"therapy set mode {therapy_mode_commands[mode]}")
        try:
            if mode == "CPAP":
                pressure = float(pressure_var.get())
                send(f"therapy set pressure {pressure:.1f}")
            else:
                pmin = float(apap_min_var.get())
                pmax = float(apap_max_var.get())
                if pmin >= pmax:
                    cmd_status.set("Min pressure must be below max")
                    return
                send(f"therapy set pressure_min {pmin:.1f}")
                send(f"therapy set pressure_max {pmax:.1f}")
        except ValueError:
            cmd_status.set("Invalid numeric field")
            return
        send(f"therapy set ramp {ramp_minutes()}")
        if epr_var.get():
            send(f"therapy set epr_type {epr_type_var.get()}")
            send(f"therapy set epr_level {epr_level_var.get()}")
        send(f"therapy set epr {1 if epr_var.get() else 0}")
        send(f"therapy set maskoff {1 if maskoff_var.get() else 0}")
        apply_loop_options(request_status=False)
        if request_status:
            send("therapy status")

    def apply_flow_units(request_status: bool = False) -> None:
        use_btps = flow_units_var.get().strip().upper() == "BTPS"
        send(f"flow set btps_enable {1 if use_btps else 0}")
        ax_f.set_ylabel("Patient flow (L/min BTPS)" if use_btps else "Patient flow (slm SFM)")
        fig.canvas.draw_idle()
        if request_status:
            send("therapy status")

    def apply_leak_options(request_status: bool = True) -> None:
        send(f"leak set vivo_enable {1 if leak_vivo_var.get() else 0}")
        send(f"leak set patient_quiet_lpf_enable {1 if leak_pause_lpf_var.get() else 0}")
        try:
            ratio = int(leak_standard_ratio_var.get())
            patient_lpf = float(leak_patient_lpf_var.get())
            high_thr = float(leak_high_threshold_var.get())
            vent_k = float(leak_simple_vent_k_var.get())
            tau = float(leak_simple_tau_var.get())
        except ValueError:
            cmd_status.set("Invalid leak tuning value")
            return
        send(f"leak set standard_ratio {ratio}")
        send(f"leak set patient_lpf_hz {patient_lpf:.2f}")
        send(f"leak set high_threshold {high_thr:.1f}")
        send(f"leak set vent_k {vent_k:.2f}")
        send(f"leak set tau_seconds {tau:.1f}")
        if request_status:
            send("therapy status")

    def apply_settings() -> None:
        apply_therapy_config(request_status=False)
        apply_blower_tuning(request_status=False)
        apply_foc_tuning(request_status=False)
        apply_flow_units(request_status=False)
        apply_leak_options(request_status=False)
        send("therapy status")

    def apply_foc_tuning(request_status: bool = True) -> None:
        for name, var in foc_vars.items():
            value = var.get().strip()
            if not value:
                cmd_status.set(f"Missing FOC tuning value: {name}")
                return
            try:
                int(value)
            except ValueError:
                cmd_status.set(f"Invalid FOC tuning value: {name}")
                return
            send(f"foc set {name} {value}")
        if request_status:
            send("therapy status")

    def apply_blower_tuning(request_status: bool = True) -> None:
        plateau_on = cpap_plateau_var.get()
        hybrid_on = cpap_hybrid_var.get()
        profile_on = plateau_on or hybrid_on
        loop_bool_keys = {
            "ff_enable",
            "flow_ff_enable",
            "dip_ff_enable",
            "physics_ff_enable",
        }
        plateau_owned_keys = {
            "under_p_gain",
            "over_p_gain",
            "out_alpha",
            "meas_alpha",
            "deriv_alpha",
            "integ_deadband",
            *loop_bool_keys,
        }
        for name, var in blower_vars.items():
            if profile_on and name in plateau_owned_keys:
                continue
            if name in ("tube_R_lam", "tube_R_turb"):
                continue  # tube comp, sent via "tubecomp set" below
            if isinstance(var, tk.BooleanVar):
                if name in loop_bool_keys:
                    continue
                send(f"blower set {name} {1 if var.get() else 0}")
                continue
            value = var.get().strip()
            if not value:
                cmd_status.set(f"Missing blower tuning value: {name}")
                return
            try:
                float(value)
            except ValueError:
                cmd_status.set(f"Invalid blower tuning value: {name}")
                return
            send(f"blower set {name} {value}")
        send(f"tubecomp set enable {1 if tubecomp_enable_var.get() else 0}")
        for tube_name, key in (("R_lam", "tube_R_lam"), ("R_turb", "tube_R_turb")):
            value = blower_vars[key].get().strip()
            if not value:
                cmd_status.set(f"Missing blower tuning value: {key}")
                return
            try:
                float(value)
            except ValueError:
                cmd_status.set(f"Invalid blower tuning value: {key}")
                return
            send(f"tubecomp set {tube_name} {value}")
        if hybrid_on:
            send("blower set cpap_plateau_enable 0")
            send("blower set cpap_hybrid_enable 1")
        elif plateau_on:
            send("blower set cpap_hybrid_enable 0")
            send("blower set cpap_plateau_enable 1")
        if request_status:
            send("therapy status")

    def start_therapy() -> None:
        # Keep the command stream short: a long blower/FOC flood before start
        # can stall the firmware USB task and freeze the LCD.
        send("blower manual stop")
        mode = mode_var.get()
        if mode not in therapy_mode_commands:
            cmd_status.set("Invalid therapy mode")
            return
        send(f"therapy set mode {therapy_mode_commands[mode]}")
        try:
            if mode == "CPAP":
                pressure = float(pressure_var.get())
                send(f"therapy set pressure {pressure:.1f}")
            else:
                pmin = float(apap_min_var.get())
                pmax = float(apap_max_var.get())
                if pmin >= pmax:
                    cmd_status.set("Min pressure must be below max")
                    return
                send(f"therapy set pressure_min {pmin:.1f}")
                send(f"therapy set pressure_max {pmax:.1f}")
        except ValueError:
            cmd_status.set("Invalid numeric field")
            return
        send(f"therapy set ramp {ramp_minutes()}")
        send(f"therapy set epr {1 if epr_var.get() else 0}")
        if epr_var.get():
            level = int(epr_level_var.get())
            send(f"therapy set epr_type {epr_type_var.get()}")
            send(f"therapy set epr_level {level}")
            send(f"therapy set epr_relief {level:.1f}")
        send(f"therapy set maskoff {1 if maskoff_var.get() else 0}")

        def _start_after_config() -> None:
            send("therapy start")
            send("therapy status")

        root.after(600, _start_after_config)

    def stop_therapy() -> None:
        send("therapy stop")
        send("blower manual stop")

    def refresh_status() -> None:
        ack, err, st = reader.control_snapshot()
        if err:
            ack_status.set(err)
        elif ack:
            ack_status.set(ack)
        if st:
            state = state_names.get(int(st.get("state", -1)), str(st.get("state", "?")))
            mode_label = therapy_mode_names.get(int(st.get("mode", 0)), "CPAP")
            range_text = ""
            if int(st.get("mode", 0)) != 0:
                range_text = "  Range={:.1f}-{:.1f}".format(
                    float(st.get("pressure_min", 0.0)),
                    float(st.get("pressure_max", 0.0)),
                )
            epr_text = "On" if int(st.get("epr", 0)) else "Off"
            if int(st.get("epr", 0)):
                phase_names = {0: "---", 1: "Inhale", 2: "Exhale"}
                epr_text = "On L{level} R{relief:.0f} {phase}".format(
                    level=int(st.get("epr_level", 0)),
                    relief=float(st.get("epr_relief", 0.0)),
                    phase=phase_names.get(int(st.get("epr_phase", 0)), "?"),
                )
            live_status.set(
                "Mode={mode}{range_text}  State={state}  Target={target:.1f}  "
                "Setpoint={setpoint:.1f}  Measured={measured:.1f}  Leak={leak:.0f}  "
                "EPR={epr}  MaskOff={maskoff}  Fault={fault}".format(
                    mode=mode_label,
                    range_text=range_text,
                    state=state,
                    target=float(st.get("target", 0.0)),
                    setpoint=float(st.get("setpoint", 0.0)),
                    measured=float(st.get("measured", 0.0)),
                    leak=float(st.get("leak", 0.0)),
                    epr=epr_text,
                    maskoff="Active" if int(st.get("maskoff", 0)) else "No",
                    fault=int(st.get("fault", 0)),
                )
            )
        refresh_metric_panels()
        root.after(500, refresh_status)

    controls = ttk.LabelFrame(control_inner, text="Therapy", padding=10)
    controls.grid(row=0, column=0, sticky="nsew", padx=(0, 12), pady=(0, 12))
    options = ttk.LabelFrame(control_inner, text="Options", padding=10)
    options.grid(row=0, column=1, sticky="nsew", pady=(0, 12))
    loop_frame = ttk.LabelFrame(control_inner, text="Pressure Loop (CM7)", padding=10)
    loop_frame.grid(row=1, column=0, sticky="nsew", padx=(0, 6), pady=(0, 12))
    manual_frame = ttk.LabelFrame(control_inner, text="Manual Blower (open loop)", padding=10)
    manual_frame.grid(row=1, column=1, sticky="nsew", padx=(6, 0), pady=(0, 12))
    leak_frame = ttk.LabelFrame(control_inner, text="Leak Estimation", padding=10)
    leak_frame.grid(row=2, column=0, columnspan=2, sticky="nsew", pady=(0, 12))
    flow_frame = ttk.LabelFrame(control_inner, text="Orifice Flow (Round Hole_2 / SFM3300)", padding=10)
    flow_frame.grid(row=3, column=0, columnspan=2, sticky="nsew", pady=(0, 12))
    tuning = ttk.LabelFrame(control_inner, text="Blower Tuning", padding=10)
    tuning.grid(row=4, column=0, columnspan=2, sticky="nsew", pady=(0, 12))
    more = ttk.LabelFrame(control_inner, text="More / Status", padding=10)
    more.grid(row=5, column=0, columnspan=2, sticky="nsew", pady=(0, 0))
    control_inner.columnconfigure(0, weight=1)
    control_inner.columnconfigure(1, weight=1)
    control_inner.rowconfigure(4, weight=0)
    control_inner.rowconfigure(5, weight=1)

    ttk.Label(controls, text="Mode").grid(row=0, column=0, sticky="w")
    ttk.Combobox(
        controls,
        textvariable=mode_var,
        values=["CPAP", "Autoset", "Autoset for her"],
        width=16,
        state="readonly",
    ).grid(row=0, column=1, sticky="w", padx=8, pady=4)

    cpap_pressure_lbl = ttk.Label(controls, text="CPAP Pressure (cmH2O)")
    cpap_pressure_sb = ttk.Spinbox(
        controls, from_=4.0, to=25.0, increment=0.5, textvariable=pressure_var, width=8
    )
    apap_min_lbl = ttk.Label(controls, text="Min Pressure (cmH2O)")
    apap_min_sb = ttk.Spinbox(
        controls, from_=4.0, to=25.0, increment=0.5, textvariable=apap_min_var, width=8
    )
    apap_max_lbl = ttk.Label(controls, text="Max Pressure (cmH2O)")
    apap_max_sb = ttk.Spinbox(
        controls, from_=4.0, to=25.0, increment=0.5, textvariable=apap_max_var, width=8
    )

    def update_mode_visibility(*_args: object) -> None:
        if mode_var.get() == "CPAP":
            cpap_pressure_lbl.grid(row=1, column=0, sticky="w")
            cpap_pressure_sb.grid(row=1, column=1, sticky="w", padx=8, pady=4)
            apap_min_lbl.grid_remove()
            apap_min_sb.grid_remove()
            apap_max_lbl.grid_remove()
            apap_max_sb.grid_remove()
        else:
            cpap_pressure_lbl.grid_remove()
            cpap_pressure_sb.grid_remove()
            apap_min_lbl.grid(row=1, column=0, sticky="w")
            apap_min_sb.grid(row=1, column=1, sticky="w", padx=8, pady=4)
            apap_max_lbl.grid(row=2, column=0, sticky="w")
            apap_max_sb.grid(row=2, column=1, sticky="w", padx=8, pady=4)

    mode_var.trace_add("write", update_mode_visibility)
    update_mode_visibility()

    ttk.Label(controls, text="Ramp Time (min)").grid(row=3, column=0, sticky="w")
    ttk.Combobox(
        controls,
        textvariable=ramp_var,
        values=["0", "Auto/5", "5", "10", "15", "20", "25", "30", "35", "40", "45"],
        width=8,
        state="readonly",
    ).grid(row=3, column=1, sticky="w", padx=8, pady=4)
    ttk.Checkbutton(controls, text="Pressure Relief (EPR)", variable=epr_var).grid(
        row=4, column=0, sticky="w", pady=4
    )

    epr_options = ttk.LabelFrame(controls, text="EPR Settings", padding=8)
    epr_options.grid(row=5, column=0, columnspan=2, sticky="ew", pady=(0, 8))

    def update_epr_options_visibility(*_args: object) -> None:
        if epr_var.get():
            epr_options.grid()
        else:
            epr_options.grid_remove()

    epr_var.trace_add("write", update_epr_options_visibility)

    ttk.Label(epr_options, text="EPR Type").grid(row=0, column=0, sticky="w", pady=2)
    epr_type_frame = ttk.Frame(epr_options)
    epr_type_frame.grid(row=0, column=1, sticky="w", pady=2)
    ttk.Radiobutton(
        epr_type_frame, text="Full time", variable=epr_type_var, value="fulltime"
    ).pack(side=tk.LEFT, padx=(0, 12))
    ttk.Radiobutton(
        epr_type_frame, text="Ramp only", variable=epr_type_var, value="ramp"
    ).pack(side=tk.LEFT)

    ttk.Label(epr_options, text="EPR Level (cmH2O)").grid(row=1, column=0, sticky="w", pady=2)
    epr_level_frame = ttk.Frame(epr_options)
    epr_level_frame.grid(row=1, column=1, sticky="w", pady=2)
    for level in (1, 2, 3):
        ttk.Radiobutton(
            epr_level_frame, text=str(level), variable=epr_level_var, value=level
        ).pack(side=tk.LEFT, padx=(0, 10))

    update_epr_options_visibility()

    ttk.Checkbutton(controls, text="Mask-off low-speed recovery", variable=maskoff_var).grid(
        row=6, column=0, columnspan=2, sticky="w", pady=4
    )
    ttk.Button(controls, text="Apply Settings", command=apply_settings).grid(row=7, column=0, sticky="ew", pady=8)
    ttk.Button(controls, text="Start Therapy", command=start_therapy).grid(row=7, column=1, sticky="ew", pady=8, padx=8)
    ttk.Button(controls, text="Stop Therapy", command=stop_therapy).grid(row=8, column=0, columnspan=2, sticky="ew")

    ttk.Checkbutton(options, text="SmartStart (UI placeholder)", variable=smart_start_var).grid(
        row=0, column=0, columnspan=2, sticky="w", pady=4
    )
    ttk.Checkbutton(options, text="SmartStop (UI placeholder)", variable=smart_stop_var).grid(
        row=1, column=0, columnspan=2, sticky="w", pady=4
    )
    ttk.Label(options, text="Tube Temperature").grid(row=2, column=0, sticky="w")
    ttk.Combobox(options, textvariable=tube_mode_var, values=["Off", "Auto", "Manual"], width=10, state="readonly").grid(
        row=2, column=1, sticky="w", padx=8, pady=4
    )
    ttk.Label(options, text="Manual Temp (C)").grid(row=3, column=0, sticky="w")
    ttk.Spinbox(options, from_=16, to=30, increment=1, textvariable=tube_temp_var, width=8).grid(
        row=3, column=1, sticky="w", padx=8, pady=4
    )
    ttk.Label(options, text="Climate Control").grid(row=4, column=0, sticky="w")
    ttk.Combobox(options, textvariable=climate_var, values=["Auto", "Manual"], width=10, state="readonly").grid(
        row=4, column=1, sticky="w", padx=8, pady=4
    )
    ttk.Label(options, text="Humidity Level").grid(row=5, column=0, sticky="w")
    ttk.Combobox(options, textvariable=humidity_var, values=["0", "1", "2", "3", "4", "5", "6", "7", "8"], width=8, state="readonly").grid(
        row=5, column=1, sticky="w", padx=8, pady=4
    )
    ttk.Checkbutton(options, text="Airplane Mode (UI placeholder)", variable=airplane_var).grid(
        row=6, column=0, columnspan=2, sticky="w", pady=4
    )

    ttk.Checkbutton(loop_frame, text="Outer loop", variable=loop_enable_var).grid(
        row=0, column=0, sticky="w", padx=(0, 10), pady=2
    )
    ttk.Checkbutton(loop_frame, text="PID trim", variable=pid_enable_var).grid(
        row=0, column=1, sticky="w", padx=(0, 10), pady=2
    )
    legacy_ff_checkbuttons.append(
        ttk.Checkbutton(loop_frame, text="Sqrt FF", variable=blower_vars["ff_enable"],
                        command=on_legacy_ff_toggle)
    )
    legacy_ff_checkbuttons[-1].grid(row=0, column=2, sticky="w", padx=(0, 10), pady=2)
    legacy_ff_checkbuttons.append(
        ttk.Checkbutton(loop_frame, text="Flow FF", variable=blower_vars["flow_ff_enable"],
                        command=on_legacy_ff_toggle)
    )
    legacy_ff_checkbuttons[-1].grid(row=0, column=3, sticky="w", padx=(0, 10), pady=2)
    dip_ff_checkbutton = ttk.Checkbutton(
        loop_frame, text="Dip FF", variable=blower_vars["dip_ff_enable"]
    )
    dip_ff_checkbutton.grid(row=0, column=4, sticky="w", padx=(0, 10), pady=2)
    physics_ff_checkbutton = ttk.Checkbutton(
        loop_frame,
        text="Physics FF",
        variable=blower_vars["physics_ff_enable"],
        command=on_physics_ff_toggle,
    )
    physics_ff_checkbutton.grid(row=0, column=5, sticky="w", pady=2)
    ttk.Checkbutton(
        loop_frame,
        text="RPM floor",
        variable=rpm_floor_var,
    ).grid(row=1, column=0, sticky="w", pady=(6, 2))
    ttk.Checkbutton(
        loop_frame,
        text="CPAP Plateau",
        variable=cpap_plateau_var,
        command=on_cpap_plateau_toggle,
    ).grid(row=1, column=1, sticky="w", pady=(6, 2))
    ttk.Checkbutton(
        loop_frame,
        text="CPAP Hybrid",
        variable=cpap_hybrid_var,
        command=on_cpap_hybrid_toggle,
    ).grid(row=1, column=2, sticky="w", pady=(6, 2))
    ttk.Checkbutton(
        loop_frame,
        text="Physio FOT",
        variable=physio_fot_var,
        command=on_physio_fot_toggle,
    ).grid(row=1, column=3, sticky="w", pady=(6, 2))
    ttk.Button(loop_frame, text="Apply Loop Options", command=apply_loop_options).grid(
        row=1, column=4, sticky="w", pady=(6, 2)
    )
    # The other two expiratory floors. All three must be cleared to let the
    # loop command speeds below the zero-flow steady state during expiration;
    # these two ride on Apply Blower Tuning, "RPM floor" on Apply Loop Options.
    ttk.Checkbutton(
        loop_frame,
        text="FF ss floor",
        variable=blower_vars["ff_ss_floor_enable"],
    ).grid(row=2, column=0, sticky="w", pady=(6, 2))
    ttk.Checkbutton(
        loop_frame,
        text="Exhale I clamp",
        variable=blower_vars["exhale_integ_clamp_enable"],
    ).grid(row=2, column=1, sticky="w", pady=(6, 2))
    ttk.Checkbutton(
        loop_frame,
        text="Vent floor",
        variable=blower_vars["vent_floor_enable"],
    ).grid(row=2, column=2, sticky="w", pady=(6, 2))
    # Not a floor, but the other expiratory FF behaviour: inside the exp-tail
    # band (|Q_patient| < "exp tail") physicsFfStep only ever slews the FF UP,
    # so a FF still descending from inspiration freezes there.
    ttk.Checkbutton(
        loop_frame,
        text="Exp tail hold",
        variable=blower_vars["physics_ff_exp_tail_hold_enable"],
    ).grid(row=2, column=3, sticky="w", pady=(6, 2))
    ttk.Checkbutton(
        loop_frame,
        text="Exp neg-I clamp",
        variable=blower_vars["exp_neg_i_clamp_enable"],
    ).grid(row=2, column=4, sticky="w", pady=(6, 2))
    ttk.Checkbutton(
        loop_frame,
        text="Rise FF",
        variable=blower_vars["rise_ff_enable"],
    ).grid(row=2, column=5, sticky="w", pady=(6, 2))
    ttk.Checkbutton(
        loop_frame,
        text="Exp flow FF",
        variable=blower_vars["exp_flow_ff_enable"],
    ).grid(row=3, column=0, sticky="w", pady=(6, 2))
    ttk.Checkbutton(
        loop_frame,
        text="I err LPF",
        variable=blower_vars["integ_err_lpf_enable"],
    ).grid(row=3, column=1, sticky="w", pady=(6, 2))
    ttk.Checkbutton(
        loop_frame,
        text="Hold I trim",
        variable=integ_hold_var,
        command=on_integ_hold_toggle,
    ).grid(row=3, column=2, sticky="w", pady=(6, 2))
    ttk.Label(
        loop_frame,
        text="Hybrid = Physics FF on insp; exp freezes RPM (boost if P dips). Mutex with Plateau. "
        "Uncheck Hybrid/Plateau to A/B raw Physics FF. Physio FOT = periodic 4 Hz probe.\n"
        "Expiratory floors: RPM floor + FF ss floor + Exhale I clamp. While any is set the loop "
        "cannot slow below the zero-flow speed, so an expiratory bump can only bleed off passively.\n"
        "Hold I trim = freeze the integrator where it is (applies immediately). The integrator is the "
        "only term that can move the BASELINE, so holding it stops the between-breath level drifting "
        "and shows the raw P + FF waveform. Any genuine steady-state error stays uncorrected while held.",
        wraplength=560,
    ).grid(row=4, column=0, columnspan=6, sticky="w", padx=(0, 0), pady=(4, 2))
    sync_ff_checkbox_states()

    ttk.Checkbutton(
        leak_frame,
        text="Vivo-style leak (table × MeanRatio)",
        variable=leak_vivo_var,
    ).grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 6))
    ttk.Checkbutton(
        leak_frame,
        text="Pause 0.35 Hz LPF",
        variable=leak_pause_lpf_var,
        command=lambda: send(
            f"leak set patient_quiet_lpf_enable {1 if leak_pause_lpf_var.get() else 0}"
        ),
    ).grid(row=0, column=2, columnspan=2, sticky="w", pady=(0, 6))

    ttk.Label(leak_frame, text="MeanRatio start").grid(row=1, column=0, sticky="w", pady=2)
    ttk.Entry(leak_frame, textvariable=leak_standard_ratio_var, width=8).grid(
        row=1, column=1, sticky="w", padx=(8, 16), pady=2
    )
    ttk.Label(leak_frame, text="Patient LPF (Hz)").grid(row=1, column=2, sticky="w", pady=2)
    ttk.Entry(leak_frame, textvariable=leak_patient_lpf_var, width=8).grid(
        row=1, column=3, sticky="w", padx=(8, 16), pady=2
    )
    ttk.Label(leak_frame, text="High leak (slm)").grid(row=1, column=4, sticky="w", pady=2)
    ttk.Entry(leak_frame, textvariable=leak_high_threshold_var, width=8).grid(
        row=1, column=5, sticky="w", padx=(8, 0), pady=2
    )

    ttk.Label(leak_frame, text="Simple vent K").grid(row=2, column=0, sticky="w", pady=2)
    ttk.Entry(leak_frame, textvariable=leak_simple_vent_k_var, width=8).grid(
        row=2, column=1, sticky="w", padx=(8, 16), pady=2
    )
    ttk.Label(leak_frame, text="Simple tau (s)").grid(row=2, column=2, sticky="w", pady=2)
    ttk.Entry(leak_frame, textvariable=leak_simple_tau_var, width=8).grid(
        row=2, column=3, sticky="w", padx=(8, 16), pady=2
    )
    ttk.Button(leak_frame, text="Apply Leak Settings", command=apply_leak_options).grid(
        row=2, column=4, columnspan=2, sticky="e", padx=(8, 0), pady=2
    )

    ttk.Label(
        leak_frame,
        textvariable=leak_live_var,
        wraplength=900,
        justify=tk.LEFT,
    ).grid(row=3, column=0, columnspan=6, sticky="w", pady=(8, 0))
    for col in range(6):
        leak_frame.columnconfigure(col, weight=1 if col % 2 else 0)

    ttk.Label(flow_frame, text="Flow units").grid(row=0, column=0, sticky="w", pady=2)
    flow_units_row = ttk.Frame(flow_frame)
    flow_units_row.grid(row=0, column=1, columnspan=3, sticky="w", pady=2)
    ttk.Radiobutton(
        flow_units_row,
        text="SLM (SFM3300 as-calibrated)",
        variable=flow_units_var,
        value="SLM",
        command=lambda: apply_flow_units(False),
    ).pack(side=tk.LEFT, padx=(0, 16))
    ttk.Radiobutton(
        flow_units_row,
        text="BTPS (37 °C sat)",
        variable=flow_units_var,
        value="BTPS",
        command=lambda: apply_flow_units(False),
    ).pack(side=tk.LEFT)
    ttk.Button(flow_frame, text="Apply Flow Units", command=lambda: apply_flow_units(True)).grid(
        row=0, column=4, sticky="w", padx=(16, 0), pady=2
    )
    ttk.Label(
        flow_frame,
        text="LUT = Round Hole_2 ΔP→Q vs SFM3300-250-D SLM. BTPS scales firmware flow "
        "(therapy/leak/telemetry) by (310.15/293.15)×(P/(P−62.67)) using ambient P from "
        "measAS.pressure (mbar, ~10 Hz). Default SLM.",
        wraplength=900,
        justify=tk.LEFT,
    ).grid(row=1, column=0, columnspan=5, sticky="w", pady=(6, 0))
    for col in range(5):
        flow_frame.columnconfigure(col, weight=1 if col % 2 else 0)

    ttk.Label(manual_frame, text="RPM").grid(row=0, column=0, sticky="w", pady=2)
    ttk.Spinbox(manual_frame, from_=0, to=45000, increment=500, textvariable=manual_rpm_var, width=8).grid(
        row=0, column=1, sticky="w", padx=(4, 10), pady=2
    )
    ttk.Label(manual_frame, text="Ramp ms").grid(row=0, column=2, sticky="w", pady=2)
    ttk.Spinbox(manual_frame, from_=0, to=65535, increment=500, textvariable=manual_ramp_ms_var, width=7).grid(
        row=0, column=3, sticky="w", padx=(4, 10), pady=2
    )
    ttk.Button(manual_frame, text="Start", command=start_manual_blower).grid(
        row=0, column=4, sticky="ew", padx=(0, 4), pady=2
    )
    ttk.Button(manual_frame, text="Update", command=update_manual_blower_speed).grid(
        row=0, column=5, sticky="ew", padx=(0, 4), pady=2
    )
    ttk.Button(manual_frame, text="Stop", command=stop_manual_blower).grid(
        row=0, column=6, sticky="ew", pady=2
    )
    for col in range(4, 7):
        manual_frame.columnconfigure(col, weight=1)

    def add_entry(parent, row: int, pair: int, label: str, key: str, width: int = 7) -> None:
        """Place label+entry pair; pair 0..2 maps to three columns across the row."""
        col = pair * 2
        ttk.Label(parent, text=label).grid(row=row, column=col, sticky="w", padx=(0, 4), pady=2)
        ttk.Entry(parent, textvariable=blower_vars[key], width=width).grid(
            row=row, column=col + 1, sticky="w", padx=(0, 16), pady=2
        )

    for pair in range(3):
        tuning.columnconfigure(pair * 2, weight=0)
        tuning.columnconfigure(pair * 2 + 1, weight=0)

    ttk.Label(tuning, text="PID gains").grid(row=0, column=0, columnspan=6, sticky="w", pady=(0, 4))
    add_entry(tuning, 1, 0, "LOW Kp", "kp_low", 7)
    add_entry(tuning, 1, 1, "LOW Ki", "ki_low", 7)
    add_entry(tuning, 1, 2, "LOW Kd", "kd_low", 6)
    add_entry(tuning, 2, 0, "MID Kp", "kp_mid", 7)
    add_entry(tuning, 2, 1, "MID Ki", "ki_mid", 7)
    add_entry(tuning, 2, 2, "MID Kd", "kd_mid", 6)
    add_entry(tuning, 3, 0, "HIGH Kp", "kp_high", 7)
    add_entry(tuning, 3, 1, "HIGH Ki", "ki_high", 7)
    add_entry(tuning, 3, 2, "HIGH Kd", "kd_high", 6)

    ttk.Separator(tuning, orient=tk.HORIZONTAL).grid(row=4, column=0, columnspan=6, sticky="ew", pady=6)
    ttk.Label(tuning, text="Physics FF").grid(row=5, column=0, columnspan=6, sticky="w", pady=(0, 2))
    add_entry(tuning, 6, 0, "sqrt K", "ff_k", 7)
    add_entry(tuning, 6, 1, "c_fan", "physics_ff_c_fan", 8)
    add_entry(tuning, 6, 2, "Q idle Hz", "physics_ff_flow_lpf_hz", 6)
    add_entry(tuning, 7, 0, "Q insp Hz", "physics_ff_flow_lpf_inhale_hz", 6)
    add_entry(tuning, 7, 1, "rise", "physics_ff_rise", 7)
    add_entry(tuning, 7, 2, "fall", "physics_ff_fall", 7)
    add_entry(tuning, 8, 0, "fall exh", "physics_ff_fall_exhale", 7)
    add_entry(tuning, 8, 1, "Q exp Hz", "physics_ff_flow_lpf_exhale_hz", 6)
    add_entry(tuning, 8, 2, "exp tail", "physics_ff_exp_tail_flow_slm", 6)
    add_entry(tuning, 9, 0, "insp rise x", "physics_ff_inhale_rise_boost", 6)
    add_entry(tuning, 9, 1, "under P", "under_p_gain", 6)
    add_entry(tuning, 9, 2, "ff offset", "ff_offset", 7)

    # --- Anticipatory (lead) terms ---------------------------------------
    # These two are the only blocks that act BEFORE pressure has fallen.
    # Both are threshold-gated, and both stock thresholds sit above what a
    # 25 slm bench breath actually produces, so they contribute nothing
    # until the thresholds are lowered to match the measured signal.
    ttk.Label(tuning, text="Inspiratory lead (fires before the dip)").grid(
        row=10, column=0, columnspan=6, sticky="w", pady=(6, 2)
    )
    add_entry(tuning, 11, 0, "onset thr", "physics_ff_onset_thresh", 6)
    add_entry(tuning, 11, 1, "onset k", "physics_ff_onset_k", 6)
    add_entry(tuning, 11, 2, "onset max", "physics_ff_onset_max", 7)
    add_entry(tuning, 16, 1, "onset dQτ", "physics_ff_onset_dqdt_tau", 6)
    ttk.Checkbutton(
        tuning,
        text="Onset lead past slew",
        variable=blower_vars["physics_ff_onset_bypass_slew"],
    ).grid(row=16, column=4, columnspan=2, sticky="w", padx=(8, 0))
    add_entry(tuning, 12, 0, "dip thr", "dip_ff_thresh", 6)
    add_entry(tuning, 12, 1, "dip k", "dip_ff_k", 6)
    add_entry(tuning, 12, 2, "dip max", "dip_ff_max", 7)
    add_entry(tuning, 13, 0, "dip err min", "dip_ff_err_min", 6)
    add_entry(tuning, 13, 1, "rise thr", "rise_ff_thresh", 6)
    add_entry(tuning, 13, 2, "rise k", "rise_ff_k", 6)
    add_entry(tuning, 14, 0, "expQ k", "exp_flow_ff_k", 6)
    add_entry(tuning, 14, 1, "expQ lead", "exp_flow_ff_lead", 6)
    add_entry(tuning, 14, 2, "expQ max", "exp_flow_ff_max", 6)
    add_entry(tuning, 15, 0, "I err tau", "integ_err_lpf_tau", 6)
    add_entry(tuning, 15, 1, "I err τ0", "integ_err_lpf_tau_start", 6)
    add_entry(tuning, 15, 2, "I err warm", "integ_err_lpf_warmup", 6)
    add_entry(tuning, 16, 0, "I err band", "integ_err_lpf_band", 6)
    ttk.Label(
        tuning,
        text="onset thr = slm/s on dQ/dt (min 10)   dip thr = cmH2O/s on dP/dt (min 1).\n"
        "Set each just above the measured noise floor, not above the breath itself.",
        foreground="#555555",
        justify=tk.LEFT,
    ).grid(row=17, column=0, columnspan=6, sticky="w", pady=(2, 0))

    ttk.Separator(tuning, orient=tk.HORIZONTAL).grid(row=18, column=0, columnspan=6, sticky="ew", pady=6)
    ttk.Label(tuning, text="Tube drop (sensor → mask)").grid(row=19, column=0, columnspan=6, sticky="w", pady=(0, 2))
    add_entry(tuning, 20, 0, "R_lam", "tube_R_lam", 8)
    add_entry(tuning, 20, 1, "R_turb", "tube_R_turb", 8)
    # Column 4 is the third label/entry pair; column 2 belongs to the R_turb
    # label, which this checkbox was overlapping (it rendered as a bare "R").
    ttk.Checkbutton(tuning, text="Tube comp on", variable=tubecomp_enable_var).grid(
        row=20, column=4, columnspan=2, sticky="w", padx=(8, 0)
    )

    ttk.Label(tuning, text="Legacy flow FF").grid(row=21, column=0, columnspan=6, sticky="w", pady=(6, 2))
    add_entry(tuning, 22, 0, "K", "flow_ff_k", 6)
    add_entry(tuning, 22, 1, "max", "flow_ff_max", 6)
    add_entry(tuning, 22, 2, "deadband", "flow_ff_deadband", 6)
    add_entry(tuning, 23, 0, "rise", "flow_ff_rise", 7)
    add_entry(tuning, 23, 1, "fall", "flow_ff_fall", 7)

    ttk.Separator(tuning, orient=tk.HORIZONTAL).grid(row=24, column=0, columnspan=6, sticky="ew", pady=6)
    ttk.Label(tuning, text="Loop / filter").grid(row=25, column=0, columnspan=6, sticky="w", pady=(0, 2))
    add_entry(tuning, 26, 0, "over P", "over_p_gain", 6)
    add_entry(tuning, 26, 1, "brake", "over_brake", 7)
    add_entry(tuning, 26, 2, "RPM max", "rpm_max", 7)
    add_entry(tuning, 27, 0, "slew down", "slew_down", 6)
    add_entry(tuning, 27, 1, "slew up", "slew_up", 6)
    add_entry(tuning, 27, 2, "out alpha", "out_alpha", 6)
    add_entry(tuning, 28, 0, "I deadband", "integ_deadband", 6)
    add_entry(tuning, 28, 1, "I max", "integ_max", 7)
    # These two alphas weight OPPOSITE terms in firmware, so "raise both to
    # smooth things" makes the loop laggier and the D term noisier at once:
    #   meas:  filt = a*filt_prev + (1-a)*new   -> a up = heavier = more lag
    #   deriv: filt = a*raw       + (1-a)*filt  -> a up = lighter = more noise
    # Labels carry the direction; the firmware param names are unchanged.
    add_entry(tuning, 28, 2, "meas α  ↑=lag", "meas_alpha", 6)
    add_entry(tuning, 29, 0, "D α  ↑=noise", "deriv_alpha", 6)
    ttk.Checkbutton(
        tuning,
        text="I deadband active",
        variable=blower_vars["integ_deadband_enable"],
    ).grid(row=29, column=2, columnspan=2, sticky="w", padx=(8, 0))
    add_entry(tuning, 30, 0, "overP thr rest", "over_p_thresh", 6)
    add_entry(tuning, 30, 1, "overP thr breath", "over_p_thresh_exp", 6)
    add_entry(tuning, 30, 2, "breath win s", "over_p_exp_window", 6)
    add_entry(tuning, 31, 0, "underP thr", "under_p_thresh", 6)
    add_entry(tuning, 31, 1, "start ramp", "start_ramp", 7)
    add_entry(tuning, 31, 2, "c_fan exp", "physics_ff_c_fan_exp", 8)
    ttk.Button(tuning, text="Apply Blower Tuning", command=apply_blower_tuning).grid(
        row=32, column=2, columnspan=4, sticky="e", padx=(8, 0), pady=2
    )

    ttk.Button(more, text="Request Status", command=lambda: send("therapy status")).grid(
        row=0, column=0, sticky="w"
    )
    ttk.Button(more, text="Mask Fit (LCD placeholder)", state=tk.DISABLED).grid(
        row=0, column=1, sticky="w", padx=(12, 0)
    )
    ttk.Button(more, text="Diagnostics (LCD placeholder)", state=tk.DISABLED).grid(
        row=0, column=2, sticky="w", padx=(12, 0)
    )

    # --- Raw capture -------------------------------------------------------
    # Same content as --record, but toggleable mid-session so a capture can be
    # bracketed around one protocol run without restarting the app.
    record_name_var = tk.StringVar(
        value=f"capture_{datetime.now():%Y%m%d_%H%M%S}.log"
    )
    record_btn_var = tk.StringVar(value="Start Recording")

    def toggle_record() -> None:
        if reader.is_recording():
            cmd_status.set(reader.record_stop())
            record_btn_var.set("Start Recording")
            # Fresh default name so the next capture does not append to this one.
            record_name_var.set(f"capture_{datetime.now():%Y%m%d_%H%M%S}.log")
            return
        path = record_name_var.get().strip()
        if not path:
            cmd_status.set("Enter a capture filename first")
            return
        try:
            cmd_status.set(reader.record_start(path))
        except OSError as exc:
            cmd_status.set(f"Could not open {path}: {exc}")
            return
        record_btn_var.set("Stop Recording")

    ttk.Button(more, textvariable=record_btn_var, command=toggle_record).grid(
        row=1, column=0, sticky="w", pady=(10, 0)
    )
    ttk.Entry(more, textvariable=record_name_var, width=40).grid(
        row=1, column=1, sticky="w", padx=(12, 0), pady=(10, 0)
    )

    # Controller term decomposition on a separate "D," line. Roughly doubles
    # the stream, so it is a capture-time option rather than always-on; the
    # plots ignore "D," lines, but the recorder writes them.
    telem_terms_var = tk.BooleanVar(value=False)

    def toggle_terms() -> None:
        send(f"blower set telem_terms {1 if telem_terms_var.get() else 0}")

    ttk.Checkbutton(
        more,
        text="Log PID terms (D lines)",
        variable=telem_terms_var,
        command=toggle_terms,
    ).grid(row=1, column=2, sticky="w", padx=(12, 0), pady=(10, 0))

    ttk.Label(more, textvariable=cmd_status, wraplength=900).grid(
        row=2, column=0, columnspan=3, sticky="w", pady=(10, 0)
    )
    ttk.Label(more, textvariable=ack_status, wraplength=900).grid(
        row=3, column=0, columnspan=3, sticky="w"
    )
    ttk.Label(more, textvariable=live_status, wraplength=900).grid(
        row=4, column=0, columnspan=3, sticky="w"
    )

    foc_frame = ttk.LabelFrame(foc_tab, text="CM4 FOC Inner Loop (Motor Control SDK)", padding=10)
    foc_frame.pack(fill=tk.BOTH, expand=True)
    ttk.Label(
        foc_frame,
        text=(
            "CM4 speed/torque/flux PI numerators from drive_parameters.h. "
            "Macros set compile-time defaults; USB commands apply them at runtime via IPC."
        ),
        wraplength=900,
    ).grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 8))

    def add_foc_entry(row: int, label: str, key: str) -> None:
        ttk.Label(foc_frame, text=label).grid(row=row, column=0, sticky="w", padx=(0, 8), pady=4)
        ttk.Entry(foc_frame, textvariable=foc_vars[key], width=10).grid(row=row, column=1, sticky="w", pady=4)

    ttk.Label(foc_frame, text="Speed loop (RPM -> Iq)").grid(row=1, column=0, columnspan=2, sticky="w", pady=(4, 2))
    add_foc_entry(2, "Speed Kp", "speed_kp")
    add_foc_entry(3, "Speed Ki", "speed_ki")
    add_foc_entry(4, "Iq max (mA)", "iq_max_ma")
    add_foc_entry(5, "Iq boost (mA, 0=off)", "iq_boost_ma")
    add_foc_entry(6, "Boost full to (rpm)", "iq_boost_lo_rpm")
    add_foc_entry(7, "Boost off from (rpm)", "iq_boost_hi_rpm")
    ttk.Label(foc_frame, text="Torque loop (Iq)").grid(row=40, column=0, columnspan=2, sticky="w", pady=(12, 2))
    add_foc_entry(41, "Torque Kp", "torque_kp")
    add_foc_entry(42, "Torque Ki", "torque_ki")
    ttk.Label(foc_frame, text="Flux loop (Id)").grid(row=43, column=0, columnspan=2, sticky="w", pady=(12, 2))
    add_foc_entry(44, "Flux Kp", "flux_kp")
    add_foc_entry(45, "Flux Ki", "flux_ki")
    ttk.Button(foc_frame, text="Apply FOC Tuning", command=apply_foc_tuning).grid(
        row=46, column=0, columnspan=2, sticky="ew", pady=(16, 4)
    )

    # ---- APAP / EDF tab (scrollable) ----
    apap_canvas = tk.Canvas(apap_edf_tab, highlightthickness=0, borderwidth=0)
    apap_scrollbar = ttk.Scrollbar(apap_edf_tab, orient=tk.VERTICAL, command=apap_canvas.yview)
    apap_inner = ttk.Frame(apap_canvas, padding=12)
    apap_inner_id = apap_canvas.create_window((0, 0), window=apap_inner, anchor="nw")

    def _on_apap_inner_configure(_event: object = None) -> None:
        apap_canvas.configure(scrollregion=apap_canvas.bbox("all"))

    def _on_apap_canvas_configure(event: tk.Event) -> None:
        apap_canvas.itemconfigure(apap_inner_id, width=event.width)

    def _on_apap_mousewheel(event: tk.Event) -> None:
        apap_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

    apap_inner.bind("<Configure>", _on_apap_inner_configure)
    apap_canvas.bind("<Configure>", _on_apap_canvas_configure)
    apap_canvas.configure(yscrollcommand=apap_scrollbar.set)
    apap_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    apap_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
    apap_canvas.bind("<Enter>", lambda _e: apap_canvas.bind_all("<MouseWheel>", _on_apap_mousewheel))
    apap_canvas.bind("<Leave>", lambda _e: apap_canvas.unbind_all("<MouseWheel>"))

    edf_enable_var = tk.BooleanVar(value=False)
    now = datetime.now()
    edf_yy_var = tk.StringVar(value=f"{now.year % 100:02d}")
    edf_mo_var = tk.StringVar(value=f"{now.month:02d}")
    edf_dd_var = tk.StringVar(value=f"{now.day:02d}")
    edf_hh_var = tk.StringVar(value=f"{now.hour:02d}")
    edf_mi_var = tk.StringVar(value=f"{now.minute:02d}")
    edf_ss_var = tk.StringVar(value=f"{now.second:02d}")
    edf_status_var = tk.StringVar(value="EDF: (query firmware)")
    apap_events_var = tk.StringVar(value="APAP events: (query firmware)")

    # Defaults match apap_ctrl.c Apap_Init / file-scope initializers.
    apap_bool_vars = {
        "response_soft": tk.BooleanVar(value=False),
        "fl_enable": tk.BooleanVar(value=True),
        "hyp_enable": tk.BooleanVar(value=True),
        "snore_enable": tk.BooleanVar(value=True),
        "lower_enable": tk.BooleanVar(value=True),
        "leak_gate_enable": tk.BooleanVar(value=True),
    }
    apap_float_defaults = {
        "no_breath_timeout_s": "2.0",   # 2026-10-12 firmware defaults (APAP findings 1/3)
        "breath_window_s": "8.0",
        "resume_min_peak": "10.0",
        "resume_peak_ratio": "0.30",
        "resume_min_floor": "4.0",
        "startup_grace_s": "15.0",
        "osc_amplitude": "1.0",
        "osc_amplitude_max": "1.75",
        "osc_freq_hz": "4.0",
        "fot_eval_period_s": "2.0",
        "osc_min_p_pp": "0.15",
        "osa_flow_pp_thresh": "2.0",
        "apnea_min_s": "10.0",
        "oa_raise_per_s": "0.08",
        "oa_raise_max": "2.0",
        "her_oa_above_cmh2o": "12.0",
        "her_oa_above_scale": "0.50",
        "raise_rate_standard": "0.050",
        "raise_rate_soft": "0.020",
        "raise_rate_her_scale": "0.75",
        "fl_ratio": "0.70",
        "fl_flat_ratio": "0.72",      # R13: whole-inspiration flatness
        "fl_dip_ratio": "0.93",       # R13: M-shape notch; 0 = off
        "fl_seg_floor": "0.0",
        "fl_baseline_reject": "1.40",
        "baseline_restore_max_s": "120",
        "ca_reclass_window_s": "30",
        "breath_min_insp_ms": "400",
        "breath_min_insp_ml": "30",   # R8a: floor; relative limit below
        "breath_min_vol_ratio": "0.15",
        "hyp_ca_guard_s": "30",
        "hyp_rearm_ratio": "0.70",
        "hyp_rearm_breaths": "2",
        "epr_rel_baseline": "0",
        "rel_baseline": "0",
        "epr_baseline_slow_tau_s": "30",
        "leak_freeze_min_breaths": "5",
        "leak_freeze_max_s": "60",
        "breath_notch": "1",
        "fl_raise_scale": "1.00",
        "fl_min_peak": "4.0",
        "fl_baseline_alpha": "0.25",
        "hyp_ratio": "0.50",
        "hyp_min_duration_s": "10.0",
        "hyp_raise_scale": "0.70",
        "hyp_hold_s": "20.0",
        "hyp_raise_shape_frac": "0.50",   # raise only on shape-limited (obstructive) Hyp
        "snore_rms_thresh": "0.35",
        "snore_on_s": "1.5",
        "snore_off_s": "2.0",
        "snore_raise_scale": "0.35",
        "snore_hp_hz": "20.0",
        "lower_rate": "0.005",
        "stable_before_lower_s": "20.0",
    }
    apap_int_defaults = {
        "fl_breaths_needed": "3",
        "fl_clear_breaths": "2",
        "hyp_breaths_needed": "2",
        "fl_amp_arm": "0",            # 0 = FL on shape only (central hypopneas)
    }
    apap_float_vars = {k: tk.StringVar(value=v) for k, v in apap_float_defaults.items()}
    apap_int_vars = {k: tk.StringVar(value=v) for k, v in apap_int_defaults.items()}

    def fill_edf_datetime_now() -> None:
        n = datetime.now()
        edf_yy_var.set(f"{n.year % 100:02d}")
        edf_mo_var.set(f"{n.month:02d}")
        edf_dd_var.set(f"{n.day:02d}")
        edf_hh_var.set(f"{n.hour:02d}")
        edf_mi_var.set(f"{n.minute:02d}")
        edf_ss_var.set(f"{n.second:02d}")

    def apply_edf_settings() -> None:
        send(f"therapy edf enable {1 if edf_enable_var.get() else 0}")
        try:
            yy = int(edf_yy_var.get())
            mo = int(edf_mo_var.get())
            dd = int(edf_dd_var.get())
            hh = int(edf_hh_var.get())
            mi = int(edf_mi_var.get())
            ss = int(edf_ss_var.get())
        except ValueError:
            cmd_status.set("Invalid EDF datetime field")
            return
        send(f"therapy edf datetime {yy} {mo} {dd} {hh} {mi} {ss}")
        send("therapy edf status")

    def query_edf_status() -> None:
        send("therapy edf status")

    def apply_apap_tuning() -> None:
        for name, var in apap_bool_vars.items():
            send(f"apap set {name} {1 if var.get() else 0}")
        for name, var in apap_int_vars.items():
            value = var.get().strip()
            if not value:
                cmd_status.set(f"Missing APAP int: {name}")
                return
            try:
                int(value)
            except ValueError:
                cmd_status.set(f"Invalid APAP int: {name}")
                return
            send(f"apap set {name} {value}")
        for name, var in apap_float_vars.items():
            value = var.get().strip()
            if not value:
                cmd_status.set(f"Missing APAP float: {name}")
                return
            try:
                float(value)
            except ValueError:
                cmd_status.set(f"Invalid APAP float: {name}")
                return
            send(f"apap set {name} {value}")
        send("therapy apap events")
        cmd_status.set("APAP tuning applied")

    def query_apap_events() -> None:
        send("therapy apap events")

    def reset_apap_defaults() -> None:
        for k, v in apap_bool_vars.items():
            v.set(k != "response_soft" and k in (
                "fl_enable", "hyp_enable", "snore_enable", "lower_enable", "leak_gate_enable"
            ))
        apap_bool_vars["response_soft"].set(False)
        for k, v in apap_float_defaults.items():
            apap_float_vars[k].set(v)
        for k, v in apap_int_defaults.items():
            apap_int_vars[k].set(v)

    edf_frame = ttk.LabelFrame(apap_inner, text="Session EDF (SD card)", padding=10)
    edf_frame.pack(fill=tk.X, pady=(0, 10))
    ttk.Label(
        edf_frame,
        text=(
            "Writes 0:/YYmmddNN.EDF on card root @ 25 Hz (Flow, Pressure, Leak, TargetP); "
            "NN increments so every therapy start makes a NEW file. FAT16/FAT32 only. "
            "Watch fsize: it must be >=1280 right after start, then grow. "
            "test: 0=OK 1=ERROR 2=BUSY 3=TIMEOUT. "
            "err 0x2Sxx = open failed at stage S (1=create 2=hdr write 3=hdr sync 4=size verify), "
            "xx=FatFs code (1=DISK_ERR). err 0x3000 = data write failed, 0x31xx = close failed. "
            "dio_err 0x4 = SD CMD timeout, 0x80000000 = SD timeout (sticky until next good write). "
            "test=1 with sd_hal=0x10 = SDMMC TX FIFO underrun (needs hardware flow control); "
            "fail_step shows which self-test stage broke (5 = write-block). "
            "Always Stop Therapy to finalize the header."
        ),
        wraplength=920,
    ).grid(row=0, column=0, columnspan=8, sticky="w", pady=(0, 8))
    ttk.Checkbutton(edf_frame, text="Enable EDF logging on therapy start", variable=edf_enable_var).grid(
        row=1, column=0, columnspan=4, sticky="w", pady=2
    )
    ttk.Label(edf_frame, text="Date/time").grid(row=2, column=0, sticky="w", pady=4)
    dt_row = ttk.Frame(edf_frame)
    dt_row.grid(row=2, column=1, columnspan=7, sticky="w", pady=4)
    for lbl, var in (
        ("YY", edf_yy_var),
        ("MM", edf_mo_var),
        ("DD", edf_dd_var),
        ("HH", edf_hh_var),
        ("MI", edf_mi_var),
        ("SS", edf_ss_var),
    ):
        ttk.Label(dt_row, text=lbl).pack(side=tk.LEFT, padx=(0, 2))
        ttk.Entry(dt_row, textvariable=var, width=4).pack(side=tk.LEFT, padx=(0, 8))
    ttk.Button(edf_frame, text="Use PC clock", command=fill_edf_datetime_now).grid(
        row=3, column=0, sticky="w", pady=6
    )
    ttk.Button(edf_frame, text="Apply EDF Settings", command=apply_edf_settings).grid(
        row=3, column=1, columnspan=2, sticky="w", padx=8, pady=6
    )
    ttk.Button(edf_frame, text="Query EDF Status", command=query_edf_status).grid(
        row=3, column=3, columnspan=2, sticky="w", pady=6
    )
    ttk.Label(edf_frame, textvariable=edf_status_var, wraplength=900).grid(
        row=4, column=0, columnspan=8, sticky="w", pady=(4, 0)
    )

    apap_frame = ttk.LabelFrame(apap_inner, text="APAP algorithm tunables", padding=10)
    apap_frame.pack(fill=tk.BOTH, expand=True)
    ttk.Label(
        apap_frame,
        text=(
            "Debugger/clinical heuristics (not ResMed constants). FOT starts after "
            "FOT start (s); OA/CA score and OA step-raise only after Apnea min (s). "
            "FL uses peak drop and/or inspiratory flattening (mean/peak). "
            "Apply while idle or running. Use AutoSet / AutoSet for Her mode."
        ),
        wraplength=920,
    ).grid(row=0, column=0, columnspan=6, sticky="w", pady=(0, 8))

    def add_apap_float(parent, row: int, col: int, label: str, key: str, width: int = 8) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=col * 2, sticky="w", padx=(0, 4), pady=2)
        ttk.Entry(parent, textvariable=apap_float_vars[key], width=width).grid(
            row=row, column=col * 2 + 1, sticky="w", padx=(0, 12), pady=2
        )

    def add_apap_int(parent, row: int, col: int, label: str, key: str, width: int = 6) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=col * 2, sticky="w", padx=(0, 4), pady=2)
        ttk.Entry(parent, textvariable=apap_int_vars[key], width=width).grid(
            row=row, column=col * 2 + 1, sticky="w", padx=(0, 12), pady=2
        )

    row = 1
    ttk.Label(apap_frame, text="Response / timing").grid(row=row, column=0, columnspan=6, sticky="w", pady=(4, 2))
    row += 1
    ttk.Checkbutton(apap_frame, text="Soft response", variable=apap_bool_vars["response_soft"]).grid(
        row=row, column=0, columnspan=2, sticky="w"
    )
    ttk.Checkbutton(apap_frame, text="Leak gate raises", variable=apap_bool_vars["leak_gate_enable"]).grid(
        row=row, column=2, columnspan=2, sticky="w"
    )
    ttk.Checkbutton(apap_frame, text="Down-titration", variable=apap_bool_vars["lower_enable"]).grid(
        row=row, column=4, columnspan=2, sticky="w"
    )
    row += 1
    add_apap_float(apap_frame, row, 0, "FOT start (s)", "no_breath_timeout_s")
    add_apap_float(apap_frame, row, 1, "Breath window (s)", "breath_window_s")
    add_apap_float(apap_frame, row, 2, "Startup grace (s)", "startup_grace_s")
    row += 1
    add_apap_float(apap_frame, row, 0, "FOT resume peak (slm)", "resume_min_peak")
    add_apap_float(apap_frame, row, 1, "Resume x baseline", "resume_peak_ratio")
    add_apap_float(apap_frame, row, 2, "Resume floor (slm)", "resume_min_floor")
    row += 1
    add_apap_float(apap_frame, row, 0, "Breath min Ti (ms)", "breath_min_insp_ms")
    add_apap_float(apap_frame, row, 1, "Breath min Vi floor (mL)", "breath_min_insp_ml")
    add_apap_float(apap_frame, row, 2, "CA->Hyp window (s)", "ca_reclass_window_s")
    row += 1
    add_apap_float(apap_frame, row, 0, "Leak freeze min breaths", "leak_freeze_min_breaths")
    add_apap_float(apap_frame, row, 1, "Leak freeze max (s)", "leak_freeze_max_s")
    add_apap_float(apap_frame, row, 2, "FOT breath notch (0/1)", "breath_notch")
    row += 1
    add_apap_float(apap_frame, row, 0, "Breath Vi x median", "breath_min_vol_ratio")
    add_apap_float(apap_frame, row, 1, "Hyp->CA guard (s)", "hyp_ca_guard_s")
    row += 1
    add_apap_float(apap_frame, row, 0, "EPR rel. baseline (0/1)", "epr_rel_baseline")
    add_apap_float(apap_frame, row, 1, "APAP rel. baseline (0/1)", "rel_baseline")
    row += 1
    add_apap_float(apap_frame, row, 0, "EPR baseline slow tau (s)", "epr_baseline_slow_tau_s")
    row += 1
    add_apap_float(apap_frame, row, 0, "Hyp re-arm ratio", "hyp_rearm_ratio")
    add_apap_float(apap_frame, row, 1, "Hyp re-arm breaths", "hyp_rearm_breaths")
    row += 1
    add_apap_float(apap_frame, row, 0, "Raise std (cmH2O/s)", "raise_rate_standard")
    add_apap_float(apap_frame, row, 1, "Raise soft (cmH2O/s)", "raise_rate_soft")
    add_apap_float(apap_frame, row, 2, "For Her scale", "raise_rate_her_scale")
    row += 1
    add_apap_float(apap_frame, row, 0, "Her OA above (cmH2O)", "her_oa_above_cmh2o")
    add_apap_float(apap_frame, row, 1, "Her OA above scale", "her_oa_above_scale")
    row += 1
    add_apap_float(apap_frame, row, 0, "Lower rate (cmH2O/s)", "lower_rate")
    add_apap_float(apap_frame, row, 1, "Stable before lower (s)", "stable_before_lower_s")

    row += 1
    ttk.Separator(apap_frame, orient=tk.HORIZONTAL).grid(row=row, column=0, columnspan=6, sticky="ew", pady=8)
    row += 1
    ttk.Label(apap_frame, text="FOT / OSA").grid(row=row, column=0, columnspan=6, sticky="w", pady=(0, 2))
    row += 1
    add_apap_float(apap_frame, row, 0, "Osc amp (cmH2O)", "osc_amplitude")
    add_apap_float(apap_frame, row, 1, "Osc amp max", "osc_amplitude_max")
    add_apap_float(apap_frame, row, 2, "Osc freq (Hz)", "osc_freq_hz")
    row += 1
    add_apap_float(apap_frame, row, 0, "FOT eval (s)", "fot_eval_period_s")
    add_apap_float(apap_frame, row, 1, "Min P p-p", "osc_min_p_pp")
    add_apap_float(apap_frame, row, 2, "OSA Q p-p thr (slm)", "osa_flow_pp_thresh")
    row += 1
    add_apap_float(apap_frame, row, 0, "Apnea min (s)", "apnea_min_s")
    add_apap_float(apap_frame, row, 1, "OA raise /s", "oa_raise_per_s")
    add_apap_float(apap_frame, row, 2, "OA raise max", "oa_raise_max")

    row += 1
    ttk.Separator(apap_frame, orient=tk.HORIZONTAL).grid(row=row, column=0, columnspan=6, sticky="ew", pady=8)
    row += 1
    ttk.Checkbutton(apap_frame, text="Flow limitation", variable=apap_bool_vars["fl_enable"]).grid(
        row=row, column=0, columnspan=2, sticky="w"
    )
    row += 1
    add_apap_float(apap_frame, row, 0, "FL ratio", "fl_ratio")
    add_apap_int(apap_frame, row, 1, "FL breaths need", "fl_breaths_needed")
    add_apap_int(apap_frame, row, 2, "FL clear breaths", "fl_clear_breaths")
    row += 1
    add_apap_float(apap_frame, row, 0, "FL raise scale", "fl_raise_scale")
    add_apap_float(apap_frame, row, 1, "FL min peak (slm)", "fl_min_peak")
    add_apap_float(apap_frame, row, 2, "FL baseline α", "fl_baseline_alpha")
    row += 1
    add_apap_float(apap_frame, row, 0, "FL flat ratio", "fl_flat_ratio")
    add_apap_float(apap_frame, row, 1, "FL baseline reject x", "fl_baseline_reject")
    add_apap_float(apap_frame, row, 2, "Baseline restore max (s)", "baseline_restore_max_s")
    row += 1
    add_apap_float(apap_frame, row, 0, "FL dip ratio", "fl_dip_ratio")
    add_apap_float(apap_frame, row, 1, "FL segment floor (slm)", "fl_seg_floor")
    add_apap_int(apap_frame, row, 2, "FL amplitude arm (0/1)", "fl_amp_arm")

    row += 1
    ttk.Separator(apap_frame, orient=tk.HORIZONTAL).grid(row=row, column=0, columnspan=6, sticky="ew", pady=8)
    row += 1
    ttk.Checkbutton(apap_frame, text="Hypopnea", variable=apap_bool_vars["hyp_enable"]).grid(
        row=row, column=0, columnspan=2, sticky="w"
    )
    row += 1
    add_apap_float(apap_frame, row, 0, "Hyp ratio", "hyp_ratio")
    add_apap_int(apap_frame, row, 1, "Hyp breaths need", "hyp_breaths_needed")
    add_apap_float(apap_frame, row, 2, "Hyp min dur (s)", "hyp_min_duration_s")
    row += 1
    add_apap_float(apap_frame, row, 0, "Hyp raise scale", "hyp_raise_scale")
    add_apap_float(apap_frame, row, 1, "Hyp hold (s)", "hyp_hold_s")
    add_apap_float(apap_frame, row, 2, "Hyp raise shape frac", "hyp_raise_shape_frac")

    row += 1
    ttk.Separator(apap_frame, orient=tk.HORIZONTAL).grid(row=row, column=0, columnspan=6, sticky="ew", pady=8)
    row += 1
    ttk.Checkbutton(apap_frame, text="Snore", variable=apap_bool_vars["snore_enable"]).grid(
        row=row, column=0, columnspan=2, sticky="w"
    )
    row += 1
    add_apap_float(apap_frame, row, 0, "Snore RMS thr", "snore_rms_thresh")
    add_apap_float(apap_frame, row, 1, "Snore on (s)", "snore_on_s")
    add_apap_float(apap_frame, row, 2, "Snore off (s)", "snore_off_s")
    row += 1
    add_apap_float(apap_frame, row, 0, "Snore raise scale", "snore_raise_scale")
    add_apap_float(apap_frame, row, 1, "Snore HP (Hz)", "snore_hp_hz")

    row += 1
    btn_row = ttk.Frame(apap_frame)
    btn_row.grid(row=row, column=0, columnspan=6, sticky="ew", pady=(12, 4))
    ttk.Button(btn_row, text="Apply APAP Tuning", command=apply_apap_tuning).pack(side=tk.LEFT)
    ttk.Button(btn_row, text="Reset Defaults (UI)", command=reset_apap_defaults).pack(side=tk.LEFT, padx=8)
    ttk.Button(btn_row, text="Query APAP Events", command=query_apap_events).pack(side=tk.LEFT)
    row += 1
    ttk.Label(apap_frame, textvariable=apap_events_var, wraplength=900).grid(
        row=row, column=0, columnspan=6, sticky="w", pady=(6, 0)
    )

    def update(_frame: int):
        edf_line, apap_line = reader.edf_apap_snapshot()
        if edf_line:
            edf_status_var.set(edf_line.strip())
        if apap_line:
            apap_events_var.set(apap_line.strip())

        status_poll_counter[0] += 1
        if status_poll_counter[0] % 10 == 0:
            reader.send_command("therapy status")

        (
            times,
            pressure,
            pressure_mask,
            flow,
            pid_rpm_cmd,
            mech_rpm,
            bench_times,
            bench_rpm_cmd,
            bench_rpm_act,
            last,
            n,
            nbytes,
            nlines,
            other,
            elapsed,
            bench,
        ) = reader.snapshot()
        if times:
            waiting.set_visible(False)
            line_p.set_data(times, pressure)
            line_p_mask.set_data(times, pressure_mask)
            line_f.set_data(times, flow)
            rpm_times = [t for t, rpm in zip(times, pid_rpm_cmd) if rpm is not None]
            rpm_values = [rpm for rpm in pid_rpm_cmd if rpm is not None]
            line_pid_rpm.set_data(rpm_times, rpm_values)
            mech_times = [t for t, rpm in zip(times, mech_rpm) if rpm is not None]
            mech_values = [rpm for rpm in mech_rpm if rpm is not None]
            line_mech_rpm.set_data(mech_times, mech_values)
            t_right = max(args.window, times[-1] + 0.5)
            t_left = max(0, times[-1] - args.window)
            ax_p.set_xlim(t_left, t_right)
            ax_f.set_xlim(t_left, t_right)
            if pressure or pressure_mask:
                p_all = list(pressure) + list(pressure_mask)
                p_min, p_max = min(p_all), max(p_all)
                ax_p.set_ylim(max(0, p_min - 1), min(30, p_max + 2))
            if flow:
                f_min, f_max = min(flow), max(flow)
                margin = max(0.2, (f_max - f_min) * 0.15)
                ax_f.set_ylim(f_min - margin, f_max + margin)
            rpm_axis_values = list(rpm_values) + list(mech_values) + bench_rpm_cmd + bench_rpm_act
            if rpm_axis_values:
                rpm_min = min(rpm_axis_values)
                rpm_max = max(rpm_axis_values)
                margin = max(500.0, (rpm_max - rpm_min) * 0.1)
                ax_rpm.set_ylim(max(0, rpm_min - margin), rpm_max + margin)
        else:
            line_p_mask.set_data([], [])
            line_pid_rpm.set_data([], [])
            line_mech_rpm.set_data([], [])
            waiting.set_visible(True)
            hint = "no serial data yet" if nbytes == 0 else "serial OK, no T,... lines yet"
            waiting.set_text(
                f"Waiting for telemetry ({hint})\n"
                f"bytes={nbytes}  lines={nlines}\n"
                "Try: --monitor  or  --list for COM ports"
            )

        if bench_times:
            line_rpm_cmd.set_data(bench_times, bench_rpm_cmd)
            line_rpm_act.set_data(bench_times, bench_rpm_act)
        else:
            line_rpm_cmd.set_data([], [])
            line_rpm_act.set_data([], [])

        serial_plot_var.set(
            f"Serial: t={elapsed:4.0f}s  bytes={nbytes}  lines={nlines}  "
            f"telem={n}  bench={bench}  last={last or other or '—'}"
        )
        signal_panel.update_values(
            signal_metric_values(times, pressure, pressure_mask, flow, pid_rpm_cmd, mech_rpm)
        )
        return line_p, line_p_mask, line_pid_rpm, line_mech_rpm, line_rpm_cmd, line_rpm_act, line_f, waiting

    ani = animation.FuncAnimation(
        fig,
        update,
        interval=50,
        blit=False,
        cache_frame_data=False,
    )
    _ = ani

    print(f"Plot running on {port}. Sent 'telem on' to the device.")
    print("PID RPM cmd (orange steps) and measured RPM mech (crimson) overlay on pressure.")
    print("If the plot stays empty, run:  python cpap_telemetry_plot.py --port", port, "--monitor")
    print("Use the Therapy Controls tab to send start/stop and settings commands.")
    print("Use the FOC Tuning tab to adjust CM4 inner-loop PI gains before/during therapy.")
    print("Close the GUI window to exit.")

    refresh_status()
    send("therapy status")

    def on_close() -> None:
        root.quit()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    try:
        root.mainloop()
    finally:
        reader.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
