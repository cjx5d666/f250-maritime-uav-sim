#!/usr/bin/env python3
"""Thin F250 maritime control panel (PyQt5, dark professional theme).

One-shot environment interlock: a freshly launched P0 hover environment can run
EITHER a Route mission OR a Flight-Control check. Running either one consumes the
environment (the vehicle is no longer at a clean P0), so both task buttons lock
until the operator stops and rebuilds. Only non-destructive actions remain
available afterwards: Stop All, View Result Plot, Open Current Evidence.
"""
import argparse
from datetime import datetime, timezone
import json
import os
import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent

SENSOR_LABELS = {
    "lidar": "LiDAR",
    "depth": "Depth",
}

# UI action key -> human label (the thin-panel action set)
BUTTON_LABELS = {
    "launch": "Launch Environment",
    "stop": "Stop All",
    "route": "Run Route Mission",
    "flight_control": "Run Flight-Control Check",
    "plots": "View Result Plot",
    "open_current": "Open Current Evidence",
}

# UI action key -> backing script. "plots" opens only the current result plot.
SCRIPT_NAMES = {
    "launch": "f250_start_to_p0_hover.sh",
    "route": "f250_run_p0_p8_route.sh",
    "flight_control": "f250_run_fc_3_10_steady_state.sh",
    "result_plot": "f250_generate_result_plot.sh",
    "stop": "f250_stop_all.sh",
}

# ---------------------------------------------------------------------------
# State model (derived from runtime_state/active_task.env)
# ---------------------------------------------------------------------------
# task is a fixed identity label written by each script and never reverts:
#   launch -> route -> (stays route)   /   launch -> flight_control
# So the interlock is driven by `task`: only task=launch + ready is "fresh".
STOPPED_STATES = {"stopped", "stop_requested"}
FAILED_STATES = {
    "failed",
    "screen_exited_early",
    "blocked_existing_runtime",
    "blocked_existing_screen",
    "perception_gate_failed",
    "prealign_yaw_failed",
    "recorder_failed",
    "postprocess_failed",
}
DONE_STATES = {"complete"}
# launch writes hover_ready only after the start screen survives the fixed P0 settle delay.
LAUNCH_READY_STATES = {"hover_ready"}
LAUNCH_STARTING_STATES = {"prepared", "screen_started"}

# Environment phases (single source of truth for the interlock matrix)
PHASE_DOWN = "down"          # no runtime / stopped
PHASE_STARTING = "starting"  # launch in progress
PHASE_READY = "ready"        # fresh P0 hover, nothing consumed it yet
PHASE_RUNNING = "running"    # a task is mid-run
PHASE_CONSUMED = "consumed"  # route/FC has run; env dirty, rebuild required
PHASE_FAILED = "failed"      # last task/launch failed


def resolve_project_root():
    env_root = os.environ.get("F250_PROJECT_ROOT")
    if env_root:
        return Path(env_root).expanduser().resolve()
    for candidate in [SCRIPT_DIR / "../../../..", SCRIPT_DIR / "../../../../..", Path.cwd()]:
        candidate = candidate.resolve()
        if (candidate / "catkin_ws/src/f250_maritime_uav_sim").is_dir():
            return candidate
    raise RuntimeError("Unable to resolve F250 project root; set F250_PROJECT_ROOT")


PROJECT_ROOT = resolve_project_root()
RUNTIME_STATE_DIR = Path(os.environ.get("F250_RUNTIME_STATE_DIR", PROJECT_ROOT / "runtime_state")).expanduser().resolve()
RUN_ROOT = Path(os.environ.get("RUN_ROOT", RUNTIME_STATE_DIR / "work")).expanduser().resolve()
CURRENT_DIR = Path(os.environ.get("F250_EVIDENCE_CURRENT_DIR", PROJECT_ROOT / "evidence/current")).expanduser().resolve()
ACTIVE_STATUS = Path(os.environ.get("F250_ACTIVE_TASK_ENV", RUNTIME_STATE_DIR / "active_task.env")).expanduser().resolve()
ACTIVE_SENSOR = Path(os.environ.get("F250_ACTIVE_SENSOR_ENV", RUNTIME_STATE_DIR / "active_sensor.env")).expanduser().resolve()
ROUTE_EVIDENCE_RUN = CURRENT_DIR / "route_p0_p8"

def script_path(key):
    return SCRIPT_DIR / SCRIPT_NAMES[key]


def read_env_file(path):
    data = {}
    try:
        with open(path, "r", encoding="utf-8") as handle:
            for line in handle:
                line = line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                key, value = line.split("=", 1)
                data[key.strip()] = value.strip().strip('"').strip("'")
    except FileNotFoundError:
        pass
    return data


def read_json(path):
    try:
        with open(path, "r", encoding="utf-8") as handle:
            return json.load(handle)
    except Exception:
        return {}


def normalize_token(value):
    return str(value or "").strip().lower()


def parse_bool(value, default=False):
    if value is None or value == "":
        return default
    return normalize_token(value) in {"1", "true", "yes", "on"}


def current_sensor():
    env_sensor = os.environ.get("PERCEPTION_SOURCE") or os.environ.get("F250_SENSOR")
    if env_sensor in SENSOR_LABELS:
        return env_sensor
    stored_env = read_env_file(ACTIVE_SENSOR)
    for key in ("PERCEPTION_SOURCE", "F250_SENSOR", "sensor"):
        stored = stored_env.get(key)
        if stored in SENSOR_LABELS:
            return stored
    return "lidar"


def write_active_sensor(sensor):
    ACTIVE_SENSOR.parent.mkdir(parents=True, exist_ok=True)
    with open(ACTIVE_SENSOR, "w", encoding="utf-8") as handle:
        handle.write(f"PERCEPTION_SOURCE={sensor}\n")
        handle.write(f"F250_SENSOR={sensor}\n")
        handle.write(f"sensor={sensor}\n")
        handle.write(f"sensor_label={SENSOR_LABELS.get(sensor, sensor)}\n")
        handle.write("selected_by=control_panel\n")
        handle.write(f"updated_at={datetime.now(timezone.utc).isoformat()}\n")


def task_summary(task):
    index = read_json(CURRENT_DIR / "index.json")
    task_key = {"route": "route_p0_p8", "flight_control": "fc_3_10"}.get(task, task)
    return index.get("tasks", {}).get(task_key, {}) if isinstance(index, dict) else {}


def status_snapshot():
    """Return (status_dict, task, state, runtime_active)."""
    status = read_env_file(ACTIVE_STATUS)
    task = normalize_token(status.get("task") or status.get("source_task") or status.get("current_task"))
    state = normalize_token(status.get("state"))
    runtime_active = parse_bool(status.get("runtime_active"), default=False)
    return status, task, state, runtime_active


def environment_phase(status, task, state, runtime_active):
    """Collapse the raw status into one of the PHASE_* values."""
    if not status:
        return PHASE_DOWN
    if task == "stopped" or state in STOPPED_STATES:
        return PHASE_DOWN
    if state in FAILED_STATES:
        return PHASE_FAILED
    if task == "launch":
        if state in LAUNCH_STARTING_STATES:
            return PHASE_STARTING
        if state in LAUNCH_READY_STATES:
            return PHASE_READY
        # any other live launch state is still starting; only hover_ready is READY.
        return PHASE_STARTING if runtime_active else PHASE_DOWN
    # task is route or flight_control: the environment has been consumed.
    if task in ("route", "flight_control"):
        if state in DONE_STATES:
            return PHASE_CONSUMED
        if state in FAILED_STATES:
            return PHASE_FAILED
        return PHASE_RUNNING
    return PHASE_DOWN if not runtime_active else PHASE_RUNNING


ENVIRONMENT_TEXT = {
    PHASE_DOWN: "Not running",
    PHASE_STARTING: "Starting",
    PHASE_READY: "Ready",
    PHASE_RUNNING: "Running",
    PHASE_CONSUMED: "Relaunch needed",
    PHASE_FAILED: "Failed",
}


def sensor_display_name(value, missing="unknown"):
    token = normalize_token(value)
    if not token:
        return missing
    return SENSOR_LABELS.get(token, token)


def status_sensor_text(status):
    if not isinstance(status, dict):
        return ""
    for key in ("sensor", "perception_source", "source_p0_sensor", "F250_SENSOR", "PERCEPTION_SOURCE"):
        value = status.get(key)
        if value:
            return sensor_display_name(value, missing="")
    label = status.get("sensor_label") or status.get("SENSOR_LABEL")
    return str(label).strip() if label else ""


def with_sensor(label, sensor):
    return "%s (%s)" % (label, sensor) if sensor else label


def current_task_text(status, task, state, phase):
    sensor = status_sensor_text(status)
    if phase in (PHASE_DOWN, PHASE_READY):
        return "None"
    if phase == PHASE_STARTING:
        return with_sensor("Starting environment", sensor)
    if phase == PHASE_RUNNING:
        labels = {
            "launch": "Starting environment",
            "route": "Route mission",
            "flight_control": "Flight-control check",
        }
        return with_sensor(labels.get(task, "Running"), sensor)
    if phase == PHASE_CONSUMED:
        labels = {
            "route": "Route mission complete",
            "flight_control": "Flight-control check complete",
        }
        return labels.get(task, "Complete")
    if phase == PHASE_FAILED:
        labels = {
            "launch": "Environment failed",
            "route": "Route mission failed",
            "flight_control": "Flight-control check failed",
        }
        return labels.get(task, "Failed")
    return state or "Unknown"


def result_sensor_text(summary):
    if not isinstance(summary, dict) or not summary or summary.get("available") is False:
        return "missing"
    for key in ("sensor", "perception_source", "source_p0_sensor"):
        value = summary.get(key)
        if value:
            return sensor_display_name(value)
    return "unknown"


def results_text_from_summaries(route, flight):
    route_present = isinstance(route, dict) and bool(route) and route.get("available") is not False
    flight_present = isinstance(flight, dict) and bool(flight) and flight.get("available") is not False
    if not route_present and not flight_present:
        return "No saved results"
    return "Route %s · FC %s" % (result_sensor_text(route), result_sensor_text(flight))


def last_results_text():
    return results_text_from_summaries(task_summary("route"), task_summary("flight_control"))


def action_message(key):
    return {
        "launch": "Starting environment",
        "route": "Starting route mission",
        "flight_control": "Starting flight-control check",
        "stop": "Stop requested",
    }.get(key, "Started")


def launch_process(key, sensor=None, extra_env=None):
    path = script_path(key)
    if not path.exists():
        raise FileNotFoundError(str(path))
    env = os.environ.copy()
    env["F250_PROJECT_ROOT"] = str(PROJECT_ROOT)
    env["RUN_ROOT"] = str(RUN_ROOT)
    env["F250_RUNTIME_STATE_DIR"] = str(RUNTIME_STATE_DIR)
    env["F250_ACTIVE_TASK_ENV"] = str(ACTIVE_STATUS)
    env["F250_ACTIVE_SENSOR_ENV"] = str(ACTIVE_SENSOR)
    env["F250_EVIDENCE_CURRENT_DIR"] = str(CURRENT_DIR)
    if sensor:
        env["PERCEPTION_SOURCE"] = sensor
    if extra_env:
        env.update(extra_env)
    cmd = ["bash", str(path)] if path.suffix == ".sh" else [sys.executable, str(path)]
    return subprocess.Popen(
        cmd,
        cwd=str(PROJECT_ROOT),
        env=env,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )


def run_plots():
    """Generate/open the current route result plot. Returns a status message."""
    if not ROUTE_EVIDENCE_RUN.is_dir():
        return "No result evidence available yet"
    result = script_path("result_plot")
    if not result.exists():
        return "Result plot script is missing"
    env = os.environ.copy()
    env["F250_PROJECT_ROOT"] = str(PROJECT_ROOT)
    env["RUN_ROOT"] = str(RUN_ROOT)
    env["F250_RUNTIME_STATE_DIR"] = str(RUNTIME_STATE_DIR)
    env["F250_ACTIVE_TASK_ENV"] = str(ACTIVE_STATUS)
    env["F250_ACTIVE_SENSOR_ENV"] = str(ACTIVE_SENSOR)
    env["F250_EVIDENCE_CURRENT_DIR"] = str(CURRENT_DIR)
    subprocess.Popen(["bash", str(result)], cwd=str(PROJECT_ROOT), env=env,
                     stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                     stderr=subprocess.DEVNULL, start_new_session=True)
    return "Generating result plot"


def open_path(path):
    path = Path(path)
    if not path.exists():
        return False
    if os.environ.get("DISPLAY"):
        subprocess.Popen(["xdg-open", str(path)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return True
    return False


# ---------------------------------------------------------------------------
# Interlock matrix: phase -> which controls are enabled
# ---------------------------------------------------------------------------
def control_enabled(key, phase):
    """Single source of truth for the one-shot interlock."""
    if key == "stop":
        return True  # always available
    if key == "open_current":
        return CURRENT_DIR.exists()
    if key == "plots":
        return ROUTE_EVIDENCE_RUN.is_dir() and phase not in (PHASE_STARTING, PHASE_RUNNING)
    if key == "sensor":
        return phase == PHASE_DOWN  # only choose source before launching
    if key == "launch":
        return phase == PHASE_DOWN  # force stop+rebuild for a fresh env
    if key in ("route", "flight_control"):
        return phase == PHASE_READY  # ONLY on a fresh, unconsumed environment
    return False


# Phase -> (badge color, badge text). Saturated fills with light text so the
# capsule stays legible on the warm off-white background.
PHASE_BADGE = {
    PHASE_DOWN: ("#8a8275", "OFFLINE"),
    PHASE_STARTING: ("#b8862f", "STARTING"),
    PHASE_READY: ("#3f7d52", "READY"),
    PHASE_RUNNING: ("#b8862f", "RUNNING"),
    PHASE_CONSUMED: ("#3a7d97", "RELAUNCH NEEDED"),
    PHASE_FAILED: ("#c0563f", "FAILED"),
}

# "Warm off-white + sage accent" light theme. Window is the warm base; cards sit
# slightly brighter with a hairline border and read as gently raised panels.
PANEL_QSS = """
QWidget {
    background: #f3f1ec; color: #2a2620;
    font-family: "Inter", "Segoe UI", "Cantarell", "DejaVu Sans", sans-serif;
    font-size: 13px;
}
QLabel#title {
    font-size: 18px; font-weight: 600; color: #2a2620; letter-spacing: 0.2px;
}
QLabel#subtitle { color: #9a9081; font-size: 11px; letter-spacing: 0.8px; }
QFrame#divider { background: #e3ded3; max-height: 1px; min-height: 1px; border: none; }
QLabel#badge {
    font-size: 11px; font-weight: 700; letter-spacing: 1.0px;
    padding: 6px 13px; border-radius: 11px; color: #fbfaf7;
}
QGroupBox {
    background: #fbfaf7; border: 1px solid #e6e1d6; border-radius: 10px;
    margin-top: 12px; padding: 14px 14px 14px 14px;
    font-weight: 600; color: #9a9081; font-size: 11px; letter-spacing: 0.6px;
}
QGroupBox::title {
    subcontrol-origin: margin; left: 14px; top: 2px; padding: 0 6px;
}
QLabel.status { color: #6f675b; }
QLabel.statusval {
    color: #2a2620; font-weight: 600;
    font-family: "JetBrains Mono", "DejaVu Sans Mono", monospace; font-size: 12px;
}
QRadioButton { padding: 5px 0; spacing: 7px; color: #2a2620; }
QRadioButton::indicator { width: 15px; height: 15px; }
QPushButton {
    background: #efe9dd; border: 1px solid #d9d2c4; border-radius: 8px;
    padding: 11px 12px; color: #3a342b; font-weight: 600;
}
QPushButton:hover:enabled { background: #e8e1d2; border-color: #c8bfac; }
QPushButton:pressed:enabled { background: #ded5c2; }
QPushButton:disabled { background: #f0ede6; color: #b7afa0; border-color: #e6e1d6; }
QPushButton#primary:enabled { background: #3f7d52; border-color: #366f48; color: #fbfaf7; }
QPushButton#primary:hover:enabled { background: #46895b; }
QPushButton#primary:pressed:enabled { background: #356a45; }
QPushButton#danger:enabled { background: #c0563f; border-color: #a94a35; color: #fff5f2; }
QPushButton#danger:hover:enabled { background: #cb6048; }
QPushButton#danger:pressed:enabled { background: #a94a35; }
QLabel#message {
    color: #6f675b; padding: 10px 12px; background: #fbfaf7;
    border: 1px solid #e6e1d6; border-radius: 8px; font-size: 12px;
}
"""


class ControlPanel(object):
    def __init__(self, app):
        from PyQt5 import QtCore, QtWidgets

        self.QtCore = QtCore
        self.QtWidgets = QtWidgets
        self.app = app
        self.win = QtWidgets.QWidget()
        self.win.setWindowTitle("F250 Maritime Control Panel")
        self.win.setMinimumSize(480, 560)
        self.win.setStyleSheet(PANEL_QSS)

        self.sensor_buttons = {}
        self.action_buttons = {}
        self.status_vals = {}
        self._build()

        self.timer = QtCore.QTimer(self.win)
        self.timer.setInterval(2000)
        self.timer.timeout.connect(self.refresh_state)
        self.timer.start()
        self.refresh_state()

    def _build(self):
        Q = self.QtWidgets
        root = Q.QVBoxLayout(self.win)
        root.setContentsMargins(16, 16, 16, 14)
        root.setSpacing(10)

        # Header: title (+subtitle) + environment badge
        header = Q.QHBoxLayout()
        header.setSpacing(10)
        titlewrap = Q.QVBoxLayout()
        titlewrap.setSpacing(1)
        title = Q.QLabel("F250 Maritime")
        title.setObjectName("title")
        subtitle = Q.QLabel("CONTROL PANEL")
        subtitle.setObjectName("subtitle")
        titlewrap.addWidget(title)
        titlewrap.addWidget(subtitle)
        header.addLayout(titlewrap)
        header.addStretch(1)
        self.badge = Q.QLabel("CHECKING")
        self.badge.setObjectName("badge")
        self.badge.setAlignment(self.QtCore.Qt.AlignCenter)
        header.addWidget(self.badge, 0, self.QtCore.Qt.AlignTop)
        root.addLayout(header)

        divider = Q.QFrame()
        divider.setObjectName("divider")
        divider.setFrameShape(Q.QFrame.HLine)
        root.addWidget(divider)

        # Sensor source
        sensor_box = Q.QGroupBox("Sensor Source")
        sensor_lay = Q.QHBoxLayout(sensor_box)
        for value, label in SENSOR_LABELS.items():
            rb = Q.QRadioButton(label)
            rb.toggled.connect(lambda checked, v=value: self._on_sensor(v, checked))
            sensor_lay.addWidget(rb)
            self.sensor_buttons[value] = rb
        sensor_lay.addStretch(1)
        root.addWidget(sensor_box)

        # Status block
        status_box = Q.QGroupBox("Status")
        grid = Q.QGridLayout(status_box)
        grid.setVerticalSpacing(6)
        rows = [
            ("environment", "Environment"),
            ("current_task", "Current Task"),
            ("last_results", "Last Results"),
        ]
        for i, (key, label) in enumerate(rows):
            name = Q.QLabel(label)
            name.setProperty("class", "status")
            val = Q.QLabel("checking")
            val.setProperty("class", "statusval")
            grid.addWidget(name, i, 0, self.QtCore.Qt.AlignLeft)
            grid.addWidget(val, i, 1, self.QtCore.Qt.AlignRight)
            self.status_vals[key] = val
        grid.setColumnStretch(1, 1)
        root.addWidget(status_box)

        # Actions
        actions = Q.QGroupBox("Actions")
        ag = Q.QGridLayout(actions)
        ag.setHorizontalSpacing(8)
        ag.setVerticalSpacing(8)
        layout_map = [
            ("launch", 0, 0, "primary"),
            ("stop", 0, 1, "danger"),
            ("route", 1, 0, None),
            ("flight_control", 1, 1, None),
            ("plots", 2, 0, None),
            ("open_current", 2, 1, None),
        ]
        for key, r, c, obj in layout_map:
            btn = Q.QPushButton(BUTTON_LABELS[key])
            if obj:
                btn.setObjectName(obj)
            btn.clicked.connect(lambda _checked, k=key: self.run_action(k))
            ag.addWidget(btn, r, c)
            self.action_buttons[key] = btn
        ag.setColumnStretch(0, 1)
        ag.setColumnStretch(1, 1)
        root.addWidget(actions)

        root.addStretch(1)
        self.message = Q.QLabel("Ready")
        self.message.setObjectName("message")
        self.message.setWordWrap(True)
        root.addWidget(self.message)

    def _on_sensor(self, value, checked):
        if not checked:
            return
        if not self.sensor_buttons[value].isEnabled():
            return
        write_active_sensor(value)
        self.message.setText("Sensor source: %s" % SENSOR_LABELS.get(value, value))

    def run_action(self, key):
        try:
            if key == "open_current":
                if open_path(CURRENT_DIR):
                    self.message.setText("Opened current evidence")
                else:
                    self.message.setText("Current evidence is not available yet")
                return
            if key == "plots":
                self.message.setText(run_plots())
                return
            sensor = self._selected_sensor()
            if sensor:
                write_active_sensor(sensor)
            launch_process(key, sensor=sensor)
            self.message.setText(action_message(key))
        except Exception as exc:  # noqa: BLE001 - surface any failure to the operator
            self.message.setText(str(exc))
            if self.QtWidgets.QMessageBox:
                self.QtWidgets.QMessageBox.critical(self.win, "F250 control panel", str(exc))
        finally:
            self.refresh_state()

    def _selected_sensor(self):
        for value, rb in self.sensor_buttons.items():
            if rb.isChecked():
                return value
        return current_sensor()

    def refresh_state(self):
        status, task, state, runtime_active = status_snapshot()
        phase = environment_phase(status, task, state, runtime_active)

        color, text = PHASE_BADGE.get(phase, ("#6b7280", "UNKNOWN"))
        self.badge.setText(text)
        self.badge.setStyleSheet("background: %s;" % color)

        # Status lines
        self.status_vals["environment"].setText(ENVIRONMENT_TEXT.get(phase, "Unknown"))
        self.status_vals["current_task"].setText(current_task_text(status, task, state, phase))
        self.status_vals["last_results"].setText(last_results_text())

        # Sensor selection reflects stored value
        sensor = current_sensor()
        if sensor in self.sensor_buttons and not self.sensor_buttons[sensor].isChecked():
            self.sensor_buttons[sensor].blockSignals(True)
            self.sensor_buttons[sensor].setChecked(True)
            self.sensor_buttons[sensor].blockSignals(False)

        # Interlock
        for value, rb in self.sensor_buttons.items():
            rb.setEnabled(control_enabled("sensor", phase))
        for key, btn in self.action_buttons.items():
            btn.setEnabled(control_enabled(key, phase))

def dry_run_self_test():
    """Validate the thin-panel boundary and the interlock matrix without a GUI."""
    failures = []

    # 1. Backing scripts exist.
    for key, name in SCRIPT_NAMES.items():
        if not (SCRIPT_DIR / name).exists():
            failures.append("missing script for %s: %s" % (key, name))

    # 2. Action set is exactly the thin-panel boundary (no regressed old features).
    expected = {
        "Launch Environment",
        "Stop All",
        "Run Route Mission",
        "Run Flight-Control Check",
        "View Result Plot",
        "Open Current Evidence",
    }
    if set(BUTTON_LABELS.values()) != expected:
        failures.append("button labels differ from the thin-panel boundary")
    forbidden_words = ("map", "candidate", "history", "screenshot", "log", "metric detail")
    joined = " ".join(BUTTON_LABELS.values()).lower()
    for word in forbidden_words:
        if word in joined:
            failures.append("forbidden feature exposed in buttons: %s" % word)

    # 3. Interlock invariants (the core requirement).
    #    Route/FC must be enabled ONLY on a fresh (ready) environment.
    for key in ("route", "flight_control"):
        if not control_enabled(key, PHASE_READY):
            failures.append("%s should be enabled when environment is READY" % key)
        for bad in (PHASE_DOWN, PHASE_STARTING, PHASE_RUNNING, PHASE_CONSUMED, PHASE_FAILED):
            if control_enabled(key, bad):
                failures.append("%s must be locked in phase %s" % (key, bad))
    # Launch only when down; stop always.
    if not control_enabled("launch", PHASE_DOWN):
        failures.append("launch should be enabled when environment is DOWN")
    if control_enabled("launch", PHASE_CONSUMED):
        failures.append("launch must be locked while environment is CONSUMED (force stop first)")
    if not control_enabled("stop", PHASE_CONSUMED):
        failures.append("stop must always be available")
    # Sensor source locked once running/consumed.
    if control_enabled("sensor", PHASE_CONSUMED):
        failures.append("sensor source must be locked after the environment is consumed")
    # Result plots stay locked while a launch/task can be producing a newer result.
    if control_enabled("plots", PHASE_STARTING):
        failures.append("result plot must be locked while environment is STARTING")
    if control_enabled("plots", PHASE_RUNNING):
        failures.append("result plot must be locked while a task is RUNNING")

    # 4. State folding: READY means launch/hover_ready only; route finalizing is still running.
    launch_screen = {"task": "launch", "state": "screen_started", "runtime_active": "true"}
    if environment_phase(launch_screen, "launch", "screen_started", True) != PHASE_STARTING:
        failures.append("screen_started should remain STARTING")
    launch_ready = {"task": "launch", "state": "hover_ready", "runtime_active": "true"}
    if environment_phase(launch_ready, "launch", "hover_ready", True) != PHASE_READY:
        failures.append("hover_ready should be READY")
    for running_state in ("prealign_yaw_done", "finalizing"):
        route_status = {"task": "route", "state": running_state, "runtime_active": "true"}
        if environment_phase(route_status, "route", running_state, True) != PHASE_RUNNING:
            failures.append("route %s should remain RUNNING" % running_state)
    if environment_phase({"task": "route", "state": "complete", "runtime_active": "true"}, "route", "complete", True) != PHASE_CONSUMED:
        failures.append("route complete should be CONSUMED")

    # 5. User-facing status text stays compact and sensor-aware.
    expected_env = {
        PHASE_DOWN: "Not running",
        PHASE_STARTING: "Starting",
        PHASE_READY: "Ready",
        PHASE_RUNNING: "Running",
        PHASE_CONSUMED: "Relaunch needed",
        PHASE_FAILED: "Failed",
    }
    for phase, expected_text in expected_env.items():
        if ENVIRONMENT_TEXT.get(phase) != expected_text:
            failures.append("unexpected environment text for %s" % phase)
    if current_task_text({}, "stopped", "stopped", PHASE_DOWN) != "None":
        failures.append("down state should show no current task")
    if current_task_text({"sensor": "depth"}, "launch", "prepared", PHASE_STARTING) != "Starting environment (Depth)":
        failures.append("starting state should show launch sensor")
    if current_task_text({"sensor": "lidar"}, "route", "recording", PHASE_RUNNING) != "Route mission (LiDAR)":
        failures.append("route running state should show route sensor")
    if current_task_text({"source_p0_sensor": "depth"}, "flight_control", "background_worker_starting", PHASE_RUNNING) != "Flight-control check (Depth)":
        failures.append("FC running state should show source P0 sensor")
    if current_task_text({}, "route", "complete", PHASE_CONSUMED) != "Route mission complete":
        failures.append("consumed route state should show complete task")
    if results_text_from_summaries({"sensor": "lidar"}, {"sensor": "depth"}) != "Route LiDAR · FC Depth":
        failures.append("last results should show per-task sensors")
    if results_text_from_summaries({}, {}) != "No saved results":
        failures.append("empty evidence should show no saved results")

    if failures:
        print("\n".join(failures), file=sys.stderr)
        return 2
    print(json.dumps({
        "ok": True,
        "project_root": str(PROJECT_ROOT),
        "run_root": str(RUN_ROOT),
        "runtime_state_dir": str(RUNTIME_STATE_DIR),
        "current_evidence_dir": str(CURRENT_DIR),
        "actions": sorted(BUTTON_LABELS.keys()),
    }, sort_keys=True))
    return 0


def main():
    parser = argparse.ArgumentParser(description="Thin F250 maritime control panel (PyQt5).")
    parser.add_argument("--dry-run-self-test", action="store_true")
    args = parser.parse_args()
    if args.dry_run_self_test:
        return dry_run_self_test()
    try:
        from PyQt5 import QtWidgets
    except Exception as exc:  # noqa: BLE001
        raise SystemExit(
            "PyQt5 is unavailable (%s); install python3-pyqt5 or run from the VM desktop." % exc
        )
    app = QtWidgets.QApplication(sys.argv)
    panel = ControlPanel(app)
    panel.win.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())



