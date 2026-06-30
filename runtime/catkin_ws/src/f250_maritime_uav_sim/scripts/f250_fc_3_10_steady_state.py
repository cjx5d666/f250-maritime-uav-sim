#!/usr/bin/env python3
import argparse
import csv
import json
import math
import os
import re
import sys
import time
from datetime import datetime


def resolve_project_root():
    env_root = os.environ.get("F250_PROJECT_ROOT")
    if env_root:
        return os.path.abspath(os.path.expanduser(env_root))
    script_dir = os.path.dirname(os.path.abspath(__file__))
    for rel in ("../../../..", "../../../../.."):
        candidate = os.path.abspath(os.path.join(script_dir, rel))
        if os.path.isdir(os.path.join(candidate, "catkin_ws", "src", "f250_maritime_uav_sim")):
            return candidate
    return os.getcwd()


def env_or_project_path(env_name, *parts):
    value = os.environ.get(env_name)
    if value:
        return os.path.abspath(os.path.expanduser(value))
    return os.path.join(PROJECT_ROOT, *parts)


def first_existing_or_first(*paths):
    for path in paths:
        if os.path.exists(path):
            return path
    return paths[0]


def authority_input_path(authority, *names):
    return first_existing_or_first(*[os.path.join(authority, name) for name in names])


PROJECT_ROOT = resolve_project_root()
DEFAULT_MAP_AUTHORITY = (
    os.path.abspath(os.path.expanduser(os.environ["MAP_AUTHORITY"]))
    if os.environ.get("MAP_AUTHORITY") else
    os.path.join(
        PROJECT_ROOT,
        "map_authority",
        "p0p8_clean_scene",
    )
)
DEFAULT_HOVER = (55.0, 16.0, 10.0, 0.469929)
SCHEMA = "f250_fc_3_10_steady_state_fc_only_velocity_setpoint_v4"
AUTHORITY_INPUT_ALIASES = {
    "route_waypoints_csv": (
        "sources/route_waypoints.csv",
        "route_waypoints.csv",
    ),
    "planner_obstacles_csv": (
        "sources/planner_obstacles.csv",
        "planner_obstacles.csv",
    ),
    "visual_mesh_footprints_csv": (
        "sources/visual_mesh_footprints.csv",
        "visual_mesh_footprints.csv",
    ),
    "map_manifest_json": (
        "sources/map_manifest.json",
        "map_manifest.json",
    ),
}
FC_COMPONENTS = [
    ("E_pos", "position"),
    ("E_vel", "velocity"),
    ("E_yaw", "yaw"),
]


def now_iso():
    return datetime.now().astimezone().isoformat(timespec="seconds")


def wrap_angle_rad(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def parse_hover_target(text):
    if not text:
        return DEFAULT_HOVER
    parts = [p.strip() for p in text.split(",")]
    if len(parts) != 4:
        raise argparse.ArgumentTypeError(
            "--hover-target must be x,y,z,yaw, got %r" % text)
    try:
        return tuple(float(p) for p in parts)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc))


def parse_float_list(text):
    if isinstance(text, (list, tuple)):
        values = [float(item) for item in text]
    else:
        values = [float(item.strip()) for item in str(text).split(",") if item.strip()]
    if not values:
        raise argparse.ArgumentTypeError("expected one or more comma-separated floats")
    if any(value <= 0.0 for value in values):
        raise argparse.ArgumentTypeError("all values must be positive")
    return values


def read_status_env(path):
    values = {}
    if not path or not os.path.exists(path):
        return values
    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, value = line.split("=", 1)
            values[key] = value
    return values


def read_csv_rows(path):
    with open(path, newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def read_json(path):
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def fnum(row, key, default=0.0):
    value = row.get(key, "")
    if value in ("", None):
        return default
    return float(value)


def bval(row, key, default=False):
    value = str(row.get(key, "")).strip().lower()
    if value in ("true", "1", "yes", "y"):
        return True
    if value in ("false", "0", "no", "n"):
        return False
    return default


def csv_row_float(row, key, default=None):
    try:
        value = row.get(key, "")
    except AttributeError:
        return default
    if value in ("", None):
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def first_waypoint_from_csv(path):
    if not path:
        return None, "not_configured"
    route_path = os.path.abspath(os.path.expanduser(path))
    if not os.path.exists(route_path):
        return None, "missing"
    try:
        rows = read_csv_rows(route_path)
    except Exception as exc:
        return None, "read_error:%s" % exc
    for row in rows:
        if csv_row_float(row, "x") is None or csv_row_float(row, "y") is None:
            continue
        return row, "first_waypoint"
    return None, "no_valid_waypoint"


def vec_add(a, b):
    return [float(a[0]) + float(b[0]), float(a[1]) + float(b[1])]


def vec_sub(a, b):
    return [float(a[0]) - float(b[0]), float(a[1]) - float(b[1])]


def vec_scale(a, s):
    return [float(a[0]) * float(s), float(a[1]) * float(s)]


def vec_norm(a):
    return math.hypot(float(a[0]), float(a[1]))


def vec3_norm(a):
    return math.sqrt(
        float(a[0]) * float(a[0])
        + float(a[1]) * float(a[1])
        + float(a[2]) * float(a[2])
    )


def vec3_sub(a, b):
    return [
        float(a[0]) - float(b[0]),
        float(a[1]) - float(b[1]),
        float(a[2]) - float(b[2]),
    ]


def normalize(a):
    n = vec_norm(a)
    if n <= 1e-12:
        raise ValueError("cannot normalize zero vector")
    return [float(a[0]) / n, float(a[1]) / n]


def rotate_xy(v, angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return [c * float(v[0]) - s * float(v[1]), s * float(v[0]) + c * float(v[1])]


def mean(values):
    values = list(values)
    if not values:
        return None
    return sum(values) / float(len(values))


def stddev(values):
    values = list(values)
    if len(values) < 2:
        return 0.0
    m = mean(values)
    return math.sqrt(sum((value - m) ** 2 for value in values) / float(len(values)))


def linear_slope(times, values):
    times = list(times)
    values = list(values)
    if len(times) < 2 or len(values) < 2:
        return 0.0
    t0 = times[0]
    xs = [float(t) - t0 for t in times]
    ys = [float(value) for value in values]
    x_mean = mean(xs)
    y_mean = mean(ys)
    denom = sum((x - x_mean) ** 2 for x in xs)
    if denom <= 1e-12:
        return 0.0
    return sum((x - x_mean) * (y - y_mean) for x, y in zip(xs, ys)) / denom


def unwrap_angles(angles):
    angles = [float(angle) for angle in angles]
    if not angles:
        return []
    unwrapped = [angles[0]]
    previous = angles[0]
    offset = 0.0
    for angle in angles[1:]:
        delta = angle - previous
        if delta > math.pi:
            offset -= 2.0 * math.pi
        elif delta < -math.pi:
            offset += 2.0 * math.pi
        unwrapped.append(angle + offset)
        previous = angle
    return unwrapped


def max_present(values):
    present = [v for v in values if v is not None]
    if not present:
        return None
    return max(present)


def mean_present(values):
    present = [float(v) for v in values if v is not None]
    if not present:
        return None
    return sum(present) / len(present)


def color_text(text, code):
    if not sys.stdout.isatty() or os.environ.get("NO_COLOR"):
        return text
    return "\033[%sm%s\033[0m" % (code, text)


def color_major(text):
    return color_text(text, "1;36")


def color_minor(text):
    return color_text(text, "1;33")


def color_result(result):
    text = str(result)
    if text.upper() == "PASS":
        return color_text(text, "1;32")
    if text.upper() == "FAIL":
        return color_text(text, "1;31")
    return text


ANSI_RE = re.compile(r"\033\[[0-9;]*m")


def strip_ansi(text):
    return ANSI_RE.sub("", text)


def fmt_display_percent(value):
    if value is None:
        return "--"
    return "%.3f%%" % float(value)


def fmt_float(value, unit="", digits=3):
    if value in ("", None):
        return "--"
    text = ("%.*f" % (digits, float(value)))
    if unit:
        text += " " + unit
    return text


def fmt_seconds(value):
    if value in ("", None):
        return "--"
    return "%.3fs" % float(value)


def steady_window_text(result):
    start = result.get("eval_start_sec")
    end = result.get("eval_end_sec")
    if start in ("", None) or end in ("", None):
        return "稳态计算窗口: --"
    return "稳态计算窗口: %s ~ %s" % (fmt_seconds(start), fmt_seconds(end))


def fmt_xyz(values):
    if not isinstance(values, (list, tuple)) or len(values) < 3:
        return "(--, --, --)"
    return "(%.3f, %.3f, %.3f)" % (float(values[0]), float(values[1]), float(values[2]))


def fmt_deg(value):
    if value in ("", None):
        return "-- deg"
    return "%.3f deg" % float(value)


def pass_fail_percent(value, threshold=5.0):
    return (
        "PASS"
        if value is not None and float(value) <= float(threshold)
        else "FAIL"
    )


def safe_list(value):
    return value if isinstance(value, list) else []


def settled_component_results(phase_results, component):
    return [
        result for result in phase_results
        if result.get("evaluated_component") == component
        and result.get("settled") is True
        and result.get("error_percent") not in ("", None)
    ]


def phase_error_detail(phase, result):
    denominator = result.get("denominator")
    error = result.get("error_percent")
    if denominator in ("", None) or error in ("", None):
        return ""
    numerator = abs(float(error)) * float(denominator) / 100.0
    if result.get("evaluated_component") == "E_vel" or phase.get("kind") == "velocity":
        actual = (result.get("steady_actual") or {}).get("mean_parallel_mps")
        target = ((result.get("desired") or {}).get("target_speed_mps")
                  or result.get("velocity_speed_mps")
                  or denominator)
        if actual not in ("", None):
            numerator = abs(float(actual) - float(target))
        return (
            "e_vel 速度误差: %s = 分子(平均平行速度偏差) %s / 分母(目标速度) %s * 100"
            % (
                fmt_display_percent(error),
                fmt_float(numerator, "m/s"),
                fmt_float(denominator, "m/s"),
            )
        )
    if result.get("evaluated_component") == "E_pos" or phase.get("kind") == "position":
        return (
            "e_pos 位置误差: %s = 分子(平均位置偏差) %s / 分母(位置分母) %s * 100"
            % (
                fmt_display_percent(error),
                fmt_float(numerator, "m"),
                fmt_float(denominator, "m"),
            )
        )
    if result.get("evaluated_component") == "E_yaw" or phase.get("kind") == "yaw":
        return (
            "e_att 偏航误差: %s = 分子(平均偏航偏差) %s / 分母(偏航分母) %s * 100"
            % (
                fmt_display_percent(error),
                fmt_float(math.degrees(numerator), "deg"),
                fmt_float(math.degrees(float(denominator)), "deg"),
            )
        )
    return (
        "稳态误差: %s = 分子(误差量) %s / 分母(评价分母) %s * 100"
        % (fmt_display_percent(error), fmt_float(numerator), fmt_float(denominator))
    )


def phase_error_formula_line(result):
    denominator = result.get("denominator")
    error = result.get("error_percent")
    if denominator in ("", None) or error in ("", None):
        return "稳态误差: --"
    component = result.get("evaluated_component")
    numerator = abs(float(error)) * float(denominator) / 100.0
    if component == "E_vel":
        actual = (result.get("steady_actual") or {}).get("mean_parallel_mps")
        target = ((result.get("desired") or {}).get("target_speed_mps")
                  or result.get("velocity_speed_mps")
                  or denominator)
        if actual not in ("", None):
            numerator = abs(float(actual) - float(target))
        return "稳态误差: %s / %s * 100%% = %s" % (
            fmt_float(numerator, "m/s"),
            fmt_float(denominator, "m/s"),
            fmt_display_percent(error),
        )
    if component == "E_yaw":
        return "稳态误差: %s / %s * 100%% = %s" % (
            fmt_float(math.degrees(numerator), "deg"),
            fmt_float(math.degrees(float(denominator)), "deg"),
            fmt_display_percent(error),
        )
    unit = "m" if component == "E_pos" else ""
    return "稳态误差: %s / %s * 100%% = %s" % (
        fmt_float(numerator, unit),
        fmt_float(denominator, unit),
        fmt_display_percent(error),
    )


def phase_error_value(result):
    value = result.get("error_percent")
    if value in ("", None):
        return "--"
    return fmt_display_percent(value)


def component_average_detail(component, phase_results, value=None):
    rows = settled_component_results(phase_results, component)
    if value is None:
        value = mean_present([row.get("error_percent") for row in rows])
    names = {
        "E_vel": "速度稳态误差平均值",
        "E_pos": "位置稳态误差平均值",
        "E_yaw": "偏航稳态误差平均值",
    }
    metric_name = names.get(component, "稳态误差平均值")
    return "%s: %s" % (metric_name, fmt_display_percent(value))


def component_average_detail_from_errors(component, errors):
    keys = {
        "E_vel": ("E_vel_selected", "E_vel_percent_sum", "E_vel_settled_count", "速度稳态误差平均值", ""),
        "E_pos": ("E_pos", "E_pos_percent_sum", "E_pos_settled_count", "位置稳态误差平均值", ""),
        "E_yaw": ("E_yaw", "E_yaw_percent_sum", "E_yaw_settled_count", "偏航稳态误差平均值", ""),
    }
    value_key, _sum_key, _count_key, metric_name, _numerator_name = keys[component]
    value = errors.get(value_key)
    return "%s: %s" % (metric_name, fmt_display_percent(value))


def final_metric_detail(errors):
    e_vel = errors.get("E_vel_selected")
    e_pos = errors.get("E_pos")
    e_yaw = errors.get("E_yaw")
    final_value = errors.get("E3.10_selected")
    component_rows = [
        ("E_vel", "速度", e_vel, errors.get("E_vel_percent_sum"), errors.get("E_vel_settled_count")),
        ("E_pos", "位置", e_pos, errors.get("E_pos_percent_sum"), errors.get("E_pos_settled_count")),
        ("E_att", "偏航", e_yaw, errors.get("E_yaw_percent_sum"), errors.get("E_yaw_settled_count")),
    ]
    present = [row for row in component_rows if row[2] is not None]
    if present:
        name, label, _value, total, count = max(present, key=lambda row: float(row[2]))
        if total in ("", None) and count not in ("", None):
            total = float(_value) * float(count)
        return "3.10 控制稳定误差: max(%s, %s, %s) = %s" % (
            fmt_display_percent(e_vel),
            fmt_display_percent(e_pos),
            fmt_display_percent(e_yaw),
            fmt_display_percent(final_value),
        )
    return (
        "E3.10 控制稳定误差: %s = 分子(三项最大误差) %s / 分母(通过阈值) 5.000%%"
        % (
            fmt_display_percent(final_value),
            fmt_display_percent(final_value),
        )
    )


def phase_result_display_line(phase, result):
    return phase_error_formula_line(result)


def velocity_steady_text(result):
    actual = (result.get("steady_actual") or {}).get("mean_parallel_mps")
    return fmt_float(actual, "m/s")


def position_target_text_from_result(result):
    desired = result.get("desired")
    if isinstance(desired, list) and len(desired) >= 3:
        return fmt_xyz(desired)
    if isinstance(desired, list) and len(desired) >= 2:
        return "(%.3f, %.3f, %.3f)" % (float(desired[0]), float(desired[1]), 10.0)
    current = result.get("current_target_xyz")
    return fmt_xyz(current)


def position_steady_text(result):
    actual = result.get("steady_actual")
    if isinstance(actual, list) and len(actual) >= 3:
        return fmt_xyz(actual)
    if isinstance(actual, list) and len(actual) >= 2:
        return "(%.3f, %.3f, --)" % (float(actual[0]), float(actual[1]))
    return fmt_xyz(actual)


def fmt_yaw_rad(value):
    if value in ("", None):
        return "-- deg"
    return fmt_deg(math.degrees(wrap_angle_rad(float(value))))


def yaw_target_text_from_result(result):
    desired = result.get("desired")
    if desired not in ("", None):
        return fmt_yaw_rad(desired)
    target = result.get("target_yaw")
    return fmt_yaw_rad(target)


def yaw_steady_text(result):
    actual = result.get("steady_actual") or {}
    if isinstance(actual, dict):
        if actual.get("mean_absolute_yaw_rad") not in ("", None):
            return fmt_yaw_rad(actual.get("mean_absolute_yaw_rad"))
        err = actual.get("mean_wrapped_yaw_error_rad")
        desired = result.get("desired")
        if err not in ("", None) and desired not in ("", None):
            return fmt_yaw_rad(float(desired) + float(err))
    return "-- deg"


def target_text_from_result(result):
    component = result.get("evaluated_component")
    repeat = int(result.get("repeat") or 0)
    phase = result.get("phase", "")
    if component == "E_vel":
        return "A -> B" if repeat % 2 == 1 else "B -> A"
    if component == "E_pos":
        return position_target_text_from_result(result)
    if component == "E_yaw":
        return yaw_target_text_from_result(result)
    return phase


def phase_from_result(result):
    component = result.get("evaluated_component")
    kind = {"E_vel": "velocity", "E_pos": "position", "E_yaw": "yaw"}.get(component, "")
    phase = {"kind": kind, "repeat": result.get("repeat") or 0}
    if kind == "position":
        desired = safe_list(result.get("desired"))
        if len(desired) >= 2:
            phase["target_xy"] = [float(desired[0]), float(desired[1])]
    return phase


def enrich_replay_summary_from_samples(summary, source_path=""):
    if not source_path:
        return
    sample_path = os.path.join(os.path.dirname(os.path.abspath(source_path)), "fc_3_10_samples.csv")
    if not os.path.exists(sample_path):
        return
    try:
        rows = read_csv_rows(sample_path)
    except Exception:
        return
    by_phase = {}
    for row in rows:
        by_phase.setdefault(row.get("phase", ""), []).append(row)
    for result in summary.get("phase_results", []):
        if result.get("evaluated_component") != "E_pos":
            continue
        actual = result.get("steady_actual")
        if isinstance(actual, list) and len(actual) >= 3:
            continue
        eval_start = result.get("eval_start_sec")
        eval_end = result.get("eval_end_sec")
        if eval_start in ("", None) or eval_end in ("", None):
            continue
        phase_rows = []
        for row in by_phase.get(result.get("phase", ""), []):
            try:
                t_sec = float(row.get("t_sec", ""))
            except ValueError:
                continue
            if float(eval_start) <= t_sec <= float(eval_end):
                phase_rows.append(row)
        xs = [csv_row_float(row, "actual_x") for row in phase_rows]
        ys = [csv_row_float(row, "actual_y") for row in phase_rows]
        zs = [csv_row_float(row, "actual_z") for row in phase_rows]
        if not xs or any(value is None for value in xs + ys + zs):
            continue
        result["steady_actual"] = [mean(xs), mean(ys), mean(zs)]
        desired = result.get("desired")
        if isinstance(desired, list) and len(desired) == 2:
            target_z = csv_row_float(phase_rows[0], "desired_z", 10.0)
            result["desired"] = [float(desired[0]), float(desired[1]), float(target_z)]


def terminal_summary_from_document(summary, source_path=""):
    enrich_replay_summary_from_samples(summary, source_path)
    errors = summary.get("errors_percent") or {}
    phase_results = summary.get("phase_results") or []
    result = summary.get("result") or metric_result(errors)
    lines = [
        color_major("========== F250 FC 3.10 Metrics =========="),
        "任务状态: 生成最终结果",
        "测试内容: 3.10 控制稳定误差",
        "测试指标: 速度 / 位置 / 偏航",
        "每项测试: 10 个测试窗口",
        "",
        "速度稳态误差",
        "= 实际稳态速度与期望速度的偏差 / 期望速度 * 100%",
        "位置稳态误差",
        "= 实际稳态位置与期望位置的三维偏差",
        "  / 规划位置目标间三维距离 * 100%",
        "偏航稳态误差",
        "= 实际稳态绝对 yaw 与期望绝对 yaw 的偏差 / yaw 测试分母 * 100%",
        "",
    ]

    for component, title in [
        ("E_vel", "速度控制阶段结果"),
        ("E_pos", "位置控制阶段结果"),
        ("E_yaw", "偏航控制阶段结果"),
    ]:
        rows = [
            row for row in phase_results
            if row.get("evaluated_component") == component
        ]
        if not rows:
            continue
        for row in rows:
            label = {
                "E_vel": "速度测试",
                "E_pos": "位置测试",
                "E_yaw": "偏航测试",
            }.get(component, "测试")
            phase = phase_from_result(row)
            if row.get("settled") is True:
                lines.append(color_minor("[%s %s]" % (label, phase_round_text(phase))))
                if component == "E_vel":
                    target_speed = ((row.get("desired") or {}).get("target_speed_mps")
                                    or row.get("velocity_speed_mps")
                                    or row.get("denominator"))
                    lines.append("期望速度: %s  %s" % (
                        target_text_from_result(row),
                        fmt_float(target_speed, "m/s"),
                    ))
                    lines.append("实际稳态速度: %s" % velocity_steady_text(row))
                elif component == "E_pos":
                    lines.append("期望位置: %s" % position_target_text_from_result(row))
                    lines.append("实际稳态位置: %s" % position_steady_text(row))
                elif component == "E_yaw":
                    lines.append("期望 yaw: %s" % yaw_target_text_from_result(row))
                    lines.append("实际稳态 yaw: %s" % yaw_steady_text(row))
                lines.append(steady_window_text(row))
                lines.append(phase_result_display_line(phase, row))
                lines.append("")
            else:
                lines.append(color_minor("[%s %s]" % (label, phase_round_text(phase))))
                lines.append("稳态误差: 未达到")
                lines.append("原因: %s" % (row.get("not_settled_reason") or "unknown"))
                lines.append("")
        lines.append(color_minor("[%s]" % title))
        lines.append(component_average_detail(component, phase_results))
        lines.append("")

    final_value = errors.get("E3.10_selected")
    passed = (
        final_value is not None
        and float(final_value) <= 5.0
        and bool(errors.get("all_metric_windows_settled", False))
    )
    result = "PASS" if passed else "FAIL"
    lines.extend([
        "===== FINAL =====",
        "3.10 速度稳态误差平均值: %s" % fmt_display_percent(errors.get("E_vel_selected")),
        "3.10 位置稳态误差平均值: %s" % fmt_display_percent(errors.get("E_pos")),
        "3.10 偏航稳态误差平均值: %s" % fmt_display_percent(errors.get("E_yaw")),
        final_metric_detail(errors),
        "",
        "控制结果: %s" % color_result(result),
    ])
    return "\n".join(lines)


def velocity_key(speed_mps):
    return ("%gmps" % float(speed_mps)).replace(".", "p")


def distance_point_segment(point, start, end):
    px, py = float(point[0]), float(point[1])
    ax, ay = float(start[0]), float(start[1])
    bx, by = float(end[0]), float(end[1])
    dx = bx - ax
    dy = by - ay
    denom = dx * dx + dy * dy
    if denom <= 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / denom))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def point_to_obb_distance(point, center, size, yaw):
    dx = float(point[0]) - float(center[0])
    dy = float(point[1]) - float(center[1])
    c = math.cos(float(yaw))
    s = math.sin(float(yaw))
    local_x = c * dx + s * dy
    local_y = -s * dx + c * dy
    half_x = float(size[0]) / 2.0
    half_y = float(size[1]) / 2.0
    outside_x = max(abs(local_x) - half_x, 0.0)
    outside_y = max(abs(local_y) - half_y, 0.0)
    outside = math.hypot(outside_x, outside_y)
    if outside > 0.0:
        return outside
    return -min(half_x - abs(local_x), half_y - abs(local_y))


def obb_corners(center, size, yaw):
    half_x = float(size[0]) / 2.0
    half_y = float(size[1]) / 2.0
    local = [(-half_x, -half_y), (half_x, -half_y), (half_x, half_y), (-half_x, half_y)]
    c = math.cos(float(yaw))
    s = math.sin(float(yaw))
    corners = []
    for x, y in local:
        corners.append([
            float(center[0]) + c * x - s * y,
            float(center[1]) + s * x + c * y,
        ])
    return corners


def orientation(a, b, c):
    return (float(b[1]) - float(a[1])) * (float(c[0]) - float(b[0])) - (
        float(b[0]) - float(a[0])) * (float(c[1]) - float(b[1]))


def on_segment(a, b, c):
    return (
        min(float(a[0]), float(c[0])) - 1e-9 <= float(b[0]) <= max(float(a[0]), float(c[0])) + 1e-9
        and min(float(a[1]), float(c[1])) - 1e-9 <= float(b[1]) <= max(float(a[1]), float(c[1])) + 1e-9
    )


def segments_intersect(p1, q1, p2, q2):
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)
    if ((o1 > 0 and o2 < 0) or (o1 < 0 and o2 > 0)) and (
            (o3 > 0 and o4 < 0) or (o3 < 0 and o4 > 0)):
        return True
    if abs(o1) <= 1e-9 and on_segment(p1, p2, q1):
        return True
    if abs(o2) <= 1e-9 and on_segment(p1, q2, q1):
        return True
    if abs(o3) <= 1e-9 and on_segment(p2, p1, q2):
        return True
    if abs(o4) <= 1e-9 and on_segment(p2, q1, q2):
        return True
    return False


def point_in_polygon(point, polygon):
    x = float(point[0])
    y = float(point[1])
    inside = False
    j = len(polygon) - 1
    for i in range(len(polygon)):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        if ((yi > y) != (yj > y)) and (
                x < (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi):
            inside = not inside
        j = i
    return inside


def segment_polygon_distance(start, end, polygon):
    if point_in_polygon(start, polygon) or point_in_polygon(end, polygon):
        return 0.0
    distances = []
    for i in range(len(polygon)):
        a = polygon[i]
        b = polygon[(i + 1) % len(polygon)]
        if segments_intersect(start, end, a, b):
            return 0.0
        distances.append(distance_point_segment(start, a, b))
        distances.append(distance_point_segment(end, a, b))
        distances.append(distance_point_segment(a, start, end))
        distances.append(distance_point_segment(b, start, end))
    return min(distances) if distances else 0.0


def obstacle_variants(obstacle):
    variants = [dict(obstacle)]
    amplitude = float(obstacle.get("motion_amplitude_m") or 0.0)
    axis = obstacle.get("motion_axis_xy") or [0.0, 0.0]
    axis_norm = vec_norm(axis)
    if obstacle.get("motion_type") and amplitude > 0.0 and axis_norm > 1e-12:
        unit = normalize(axis)
        variants = []
        for scale in (-1.0, 0.0, 1.0):
            item = dict(obstacle)
            item["center"] = vec_add(obstacle["center"], vec_scale(unit, amplitude * scale))
            item["motion_sample_scale"] = scale
            variants.append(item)
    return variants


def point_clearance_to_obstacle(point, obstacle):
    clearances = []
    for item in obstacle_variants(obstacle):
        if item["shape"] == "cylinder":
            clearances.append(vec_norm(vec_sub(point, item["center"])) - float(item["radius_m"]))
        elif item["shape"] == "box":
            clearances.append(point_to_obb_distance(point, item["center"], item["size"], item["yaw_rad"]))
    return min(clearances)


def segment_clearance_to_obstacle(start, end, obstacle):
    clearances = []
    for item in obstacle_variants(obstacle):
        if item["shape"] == "cylinder":
            clearances.append(distance_point_segment(item["center"], start, end) - float(item["radius_m"]))
        elif item["shape"] == "box":
            corners = obb_corners(item["center"], item["size"], item["yaw_rad"])
            clearances.append(segment_polygon_distance(start, end, corners))
    return min(clearances)


def obstacle_from_row(row):
    shape = (row.get("shape") or "").strip().lower()
    if not shape:
        shape = "cylinder" if row.get("radius_m") else "box"
    obstacle = {
        "type": row.get("type", ""),
        "name": row.get("name", ""),
        "include_in_cloud": bval(row, "include_in_cloud", True),
        "visual": bval(row, "visual", False),
        "center": [fnum(row, "center_x"), fnum(row, "center_y")],
        "center_z": fnum(row, "center_z"),
        "shape": shape,
        "yaw_rad": fnum(row, "yaw_rad"),
        "motion_type": row.get("motion_type", ""),
        "motion_axis_xy": [fnum(row, "motion_axis_x"), fnum(row, "motion_axis_y")],
        "motion_amplitude_m": fnum(row, "motion_amplitude_m"),
        "raw": row,
    }
    if shape == "cylinder":
        obstacle["radius_m"] = fnum(row, "radius_m")
        obstacle["height_m"] = fnum(row, "height_m")
    else:
        obstacle["size"] = [fnum(row, "size_x"), fnum(row, "size_y")]
        obstacle["size_z"] = fnum(row, "size_z")
    return obstacle


def clearance_audit(geometry_items, obstacles):
    per_obstacle = []
    global_best = None
    global_item = None
    global_obstacle = None
    for obstacle in obstacles:
        best = None
        best_item = None
        for item in geometry_items:
            if item["kind"] == "point":
                clearance = point_clearance_to_obstacle(item["xy"], obstacle)
            else:
                clearance = segment_clearance_to_obstacle(item["start_xy"], item["end_xy"], obstacle)
            if best is None or clearance < best:
                best = clearance
                best_item = item["name"]
            if global_best is None or clearance < global_best:
                global_best = clearance
                global_item = item["name"]
                global_obstacle = obstacle["name"]
        per_obstacle.append({
            "name": obstacle["name"],
            "type": obstacle["type"],
            "shape": obstacle["shape"],
            "min_clearance_m": best,
            "closest_test_item": best_item,
        })
    conclusion = "SAFE" if global_best is not None and global_best > 0.0 else "UNSAFE_OR_UNKNOWN"
    return {
        "conclusion": conclusion,
        "min_clearance_m": global_best,
        "closest_test_item": global_item,
        "closest_planner_obstacle": global_obstacle,
        "per_obstacle": per_obstacle,
    }


def velocity_specs_from_args(args):
    speeds = list(getattr(args, "velocity_speeds_mps", None) or [2.0])
    lengths = list(getattr(args, "velocity_lengths_m", None) or [])
    if not lengths:
        default_lengths = {2.0: 60.0}
        lengths = [default_lengths.get(round(float(speed), 6), max(60.0, float(speed) * 22.5)) for speed in speeds]
    if len(lengths) != len(speeds):
        raise SystemExit("--velocity-lengths-m count must match --velocity-speeds-mps count")
    specs = []
    seen = set()
    for speed, length in zip(speeds, lengths):
        speed = float(speed)
        length = float(length)
        key = velocity_key(speed)
        if key in seen:
            raise SystemExit("duplicate velocity speed after normalization: %s" % speed)
        seen.add(key)
        specs.append({"speed_mps": speed, "requested_L_m": length, "key": key})
    return specs


def minimum_velocity_length(speed_mps, args):
    settle = float(args.velocity_settle_search_start_sec)
    stationarity = float(args.stationarity_window_sec)
    eval_window = float(args.eval_window_sec)
    endpoint_margin = float(args.velocity_endpoint_margin_m)
    return float(speed_mps) * (settle + stationarity + eval_window) + endpoint_margin


def velocity_geometry_items_for(a_xy, b_xy, key):
    return [
        {"name": "B_velocity_endpoint_%s" % key, "kind": "point", "xy": b_xy},
        {
            "name": "velocity_%s_A_B_segment" % key,
            "kind": "segment",
            "start_xy": a_xy,
            "end_xy": b_xy,
        },
    ]


def select_velocity_length(a_xy, safe_u, spec, obstacles, args):
    requested_original = float(spec["requested_L_m"])
    requested = requested_original
    minimum = minimum_velocity_length(float(spec["speed_mps"]), args)
    if requested < minimum:
        requested = minimum
    candidates = [requested]
    decrement = float(args.velocity_length_adjust_step_m)
    current = requested - decrement
    while current >= minimum - 1e-9:
        candidates.append(current)
        current -= decrement
    best_failed = None
    for length in candidates:
        b_xy = vec_add(a_xy, vec_scale(safe_u, length))
        audit = clearance_audit(velocity_geometry_items_for(a_xy, b_xy, spec["key"]), obstacles)
        min_clearance = audit.get("min_clearance_m")
        safe = audit.get("conclusion") == "SAFE" and min_clearance is not None and (
            float(min_clearance) >= float(args.velocity_min_clearance_m)
        )
        if safe:
            return length, b_xy, audit, abs(length - requested_original) > 1e-9
        best_failed = audit
    length = requested
    b_xy = vec_add(a_xy, vec_scale(safe_u, length))
    return length, b_xy, best_failed or clearance_audit(
        velocity_geometry_items_for(a_xy, b_xy, spec["key"]), obstacles), True


def select_visual_rows(visual_rows):
    ship_candidates = [
        row for row in visual_rows
        if "oasis" in row.get("name", "").lower()
        or "carrier" in row.get("name", "").lower()
        or row.get("style", "").lower() == "oasis"
    ]
    if not ship_candidates:
        ship_candidates = [
            row for row in visual_rows
            if "ship" in row.get("name", "").lower() or "vessel" in row.get("name", "").lower()
        ]
    if not ship_candidates:
        raise SystemExit("could not identify ship center from visual_mesh_footprints.csv")
    ship = ship_candidates[0]

    island_candidates = [
        row for row in visual_rows
        if row.get("style", "").lower() == "island"
        or "island" in row.get("name", "").lower()
        or "mountain" in row.get("name", "").lower()
    ]
    kauai = [row for row in island_candidates if "kauai" in row.get("name", "").lower()]
    if len(kauai) >= 3:
        island_candidates = kauai[:3]
    if len(island_candidates) < 3:
        raise SystemExit("could not identify the three mountain/island centers from visual_mesh_footprints.csv")
    return ship, island_candidates[:3]


def build_geometry(args):
    authority = os.path.abspath(args.map_authority)
    authority_route_path = authority_input_path(authority, *AUTHORITY_INPUT_ALIASES["route_waypoints_csv"])
    route_path = authority_route_path
    planner_path = authority_input_path(authority, *AUTHORITY_INPUT_ALIASES["planner_obstacles_csv"])
    visual_path = authority_input_path(authority, *AUTHORITY_INPUT_ALIASES["visual_mesh_footprints_csv"])
    manifest_path = authority_input_path(authority, *AUTHORITY_INPUT_ALIASES["map_manifest_json"])
    for path in (authority_route_path, planner_path, visual_path, manifest_path):
        if not os.path.exists(path):
            raise SystemExit("missing authoritative geometry input: %s" % path)

    manifest = read_json(manifest_path)
    authority_route_rows = read_csv_rows(authority_route_path)
    planner_rows = read_csv_rows(planner_path)
    visual_rows = read_csv_rows(visual_path)

    fallback_p0 = None
    for row in authority_route_rows:
        if row.get("label") == "P0" or row.get("index") == "0":
            fallback_p0 = row
            break
    if fallback_p0 is None:
        raise SystemExit("authoritative route_waypoints.csv does not contain P0")

    current_route_path = os.path.abspath(os.path.expanduser(args.route_waypoints_csv)) if args.route_waypoints_csv else ""
    route_start, route_start_reason = first_waypoint_from_csv(current_route_path)
    if route_start is None:
        route_start = fallback_p0
        route_path = authority_route_path
        route_start_source_kind = "map_authority_fallback_p0"
    else:
        route_path = current_route_path
        route_start_source_kind = "current_run_route_first_waypoint"

    ship_row, island_rows = select_visual_rows(visual_rows)
    ship_center = [fnum(ship_row, "center_x"), fnum(ship_row, "center_y")]
    island_centers = [[fnum(row, "center_x"), fnum(row, "center_y")] for row in island_rows]
    island_centroid = [
        mean(center[0] for center in island_centers),
        mean(center[1] for center in island_centers),
    ]
    ship_to_islands = vec_sub(island_centroid, ship_center)
    safe_u = vec_scale(normalize(ship_to_islands), -1.0)

    a_xy = [float(route_start["x"]), float(route_start["y"])]
    z = 10.0
    p0_yaw = csv_row_float(route_start, "yaw_rad")
    if p0_yaw is None:
        p0_yaw = csv_row_float(route_start, "yaw", 0.0)
    radius = float(args.position_radius_m)
    center_xy = vec_add(a_xy, vec_scale(safe_u, radius))
    obstacles = [obstacle_from_row(row) for row in planner_rows if bval(row, "include_in_cloud", True)]

    velocity_tests = []
    for spec in velocity_specs_from_args(args):
        length, b_xy, velocity_audit, adjusted = select_velocity_length(
            a_xy, safe_u, spec, obstacles, args)
        velocity_tests.append({
            "key": spec["key"],
            "A_xy": a_xy,
            "B_xy": b_xy,
            "z": z,
            "L_m": length,
            "requested_L_m": float(spec["requested_L_m"]),
            "speed_mps": float(spec["speed_mps"]),
            "windows": 10,
            "round_trips": 5,
            "leg_duration_sec": length / float(spec["speed_mps"]),
            "length_adjusted": adjusted,
            "minimum_length_for_settle_eval_m": minimum_velocity_length(float(spec["speed_mps"]), args),
            "endpoint_margin_m": float(args.velocity_endpoint_margin_m),
            "command_velocity_policy": "desired world XY velocity is +speed*u or -speed*u on AB legs",
            "planner_obstacle_clearance": velocity_audit,
        })

    start_vector = vec_scale(safe_u, -radius)
    decagon = []
    for index in range(10):
        xy = vec_add(center_xy, rotate_xy(start_vector, 2.0 * math.pi * (index + 1) / 10.0))
        decagon.append({"index": index, "x": xy[0], "y": xy[1], "z": z})
    position_step_denominators = []
    previous = {
        "label": "post_velocity_hold_at_A",
        "x": a_xy[0],
        "y": a_xy[1],
        "z": z,
        "source": "hold_after_velocity_at_A",
    }
    for point in decagon:
        current = {
            "label": "position_vertex_%02d" % int(point["index"]),
            "x": point["x"],
            "y": point["y"],
            "z": point["z"],
            "source": "position_target",
        }
        distance_3d = math.sqrt(
            (float(current["x"]) - float(previous["x"])) ** 2
            + (float(current["y"]) - float(previous["y"])) ** 2
            + (float(current["z"]) - float(previous["z"])) ** 2
        )
        position_step_denominators.append({
            "index": int(point["index"]) + 1,
            "phase": current["label"],
            "previous_label": previous["label"],
            "current_label": current["label"],
            "previous_xyz": [previous["x"], previous["y"], previous["z"]],
            "current_xyz": [current["x"], current["y"], current["z"]],
            "step_distance_3d_m": distance_3d,
            "usable_as_denominator": distance_3d > 1e-9,
        })
        previous = current
    position_denominator_alignment_status = (
        "blocked_zero_first_window"
        if position_step_denominators and not position_step_denominators[0]["usable_as_denominator"]
        else "ready"
    )

    yaw_offsets_deg = [90, 180, 270, 360, -90, -180, -270, -360, 180, 0]
    yaw_targets = []
    for index, offset_deg in enumerate(yaw_offsets_deg):
        denom_deg = 90 if index < 8 else 180
        raw_yaw = p0_yaw + math.radians(offset_deg)
        yaw_targets.append({
            "index": index + 1,
            "offset_deg": offset_deg,
            "target_yaw_rad": raw_yaw,
            "target_yaw_wrapped_rad": wrap_angle_rad(raw_yaw),
            "denominator_deg": denom_deg,
            "denominator_rad": math.radians(denom_deg),
        })

    geometry_items = [
        {"name": "A_route_start", "kind": "point", "xy": a_xy},
    ]
    for test in velocity_tests:
        geometry_items.extend(velocity_geometry_items_for(a_xy, test["B_xy"], test["key"]))
    for point in decagon:
        geometry_items.append({
            "name": "decagon_vertex_%02d" % point["index"],
            "kind": "point",
            "xy": [point["x"], point["y"]],
        })
    for index in range(10):
        start = decagon[index]
        end = decagon[(index + 1) % 10]
        geometry_items.append({
            "name": "decagon_edge_%02d_%02d" % (index, (index + 1) % 10),
            "kind": "segment",
            "start_xy": [start["x"], start["y"]],
            "end_xy": [end["x"], end["y"]],
        })

    audit = clearance_audit(geometry_items, obstacles)
    return {
        "schema": "f250_fc_3_10_geometry_audit_v1",
        "created_at": now_iso(),
        "authority_dir": authority,
        "source_files": {
            "route_waypoints_csv": route_path,
            "current_route_waypoints_csv": current_route_path,
            "map_authority_route_waypoints_csv": authority_route_path,
            "planner_obstacles_csv": planner_path,
            "visual_mesh_footprints_csv": visual_path,
            "map_manifest_json": manifest_path,
            "scene_yaml": manifest.get("scene_path", ""),
        },
        "source_policy": (
            "Geometry is derived from authoritative CSV/JSON/YAML-backed map package; "
            "FC start uses the current run route first waypoint when available, then falls back "
            "to authoritative P0. Legacy or cached PNG files are not used as coordinate truth."
        ),
        "route_start_source": {
            "label": route_start.get("label", "P0"),
            "name": route_start.get("name", ""),
            "x": a_xy[0],
            "y": a_xy[1],
            "z": z,
            "yaw_rad": p0_yaw,
            "z_policy": "fixed 10 m for FC Metric 3.10",
            "source_kind": route_start_source_kind,
            "source_file": route_path,
            "current_route_waypoints_csv": current_route_path,
            "fallback_source_file": authority_route_path,
            "fallback_reason": route_start_reason if route_start_source_kind == "map_authority_fallback_p0" else "",
        },
        "p0_source": {
            "label": route_start.get("label", "P0"),
            "name": route_start.get("name", ""),
            "x": a_xy[0],
            "y": a_xy[1],
            "z": z,
            "yaw_rad": p0_yaw,
            "z_policy": "fixed 10 m for FC Metric 3.10",
            "source_kind": route_start_source_kind,
            "source_file": route_path,
        },
        "ship_identification": {
            "policy": "name/style match in visual_mesh_footprints.csv",
            "name": ship_row.get("name", ""),
            "style": ship_row.get("style", ""),
            "center_xy": ship_center,
        },
        "mountain_island_identification": {
            "policy": "three kauai/island/mountain visual mesh centers in visual_mesh_footprints.csv",
            "centers": [
                {
                    "name": row.get("name", ""),
                    "style": row.get("style", ""),
                    "center_xy": [fnum(row, "center_x"), fnum(row, "center_y")],
                }
                for row in island_rows
            ],
            "centroid_xy": island_centroid,
        },
        "direction": {
            "ship_to_mountain_centroid_xy": ship_to_islands,
            "safe_u_xy": safe_u,
            "policy": "u is the normalized reverse of ship center to three-mountain/island centroid direction",
        },
        "velocity_test": {
            "note": "Legacy summary of the first velocity test; use velocity_tests for all formal speeds.",
            "A_xy": velocity_tests[0]["A_xy"],
            "B_xy": velocity_tests[0]["B_xy"],
            "z": z,
            "L_m": velocity_tests[0]["L_m"],
            "speed_mps": velocity_tests[0]["speed_mps"],
            "windows": 10,
            "round_trips": 5,
        },
        "velocity_tests": velocity_tests,
        "position_test": {
            "R_m": radius,
            "center_C_xy": center_xy,
            "decagon_points": decagon,
            "closure_point_xy": a_xy,
            "windows": 10,
            "denominator_m": radius,
            "current_formula_status": (
                "xyz_step_denominator_enabled"
                if position_denominator_alignment_status != "ready"
                else "ready_for_xyz_step_denominator"
            ),
            "current_legacy_denominator_m": radius,
            "outline_denominator_policy": "3D distance from previous position target to current position target",
            "outline_error_policy": "3D distance between mean evaluated actual XYZ and current target XYZ",
            "step_denominators": position_step_denominators,
            "denominator_alignment_status": position_denominator_alignment_status,
            "denominator_alignment_note": (
                "Position step denominator is not ready because at least one planned 3D target step is zero."
                if position_denominator_alignment_status != "ready"
                else "All current position windows have nonzero previous-target to current-target distances."
            ),
        },
        "yaw_test": {
            "base_yaw_rad": p0_yaw,
            "targets": yaw_targets,
            "windows": 10,
            "denominator_policy": "first 8 yaw windows use 90 deg denominator; last 2 use 180 deg denominator",
        },
        "planner_obstacle_clearance": audit,
        "planner_obstacle_count": len(obstacles),
        "audit_conclusion": (
            "SAFE: FC 3.10 A/B and decagon geometry do not intersect planner obstacles"
            if audit["conclusion"] == "SAFE"
            else "UNSAFE_OR_UNKNOWN: FC 3.10 geometry intersects or cannot be cleared against planner obstacles"
        ),
    }


def geometry_is_safe_for_formal(geometry, args):
    audit = geometry["planner_obstacle_clearance"]
    if audit.get("conclusion") != "SAFE":
        return False
    min_clearance = audit.get("min_clearance_m")
    if min_clearance is None or float(min_clearance) < float(args.velocity_min_clearance_m):
        return False
    for velocity_test in geometry.get("velocity_tests", []):
        vaudit = velocity_test.get("planner_obstacle_clearance", {})
        vclearance = vaudit.get("min_clearance_m")
        if vaudit.get("conclusion") != "SAFE":
            return False
        if vclearance is None or float(vclearance) < float(args.velocity_min_clearance_m):
            return False
    return True


def phase_duration_for_velocity(args):
    speeds = getattr(args, "velocity_speeds_mps", [2.0])
    lengths = getattr(args, "velocity_lengths_m", [90.0])
    return float(lengths[0]) / float(speeds[0])


def build_phases(args, geometry):
    p0 = geometry["route_start_source"]
    a_xy = [float(p0["x"]), float(p0["y"])]
    _z = p0["z"]  # P0 z coordinate (not used in phase definitions, kept for reference)
    yaw = p0["yaw_rad"]
    phases = [{
        "name": "p0_prehold",
        "kind": "hold",
        "duration": float(args.prehold_sec),
        "component": "",
        "repeat": 0,
        "target_xy": a_xy,
        "target_yaw": yaw,
    }]

    for velocity_test in geometry["velocity_tests"]:
        b_xy = velocity_test["B_xy"]
        speed = float(velocity_test["speed_mps"])
        command_speed = speed * float(args.velocity_command_gain)
        key = velocity_test["key"]
        leg_duration = float(velocity_test["leg_duration_sec"])
        for index in range(10):
            start = a_xy if index % 2 == 0 else b_xy
            end = b_xy if index % 2 == 0 else a_xy
            direction = normalize(vec_sub(end, start))
            desired_velocity = vec_scale(direction, command_speed)
            leg_yaw = math.atan2(direction[1], direction[0])
            prealign_settle_sec = max(0.0, float(args.velocity_prealign_sec))
            prealign_stable_sec = max(0.0, float(args.velocity_prealign_stable_sec))
            prealign_timeout_sec = max(0.1, float(args.velocity_prealign_timeout_sec))
            if prealign_settle_sec > 0.0:
                phases.append({
                    "name": "prealign_velocity_%s_window_%02d_%s" % (
                        key, index + 1, "A_to_B" if index % 2 == 0 else "B_to_A"),
                    "kind": "hold",
                    "duration": prealign_settle_sec + prealign_timeout_sec,
                    "component": "",
                    "repeat": index + 1,
                    "velocity_key": key,
                    "control_mode": "position",
                    "target_xy": start,
                    "target_yaw": leg_yaw,
                    "prealign_policy": "settle_then_continuous_yaw_stability_warning_only",
                    "prealign_settle_sec": prealign_settle_sec,
                    "prealign_stable_sec": prealign_stable_sec,
                    "prealign_timeout_sec": prealign_timeout_sec,
                    "prealign_tolerance_deg": float(args.velocity_prealign_tolerance_deg),
                })
            phases.append({
                "name": "velocity_%s_window_%02d_%s" % (
                    key, index + 1, "A_to_B" if index % 2 == 0 else "B_to_A"),
                "kind": "velocity",
                "duration": leg_duration,
                "component": "E_vel",
                "repeat": index + 1,
                "velocity_key": key,
                "velocity_speed_mps": speed,
                "velocity_command_speed_mps": command_speed,
                "velocity_command_gain": float(args.velocity_command_gain),
                "velocity_length_m": float(velocity_test["L_m"]),
                "control_mode": "velocity",
                "start_xy": start,
                "target_xy": end,
                "desired_velocity_xy": desired_velocity,
                "denominator": speed,
                "target_yaw": leg_yaw,
            })
            if index < 9 and float(args.velocity_interleg_hold_sec) > 0.0:
                phases.append({
                    "name": "hold_between_velocity_%s_window_%02d_at_%s" % (
                        key, index + 1, "B" if index % 2 == 0 else "A"),
                    "kind": "hold",
                    "duration": float(args.velocity_interleg_hold_sec),
                    "component": "",
                    "repeat": index + 1,
                    "control_mode": "position",
                    "target_xy": end,
                    "target_yaw": leg_yaw,
                })
            if index < 9 and float(args.velocity_reset_hold_sec) > 0.0:
                phases.append({
                    "name": "reset_between_velocity_%s_window_%02d_at_%s" % (
                        key, index + 1, "B" if index % 2 == 0 else "A"),
                    "kind": "hold",
                    "duration": float(args.velocity_reset_hold_sec),
                    "component": "",
                    "repeat": index + 1,
                    "control_mode": "position",
                    "target_xy": end,
                    "target_yaw": leg_yaw,
                })

        phases.append({
            "name": "hold_after_velocity_%s_at_A" % key,
            "kind": "hold",
            "duration": float(args.velocity_final_reset_hold_sec),
            "component": "",
            "repeat": 10,
            "control_mode": "position",
            "target_xy": a_xy,
            "target_yaw": yaw,
        })

    if getattr(args, "speed_only", False):
        phases.append({
            "name": "return_p0_hover",
            "kind": "hold",
            "duration": float(args.final_hold_sec),
            "component": "",
            "repeat": 10,
            "control_mode": "position",
            "target_xy": a_xy,
            "target_yaw": yaw,
        })
        return phases

    for point, denom in zip(
        geometry["position_test"]["decagon_points"],
        geometry["position_test"].get("step_denominators", []),
    ):
        phases.append({
            "name": "position_vertex_%02d" % int(point["index"]),
            "kind": "position",
            "duration": float(args.position_hold_sec),
            "component": "E_pos",
            "repeat": int(point["index"]) + 1,
            "control_mode": "position",
            "target_xy": [float(point["x"]), float(point["y"])],
            "target_z": float(point["z"]),
            "denominator": float(denom.get("step_distance_3d_m", args.position_radius_m)),
            "previous_target_xyz": list(denom.get("previous_xyz", [])),
            "current_target_xyz": list(denom.get("current_xyz", [point["x"], point["y"], point["z"]])),
            "target_yaw": yaw,
        })

    phases.append({
        "name": "return_p0_after_position",
        "kind": "hold",
        "duration": float(args.return_hold_sec),
        "component": "",
        "repeat": 10,
        "control_mode": "position",
        "target_xy": a_xy,
        "target_yaw": yaw,
    })

    for target in geometry["yaw_test"]["targets"]:
        phases.append({
            "name": "yaw_window_%02d_%+ddeg" % (int(target["index"]), int(target["offset_deg"])),
            "kind": "yaw",
            "duration": float(args.yaw_hold_sec),
            "component": "E_yaw",
            "repeat": int(target["index"]),
            "control_mode": "position",
            "target_xy": a_xy,
            "target_yaw": float(target["target_yaw_rad"]),
            "denominator": float(target["denominator_rad"]),
        })

    phases.append({
        "name": "return_p0_hover",
        "kind": "hold",
        "duration": float(args.final_hold_sec),
        "component": "",
        "repeat": 10,
        "control_mode": "position",
        "target_xy": a_xy,
        "target_yaw": yaw,
    })
    return phases


def planned_phase_records(phases):
    records = []
    start = 0.0
    for phase in phases:
        end = start + float(phase["duration"])
        records.append({"phase": phase, "start_sec": start, "end_sec": end})
        start = end
    return records


def reference_for_phase(phase, phase_elapsed, hover):
    z = float(hover[2])
    if phase.get("target_z") not in ("", None):
        z = float(phase.get("target_z"))
    yaw = float(phase.get("target_yaw", hover[3]))
    xy = phase.get("target_xy", [hover[0], hover[1]])
    position = [float(xy[0]), float(xy[1]), z]
    velocity = [0.0, 0.0, 0.0]
    acceleration = [0.0, 0.0, 0.0]
    if phase["kind"] == "velocity":
        start = phase["start_xy"]
        desired = phase["desired_velocity_xy"]
        position = [
            float(start[0]) + float(desired[0]) * max(0.0, phase_elapsed),
            float(start[1]) + float(desired[1]) * max(0.0, phase_elapsed),
            z,
        ]
        velocity = [float(desired[0]), float(desired[1]), 0.0]
    return {
        "position": position,
        "velocity": velocity,
        "acceleration": acceleration,
        "yaw": yaw,
        "yaw_dot": 0.0,
    }


def sample_from_actual(t_sec, phase, phase_elapsed, ref, actual):
    if actual is None:
        actual = {
            "position": [None, None, None],
            "velocity": [None, None, None],
            "yaw": None,
        }
    pos = actual["position"]
    vel = actual["velocity"]
    yaw = actual["yaw"]
    component = phase.get("component", "")
    denominator = phase.get("denominator", "")
    position_error_m = ""
    velocity_error_mps = ""
    velocity_parallel_mps = ""
    velocity_cross_mps = ""
    cross_track_error_m = ""
    along_track_distance_m = ""
    yaw_error_rad = ""
    e_pos = e_vel = e_yaw = None
    if phase["kind"] == "position" and pos[0] is not None and pos[1] is not None and pos[2] is not None:
        target_z = float(phase.get("target_z", ref["position"][2]))
        position_error_m = vec3_norm(vec3_sub([pos[0], pos[1], pos[2]], [phase["target_xy"][0], phase["target_xy"][1], target_z]))
        e_pos = position_error_m / float(denominator) * 100.0
    elif phase["kind"] == "velocity" and vel[0] is not None and vel[1] is not None:
        direction = normalize(phase["desired_velocity_xy"])
        cross = [-direction[1], direction[0]]
        actual_v = [float(vel[0]), float(vel[1])]
        velocity_parallel_mps = actual_v[0] * direction[0] + actual_v[1] * direction[1]
        velocity_cross_mps = actual_v[0] * cross[0] + actual_v[1] * cross[1]
        if pos[0] is not None and pos[1] is not None:
            rel = vec_sub([pos[0], pos[1]], phase["start_xy"])
            cross_track_error_m = rel[0] * cross[0] + rel[1] * cross[1]
            along_track_distance_m = rel[0] * direction[0] + rel[1] * direction[1]
        target_speed = float(phase["velocity_speed_mps"])
        velocity_error_mps = abs(float(velocity_parallel_mps) - target_speed)
        e_vel = velocity_error_mps / float(denominator) * 100.0
    elif phase["kind"] == "yaw" and yaw is not None:
        yaw_error_rad = wrap_angle_rad(float(yaw) - float(phase["target_yaw"]))
        e_yaw = abs(yaw_error_rad) / float(denominator) * 100.0
    return {
        "t_sec": t_sec,
        "phase": phase["name"],
        "kind": phase["kind"],
        "phase_elapsed_sec": phase_elapsed,
        "evaluated_component": component,
        "repeat": phase.get("repeat", ""),
        "desired_x": ref["position"][0],
        "desired_y": ref["position"][1],
        "desired_z": ref["position"][2],
        "desired_vx": ref["velocity"][0],
        "desired_vy": ref["velocity"][1],
        "desired_vz": ref["velocity"][2],
        "desired_yaw": ref["yaw"],
        "actual_x": pos[0],
        "actual_y": pos[1],
        "actual_z": pos[2],
        "actual_vx_world": vel[0],
        "actual_vy_world": vel[1],
        "actual_vz": vel[2],
        "actual_yaw": yaw,
        "target_x": phase.get("target_xy", ["", ""])[0],
        "target_y": phase.get("target_xy", ["", ""])[1],
        "target_yaw": phase.get("target_yaw", ""),
        "denominator": denominator,
        "position_error_m": position_error_m,
        "velocity_error_mps": velocity_error_mps,
        "velocity_parallel_mps": velocity_parallel_mps,
        "velocity_cross_mps": velocity_cross_mps,
        "cross_track_error_m": cross_track_error_m,
        "along_track_distance_m": along_track_distance_m,
        "yaw_error_rad": yaw_error_rad,
        "E_pos_percent": e_pos,
        "E_vel_percent": e_vel,
        "E_yaw_percent": e_yaw,
    }


def synthetic_actual(phase, ref):
    position = list(ref["position"])
    velocity = list(ref["velocity"])
    yaw = ref["yaw"]
    if phase["kind"] == "position":
        position[0] += 0.012 * float(phase["denominator"])
        position[1] -= 0.006 * float(phase["denominator"])
    elif phase["kind"] == "velocity":
        direction = normalize(ref["velocity"][:2])
        target_speed = float(phase["velocity_speed_mps"])
        velocity[0] = direction[0] * target_speed * 0.995
        velocity[1] = direction[1] * target_speed * 0.995
    elif phase["kind"] == "yaw":
        yaw = float(phase["target_yaw"]) - 0.018 * float(phase["denominator"])
    return {"position": position, "velocity": velocity, "yaw": yaw}


def generate_synthetic_samples(phases, records, hover, args):
    samples = []
    period = 1.0 / max(0.1, float(args.synthetic_rate_hz))
    for record in records:
        phase = record["phase"]
        t = record["start_sec"]
        while t <= record["end_sec"] + 1e-9:
            phase_elapsed = t - record["start_sec"]
            ref = reference_for_phase(phase, phase_elapsed, hover)
            samples.append(sample_from_actual(
                t, phase, phase_elapsed, ref, synthetic_actual(phase, ref)))
            t += period
    return samples


def sample_value(sample, key):
    value = sample.get(key)
    if value in ("", None):
        return None
    return float(value)


def samples_between(samples, start_elapsed, end_elapsed):
    return [
        sample for sample in samples
        if sample_value(sample, "phase_elapsed_sec") is not None
        and float(start_elapsed) - 1e-9 <= float(sample["phase_elapsed_sec"]) <= float(end_elapsed) + 1e-9
    ]


def enough_samples(samples, args):
    return len(samples) >= int(args.min_window_samples)


def position_stationarity_metrics(samples):
    times = [sample_value(s, "phase_elapsed_sec") for s in samples]
    xs = [sample_value(s, "actual_x") for s in samples]
    ys = [sample_value(s, "actual_y") for s in samples]
    vxs = [sample_value(s, "actual_vx_world") for s in samples]
    vys = [sample_value(s, "actual_vy_world") for s in samples]
    if any(value is None for value in times + xs + ys):
        return {"valid": False, "reason": "missing_position_samples"}
    speeds = [
        math.hypot(vx, vy)
        for vx, vy in zip(vxs, vys)
        if vx is not None and vy is not None
    ]
    x_slope = linear_slope(times, xs)
    y_slope = linear_slope(times, ys)
    return {
        "valid": True,
        "sample_count": len(samples),
        "speed_mean_mps": mean(speeds) if speeds else None,
        "position_std_m": math.hypot(stddev(xs), stddev(ys)),
        "position_slope_mps": math.hypot(x_slope, y_slope),
        "x_slope_mps": x_slope,
        "y_slope_mps": y_slope,
    }


def velocity_stationarity_metrics(samples, phase, eval_end_elapsed, args):
    times = [sample_value(s, "phase_elapsed_sec") for s in samples]
    vxs = [sample_value(s, "actual_vx_world") for s in samples]
    vys = [sample_value(s, "actual_vy_world") for s in samples]
    if any(value is None for value in times + vxs + vys):
        return {"valid": False, "reason": "missing_velocity_samples"}
    parallel = [sample_value(s, "velocity_parallel_mps") for s in samples]
    cross = [sample_value(s, "velocity_cross_mps") for s in samples]
    cross_track = [sample_value(s, "cross_track_error_m") for s in samples]
    along_track = [sample_value(s, "along_track_distance_m") for s in samples]
    if any(value is None for value in parallel + cross + cross_track + along_track):
        return {"valid": False, "reason": "missing_velocity_axis_samples"}
    target_speed = float(phase["velocity_speed_mps"])
    speed_abs_error = [abs(value - target_speed) for value in parallel]
    along_min = min(along_track)
    along_max = max(along_track)
    endpoint_margin_remaining_m = max(
        0.0,
        float(phase["velocity_length_m"]) - (float(eval_end_elapsed) * target_speed),
    )
    return {
        "valid": True,
        "sample_count": len(samples),
        "velocity_std_mps": math.hypot(stddev(vxs), stddev(vys)),
        "speed_std_mps": stddev(parallel),
        "velocity_slope_mps2": abs(linear_slope(times, parallel)),
        "speed_slope_mps2": abs(linear_slope(times, parallel)),
        "parallel_speed_mean_mps": mean(parallel),
        "parallel_speed_error_mean_mps": mean(speed_abs_error),
        "parallel_speed_error_max_mps": max(speed_abs_error),
        "cross_speed_abs_mean_mps": mean(abs(value) for value in cross),
        "cross_speed_abs_max_mps": max(abs(value) for value in cross),
        "cross_track_abs_mean_m": mean(abs(value) for value in cross_track),
        "cross_track_abs_max_m": max(abs(value) for value in cross_track),
        "along_track_min_m": along_min,
        "along_track_max_m": along_max,
        "along_track_progress_m": along_max - along_min,
        "start_margin_required_m": float(args.velocity_start_margin_m),
        "endpoint_margin_remaining_m": endpoint_margin_remaining_m,
        "endpoint_margin_required_m": float(args.velocity_endpoint_margin_m),
    }


def yaw_stationarity_metrics(samples):
    times = [sample_value(s, "phase_elapsed_sec") for s in samples]
    yaws = [sample_value(s, "actual_yaw") for s in samples]
    if any(value is None for value in times + yaws):
        return {"valid": False, "reason": "missing_yaw_samples"}
    unwrapped = unwrap_angles(yaws)
    rates = []
    for index in range(1, len(unwrapped)):
        dt = times[index] - times[index - 1]
        if dt > 1e-9:
            rates.append(abs((unwrapped[index] - unwrapped[index - 1]) / dt))
    return {
        "valid": True,
        "sample_count": len(samples),
        "yaw_std_rad": stddev(unwrapped),
        "yaw_slope_radps": abs(linear_slope(times, unwrapped)),
        "yaw_rate_mean_radps": mean(rates) if rates else 0.0,
    }


def metrics_pass(metrics, phase, args):
    if not metrics.get("valid"):
        return False
    if phase["kind"] == "position":
        speed_mean = metrics.get("speed_mean_mps")
        speed_ok = True if speed_mean is None else speed_mean <= float(args.position_stationary_speed_mean_mps)
        return (
            speed_ok
            and metrics["position_std_m"] <= float(args.position_stationary_std_m)
            and metrics["position_slope_mps"] <= float(args.position_stationary_slope_mps)
        )
    if phase["kind"] == "velocity":
        return (
            metrics["velocity_std_mps"] <= float(args.velocity_stationary_std_mps)
            and metrics["speed_std_mps"] <= float(args.velocity_stationary_speed_std_mps)
            and metrics["velocity_slope_mps2"] <= float(args.velocity_stationary_slope_mps2)
            and metrics["parallel_speed_error_mean_mps"] <= float(args.velocity_parallel_error_mean_mps)
            and metrics["parallel_speed_error_max_mps"] <= float(args.velocity_parallel_error_max_mps)
            and metrics["cross_speed_abs_mean_mps"] <= float(args.velocity_cross_speed_mean_mps)
            and metrics["cross_speed_abs_max_mps"] <= float(args.velocity_cross_speed_max_mps)
            and metrics["cross_track_abs_max_m"] <= float(args.velocity_cross_track_max_m)
            and metrics["along_track_min_m"] >= float(args.velocity_start_margin_m) - 1e-6
            and metrics["endpoint_margin_remaining_m"] >= float(args.velocity_endpoint_margin_m) - 1e-6
        )
    if phase["kind"] == "yaw":
        return (
            metrics["yaw_std_rad"] <= float(args.yaw_stationary_std_rad)
            and metrics["yaw_slope_radps"] <= float(args.yaw_stationary_slope_radps)
            and metrics["yaw_rate_mean_radps"] <= float(args.yaw_stationary_rate_mean_radps)
        )
    return False


def stationarity_for_phase(phase, samples, start_elapsed, end_elapsed, eval_end_elapsed, args):
    window_samples = samples_between(samples, start_elapsed, end_elapsed)
    if not enough_samples(window_samples, args):
        return {"valid": False, "reason": "insufficient_stationarity_samples", "sample_count": len(window_samples)}
    if phase["kind"] == "position":
        return position_stationarity_metrics(window_samples)
    if phase["kind"] == "velocity":
        return velocity_stationarity_metrics(window_samples, phase, eval_end_elapsed, args)
    if phase["kind"] == "yaw":
        return yaw_stationarity_metrics(window_samples)
    return {"valid": False, "reason": "no_stationarity_for_phase_kind"}


def settle_search_start_for_phase(phase, args):
    if phase["kind"] == "position":
        return float(args.position_settle_search_start_sec)
    if phase["kind"] == "velocity":
        return float(args.velocity_settle_search_start_sec)
    if phase["kind"] == "yaw":
        return float(args.yaw_settle_search_start_sec)
    return 0.0


def latest_eval_end_for_phase(phase, args):
    duration = float(phase["duration"])
    if phase["kind"] == "velocity":
        return duration - float(args.velocity_endpoint_margin_m) / float(phase["velocity_speed_mps"])
    return duration


def compute_eval_error(phase, eval_samples):
    denominator = phase.get("denominator", "")
    if phase.get("component") == "E_pos":
        xs = [sample_value(s, "actual_x") for s in eval_samples]
        ys = [sample_value(s, "actual_y") for s in eval_samples]
        zs = [sample_value(s, "actual_z") for s in eval_samples]
        if any(value is None for value in xs + ys + zs):
            return "", "", ""
        steady_actual = [mean(xs), mean(ys), mean(zs)]
        desired = [float(phase["target_xy"][0]), float(phase["target_xy"][1]), float(phase.get("target_z", 10.0))]
        error = vec3_norm(vec3_sub(steady_actual, desired)) / float(denominator) * 100.0
        return desired, steady_actual, error
    if phase.get("component") == "E_vel":
        parallel = [sample_value(s, "velocity_parallel_mps") for s in eval_samples]
        cross = [sample_value(s, "velocity_cross_mps") for s in eval_samples]
        if any(value is None for value in parallel + cross):
            return "", "", ""
        target_speed = float(phase["velocity_speed_mps"])
        steady_actual = {
            "mean_parallel_mps": mean(parallel),
            "mean_cross_mps": mean(cross),
            "mean_abs_cross_mps": mean(abs(value) for value in cross),
        }
        desired = {
            "target_speed_mps": target_speed,
            "direction_xy": normalize(phase["desired_velocity_xy"]),
        }
        error = abs(float(steady_actual["mean_parallel_mps"]) - target_speed) / float(denominator) * 100.0
        return desired, steady_actual, error
    if phase.get("component") == "E_yaw":
        errors = [
            wrap_angle_rad(float(s["actual_yaw"]) - float(phase["target_yaw"]))
            for s in eval_samples
            if s.get("actual_yaw") not in ("", None)
        ]
        if not errors:
            return "", "", ""
        mean_error = mean(errors)
        desired = float(phase["target_yaw"])
        steady_actual = {
            "mean_wrapped_yaw_error_rad": mean_error,
            "mean_absolute_yaw_rad": wrap_angle_rad(desired + float(mean_error)),
        }
        error = abs(float(steady_actual["mean_wrapped_yaw_error_rad"])) / float(denominator) * 100.0
        return desired, steady_actual, error
    return "", "", ""


def outline_steady_samples_for_phase(phase, phase_samples, args):
    if phase.get("component") != "E_vel":
        return None
    window_sec = float(args.outline_stationarity_window_sec)
    eval_window_sec = float(args.outline_eval_window_sec)
    target_speed = float(phase["velocity_speed_mps"])
    tolerance = abs(target_speed) * float(args.outline_velocity_tolerance_ratio)
    search_start = settle_search_start_for_phase(phase, args)
    latest_eval_end = latest_eval_end_for_phase(phase, args)
    latest_settle_start = latest_eval_end - window_sec - eval_window_sec
    if latest_settle_start < search_start - 1e-9:
        return {
            "settled": False,
            "not_settled_reason": "insufficient_phase_duration_for_outline_settle_plus_eval",
            "stationarity_window_sec": window_sec,
            "eval_window_sec": eval_window_sec,
            "tolerance_ratio": float(args.outline_velocity_tolerance_ratio),
            "tolerance_mps": tolerance,
            "settled_start_sec": "",
            "eval_start_sec": "",
            "eval_end_sec": "",
            "sample_count": len(phase_samples),
            "steady_sample_count": "",
            "eval_sample_count": "",
            "error_percent": "",
            "steady_actual": "",
        }
    candidate = search_start
    best_reason = ""
    while candidate <= latest_settle_start + 1e-9:
        stationarity_start = candidate
        stationarity_end = candidate + window_sec
        eval_start = stationarity_end
        eval_end = eval_start + eval_window_sec
        steady_samples = samples_between(phase_samples, stationarity_start, stationarity_end)
        if not enough_samples(steady_samples, args):
            best_reason = "insufficient_outline_stationarity_samples"
            candidate += float(args.settle_search_step_sec)
            continue
        parallel = [sample_value(s, "velocity_parallel_mps") for s in steady_samples]
        if any(value is None for value in parallel):
            best_reason = "missing_outline_velocity_samples"
            candidate += float(args.settle_search_step_sec)
            continue
        if all(abs(value - target_speed) <= tolerance for value in parallel):
            eval_samples = samples_between(phase_samples, eval_start, eval_end)
            if not enough_samples(eval_samples, args):
                best_reason = "insufficient_outline_eval_samples"
                candidate += float(args.settle_search_step_sec)
                continue
            desired, steady_actual, error = compute_eval_error(phase, eval_samples)
            return {
                "settled": True,
                "not_settled_reason": "",
                "stationarity_window_sec": window_sec,
                "eval_window_sec": eval_window_sec,
                "tolerance_ratio": float(args.outline_velocity_tolerance_ratio),
                "tolerance_mps": tolerance,
                "settled_start_sec": stationarity_start,
                "eval_start_sec": eval_start,
                "eval_end_sec": eval_end,
                "sample_count": len(phase_samples),
                "steady_sample_count": len(steady_samples),
                "eval_sample_count": len(eval_samples),
                "error_percent": error,
                "steady_actual": steady_actual,
                "desired": desired,
            }
        best_reason = "no_outline_1s_window_within_expected_plus_minus_2pct"
        candidate += float(args.settle_search_step_sec)
    return {
        "settled": False,
        "not_settled_reason": best_reason or "no_outline_stationary_window",
        "stationarity_window_sec": window_sec,
        "eval_window_sec": eval_window_sec,
        "tolerance_ratio": float(args.outline_velocity_tolerance_ratio),
        "tolerance_mps": tolerance,
        "settled_start_sec": "",
        "eval_start_sec": "",
        "eval_end_sec": "",
        "sample_count": len(phase_samples),
        "steady_sample_count": "",
        "eval_sample_count": "",
        "error_percent": "",
        "steady_actual": "",
    }


def outline_velocity_results(samples, records, args):
    by_name = {}
    for sample in samples:
        by_name.setdefault(sample["phase"], []).append(sample)
    results = []
    for record in records:
        phase = record["phase"]
        result = outline_steady_samples_for_phase(phase, by_name.get(phase["name"], []), args)
        if result is None:
            continue
        item = {
            "phase": phase["name"],
            "kind": phase["kind"],
            "start_sec": record["start_sec"],
            "end_sec": record["end_sec"],
            "evaluated_component": phase.get("component", ""),
            "repeat": phase.get("repeat", ""),
            "velocity_key": phase.get("velocity_key", ""),
            "velocity_speed_mps": phase.get("velocity_speed_mps", ""),
            "denominator": phase.get("denominator", ""),
        }
        item.update(result)
        if item.get("eval_start_sec") not in ("", None):
            item["eval_start_sec"] = record["start_sec"] + float(item["eval_start_sec"])
            item["eval_end_sec"] = record["start_sec"] + float(item["eval_end_sec"])
        if item.get("settled_start_sec") not in ("", None):
            item["settled_start_sec"] = record["start_sec"] + float(item["settled_start_sec"])
        results.append(item)
    return results


def outline_velocity_summary(outline_results):
    settled_errors = [
        float(result["error_percent"])
        for result in outline_results
        if result.get("settled") is True and result.get("error_percent") not in ("", None)
    ]
    not_settled = [
        result.get("phase", "")
        for result in outline_results
        if result.get("settled") is not True
    ]
    worst = None
    if settled_errors:
        settled = [
            result for result in outline_results
            if result.get("settled") is True and result.get("error_percent") not in ("", None)
        ]
        worst = max(settled, key=lambda result: float(result["error_percent"]))
    return {
        "policy": "outline_velocity_1s_within_expected_plus_minus_2pct_then_5s_eval_mean",
        "settled_window_sec": 1.0,
        "eval_window_sec": 5.0,
        "tolerance_ratio": 0.02,
        "window_count": len(outline_results),
        "settled_count": len(outline_results) - len(not_settled),
        "all_settled": len(not_settled) == 0 and bool(outline_results),
        "not_settled_phases": not_settled,
        "E_vel_avg": mean_present(settled_errors),
        "E_vel_max": max_present(settled_errors),
        "worst_window_repeat": worst.get("repeat") if worst else None,
        "worst_window_phase": worst.get("phase") if worst else None,
        "worst_window_time_sec": [worst.get("start_sec"), worst.get("end_sec")] if worst else None,
        "worst_window_eval_time_sec": [worst.get("eval_start_sec"), worst.get("eval_end_sec")] if worst else None,
    }


def phase_float(phase, key, default):
    value = phase.get(key, default)
    if value in ("", None):
        value = default
    return float(value)


def prealign_phase_settings(phase, args):
    return {
        "settle_sec": max(0.0, phase_float(phase, "prealign_settle_sec", getattr(args, "velocity_prealign_sec", 5.0))),
        "stable_sec": max(0.0, phase_float(phase, "prealign_stable_sec", getattr(args, "velocity_prealign_stable_sec", 3.0))),
        "timeout_sec": max(0.1, phase_float(phase, "prealign_timeout_sec", getattr(args, "velocity_prealign_timeout_sec", 15.0))),
        "tolerance_deg": phase_float(phase, "prealign_tolerance_deg", getattr(args, "velocity_prealign_tolerance_deg", 1.6)),
    }


def prealign_phase_audit(phase, phase_samples, args):
    if not phase.get("prealign_policy"):
        return {}
    target_yaw = phase.get("target_yaw", "")
    settings = prealign_phase_settings(phase, args)
    if target_yaw in ("", None):
        return {
            "settled": "",
            "not_settled_reason": "prealign_missing_target_yaw",
            "stationarity_metrics": {
                "prealign_policy": phase.get("prealign_policy", ""),
                "warning": True,
                "reason": "missing_target_yaw",
            },
        }
    rows = []
    for sample in phase_samples:
        if sample.get("actual_yaw") in ("", None) or sample.get("phase_elapsed_sec") in ("", None):
            continue
        rows.append((float(sample["phase_elapsed_sec"]), float(sample["actual_yaw"])))
    if not rows:
        return {
            "settled": "",
            "not_settled_reason": "prealign_no_yaw_sample",
            "stationarity_metrics": {
                "prealign_policy": phase.get("prealign_policy", ""),
                "target_yaw_rad": float(target_yaw),
                "tolerance_deg": settings["tolerance_deg"],
                "settle_sec": settings["settle_sec"],
                "stable_sec": settings["stable_sec"],
                "timeout_sec": settings["timeout_sec"],
                "warning": True,
                "reason": "no_yaw_sample",
            },
        }
    tolerance_deg = settings["tolerance_deg"]
    stable_since = None
    stable_for = 0.0
    release_elapsed = None
    max_error_after_settle = None
    aligned = False
    final_yaw = rows[-1][1]
    final_error_rad = wrap_angle_rad(final_yaw - float(target_yaw))
    final_error_deg = abs(math.degrees(final_error_rad))
    for elapsed, yaw in rows:
        error_rad = wrap_angle_rad(yaw - float(target_yaw))
        error_deg = abs(math.degrees(error_rad))
        if elapsed < settings["settle_sec"]:
            continue
        max_error_after_settle = error_deg if max_error_after_settle is None else max(max_error_after_settle, error_deg)
        if error_deg <= tolerance_deg:
            if stable_since is None:
                stable_since = elapsed
            stable_for = elapsed - stable_since
            if stable_for >= settings["stable_sec"]:
                aligned = True
                release_elapsed = elapsed
                final_yaw = yaw
                final_error_rad = error_rad
                final_error_deg = error_deg
                break
        else:
            stable_since = None
            stable_for = 0.0
    reason = "" if aligned else "prealign_yaw_warning"
    return {
        "settled": aligned,
        "not_settled_reason": reason,
        "stationarity_metrics": {
            "prealign_policy": phase.get("prealign_policy", ""),
            "target_yaw_rad": float(target_yaw),
            "final_yaw_rad": final_yaw,
            "final_yaw_error_rad": final_error_rad,
            "final_yaw_error_deg": final_error_deg,
            "tolerance_deg": tolerance_deg,
            "settle_sec": settings["settle_sec"],
            "stable_sec": settings["stable_sec"],
            "timeout_sec": settings["timeout_sec"],
            "max_total_sec": settings["settle_sec"] + settings["timeout_sec"],
            "stable_started_at_sec": stable_since,
            "stable_for_sec": stable_for,
            "release_elapsed_sec": release_elapsed,
            "max_error_after_settle_deg": max_error_after_settle,
            "aligned": aligned,
            "warning": not aligned,
            "sample_count": len(rows),
        },
        "desired": float(target_yaw),
        "steady_actual": {"final_yaw_rad": final_yaw},
    }


def settled_phase_result(phase, record, phase_samples, args):
    stationarity_window = float(args.stationarity_window_sec)
    eval_window = float(args.eval_window_sec)
    settle_search_start = settle_search_start_for_phase(phase, args)
    latest_eval_end = latest_eval_end_for_phase(phase, args)
    latest_settle_start = latest_eval_end - stationarity_window - eval_window
    denominator = phase.get("denominator", "")
    base = {
        "phase": phase["name"],
        "kind": phase["kind"],
        "start_sec": record["start_sec"],
        "end_sec": record["end_sec"],
        "duration_sec": record.get("duration_sec", phase["duration"]),
        "steady_window_sec": "",
        "eval_window_sec": eval_window,
        "stationarity_window_sec": stationarity_window,
        "transient_start_sec": record["start_sec"],
        "settle_search_start_sec": record["start_sec"] + settle_search_start,
        "settled_start_sec": "",
        "eval_start_sec": "",
        "eval_end_sec": "",
        "settled": "",
        "not_settled_reason": "",
        "stationarity_metrics": "",
        "evaluated_component": phase.get("component", ""),
        "repeat": phase.get("repeat", ""),
        "velocity_key": phase.get("velocity_key", ""),
        "velocity_speed_mps": phase.get("velocity_speed_mps", ""),
        "desired": "",
        "steady_actual": "",
        "denominator": denominator if phase.get("component") else "",
        "error_percent": "",
        "sample_count": len(phase_samples),
        "steady_sample_count": "",
        "eval_sample_count": "",
    }
    if phase.get("prealign_policy"):
        base.update(prealign_phase_audit(phase, phase_samples, args))
        return base
    if not phase.get("component"):
        return base
    if latest_settle_start < settle_search_start - 1e-9:
        base["settled"] = False
        base["not_settled_reason"] = "insufficient_phase_duration_for_settle_plus_eval"
        return base

    best_metrics = None
    search_step = float(args.settle_search_step_sec)
    candidate = settle_search_start
    while candidate <= latest_settle_start + 1e-9:
        stationarity_start = candidate
        stationarity_end = candidate + stationarity_window
        eval_start = stationarity_end
        eval_end = eval_start + eval_window
        metrics = stationarity_for_phase(
            phase, phase_samples, stationarity_start, stationarity_end, eval_end, args)
        best_metrics = metrics
        if metrics_pass(metrics, phase, args):
            eval_samples = samples_between(phase_samples, eval_start, eval_end)
            if enough_samples(eval_samples, args):
                desired, steady_actual, error = compute_eval_error(phase, eval_samples)
                base.update({
                    "settled_start_sec": record["start_sec"] + stationarity_start,
                    "eval_start_sec": record["start_sec"] + eval_start,
                    "eval_end_sec": record["start_sec"] + eval_end,
                    "settled": True,
                    "stationarity_metrics": metrics,
                    "desired": desired,
                    "steady_actual": steady_actual,
                    "error_percent": error,
                    "steady_sample_count": len(samples_between(phase_samples, stationarity_start, stationarity_end)),
                    "eval_sample_count": len(eval_samples),
                })
                return base
            best_metrics = dict(metrics)
            best_metrics["eval_sample_count"] = len(eval_samples)
            best_metrics["reason"] = "insufficient_eval_samples"
        candidate += search_step

    base["settled"] = False
    base["not_settled_reason"] = "no_stationary_window"
    if best_metrics:
        base["stationarity_metrics"] = best_metrics
    return base


def record_with_sample_times(record, phase_samples):
    result_record = dict(record)
    times = [float(sample["t_sec"]) for sample in phase_samples if sample.get("t_sec") not in ("", None)]
    if times:
        result_record["start_sec"] = min(times)
        result_record["end_sec"] = max(times)
        result_record["duration_sec"] = max(0.0, result_record["end_sec"] - result_record["start_sec"])
    return result_record


def compute_phase_results(samples, records, args):
    results = []
    by_name = {}
    for sample in samples:
        by_name.setdefault(sample["phase"], []).append(sample)
    for record in records:
        phase = record["phase"]
        phase_samples = by_name.get(phase["name"], [])
        result_record = record_with_sample_times(record, phase_samples)
        results.append(settled_phase_result(phase, result_record, phase_samples, args))
    return results


def prealign_summary_from_phase_results(results):
    audits = []
    for result in results:
        metrics = result.get("stationarity_metrics")
        if isinstance(metrics, dict) and metrics.get("prealign_policy"):
            audits.append({
                "phase": result.get("phase", ""),
                "aligned": metrics.get("aligned"),
                "warning": metrics.get("warning"),
                "final_yaw_error_deg": metrics.get("final_yaw_error_deg"),
                "tolerance_deg": metrics.get("tolerance_deg"),
                "sample_count": metrics.get("sample_count"),
            })
    return {
        "count": len(audits),
        "warning_count": sum(1 for item in audits if item.get("warning") is True),
        "audits": audits,
    }


def errors_from_phase_results(results):
    values = {"E_pos": [], "E_yaw": []}
    velocity_values = {}
    window_max = {"E_pos": [], "E_yaw": []}
    velocity_window_max = {}
    not_settled = []
    for result in results:
        component = result["evaluated_component"]
        error = result["error_percent"]
        if component in ("E_pos", "E_vel", "E_yaw") and result.get("settled") is not True:
            not_settled.append(result["phase"])
            continue
        if component in values and error not in ("", None):
            values[component].append(float(error))
            window_max[component].append(float(error))
        elif component == "E_vel" and error not in ("", None):
            velocity_key = result.get("velocity_key") or "unknown"
            velocity_values.setdefault(velocity_key, []).append(float(error))
            velocity_window_max.setdefault(velocity_key, []).append(float(error))
    errors = {component: mean_present(component_values) for component, component_values in values.items()}
    for key, component_values in velocity_values.items():
        errors["E_vel_%s" % key] = mean_present(component_values)
        errors["E_vel_%s_window_max" % key] = max_present(velocity_window_max.get(key, []))
    e_vel_candidates = [
        (key, value) for key, value in errors.items()
        if key.startswith("E_vel_") and not key.endswith("_window_max") and value is not None
    ]
    if e_vel_candidates:
        selected_key, selected_value = min(e_vel_candidates, key=lambda item: item[1])
        selected_suffix = selected_key.replace("E_vel_", "")
        errors["selected_velocity_speed_mps"] = float(
            selected_suffix.replace("mps", "").replace("p", "."))
        errors["E_vel_selected"] = selected_value
        errors["E_vel_window_max"] = errors.get("E_vel_%s_window_max" % selected_suffix)
    else:
        errors["selected_velocity_speed_mps"] = None
        errors["E_vel_selected"] = None
        errors["E_vel_window_max"] = None
    errors["E_pos_window_max"] = max_present(window_max["E_pos"])
    errors["E_yaw_window_max"] = max_present(window_max["E_yaw"])
    for key, value in list(errors.items()):
        if key.startswith("E_vel_") and key not in ("E_vel_selected", "E_vel_window_max") and not key.endswith("_window_max"):
            suffix = key.replace("E_vel_", "")
            errors["E3.10_%s" % suffix] = max_present([errors["E_pos"], value, errors["E_yaw"]])
    errors["E3.10_selected"] = max_present([errors["E_pos"], errors["E_vel_selected"], errors["E_yaw"]])
    errors["E_vel"] = errors["E_vel_selected"]
    errors["E3.10"] = errors["E3.10_selected"]
    errors["all_metric_windows_settled"] = len(not_settled) == 0
    errors["not_settled_phases"] = not_settled
    velocity_formal = [
        result for result in results
        if result.get("evaluated_component") == "E_vel"
    ]
    velocity_settled = [
        result for result in velocity_formal
        if result.get("settled") is True and result.get("error_percent") not in ("", None)
    ]
    velocity_worst = None
    if velocity_settled:
        velocity_worst = max(velocity_settled, key=lambda result: float(result["error_percent"]))
    pos_settled = [
        result for result in results
        if result.get("evaluated_component") == "E_pos"
        and result.get("settled") is True
        and result.get("error_percent") not in ("", None)
    ]
    yaw_settled = [
        result for result in results
        if result.get("evaluated_component") == "E_yaw"
        and result.get("settled") is True
        and result.get("error_percent") not in ("", None)
    ]
    errors["E_vel_window_count"] = len(velocity_formal)
    errors["E_vel_settled_count"] = len(velocity_settled)
    errors["E_vel_percent_sum"] = sum(float(result["error_percent"]) for result in velocity_settled) if velocity_settled else None
    errors["E_pos_settled_count"] = len(pos_settled)
    errors["E_pos_percent_sum"] = sum(float(result["error_percent"]) for result in pos_settled) if pos_settled else None
    errors["E_yaw_settled_count"] = len(yaw_settled)
    errors["E_yaw_percent_sum"] = sum(float(result["error_percent"]) for result in yaw_settled) if yaw_settled else None
    errors["E_vel_all_windows_settled"] = len(velocity_formal) == len(velocity_settled) and bool(velocity_formal)
    errors["E_vel_not_settled_phases"] = [
        result.get("phase", "")
        for result in velocity_formal
        if result.get("settled") is not True
    ]
    errors["E_vel_worst_window_repeat"] = velocity_worst.get("repeat") if velocity_worst else None
    errors["E_vel_worst_window_phase"] = velocity_worst.get("phase") if velocity_worst else None
    errors["E_vel_worst_window_time_sec"] = [
        velocity_worst.get("start_sec"),
        velocity_worst.get("end_sec"),
    ] if velocity_worst else None
    errors["E_vel_worst_window_eval_time_sec"] = [
        velocity_worst.get("eval_start_sec"),
        velocity_worst.get("eval_end_sec"),
    ] if velocity_worst else None
    return errors



def terminal_summary(errors):
    if errors.get("speed_only"):
        old_avg = errors.get("E_vel_selected")
        old_max = errors.get("E_vel_window_max")
        outline = errors.get("outline_velocity") or {}
        result = "PASS" if errors.get("E_vel_all_windows_settled") else "FAIL"
        return "\n".join([
            "任务状态: 生成速度小测试结果",
            "",
            "===== SPEED-ONLY FINAL =====",
            "旧口径 3 s 稳态窗: avg %s | max %s | settled %s" % (
                fmt_display_percent(old_avg),
                fmt_display_percent(old_max),
                errors.get("E_vel_all_windows_settled"),
            ),
            "大纲口径 1 s ±2%% + 5 s 均值: avg %s | max %s | settled %s" % (
                fmt_display_percent(outline.get("E_vel_avg")),
                fmt_display_percent(outline.get("E_vel_max")),
                outline.get("all_settled"),
            ),
            "控制结果: %s" % color_result(result),
        ])
    if errors.get("diagnostic_only") == "geometry_check_no_samples":
        return "\n".join([
            "===== FINAL =====",
            "geometry check complete",
            "result %s" % color_result("PASS"),
        ])
    final_value = errors.get("E3.10_selected")
    passed = (
        final_value is not None
        and float(final_value) <= 5.0
        and bool(errors.get("all_metric_windows_settled", False))
    )
    result = "PASS" if passed else "FAIL"
    lines = [
        "任务状态: 生成最终结果",
        "",
        "===== FINAL =====",
        component_average_detail_from_errors("E_vel", errors),
        component_average_detail_from_errors("E_pos", errors),
        component_average_detail_from_errors("E_yaw", errors),
        final_metric_detail(errors),
        "",
        "控制结果: %s" % color_result(result),
    ]
    return "\n".join(lines)

def metric_result(errors):
    if errors.get("diagnostic_only") == "geometry_check_no_samples":
        return "GEOMETRY_PASS"
    if errors.get("speed_only"):
        final_value = errors.get("E_vel_selected")
        passed = (
            final_value is not None
            and float(final_value) <= 5.0
            and bool(errors.get("E_vel_all_windows_settled", False))
        )
        return "PASS" if passed else "FAIL"
    final_value = errors.get("E3.10_selected")
    passed = (
        final_value is not None
        and float(final_value) <= 5.0
        and bool(errors.get("all_metric_windows_settled", False))
    )
    return "PASS" if passed else "FAIL"


def component_for_phase(phase):
    return phase.get("component") or ""


def next_component(records, start_index):
    for record in records[start_index + 1:]:
        component = component_for_phase(record["phase"])
        if component:
            return component
    return ""


def fc_plan_line(geometry):
    velocity_rounds = sum(int(test.get("windows", 0)) for test in geometry.get("velocity_tests", []))
    speed = 0.0
    if geometry.get("velocity_tests"):
        speed = float(geometry["velocity_tests"][0].get("speed_mps", 0.0))
    position_rounds = int(geometry.get("position_test", {}).get("windows", 10))
    yaw_rounds = int(geometry.get("yaw_test", {}).get("windows", 10))
    return (
        "plan: velocity %.1f m/s %d rounds, position %d rounds, yaw %d rounds"
        % (speed, velocity_rounds, position_rounds, yaw_rounds)
    )


def phase_round_text(phase):
    return "%02d/10" % int(phase.get("repeat") or 0)


def phase_target_text(phase, hover):
    if phase["kind"] == "velocity":
        repeat = int(phase.get("repeat") or 0)
        direction = "A -> B" if repeat % 2 == 1 else "B -> A"
        speed = phase.get("velocity_speed_mps") or phase.get("denominator")
        if speed not in ("", None):
            return "%s %s" % (direction, fmt_float(speed, "m/s"))
        return direction
    if phase["kind"] == "position":
        xy = phase.get("target_xy", [hover[0], hover[1]])
        z = phase.get("target_z", hover[2])
        return fmt_xyz([xy[0], xy[1], z])
    if phase["kind"] == "yaw":
        return fmt_yaw_rad(phase.get("target_yaw"))
    return ""


def phase_metric_name(phase):
    if phase["kind"] == "velocity":
        return "e_vel"
    if phase["kind"] == "position":
        return "e_pos"
    if phase["kind"] == "yaw":
        return "e_att"
    return ""


def phase_begin_block(phase, hover):
    kind = phase["kind"]
    label = {"velocity": "速度测试", "position": "位置测试", "yaw": "偏航测试"}.get(kind, kind)
    target_label = {"velocity": "期望速度", "position": "期望位置", "yaw": "期望 yaw"}.get(kind, "期望值")
    return "\n".join([
        color_minor("[%s %s]" % (label, phase_round_text(phase))),
        "%s: %s" % (target_label, phase_target_text(phase, hover)),
    ])


def phase_end_block(phase, result, hover):
    kind = phase["kind"]
    lines = []
    if result.get("settled") is True:
        if kind == "velocity":
            lines.append("实际稳态速度: %s" % velocity_steady_text(result))
        elif kind == "position":
            lines.append("实际稳态位置: %s" % position_steady_text(result))
        elif kind == "yaw":
            lines.append("实际稳态 yaw: %s" % yaw_steady_text(result))
        lines.append(steady_window_text(result))
        lines.append(phase_result_display_line(phase, result))
    else:
        reason = result.get("not_settled_reason") or "unknown"
        lines.append("稳态状态: 未达到")
        lines.append("原因: %s" % reason)
    return "\n".join(lines)


def component_header(component, geometry):
    if component == "E_vel":
        metric_text = "速度"
        if geometry.get("speed_only"):
            metric_text = "速度-only"
        return "\n".join([
            color_major("========== F250 FC 3.10 Metrics =========="),
            "任务状态: 速度控制测试",
            "测试内容: 3.10 控制稳定误差",
            "测试指标: %s" % metric_text,
            "速度测试: 10 次 AB/BA 窗口",
            "",
            "速度稳态误差",
            "= 实际稳态速度与期望速度的偏差 / 期望速度 * 100%",
        ])
    if component == "E_pos":
        return "\n".join([
            color_major("========== F250 FC 3.10 Metrics =========="),
            "任务状态: 位置控制测试",
            "",
            "位置稳态误差",
            "= 实际稳态位置与期望位置的三维偏差",
            "  / 规划位置目标间三维距离 * 100%",
        ])
    if component == "E_yaw":
        return "\n".join([
            color_major("========== F250 FC 3.10 Metrics =========="),
            "任务状态: 偏航控制测试",
            "",
            "偏航稳态误差",
            "= 实际稳态绝对 yaw 与期望绝对 yaw 的偏差 / yaw 测试分母 * 100%",
        ])
    return ""


def component_done_block(component, phase_results):
    errors = errors_from_phase_results(phase_results)
    if component == "E_vel":
        return "\n".join([
            color_minor("[速度控制阶段结果]"),
            component_average_detail(component, phase_results, errors.get("E_vel_2mps")),
        ])
    if component == "E_pos":
        return "\n".join([
            color_minor("[位置控制阶段结果]"),
            component_average_detail(component, phase_results, errors.get("E_pos")),
        ])
    if component == "E_yaw":
        return "\n".join([
            color_minor("[偏航控制阶段结果]"),
            component_average_detail(component, phase_results, errors.get("E_yaw")),
        ])
    return ""

def emit_progress_from_samples(display, records, samples, hover, args, geometry):
    if display is None:
        return
    by_name = {}
    for sample in samples:
        by_name.setdefault(sample["phase"], []).append(sample)
    phase_results = []
    current_component = ""
    for index, record in enumerate(records):
        phase = record["phase"]
        component = component_for_phase(phase)
        if not component:
            continue
        if component != current_component:
            header = component_header(component, geometry)
            if header:
                display.write("\n" + header)
            current_component = component
        display.write("\n" + phase_begin_block(phase, hover))
        result = settled_phase_result(phase, record, by_name.get(phase["name"], []), args)
        phase_results.append(result)
        display.write(phase_end_block(phase, result, hover))
        if next_component(records, index) != component:
            done = component_done_block(component, phase_results)
            if done:
                display.write("\n" + done)


class Display:
    def __init__(self, path):
        self.path = path
        if path:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            open(path, "a", encoding="utf-8").close()

    def write(self, text):
        print(text, flush=True)
        if self.path:
            with open(self.path, "a", encoding="utf-8") as handle:
                handle.write(strip_ansi(text).rstrip() + "\n")

    def over(self):
        print("OVER", flush=True)
        if self.path:
            with open(self.path, "a", encoding="utf-8") as handle:
                handle.write("OVER\n")

    def close(self):
        return


class LiveRunner:
    def __init__(self, args, hover, records, display=None, geometry=None):
        self.args = args
        self.hover = hover
        self.records = records
        self.display = display
        self.geometry = geometry or {}
        self.samples = []
        self.phase_results = []
        self.latest_odom = None
        self.rospy = None
        self.PositionCommand = None
        self.String = None
        self.publisher = None
        self.mode_publisher = None

    def setup_ros(self):
        import rospy
        from nav_msgs.msg import Odometry
        from quadrotor_msgs.msg import PositionCommand
        from std_msgs.msg import String
        self.rospy = rospy
        self.PositionCommand = PositionCommand
        self.String = String
        rospy.init_node("f250_fc_3_10_steady_state", anonymous=True)
        self.publisher = rospy.Publisher(self.args.command_topic, PositionCommand, queue_size=1)
        self.mode_publisher = rospy.Publisher(self.args.control_mode_topic, String, queue_size=1)
        rospy.Subscriber(self.args.odom_topic, Odometry, self.odom_cb, queue_size=1)

    def odom_cb(self, msg):
        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        world_vx = cos_yaw * float(vel.x) - sin_yaw * float(vel.y)
        world_vy = sin_yaw * float(vel.x) + cos_yaw * float(vel.y)
        self.latest_odom = {
            "position": [float(pos.x), float(pos.y), float(pos.z)],
            "velocity": [world_vx, world_vy, float(vel.z)],
            "yaw": yaw,
        }

    def wait_for_odom(self):
        deadline = time.monotonic() + float(self.args.odom_timeout_sec)
        while not self.rospy.is_shutdown() and time.monotonic() < deadline:
            if self.latest_odom is not None:
                return True
            time.sleep(0.05)
        return self.latest_odom is not None

    def make_command(self, ref, trajectory_id):
        msg = self.PositionCommand()
        msg.header.stamp = self.rospy.Time.now()
        msg.header.frame_id = self.args.frame_id
        msg.position.x, msg.position.y, msg.position.z = ref["position"]
        msg.velocity.x, msg.velocity.y, msg.velocity.z = ref["velocity"]
        msg.acceleration.x, msg.acceleration.y, msg.acceleration.z = ref["acceleration"]
        msg.yaw = ref["yaw"]
        msg.yaw_dot = ref["yaw_dot"]
        msg.trajectory_id = trajectory_id
        msg.trajectory_flag = getattr(self.PositionCommand, "TRAJECTORY_STATUS_READY", 1)
        return msg

    def publish_mode(self, mode):
        if self.mode_publisher is not None and self.String is not None:
            self.mode_publisher.publish(self.String(data=str(mode)))

    def publish_mode_for_phase(self, phase):
        self.publish_mode(phase.get("control_mode", "position"))

    def publish_reference(self, ref, trajectory_id):
        self.publisher.publish(self.make_command(ref, trajectory_id))

    def publish_p0_for(self, duration_sec):
        if self.publisher is None or self.rospy is None:
            return
        phase = {
            "name": "return_p0_hover",
            "kind": "hold",
            "duration": duration_sec,
            "component": "",
            "target_xy": [self.hover[0], self.hover[1]],
            "target_yaw": self.hover[3],
        }
        trajectory_id = int(time.time()) & 0xFFFFFFFF
        period = 1.0 / max(1.0, float(self.args.rate_hz))
        end = time.monotonic() + max(0.0, float(duration_sec))
        self.publish_mode("position")
        while not self.rospy.is_shutdown() and time.monotonic() < end:
            self.publish_mode("position")
            self.publish_reference(reference_for_phase(phase, 0.0, self.hover), trajectory_id)
            time.sleep(period)

    def run(self):
        self.setup_ros()
        if not self.wait_for_odom():
            self.publish_p0_for(self.args.final_command_sec)
            return self.samples, "failed_no_odom", 2

        trajectory_id = int(time.time()) & 0xFFFFFFFF
        period = 1.0 / max(1.0, float(self.args.rate_hz))
        global_start = time.monotonic()
        try:
            current_component = ""
            for record_index, record in enumerate(self.records):
                phase = record["phase"]
                component = component_for_phase(phase)
                self.publish_mode_for_phase(phase)
                if component and component != current_component:
                    header = component_header(component, self.geometry)
                    if self.display and header:
                        self.display.write("\n" + header)
                    current_component = component
                if component and self.display:
                    self.display.write("\n" + phase_begin_block(phase, self.hover))
                phase_start = time.monotonic()
                phase_samples = []
                prealign_stable_since = None
                while not self.rospy.is_shutdown():
                    phase_elapsed = time.monotonic() - phase_start
                    if phase_elapsed > float(phase["duration"]):
                        break
                    ref = reference_for_phase(phase, phase_elapsed, self.hover)
                    self.publish_mode_for_phase(phase)
                    self.publish_reference(ref, trajectory_id)
                    sample = sample_from_actual(
                        time.monotonic() - global_start,
                        phase,
                        phase_elapsed,
                        ref,
                        self.latest_odom,
                    )
                    self.samples.append(sample)
                    phase_samples.append(sample)
                    if phase.get("prealign_policy"):
                        settings = prealign_phase_settings(phase, self.args)
                        actual_yaw = sample.get("actual_yaw")
                        if actual_yaw not in ("", None) and phase_elapsed >= settings["settle_sec"]:
                            error_deg = abs(math.degrees(wrap_angle_rad(float(actual_yaw) - float(phase["target_yaw"]))))
                            if error_deg <= settings["tolerance_deg"]:
                                if prealign_stable_since is None:
                                    prealign_stable_since = phase_elapsed
                                if phase_elapsed - prealign_stable_since >= settings["stable_sec"]:
                                    break
                            else:
                                prealign_stable_since = None
                    time.sleep(period)
                if component:
                    result_record = record_with_sample_times(record, phase_samples)
                    result = settled_phase_result(phase, result_record, phase_samples, self.args)
                    self.phase_results.append(result)
                    if self.display:
                        self.display.write(phase_end_block(phase, result, self.hover))
                        if next_component(self.records, record_index) != component:
                            done = component_done_block(component, self.phase_results)
                            if done:
                                self.display.write("\n" + done)
        finally:
            self.publish_mode("position")
            self.publish_p0_for(self.args.final_command_sec)
        return self.samples, "complete_returned_p0", 0


def write_csv(path, rows, fieldnames):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def output_paths(args):
    return {
        "summary_json": args.summary_json or os.path.join(args.run_dir, "fc_3_10_summary.json"),
        "samples_csv": args.samples_csv or os.path.join(args.run_dir, "fc_3_10_samples.csv"),
        "phase_csv": args.phase_csv or os.path.join(args.run_dir, "fc_3_10_phases.csv"),
        "terminal_display_log": args.display_log or os.path.join(args.run_dir, "fc_3_10_terminal.log"),
        "geometry_audit_json": args.geometry_audit_json or os.path.join(args.run_dir, "fc_3_10_geometry_audit.json"),
        "decagon_csv": args.decagon_csv or os.path.join(args.run_dir, "fc_3_10_decagon_points.csv"),
    }


def write_geometry_outputs(paths, geometry):
    os.makedirs(os.path.dirname(paths["geometry_audit_json"]), exist_ok=True)
    with open(paths["geometry_audit_json"], "w", encoding="utf-8") as handle:
        json.dump(geometry, handle, indent=2, sort_keys=True)
        handle.write("\n")
    rows = []
    for point in geometry["position_test"]["decagon_points"]:
        rows.append({
            "index": point["index"],
            "x": point["x"],
            "y": point["y"],
            "z": point["z"],
        })
    write_csv(paths["decagon_csv"], rows, ["index", "x", "y", "z"])


def formula_metadata(args):
    return {
        "scope": "FC-only steady-state evidence; independent of route/planner acceptance",
        "transient_policy": (
            "For each constant target phase, ignore transient response before settle_search_start; "
            "search for a stationary window, then compute error only over the following eval window."
        ),
        "E_pos_i": (
            "norm3d(mean(eval actual_xyz) - target_xyz) / previous_to_current_step_3d * 100"
        ),
        "E_vel_i": "abs(mean(v_parallel) - target_speed) / target_speed * 100",
        "E_yaw_i": "abs(mean(wrap(actual_yaw - target_yaw))) / yaw_denominator * 100",
        "E_pos": "mean_i(E_pos_i) over 10 decagon position windows",
        "E_vel_2mps": "mean_i(E_vel_i) over 10 straight velocity windows commanded at 2 m/s",
        "E_vel_selected": "E_vel_2mps when all formal 2 m/s velocity windows are settled",
        "E_yaw": "mean_i(E_yaw_i) over 10 yaw windows, displayed as e_att",
        "E3.10_2mps": "max(E_pos, E_vel_2mps, E_yaw)",
        "E3.10_selected": "max(E_pos, E_vel_selected, E_yaw)",
        "velocity_frame_policy": (
            "MAVROS Odometry twist is treated as body-frame linear velocity and rotated "
            "by current yaw into local/world XY before comparison"
        ),
        "velocity_control_policy": (
            "Velocity prealign hold phases, when enabled, point yaw along the upcoming AB/BA leg, wait for continuous yaw stability, and carry no evaluated component. "
            "Velocity windows request bridge mode velocity and publish MAVROS raw-local setpoints: "
            "XY velocity, Z position hold, and yaw hold along the current leg. Line endpoints define safety and scoring bounds, not XY position targets. "
            "Command speed may use a transparent gain while scoring remains against target speed."
        ),
        "not_settled_policy": "A phase that does not meet stationarity produces no formal steady-state error.",
        "legacy_eval_window_policy": "The 5 s eval window is averaged after a preceding 3 s stationarity gate; it is not re-gated as a second steady-state window.",
        "outline_velocity_eval_window_policy": "For comparison, velocity also reports the outline policy: first continuous 1 s within expected speed +/-2%, then mean error over the following at least 5 s eval window with no second steady gate.",
        "velocity_prealign_sec": float(args.velocity_prealign_sec),
        "velocity_prealign_stable_sec": float(args.velocity_prealign_stable_sec),
        "velocity_prealign_timeout_sec": float(args.velocity_prealign_timeout_sec),
        "velocity_prealign_tolerance_deg": float(args.velocity_prealign_tolerance_deg),
        "velocity_prealign_enabled": float(args.velocity_prealign_sec) > 0.0,
        "velocity_prealign_policy": "non-metric position-hold phase before every AB/BA velocity window when enabled; target yaw follows the upcoming leg direction; settle 5s, then require continuous 3s within 1.6deg, warning-only if not stable before timeout",
        "speed_only": bool(args.speed_only),
        "eval_window_sec": float(args.eval_window_sec),
        "stationarity_window_sec": float(args.stationarity_window_sec),
        "outline_stationarity_window_sec": float(args.outline_stationarity_window_sec),
        "outline_velocity_tolerance_ratio": float(args.outline_velocity_tolerance_ratio),
        "outline_eval_window_sec": float(args.outline_eval_window_sec),
        "settle_search_step_sec": float(args.settle_search_step_sec),
        "position_settle_search_start_sec": float(args.position_settle_search_start_sec),
        "velocity_settle_search_start_sec": float(args.velocity_settle_search_start_sec),
        "yaw_settle_search_start_sec": float(args.yaw_settle_search_start_sec),
        "velocity_speeds_mps": list(args.velocity_speeds_mps),
        "velocity_lengths_m": list(args.velocity_lengths_m),
        "velocity_command_gain": float(args.velocity_command_gain),
        "velocity_interleg_hold_sec": float(args.velocity_interleg_hold_sec),
        "velocity_reset_hold_sec": float(args.velocity_reset_hold_sec),
        "velocity_final_reset_hold_sec": float(args.velocity_final_reset_hold_sec),
        "position_radius_m": float(args.position_radius_m),
        "stationarity_thresholds": {
            "position_speed_mean_mps": float(args.position_stationary_speed_mean_mps),
            "position_std_m": float(args.position_stationary_std_m),
            "position_slope_mps": float(args.position_stationary_slope_mps),
            "velocity_std_mps": float(args.velocity_stationary_std_mps),
            "velocity_speed_std_mps": float(args.velocity_stationary_speed_std_mps),
            "velocity_slope_mps2": float(args.velocity_stationary_slope_mps2),
            "velocity_parallel_error_mean_mps": float(args.velocity_parallel_error_mean_mps),
            "velocity_parallel_error_max_mps": float(args.velocity_parallel_error_max_mps),
            "velocity_cross_speed_mean_mps": float(args.velocity_cross_speed_mean_mps),
            "velocity_cross_speed_max_mps": float(args.velocity_cross_speed_max_mps),
            "velocity_cross_track_max_m": float(args.velocity_cross_track_max_m),
            "velocity_start_margin_m": float(args.velocity_start_margin_m),
            "yaw_std_rad": float(args.yaw_stationary_std_rad),
            "yaw_slope_radps": float(args.yaw_stationary_slope_radps),
            "yaw_rate_mean_radps": float(args.yaw_stationary_rate_mean_radps),
        },
    }


def summary_document(args, hover, geometry, phase_results, errors, samples, paths, p0_status_values, run_state):
    return {
        "schema": SCHEMA,
        "created_at": now_iso(),
        "vehicle": "f250",
        "run_label": args.run_label,
        "run_dir": args.run_dir,
        "dry_run": bool(args.dry_run),
        "geometry_check": bool(args.geometry_check),
        "run_state": run_state,
        "result": metric_result(errors),
        "metric_policy": (
            "Metric 3.10 is independent FC-only steady-state evidence. "
            "It is not route/planner acceptance and does not write route pass/fail or yaw pass/fail files."
        ),
        "speed_only": bool(args.speed_only),
        "topics": {
            "actual_state": args.odom_topic,
            "command": args.command_topic,
            "existing_setpoint_chain": "PositionCommand on /planning/pos_cmd to bridge; FC velocity phases request velocity mode",
            "setpoint_topic": args.setpoint_topic,
            "velocity_setpoint_topic": args.velocity_setpoint_topic,
            "control_mode_topic": args.control_mode_topic,
        },
        "source_p0_hover": {
            "status_env": args.p0_status or "",
            "run_dir": args.p0_run_dir or p0_status_values.get("run_dir", ""),
            "state": p0_status_values.get("state", ""),
            "screen_name": p0_status_values.get("screen_name", ""),
            "hover_target_raw": p0_status_values.get("hover_target", ""),
            "hover_target_arg": list(args.hover_target),
            "route_waypoints_csv": args.route_waypoints_csv or p0_status_values.get("route_waypoints_csv", ""),
            "sensor": (
                p0_status_values.get("perception_source")
                or p0_status_values.get("sensor")
                or p0_status_values.get("source_p0_sensor", "")
            ),
            "raw_cloud_topic": (
                p0_status_values.get("raw_cloud_topic")
                or p0_status_values.get("source_p0_raw_cloud_topic", "")
            ),
        },
        "command_hover_target": {
            "x": hover[0],
            "y": hover[1],
            "z": hover[2],
            "yaw": hover[3],
            "source": (
                "current run route first waypoint with fixed z=10 m; "
                "authoritative P0 fallback when no current route CSV is available"
            ),
        },
        "test_design": {
            "velocity": geometry["velocity_test"],
            "velocity_tests": geometry["velocity_tests"],
            "position": geometry["position_test"],
            "yaw": geometry["yaw_test"],
        },
        "formula_metadata": formula_metadata(args),
        "errors_percent": errors,
        "velocity_prealign_audit": prealign_summary_from_phase_results(phase_results),
        "phase_results": phase_results,
        "sample_count": len(samples),
        "outputs": paths,
        "geometry_audit": {
            "path": paths["geometry_audit_json"],
            "audit_conclusion": geometry["audit_conclusion"],
            "planner_obstacle_clearance": geometry["planner_obstacle_clearance"],
        },
    }


SAMPLE_FIELDS = [
    "t_sec", "phase", "kind", "phase_elapsed_sec", "evaluated_component", "repeat",
    "desired_x", "desired_y", "desired_z", "desired_vx", "desired_vy", "desired_vz", "desired_yaw",
    "actual_x", "actual_y", "actual_z", "actual_vx_world", "actual_vy_world", "actual_vz", "actual_yaw",
    "target_x", "target_y", "target_yaw", "denominator",
    "position_error_m", "velocity_error_mps", "yaw_error_rad",
    "velocity_parallel_mps", "velocity_cross_mps", "cross_track_error_m", "along_track_distance_m",
    "E_pos_percent", "E_vel_percent", "E_yaw_percent",
]

PHASE_FIELDS = [
    "phase", "kind", "start_sec", "end_sec", "duration_sec", "steady_window_sec",
    "eval_window_sec", "stationarity_window_sec", "transient_start_sec",
    "settle_search_start_sec", "settled_start_sec", "eval_start_sec", "eval_end_sec",
    "settled", "not_settled_reason", "stationarity_metrics",
    "evaluated_component", "repeat", "velocity_key", "velocity_speed_mps",
    "desired", "steady_actual", "denominator", "error_percent",
    "sample_count", "steady_sample_count", "eval_sample_count",
]


def write_outputs(args, hover, geometry, samples, records, paths, p0_status_values, run_state):
    if args.geometry_check and not samples:
        phase_results = []
        errors = {
            "E_pos": None,
            "E_vel_2mps": None,
            "E_vel_selected": None,
            "selected_velocity_speed_mps": None,
            "E_yaw": None,
            "E3.10_2mps": None,
            "E3.10_selected": None,
            "E_vel": None,
            "E3.10": None,
            "all_metric_windows_settled": True,
            "not_settled_phases": [],
            "diagnostic_only": "geometry_check_no_samples",
        }
    else:
        phase_results = compute_phase_results(samples, records, args)
        errors = errors_from_phase_results(phase_results)
        if args.speed_only:
            outline_results = outline_velocity_results(samples, records, args)
            errors["speed_only"] = True
            errors["all_metric_windows_settled"] = errors.get("E_vel_all_windows_settled", False)
            errors["outline_velocity"] = outline_velocity_summary(outline_results)
        else:
            outline_results = outline_velocity_results(samples, records, args)
            errors["outline_velocity"] = outline_velocity_summary(outline_results)
    write_csv(paths["samples_csv"], samples, SAMPLE_FIELDS)
    write_csv(paths["phase_csv"], phase_results, PHASE_FIELDS)
    summary = summary_document(
        args, hover, geometry, phase_results, errors, samples, paths, p0_status_values, run_state)
    if errors.get("speed_only"):
        outline_results = outline_velocity_results(samples, records, args)
        errors["outline_velocity"] = outline_velocity_summary(outline_results)
        summary["outline_velocity"] = errors["outline_velocity"]
        summary["speed_only"] = True
        summary["errors_percent"] = errors
    os.makedirs(os.path.dirname(paths["summary_json"]), exist_ok=True)
    with open(paths["summary_json"], "w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2, sort_keys=True)
        handle.write("\n")
    return summary


def positive_duration(value):
    value = float(value)
    if value <= 0.0:
        raise argparse.ArgumentTypeError("duration must be positive")
    return value


def positive_float(value):
    value = float(value)
    if value <= 0.0:
        raise argparse.ArgumentTypeError("value must be positive")
    return value


def build_parser():
    parser = argparse.ArgumentParser(
        description="F250 Metric 3.10 FC-only steady-state test.")
    parser.add_argument("--run-dir", default="")
    parser.add_argument("--run-label", default="")
    parser.add_argument("--summary-json", default="")
    parser.add_argument("--replay-summary-json", default="",
                        help="Read an existing FC summary JSON and print a screenshot-friendly terminal summary without running ROS.")
    parser.add_argument("--replay-output-log", default="",
                        help="Optional output log for --replay-summary-json; defaults to stdout only.")
    parser.add_argument("--samples-csv", default="")
    parser.add_argument("--phase-csv", default="")
    parser.add_argument("--display-log", default="")
    parser.add_argument("--geometry-audit-json", default="")
    parser.add_argument("--decagon-csv", default="")
    parser.add_argument("--p0-status", default="")
    parser.add_argument("--p0-run-dir", default="")
    parser.add_argument("--route-waypoints-csv", default="",
                        help="Current run route_waypoints.csv; first waypoint is used as FC start when available.")
    parser.add_argument("--hover-target", type=parse_hover_target, default=DEFAULT_HOVER)
    parser.add_argument("--map-authority", default=DEFAULT_MAP_AUTHORITY)
    parser.add_argument("--dry-run", action="store_true",
                        help="Generate synthetic structured outputs without connecting to ROS.")
    parser.add_argument("--geometry-check", action="store_true",
                        help="Write geometry audit and structured outputs without connecting to ROS.")
    parser.add_argument("--speed-only", action="store_true",
                        help="Run only the ten 2.0 m/s velocity windows and return to P0; skip position and yaw phases.")
    parser.add_argument("--odom-topic", default="/mavros/local_position/odom")
    parser.add_argument("--command-topic", default="/planning/pos_cmd")
    parser.add_argument("--setpoint-topic", default="/mavros/setpoint_position/local")
    parser.add_argument("--velocity-setpoint-topic", default="/mavros/setpoint_raw/local")
    parser.add_argument("--control-mode-topic", default="/maritime/fc_control_mode")
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--rate-hz", type=positive_float, default=10.0)
    parser.add_argument("--synthetic-rate-hz", type=positive_float, default=10.0)
    parser.add_argument("--odom-timeout-sec", type=positive_duration, default=10.0)
    parser.add_argument("--prehold-sec", type=positive_duration, default=3.0)
    parser.add_argument("--position-hold-sec", type=positive_duration, default=18.0)
    parser.add_argument("--yaw-hold-sec", type=positive_duration, default=16.0)
    parser.add_argument("--return-hold-sec", type=positive_duration, default=3.0)
    parser.add_argument("--final-hold-sec", type=positive_duration, default=5.0)
    parser.add_argument("--final-command-sec", type=float, default=2.0)
    parser.add_argument("--eval-window-sec", type=positive_duration, default=5.0)
    parser.add_argument("--stationarity-window-sec", type=positive_duration, default=3.0)
    parser.add_argument("--outline-stationarity-window-sec", type=positive_duration, default=1.0)
    parser.add_argument("--outline-velocity-tolerance-ratio", type=positive_float, default=0.02)
    parser.add_argument("--outline-eval-window-sec", type=positive_duration, default=5.0)
    parser.add_argument("--settle-search-step-sec", type=positive_duration, default=0.5)
    parser.add_argument("--min-window-samples", type=int, default=8)
    parser.add_argument("--position-settle-search-start-sec", type=float, default=7.0)
    parser.add_argument("--velocity-settle-search-start-sec", type=float, default=9.0)
    parser.add_argument("--yaw-settle-search-start-sec", type=float, default=5.0)
    parser.add_argument("--position-stationary-speed-mean-mps", type=positive_float, default=0.18)
    parser.add_argument("--position-stationary-std-m", type=positive_float, default=0.30)
    parser.add_argument("--position-stationary-slope-mps", type=positive_float, default=0.08)
    parser.add_argument("--velocity-stationary-std-mps", type=positive_float, default=0.45)
    parser.add_argument("--velocity-stationary-speed-std-mps", type=positive_float, default=0.35)
    parser.add_argument("--velocity-stationary-slope-mps2", type=positive_float, default=0.18)
    parser.add_argument("--velocity-parallel-error-mean-mps", type=positive_float, default=0.08)
    parser.add_argument("--velocity-parallel-error-max-mps", type=positive_float, default=0.16)
    parser.add_argument("--velocity-cross-speed-mean-mps", type=positive_float, default=0.18)
    parser.add_argument("--velocity-cross-speed-max-mps", type=positive_float, default=0.35)
    parser.add_argument("--velocity-cross-track-max-m", type=positive_float, default=2.50)
    parser.add_argument("--velocity-start-margin-m", type=positive_float, default=6.0)
    parser.add_argument("--yaw-stationary-std-rad", type=positive_float, default=0.08)
    parser.add_argument("--yaw-stationary-slope-radps", type=positive_float, default=0.04)
    parser.add_argument("--yaw-stationary-rate-mean-radps", type=positive_float, default=0.08)
    parser.add_argument("--velocity-speeds-mps", type=parse_float_list, default=[2.0])
    parser.add_argument("--velocity-lengths-m", type=parse_float_list, default=[60.0])
    parser.add_argument("--velocity-command-gain", type=positive_float, default=1.07)
    parser.add_argument("--velocity-prealign-sec", type=float, default=5.0)
    parser.add_argument("--velocity-prealign-stable-sec", type=float, default=3.0)
    parser.add_argument("--velocity-prealign-timeout-sec", type=float, default=15.0)
    parser.add_argument("--velocity-prealign-tolerance-deg", type=float, default=1.6)
    parser.add_argument("--velocity-interleg-hold-sec", type=float, default=0.0)
    parser.add_argument("--velocity-reset-hold-sec", type=float, default=4.0)
    parser.add_argument("--velocity-final-reset-hold-sec", type=float, default=8.0)
    parser.add_argument("--velocity-min-clearance-m", type=float, default=1.0)
    parser.add_argument("--velocity-endpoint-margin-m", type=positive_float, default=8.0)
    parser.add_argument("--velocity-length-adjust-step-m", type=positive_float, default=5.0)
    parser.add_argument("--steady-window-sec", type=positive_duration, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--velocity-speed-mps", type=positive_float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--velocity-length-m", type=positive_float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--position-radius-m", type=positive_float, default=10.0)

    parser.add_argument("--repeat-count", type=int, default=10, help=argparse.SUPPRESS)
    parser.add_argument("--formal-repeat-count", type=int, default=10, help=argparse.SUPPRESS)
    parser.add_argument("--position-step-x-m", type=float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--position-step-y-m", type=float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--velocity-step-x-mps", type=float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--velocity-step-y-mps", type=float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--yaw-step-rad", type=float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--step-direction-policy", default=None, help=argparse.SUPPRESS)
    return parser


def validate_args(args):
    if args.repeat_count != 10 or args.formal_repeat_count != 10:
        raise SystemExit("new Metric 3.10 design always uses 10 windows per component; do not override repeat count")
    legacy_args = [
        ("--position-step-x-m", args.position_step_x_m),
        ("--position-step-y-m", args.position_step_y_m),
        ("--velocity-step-x-mps", args.velocity_step_x_mps),
        ("--velocity-step-y-mps", args.velocity_step_y_mps),
        ("--yaw-step-rad", args.yaw_step_rad),
        ("--step-direction-policy", args.step_direction_policy),
    ]
    used_legacy = [name for name, value in legacy_args if value is not None]
    if used_legacy:
        raise SystemExit("legacy one-step arguments are retired for the new design: %s" % ", ".join(used_legacy))
    if args.velocity_speed_mps is not None or args.velocity_length_m is not None or args.steady_window_sec is not None:
        raise SystemExit(
            "--velocity-speed-mps, --velocity-length-m, and --steady-window-sec are retired; "
            "use --velocity-speeds-mps, --velocity-lengths-m, --eval-window-sec, and settled search args")
    if args.final_command_sec < 0.0:
        raise SystemExit("--final-command-sec must be >= 0")
    if args.min_window_samples <= 1:
        raise SystemExit("--min-window-samples must be > 1")
    if args.position_settle_search_start_sec < 0.0 or args.velocity_settle_search_start_sec < 0.0 or args.yaw_settle_search_start_sec < 0.0:
        raise SystemExit("settle search starts must be >= 0")
    if args.outline_velocity_tolerance_ratio <= 0.0:
        raise SystemExit("--outline-velocity-tolerance-ratio must be > 0")
    if args.velocity_prealign_sec < 0.0:
        raise SystemExit("--velocity-prealign-sec must be >= 0")
    if args.velocity_prealign_stable_sec < 0.0:
        raise SystemExit("--velocity-prealign-stable-sec must be >= 0")
    if args.velocity_prealign_timeout_sec < 0.0:
        raise SystemExit("--velocity-prealign-timeout-sec must be >= 0")
    if args.velocity_prealign_tolerance_deg <= 0.0:
        raise SystemExit("--velocity-prealign-tolerance-deg must be > 0")
    if args.velocity_interleg_hold_sec < 0.0:
        raise SystemExit("--velocity-interleg-hold-sec must be >= 0")
    if args.velocity_reset_hold_sec < 0.0:
        raise SystemExit("--velocity-reset-hold-sec must be >= 0")
    if args.velocity_final_reset_hold_sec < 0.0:
        raise SystemExit("--velocity-final-reset-hold-sec must be >= 0")
    required_position = args.position_settle_search_start_sec + args.stationarity_window_sec + args.eval_window_sec
    required_yaw = args.yaw_settle_search_start_sec + args.stationarity_window_sec + args.eval_window_sec
    if args.position_hold_sec < required_position:
        raise SystemExit("--position-hold-sec must fit settle search + stationarity + eval windows")
    if args.yaw_hold_sec < required_yaw:
        raise SystemExit("--yaw-hold-sec must fit settle search + stationarity + eval windows")
    if len(args.velocity_speeds_mps) != len(args.velocity_lengths_m):
        raise SystemExit("--velocity-speeds-mps and --velocity-lengths-m must have the same count")
    if len(args.velocity_speeds_mps) != 1 or abs(float(args.velocity_speeds_mps[0]) - 2.0) > 1e-9:
        raise SystemExit("FC Metric 3.10 now uses only the 2.0 m/s velocity test")
    for speed, length in zip(args.velocity_speeds_mps, args.velocity_lengths_m):
        required_length = minimum_velocity_length(speed, args)
        if length < required_length:
            # build_geometry will auto-extend, but flag the policy in stdout by keeping the adjusted field.
            continue


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.replay_summary_json:
        summary = read_json(args.replay_summary_json)
        text = terminal_summary_from_document(summary, args.replay_summary_json)
        if args.replay_output_log:
            os.makedirs(os.path.dirname(os.path.abspath(args.replay_output_log)), exist_ok=True)
            with open(args.replay_output_log, "w", encoding="utf-8") as handle:
                handle.write(strip_ansi(text).rstrip() + "\nOVER\n")
        print(text, flush=True)
        print("OVER", flush=True)
        return 0
    if not args.run_dir:
        parser.error("--run-dir is required unless --replay-summary-json is used")
    validate_args(args)
    os.makedirs(args.run_dir, exist_ok=True)
    paths = output_paths(args)
    geometry = build_geometry(args)
    geometry["speed_only"] = bool(args.speed_only)
    write_geometry_outputs(paths, geometry)
    if not args.dry_run and not args.geometry_check and not geometry_is_safe_for_formal(geometry, args):
        raise SystemExit("FC 3.10 geometry is not safe for formal run; inspect %s" % paths["geometry_audit_json"])
    p0 = geometry["route_start_source"]
    hover = (float(p0["x"]), float(p0["y"]), float(p0["z"]), float(p0["yaw_rad"]))
    phases = build_phases(args, geometry)
    records = planned_phase_records(phases)
    p0_status_values = read_status_env(args.p0_status)
    display = Display(paths["terminal_display_log"])
    try:
        display.write("========== F250 FC 3.10 Metrics ==========")
        display.write("任务状态: 启动中")
        display.write("测试内容: 控制稳定误差")
        display.write("测试指标: 速度-only" if args.speed_only else "测试指标: 速度 / 位置 / 偏航")
        display.write("速度测试: 10 次 AB/BA 窗口" if args.speed_only else "每项测试: 10 个测试窗口")
        display.write("")
        display.write("任务状态: 检查环境")
        display.write("检查结果: 通过")
        if args.geometry_check:
            samples = []
            run_state = "geometry_check_complete"
            exit_code = 0
        elif args.dry_run:
            samples = generate_synthetic_samples(phases, records, hover, args)
            emit_progress_from_samples(display, records, samples, hover, args, geometry)
            run_state = "dry_run_complete"
            exit_code = 0
        else:
            runner = LiveRunner(args, hover, records, display=display, geometry=geometry)
            samples, run_state, exit_code = runner.run()
        summary = write_outputs(args, hover, geometry, samples, records, paths, p0_status_values, run_state)
        if (
            not args.dry_run
            and not args.geometry_check
            and exit_code == 0
            and not summary["errors_percent"].get("all_metric_windows_settled", False)
        ):
            run_state = "failed_not_settled"
            summary = write_outputs(
                args, hover, geometry, samples, records, paths, p0_status_values, run_state)
            exit_code = 3
        display.write("\n" + terminal_summary(summary["errors_percent"]))
        display.over()
        return exit_code
    finally:
        display.close()


if __name__ == "__main__":
    sys.exit(main())
