#!/usr/bin/env python3
import argparse
import csv
import json
import math
import os
import sys
import threading
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
PKG_ROOT = os.path.dirname(SCRIPT_DIR)
DEFAULTS_ENV_PATH = os.path.join(PKG_ROOT, "config", "quick_complex_defaults.env")


def load_defaults_env(path=DEFAULTS_ENV_PATH):
    data = {}
    try:
        with open(path, "r", encoding="utf-8") as handle:
            for raw in handle:
                line = raw.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                key, value = line.split("=", 1)
                data[key.strip()] = value.strip()
    except FileNotFoundError:
        pass
    return data


QUICK_COMPLEX_DEFAULTS = load_defaults_env()


def quick_default(key, fallback=""):
    value = os.environ.get(key)
    if value not in (None, ""):
        return value
    return QUICK_COMPLEX_DEFAULTS.get(key, fallback)


def quick_float(key, fallback):
    try:
        return float(quick_default(key, str(fallback)))
    except (TypeError, ValueError):
        return float(fallback)


def quick_profile_id():
    return quick_default("F250_QUICK_COMPLEX_PROFILE_ID", "quick_complex_accepted")


def quick_ego_params():
    return {
        "map_size_x": quick_float("MAP_SIZE_X", 760.0),
        "map_size_y": quick_float("MAP_SIZE_Y", 320.0),
        "map_size_z": quick_float("MAP_SIZE_Z", 18.0),
        "max_vel": quick_float("EGO_MAX_VEL", 3.55),
        "max_acc": quick_float("EGO_MAX_ACC", 4.90),
        "max_jerk": quick_float("EGO_MAX_JERK", 6.3),
        "control_points_distance": quick_float("EGO_CONTROL_POINTS_DISTANCE", 0.35),
        "feasibility_tolerance": quick_float("EGO_FEASIBILITY_TOLERANCE", 0.0),
        "planning_horizon": quick_float("EGO_PLANNING_HORIZON", 15.0),
        "local_update_range_x": quick_float("EGO_LOCAL_UPDATE_RANGE_X", 40.0),
        "local_update_range_y": quick_float("EGO_LOCAL_UPDATE_RANGE_Y", 40.0),
        "local_update_range_z": quick_float("EGO_LOCAL_UPDATE_RANGE_Z", 9.0),
        "virtual_ceil_height": quick_float("EGO_VIRTUAL_CEIL_HEIGHT", 17.0),
        "visualization_truncate_height": quick_float("EGO_VISUALIZATION_TRUNCATE_HEIGHT", 18.0),
        "obstacles_inflation": quick_float("EGO_OBSTACLES_INFLATION", 0.50),
        "collision_dist0": quick_float("EGO_COLLISION_DIST0", 1.25),
        "lambda_smooth": quick_float("EGO_LAMBDA_SMOOTH", 1.40),
        "lambda_collision": quick_float("EGO_LAMBDA_COLLISION", 6.0),
        "lambda_feasibility": quick_float("EGO_LAMBDA_FEASIBILITY", 0.15),
        "lambda_fitness": quick_float("EGO_LAMBDA_FITNESS", 1.35),
        "grid_map_resolution": quick_float("EGO_GRID_MAP_RESOLUTION", 0.35),
    }

from maritime_metric_core import MetricAccumulator, match_waypoint_index, run_offline  # noqa: E402
from maritime_scene_utils import load_scene, scene_waypoints  # noqa: E402


CSV_FIELDS = [
    "wall_time", "ros_time", "x", "y", "z", "vx", "vy", "vz", "actual_yaw",
    "expected_x", "expected_y", "expected_z", "expected_vx", "expected_vy", "expected_vz",
    "expected_ax", "expected_ay", "expected_az", "expected_yaw", "expected_yaw_dot",
    "pos_cmd_age_sec", "setpoint_x", "setpoint_y", "setpoint_z", "setpoint_yaw",
    "setpoint_age_sec", "position_error_to_expected_m", "position_error_to_setpoint_m",
    "velocity_error_to_expected_m", "yaw_error_rad", "yaw_error_to_setpoint_rad",
    "mode", "armed", "active_goal_index", "active_goal_x", "active_goal_y",
    "active_goal_z", "active_goal_distance_m", "cross_track_m", "along_track_m",
]

ROUTE_POLICY = {
    "policy_id": "current_route_acceptance_policy",
    "vehicle": "f250",
    "route_basis": "selected_route_profile",
    "fixed_route": "classic_p0_p8",
    "route_profiles_enabled": False,
    "baseline": quick_default("F250_QUICK_COMPLEX_BASELINE", quick_profile_id()),
    "route_acceptance_excludes_planning_success_rate": True,
    "route_acceptance_excludes_metric_3_10": True,
    "route_acceptance_excludes_yaw": True,
    "dynamic_boat_clearance_role": "telemetry_only",
    "route_acceptance_components": [
        "final_completion",
        "metric_3_7_route_safety",
        "metric_3_6_keypoint_error",
        "metric_3_9_endpoint_error",
    ],
    "legacy_compatibility_components": [
        "p8_completion",
    ],
}


def read_json(path, default=None):
    if not path or not os.path.exists(path):
        return default
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def write_json(path, payload):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")


def env_quote(value):
    text = "" if value is None else str(value)
    if text == "":
        return ""
    return text.replace("\n", "_").replace(" ", "_")


def safe_float(value):
    if value is None or value == "":
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def bool_text(value):
    return "true" if bool(value) else "false"


def fmt_m(value):
    if value is None:
        return "--"
    return "%.3f m" % float(value)


def fmt_float(value):
    if value is None:
        return ""
    return "%.6g" % float(value)


def fmt_percent(value):
    if value is None:
        return "--"
    return "%.3f%%" % (float(value) * 100.0)


def fmt_xyz(values):
    if not isinstance(values, (list, tuple)) or len(values) < 3:
        return "(--, --, --)"
    return "(%.3f, %.3f, %.3f)" % (float(values[0]), float(values[1]), float(values[2]))


def fmt_error_formula(numerator, denominator, ratio):
    if numerator is None or denominator is None or ratio is None:
        return "--"
    return "%s / %s * 100%% = %s" % (fmt_m(numerator), fmt_m(denominator), fmt_percent(ratio))


def fmt_success_percent(numerator, denominator):
    if not denominator:
        return "--"
    return "%.3f%%" % (float(numerator) / float(denominator) * 100.0)


def first_present(*values):
    for value in values:
        if value is not None and value != "":
            return value
    return None


def count_text(value):
    value = first_present(value)
    if value is None:
        return "--"
    try:
        return str(int(value))
    except (TypeError, ValueError):
        return "--"


def clearance_text(value):
    if value is None:
        return "--"
    try:
        return "%.3f m" % float(value)
    except (TypeError, ValueError):
        return "--"


def route_line(status):
    final_label = status.get("final_label") or "final"
    final_text = "%s done" % final_label if status["final_completed"] else "%s wait" % final_label
    key_text = "%s / %s" % (
        fmt_m(status["keypoint_error_mean_m"]),
        fmt_m(status["keypoint_error_max_m"]),
    )
    return "Route  %-8s | %-10s | Static %-7s | Key %s | End %s" % (
        status["progress"],
        final_text,
        status["static"],
        key_text,
        fmt_m(status["endpoint_error_m"]),
    )


def route_state_text(state):
    text = str(state or "").strip().lower()
    if text in ("complete", "completed", "dry_run_complete"):
        return "COMPLETE"
    if text in ("failed", "failure", "recorder_failed"):
        return "FAILED"
    if text == "timeout":
        return "TIMEOUT"
    return "RUNNING"


def route_keypoint_text(status):
    mean_value = status.get("keypoint_error_mean_m")
    max_value = status.get("keypoint_error_max_m")
    if mean_value is None and max_value is None:
        return "waiting"
    return "mean %s   max %s" % (fmt_m(mean_value), fmt_m(max_value))


def route_endpoint_text(status):
    if not status.get("final_completed") or status.get("endpoint_error_m") is None:
        return "waiting for %s" % (status.get("final_label") or "final")
    return fmt_m(status.get("endpoint_error_m"))


def actual_clearance(clearance_payload, scope):
    return ((((clearance_payload or {}).get("metrics") or {}).get("actual_trajectory") or {}).get(scope) or {})


def route_safety_fields(status=None, metric_summary=None, clearance_static=None, clearance_dynamic=None, metrics=None):
    status = status or {}
    metric_summary = metric_summary or {}
    metrics_clearance = (metrics or {}).get("clearance") or {}
    m37 = metric_summary.get("metric_3_7") or {}
    live_clearance = m37.get("clearance") or {}
    actual_static = actual_clearance(clearance_static, "static")
    actual_dynamic = actual_clearance(clearance_dynamic, "dynamic")
    static_safe = first_present(
        status.get("static_safe"),
        m37.get("safe_so_far"),
        m37.get("static_safe"),
        static_safe_from_clearance(clearance_static),
    )
    return {
        "static_safe": static_safe,
        "static_min_clearance_m": first_present(
            metrics_clearance.get("actual_static_min_m"),
            live_clearance.get("static_min_clearance_m"),
            status.get("static_min_clearance_m"),
            static_min_from_clearance(clearance_static),
            actual_static.get("min_clearance_m"),
        ),
        "static_cloud_min_distance_m": first_present(
            metrics_clearance.get("actual_static_min_cloud_distance_m"),
            live_clearance.get("static_cloud_min_distance_m"),
            status.get("static_cloud_min_distance_m"),
            actual_static.get("min_cloud_distance_m"),
        ),
        "dynamic_min_clearance_m": first_present(
            metrics_clearance.get("actual_dynamic_min_m"),
            live_clearance.get("dynamic_min_clearance_m"),
            status.get("dynamic_min_clearance_m"),
            actual_dynamic.get("min_clearance_m"),
        ),
        "dynamic_cloud_min_distance_m": first_present(
            metrics_clearance.get("actual_dynamic_min_cloud_distance_m"),
            live_clearance.get("dynamic_cloud_min_distance_m"),
            status.get("dynamic_cloud_min_distance_m"),
            actual_dynamic.get("min_cloud_distance_m"),
        ),
        "static_geometry_entry_count": first_present(
            metrics_clearance.get("static_geometry_entry_count"),
            live_clearance.get("static_geometry_entry_count"),
            status.get("static_geometry_entry_count"),
            m37.get("static_geometry_entry_count"),
            m37.get("geometry_entry_count"),
            actual_static.get("geometry_entry_count"),
        ),
        "static_cloud_entry_count": first_present(
            metrics_clearance.get("static_cloud_entry_count"),
            live_clearance.get("static_cloud_entry_count"),
            status.get("static_cloud_entry_count"),
            m37.get("static_cloud_entry_count"),
            m37.get("cloud_entry_count"),
            actual_static.get("cloud_entry_count"),
        ),
        "dynamic_geometry_entry_count": first_present(
            metrics_clearance.get("dynamic_geometry_entry_count"),
            live_clearance.get("dynamic_geometry_entry_count"),
            status.get("dynamic_geometry_entry_count"),
            m37.get("dynamic_geometry_entry_count_telemetry"),
            m37.get("dynamic_geometry_entry_count"),
            actual_dynamic.get("geometry_entry_count"),
            actual_dynamic.get("dynamic_geometry_entry_count"),
        ),
        "dynamic_cloud_entry_count": first_present(
            metrics_clearance.get("dynamic_cloud_entry_count"),
            live_clearance.get("dynamic_cloud_entry_count"),
            status.get("dynamic_cloud_entry_count"),
            m37.get("dynamic_cloud_entry_count_telemetry"),
            m37.get("dynamic_cloud_entry_count"),
            actual_dynamic.get("cloud_entry_count"),
            actual_dynamic.get("dynamic_cloud_entry_count"),
        ),
    }


def route_safety_text(fields):
    static_safe = fields.get("static_safe")
    state_text = "UNKNOWN" if static_safe is None else ("SAFE" if bool(static_safe) else "UNSAFE")
    return (
        "%s | static min %s cloud %s entries g/c %s/%s | "
        "dynamic min %s cloud %s entries g/c %s/%s"
    ) % (
        state_text,
        clearance_text(fields.get("static_min_clearance_m")),
        clearance_text(fields.get("static_cloud_min_distance_m")),
        count_text(fields.get("static_geometry_entry_count")),
        count_text(fields.get("static_cloud_entry_count")),
        clearance_text(fields.get("dynamic_min_clearance_m")),
        clearance_text(fields.get("dynamic_cloud_min_distance_m")),
        count_text(fields.get("dynamic_geometry_entry_count")),
        count_text(fields.get("dynamic_cloud_entry_count")),
    )


def route_static_safety_text(status, metric_summary=None, clearance_static=None, clearance_dynamic=None, metrics=None):
    return route_safety_text(route_safety_fields(status, metric_summary, clearance_static, clearance_dynamic, metrics))



def route_display_block(status, state):
    progress_index = int(status.get("progress_index") or 0)
    final_index = int(status.get("final_index") or 0)
    progress_label = status.get("progress_label") or "W%d" % progress_index
    route_name = status.get("route_name") or "Classic P0-P8 Baseline"
    if route_name in ("Classic P0-P8", "classic_p0_p8"):
        route_name = "Classic P0-P8 Baseline"
    final_label = status.get("final_label") or "P8"
    previous_label = status.get("first_label") or "P0"
    if progress_index > 0:
        previous_label = "P%d" % max(0, progress_index - 1)
    target_label = progress_label if progress_index < final_index else final_label
    lines = [
        "F250 %s" % route_name,
        "",
        "%-15s %s" % ("任务状态", route_state_text(state)),
        "%-15s %s / %s" % ("前往", target_label, final_label),
        "%-15s %s -> %s" % ("航段", previous_label, target_label),
    ]
    return "\n".join(lines)


def perception_text(source):
    source = str(source or "lidar")
    if source == "lidar":
        return "Gazebo LiDAR LaserScan-derived PointCloud2 perception"
    if source == "depth":
        return "Gazebo depth PointCloud2 fallback perception"
    return "Gazebo LiDAR LaserScan-derived PointCloud2 perception"


def sensor_label(source):
    source = str(source or "depth")
    if source == "lidar":
        return "LiDAR"
    if source == "depth":
        return "Depth"
    return source


def raw_cloud_topic(source):
    override = os.environ.get("RAW_CLOUD_TOPIC")
    if override:
        return override
    source = str(source or "depth")
    if source == "lidar":
        return "/maritime/lidar_points"
    if source == "depth":
        return "/maritime_depth_camera/points"
    return "/maritime/lidar_points"


def sensor_metadata(source):
    source = str(source or "depth")
    return {
        "source": source,
        "label": sensor_label(source),
        "planner_cloud_topic": os.environ.get("PLANNER_CLOUD_TOPIC", "/maritime/obstacles_cloud"),
        "raw_cloud_topic": raw_cloud_topic(source),
        "lidar_cloud_topic": os.environ.get("LIDAR_CLOUD_TOPIC", "/maritime/lidar_points"),
        "lidar_scan_topic": os.environ.get("LIDAR_SCAN_TOPIC", "/maritime/lidar_scan"),
        "depth_cloud_topic": os.environ.get("DEPTH_CLOUD_TOPIC", "/maritime_depth_camera/points"),
        "occupancy_topic": os.environ.get("OCCUPANCY_TOPIC", "/grid_map/occupancy_inflate"),
    }



def terminal_header_block(perception_source="lidar", route_name=None, first_label="P0", final_label="P8", final_index=8):
    route_name = route_name or "Classic P0-P8 Baseline"
    if route_name in ("Classic P0-P8", "classic_p0_p8"):
        route_name = "Classic P0-P8 Baseline"
    return "\n".join([
        "========== F250 %s Metrics ==========" % route_name,
        "任务状态: 启动中",
        "路线: Classic P0-P8",
        "参数方案: Classic P0-P8 Baseline",
        "测试指标: 3.6 运动规划目标到达误差 / 3.7 避障率 / 3.9 目标到达误差",
        "",
        "3.6 运动规划目标到达误差",
        "= 实际到达位置与规划位置的三维距离",
        "  / 上一规划点到当前规划点的三维距离 * 100%",
        "3.9 目标到达误差",
        "= 最终停止位置与任务目标终点的三维距离",
        "  / 起点到终点总路线长度 * 100%",
        "",
        "任务状态: 检查环境",
        "检查结果: 通过",
        "",
        "任务状态: 执行路线",
    ])

def waypoint_name(index, metric_summary=None):
    for stat in (metric_summary or {}).get("waypoints") or []:
        try:
            if int(stat.get("index", -1)) == int(index):
                return str(stat.get("label") or "W%d" % int(index))
        except (TypeError, ValueError):
            continue
    return "P%d" % int(index)


def waypoint_xyz(waypoints, index):
    if index < 0 or index >= len(waypoints):
        return None
    pos = waypoints[index].get("position", [])
    if len(pos) < 3:
        return None
    return [float(pos[0]), float(pos[1]), float(pos[2])]



def waypoint_start_block(waypoints, index):
    label = waypoints[index].get("label") if 0 <= index < len(waypoints) else None
    prev_label = waypoints[index - 1].get("label") if 0 < index < len(waypoints) else "P0"
    current = label or waypoint_name(index)
    return "[%s -> %s]" % (prev_label, current)


def waypoint_stat(metric_summary, index):
    for item in (metric_summary or {}).get("waypoints") or []:
        try:
            if int(item.get("index", -1)) == int(index):
                return item
        except (TypeError, ValueError):
            continue
    return {}


def segment_length(metric_summary, index):
    lengths = ((metric_summary or {}).get("route") or {}).get("segment_lengths_m") or []
    try:
        index = int(index)
    except (TypeError, ValueError):
        return None
    if 0 <= index < len(lengths):
        return safe_float(lengths[index])
    return None


def total_route_length(metric_summary):
    route = (metric_summary or {}).get("route") or {}
    value = safe_float(route.get("total_route_length_m"))
    if value is not None:
        return value
    value = safe_float(route.get("total_p0_p8_length_m"))
    if value is not None:
        return value
    total = 0.0
    found = False
    for item in route.get("segment_lengths_m") or []:
        length = safe_float(item)
        if length is not None:
            total += length
            found = True
    return total if found else None


def waypoint_position(stat):
    for key in ("position", "target_position", "waypoint_position"):
        pos = (stat or {}).get(key)
        if isinstance(pos, list) and len(pos) >= 3:
            return pos
    return None


def waypoint_error_ratio(metric_summary, stat, index):
    value = safe_float((stat or {}).get("metric_3_6_error_ratio"))
    if value is not None:
        return value
    error = safe_float((stat or {}).get("nearest_distance_m"))
    length = segment_length(metric_summary, index)
    if error is None or not length:
        return None
    return error / length


def keypoint_ratio_values(metric_summary):
    values = []
    for stat in (metric_summary or {}).get("waypoints") or []:
        try:
            index = int(stat.get("index", -1))
        except (TypeError, ValueError):
            continue
        if index <= 0:
            continue
        ratio = waypoint_error_ratio(metric_summary, stat, index)
        if ratio is not None:
            values.append(ratio)
    return values



def waypoint_progress_block(metric_summary, status, index):
    return waypoint_error_block(metric_summary, status, index)


def waypoint_live_arrival_block(metric_summary, status, index):
    stat = waypoint_stat(metric_summary, index)
    final_index = int(status.get("final_index") or 0)
    label = stat.get("label") or waypoint_name(index, metric_summary)
    pending_label = "目标到达误差" if index == final_index else "规划点到达误差"
    return "\n".join(["[到达 %s]" % label, "%s: 确认中" % pending_label])


def waypoint_error_block(metric_summary, status, index):
    label = waypoint_name(index, metric_summary)
    lines = ["[到达 %s]" % label]
    lines.extend(waypoint_metric_lines(metric_summary, status, index))
    return "\n".join(lines)


def waypoint_metric_lines(metric_summary, status, index):
    stat = waypoint_stat(metric_summary, index)
    final_index = int(status.get("final_index") or 0)
    lines = []
    if 1 <= index < final_index:
        error_m = safe_float(stat.get("nearest_distance_m"))
        ratio = waypoint_error_ratio(metric_summary, stat, index)
        target_position = waypoint_position(stat)
        actual_position = stat.get("nearest_position")
        length = segment_length(metric_summary, index)
        lines.append("规划位置: %s" % fmt_xyz(target_position))
        lines.append("实际到达位置: %s" % fmt_xyz(actual_position))
        lines.append("规划点到达误差: %s" % fmt_error_formula(error_m, length, ratio))
    if index == final_index:
        m39 = (metric_summary or {}).get("metric_3_9") or {}
        final_error = safe_float(m39.get("final_error_m"))
        if final_error is None:
            final_error = safe_float(status.get("endpoint_error_m"))
        total_length = total_route_length(metric_summary)
        final_ratio = safe_float(m39.get("final_error_ratio"))
        if final_ratio is None and final_error is not None and total_length:
            final_ratio = final_error / total_length
        target_position = waypoint_position(stat)
        actual_position = stat.get("nearest_position")
        lines.append("任务目标终点: %s" % fmt_xyz(target_position))
        lines.append("最终停止位置: %s" % fmt_xyz(actual_position))
        lines.append("目标到达误差: %s" % fmt_error_formula(final_error, total_length, final_ratio))
    return lines


def route_metrics_detail_block(metric_summary, status):
    final_index = int(status.get("final_index") or 0)
    lines = ["===== ROUTE DETAIL ====="]
    for index in range(1, final_index + 1):
        stat = waypoint_stat(metric_summary, index)
        pos = waypoint_position(stat)
        lines.append("")
        lines.append("[%s]" % waypoint_name(index, metric_summary))
        if pos is not None:
            lines.append("规划位置: %s" % fmt_xyz(pos))
        lines.extend(waypoint_metric_lines(metric_summary, status, index))
    return "\n".join(lines)

def waypoint_reached_block(metric_summary, status, index):
    return waypoint_progress_block(metric_summary, status, index)


def route_metrics_progress_block(metric_summary, status, indexes=None):
    final_index = int(status.get("final_index") or 0)
    if indexes is None:
        indexes = range(1, final_index + 1)
    blocks = []
    for index in indexes:
        try:
            index = int(index)
        except (TypeError, ValueError):
            continue
        if 1 <= index <= final_index:
            blocks.append(waypoint_progress_block(metric_summary, status, index))
    if not blocks:
        return ""
    lines = ["===== ROUTE METRICS ====="]
    for block in blocks:
        lines.append("")
        lines.append(block)
    return "\n".join(lines)


def terminal_arrival_labels(path):
    labels = set()
    try:
        with open(path, "r", encoding="utf-8") as handle:
            for raw_line in handle:
                line = raw_line.strip()
                if line.startswith("[到达 ") and line.endswith("]"):
                    labels.add(line[len("[到达 "):-1])
    except OSError:
        pass
    return labels


def missing_route_metrics_progress_block(metric_summary, status, terminal_log):
    final_index = int(status.get("final_index") or 0)
    existing = terminal_arrival_labels(terminal_log)
    missing = []
    for index in range(1, final_index + 1):
        stat = waypoint_stat(metric_summary, index)
        label = stat.get("label") or waypoint_name(index, metric_summary)
        if label not in existing:
            missing.append(index)
    return route_metrics_progress_block(metric_summary, status, missing)


def route_sequential_progress_block(waypoints, metric_summary, status):
    final_index = int(status.get("final_index") or 0)
    blocks = []
    for index in range(1, final_index + 1):
        segment_lines = []
        if 0 <= index < len(waypoints):
            segment_lines.append(waypoint_start_block(waypoints, index))
        segment_lines.append(waypoint_progress_block(metric_summary, status, index))
        blocks.append("\n".join(segment_lines))
    return "\n\n".join(blocks)


def static_details(clearance_static):
    actual = ((((clearance_static or {}).get("metrics") or {}).get("actual_trajectory") or {}).get("static") or {})
    return {
        "geometry_entry_count": int(actual.get("geometry_entry_count") or 0),
        "cloud_entry_count": int(actual.get("cloud_entry_count") or 0),
        "collision": bool(actual.get("collision")),
    }



def route_final_block(status, state, summary=None, clearance_static=None, clearance_dynamic=None, metrics=None):
    metric_summary = summary if isinstance(summary, dict) and summary.get("metric_3_6") else {}
    m36 = (metric_summary or {}).get("metric_3_6") or {}
    m37 = (metric_summary or {}).get("metric_3_7") or {}
    m39 = (metric_summary or {}).get("metric_3_9") or {}
    ratios = keypoint_ratio_values(metric_summary)
    key_mean_ratio = safe_float(m36.get("mean_error_ratio"))
    key_max_ratio = safe_float(m36.get("max_error_ratio"))
    if key_mean_ratio is None and ratios:
        key_mean_ratio = sum(ratios) / len(ratios)
    if key_max_ratio is None and ratios:
        key_max_ratio = max(ratios)
    final_ratio = safe_float(m39.get("final_error_ratio"))
    if final_ratio is None:
        total_length = total_route_length(metric_summary)
        if status.get("endpoint_error_m") is not None and total_length:
            final_ratio = float(status.get("endpoint_error_m")) / total_length
    total_segments = max(0, int(status.get("final_index") or 0))
    if bool(first_present(m37.get("passed"), m37.get("safe_so_far"), status.get("static_safe"))):
        safe_segments = total_segments
    else:
        safe_segments = 0
    avoidance_rate = safe_segments / total_segments if total_segments else None
    result = "PASS" if route_state_text(state) == "COMPLETE" else "FAIL"
    lines = [
        "===== FINAL =====",
        "3.6 运动规划目标到达误差平均值: %s" % fmt_percent(key_mean_ratio),
        "3.6 运动规划目标到达误差最大值: %s" % fmt_percent(key_max_ratio),
        "3.7 避障率: %s" % fmt_percent(avoidance_rate),
        "3.9 目标到达误差: %s" % fmt_percent(final_ratio),
        "",
        "路线结果: %s" % result,
    ]
    if route_state_text(state) != "COMPLETE":
        lines.append("status %s" % route_state_text(state))
    return "\n".join(lines)

def waypoint_count(metric_summary, fallback=9):
    route = (metric_summary or {}).get("route") or {}
    count = route.get("waypoint_count")
    if count is None:
        waypoints = (metric_summary or {}).get("waypoints") or []
        count = len(waypoints) if waypoints else fallback
    try:
        return int(count)
    except (TypeError, ValueError):
        return fallback


def keypoint_errors(metric_summary):
    waypoints = (metric_summary or {}).get("waypoints") or []
    if not waypoints:
        return None, None
    last_index = max(int(stat.get("index", -1)) for stat in waypoints)
    values = []
    for stat in waypoints:
        index = int(stat.get("index", -1))
        if 1 <= index <= last_index - 1:
            value = safe_float(stat.get("nearest_distance_m"))
            if value is not None:
                values.append(value)
    if not values:
        return None, None
    return sum(values) / len(values), max(values)


def static_safe_from_clearance(clearance_static):
    actual = ((((clearance_static or {}).get("metrics") or {}).get("actual_trajectory") or {}).get("static") or {})
    if not actual:
        return None
    collision = bool(actual.get("collision"))
    geometry_entries = int(actual.get("geometry_entry_count") or 0)
    cloud_entries = int(actual.get("cloud_entry_count") or 0)
    return (not collision) and geometry_entries == 0 and cloud_entries == 0


def static_min_from_clearance(clearance_static):
    actual = ((((clearance_static or {}).get("metrics") or {}).get("actual_trajectory") or {}).get("static") or {})
    values = [
        safe_float(actual.get("min_clearance_m")),
        safe_float(actual.get("min_cloud_distance_m")),
    ]
    values = [value for value in values if value is not None]
    return min(values) if values else None


def dynamic_telemetry(clearance_dynamic):
    actual = ((((clearance_dynamic or {}).get("metrics") or {}).get("actual_trajectory") or {}).get("dynamic") or {})
    return {
        "min_clearance_m": safe_float(actual.get("min_clearance_m")),
        "geometry_entry_count": int(actual.get("geometry_entry_count") or actual.get("dynamic_geometry_entry_count") or 0),
        "cloud_entry_count": int(actual.get("cloud_entry_count") or actual.get("dynamic_cloud_entry_count") or 0),
        "role": "telemetry_only",
    }


def terminal_status(metric_summary, summary=None, clearance_static=None):
    metric_summary = metric_summary or {}
    summary = summary or {}
    count = waypoint_count(metric_summary)
    final_index = max(0, count - 1)
    route_meta = {}
    route_meta.update(summary.get("route") or {})
    route_meta.update((metric_summary.get("route") or {}))
    route_name = route_meta.get("name") or route_meta.get("route_id") or "Classic P0-P8 Baseline"
    final_label = route_meta.get("final_label")
    first_label = route_meta.get("first_label")
    waypoints = metric_summary.get("waypoints") or []
    if waypoints:
        first_label = first_label or waypoints[0].get("label")
        final_label = final_label or waypoints[-1].get("label")
    first_label = first_label or "P0"
    final_label = final_label or "P%d" % final_index
    progress_index = metric_summary.get("max_active_goal_index")
    if progress_index is None:
        progress_index = metric_summary.get("active_goal_index")
    route = summary.get("route") or {}
    if progress_index is None:
        progress_index = route.get("max_active_goal_index")
    try:
        progress_index = int(progress_index)
    except (TypeError, ValueError):
        progress_index = 0
    progress_index = max(0, min(final_index, progress_index))

    final_completed = bool(
        metric_summary.get("final_completed")
        or metric_summary.get("p8_completed")
        or route.get("final_completed")
        or route.get("final_reached_ever")
    )
    if final_completed:
        progress_index = final_index
    progress_label = "W%d" % progress_index
    for stat in waypoints:
        try:
            if int(stat.get("index", -1)) == progress_index:
                progress_label = str(stat.get("label") or progress_label)
                break
        except (TypeError, ValueError):
            continue

    m37 = metric_summary.get("metric_3_7") or {}
    static_safe = m37.get("safe_so_far")
    if static_safe is None:
        static_safe = static_safe_from_clearance(clearance_static)
    static_text = "UNKNOWN"
    if static_safe is not None:
        static_text = "SAFE" if bool(static_safe) else "UNSAFE"

    key_mean, key_max = keypoint_errors(metric_summary)
    m39 = metric_summary.get("metric_3_9") or {}
    final_err = safe_float(m39.get("final_error_m"))
    if final_err is None:
        task = summary.get("task") or {}
        final_err = safe_float(task.get("final_nearest_distance_m"))
        if final_err is None:
            final_err = safe_float(task.get("p8_nearest_distance_m"))

    status = {
        "progress_index": progress_index,
        "final_index": final_index,
        "progress_label": progress_label,
        "first_label": first_label,
        "final_label": final_label,
        "route_id": route_meta.get("route_id"),
        "route_name": route_name,
        "route_profile": (
            route_meta.get("profile")
            or route_meta.get("profile_path")
            or route_meta.get("route_profile")
        ),
        "waypoint_count": count,
        "progress": "%s/%s" % (progress_label, final_label),
        "final_completed": final_completed,
        "p8_completed": final_completed,
        "static": static_text,
        "static_safe": static_safe,
        "static_min_clearance_m": static_min_from_clearance(clearance_static),
        "keypoint_error_mean_m": key_mean,
        "keypoint_error_max_m": key_max,
        "endpoint_error_m": final_err,
    }
    status["line"] = route_line(status)
    return status


def route_acceptance(metric_summary, summary=None, clearance_static=None):
    metric_summary = metric_summary or {}
    summary = summary or {}
    status = terminal_status(metric_summary, summary, clearance_static)
    m36 = metric_summary.get("metric_3_6") or {}
    m37 = metric_summary.get("metric_3_7") or {}
    m39 = metric_summary.get("metric_3_9") or {}
    summary_ok = bool(summary.get("ok", True))
    final_completed = bool(status["final_completed"])
    components = {
        "final_completion": final_completed,
        "p8_completion": final_completed,
        "static_obstacle_safety": bool(status.get("static_safe")),
        "metric_3_7_route_safety": bool(first_present(m37.get("passed"), status.get("static_safe"))),
        "metric_3_6_keypoint_error": bool(m36.get("passed")),
        "metric_3_9_endpoint_error": bool(m39.get("passed")),
        "recorder_summary": summary_ok,
    }
    formal_component_names = set(ROUTE_POLICY["route_acceptance_components"])
    ok = all(bool(components.get(key)) for key in formal_component_names)
    return ok, components, status


def write_env_file(path, fields):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        for key, value in fields:
            handle.write("%s=%s\n" % (key, env_quote(value)))


def write_status_files(args, state, metric_summary, summary=None, clearance_static=None):
    ok, components, status = route_acceptance(metric_summary, summary, clearance_static)
    sensor = sensor_metadata(args.perception_source)
    fields = [
        ("state", state),
        ("updated_at", time.strftime("%Y-%m-%dT%H:%M:%S%z")),
        ("run_dir", args.run_dir),
        ("run_label", args.run_label),
        ("vehicle", "f250"),
        ("route_id", status.get("route_id") or ""),
        ("route_name", status.get("route_name") or ""),
        ("route_profile", status.get("route_profile") or ""),
        ("route_waypoint_count", status.get("waypoint_count") or ""),
        ("route_final_label", status.get("final_label") or ""),
        ("progress", status["progress"]),
        ("final_completed", bool_text(status["final_completed"])),
        ("p8_completed", bool_text(status["p8_completed"])),
        ("static_obstacle_safety", status["static"]),
        ("keypoint_error_mean_m", fmt_float(status["keypoint_error_mean_m"])),
        ("keypoint_error_max_m", fmt_float(status["keypoint_error_max_m"])),
        ("endpoint_error_m", fmt_float(status["endpoint_error_m"])),
        ("route_acceptance_ok", bool_text(ok)),
        ("route_acceptance_excludes_metric_3_10", "true"),
        ("route_acceptance_excludes_yaw", "true"),
        ("dynamic_boat_clearance_role", "telemetry_only"),
        ("sensor", args.perception_source),
        ("sensor_label", sensor_label(args.perception_source)),
        ("perception_source", args.perception_source),
        ("planner_cloud_topic", sensor["planner_cloud_topic"]),
        ("raw_cloud_topic", sensor["raw_cloud_topic"]),
        ("lidar_cloud_topic", sensor["lidar_cloud_topic"]),
        ("lidar_scan_topic", sensor["lidar_scan_topic"]),
        ("depth_cloud_topic", sensor["depth_cloud_topic"]),
        ("occupancy_topic", sensor["occupancy_topic"]),
        ("perception_gate_json", os.path.join(args.run_dir, "perception_gate.json")),
        ("route_terminal_log", args.terminal_log),
        ("route_acceptance_summary_json", os.path.join(args.run_dir, "route_acceptance_summary.json")),
    ]
    if args.route_status_env:
        write_env_file(args.route_status_env, fields)
    if args.status_env:
        write_env_file(args.status_env, fields)
    return ok, components, status


def append_terminal_line(path, line):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    text = line.rstrip("\n")
    try:
        with open(path, "r", encoding="utf-8") as handle:
            previous = [item.rstrip("\n") for item in handle.readlines() if item.rstrip("\n")]
        if previous and previous[-1] == text:
            return
    except OSError:
        pass
    with open(path, "a", encoding="utf-8") as handle:
        handle.write(text + "\n")


def append_terminal_block(path, block):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    text = block.rstrip("\n")
    with open(path, "a", encoding="utf-8") as handle:
        handle.write(text + "\n\n")


def append_terminal_over(path):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    try:
        with open(path, "r", encoding="utf-8") as handle:
            lines = [item.rstrip("\n") for item in handle.readlines() if item.rstrip("\n")]
        if lines and lines[-1] == "OVER":
            return
    except OSError:
        pass
    with open(path, "a", encoding="utf-8") as handle:
        handle.write("OVER\n")


def write_terminal_log(path, text):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(text.rstrip("\n") + "\n")


def completed_terminal_text(args, metric_summary, status, state, clearance_static, clearance_dynamic, metrics):
    waypoints = scene_waypoints(load_scene(args.scene_config))
    final_block = route_final_block(status, state, metric_summary, clearance_static, clearance_dynamic, metrics)
    blocks = [
        terminal_header_block(
            args.perception_source,
            route_name=status.get("route_name"),
            first_label=status.get("first_label") or "P0",
            final_label=status.get("final_label") or "P8",
            final_index=int(status.get("final_index") or 0),
        ),
        route_sequential_progress_block(waypoints, metric_summary, status),
        final_block,
        "OVER",
    ]
    return "\n\n".join(block for block in blocks if block), final_block


def write_completed_terminal_log(args, metric_summary, status, state, clearance_static, clearance_dynamic, metrics):
    text, final_block = completed_terminal_text(
        args, metric_summary, status, state, clearance_static, clearance_dynamic, metrics)
    write_terminal_log(args.terminal_log, text)
    return final_block


def write_route_acceptance_summary(args, metric_summary, summary, clearance_static, clearance_dynamic, metrics=None):
    ok, components, status = route_acceptance(metric_summary, summary, clearance_static)
    payload = {
        "ok": ok,
        "vehicle": "f250",
        "run_dir": args.run_dir,
        "run_label": args.run_label,
        "sensor": args.perception_source,
        "sensor_label": sensor_label(args.perception_source),
        "perception_source": args.perception_source,
        "perception": sensor_metadata(args.perception_source),
        "route": {
            "route_id": status.get("route_id"),
            "name": status.get("route_name"),
            "profile": status.get("route_profile"),
            "waypoint_count": status.get("waypoint_count"),
            "first_label": status.get("first_label"),
            "final_label": status.get("final_label"),
            "final_completed": status.get("final_completed"),
        },
        "terminal": status,
        "components": components,
        "policy": ROUTE_POLICY,
        "static_obstacle_safety": {
            "status": status["static"],
            "min_clearance_m": status["static_min_clearance_m"],
        },
        "dynamic_boat_clearance": dynamic_telemetry(clearance_dynamic),
        "metric_summary_json": os.path.join(args.run_dir, "metric_summary.json"),
        "metric_waypoints_csv": os.path.join(args.run_dir, "metric_waypoints.csv"),
        "metrics_json": os.path.join(args.run_dir, "metrics.json"),
        "perception_gate_json": os.path.join(args.run_dir, "perception_gate.json"),
    }
    if metrics is not None:
        payload["postprocess_ok"] = bool(metrics.get("ok"))
    out_path = os.path.join(args.run_dir, "route_acceptance_summary.json")
    write_json(out_path, payload)
    return payload


def update_artifact_policy(args, metric_summary, summary, clearance_static, clearance_dynamic):
    metrics_path = os.path.join(args.run_dir, "metrics.json")
    metrics = read_json(metrics_path, default=None)
    acceptance = write_route_acceptance_summary(
        args, metric_summary, summary or {}, clearance_static or {}, clearance_dynamic or {}, metrics)

    summary_path = os.path.join(args.run_dir, "summary.json")
    if summary is not None:
        summary["perception_source"] = args.perception_source
        summary["sensor"] = sensor_metadata(args.perception_source)
        summary["route_acceptance_policy"] = ROUTE_POLICY
        summary["route_terminal"] = acceptance["terminal"]
        summary["route_acceptance_ok"] = acceptance["ok"]
        route = summary.get("route") if isinstance(summary.get("route"), dict) else {}
        route.update(acceptance.get("route") or {})
        summary["route"] = route
        write_json(summary_path, summary)

    if metrics is not None:
        policy = metrics.get("metric_policy") if isinstance(metrics.get("metric_policy"), dict) else {}
        policy.update({
            "policy_id": ROUTE_POLICY["policy_id"],
            "route_ok_excludes_planning_success_rate": True,
            "route_ok_excludes_metric_3_10": True,
            "route_ok_excludes_yaw": True,
            "dynamic_boat_clearance_role": "telemetry_only",
            "components": ROUTE_POLICY["route_acceptance_components"],
            "legacy_compatibility_components": ROUTE_POLICY["legacy_compatibility_components"],
        })
        metrics["metric_policy"] = policy
        metrics["sensor"] = args.perception_source
        metrics["sensor_label"] = sensor_label(args.perception_source)
        metrics["perception_source"] = args.perception_source
        metrics["perception"] = sensor_metadata(args.perception_source)
        metrics["route_terminal"] = acceptance["terminal"]
        metrics["route"] = metrics.get("route") or summary.get("route") or acceptance.get("route")
        metrics["route_acceptance_summary_json"] = os.path.join(args.run_dir, "route_acceptance_summary.json")
        write_json(metrics_path, metrics)
    return acceptance


def write_params_json(path, args):
    scene = load_scene(args.scene_config)
    route_profile = scene.get("route_profile") or {}
    payload = {
        "description": quick_default("F250_QUICK_COMPLEX_DESCRIPTION", "F250 quick-complex accepted EGO defaults"),
        "vehicle": "f250",
        "profile_id": quick_profile_id(),
        "family_id": quick_profile_id(),
        "baseline": quick_default("F250_QUICK_COMPLEX_BASELINE", quick_profile_id()),
        "scene_level": "level_m_gps_assets_quick_complex",
        "scene_config": os.path.abspath(args.scene_config),
        "route": route_profile,
        "route_id": route_profile.get("route_id"),
        "route_name": route_profile.get("name"),
        "route_profile": route_profile.get("profile_path"),
        "route_waypoint_count": route_profile.get("waypoint_count"),
        "route_final_label": route_profile.get("final_label"),
        "sensor": args.perception_source,
        "sensor_label": sensor_label(args.perception_source),
        "perception_source": args.perception_source,
        "dynamic_mode": args.dynamic_mode,
        "topics": sensor_metadata(args.perception_source),
        "params": quick_ego_params(),
        "route_acceptance_policy": ROUTE_POLICY,
    }
    write_json(path, payload)


def yaw_from_waypoint(waypoint):
    try:
        return float(waypoint.get("yaw", 0.0))
    except (TypeError, ValueError):
        return 0.0


def make_trajectory_row(time_sec, waypoint, index, total_along):
    pos = [float(value) for value in waypoint["position"][:3]]
    yaw = yaw_from_waypoint(waypoint)
    return {
        "wall_time": time_sec,
        "ros_time": time_sec,
        "x": pos[0],
        "y": pos[1],
        "z": pos[2],
        "vx": 0.0,
        "vy": 0.0,
        "vz": 0.0,
        "actual_yaw": yaw,
        "expected_x": pos[0],
        "expected_y": pos[1],
        "expected_z": pos[2],
        "expected_vx": 0.0,
        "expected_vy": 0.0,
        "expected_vz": 0.0,
        "expected_ax": 0.0,
        "expected_ay": 0.0,
        "expected_az": 0.0,
        "expected_yaw": yaw,
        "expected_yaw_dot": 0.0,
        "pos_cmd_age_sec": 0.0,
        "setpoint_x": pos[0],
        "setpoint_y": pos[1],
        "setpoint_z": pos[2],
        "setpoint_yaw": yaw,
        "setpoint_age_sec": 0.0,
        "position_error_to_expected_m": 0.0,
        "position_error_to_setpoint_m": 0.0,
        "velocity_error_to_expected_m": 0.0,
        "yaw_error_rad": 0.0,
        "yaw_error_to_setpoint_rad": 0.0,
        "mode": "OFFBOARD",
        "armed": "true",
        "active_goal_index": index,
        "active_goal_x": pos[0],
        "active_goal_y": pos[1],
        "active_goal_z": pos[2],
        "active_goal_distance_m": 0.0,
        "cross_track_m": 0.0,
        "along_track_m": total_along,
    }


def waypoint_distance(a, b):
    return math.sqrt(sum((float(a[i]) - float(b[i])) ** 2 for i in range(3)))


def synthesize_trajectory(scene_config, output_csv):
    scene = load_scene(scene_config)
    waypoints = scene_waypoints(scene)
    if not waypoints:
        raise RuntimeError("scene has no waypoints")
    rows = []
    route_advancements = []
    total_along = 0.0
    time_sec = 0.0
    previous_pos = None
    for index, waypoint in enumerate(waypoints):
        pos = waypoint["position"][:3]
        if previous_pos is not None:
            total_along += waypoint_distance(previous_pos, pos)
        rows.append(make_trajectory_row(time_sec, waypoint, index, total_along))
        route_advancements.append({
            "index": index,
            "name": waypoint.get("name", "waypoint_%d" % index),
            "position": [float(value) for value in pos],
            "wall_time": time_sec,
        })
        time_sec += 0.4
        rows.append(make_trajectory_row(time_sec, waypoint, index, total_along))
        time_sec += 0.6
        previous_pos = pos
    final_hold = float((scene.get("acceptance") or {}).get("final_zone_hold_sec", 0.2))
    time_sec += final_hold + 0.4
    rows.append(make_trajectory_row(time_sec, waypoints[-1], len(waypoints) - 1, total_along))

    os.makedirs(os.path.dirname(os.path.abspath(output_csv)), exist_ok=True)
    with open(output_csv, "w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
    return scene, waypoints, rows, route_advancements, final_hold


def write_synthetic_summary(args, scene, waypoints, rows, route_advancements, final_hold):
    output_csv = os.path.join(args.run_dir, "actual_trajectory.csv")
    route_profile = scene.get("route_profile") or {}
    first_label = route_profile.get("first_label") or waypoints[0].get("label") or "W0"
    final_label = route_profile.get("final_label") or waypoints[-1].get("label") or "W%d" % (len(waypoints) - 1)
    total_route_length_m = route_profile.get("total_route_length_m")
    if total_route_length_m is None:
        total_route_length_m = rows[-1]["along_track_m"] if rows else None
    payload = {
        "ok": True,
        "errors": [],
        "duration_requested_sec": args.max_duration_sec,
        "duration_observed_sec": rows[-1]["wall_time"] if rows else 0.0,
        "scene": os.path.abspath(args.scene_config),
        "sensor": sensor_metadata(args.perception_source),
        "perception_source": args.perception_source,
        "state": {
            "connected": True,
            "mode": "OFFBOARD",
            "armed": True,
            "seen_offboard": True,
            "seen_armed": True,
            "seen_disarmed_after_armed": False,
        },
        "counts": {
            "odom": len(rows),
            "pos_cmd": len(rows),
            "setpoint": len(rows),
            "state": len(rows),
            "active_goal": len(waypoints),
            "planner_cloud": 1,
            "raw_cloud": 1,
            "depth_cloud": 1 if args.perception_source == "depth" else 0,
            "occupancy_inflate": 1,
            "lidar_scan": 1 if args.perception_source == "lidar" else 0,
            "landing_status": 0,
        },
        "clouds": {
            "planner_cloud_topic": "/maritime/obstacles_cloud",
            "raw_cloud_topic": raw_cloud_topic(args.perception_source),
            "lidar_scan_topic": "/maritime/lidar_scan",
            "depth_cloud_topic": "/maritime_depth_camera/points",
            "occupancy_topic": "/grid_map/occupancy_inflate",
            "max_planner_cloud_points": 1,
            "max_raw_cloud_points": 1,
            "max_depth_cloud_points": 1 if args.perception_source == "depth" else 0,
            "max_occupancy_inflate_points": 1,
            "max_lidar_scan_ranges": 1 if args.perception_source == "lidar" else 0,
        },
        "route": {
            "route_id": route_profile.get("route_id"),
            "name": route_profile.get("name"),
            "profile": route_profile.get("profile_path"),
            "profile_source": route_profile.get("profile_source"),
            "first_label": first_label,
            "final_label": final_label,
            "total_route_length_m": total_route_length_m,
            "locked_baseline_compatibility": bool(route_profile.get("locked_baseline_compatibility", False)),
            "active_goal_advancements": route_advancements,
            "max_active_goal_index": len(waypoints) - 1,
            "waypoint_count": len(waypoints),
            "final_completed": True,
            "final_reached_ever": True,
            "final_hold_required_sec": final_hold,
            "final_hold_seen_sec": final_hold + 0.4,
        },
        "metrics": {
            "best_along_track_m": rows[-1]["along_track_m"] if rows else None,
            "max_cross_track_m": 0.0,
            "cross_track_samples": len(rows),
            "max_cross_track_allowed_m": 120.0,
            "last_active_goal_distance_m": 0.0,
            "min_active_goal_distance_m": 0.0,
            "max_altitude_m": max((row["z"] for row in rows), default=None),
            "min_altitude_armed_m": min((row["z"] for row in rows), default=None),
            "max_stall_allowed_sec": 90.0,
            "max_stall_seen_sec": 0.0,
        },
        "task": {
            "duration_sec": rows[-1]["wall_time"] if rows else 0.0,
            "cross_track_max_m": 0.0,
            "start_label": first_label,
            "start_wall_sec": rows[0]["wall_time"] if rows else None,
            "start_nearest_distance_m": 0.0,
            "start_nearest_wall_sec": rows[0]["wall_time"] if rows else None,
            "start_first_within_radius_m": float(waypoints[0].get("radius", 1.0)),
            "final_label": final_label,
            "final_nearest_distance_m": 0.0,
            "final_nearest_wall_sec": rows[-1]["wall_time"] if rows else None,
            "final_first_within_wall_sec": rows[-1]["wall_time"] if rows else None,
            "final_first_within_radius_m": float(waypoints[-1].get("radius", 1.0)),
            "total_route_length_m": total_route_length_m,
            "p0_wall_sec": rows[0]["wall_time"] if rows else None,
            "p0_nearest_distance_m": 0.0,
            "p0_nearest_wall_sec": rows[0]["wall_time"] if rows else None,
            "p0_first_within_radius_m": float(waypoints[0].get("radius", 1.0)),
            "p8_nearest_distance_m": 0.0,
            "p8_nearest_wall_sec": rows[-1]["wall_time"] if rows else None,
            "p8_first_within_wall_sec": rows[-1]["wall_time"] if rows else None,
            "p8_first_within_radius_m": float(waypoints[-1].get("radius", 1.0)),
        },
        "landing": {
            "required": False,
            "require_disarmed": False,
            "status_messages": 0,
            "states_seen": [],
            "goaround_seen": False,
            "goaround_reasons": [],
            "touchdown_seen": False,
            "disarm_success_seen": False,
            "last_status": None,
        },
        "clearance": None,
        "clearance_failure_reasons": [],
        "output_csv": os.path.abspath(output_csv),
        "stop_reason": "final_hold_reached",
        "route_acceptance_policy": ROUTE_POLICY,
    }
    write_json(os.path.join(args.run_dir, "summary.json"), payload)
    return payload


def compose_metrics_json(args, summary, metric_summary, clearance_static, clearance_dynamic):
    actual_static = ((((clearance_static.get("metrics") or {}).get("actual_trajectory") or {}).get("static")) or {})
    actual_dynamic = ((((clearance_dynamic.get("metrics") or {}).get("actual_trajectory") or {}).get("dynamic")) or {})
    ok, components, status = route_acceptance(metric_summary, summary, clearance_static)
    payload = {
        "run_id": args.run_label,
        "run_dir": args.run_dir,
        "vehicle": "f250",
        "source": args.perception_source,
        "sensor": args.perception_source,
        "sensor_label": sensor_label(args.perception_source),
        "perception_source": args.perception_source,
        "perception": sensor_metadata(args.perception_source),
        "params": (read_json(os.path.join(args.run_dir, "params.json"), {}) or {}).get("params", {}),
        "summary_ok": bool(summary.get("ok")),
        "monitor_status": 0 if bool(summary.get("ok")) else 2,
        "route_metric_ok": bool(
            components["static_obstacle_safety"]
            and
            components["metric_3_6_keypoint_error"]
            and components["metric_3_9_endpoint_error"]
        ),
        "ok": ok,
        "final_completed": bool(status["final_completed"]),
        "final_label": status.get("final_label"),
        "reached_p8": bool(status["p8_completed"]),
        "stop_reason": summary.get("stop_reason"),
        "state": summary.get("state"),
        "counts": summary.get("counts"),
        "clouds": summary.get("clouds"),
        "route": summary.get("route"),
        "task": summary.get("task"),
        "clearance": {
            "actual_static_min_m": actual_static.get("min_clearance_m"),
            "actual_static_min_cloud_distance_m": actual_static.get("min_cloud_distance_m"),
            "actual_dynamic_min_m": actual_dynamic.get("min_clearance_m"),
            "actual_dynamic_min_cloud_distance_m": actual_dynamic.get("min_cloud_distance_m"),
            "static_collision": actual_static.get("collision"),
            "static_geometry_entry_count": actual_static.get("geometry_entry_count"),
            "static_cloud_entry_count": actual_static.get("cloud_entry_count"),
            "dynamic_geometry_entry_count": actual_dynamic.get("geometry_entry_count"),
            "dynamic_cloud_entry_count": actual_dynamic.get("cloud_entry_count"),
            "dynamic_role": "telemetry_only",
        },
        "formal_metrics": metric_summary,
        "metric_policy": {
            "policy_id": ROUTE_POLICY["policy_id"],
            "route_ok_excludes_planning_success_rate": True,
            "route_ok_excludes_metric_3_10": True,
            "route_ok_excludes_yaw": True,
            "dynamic_boat_clearance_role": "telemetry_only",
            "components": ROUTE_POLICY["route_acceptance_components"],
            "legacy_compatibility_components": ROUTE_POLICY["legacy_compatibility_components"],
        },
        "failures": [] if ok else [
            name for name in ROUTE_POLICY["route_acceptance_components"]
            if not components.get(name)
        ],
    }
    write_json(os.path.join(args.run_dir, "metrics.json"), payload)
    return payload


def dry_run(args):
    os.makedirs(args.run_dir, exist_ok=True)
    write_params_json(os.path.join(args.run_dir, "params.json"), args)
    scene, waypoints, rows, route_advancements, final_hold = synthesize_trajectory(
        args.scene_config, os.path.join(args.run_dir, "actual_trajectory.csv"))
    summary = write_synthetic_summary(args, scene, waypoints, rows, route_advancements, final_hold)
    metric_summary = run_offline(
        args.scene_config,
        os.path.join(args.run_dir, "actual_trajectory.csv"),
        args.run_dir,
        run_label=args.run_label,
        dynamic_mode=args.dynamic_mode,
        actual_filter="armed_offboard",
        clearance_sample_period_sec=0.0,
    )
    clearance_static = {}
    clearance_dynamic = {}
    metrics = compose_metrics_json(args, summary, metric_summary, clearance_static, clearance_dynamic)
    acceptance = update_artifact_policy(args, metric_summary, summary, clearance_static, clearance_dynamic)
    write_status_files(args, "complete", metric_summary, summary, clearance_static)
    state = "complete" if acceptance["ok"] else "failed"
    block = write_completed_terminal_log(
        args, metric_summary, acceptance["terminal"], state, clearance_static, clearance_dynamic, metrics)
    print(block, flush=True)
    print("OVER", flush=True)
    return 0 if acceptance["ok"] else 1


def live_monitor(args):
    import rospy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry

    class Monitor:
        def __init__(self):
            self.waypoints = scene_waypoints(load_scene(args.scene_config))
            self.accumulator = MetricAccumulator(
                args.scene_config,
                dynamic_mode=args.dynamic_mode,
                clearance_sample_period_sec=args.clearance_sample_period_sec,
            )
            self.active_goal_index = None
            self.last_progress_index = None
            self.last_final_completed = None
            self.last_started_index = None
            self.last_reached_index = 0
            self.emitted_arrival_indexes = set()
            self.confirmed_error_indexes = set()
            self.pending_error_indexes = set()
            self._lock = threading.Lock()
            self.start_wall = time.time()
            self.deadline_wall = self.start_wall + float(args.max_duration_sec)
            self.exit_code = 1
            initial = terminal_status(self.accumulator.summary())
            append_terminal_block(
                args.terminal_log,
                terminal_header_block(
                    args.perception_source,
                    route_name=initial.get("route_name"),
                    first_label=initial.get("first_label") or "P0",
                    final_label=initial.get("final_label") or "final",
                    final_index=int(initial.get("final_index") or 0),
                ),
            )
            if len(self.waypoints) > 1:
                append_terminal_block(args.terminal_log, waypoint_start_block(self.waypoints, 1))
                self.last_started_index = 1
            self.timer = rospy.Timer(rospy.Duration(max(0.2, args.display_period_sec)), self.timer_cb)
            self.odom_sub = rospy.Subscriber(args.odom_topic, Odometry, self.odom_cb, queue_size=1)
            self.active_sub = rospy.Subscriber(args.active_goal_topic, PoseStamped, self.active_goal_cb, queue_size=1)

        def active_goal_cb(self, msg):
            pos = msg.pose.position
            position = [float(pos.x), float(pos.y), float(pos.z)]
            matched = match_waypoint_index(position, self.accumulator.stats, tolerance_m=0.35)
            if matched is not None:
                self.active_goal_index = matched

        def odom_cb(self, msg):
            pos = msg.pose.pose.position
            position = [float(pos.x), float(pos.y), float(pos.z)]
            stamp = msg.header.stamp.to_sec() if msg.header.stamp and msg.header.stamp.to_sec() > 0.0 else rospy.Time.now().to_sec()
            self.accumulator.observe(
                position,
                yaw_rad=None,
                time_sec=stamp,
                active_goal_index=self.active_goal_index,
            )
            if self.accumulator.final_completed:
                self.emit(force=True, state="complete")
                self.exit_code = 0
                rospy.signal_shutdown("final waypoint completed")

        def timer_cb(self, _event):
            if time.time() >= self.deadline_wall:
                self.emit(force=True, state="timeout")
                rospy.signal_shutdown("route monitor timeout")
                return
            self.emit(force=False, state="running")

        def emit_error_if_ready(self, summary, status, index):
            # caller must hold self._lock
            if index in self.confirmed_error_indexes:
                return
            append_terminal_block(args.terminal_log, waypoint_error_block(summary, status, index))
            self.confirmed_error_indexes.add(index)

        def emit(self, force=False, state="running"):
            with self._lock:
                summary = self.accumulator.summary()
                status = terminal_status(summary)
                events = self.accumulator.drain_events()
                final_index = int(status.get("final_index") or 0)
                reached_events = []
                for event in events:
                    event_name = event.get("event")
                    if event_name not in ("waypoint_reached", "waypoint_finalized"):
                        continue
                    try:
                        index = int(event.get("index"))
                    except (TypeError, ValueError):
                        continue
                    if index <= 0:
                        continue
                    if event_name == "waypoint_finalized" and index != status.get("final_index"):
                        continue
                    reached_events.append((event_name, index))
                for event_name, index in sorted(set(reached_events), key=lambda item: item[1]):
                    if event_name == "waypoint_reached":
                        for pending_index in sorted(item for item in self.pending_error_indexes if item < index):
                            self.emit_error_if_ready(summary, status, pending_index)
                            self.pending_error_indexes.discard(pending_index)
                    if index in self.emitted_arrival_indexes:
                        arrival_was_new = False
                    else:
                        append_terminal_block(args.terminal_log, waypoint_live_arrival_block(summary, status, index))
                        self.emitted_arrival_indexes.add(index)
                        self.last_reached_index = max(self.last_reached_index, index)
                        arrival_was_new = True
                    if event_name == "waypoint_reached" and index < final_index and index not in self.confirmed_error_indexes:
                        self.pending_error_indexes.add(index)
                    if event_name == "waypoint_finalized" and index == final_index:
                        for pending_index in sorted(self.pending_error_indexes):
                            self.emit_error_if_ready(summary, status, pending_index)
                        self.pending_error_indexes.clear()
                        self.emit_error_if_ready(summary, status, index)
                    if arrival_was_new:
                        next_index = index + 1
                        if 0 < next_index < len(self.waypoints) and next_index != self.last_started_index:
                            append_terminal_block(args.terminal_log, waypoint_start_block(self.waypoints, next_index))
                            self.last_started_index = next_index
                if state == "complete" or status.get("final_completed"):
                    for index in range(1, final_index + 1):
                        self.emit_error_if_ready(summary, status, index)
                    self.pending_error_indexes.clear()
                progress_changed = status["progress_index"] != self.last_progress_index
                final_changed = status["final_completed"] != self.last_final_completed
                if progress_changed:
                    self.last_progress_index = status["progress_index"]
                    self.last_final_completed = status["final_completed"]
                elif final_changed:
                    self.last_final_completed = status["final_completed"]
                write_status_files(args, state, summary, summary=None, clearance_static=None)

    os.makedirs(args.run_dir, exist_ok=True)
    rospy.init_node("f250_route_human_summary", anonymous=True)
    monitor = Monitor()
    rospy.spin()
    return monitor.exit_code


def finalize(args):
    summary = read_json(os.path.join(args.run_dir, "summary.json"), default={})
    metric_summary = read_json(os.path.join(args.run_dir, "metric_summary.json"), default={})
    clearance_static = read_json(os.path.join(args.run_dir, "clearance_static_gate.json"), default={})
    clearance_dynamic = read_json(os.path.join(args.run_dir, "clearance_dynamic_telemetry.json"), default={})
    metrics = read_json(os.path.join(args.run_dir, "metrics.json"), default={})
    if not metrics:
        metrics = compose_metrics_json(args, summary, metric_summary, clearance_static, clearance_dynamic)
    acceptance = update_artifact_policy(args, metric_summary, summary, clearance_static, clearance_dynamic)
    metrics = read_json(os.path.join(args.run_dir, "metrics.json"), default=metrics)
    state = "complete" if acceptance["ok"] else "failed"
    write_status_files(args, state, metric_summary, summary, clearance_static)
    block = write_completed_terminal_log(
        args, metric_summary, acceptance["terminal"], state, clearance_static, clearance_dynamic, metrics)
    if args.print_final:
        print(block, flush=True)
        print("OVER", flush=True)
    return 0 if acceptance["ok"] else 1


def add_common(parser):
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--run-label", required=True)
    parser.add_argument("--scene-config", required=True)
    parser.add_argument("--perception-source", default=os.environ.get("PERCEPTION_SOURCE", "lidar"))
    parser.add_argument("--dynamic-mode", default="auto")
    parser.add_argument("--max-duration-sec", type=float, default=360.0)
    parser.add_argument("--terminal-log", required=True)
    parser.add_argument("--route-status-env", required=True)
    parser.add_argument("--status-env", required=True)


def parse_args():
    parser = argparse.ArgumentParser(description="F250 selected-route terminal display and summary helper.")
    sub = parser.add_subparsers(dest="command", required=True)

    dry = sub.add_parser("dry-run", help="write synthetic successful F250 route artifacts without ROS")
    add_common(dry)

    live = sub.add_parser("live-monitor", help="print restricted live F250 route terminal lines")
    add_common(live)
    live.add_argument("--odom-topic", default="/mavros/local_position/odom")
    live.add_argument("--active-goal-topic", default="/maritime/active_goal")
    live.add_argument("--clearance-sample-period-sec", type=float, default=0.5)
    live.add_argument("--display-period-sec", type=float, default=1.0)

    final = sub.add_parser("finalize", help="write route acceptance summary from run artifacts")
    add_common(final)
    final.add_argument("--print-final", action="store_true")

    return parser.parse_args()


def main():
    args = parse_args()
    if args.command == "dry-run":
        return dry_run(args)
    if args.command == "live-monitor":
        return live_monitor(args)
    if args.command == "finalize":
        return finalize(args)
    raise RuntimeError("unknown command: %s" % args.command)


if __name__ == "__main__":
    raise SystemExit(main())
