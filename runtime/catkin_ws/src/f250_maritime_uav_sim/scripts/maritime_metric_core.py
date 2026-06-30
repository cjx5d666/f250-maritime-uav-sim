#!/usr/bin/env python3
import csv
import json
import math
import os
import time

from maritime_clearance import PointCloudIndex
from maritime_clearance import dynamic_planner_obstacles_at, min_obstacle_distance
from maritime_clearance import sample_bruteforce_cloud_distance, static_planner_obstacles
from maritime_scene_utils import dynamic_mode_enabled, load_scene, scene_cloud_points
from maritime_scene_utils import scene_dynamic_cloud_points
from maritime_scene_utils import scene_waypoints


def now_label():
    return time.strftime("%Y%m%d_%H%M%S")


def wrap_angle_rad(value):
    return (float(value) + math.pi) % (2.0 * math.pi) - math.pi


def distance(a, b):
    return math.sqrt(
        (float(a[0]) - float(b[0])) ** 2 +
        (float(a[1]) - float(b[1])) ** 2 +
        (float(a[2]) - float(b[2])) ** 2
    )


def safe_float(value, default=None):
    if value is None or value == "":
        return default
    try:
        return float(value)
    except Exception:
        return default


def truthy(value):
    return str(value).strip().lower() in ("1", "true", "yes", "y", "on")


def count_pass(values):
    return sum(1 for value in values if value)


def waypoint_label(index, waypoint):
    label = waypoint.get("label")
    if label:
        return str(label)
    name = str(waypoint.get("name", ""))
    prefix = name.split("_")[0].upper()
    return prefix if prefix else "P%d" % index


def waypoint_position(waypoint):
    return [float(value) for value in waypoint["position"][:3]]


def route_length(points, start=0, end=None):
    if end is None:
        end = len(points) - 1
    total = 0.0
    for index in range(max(1, start + 1), end + 1):
        total += distance(points[index - 1], points[index])
    return total


def match_waypoint_index(position, waypoints, tolerance_m=0.25):
    if position is None:
        return None
    best_index = None
    best_distance = None
    for index, waypoint in enumerate(waypoints):
        value = distance(position, waypoint["position"])
        if best_distance is None or value < best_distance:
            best_distance = value
            best_index = index
    if best_distance is None or best_distance > float(tolerance_m):
        return None
    return best_index


class MetricAccumulator:
    def __init__(self, scene_or_path, dynamic_mode="auto", clearance_sample_period_sec=0.2,
                 include_cloud_distance=True, cloud_entry_radius_m=None,
                 cloud_search_radius_m=None, final_zone_hold_sec=None):
        self.scene = load_scene(scene_or_path) if isinstance(scene_or_path, str) else scene_or_path
        self.dynamic_mode = dynamic_mode
        self.waypoints = scene_waypoints(self.scene)
        if not self.waypoints:
            raise RuntimeError("scene has no waypoints")
        self.positions = [waypoint_position(waypoint) for waypoint in self.waypoints]
        self.segment_lengths = [None]
        for index in range(1, len(self.positions)):
            self.segment_lengths.append(distance(self.positions[index - 1], self.positions[index]))
        self.total_route_length_m = route_length(self.positions, 0, len(self.positions) - 1)
        acceptance = self.scene.get("acceptance") or {}
        self.final_zone_hold_sec = (
            float(final_zone_hold_sec) if final_zone_hold_sec is not None
            else float(acceptance.get("final_zone_hold_sec", 0.0))
        )
        self.clearance_sample_period_sec = max(0.0, float(clearance_sample_period_sec))
        self.include_cloud_distance = bool(include_cloud_distance)
        resolution = float(self.scene.get("cloud_resolution", 0.25))
        min_static = float(acceptance.get("min_obstacle_distance_m", 0.0))
        min_dynamic = float(acceptance.get("min_dynamic_obstacle_distance_m", min_static))
        self.cloud_entry_radius_m = (
            float(cloud_entry_radius_m) if cloud_entry_radius_m is not None else resolution / 2.0
        )
        self.cloud_search_radius_m = (
            float(cloud_search_radius_m) if cloud_search_radius_m is not None
            else max(self.cloud_entry_radius_m, min_static, min_dynamic, resolution)
        )

        self.static_obstacles = static_planner_obstacles(self.scene)
        self.dynamic_enabled = dynamic_mode_enabled(self.scene, dynamic_mode)
        self.static_cloud_index = None
        if self.include_cloud_distance:
            labeled = scene_cloud_points(self.scene, include_labels=True, include_dynamic=False)
            self.static_cloud_index = PointCloudIndex(labeled, max(resolution, self.cloud_entry_radius_m, 0.25))

        self.stats = [self._new_waypoint_stat(index, waypoint) for index, waypoint in enumerate(self.waypoints)]
        self.active_goal_index = None
        self.max_active_goal_index = -1
        self.sample_count = 0
        self.started_time_sec = None
        self.last_sample_time_sec = None
        self.last_clearance_time_sec = None
        self.final_hold_enter_time_sec = None
        self.final_hold_seen_sec = 0.0
        self.p8_completed = False
        self.final_completed = False
        self.events = []
        self._written_paths = []

        self.clearance = {
            "static_min_clearance_m": None,
            "static_nearest_obstacle": None,
            "static_geometry_entry_count": 0,
            "static_cloud_min_distance_m": None,
            "static_nearest_cloud_label": None,
            "static_cloud_entry_count": 0,
            "dynamic_min_clearance_m": None,
            "dynamic_nearest_obstacle": None,
            "dynamic_geometry_entry_count": 0,
            "dynamic_cloud_min_distance_m": None,
            "dynamic_nearest_cloud_label": None,
            "dynamic_cloud_entry_count": 0,
        }

    def _new_waypoint_stat(self, index, waypoint):
        return {
            "index": index,
            "label": waypoint_label(index, waypoint),
            "name": waypoint.get("name", "waypoint_%d" % index),
            "position": waypoint_position(waypoint),
            "yaw_rad": float(waypoint.get("yaw", 0.0)),
            "radius_m": float(waypoint.get("radius", (self.scene.get("acceptance") or {}).get("position_tolerance_m", 1.0))),
            "hold_time_sec": float(waypoint.get("hold_time", 0.0)),
            "max_duration_sec": float(waypoint.get("max_duration_sec", 0.0)),
            "status": "pending",
            "finalized": False,
            "finalized_time_sec": None,
            "finalize_reason": None,
            "reached": False,
            "nearest_distance_m": None,
            "nearest_position": None,
            "nearest_time_sec": None,
            "nearest_actual_yaw_rad": None,
            "nearest_yaw_error_rad": None,
            "active_nearest_distance_m": None,
            "active_nearest_position": None,
            "active_nearest_time_sec": None,
            "active_nearest_actual_yaw_rad": None,
            "active_nearest_yaw_error_rad": None,
            "metric_3_6_error_ratio": None,
            "metric_3_6_pass": None,
            "metric_3_9_final_error_ratio": None,
        }

    def observe(self, position, yaw_rad=None, time_sec=None, active_goal_index=None):
        if time_sec is None:
            time_sec = time.time()
        time_sec = float(time_sec)
        position = [float(value) for value in position[:3]]
        if self.started_time_sec is None:
            self.started_time_sec = time_sec
        self.sample_count += 1
        self.last_sample_time_sec = time_sec

        self._update_waypoint_nearest(position, yaw_rad, time_sec, active_goal_index)
        self._update_clearance(position, time_sec)
        self._update_final_hold(position, time_sec)

        if active_goal_index is not None:
            self._set_active_goal(int(active_goal_index), time_sec)

    def _update_waypoint_nearest(self, position, yaw_rad, time_sec, active_goal_index):
        for index, stat in enumerate(self.stats):
            value = distance(position, stat["position"])
            self._maybe_update_nearest(stat, "", value, position, yaw_rad, time_sec)
            if value <= stat["radius_m"] and not stat["reached"]:
                stat["reached"] = True
                self._update_metric_fields(stat)
                self.events.append({
                    "event": "waypoint_reached",
                    "index": index,
                    "label": stat["label"],
                    "name": stat["name"],
                    "nearest_distance_m": stat["nearest_distance_m"],
                    "yaw_error_rad": stat["nearest_yaw_error_rad"],
                    "metric_3_6_error_ratio": stat["metric_3_6_error_ratio"],
                    "route_position_pass": True,
                    "reason": "radius_reached",
                    "time_sec": float(time_sec),
                })
            if active_goal_index is not None and int(active_goal_index) == index:
                self._maybe_update_nearest(stat, "active_", value, position, yaw_rad, time_sec)

    def _maybe_update_nearest(self, stat, prefix, value, position, yaw_rad, time_sec):
        key = "%snearest_distance_m" % prefix
        if stat[key] is not None and value >= stat[key]:
            return
        yaw_error = None
        if yaw_rad is not None:
            yaw_error = wrap_angle_rad(float(yaw_rad) - stat["yaw_rad"])
        stat[key] = value
        stat["%snearest_position" % prefix] = [float(position[0]), float(position[1]), float(position[2])]
        stat["%snearest_time_sec" % prefix] = time_sec
        stat["%snearest_actual_yaw_rad" % prefix] = float(yaw_rad) if yaw_rad is not None else None
        stat["%snearest_yaw_error_rad" % prefix] = yaw_error

    def _set_active_goal(self, index, time_sec):
        if index < 0 or index >= len(self.stats):
            return
        previous = self.active_goal_index
        if previous == index:
            self.stats[index]["status"] = "current"
            return
        self.active_goal_index = index
        self.max_active_goal_index = max(self.max_active_goal_index, index)
        for stat in self.stats:
            if stat["finalized"]:
                continue
            stat["status"] = "current" if stat["index"] == index else "pending"

    def _update_clearance(self, position, time_sec):
        if self.clearance_sample_period_sec > 0.0 and self.last_clearance_time_sec is not None:
            if time_sec - self.last_clearance_time_sec < self.clearance_sample_period_sec:
                return
        self.last_clearance_time_sec = time_sec

        static_value, static_obstacle = min_obstacle_distance(position, self.static_obstacles)
        self._update_min_clearance("static", static_value, static_obstacle)
        if static_value is not None and static_value <= 0.0:
            self.clearance["static_geometry_entry_count"] += 1

        if self.include_cloud_distance and self.static_cloud_index is not None:
            cloud_value, label = self.static_cloud_index.nearest(position, self.cloud_search_radius_m)
            self._update_cloud("static", cloud_value, label)

        if not self.dynamic_enabled:
            return
        dynamic_obstacles = dynamic_planner_obstacles_at(self.scene, time_sec, self.dynamic_mode)
        dynamic_value, dynamic_obstacle = min_obstacle_distance(position, dynamic_obstacles)
        self._update_min_clearance("dynamic", dynamic_value, dynamic_obstacle)
        if dynamic_value is not None and dynamic_value <= 0.0:
            self.clearance["dynamic_geometry_entry_count"] += 1
        if self.include_cloud_distance and dynamic_value is not None and dynamic_value <= self.cloud_search_radius_m:
            dynamic_points = scene_dynamic_cloud_points(self.scene, dynamic_time_sec=time_sec, include_labels=True)
            cloud_value, label = sample_bruteforce_cloud_distance(position, dynamic_points) if dynamic_points else (None, None)
            self._update_cloud("dynamic", cloud_value, label)

    def _update_min_clearance(self, scope, value, obstacle):
        if value is None:
            return
        key = "%s_min_clearance_m" % scope
        if self.clearance[key] is None or value < self.clearance[key]:
            self.clearance[key] = float(value)
            self.clearance["%s_nearest_obstacle" % scope] = obstacle.get("name") if obstacle else None

    def _update_cloud(self, scope, value, label):
        if value is None:
            return
        key = "%s_cloud_min_distance_m" % scope
        if self.clearance[key] is None or value < self.clearance[key]:
            self.clearance[key] = float(value)
            self.clearance["%s_nearest_cloud_label" % scope] = label
        if value <= self.cloud_entry_radius_m:
            self.clearance["%s_cloud_entry_count" % scope] += 1

    def _update_final_hold(self, position, time_sec):
        last_index = len(self.stats) - 1
        p8 = self.stats[last_index]
        in_final = distance(position, p8["position"]) <= p8["radius_m"]
        if in_final:
            if self.final_hold_enter_time_sec is None:
                self.final_hold_enter_time_sec = time_sec
            self.final_hold_seen_sec = max(0.0, time_sec - self.final_hold_enter_time_sec)
            if self.final_hold_seen_sec >= self.final_zone_hold_sec and not self.p8_completed:
                self.p8_completed = True
                self.final_completed = True
                self.finalize_waypoint(last_index, time_sec, "final_hold_reached")
        else:
            self.final_hold_enter_time_sec = None
            self.final_hold_seen_sec = 0.0

    def finalize_waypoint(self, index, time_sec, reason):
        if index < 0 or index >= len(self.stats):
            return
        stat = self.stats[index]
        if stat["finalized"]:
            return
        stat["finalized"] = True
        stat["finalized_time_sec"] = float(time_sec)
        stat["finalize_reason"] = reason
        stat["status"] = "passed" if self.waypoint_passed(index) else "failed"
        self._update_metric_fields(stat)
        self.events.append({
            "event": "waypoint_finalized",
            "index": index,
            "label": stat["label"],
            "name": stat["name"],
            "status": stat["status"],
            "nearest_distance_m": stat["nearest_distance_m"],
            "yaw_error_rad": stat["nearest_yaw_error_rad"],
            "metric_3_6_error_ratio": stat["metric_3_6_error_ratio"],
            "route_position_pass": self.waypoint_passed(index),
            "reason": reason,
            "time_sec": float(time_sec),
        })

    def _update_metric_fields(self, stat):
        index = stat["index"]
        nearest = stat["nearest_distance_m"]
        if 1 <= index <= len(self.stats) - 2 and nearest is not None:
            denom = self.segment_lengths[index] or 0.0
            stat["metric_3_6_error_ratio"] = nearest / denom if denom > 0.0 else None
            stat["metric_3_6_pass"] = nearest <= stat["radius_m"]
        if index == len(self.stats) - 1 and nearest is not None and self.total_route_length_m > 0.0:
            stat["metric_3_9_final_error_ratio"] = nearest / self.total_route_length_m

    def waypoint_passed(self, index):
        stat = self.stats[index]
        nearest = stat["nearest_distance_m"]
        return nearest is not None and nearest <= stat["radius_m"]

    def drain_events(self):
        events = list(self.events)
        self.events = []
        return events

    def refresh_metric_fields(self):
        for stat in self.stats:
            self._update_metric_fields(stat)

    def status_for_marker(self, index):
        stat = self.stats[index]
        if self.active_goal_index == index and not stat["finalized"]:
            return "current"
        nearest = stat["nearest_distance_m"]
        if stat["finalized"]:
            if not self.waypoint_passed(index):
                return "failed"
            if nearest is not None and stat["radius_m"] > 0.0 and nearest / stat["radius_m"] >= 0.8:
                return "near_threshold"
            return "passed"
        if nearest is not None and stat["radius_m"] > 0.0 and nearest <= stat["radius_m"] * 1.15:
            return "near_threshold"
        return "pending"

    def summary(self):
        self.refresh_metric_fields()
        metric36_values = [
            stat["metric_3_6_error_ratio"] for stat in self.stats[1:-1]
            if stat["metric_3_6_error_ratio"] is not None
        ]
        metric36_passes = [
            bool(stat["metric_3_6_pass"]) for stat in self.stats[1:-1]
            if stat["metric_3_6_pass"] is not None
        ]
        final = self.stats[-1]
        route_profile = self.scene.get("route_profile") or {}
        route_id = route_profile.get("route_id")
        route_name = route_profile.get("name")
        route_profile_path = route_profile.get("profile_path")
        final_label = route_profile.get("final_label") or final["label"]
        static_geometry_entries = int(self.clearance["static_geometry_entry_count"])
        static_cloud_entries = int(self.clearance["static_cloud_entry_count"])
        dynamic_geometry_entries = int(self.clearance["dynamic_geometry_entry_count"])
        dynamic_cloud_entries = int(self.clearance["dynamic_cloud_entry_count"])
        static_safe = static_geometry_entries == 0 and static_cloud_entries == 0
        dynamic_entries = dynamic_geometry_entries + dynamic_cloud_entries
        metric36_pass = bool(metric36_passes and all(metric36_passes))
        metric37_pass = static_safe and self.final_completed
        metric39_pass = bool(self.final_completed and self.waypoint_passed(len(self.stats) - 1))
        route_ok = bool(
            static_safe and self.final_completed and metric36_pass and metric37_pass and
            metric39_pass
        )
        return {
            "ok": route_ok,
            "scene": os.path.abspath(self.scene.get("_scene_path", "")) if self.scene.get("_scene_path") else None,
            "sample_count": int(self.sample_count),
            "active_goal_index": self.active_goal_index,
            "max_active_goal_index": self.max_active_goal_index,
            "final_completed": bool(self.final_completed),
            "final_label": final_label,
            "p8_completed": bool(self.p8_completed),
            "final_hold_seen_sec": float(self.final_hold_seen_sec),
            "route": {
                "route_id": route_id,
                "name": route_name,
                "profile": route_profile_path,
                "profile_source": route_profile.get("profile_source"),
                "waypoint_count": len(self.stats),
                "first_label": self.stats[0]["label"],
                "final_label": final_label,
                "total_route_length_m": self.total_route_length_m,
                "locked_baseline_compatibility": bool(route_profile.get("locked_baseline_compatibility", False)),
                "total_p0_p8_length_m": self.total_route_length_m,
                "segment_lengths_m": self.segment_lengths,
            },
            "metric_3_6": {
                "description": "P1-P7 nearest 3D waypoint error divided by previous segment length",
                "mean_error_ratio": (sum(metric36_values) / len(metric36_values)) if metric36_values else None,
                "max_error_ratio": max(metric36_values) if metric36_values else None,
                "point_count": len(metric36_values),
                "pass_count": count_pass(metric36_passes),
                "passed": metric36_pass,
            },
            "metric_3_7": {
                "description": "No static planner-visible entry and final waypoint hold completed; dynamic clearance is telemetry",
                "passed": bool(metric37_pass),
                "safe_so_far": bool(static_safe),
                "static_safe": bool(static_safe),
                "collision": static_geometry_entries > 0,
                "geometry_entry_count": static_geometry_entries,
                "cloud_entry_count": static_cloud_entries,
                "static_geometry_entry_count": static_geometry_entries,
                "static_cloud_entry_count": static_cloud_entries,
                "dynamic_entry_count_telemetry": dynamic_entries,
                "dynamic_geometry_entry_count_telemetry": dynamic_geometry_entries,
                "dynamic_cloud_entry_count_telemetry": dynamic_cloud_entries,
                "clearance": self.clearance,
            },
            "metric_3_9": {
                "description": "Final-hold nearest endpoint error divided by selected route length",
                "final_label": final_label,
                "final_error_m": final["nearest_distance_m"],
                "final_error_ratio": final["metric_3_9_final_error_ratio"],
                "passed": metric39_pass,
            },
            "policy_note": (
                "Route/planner evaluation excludes planning success rate, Metric 3.10, and yaw pass/fail. "
                "Metric 3.10 is a separate FC steady-state scalar test."
            ),
            "waypoints": self.stats,
        }

    def write_outputs(self, output_dir, run_label=None, create_run_subdir=True):
        label = run_label or "metric_%s" % now_label()
        base = os.path.abspath(output_dir)
        out_dir = os.path.join(base, label) if create_run_subdir else base
        os.makedirs(out_dir, exist_ok=True)
        summary = self.summary()
        summary["run_label"] = label
        summary_path = os.path.join(out_dir, "metric_summary.json")
        csv_path = os.path.join(out_dir, "metric_waypoints.csv")
        with open(summary_path, "w", encoding="utf-8") as handle:
            json.dump(summary, handle, indent=2, sort_keys=True)
            handle.write("\n")
        fields = [
            "index", "label", "name", "x", "y", "z", "yaw_rad", "radius_m",
            "nearest_distance_m", "nearest_x", "nearest_y", "nearest_z",
            "nearest_time_sec", "nearest_actual_yaw_rad", "nearest_yaw_error_rad",
            "active_nearest_distance_m", "metric_3_6_error_ratio", "metric_3_6_pass",
            "metric_3_9_final_error_ratio", "route_position_pass", "status",
            "finalized", "finalize_reason",
        ]
        with open(csv_path, "w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            for stat in self.stats:
                nearest = stat["nearest_position"] or [None, None, None]
                writer.writerow({
                    "index": stat["index"],
                    "label": stat["label"],
                    "name": stat["name"],
                    "x": stat["position"][0],
                    "y": stat["position"][1],
                    "z": stat["position"][2],
                    "yaw_rad": stat["yaw_rad"],
                    "radius_m": stat["radius_m"],
                    "nearest_distance_m": stat["nearest_distance_m"],
                    "nearest_x": nearest[0],
                    "nearest_y": nearest[1],
                    "nearest_z": nearest[2],
                    "nearest_time_sec": stat["nearest_time_sec"],
                    "nearest_actual_yaw_rad": stat["nearest_actual_yaw_rad"],
                    "nearest_yaw_error_rad": stat["nearest_yaw_error_rad"],
                    "active_nearest_distance_m": stat["active_nearest_distance_m"],
                    "metric_3_6_error_ratio": stat["metric_3_6_error_ratio"],
                    "metric_3_6_pass": stat["metric_3_6_pass"],
                    "metric_3_9_final_error_ratio": stat["metric_3_9_final_error_ratio"],
                    "route_position_pass": self.waypoint_passed(stat["index"]),
                    "status": stat["status"],
                    "finalized": stat["finalized"],
                    "finalize_reason": stat["finalize_reason"],
                })
        self._written_paths = [summary_path, csv_path]
        return self._written_paths


def trajectory_rows(path, actual_filter="armed_offboard"):
    with open(path, "r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            x = safe_float(row.get("x"))
            y = safe_float(row.get("y"))
            z = safe_float(row.get("z"))
            if x is None or y is None or z is None:
                continue
            normalized_filter = str(actual_filter or "armed_offboard").lower()
            armed = truthy(row.get("armed", "false"))
            mode = str(row.get("mode", ""))
            if normalized_filter == "armed" and not armed:
                continue
            if normalized_filter == "armed_offboard" and not (armed and mode == "OFFBOARD"):
                continue
            time_sec = safe_float(row.get("ros_time"))
            if time_sec is None or time_sec <= 0.0:
                time_sec = safe_float(row.get("wall_time"), 0.0)
            yaw = safe_float(row.get("actual_yaw"))
            active_index = safe_float(row.get("active_goal_index"))
            if active_index is not None:
                active_index = int(active_index)
            active_position = None
            ax = safe_float(row.get("active_goal_x"))
            ay = safe_float(row.get("active_goal_y"))
            az = safe_float(row.get("active_goal_z"))
            if ax is not None and ay is not None and az is not None:
                active_position = [ax, ay, az]
            yield {
                "position": [x, y, z],
                "yaw_rad": yaw,
                "time_sec": time_sec,
                "active_goal_index": active_index,
                "active_goal_position": active_position,
            }


def run_offline(scene_config, trajectory_csv, output_dir, run_label=None, dynamic_mode="auto",
                actual_filter="armed_offboard", clearance_sample_period_sec=0.0):
    accumulator = MetricAccumulator(
        scene_config,
        dynamic_mode=dynamic_mode,
        clearance_sample_period_sec=clearance_sample_period_sec,
    )
    for row in trajectory_rows(trajectory_csv, actual_filter=actual_filter):
        active_index = row["active_goal_index"]
        if active_index is None:
            active_index = match_waypoint_index(row["active_goal_position"], accumulator.stats)
        accumulator.observe(
            row["position"],
            yaw_rad=row["yaw_rad"],
            time_sec=row["time_sec"],
            active_goal_index=active_index,
        )
    if output_dir:
        accumulator.write_outputs(output_dir, run_label=run_label, create_run_subdir=False)
    return accumulator.summary()
