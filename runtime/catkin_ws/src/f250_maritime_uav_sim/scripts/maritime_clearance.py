#!/usr/bin/env python3
import csv
import math
import os
from collections import defaultdict

from maritime_scene_utils import cylinder_z_range
from maritime_scene_utils import dynamic_collision_proxy_instances
from maritime_scene_utils import dynamic_mode_enabled, dynamic_obstacle_center, dynamic_obstacle_yaw
from maritime_scene_utils import load_scene, scene_box, scene_cloud_points
from maritime_scene_utils import scene_dynamic_cloud_points, scene_dynamic_obstacles
from maritime_scene_utils import visual_collision_proxy_instances
from maritime_scene_utils import scene_waypoints


def squared_distance(a, b):
    return (
        (float(a[0]) - float(b[0])) ** 2 +
        (float(a[1]) - float(b[1])) ** 2 +
        (float(a[2]) - float(b[2])) ** 2
    )


def distance(a, b):
    return math.sqrt(squared_distance(a, b))


def as_point(value):
    return [float(value[0]), float(value[1]), float(value[2])]


def truthy(value):
    return str(value).strip().lower() in ("1", "true", "yes", "y", "on")


def signed_box_distance(point, center, size, yaw=0.0):
    px, py, pz = as_point(point)
    cx, cy, cz = as_point(center)
    sx, sy, sz = as_point(size)
    yaw = float(yaw or 0.0)
    rel_x = px - cx
    rel_y = py - cy
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    local_x = rel_x * cos_yaw + rel_y * sin_yaw
    local_y = -rel_x * sin_yaw + rel_y * cos_yaw
    dx = abs(local_x) - sx / 2.0
    dy = abs(local_y) - sy / 2.0
    dz = abs(pz - cz) - sz / 2.0
    outside = [max(dx, 0.0), max(dy, 0.0), max(dz, 0.0)]
    outside_distance = math.sqrt(outside[0] ** 2 + outside[1] ** 2 + outside[2] ** 2)
    if outside_distance > 0.0:
        return outside_distance
    return max(dx, dy, dz)


def signed_cylinder_distance(point, center, radius, z_min, z_max):
    px, py, pz = as_point(point)
    cx, cy, _cz = as_point(center)
    radial = math.sqrt((px - cx) ** 2 + (py - cy) ** 2) - float(radius)
    below = float(z_min) - pz
    above = pz - float(z_max)
    z_outside = max(below, above, 0.0)
    radial_outside = max(radial, 0.0)
    if radial_outside > 0.0 or z_outside > 0.0:
        return math.sqrt(radial_outside ** 2 + z_outside ** 2)
    return max(radial, below, above)


def obstacle_distance(point, obstacle):
    if obstacle["shape"] == "box":
        return signed_box_distance(point, obstacle["center"], obstacle["size"],
                                   obstacle.get("yaw", 0.0))
    if obstacle["shape"] == "cylinder":
        return signed_cylinder_distance(
            point, obstacle["center"], obstacle["radius"], obstacle["z_min"], obstacle["z_max"])
    raise ValueError("unsupported obstacle shape: %s" % obstacle["shape"])


def min_obstacle_distance(point, obstacles):
    best = None
    best_obstacle = None
    for obstacle in obstacles:
        value = obstacle_distance(point, obstacle)
        if best is None or value < best:
            best = value
            best_obstacle = obstacle
    return best, best_obstacle


def make_box_obstacle(name, center, size, category, source_key, yaw=0.0):
    return {
        "name": str(name),
        "shape": "box",
        "center": as_point(center),
        "size": as_point(size),
        "yaw": float(yaw or 0.0),
        "category": category,
        "source_key": source_key,
    }


def make_cylinder_obstacle(name, item, category, source_key):
    z_min, z_max = cylinder_z_range(item)
    return {
        "name": str(name),
        "shape": "cylinder",
        "center": as_point(item.get("center")),
        "radius": float(item.get("radius")),
        "z_min": float(z_min),
        "z_max": float(z_max),
        "category": category,
        "source_key": source_key,
    }


def dynamic_proxy_obstacles(item, time_sec):
    obstacles = []
    for proxy in dynamic_collision_proxy_instances(item, time_sec, include_cloud_only=True):
        name = proxy.get("name", item.get("name", "dynamic_obstacle"))
        shape = str(proxy.get("shape", "box")).lower()
        if shape == "box":
            obstacles.append(make_box_obstacle(
                name, proxy.get("center"), proxy.get("size"), "dynamic", "dynamic_collision_proxies",
                yaw=float(proxy.get("yaw", 0.0))))
        elif shape == "cylinder":
            obstacles.append(make_cylinder_obstacle(name, proxy, "dynamic", "dynamic_collision_proxies"))
        else:
            raise ValueError("unsupported dynamic collision proxy shape: %s" % shape)
    return obstacles


def dynamic_parent_obstacle(item, index, time_sec):
    envelope = item.get("safety_envelope") or {}
    name = item.get("name", "dynamic_obstacle_%d" % index)
    parent_center = dynamic_obstacle_center(item, time_sec)
    parent_yaw = dynamic_obstacle_yaw(item, time_sec)
    local_center = as_point(envelope.get("center", [0.0, 0.0, 0.0]))
    cos_yaw = math.cos(parent_yaw)
    sin_yaw = math.sin(parent_yaw)
    center = [
        parent_center[0] + local_center[0] * cos_yaw - local_center[1] * sin_yaw,
        parent_center[1] + local_center[0] * sin_yaw + local_center[1] * cos_yaw,
        parent_center[2] + local_center[2],
    ]
    shape = str(envelope.get("shape", item.get("shape", "box"))).lower()
    source_key = "dynamic_safety_envelope" if envelope else "dynamic_obstacles"
    if shape == "box":
        return make_box_obstacle(
            name, center, envelope.get("size", item.get("size")), "dynamic", source_key,
            yaw=parent_yaw + float(envelope.get("yaw", 0.0)))
    if shape == "cylinder":
        cylinder = dict(item)
        cylinder.update(envelope)
        cylinder["center"] = center
        return make_cylinder_obstacle(name, cylinder, "dynamic", source_key)
    if shape == "composite":
        raise ValueError("dynamic obstacle %s has composite shape but no collision proxies" % name)
    raise ValueError("unsupported dynamic safety envelope shape: %s" % shape)


def static_planner_obstacles(scene):
    obstacles = []

    for key in ("deck", "landing_box", "takeoff_deck_zone", "landing_deck_zone"):
        zone = scene_box(scene, key)
        if zone and zone.get("include_in_cloud", False):
            obstacles.append(make_box_obstacle(
                zone.get("name", key), zone["center"], zone["size"], "static", key,
                yaw=zone.get("yaw", 0.0)))

    for index, item in enumerate(scene.get("bridge_piers", []) or []):
        if not item.get("include_in_cloud", True):
            continue
        obstacles.append(make_cylinder_obstacle(
            item.get("name", "bridge_pier_%d" % index), item, "static", "bridge_piers"))

    for index, item in enumerate(scene.get("buoys", []) or []):
        if not item.get("include_in_cloud", True):
            continue
        obstacles.append(make_cylinder_obstacle(item.get("name", "buoy_%d" % index), item, "static", "buoys"))

    for index, item in enumerate(scene.get("docks", []) or []):
        if not item.get("include_in_cloud", True):
            continue
        obstacles.append(make_box_obstacle(
            item.get("name", "dock_%d" % index), item.get("center"), item.get("size"), "static", "docks",
            yaw=float(item.get("yaw", 0.0))))

    for index, item in enumerate(scene.get("box_obstacles", []) or []):
        if not item.get("include_in_cloud", True):
            continue
        obstacles.append(make_box_obstacle(
            item.get("name", "box_obstacle_%d" % index),
            item.get("center"), item.get("size"), "static", "box_obstacles",
            yaw=float(item.get("yaw", 0.0))))

    for index, item in enumerate(visual_collision_proxy_instances(scene, include_cloud_only=True)):
        name = item.get("name", "visual_collision_proxy_%d" % index)
        shape = str(item.get("shape", "box")).lower()
        if shape == "box":
            obstacles.append(make_box_obstacle(
                name, item.get("center"), item.get("size"), "static", "visual_collision_proxies",
                yaw=float(item.get("yaw", 0.0))))
        elif shape == "cylinder":
            obstacles.append(make_cylinder_obstacle(name, item, "static", "visual_collision_proxies"))
        else:
            raise ValueError("unsupported visual collision proxy shape: %s" % shape)

    return obstacles


def dynamic_planner_obstacles_at(scene, time_sec=0.0, dynamic_mode="auto"):
    if not dynamic_mode_enabled(scene, dynamic_mode):
        return []
    obstacles = []
    for index, item in enumerate(scene_dynamic_obstacles(scene)):
        if not item.get("include_in_cloud", True):
            continue
        proxy_obstacles = dynamic_proxy_obstacles(item, time_sec)
        if item.get("collision_proxies"):
            if not proxy_obstacles:
                raise ValueError("dynamic obstacle %s collision_proxies did not yield planner obstacles" %
                                 item.get("name", "dynamic_obstacle_%d" % index))
            obstacles.extend(proxy_obstacles)
            continue
        obstacles.append(dynamic_parent_obstacle(item, index, time_sec))
    return obstacles


def dynamic_periods(scene):
    periods = []
    for item in scene_dynamic_obstacles(scene):
        motion = item.get("motion") or {}
        if str(motion.get("type", "static")).lower() == "sinusoid":
            try:
                periods.append(float(motion.get("period_sec", 0.0)))
            except Exception:
                pass
    return [period for period in periods if period > 0.0]


def default_dynamic_horizon_sec(scene):
    periods = dynamic_periods(scene)
    if periods:
        return max(periods)
    waypoint_total = 0.0
    for waypoint in scene.get("waypoints", []) or []:
        waypoint_total += float(waypoint.get("max_duration_sec", 0.0) or 0.0)
    return max(waypoint_total, 1.0)


def dynamic_sample_times(scene, count=64, horizon_sec=None):
    if not dynamic_mode_enabled(scene, "auto") or not scene_dynamic_obstacles(scene):
        return []
    samples = max(2, int(count))
    horizon = float(horizon_sec) if horizon_sec is not None else default_dynamic_horizon_sec(scene)
    if horizon <= 0.0:
        horizon = 1.0
    return [horizon * float(index) / float(samples - 1) for index in range(samples)]


def waypoint_samples(scene):
    samples = []
    for index, waypoint in enumerate(scene_waypoints(scene)):
        samples.append({
            "point": waypoint["position"],
            "kind": "waypoint",
            "index": index,
            "label": waypoint["name"],
            "time_sec": None,
        })
    return samples


def polyline_samples(points, spacing_m=0.25):
    spacing = max(float(spacing_m), 0.01)
    if not points:
        return []
    samples = [{
        "point": as_point(points[0]),
        "kind": "polyline",
        "index": 0,
        "label": "polyline_0",
        "time_sec": None,
    }]
    sample_index = 1
    for segment_index in range(len(points) - 1):
        start = as_point(points[segment_index])
        end = as_point(points[segment_index + 1])
        length = distance(start, end)
        steps = max(1, int(math.ceil(length / spacing)))
        for step in range(1, steps + 1):
            u = float(step) / float(steps)
            point = [
                start[0] + (end[0] - start[0]) * u,
                start[1] + (end[1] - start[1]) * u,
                start[2] + (end[2] - start[2]) * u,
            ]
            samples.append({
                "point": point,
                "kind": "polyline",
                "index": sample_index,
                "segment_index": segment_index,
                "segment_u": u,
                "label": "segment_%d" % segment_index,
                "time_sec": None,
            })
            sample_index += 1
    return samples


def planned_polyline_samples(scene, spacing_m=0.25):
    return polyline_samples([waypoint["position"] for waypoint in scene_waypoints(scene)], spacing_m)


def parse_float(value, default=None):
    if value is None or value == "":
        return default
    try:
        return float(value)
    except Exception:
        return default


def trajectory_samples_from_csv(path, filter_mode="armed_offboard"):
    samples = []
    if not path:
        return samples
    with open(path, "r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for index, row in enumerate(reader):
            x = parse_float(row.get("x"))
            y = parse_float(row.get("y"))
            z = parse_float(row.get("z"))
            if x is None or y is None or z is None:
                continue
            armed = truthy(row.get("armed", "false"))
            mode = str(row.get("mode", ""))
            normalized_filter = str(filter_mode or "armed_offboard").lower()
            if normalized_filter == "armed" and not armed:
                continue
            if normalized_filter == "armed_offboard" and not (armed and mode == "OFFBOARD"):
                continue
            ros_time = parse_float(row.get("ros_time"))
            wall_time = parse_float(row.get("wall_time"), 0.0)
            time_sec = ros_time if ros_time is not None and ros_time > 0.0 else wall_time
            samples.append({
                "point": [x, y, z],
                "kind": "actual",
                "index": index,
                "label": "row_%d" % index,
                "time_sec": time_sec,
                "wall_time": wall_time,
                "ros_time": ros_time,
                "mode": mode,
                "armed": armed,
            })
    return samples


class PointCloudIndex:
    def __init__(self, labeled_points, cell_size):
        self.cell_size = max(float(cell_size), 0.01)
        self.cells = defaultdict(list)
        self.count = 0
        for labeled in labeled_points:
            if len(labeled) == 2 and isinstance(labeled[0], str):
                label, point = labeled
            else:
                label, point = "", labeled
            key = self.cell_key(point)
            self.cells[key].append((label, as_point(point)))
            self.count += 1

    def cell_key(self, point):
        return (
            int(math.floor(float(point[0]) / self.cell_size)),
            int(math.floor(float(point[1]) / self.cell_size)),
            int(math.floor(float(point[2]) / self.cell_size)),
        )

    def nearest(self, point, max_distance=None):
        if self.count <= 0:
            return None, None
        base = self.cell_key(point)
        best_sq = None
        best_label = None
        if max_distance is None:
            max_expand = 128
            max_sq = None
        else:
            max_expand = int(math.ceil(float(max_distance) / self.cell_size))
            max_sq = float(max_distance) ** 2
        radius = 0
        while radius <= max_expand:
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    for dz in range(-radius, radius + 1):
                        if max(abs(dx), abs(dy), abs(dz)) != radius:
                            continue
                        for label, candidate in self.cells.get((base[0] + dx, base[1] + dy, base[2] + dz), []):
                            value = squared_distance(point, candidate)
                            if max_sq is not None and value > max_sq:
                                continue
                            if best_sq is None or value < best_sq:
                                best_sq = value
                                best_label = label
            if best_sq is not None and max_distance is None and math.sqrt(best_sq) <= radius * self.cell_size:
                break
            radius += 1
        if best_sq is None:
            return None, None
        return math.sqrt(best_sq), best_label


def sample_bruteforce_cloud_distance(point, labeled_points):
    best_sq = None
    best_label = None
    for label, candidate in labeled_points:
        value = squared_distance(point, candidate)
        if best_sq is None or value < best_sq:
            best_sq = value
            best_label = label
    if best_sq is None:
        return None, None
    return math.sqrt(best_sq), best_label


def update_min_result(result, section, value, obstacle, sample, extra=None):
    if value is None:
        return
    metric = result[section]
    if metric["min_clearance_m"] is None or value < metric["min_clearance_m"]:
        metric["min_clearance_m"] = value
        metric["nearest_obstacle"] = obstacle.get("name") if obstacle else None
        metric["nearest_source_key"] = obstacle.get("source_key") if obstacle else None
        metric["sample"] = sample_summary(sample)
        if extra:
            metric.update(extra)


def update_cloud_result(result, section, value, label, sample, entry_radius):
    if value is None:
        return
    metric = result[section]
    if metric["min_cloud_distance_m"] is None or value < metric["min_cloud_distance_m"]:
        metric["min_cloud_distance_m"] = value
        metric["nearest_cloud_label"] = label
        metric["nearest_cloud_sample"] = sample_summary(sample)
    if value <= entry_radius:
        metric["cloud_entry_count"] += 1
        if metric["first_cloud_entry"] is None:
            metric["first_cloud_entry"] = sample_summary(sample)


def sample_summary(sample):
    summary = {
        "kind": sample.get("kind"),
        "index": sample.get("index"),
        "label": sample.get("label"),
        "point": [round(float(v), 4) for v in sample.get("point", [0.0, 0.0, 0.0])],
    }
    for key in ("segment_index", "segment_u", "time_sec", "wall_time", "ros_time", "mode", "armed"):
        if key in sample and sample[key] is not None:
            value = sample[key]
            summary[key] = round(float(value), 4) if isinstance(value, float) else value
    return summary


def empty_metric():
    return {
        "min_clearance_m": None,
        "nearest_obstacle": None,
        "nearest_source_key": None,
        "sample": None,
        "geometry_entry_count": 0,
        "first_geometry_entry": None,
        "min_cloud_distance_m": None,
        "nearest_cloud_label": None,
        "nearest_cloud_sample": None,
        "cloud_entry_count": 0,
        "first_cloud_entry": None,
    }


def evaluate_sample_set(name, samples, scene, static_obstacles, static_cloud_index,
                        dynamic_mode="auto", dynamic_times=None, use_sample_time_for_dynamic=False,
                        cloud_entry_radius_m=0.125, cloud_search_radius_m=1.0,
                        include_cloud_distance=True):
    result = {
        "sample_count": len(samples),
        "static": empty_metric(),
        "dynamic": empty_metric(),
    }
    if not samples:
        return result

    for sample in samples:
        point = sample["point"]
        static_value, static_obstacle = min_obstacle_distance(point, static_obstacles)
        update_min_result(result, "static", static_value, static_obstacle, sample)
        if static_value is not None and static_value <= 0.0:
            result["static"]["geometry_entry_count"] += 1
            if result["static"]["first_geometry_entry"] is None:
                result["static"]["first_geometry_entry"] = sample_summary(sample)

        if include_cloud_distance and static_cloud_index is not None:
            cloud_value, cloud_label = static_cloud_index.nearest(point, cloud_search_radius_m)
            update_cloud_result(result, "static", cloud_value, cloud_label, sample, cloud_entry_radius_m)

        if use_sample_time_for_dynamic:
            times = [sample.get("time_sec", 0.0) or 0.0]
        else:
            times = dynamic_times or []
        for dynamic_time in times:
            dynamic_obstacles = dynamic_planner_obstacles_at(scene, dynamic_time, dynamic_mode)
            dynamic_value, dynamic_obstacle = min_obstacle_distance(point, dynamic_obstacles)
            update_min_result(
                result, "dynamic", dynamic_value, dynamic_obstacle, sample,
                extra={"nearest_dynamic_time_sec": dynamic_time})
            if dynamic_value is not None and dynamic_value <= 0.0:
                result["dynamic"]["geometry_entry_count"] += 1
                if result["dynamic"]["first_geometry_entry"] is None:
                    entered = sample_summary(sample)
                    entered["dynamic_time_sec"] = round(float(dynamic_time), 4)
                    result["dynamic"]["first_geometry_entry"] = entered
            if (include_cloud_distance and dynamic_value is not None and
                    dynamic_value <= cloud_search_radius_m):
                dynamic_points = scene_dynamic_cloud_points(scene, dynamic_time_sec=dynamic_time, include_labels=True)
                if not dynamic_points:
                    continue
                cloud_value, cloud_label = sample_bruteforce_cloud_distance(point, dynamic_points)
                update_cloud_result(result, "dynamic", cloud_value, cloud_label, sample, cloud_entry_radius_m)

    result["name"] = name
    return result


def threshold_value(scene, explicit, key, default):
    if explicit is not None:
        return float(explicit)
    return float((scene.get("acceptance") or {}).get(key, default))


def clearance_failures(metrics, min_static, min_dynamic):
    failures = []
    for name, result in metrics.items():
        static = result.get("static", {})
        if static.get("geometry_entry_count", 0) > 0:
            failures.append("%s enters static planner-visible geometry" % name)
        if static.get("cloud_entry_count", 0) > 0:
            failures.append("%s touches static planner-visible cloud samples" % name)
    return failures


def clearance_threshold_shortfalls(metrics, min_static, min_dynamic):
    shortfalls = []
    for name, result in metrics.items():
        static = result.get("static", {})
        dynamic = result.get("dynamic", {})
        static_min = static.get("min_clearance_m")
        dynamic_min = dynamic.get("min_clearance_m")
        if static_min is not None and static_min < min_static:
            shortfalls.append("%s static clearance %.3f m below %.3f m" % (name, static_min, min_static))
        if dynamic_min is not None and dynamic_min < min_dynamic:
            shortfalls.append("%s dynamic clearance %.3f m below %.3f m" % (name, dynamic_min, min_dynamic))
    return shortfalls


def evaluate_clearance(scene_or_path, trajectory_csv=None, dynamic_mode="auto",
                       sample_spacing_m=0.25, dynamic_samples=64, dynamic_horizon_sec=None,
                       min_obstacle_distance_m=None, min_dynamic_obstacle_distance_m=None,
                       actual_filter="armed_offboard", include_cloud_distance=True,
                       cloud_entry_radius_m=None, cloud_search_radius_m=None):
    scene = load_scene(scene_or_path) if isinstance(scene_or_path, str) else scene_or_path
    acceptance = scene.get("acceptance") or {}
    min_static = threshold_value(scene, min_obstacle_distance_m, "min_obstacle_distance_m", 0.0)
    min_dynamic = threshold_value(scene, min_dynamic_obstacle_distance_m, "min_dynamic_obstacle_distance_m", min_static)
    resolution = float(scene.get("cloud_resolution", 0.25))
    entry_radius = float(cloud_entry_radius_m) if cloud_entry_radius_m is not None else resolution / 2.0
    search_radius = float(cloud_search_radius_m) if cloud_search_radius_m is not None else max(
        entry_radius, min_static, min_dynamic, resolution)
    static_obstacles = static_planner_obstacles(scene)
    dynamic_times = dynamic_sample_times(scene, dynamic_samples, dynamic_horizon_sec)
    static_cloud_index = None
    static_cloud_points = []
    if include_cloud_distance:
        static_cloud_points = scene_cloud_points(scene, include_labels=True, include_dynamic=False)
        static_cloud_index = PointCloudIndex(static_cloud_points, max(resolution, entry_radius, 0.25))

    planned_waypoints = waypoint_samples(scene)
    planned_polyline = planned_polyline_samples(scene, sample_spacing_m)
    actual_samples = trajectory_samples_from_csv(trajectory_csv, actual_filter) if trajectory_csv else []

    metrics = {
        "planned_waypoints": evaluate_sample_set(
            "planned_waypoints", planned_waypoints, scene, static_obstacles, static_cloud_index,
            dynamic_mode=dynamic_mode, dynamic_times=dynamic_times,
            use_sample_time_for_dynamic=False, cloud_entry_radius_m=entry_radius,
            cloud_search_radius_m=search_radius, include_cloud_distance=include_cloud_distance),
        "planned_polyline": evaluate_sample_set(
            "planned_polyline", planned_polyline, scene, static_obstacles, static_cloud_index,
            dynamic_mode=dynamic_mode, dynamic_times=dynamic_times,
            use_sample_time_for_dynamic=False, cloud_entry_radius_m=entry_radius,
            cloud_search_radius_m=search_radius, include_cloud_distance=include_cloud_distance),
    }
    if trajectory_csv:
        metrics["actual_trajectory"] = evaluate_sample_set(
            "actual_trajectory", actual_samples, scene, static_obstacles, static_cloud_index,
            dynamic_mode=dynamic_mode, dynamic_times=None, use_sample_time_for_dynamic=True,
            cloud_entry_radius_m=entry_radius, cloud_search_radius_m=search_radius,
            include_cloud_distance=include_cloud_distance)

    failures = clearance_failures(metrics, min_static, min_dynamic)
    threshold_shortfalls = clearance_threshold_shortfalls(metrics, min_static, min_dynamic)
    if trajectory_csv and not actual_samples:
        failures.append("actual_trajectory has no samples after filter %s" % actual_filter)

    return {
        "ok": not failures,
        "scene": os.path.abspath(scene.get("_scene_path", "")) if scene.get("_scene_path") else None,
        "trajectory_csv": os.path.abspath(trajectory_csv) if trajectory_csv else None,
        "dynamic_mode": dynamic_mode,
        "thresholds": {
            "min_obstacle_distance_m": min_static,
            "min_dynamic_obstacle_distance_m": min_dynamic,
            "cloud_entry_radius_m": entry_radius,
            "cloud_search_radius_m": search_radius,
            "sample_spacing_m": float(sample_spacing_m),
            "dynamic_samples": int(dynamic_samples),
            "dynamic_horizon_sec": float(dynamic_horizon_sec) if dynamic_horizon_sec is not None else default_dynamic_horizon_sec(scene),
            "actual_filter": actual_filter,
            "include_cloud_distance": bool(include_cloud_distance),
        },
        "planner_visible": {
            "static_geometry_obstacles": len(static_obstacles),
            "static_cloud_points": len(static_cloud_points),
            "dynamic_obstacles": len(scene_dynamic_obstacles(scene)) if dynamic_mode_enabled(scene, dynamic_mode) else 0,
            "dynamic_sample_times": len(dynamic_times),
        },
        "metrics": metrics,
        "failure_reasons": failures,
        "threshold_shortfalls": threshold_shortfalls,
        "policy_note": (
            "ok gates planner-visible geometry/cloud entry only. "
            "min_obstacle_distance_m shortfalls are telemetry unless separately promoted by the user."
        ),
        "acceptance": {
            "min_obstacle_distance_m": acceptance.get("min_obstacle_distance_m"),
            "min_dynamic_obstacle_distance_m": acceptance.get("min_dynamic_obstacle_distance_m"),
        },
    }
