#!/usr/bin/env python3
import argparse
import copy
import csv
import json
import math
import os
import re
import shlex
import sys

import yaml

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, os.pardir))
DEFAULT_ROUTES_DIR = os.path.join(PACKAGE_ROOT, "config", "routes")
DEFAULT_SCENE = os.path.join(PACKAGE_ROOT, "config", "scenes", "level_m_gps_assets_quick_complex.yaml")
DEFAULT_ROUTE_ID = "classic_p0_p8"
ROUTE_ALIASES = {
    "mixed": DEFAULT_ROUTE_ID,
    "classic": DEFAULT_ROUTE_ID,
    "p0_p8": DEFAULT_ROUTE_ID,
    "p0-p8": DEFAULT_ROUTE_ID,
    "classic-p0-p8": DEFAULT_ROUTE_ID,
    "classic_p0_p8": DEFAULT_ROUTE_ID,
    "default": DEFAULT_ROUTE_ID,
    "stable": DEFAULT_ROUTE_ID,
    "stable-demo": DEFAULT_ROUTE_ID,
    "stable_demo": DEFAULT_ROUTE_ID,
    "stable-demo-route": DEFAULT_ROUTE_ID,
    "stable_demo_route": DEFAULT_ROUTE_ID,
    "mixed-comprehensive": DEFAULT_ROUTE_ID,
    "mixed_comprehensive": DEFAULT_ROUTE_ID,
    "mixed-comprehensive-route": DEFAULT_ROUTE_ID,
    "mixed_comprehensive_route": DEFAULT_ROUTE_ID,
    "route2": DEFAULT_ROUTE_ID,
    "route-2": DEFAULT_ROUTE_ID,
    "route_2": DEFAULT_ROUTE_ID,
    "default2": DEFAULT_ROUTE_ID,
    "default-2": DEFAULT_ROUTE_ID,
    "default_route_2": DEFAULT_ROUTE_ID,
    "route3": DEFAULT_ROUTE_ID,
    "route-3": DEFAULT_ROUTE_ID,
    "route_3": DEFAULT_ROUTE_ID,
    "default3": DEFAULT_ROUTE_ID,
    "default-3": DEFAULT_ROUTE_ID,
    "default_route_3": DEFAULT_ROUTE_ID,
    "obstacle": DEFAULT_ROUTE_ID,
    "obstacle-showcase": DEFAULT_ROUTE_ID,
    "obstacle_showcase": DEFAULT_ROUTE_ID,
    "obstacle-showcase-route": DEFAULT_ROUTE_ID,
    "obstacle_showcase_route": DEFAULT_ROUTE_ID,
}
OPTIONAL_METADATA_KEYS = (
    "display_name",
    "ui_order",
    "compatibility_aliases",
    "validation_basis",
)


def read_yaml(path):
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def write_yaml(path, payload):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        yaml.safe_dump(payload, handle, default_flow_style=False, sort_keys=False)


def write_json(path, payload):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")


def sanitize_id(value):
    text = str(value or "").strip().lower()
    text = re.sub(r"[^a-z0-9_.-]+", "_", text)
    text = text.strip("._-")
    return text or "custom_route"


def route_label(index, waypoint):
    explicit = waypoint.get("label")
    if explicit:
        return str(explicit)
    name = str(waypoint.get("name") or "")
    match = re.match(r"^p(\d+)(?:_|$)", name.lower())
    if match:
        return "P%s" % match.group(1)
    return "W%d" % index


def route_length(waypoints):
    total = 0.0
    previous = None
    for waypoint in waypoints:
        pos = waypoint["position"]
        if previous is not None:
            total += math.sqrt(sum((float(pos[i]) - float(previous[i])) ** 2 for i in range(3)))
        previous = pos
    return total


def finite_float(value, context):
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise ValueError("%s must be a finite number" % context)
    if not math.isfinite(result):
        raise ValueError("%s must be a finite number" % context)
    return result


def normalize_waypoint(raw, index, default_acceptance):
    if not isinstance(raw, dict):
        raise ValueError("waypoints[%d] must be a mapping" % index)
    if "position" not in raw:
        raise ValueError("waypoints[%d] requires position" % index)
    position = raw["position"]
    if not isinstance(position, (list, tuple)) or len(position) != 3:
        raise ValueError("waypoints[%d].position must contain 3 numbers" % index)
    waypoint = dict(raw)
    waypoint["position"] = [
        finite_float(position[0], "waypoints[%d].position[0]" % index),
        finite_float(position[1], "waypoints[%d].position[1]" % index),
        finite_float(position[2], "waypoints[%d].position[2]" % index),
    ]
    waypoint["name"] = str(waypoint.get("name") or "waypoint_%d" % index)
    waypoint["label"] = route_label(index, waypoint)
    waypoint["yaw"] = finite_float(waypoint.get("yaw", 0.0), "waypoints[%d].yaw" % index)
    waypoint["radius"] = finite_float(
        waypoint.get("radius", default_acceptance.get("position_tolerance_m", 1.0)),
        "waypoints[%d].radius" % index,
    )
    waypoint["hold_time"] = finite_float(waypoint.get("hold_time", 0.0), "waypoints[%d].hold_time" % index)
    waypoint["max_duration_sec"] = finite_float(
        waypoint.get("max_duration_sec", 0.0),
        "waypoints[%d].max_duration_sec" % index,
    )
    return waypoint


def normalize_profile(profile, path, base_scene=None):
    if not isinstance(profile, dict):
        raise ValueError("route profile must be a mapping: %s" % path)
    waypoints_raw = profile.get("waypoints") or []
    if len(waypoints_raw) < 2:
        raise ValueError("route profile requires at least two waypoints: %s" % path)
    acceptance = (base_scene or {}).get("acceptance") or {}
    waypoints = [normalize_waypoint(item, index, acceptance) for index, item in enumerate(waypoints_raw)]
    route_id = sanitize_id(profile.get("route_id") or os.path.splitext(os.path.basename(path))[0])
    name = str(profile.get("name") or route_id)
    final_label = str(profile.get("final_label") or waypoints[-1]["label"])
    normalized = {
        "schema": str(profile.get("schema") or "f250_route_profile_v1"),
        "route_id": route_id,
        "name": name,
        "description": str(profile.get("description") or ""),
        "default": bool(profile.get("default", route_id == DEFAULT_ROUTE_ID)),
        "locked_baseline_compatibility": bool(profile.get("locked_baseline_compatibility", False)),
        "scene_level": str(profile.get("scene_level") or (base_scene or {}).get("scene_level") or "level_m_gps_assets_quick_complex"),
        "final_label": final_label,
        "profile_path": os.path.abspath(path),
        "profile_source": "builtin" if is_builtin_profile_path(path) else "custom",
        "waypoints": waypoints,
        "waypoint_count": len(waypoints),
        "first_label": waypoints[0]["label"],
        "total_route_length_m": route_length(waypoints),
    }
    normalized["display_name"] = str(profile.get("display_name") or name)
    normalized["ui_order"] = int(profile.get("ui_order", 999))
    aliases = profile.get("compatibility_aliases") or []
    if isinstance(aliases, (list, tuple)):
        normalized["compatibility_aliases"] = [str(item) for item in aliases]
    else:
        normalized["compatibility_aliases"] = [str(aliases)]
    normalized["validation_basis"] = str(profile.get("validation_basis") or "")
    return normalized


def is_builtin_profile_path(path):
    try:
        common = os.path.commonpath([os.path.abspath(path), os.path.abspath(DEFAULT_ROUTES_DIR)])
    except ValueError:
        return False
    return common == os.path.abspath(DEFAULT_ROUTES_DIR)


def builtin_profile_path(route_id):
    resolved = ROUTE_ALIASES.get(str(route_id or "").strip().lower(), str(route_id or "").strip())
    if not resolved:
        resolved = DEFAULT_ROUTE_ID
    if os.path.sep in resolved or resolved.endswith(".yaml") or resolved.endswith(".yml"):
        return os.path.abspath(os.path.expanduser(resolved))
    return os.path.join(DEFAULT_ROUTES_DIR, "%s.yaml" % resolved)


def resolve_profile_path(route_id=None, route_profile=None):
    if route_profile:
        return os.path.abspath(os.path.expanduser(route_profile))
    return builtin_profile_path(route_id or DEFAULT_ROUTE_ID)


def load_route_profile(route_id=None, route_profile=None, base_scene_path=DEFAULT_SCENE):
    base_scene = read_yaml(base_scene_path)
    profile_path = resolve_profile_path(route_id, route_profile)
    if not os.path.exists(profile_path):
        raise FileNotFoundError("missing route profile: %s" % profile_path)
    profile = normalize_profile(read_yaml(profile_path), profile_path, base_scene=base_scene)
    profile["base_scene"] = os.path.abspath(base_scene_path)
    return profile


def apply_route_to_scene(base_scene_path, profile, output_scene_path):
    scene = read_yaml(base_scene_path)
    effective = copy.deepcopy(scene)
    effective["waypoints"] = []
    for waypoint in profile["waypoints"]:
        item = {
            "name": waypoint["name"],
            "label": waypoint["label"],
            "position": [float(value) for value in waypoint["position"]],
            "yaw": float(waypoint.get("yaw", 0.0)),
            "radius": float(waypoint.get("radius", 1.0)),
            "hold_time": float(waypoint.get("hold_time", 0.0)),
            "max_duration_sec": float(waypoint.get("max_duration_sec", 0.0)),
        }
        effective["waypoints"].append(item)
    effective["route_profile"] = profile_metadata(profile)
    write_yaml(output_scene_path, effective)
    return os.path.abspath(output_scene_path)


def profile_metadata(profile):
    keys = (
        "route_id", "name", "description", "profile_path", "profile_source",
        "base_scene", "scene_level", "waypoint_count", "first_label", "final_label",
        "total_route_length_m", "locked_baseline_compatibility",
        "display_name", "ui_order", "compatibility_aliases", "validation_basis",
    )
    return {key: profile.get(key) for key in keys}


def hover_metadata(profile):
    first = profile["waypoints"][0]
    x, y, z = [float(value) for value in first["position"][:3]]
    yaw = float(first.get("yaw", 0.0))
    return {
        "hover_x": x,
        "hover_y": y,
        "hover_z": z,
        "hover_yaw": yaw,
        "hover_target": "%.6g,%.6g,%.6g,%.6g" % (x, y, z, yaw),
        "spawn_x": x,
        "spawn_y": y,
        "spawn_z": max(0.0, z - 5.18),
        "spawn_yaw": yaw,
    }


def summary(profile, effective_scene=None):
    data = profile_metadata(profile)
    data.update(hover_metadata(profile))
    data["effective_scene"] = os.path.abspath(effective_scene) if effective_scene else None
    data["waypoints"] = [
        {
            "index": index,
            "label": waypoint["label"],
            "name": waypoint["name"],
            "position": waypoint["position"],
            "yaw": waypoint.get("yaw", 0.0),
            "radius": waypoint.get("radius", 1.0),
            "hold_time": waypoint.get("hold_time", 0.0),
            "max_duration_sec": waypoint.get("max_duration_sec", 0.0),
        }
        for index, waypoint in enumerate(profile["waypoints"])
    ]
    return data


def write_env(path, profile, effective_scene=None, selected_by="default"):
    data = summary(profile, effective_scene)
    fields = [
        ("ROUTE_ID", data["route_id"]),
        ("ROUTE_NAME", data["name"]),
        ("ROUTE_DISPLAY_NAME", data.get("display_name") or data["name"]),
        ("ROUTE_UI_ORDER", data.get("ui_order") or ""),
        ("ROUTE_PROFILE", data["profile_path"]),
        ("ROUTE_PROFILE_SOURCE", data["profile_source"]),
        ("ROUTE_SELECTED_BY", selected_by),
        ("ROUTE_BASE_SCENE", data["base_scene"]),
        ("ROUTE_EFFECTIVE_SCENE", data["effective_scene"] or ""),
        ("ROUTE_WAYPOINT_COUNT", data["waypoint_count"]),
        ("ROUTE_FIRST_LABEL", data["first_label"]),
        ("ROUTE_FINAL_LABEL", data["final_label"]),
        ("ROUTE_TOTAL_LENGTH_M", "%.9g" % float(data["total_route_length_m"])),
        ("ROUTE_LOCKED_BASELINE_COMPATIBILITY", "true" if data["locked_baseline_compatibility"] else "false"),
        ("ROUTE_HOVER_TARGET", data["hover_target"]),
        ("ROUTE_HOVER_X", "%.9g" % float(data["hover_x"])),
        ("ROUTE_HOVER_Y", "%.9g" % float(data["hover_y"])),
        ("ROUTE_HOVER_Z", "%.9g" % float(data["hover_z"])),
        ("ROUTE_HOVER_YAW", "%.9g" % float(data["hover_yaw"])),
        ("ROUTE_SPAWN_X", "%.9g" % float(data["spawn_x"])),
        ("ROUTE_SPAWN_Y", "%.9g" % float(data["spawn_y"])),
        ("ROUTE_SPAWN_Z", "%.9g" % float(data["spawn_z"])),
        ("ROUTE_SPAWN_YAW", "%.9g" % float(data["spawn_yaw"])),
    ]
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        for key, value in fields:
            handle.write("%s=%s\n" % (key, shlex.quote("" if value is None else str(value))))


def write_csv(path, profile):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    fields = ["index", "label", "name", "x", "y", "z", "yaw_rad", "radius_m", "hold_time_sec", "max_duration_sec"]
    with open(path, "w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for index, waypoint in enumerate(profile["waypoints"]):
            writer.writerow({
                "index": index,
                "label": waypoint["label"],
                "name": waypoint["name"],
                "x": waypoint["position"][0],
                "y": waypoint["position"][1],
                "z": waypoint["position"][2],
                "yaw_rad": waypoint.get("yaw", 0.0),
                "radius_m": waypoint.get("radius", 1.0),
                "hold_time_sec": waypoint.get("hold_time", 0.0),
                "max_duration_sec": waypoint.get("max_duration_sec", 0.0),
            })


def list_profiles():
    rows = []
    if not os.path.isdir(DEFAULT_ROUTES_DIR):
        print(f"WARNING: routes directory does not exist: {DEFAULT_ROUTES_DIR}", file=sys.stderr)
        return rows
    for name in sorted(os.listdir(DEFAULT_ROUTES_DIR)):
        if not name.endswith((".yaml", ".yml")):
            continue
        path = os.path.join(DEFAULT_ROUTES_DIR, name)
        try:
            profile = load_route_profile(route_profile=path)
            if profile.get("route_id") == DEFAULT_ROUTE_ID:
                rows.append(profile)
        except Exception as exc:
            rows.append({"route_id": os.path.splitext(name)[0], "name": name, "error": str(exc), "profile_path": path})
    rows.sort(key=lambda item: (int(item.get("ui_order", 999)), str(item.get("route_id") or "")))
    return rows


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Resolve F250 editable route profiles.")
    parser.add_argument("--route-id", default=None)
    parser.add_argument("--route-profile", default=None)
    parser.add_argument("--base-scene", default=DEFAULT_SCENE)
    parser.add_argument("--effective-scene", default=None)
    parser.add_argument("--summary-json", default=None)
    parser.add_argument("--env-out", default=None)
    parser.add_argument("--csv-out", default=None)
    parser.add_argument("--selected-by", default="default")
    parser.add_argument("--print-shell", action="store_true")
    parser.add_argument("--print-json", action="store_true")
    parser.add_argument("--list", action="store_true")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv or sys.argv[1:])
    if args.list:
        print(json.dumps([profile_metadata(item) for item in list_profiles()], indent=2, sort_keys=True))
        return 0
    try:
        profile = load_route_profile(args.route_id, args.route_profile, args.base_scene)
    except (OSError, ValueError) as exc:
        print("f250_route_profiles: %s" % exc, file=sys.stderr)
        return 2
    effective_scene = None
    if args.effective_scene:
        effective_scene = apply_route_to_scene(args.base_scene, profile, args.effective_scene)
    data = summary(profile, effective_scene)
    if args.summary_json:
        write_json(args.summary_json, data)
    if args.env_out:
        write_env(args.env_out, profile, effective_scene, selected_by=args.selected_by)
    if args.csv_out:
        write_csv(args.csv_out, profile)
    if args.print_shell:
        for key, value in data.items():
            if key == "waypoints":
                continue
            print("%s=%s" % (key, "" if value is None else value))
    if args.print_json or not (args.summary_json or args.env_out or args.csv_out or args.print_shell):
        print(json.dumps(data, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
