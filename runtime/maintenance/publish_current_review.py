#!/usr/bin/env python3
"""Publish accepted F250 task evidence into evidence/current.

This tool deliberately publishes a whitelist of machine evidence instead of
copying whole run directories. Runtime logs, GUI captures, reports, launch
context copies, and other transient files stay in runtime_state/work and are not
retained evidence.
"""
import argparse
import json
import os
import shutil
import subprocess
from datetime import datetime, timezone
from pathlib import Path


def utc_now():
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def resolve_project_root():
    env_root = os.environ.get("F250_PROJECT_ROOT")
    if env_root:
        return Path(env_root).expanduser().resolve()
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "catkin_ws/src/f250_maritime_uav_sim").is_dir():
            return parent
    return Path.cwd().resolve()


def runtime_state_dir(root):
    return Path(os.environ.get("F250_RUNTIME_STATE_DIR", root / "runtime_state")).expanduser().resolve()


def runtime_work_dir(root):
    state = runtime_state_dir(root)
    return Path(os.environ.get("RUN_ROOT", state / "work")).expanduser().resolve()


def current_evidence_dir(root):
    return Path(os.environ.get("F250_EVIDENCE_CURRENT_DIR", root / "evidence/current")).expanduser().resolve()


def active_sensor_env(root):
    state = runtime_state_dir(root)
    return Path(os.environ.get("F250_ACTIVE_SENSOR_ENV", state / "active_sensor.env")).expanduser().resolve()


def normalize_kind(value):
    if value == "fc":
        return "fc_3_10"
    if value == "flight_control":
        return "fc_3_10"
    if value == "route":
        return "route_p0_p8"
    if value not in {"route_p0_p8", "fc_3_10"}:
        raise SystemExit(f"unsupported kind: {value}")
    return value


def read_json(path):
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except Exception:
        return {}


def read_env(path):
    data = {}
    try:
        with open(path, "r", encoding="utf-8") as handle:
            for line in handle:
                line = line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                key, value = line.split("=", 1)
                data[key] = value.strip().strip('"').strip("'")
    except FileNotFoundError:
        pass
    return data


def write_json(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_env(path, data, keys):
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = []
    for key in keys:
        if key in data and data[key] not in (None, ""):
            lines.append(f"{key}={data[key]}")
    path.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")


def copy_required(src, dst):
    src = Path(src)
    if not src.is_file():
        raise SystemExit(f"missing required evidence file: {src}")
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)


def copy_optional(src, dst):
    src = Path(src)
    if src.is_file():
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)
        return True
    return False


def bool_env(name, default=True):
    raw = os.environ.get(name)
    if raw is None:
        return default
    return raw.strip().lower() not in {"", "0", "false", "no", "off"}


def refresh_result_plot(root, current, quiet=False):
    if not bool_env("F250_REFRESH_RESULT_PLOT_AFTER_PUBLISH", True):
        return

    route_dir = current / "route_p0_p8"
    trajectory = route_dir / "measurements/actual_trajectory.csv"
    if not route_dir.is_dir() or not trajectory.is_file():
        return

    renderer = root / "map_authority/p0p8_clean_scene/render_map.py"
    target = root / "map_authority/p0p8_clean_scene/route_result.png"
    if not renderer.is_file():
        raise SystemExit(f"missing map renderer: {renderer}")

    env = {
        **os.environ,
        "F250_PROJECT_ROOT": str(root),
        "F250_EVIDENCE_CURRENT_DIR": str(current),
        "PYTHONDONTWRITEBYTECODE": "1",
    }
    subprocess.run(
        ["python3", str(renderer), "--target", "result", "--current-dir", str(current)],
        check=True,
        cwd=str(renderer.parent),
        env=env,
    )
    if not target.is_file():
        raise SystemExit(f"result plot was not generated: {target}")
    subprocess.run([str(root / "maintenance/generate_current_index.py"), "--quiet"], check=True, env=env)
    if not quiet:
        print(target)


def route_status_payload(run_dir):
    status = read_env(run_dir / "status.env") or read_env(run_dir / "route_status.env")
    status.setdefault("task", "route")
    status.setdefault("vehicle", "f250")
    return status


def fc_status_payload(run_dir):
    status = read_env(run_dir / "status.env")
    status.setdefault("task", "flight_control")
    status.setdefault("vehicle", "f250")
    return status


def publish_route(run_src, task_dir):
    status = route_status_payload(run_src)
    route_dir = task_dir
    if route_dir.exists():
        shutil.rmtree(route_dir)
    route_dir.mkdir(parents=True, exist_ok=True)

    status_keys = [
        "state", "task", "updated_at", "run_label", "vehicle", "dry_run",
        "route_id", "route_name", "route_waypoint_count", "route_first_label",
        "route_final_label", "progress", "final_completed", "p8_completed",
        "static_obstacle_safety", "keypoint_error_mean_m", "keypoint_error_max_m",
        "endpoint_error_m", "route_acceptance_ok",
        "route_acceptance_excludes_metric_3_10", "route_acceptance_excludes_yaw",
        "dynamic_boat_clearance_role", "sensor", "sensor_label", "perception_source",
        "planner_cloud_topic", "raw_cloud_topic", "lidar_cloud_topic", "lidar_scan_topic",
        "depth_cloud_topic", "occupancy_topic",
    ]
    write_env(route_dir / "status.env", status, status_keys)

    copy_required(run_src / "route_waypoints.csv", route_dir / "inputs/route_waypoints.csv")
    copy_required(run_src / "route_effective_scene.yaml", route_dir / "inputs/effective_scene.yaml")
    copy_optional(run_src / "params.json", route_dir / "inputs/params.json")
    copy_required(run_src / "actual_trajectory.csv", route_dir / "measurements/actual_trajectory.csv")
    copy_required(run_src / "route_acceptance_summary.json", route_dir / "metrics/acceptance_summary.json")
    copy_required(run_src / "metric_summary.json", route_dir / "metrics/metric_summary.json")
    copy_required(run_src / "metrics.json", route_dir / "metrics/metrics_full.json")
    copy_required(run_src / "metric_waypoints.csv", route_dir / "metrics/waypoint_errors.csv")

    acceptance = read_json(route_dir / "metrics/acceptance_summary.json")
    metrics_full = read_json(route_dir / "metrics/metrics_full.json")
    updated = utc_now()
    manifest = {
        "schema": "f250_current_evidence_manifest_v1",
        "task": "route_p0_p8",
        "source_task": "route_run",
        "updated_at": updated,
        "state": status.get("state") or metrics_full.get("stop_reason"),
        "sensor": status.get("sensor") or acceptance.get("sensor"),
        "outcome": metrics_full.get("stop_reason") or acceptance.get("outcome"),
        "files": {
            "status": "status.env",
            "route_waypoints": "inputs/route_waypoints.csv",
            "effective_scene": "inputs/effective_scene.yaml",
            "params": "inputs/params.json" if (route_dir / "inputs/params.json").exists() else None,
            "actual_trajectory": "measurements/actual_trajectory.csv",
            "acceptance_summary": "metrics/acceptance_summary.json",
            "metric_summary": "metrics/metric_summary.json",
            "metrics_full": "metrics/metrics_full.json",
            "waypoint_errors": "metrics/waypoint_errors.csv",
        },
    }
    write_json(route_dir / "manifest.json", manifest)


def publish_fc(run_src, task_dir):
    status = fc_status_payload(run_src)
    active_sensor = read_env(active_sensor_env(resolve_project_root()))
    if not status.get("sensor"):
        status["sensor"] = active_sensor.get("sensor") or active_sensor.get("PERCEPTION_SOURCE") or ""
    if not status.get("perception_source"):
        status["perception_source"] = status.get("sensor") or ""
    if not status.get("sensor_label"):
        status["sensor_label"] = active_sensor.get("sensor_label") or ""
    fc_dir = task_dir
    if fc_dir.exists():
        shutil.rmtree(fc_dir)
    fc_dir.mkdir(parents=True, exist_ok=True)

    status_keys = [
        "state", "task", "runtime_active", "updated_at", "run_label", "vehicle",
        "metric", "dry_run", "geometry_check", "speed_only", "disable_velocity_prealign",
        "route_id", "route_name", "route_final_label", "hover_target", "sensor",
        "sensor_label", "perception_source", "route_acceptance_written",
    ]
    write_env(fc_dir / "status.env", status, status_keys)

    copy_required(run_src / "fc_3_10_decagon_points.csv", fc_dir / "inputs/decagon_points.csv")
    copy_required(run_src / "fc_3_10_samples.csv", fc_dir / "measurements/samples.csv")
    copy_required(run_src / "fc_3_10_phases.csv", fc_dir / "measurements/phases.csv")
    copy_required(run_src / "fc_3_10_summary.json", fc_dir / "metrics/summary.json")
    copy_required(run_src / "fc_3_10_geometry_audit.json", fc_dir / "metrics/geometry_audit.json")

    summary = read_json(fc_dir / "metrics/summary.json")
    updated = utc_now()
    manifest = {
        "schema": "f250_current_evidence_manifest_v1",
        "task": "fc_3_10",
        "source_task": "flight_control_run",
        "updated_at": updated,
        "state": status.get("state") or summary.get("run_state"),
        "sensor": status.get("source_p0_sensor") or status.get("sensor") or status.get("perception_source"),
        "outcome": summary.get("result"),
        "files": {
            "status": "status.env",
            "decagon_points": "inputs/decagon_points.csv",
            "samples": "measurements/samples.csv",
            "phases": "measurements/phases.csv",
            "summary": "metrics/summary.json",
            "geometry_audit": "metrics/geometry_audit.json",
        },
    }
    write_json(fc_dir / "manifest.json", manifest)


def main():
    parser = argparse.ArgumentParser(description="Publish one F250 task into evidence/current without retaining runtime history.")
    parser.add_argument("--kind", required=True, choices=["route", "route_p0_p8", "flight_control", "fc", "fc_3_10"])
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--p0-run-dir")  # accepted for caller compatibility; no longer copied into evidence
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()

    root = resolve_project_root()
    kind = normalize_kind(args.kind)
    run_src = Path(args.run_dir).expanduser().resolve()
    if not run_src.is_dir():
        raise SystemExit(f"missing run directory: {run_src}")

    current = current_evidence_dir(root)
    current.mkdir(parents=True, exist_ok=True)
    if kind == "route_p0_p8":
        publish_route(run_src, current / "route_p0_p8")
    elif kind == "fc_3_10":
        publish_fc(run_src, current / "fc_3_10")

    env = {
        **os.environ,
        "F250_PROJECT_ROOT": str(root),
        "RUN_ROOT": str(runtime_work_dir(root)),
        "F250_RUNTIME_STATE_DIR": str(runtime_state_dir(root)),
        "F250_ACTIVE_SENSOR_ENV": str(active_sensor_env(root)),
        "F250_EVIDENCE_CURRENT_DIR": str(current),
    }
    subprocess.run([str(root / "maintenance/sanitize_current_evidence.py"), "--quiet"], check=True, env=env)
    subprocess.run([str(root / "maintenance/generate_current_index.py"), "--quiet"], check=True, env=env)
    refresh_result_plot(root, current, quiet=args.quiet)
    if not args.quiet:
        print(current / kind)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
