#!/usr/bin/env python3
"""Build the external tester delivery package from evidence/current.

The package is a human-facing derivative, not primary evidence. It keeps full
sample timing for route and FC review while simplifying columns and filenames.
Primary machine evidence remains under evidence/current.
"""
import argparse
import ast
import csv
import json
import os
import shutil
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


def read_json(path):
    with Path(path).open("r", encoding="utf-8") as handle:
        return json.load(handle)


def copy_selected_rows(src_csv, dst_csv, columns, row_filter=None):
    src_csv = Path(src_csv)
    if not src_csv.is_file():
        raise FileNotFoundError(src_csv)
    dst_csv.parent.mkdir(parents=True, exist_ok=True)
    rows = 0
    with src_csv.open("r", encoding="utf-8", newline="") as src:
        reader = csv.DictReader(src)
        if not reader.fieldnames:
            raise ValueError(f"empty CSV header: {src_csv}")
        selected = [col for col in columns if col in reader.fieldnames]
        if not selected:
            raise ValueError(f"none of the requested columns exist in {src_csv}")
        with dst_csv.open("w", encoding="utf-8", newline="") as dst:
            writer = csv.DictWriter(dst, fieldnames=selected, extrasaction="ignore")
            writer.writeheader()
            for row in reader:
                if row_filter is not None and not row_filter(row):
                    continue
                writer.writerow({col: row.get(col, "") for col in selected})
                rows += 1
    return rows, selected


def read_component_windows(phases_csv, component):
    phases_csv = Path(phases_csv)
    if not phases_csv.is_file():
        return []
    rows = []
    with phases_csv.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            if row.get("evaluated_component") == component:
                rows.append(row)
    return rows


def add_summary_row(rows, task, metric, item, result, value="", unit="", target="", actual="", eval_window="", notes=""):
    rows.append({
        "task": task,
        "metric": metric,
        "item": item,
        "result": result,
        "value": value,
        "unit": unit,
        "target": target,
        "actual": actual,
        "eval_window": eval_window,
        "notes": notes,
    })


def fmt(value):
    if value is None:
        return ""
    if isinstance(value, float):
        return f"{value:.12g}"
    return str(value)


def pass_fail(value):
    if isinstance(value, bool):
        return "PASS" if value else "FAIL"
    if isinstance(value, str):
        upper = value.upper()
        if upper in {"PASS", "FAIL", "SAFE"}:
            return "PASS" if upper in {"PASS", "SAFE"} else "FAIL"
    return str(value)



def parse_literal(value):
    if value in (None, ""):
        return value
    try:
        return ast.literal_eval(value)
    except Exception:
        return value


def fmt_vector(values, unit=""):
    if not isinstance(values, (list, tuple)):
        return fmt(values)
    body = ", ".join(fmt(v) for v in values)
    return f"({body}) {unit}".strip()


def window_summary_values(metric, row):
    desired = parse_literal(row.get("desired", ""))
    actual = parse_literal(row.get("steady_actual", ""))
    if metric == "velocity":
        target = ""
        observed = ""
        if isinstance(desired, dict):
            target = f"{fmt(desired.get('target_speed_mps'))} m/s"
        if isinstance(actual, dict):
            observed = f"parallel {fmt(actual.get('mean_parallel_mps'))} m/s; cross {fmt(actual.get('mean_cross_mps'))} m/s"
        return target, observed
    if metric == "position":
        return fmt_vector(desired, "m"), fmt_vector(actual, "m")
    if metric == "yaw":
        target = f"{fmt(desired)} rad" if desired != "" else ""
        observed = ""
        if isinstance(actual, dict):
            observed = f"mean yaw error {fmt(actual.get('mean_wrapped_yaw_error_rad'))} rad"
        return target, observed
    return fmt(desired), fmt(actual)

def build_summary(current, generated_at):
    route = read_json(current / "route_p0_p8/metrics/acceptance_summary.json")
    fc = read_json(current / "fc_3_10/metrics/summary.json")
    route_status = {}
    fc_status = {}
    for path, target in [
        (current / "route_p0_p8/status.env", route_status),
        (current / "fc_3_10/status.env", fc_status),
    ]:
        if path.is_file():
            for line in path.read_text(encoding="utf-8").splitlines():
                if "=" in line:
                    key, value = line.split("=", 1)
                    target[key] = value

    terminal = route.get("terminal", {})
    errors = fc.get("errors_percent", {})
    rows = []
    add_summary_row(rows, "package", "metadata", "generated_at", "INFO", generated_at, "", notes="delivery package generation time")
    add_summary_row(rows, "package", "metadata", "source", "INFO", "evidence/current", "", notes="primary evidence remains in runtime evidence store")

    add_summary_row(rows, "route", "overall", "route_acceptance", pass_fail(route.get("ok")), route.get("ok"), "", notes=route_status.get("updated_at", ""))
    add_summary_row(rows, "route", "sensor", "sensor", "INFO", route.get("sensor", ""), "", notes=route.get("sensor_label", ""))
    add_summary_row(rows, "route", "3.6", "keypoint_error_mean", pass_fail(route.get("components", {}).get("metric_3_6_keypoint_error")), terminal.get("keypoint_error_mean_m"), "m", notes="mean P1-P7 waypoint error")
    add_summary_row(rows, "route", "3.6", "keypoint_error_max", pass_fail(route.get("components", {}).get("metric_3_6_keypoint_error")), terminal.get("keypoint_error_max_m"), "m", notes="max P1-P7 waypoint error")
    add_summary_row(rows, "route", "3.9", "endpoint_error", pass_fail(route.get("components", {}).get("metric_3_9_endpoint_error")), terminal.get("endpoint_error_m"), "m", notes="P8 endpoint error")
    add_summary_row(rows, "route", "3.7", "static_obstacle_safety", pass_fail(terminal.get("static_safe")), terminal.get("static", ""), "", notes="static obstacle safety")
    add_summary_row(rows, "route", "completion", "p8_completed", pass_fail(terminal.get("p8_completed")), terminal.get("progress", ""), "", notes="final waypoint completion")

    add_summary_row(rows, "flight_control", "overall", "fc_result", pass_fail(fc.get("result")), fc.get("result"), "", notes=fc_status.get("updated_at", ""))
    add_summary_row(rows, "flight_control", "3.10", "E3.10_selected", pass_fail(fc.get("result")), errors.get("E3.10_selected"), "%", notes="max(E_pos, E_vel_selected, E_yaw)")
    add_summary_row(rows, "flight_control", "position", "E_pos", pass_fail(errors.get("all_metric_windows_settled")), errors.get("E_pos"), "%", notes=f"settled {errors.get('E_pos_settled_count', '')}/10")
    add_summary_row(rows, "flight_control", "velocity", "E_vel_selected", pass_fail(errors.get("E_vel_all_windows_settled")), errors.get("E_vel_selected"), "%", notes=f"settled {errors.get('E_vel_settled_count', '')}/{errors.get('E_vel_window_count', '')}")
    add_summary_row(rows, "flight_control", "yaw", "E_yaw", pass_fail(errors.get("all_metric_windows_settled")), errors.get("E_yaw"), "%", notes=f"settled {errors.get('E_yaw_settled_count', '')}/10")
    add_summary_row(rows, "flight_control", "settling", "all_metric_windows_settled", pass_fail(errors.get("all_metric_windows_settled")), errors.get("all_metric_windows_settled"), "", notes="formal metric windows")

    component_info = [
        ("velocity", "E_vel", "speed_window"),
        ("position", "E_pos", "position_window"),
        ("yaw", "E_yaw", "yaw_window"),
    ]
    phases = current / "fc_3_10/measurements/phases.csv"
    for metric, component, item_prefix in component_info:
        for row in read_component_windows(phases, component):
            repeat = row.get("repeat", "")
            target, actual = window_summary_values(metric, row)
            add_summary_row(
                rows,
                "flight_control",
                metric,
                f"{item_prefix}_{repeat}",
                pass_fail(row.get("settled") == "True"),
                row.get("error_percent", ""),
                "%",
                target=target,
                actual=actual,
                eval_window=f"{row.get('eval_start_sec', '')}-{row.get('eval_end_sec', '')} s",
                notes=f"phase={row.get('phase', '')}",
            )
    return rows


def write_summary_csv(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = ["task", "metric", "item", "result", "value", "unit", "target", "actual", "eval_window", "notes"]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: fmt(row.get(key, "")) for key in fieldnames})


def build_route(current, out_dir):
    route_ev = current / "route_p0_p8"
    trajectory_cols = [
        "wall_time", "ros_time", "x", "y", "z", "vx", "vy", "vz", "actual_yaw",
        "expected_x", "expected_y", "expected_z", "expected_yaw",
        "setpoint_x", "setpoint_y", "setpoint_z", "setpoint_yaw",
        "position_error_to_expected_m", "position_error_to_setpoint_m",
        "velocity_error_to_expected_m", "yaw_error_rad", "yaw_error_to_setpoint_rad",
        "mode", "armed", "active_goal_index", "active_goal_distance_m",
        "cross_track_m", "along_track_m",
    ]
    waypoint_cols = [
        "index", "label", "name", "x", "y", "z", "yaw_rad", "radius_m",
        "nearest_distance_m", "nearest_x", "nearest_y", "nearest_z", "nearest_time_sec",
        "nearest_actual_yaw_rad", "nearest_yaw_error_rad", "active_nearest_distance_m",
        "metric_3_6_error_ratio", "metric_3_6_pass", "metric_3_9_final_error_ratio",
        "route_position_pass", "status", "finalized", "finalize_reason",
    ]
    trajectory_rows, _ = copy_selected_rows(route_ev / "measurements/actual_trajectory.csv", out_dir / "trajectory.csv", trajectory_cols)
    waypoint_rows, _ = copy_selected_rows(route_ev / "metrics/waypoint_errors.csv", out_dir / "waypoint_errors.csv", waypoint_cols)
    return {"trajectory_rows": trajectory_rows, "waypoint_rows": waypoint_rows}


def build_flight_control(current, out_dir):
    fc_ev = current / "fc_3_10"
    samples = fc_ev / "measurements/samples.csv"
    common = ["t_sec", "phase", "kind", "phase_elapsed_sec", "evaluated_component", "repeat"]
    speed_cols = common + [
        "desired_vx", "desired_vy", "desired_vz", "actual_vx_world", "actual_vy_world", "actual_vz",
        "velocity_parallel_mps", "velocity_cross_mps", "velocity_error_mps", "E_vel_percent",
    ]
    position_cols = common + [
        "desired_x", "desired_y", "desired_z", "actual_x", "actual_y", "actual_z",
        "position_error_m", "E_pos_percent",
    ]
    yaw_cols = common + ["desired_yaw", "actual_yaw", "yaw_error_rad", "E_yaw_percent"]
    counts = {}
    counts["speed_rows"], _ = copy_selected_rows(samples, out_dir / "speed_samples.csv", speed_cols, lambda row: row.get("evaluated_component") == "E_vel")
    counts["position_rows"], _ = copy_selected_rows(samples, out_dir / "position_samples.csv", position_cols, lambda row: row.get("evaluated_component") == "E_pos")
    counts["yaw_rows"], _ = copy_selected_rows(samples, out_dir / "yaw_samples.csv", yaw_cols, lambda row: row.get("evaluated_component") == "E_yaw")
    return counts


def copy_plots(root, out_dir):
    map_dir = root / "map_authority/p0p8_clean_scene"
    copied = []
    for name in ["route_map.png", "route_result.png"]:
        src = map_dir / name
        if not src.is_file():
            raise FileNotFoundError(src)
        out_dir.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, out_dir / name)
        copied.append(name)
    return copied


def write_readme(path, generated_at, counts):
    text = f"""# F250 Maritime Delivery Package

Generated: {generated_at}

This folder is a human-readable package derived from the runtime `evidence/current` store.
It is not the primary machine evidence. The primary evidence remains in the runtime workspace.

## Start Here

- `SUMMARY.csv`: overall route and flight-control results, plus the 10-window FC metric rows.
- `plots/route_map.png`: planned route on the current map.
- `plots/route_result.png`: planned route plus actual flown trajectory.

## Evidence Tables

- `route/trajectory.csv`: complete route trajectory samples with simplified columns; rows: {counts.get('trajectory_rows', 0)}.
- `route/waypoint_errors.csv`: per-waypoint route error table; rows: {counts.get('waypoint_rows', 0)}.
- `flight_control/speed_samples.csv`: complete FC speed-test samples with simplified columns; rows: {counts.get('speed_rows', 0)}.
- `flight_control/position_samples.csv`: complete FC position-test samples with simplified columns; rows: {counts.get('position_rows', 0)}.
- `flight_control/yaw_samples.csv`: complete FC yaw-test samples with simplified columns; rows: {counts.get('yaw_rows', 0)}.

The sample tables keep the full original time sampling for their metric sections. They only remove unrelated internal columns.
"""
    path.write_text(text, encoding="utf-8")


def looks_like_owned_package(path):
    return (
        (path / "README.md").exists()
        or (path / "SUMMARY.csv").exists()
        or (path / "route").exists()
        or (path / "flight_control").exists()
        or (path / "plots").exists()
    )


def main():
    parser = argparse.ArgumentParser(description="Derive the external tester delivery package from evidence/current.")
    parser.add_argument("--out", help="Output directory (default: ~/delivery_package)")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()

    root = resolve_project_root()
    current = Path(os.environ.get("F250_EVIDENCE_CURRENT_DIR", root / "evidence/current")).expanduser().resolve()
    if not current.is_dir():
        raise SystemExit(f"missing current evidence: {current}")

    default_out = Path.home() / "delivery_package"
    out_dir = Path(args.out).expanduser().resolve() if args.out else default_out
    if out_dir.exists():
        is_default = out_dir == default_out.resolve()
        is_empty = not any(out_dir.iterdir())
        if not (is_default or is_empty or looks_like_owned_package(out_dir)):
            raise SystemExit(f"refusing to overwrite non-package output directory: {out_dir}")
        shutil.rmtree(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    generated_at = utc_now()
    counts = {}
    counts.update(build_route(current, out_dir / "route"))
    counts.update(build_flight_control(current, out_dir / "flight_control"))
    copy_plots(root, out_dir / "plots")
    write_summary_csv(out_dir / "SUMMARY.csv", build_summary(current, generated_at))
    write_readme(out_dir / "README.md", generated_at, counts)

    if not args.quiet:
        print(out_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())