#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time
from datetime import datetime
from pathlib import Path


def wrap_angle_rad(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def read_route_pair(path: str):
    """Return (p0, p1) if >= 2 valid rows; otherwise (rows_list, None) so caller can warn."""
    with open(path, newline="", encoding="utf-8") as handle:
        rows = [row for row in csv.DictReader(handle) if row.get("x") and row.get("y")]
    if len(rows) < 2:
        return rows, None  # caller checks for None p1
    return rows[0], rows[1]


def waypoint_xyz(row: dict[str, str]) -> list[float]:
    return [float(row["x"]), float(row["y"]), float(row.get("z") or 10.0)]


def make_payload(args, p0, p1, target_yaw, status, final_yaw, final_error, elapsed, samples,
                 stable_for_sec=0.0, stable_started_at_sec=None):
    settle_sec = max(0.0, float(args.settle_sec))
    stable_sec = max(0.0, float(args.stable_sec))
    stable_timeout_sec = max(0.1, float(args.timeout_sec))
    aligned = status == "aligned_stable"
    return {
        "schema": "f250_prealign_yaw_v1",
        "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "route_waypoints_csv": os.path.abspath(args.route_waypoints_csv),
        "command_topic": args.command_topic,
        "odom_topic": args.odom_topic,
        "control_mode_topic": args.control_mode_topic,
        "p0": {
            "label": p0.get("label", "P0"),
            "x": float(p0["x"]),
            "y": float(p0["y"]),
            "z": float(p0.get("z") or 10.0),
        },
        "p1": {
            "label": p1.get("label", "P1"),
            "x": float(p1["x"]),
            "y": float(p1["y"]),
            "z": float(p1.get("z") or 10.0),
        },
        "target_yaw_rad": target_yaw,
        "target_yaw_deg": math.degrees(target_yaw),
        "tolerance_deg": float(args.tolerance_deg),
        "settle_sec": settle_sec,
        "stable_sec": stable_sec,
        "stable_timeout_sec": stable_timeout_sec,
        "timeout_sec": stable_timeout_sec,
        "max_total_sec": settle_sec + stable_timeout_sec,
        "elapsed_sec": elapsed,
        "stable_for_sec": stable_for_sec,
        "stable_started_at_sec": stable_started_at_sec,
        "final_yaw_rad": final_yaw,
        "final_yaw_error_rad": final_error,
        "final_yaw_error_deg": None if final_error is None else math.degrees(abs(final_error)),
        "aligned": aligned,
        "status": status,
        "warning": not aligned,
        "sample_count": samples,
        "policy": "settle_then_continuous_yaw_stability_warning_only; route release continues unless ROS setup itself fails",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Yaw-align F250 at P0 toward P1 before route release.")
    parser.add_argument("--route-waypoints-csv", required=True)
    parser.add_argument("--output-json", required=True)
    parser.add_argument("--log", required=True)
    parser.add_argument("--settle-sec", type=float, default=5.0)
    parser.add_argument("--stable-sec", type=float, default=3.0)
    parser.add_argument("--timeout-sec", type=float, default=15.0)
    parser.add_argument("--tolerance-deg", type=float, default=1.6)
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument("--command-topic", default="/planning/pos_cmd")
    parser.add_argument("--control-mode-topic", default="/maritime/fc_control_mode")
    parser.add_argument("--odom-topic", default="/mavros/local_position/odom")
    parser.add_argument("--frame-id", default="map")
    args = parser.parse_args()

    os.makedirs(os.path.dirname(os.path.abspath(args.output_json)), exist_ok=True)

    raw_pair = read_route_pair(args.route_waypoints_csv)
    if raw_pair[1] is None:
        # fewer than 2 valid waypoints — warn and continue with exit 0
        rows_found = raw_pair[0]
        warn_msg = (
            f"WARNING: route waypoint file has fewer than 2 valid waypoints "
            f"(found {len(rows_found)}): {args.route_waypoints_csv}; skipping prealign"
        )
        print(warn_msg, file=sys.stderr)
        payload = {
            "schema": "f250_prealign_yaw_v1",
            "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "route_waypoints_csv": os.path.abspath(args.route_waypoints_csv),
            "status": "skipped_insufficient_waypoints",
            "warning": True,
            "aligned": False,
            "policy": "settle_then_continuous_yaw_stability_warning_only; route release continues unless ROS setup itself fails",
            "warning_detail": warn_msg,
        }
        Path(args.output_json).write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        Path(args.log).write_text(warn_msg + "\n", encoding="utf-8")
        return 0
    p0, p1 = raw_pair

    dx = float(p1["x"]) - float(p0["x"])
    dy = float(p1["y"]) - float(p0["y"])
    if math.hypot(dx, dy) <= 1e-9:
        warn_msg = "WARNING: P0->P1 direction is zero length; skipping prealign yaw"
        print(warn_msg, file=sys.stderr)
        payload = make_payload(args, p0, p1, 0.0, "skipped_zero_length_direction", None, None, 0.0, 0)
        payload["warning_detail"] = warn_msg
        Path(args.output_json).write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        Path(args.log).write_text(warn_msg + "\n", encoding="utf-8")
        return 0
    target_yaw = math.atan2(dy, dx)
    p0_xyz = waypoint_xyz(p0)

    log_lines = [
        f"target_yaw_rad={target_yaw:.9f}",
        f"target_yaw_deg={math.degrees(target_yaw):.3f}",
        f"settle_sec={float(args.settle_sec):.3f}",
        f"stable_sec={float(args.stable_sec):.3f}",
        f"stable_timeout_sec={float(args.timeout_sec):.3f}",
        f"tolerance_deg={float(args.tolerance_deg):.3f}",
    ]

    try:
        import rospy
        from nav_msgs.msg import Odometry
        from quadrotor_msgs.msg import PositionCommand
        from std_msgs.msg import String
    except Exception as exc:
        payload = make_payload(args, p0, p1, target_yaw, "ros_import_failed", None, None, 0.0, 0)
        payload["error"] = str(exc)
        Path(args.output_json).write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        Path(args.log).write_text("\n".join(log_lines + [f"error={exc}"]) + "\n", encoding="utf-8")
        return 2

    latest_yaw = {"value": None}

    def odom_cb(msg):
        latest_yaw["value"] = yaw_from_quaternion(msg.pose.pose.orientation)

    rospy.init_node("f250_prealign_yaw", anonymous=True)
    pub = rospy.Publisher(args.command_topic, PositionCommand, queue_size=1)
    mode_pub = rospy.Publisher(args.control_mode_topic, String, queue_size=1)
    rospy.Subscriber(args.odom_topic, Odometry, odom_cb, queue_size=1)

    rate_hz = max(1.0, float(args.rate_hz))
    tolerance = math.radians(float(args.tolerance_deg))
    settle_sec = max(0.0, float(args.settle_sec))
    stable_sec = max(0.0, float(args.stable_sec))
    stable_timeout_sec = max(0.1, float(args.timeout_sec))
    start = time.monotonic()
    deadline = start + settle_sec + stable_timeout_sec
    samples = 0
    status = "timeout_not_stable"
    final_yaw = None
    final_error = None
    stable_since_elapsed = None
    stable_for_sec = 0.0

    while not rospy.is_shutdown() and time.monotonic() <= deadline:
        now = time.monotonic()
        elapsed = now - start
        msg = PositionCommand()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = args.frame_id
        msg.position.x, msg.position.y, msg.position.z = p0_xyz
        msg.velocity.x = msg.velocity.y = msg.velocity.z = 0.0
        msg.acceleration.x = msg.acceleration.y = msg.acceleration.z = 0.0
        msg.yaw = target_yaw
        msg.yaw_dot = 0.0
        msg.trajectory_id = int(time.time()) & 0xFFFFFFFF
        msg.trajectory_flag = getattr(PositionCommand, "TRAJECTORY_STATUS_READY", 1)
        mode_pub.publish(String(data="position"))
        pub.publish(msg)
        samples += 1
        if latest_yaw["value"] is not None:
            final_yaw = float(latest_yaw["value"])
            final_error = wrap_angle_rad(final_yaw - target_yaw)
            if elapsed >= settle_sec and abs(final_error) <= tolerance:
                if stable_since_elapsed is None:
                    stable_since_elapsed = elapsed
                stable_for_sec = elapsed - stable_since_elapsed
                if stable_for_sec >= stable_sec:
                    status = "aligned_stable"
                    break
            elif elapsed >= settle_sec:
                stable_since_elapsed = None
                stable_for_sec = 0.0
        time.sleep(1.0 / rate_hz)

    elapsed = time.monotonic() - start
    if latest_yaw["value"] is None and status != "aligned_stable":
        status = "no_odom_warning"
    payload = make_payload(args, p0, p1, target_yaw, status, final_yaw, final_error, elapsed, samples,
                           stable_for_sec=stable_for_sec, stable_started_at_sec=stable_since_elapsed)
    Path(args.output_json).write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    log_lines.extend([
        f"status={status}",
        f"aligned={str(status == 'aligned_stable').lower()}",
        f"elapsed_sec={elapsed:.3f}",
        f"stable_for_sec={stable_for_sec:.3f}",
        f"stable_started_at_sec={'' if stable_since_elapsed is None else format(stable_since_elapsed, '.3f')}",
        f"final_yaw_rad={'' if final_yaw is None else format(final_yaw, '.9f')}",
        f"final_yaw_error_deg={'' if final_error is None else format(math.degrees(abs(final_error)), '.3f')}",
        f"warning={str(status != 'aligned_stable').lower()}",
    ])
    Path(args.log).write_text("\n".join(log_lines) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
