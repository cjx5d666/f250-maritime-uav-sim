#!/usr/bin/env python3
import argparse
import csv
import json
import math
import os
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2
from mavros_msgs.msg import State
from quadrotor_msgs.msg import PositionCommand

from maritime_scene_utils import load_scene, scene_waypoints


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def safe_float(value):
    if value is None:
        return None
    try:
        return float(value)
    except Exception:
        return None


def now_label():
    return time.strftime("%Y%m%d_%H%M%S")


def sensor_label(source):
    source = str(source or "lidar")
    if source == "lidar":
        return "LiDAR"
    if source == "depth":
        return "Depth"
    return source


def point_distance(a, b):
    return math.sqrt(
        (a[0] - b[0]) ** 2 +
        (a[1] - b[1]) ** 2 +
        (a[2] - b[2]) ** 2
    )


def horiz_distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def waypoint_label(index, waypoint):
    label = waypoint.get("label")
    if label:
        return str(label)
    name = str(waypoint.get("name", ""))
    prefix = name.split("_")[0].upper()
    return prefix if prefix else "P%d" % index


def route_length(points):
    total = 0.0
    for index in range(1, len(points)):
        total += point_distance(points[index - 1], points[index])
    return total


def route_projection_xy(points, position):
    px, py = position[0], position[1]
    best = None
    cumulative = 0.0
    for idx in range(len(points) - 1):
        x1, y1 = points[idx][0], points[idx][1]
        x2, y2 = points[idx + 1][0], points[idx + 1][1]
        dx = x2 - x1
        dy = y2 - y1
        seg_len = math.hypot(dx, dy)
        if seg_len <= 1e-9:
            continue
        t = ((px - x1) * dx + (py - y1) * dy) / (seg_len ** 2)
        t = max(0.0, min(1.0, t))
        proj_x = x1 + t * dx
        proj_y = y1 + t * dy
        cross = math.hypot(px - proj_x, py - proj_y)
        along = cumulative + t * seg_len
        candidate = (cross, along)
        if best is None or candidate[0] < best[0]:
            best = candidate
        cumulative += seg_len
    return best if best is not None else (None, None)


class QuickComplexRecorder:
    def __init__(self, args):
        self.args = args
        self.scene = load_scene(args.scene_config)
        self.waypoints = scene_waypoints(self.scene)
        self.route_points = [wp["position"][:3] for wp in self.waypoints]
        self.route_profile = self.scene.get("route_profile") or {}
        self.route_id = self.route_profile.get("route_id")
        self.route_name = self.route_profile.get("name")
        self.route_profile_path = self.route_profile.get("profile_path")
        self.first_label = self.route_profile.get("first_label") or (
            waypoint_label(0, self.waypoints[0]) if self.waypoints else "W0")
        self.final_label = self.route_profile.get("final_label") or (
            waypoint_label(len(self.waypoints) - 1, self.waypoints[-1]) if self.waypoints else "final")
        self.total_route_length_m = float(
            self.route_profile.get("total_route_length_m") or route_length(self.route_points))
        self.final_radius = float(self.waypoints[-1].get("radius", 1.0)) if self.waypoints else 1.0
        self.final_hold_required = float(self.waypoints[-1].get("hold_time", 0.2)) if self.waypoints else 0.2
        self.start_wall = time.time()
        self.deadline_wall = self.start_wall + float(args.max_duration_sec)

        self.latest_pos_cmd = None
        self.latest_pos_cmd_stamp = None
        self.latest_setpoint = None
        self.latest_setpoint_stamp = None
        self.latest_state = None
        self.latest_active_goal = None
        self.latest_active_goal_index = None
        self.last_position = None
        self.last_active_goal_distance = None
        self.min_active_goal_distance = None
        self.last_cross_track = None
        self.last_along_track = None

        self.rows = []
        self.active_goal_advancements = []
        self.active_goal_count = 0
        self.counts = {
            "odom": 0,
            "pos_cmd": 0,
            "setpoint": 0,
            "state": 0,
            "active_goal": 0,
            "planner_cloud": 0,
            "raw_cloud": 0,
            "depth_cloud": 0,
            "occupancy_inflate": 0,
            "lidar_scan": 0,
            "landing_status": 0,
        }
        self.max_clouds = {
            "planner_cloud_points": 0,
            "raw_cloud_points": 0,
            "depth_cloud_points": 0,
            "occupancy_inflate_points": 0,
            "lidar_scan_ranges": 0,
        }
        self.final_hold_enter_wall = None
        self.final_hold_seen_sec = 0.0
        self.final_reached_ever = False
        self.stop_reason = "timeout"
        self.state_flags = {
            "connected": False,
            "mode": "",
            "armed": False,
            "seen_offboard": False,
            "seen_armed": False,
            "seen_disarmed_after_armed": False,
        }

        rospy.Subscriber(args.odom_topic, Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber(args.pos_cmd_topic, PositionCommand, self.pos_cmd_cb, queue_size=20)
        rospy.Subscriber(args.setpoint_topic, PoseStamped, self.setpoint_cb, queue_size=20)
        rospy.Subscriber(args.state_topic, State, self.state_cb, queue_size=20)
        rospy.Subscriber(args.active_goal_topic, PoseStamped, self.active_goal_cb, queue_size=20)
        rospy.Subscriber(args.planner_cloud_topic, PointCloud2, self.planner_cloud_cb, queue_size=5)
        rospy.Subscriber(args.raw_cloud_topic, PointCloud2, self.raw_cloud_cb, queue_size=5)
        rospy.Subscriber(args.depth_cloud_topic, PointCloud2, self.depth_cloud_cb, queue_size=5)
        rospy.Subscriber(args.occupancy_topic, PointCloud2, self.occupancy_cb, queue_size=5)
        rospy.Subscriber(args.lidar_scan_topic, LaserScan, self.lidar_scan_cb, queue_size=5)

    def pos_cmd_cb(self, msg):
        self.counts["pos_cmd"] += 1
        self.latest_pos_cmd_stamp = rospy.Time.now()
        self.latest_pos_cmd = {
            "position": [float(msg.position.x), float(msg.position.y), float(msg.position.z)],
            "velocity": [float(msg.velocity.x), float(msg.velocity.y), float(msg.velocity.z)],
            "acceleration": [float(msg.acceleration.x), float(msg.acceleration.y), float(msg.acceleration.z)],
            "yaw": float(msg.yaw),
            "yaw_dot": float(msg.yaw_dot),
        }

    def setpoint_cb(self, msg):
        self.counts["setpoint"] += 1
        self.latest_setpoint_stamp = rospy.Time.now()
        self.latest_setpoint = {
            "position": [float(msg.pose.position.x), float(msg.pose.position.y), float(msg.pose.position.z)],
            "yaw": quaternion_to_yaw(msg.pose.orientation),
        }

    def state_cb(self, msg):
        self.counts["state"] += 1
        self.latest_state = msg
        self.state_flags["connected"] = bool(msg.connected)
        self.state_flags["mode"] = msg.mode
        self.state_flags["armed"] = bool(msg.armed)
        if msg.mode == "OFFBOARD":
            self.state_flags["seen_offboard"] = True
        if msg.armed:
            self.state_flags["seen_armed"] = True
        if self.state_flags["seen_armed"] and not msg.armed:
            self.state_flags["seen_disarmed_after_armed"] = True

    def active_goal_cb(self, msg):
        self.counts["active_goal"] += 1
        position = [float(msg.pose.position.x), float(msg.pose.position.y), float(msg.pose.position.z)]
        index = self.match_waypoint_index(position)
        self.latest_active_goal = position
        if index is None:
            return
        if self.latest_active_goal_index != index:
            self.latest_active_goal_index = index
            waypoint = self.waypoints[index]
            self.active_goal_advancements.append({
                "index": index,
                "label": waypoint_label(index, waypoint),
                "name": waypoint.get("name", "waypoint_%d" % index),
                "position": waypoint["position"][:3],
                "wall_time": time.time() - self.start_wall,
            })

    def planner_cloud_cb(self, msg):
        self.counts["planner_cloud"] += 1
        self.max_clouds["planner_cloud_points"] = max(
            self.max_clouds["planner_cloud_points"], int(msg.width * msg.height))

    def raw_cloud_cb(self, msg):
        self.counts["raw_cloud"] += 1
        self.max_clouds["raw_cloud_points"] = max(
            self.max_clouds["raw_cloud_points"], int(msg.width * msg.height))

    def depth_cloud_cb(self, msg):
        self.counts["depth_cloud"] += 1
        self.max_clouds["depth_cloud_points"] = max(
            self.max_clouds["depth_cloud_points"], int(msg.width * msg.height))

    def occupancy_cb(self, msg):
        self.counts["occupancy_inflate"] += 1
        self.max_clouds["occupancy_inflate_points"] = max(
            self.max_clouds["occupancy_inflate_points"], int(msg.width * msg.height))

    def lidar_scan_cb(self, msg):
        self.counts["lidar_scan"] += 1
        self.max_clouds["lidar_scan_ranges"] = max(
            self.max_clouds["lidar_scan_ranges"], len(msg.ranges))

    def match_waypoint_index(self, position):
        best_index = None
        best_distance = None
        for index, waypoint in enumerate(self.waypoints):
            dist = point_distance(position, waypoint["position"][:3])
            if best_distance is None or dist < best_distance:
                best_distance = dist
                best_index = index
        if best_distance is None or best_distance > 0.5:
            return None
        return best_index

    def odom_cb(self, msg):
        self.counts["odom"] += 1
        now_wall = time.time()
        wall_rel = now_wall - self.start_wall
        ros_time = msg.header.stamp.to_sec()
        pos = [float(msg.pose.pose.position.x), float(msg.pose.pose.position.y), float(msg.pose.pose.position.z)]
        vel = [float(msg.twist.twist.linear.x), float(msg.twist.twist.linear.y), float(msg.twist.twist.linear.z)]
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)

        active_goal_distance = None
        active_goal_x = active_goal_y = active_goal_z = None
        if self.latest_active_goal is not None:
            active_goal_x, active_goal_y, active_goal_z = self.latest_active_goal
            active_goal_distance = point_distance(pos, self.latest_active_goal)
            self.last_active_goal_distance = active_goal_distance
            if self.min_active_goal_distance is None or active_goal_distance < self.min_active_goal_distance:
                self.min_active_goal_distance = active_goal_distance

        cross_track, along_track = route_projection_xy(self.route_points, pos)
        self.last_cross_track = cross_track
        self.last_along_track = along_track
        self.last_position = pos

        expected = self.latest_pos_cmd or {}
        setpoint = self.latest_setpoint or {}
        expected_pos = expected.get("position")
        expected_vel = expected.get("velocity")
        expected_acc = expected.get("acceleration")
        expected_yaw = expected.get("yaw")
        expected_yaw_dot = expected.get("yaw_dot")
        setpoint_pos = setpoint.get("position")
        setpoint_yaw = setpoint.get("yaw")

        pos_cmd_age = None
        if self.latest_pos_cmd_stamp is not None:
            pos_cmd_age = (rospy.Time.now() - self.latest_pos_cmd_stamp).to_sec()
        setpoint_age = None
        if self.latest_setpoint_stamp is not None:
            setpoint_age = (rospy.Time.now() - self.latest_setpoint_stamp).to_sec()

        pos_err_expected = None
        vel_err_expected = None
        yaw_err_expected = None
        if expected_pos is not None:
            pos_err_expected = point_distance(pos, expected_pos)
        if expected_vel is not None:
            vel_err_expected = point_distance(vel, expected_vel)
        if expected_yaw is not None:
            yaw_err_expected = math.atan2(math.sin(yaw - expected_yaw), math.cos(yaw - expected_yaw))

        pos_err_setpoint = None
        yaw_err_setpoint = None
        if setpoint_pos is not None:
            pos_err_setpoint = point_distance(pos, setpoint_pos)
        if setpoint_yaw is not None:
            yaw_err_setpoint = math.atan2(math.sin(yaw - setpoint_yaw), math.cos(yaw - setpoint_yaw))

        self.rows.append([
            wall_rel,
            ros_time,
            pos[0], pos[1], pos[2],
            vel[0], vel[1], vel[2],
            yaw,
            *(expected_pos or ["", "", ""]),
            *(expected_vel or ["", "", ""]),
            *(expected_acc or ["", "", ""]),
            expected_yaw if expected_yaw is not None else "",
            expected_yaw_dot if expected_yaw_dot is not None else "",
            pos_cmd_age if pos_cmd_age is not None else "",
            *(setpoint_pos or ["", "", ""]),
            setpoint_yaw if setpoint_yaw is not None else "",
            setpoint_age if setpoint_age is not None else "",
            pos_err_expected if pos_err_expected is not None else "",
            pos_err_setpoint if pos_err_setpoint is not None else "",
            vel_err_expected if vel_err_expected is not None else "",
            yaw_err_expected if yaw_err_expected is not None else "",
            yaw_err_setpoint if yaw_err_setpoint is not None else "",
            self.latest_state.mode if self.latest_state else "",
            "true" if (self.latest_state and self.latest_state.armed) else "false",
            self.latest_active_goal_index if self.latest_active_goal_index is not None else "",
            active_goal_x if active_goal_x is not None else "",
            active_goal_y if active_goal_y is not None else "",
            active_goal_z if active_goal_z is not None else "",
            active_goal_distance if active_goal_distance is not None else "",
            cross_track if cross_track is not None else "",
            along_track if along_track is not None else "",
        ])

        if self.latest_active_goal_index == len(self.waypoints) - 1 and active_goal_distance is not None:
            if active_goal_distance <= self.final_radius:
                if self.final_hold_enter_wall is None:
                    self.final_hold_enter_wall = now_wall
                self.final_hold_seen_sec = max(self.final_hold_seen_sec, now_wall - self.final_hold_enter_wall)
                self.final_reached_ever = True
                if self.final_hold_seen_sec >= self.final_hold_required:
                    self.stop_reason = "final_hold_reached"
            else:
                self.final_hold_enter_wall = None
        if now_wall >= self.deadline_wall and self.stop_reason != "final_hold_reached":
            self.stop_reason = "timeout"

    def done(self):
        return self.stop_reason == "final_hold_reached" or time.time() >= self.deadline_wall

    def summary(self):
        def safe_get(row, index, default=None):
            try:
                return row[index] if index < len(row) else default
            except (IndexError, TypeError):
                return default

        max_altitude = max((safe_get(row, 4) for row in self.rows if safe_get(row, 4) is not None), default=None)
        armed_rows = [row for row in self.rows if safe_get(row, 32) == "true"]
        min_altitude_armed = min((safe_get(row, 4) for row in armed_rows if safe_get(row, 4) is not None), default=None)
        cross_tracks = [safe_get(row, 38) for row in self.rows if isinstance(safe_get(row, 38), float)]
        along_tracks = [safe_get(row, 39) for row in self.rows if isinstance(safe_get(row, 39), float)]
        start_waypoint = self.waypoints[0] if self.waypoints else None
        final_waypoint = self.waypoints[-1] if self.waypoints else None

        def waypoint_metrics(waypoint):
            if waypoint is None:
                return None, None, None
            radius = float(waypoint.get("radius", 1.0))
            nearest = None
            nearest_t = None
            first_within_t = None
            for row in self.rows:
                position = [safe_get(row, 2, 0.0), safe_get(row, 3, 0.0), safe_get(row, 4, 0.0)]
                dist = point_distance(position, waypoint["position"][:3])
                t_sec = safe_get(row, 0)
                if t_sec is None:
                    continue
                if nearest is None or dist < nearest:
                    nearest = dist
                    nearest_t = t_sec
                if first_within_t is None and dist <= radius:
                    first_within_t = t_sec
            return nearest, nearest_t, first_within_t

        start_near, start_near_t, start_first_within = waypoint_metrics(start_waypoint)
        final_near, final_near_t, final_first_within = waypoint_metrics(final_waypoint)
        scored_duration = None
        if start_first_within is not None and final_first_within is not None:
            scored_duration = max(0.0, final_first_within - start_first_within)

        max_stall_seen = 0.0
        if self.rows:
            previous = None
            current_span = 0.0
            for row in self.rows:
                dist = safe_get(row, 37)
                t_sec = safe_get(row, 0)
                if previous is not None and dist is not None and isinstance(dist, float) and t_sec is not None:
                    if dist > 3.0:
                        current_span += t_sec - previous
                        max_stall_seen = max(max_stall_seen, current_span)
                    else:
                        current_span = 0.0
                if t_sec is not None:
                    previous = t_sec

        ok = (
            self.state_flags["seen_offboard"]
            and self.state_flags["seen_armed"]
            and self.latest_active_goal_index == len(self.waypoints) - 1
            and self.final_hold_seen_sec >= self.final_hold_required
        )
        errors = []
        if not self.state_flags["seen_offboard"]:
            errors.append("PX4 never entered OFFBOARD")
        if not self.state_flags["seen_armed"]:
            errors.append("PX4 never armed")
        if self.latest_active_goal_index != len(self.waypoints) - 1:
            errors.append("active goal never reached %s" % self.final_label)
        if self.final_hold_seen_sec < self.final_hold_required:
            errors.append("final hold %.3f below %.3f" % (self.final_hold_seen_sec, self.final_hold_required))

        return {
            "ok": ok,
            "errors": errors,
            "duration_requested_sec": self.args.max_duration_sec,
            "duration_observed_sec": safe_get(self.rows[-1], 0, 0.0) if self.rows else 0.0,
            "scene": os.path.abspath(self.args.scene_config),
            "sensor": {
                "source": self.args.perception_source,
                "label": sensor_label(self.args.perception_source),
                "planner_cloud_topic": self.args.planner_cloud_topic,
                "raw_cloud_topic": self.args.raw_cloud_topic,
                "lidar_scan_topic": self.args.lidar_scan_topic,
                "depth_cloud_topic": self.args.depth_cloud_topic,
                "occupancy_topic": self.args.occupancy_topic,
            },
            "perception_source": self.args.perception_source,
            "state": {
                "connected": self.state_flags["connected"],
                "mode": self.state_flags["mode"],
                "armed": self.state_flags["armed"],
                "seen_offboard": self.state_flags["seen_offboard"],
                "seen_armed": self.state_flags["seen_armed"],
                "seen_disarmed_after_armed": self.state_flags["seen_disarmed_after_armed"],
            },
            "counts": self.counts,
            "clouds": {
                "planner_cloud_topic": self.args.planner_cloud_topic,
                "raw_cloud_topic": self.args.raw_cloud_topic,
                "lidar_scan_topic": self.args.lidar_scan_topic,
                "depth_cloud_topic": self.args.depth_cloud_topic,
                "occupancy_topic": self.args.occupancy_topic,
                "max_planner_cloud_points": self.max_clouds["planner_cloud_points"],
                "max_raw_cloud_points": self.max_clouds["raw_cloud_points"],
                "max_depth_cloud_points": self.max_clouds["depth_cloud_points"],
                "max_occupancy_inflate_points": self.max_clouds["occupancy_inflate_points"],
                "max_lidar_scan_ranges": self.max_clouds["lidar_scan_ranges"],
            },
            "route": {
                "route_id": self.route_id,
                "name": self.route_name,
                "profile": self.route_profile_path,
                "profile_source": self.route_profile.get("profile_source"),
                "first_label": self.first_label,
                "final_label": self.final_label,
                "total_route_length_m": self.total_route_length_m,
                "locked_baseline_compatibility": bool(self.route_profile.get("locked_baseline_compatibility", False)),
                "active_goal_advancements": self.active_goal_advancements,
                "max_active_goal_index": self.latest_active_goal_index,
                "waypoint_count": len(self.waypoints),
                "final_completed": self.final_reached_ever and self.final_hold_seen_sec >= self.final_hold_required,
                "final_reached_ever": self.final_reached_ever,
                "final_hold_required_sec": self.final_hold_required,
                "final_hold_seen_sec": self.final_hold_seen_sec,
            },
            "metrics": {
                "best_along_track_m": max(along_tracks) if along_tracks else None,
                "max_cross_track_m": max(cross_tracks) if cross_tracks else None,
                "cross_track_samples": len(cross_tracks),
                "max_cross_track_allowed_m": self.args.max_cross_track_allowed_m,
                "last_active_goal_distance_m": self.last_active_goal_distance,
                "min_active_goal_distance_m": self.min_active_goal_distance,
                "max_altitude_m": max_altitude,
                "min_altitude_armed_m": min_altitude_armed,
                "max_stall_allowed_sec": self.args.max_stall_allowed_sec,
                "max_stall_seen_sec": max_stall_seen,
            },
            "task": {
                "duration_sec": scored_duration,
                "cross_track_max_m": max(cross_tracks) if cross_tracks else None,
                "start_label": self.first_label,
                "start_wall_sec": start_first_within,
                "start_nearest_distance_m": start_near,
                "start_nearest_wall_sec": start_near_t,
                "start_first_within_radius_m": float(start_waypoint.get("radius", 1.0)) if start_waypoint else None,
                "final_label": self.final_label,
                "final_nearest_distance_m": final_near,
                "final_nearest_wall_sec": final_near_t,
                "final_first_within_wall_sec": final_first_within,
                "final_first_within_radius_m": float(final_waypoint.get("radius", 1.0)) if final_waypoint else None,
                "total_route_length_m": self.total_route_length_m,
                "p0_wall_sec": start_first_within,
                "p0_nearest_distance_m": start_near,
                "p0_nearest_wall_sec": start_near_t,
                "p0_first_within_radius_m": float(start_waypoint.get("radius", 1.0)) if start_waypoint else None,
                "p8_nearest_distance_m": final_near,
                "p8_nearest_wall_sec": final_near_t,
                "p8_first_within_wall_sec": final_first_within,
                "p8_first_within_radius_m": float(final_waypoint.get("radius", 1.0)) if final_waypoint else None,
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
            "output_csv": os.path.abspath(self.args.output_csv),
            "stop_reason": self.stop_reason,
        }

    def write_outputs(self):
        os.makedirs(os.path.dirname(os.path.abspath(self.args.output_csv)), exist_ok=True)
        with open(self.args.output_csv, "w", encoding="utf-8", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow([
                "wall_time", "ros_time", "x", "y", "z", "vx", "vy", "vz", "actual_yaw",
                "expected_x", "expected_y", "expected_z", "expected_vx", "expected_vy", "expected_vz",
                "expected_ax", "expected_ay", "expected_az", "expected_yaw", "expected_yaw_dot",
                "pos_cmd_age_sec", "setpoint_x", "setpoint_y", "setpoint_z", "setpoint_yaw", "setpoint_age_sec",
                "position_error_to_expected_m", "position_error_to_setpoint_m", "velocity_error_to_expected_m",
                "yaw_error_rad", "yaw_error_to_setpoint_rad", "mode", "armed",
                "active_goal_index", "active_goal_x", "active_goal_y", "active_goal_z", "active_goal_distance_m",
                "cross_track_m", "along_track_m",
            ])
            writer.writerows(self.rows)
        summary = self.summary()
        os.makedirs(os.path.dirname(os.path.abspath(self.args.summary_json)), exist_ok=True)
        with open(self.args.summary_json, "w", encoding="utf-8") as handle:
            json.dump(summary, handle, indent=2, sort_keys=True)
            handle.write("\n")


def parse_args():
    parser = argparse.ArgumentParser(description="Record F250 quick-complex actual trajectory and summary.")
    parser.add_argument("--scene-config", required=True)
    parser.add_argument("--output-csv", required=True)
    parser.add_argument("--summary-json", required=True)
    parser.add_argument("--max-duration-sec", type=float, default=360.0)
    parser.add_argument("--max-cross-track-allowed-m", type=float, default=120.0)
    parser.add_argument("--max-stall-allowed-sec", type=float, default=90.0)
    parser.add_argument("--odom-topic", default="/mavros/local_position/odom")
    parser.add_argument("--pos-cmd-topic", default="/planning/pos_cmd")
    parser.add_argument("--setpoint-topic", default="/mavros/setpoint_position/local")
    parser.add_argument("--state-topic", default="/mavros/state")
    parser.add_argument("--active-goal-topic", default="/maritime/active_goal")
    parser.add_argument("--perception-source", default=os.environ.get("PERCEPTION_SOURCE", "lidar"))
    parser.add_argument("--planner-cloud-topic", default="/maritime/obstacles_cloud")
    parser.add_argument("--raw-cloud-topic", default="/maritime/lidar_points")
    parser.add_argument("--lidar-scan-topic", default="/maritime/lidar_scan")
    parser.add_argument("--depth-cloud-topic", default="/maritime_depth_camera/points")
    parser.add_argument("--occupancy-topic", default="/grid_map/occupancy_inflate")
    return parser.parse_args()


def main():
    args = parse_args()
    rospy.init_node("f250_quick_complex_record", anonymous=True)
    recorder = QuickComplexRecorder(args)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and not recorder.done():
        rate.sleep()
    recorder.rows = list(recorder.rows)  # snapshot before callbacks can append further
    recorder.write_outputs()
    print(os.path.abspath(args.summary_json))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
