#!/usr/bin/env python3
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from maritime_metric_core import MetricAccumulator, run_offline, wrap_angle_rad
from maritime_metric_core import match_waypoint_index, now_label


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def make_color(r, g, b, a=1.0):
    from std_msgs.msg import ColorRGBA
    return ColorRGBA(float(r), float(g), float(b), float(a))


def point_from_list(values):
    from geometry_msgs.msg import Point
    return Point(x=float(values[0]), y=float(values[1]), z=float(values[2]))


def pass_text(value):
    if value is None:
        return "--"
    return "PASS" if value else "FAIL"


def yes_text(value):
    return "YES" if value else "NO"


def ratio_text(value):
    if value is None:
        return "--"
    return "%.5f" % float(value)


def meters_text(value):
    if value is None:
        return "--"
    return "%.3f m" % float(value)


def radians_text(value):
    if value is None:
        return "--"
    return "%.3f rad" % wrap_angle_rad(value)


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


def metric37_state_text(metric37):
    for key in ("safe_so_far", "static_safe", "passed"):
        if key in metric37 and metric37.get(key) is not None:
            return "SAFE" if bool(metric37.get(key)) else "UNSAFE"
    return "UNKNOWN"


def metric37_safety_text(metric37):
    clearance = metric37.get("clearance") or {}
    static_geometry_entries = first_present(
        clearance.get("static_geometry_entry_count"),
        metric37.get("static_geometry_entry_count"),
        metric37.get("geometry_entry_count"),
    )
    static_cloud_entries = first_present(
        clearance.get("static_cloud_entry_count"),
        metric37.get("static_cloud_entry_count"),
        metric37.get("cloud_entry_count"),
    )
    dynamic_geometry_entries = first_present(
        clearance.get("dynamic_geometry_entry_count"),
        metric37.get("dynamic_geometry_entry_count_telemetry"),
        metric37.get("dynamic_geometry_entry_count"),
    )
    dynamic_cloud_entries = first_present(
        clearance.get("dynamic_cloud_entry_count"),
        metric37.get("dynamic_cloud_entry_count_telemetry"),
        metric37.get("dynamic_cloud_entry_count"),
    )
    return (
        "%s | static min %s cloud %s entries %s/%s | "
        "dynamic min %s cloud %s entries %s/%s"
    ) % (
        metric37_state_text(metric37),
        meters_text(clearance.get("static_min_clearance_m")),
        meters_text(clearance.get("static_cloud_min_distance_m")),
        count_text(static_geometry_entries),
        count_text(static_cloud_entries),
        meters_text(clearance.get("dynamic_min_clearance_m")),
        meters_text(clearance.get("dynamic_cloud_min_distance_m")),
        count_text(dynamic_geometry_entries),
        count_text(dynamic_cloud_entries),
    )


def max_abs_yaw_error(summary):
    values = []
    for stat in summary.get("waypoints", []):
        for key in ("nearest_yaw_error_rad", "active_nearest_yaw_error_rad"):
            value = stat.get(key)
            if value is not None:
                values.append(abs(wrap_angle_rad(value)))
    return max(values) if values else None


def terminal_waypoint_status(stat):
    nearest = stat.get("nearest_distance_m")
    radius = stat.get("radius_m")
    if nearest is None or radius is None:
        return "--"
    return "PASS" if float(nearest) <= float(radius) else "FAIL"


def next_waypoint_line(stat):
    return "NEXT  %s" % stat["label"]


def waypoint_terminal_block(stat, summary, status=None, reason=None):
    m37 = summary["metric_3_7"]
    waypoint_count = len(summary.get("waypoints", []))
    index = int(stat["index"])
    lines = [
        "================ %s ================" % stat["label"],
        "%-10s %s" % ("status", status or terminal_waypoint_status(stat)),
        "%-10s %s   / radius %s" % (
            "pos err",
            meters_text(stat.get("nearest_distance_m")),
            meters_text(stat.get("radius_m")),
        ),
    ]
    if 1 <= index <= waypoint_count - 2:
        lines.append("%-10s %s   (P1-P7 only)" % (
            "3.6 ratio",
            ratio_text(stat.get("metric_3_6_error_ratio")),
        ))
    lines.append("%-10s %s" % (
        "3.7 safe",
        pass_text(m37["safe_so_far"]),
    ))
    if index == waypoint_count - 1:
        m39 = summary["metric_3_9"]
        lines.append("%-10s final err %s / ratio %s" % (
            "3.9",
            meters_text(m39["final_error_m"]),
            ratio_text(m39["final_error_ratio"]),
        ))
    lines.append("%-10s %s" % ("yaw note", radians_text(stat.get("nearest_yaw_error_rad"))))
    lines.append("%-10s %s" % ("reason", reason or stat.get("finalize_reason") or "--"))
    return "\n".join(lines)


def final_summary_terminal_block(summary):
    m36 = summary["metric_3_6"]
    m37 = summary["metric_3_7"]
    m39 = summary["metric_3_9"]
    lines = [
        "================ FINAL SUMMARY ================",
        "%-10s %s / hold %.3f sec" % (
            summary.get("final_label") or "final",
            "completed" if summary.get("final_completed", summary["p8_completed"]) else "not complete",
            float(summary["final_hold_seen_sec"]),
        ),
        "%-10s mean %s / max %s" % (
            "3.6",
            ratio_text(m36["mean_error_ratio"]),
            ratio_text(m36["max_error_ratio"]),
        ),
        "%-10s %s" % (
            "3.7",
            metric37_safety_text(m37),
        ),
        "%-10s final err %s / ratio %s" % (
            "3.9",
            meters_text(m39["final_error_m"]),
            ratio_text(m39["final_error_ratio"]),
        ),
        "%-10s max %s" % ("yaw note", radians_text(max_abs_yaw_error(summary))),
        "%-10s %s" % ("3.10", "separate FC steady-state test"),
    ]
    return "\n".join(lines)


class MetricMonitorNode:
    COLORS = {
        "pending": (0.45, 0.45, 0.45, 0.72),
        "current": (0.05, 0.32, 1.0, 0.95),
        "passed": (0.05, 0.75, 0.22, 0.95),
        "failed": (0.95, 0.08, 0.08, 0.95),
        "near_threshold": (1.0, 0.82, 0.05, 0.98),
    }

    def __init__(self):
        import rospy
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String
        from visualization_msgs.msg import MarkerArray

        self.rospy = rospy
        self.String = String
        self.MarkerArray = MarkerArray

        scene_config = rospy.get_param("~scene_config", None)
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.dynamic_mode = rospy.get_param("~dynamic_mode", "auto")
        self.odom_topic = rospy.get_param("~odom_topic", "/mavros/local_position/odom")
        self.active_goal_topic = rospy.get_param("~active_goal_topic", "/maritime/active_goal")
        self.marker_topic = rospy.get_param("~marker_topic", "/maritime/metric_markers")
        self.status_topic = rospy.get_param("~status_topic", "/maritime/metric_status")
        self.output_dir = rospy.get_param("~output_dir", "")
        self.run_label = rospy.get_param("~run_label", "metric_%s" % now_label())
        self.create_run_subdir = bool(rospy.get_param("~create_run_subdir", True))
        self.show_global_label = bool(rospy.get_param("~show_global_label", False))
        clearance_period = float(rospy.get_param("~clearance_sample_period_sec", 0.2))
        final_hold = rospy.get_param("~final_zone_hold_sec", None)
        if final_hold == "":
            final_hold = None
        self.accumulator = MetricAccumulator(
            scene_config,
            dynamic_mode=self.dynamic_mode,
            clearance_sample_period_sec=clearance_period,
            final_zone_hold_sec=final_hold,
        )
        self.active_goal_index = None
        self.wrote_outputs = False
        self.last_status_json = "{}"
        self.pending_current_log_index = None
        self.display_log_path = os.environ.get("MARITIME_METRIC_DISPLAY_LOG", "").strip()
        if self.display_log_path:
            self.ensure_display_log()

        self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=1, latch=True)
        self.active_goal_sub = rospy.Subscriber(self.active_goal_topic, PoseStamped, self.active_goal_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)

        publish_rate_hz = max(0.5, min(10.0, float(rospy.get_param("~publish_rate_hz", 2.0))))
        rospy.Timer(rospy.Duration(1.0 / publish_rate_hz), self.publish_callback)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("maritime metric monitor active: markers=%s status=%s output_dir=%s",
                      self.marker_topic, self.status_topic, self.output_dir or "<disabled>")

    def active_goal_callback(self, msg):
        pos = msg.pose.position
        active_position = [float(pos.x), float(pos.y), float(pos.z)]
        matched_index = match_waypoint_index(active_position, self.accumulator.stats, tolerance_m=0.35)
        if matched_index is not None and matched_index != self.active_goal_index:
            self.pending_current_log_index = matched_index
        self.active_goal_index = matched_index

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        stamp = msg.header.stamp.to_sec() if msg.header.stamp and msg.header.stamp.to_sec() > 0.0 else self.rospy.Time.now().to_sec()
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.accumulator.observe(
            [pos.x, pos.y, pos.z],
            yaw_rad=yaw,
            time_sec=stamp,
            active_goal_index=self.active_goal_index,
        )
        for event in self.accumulator.drain_events():
            self.log_event(event)
        self.log_current_waypoint_if_needed()
        if self.accumulator.final_completed and not self.wrote_outputs:
            self.write_outputs("final_hold_reached")

    def log_event(self, event):
        index = event.get("index")
        if index is None or index < 0 or index >= len(self.accumulator.stats):
            return
        status = "PASS" if event.get("status") == "passed" else "FAIL"
        block = waypoint_terminal_block(
            self.accumulator.stats[index],
            self.accumulator.summary(),
            status,
            event.get("reason"),
        )
        self.rospy.loginfo(block)
        self.append_display_log(block)

    def log_current_waypoint_if_needed(self):
        index = self.pending_current_log_index
        self.pending_current_log_index = None
        if index is None or index < 0 or index >= len(self.accumulator.stats):
            return
        stat = self.accumulator.stats[index]
        if stat["finalized"]:
            return
        line = next_waypoint_line(stat)
        self.rospy.loginfo(line)
        self.append_display_log(line)

    def ensure_display_log(self):
        try:
            parent = os.path.dirname(self.display_log_path)
            if parent:
                os.makedirs(parent, exist_ok=True)
            with open(self.display_log_path, "a", encoding="utf-8"):
                pass
        except OSError as exc:
            self.rospy.logwarn("cannot prepare metric display log %s: %s", self.display_log_path, exc)

    def append_display_log(self, text):
        if not self.display_log_path:
            return
        try:
            with open(self.display_log_path, "a", encoding="utf-8") as handle:
                handle.write(text.rstrip("\n") + "\n")
        except OSError as exc:
            self.rospy.logwarn("cannot append metric display log %s: %s", self.display_log_path, exc)

    def write_outputs(self, reason):
        if not self.output_dir:
            self.wrote_outputs = True
            self.append_display_log(final_summary_terminal_block(self.accumulator.summary()))
            return
        self.accumulator.write_outputs(
            self.output_dir,
            run_label=self.run_label,
            create_run_subdir=self.create_run_subdir,
        )
        self.wrote_outputs = True
        self.log_final_summary(reason)

    def log_final_summary(self, reason):
        block = final_summary_terminal_block(self.accumulator.summary())
        self.rospy.loginfo(block)
        self.append_display_log(block)

    def shutdown(self):
        if not self.wrote_outputs and self.accumulator.sample_count > 0 and self.output_dir:
            self.write_outputs("shutdown")

    def publish_callback(self, _event):
        self.publish_status()
        self.publish_markers()

    def publish_status(self):
        summary = self.accumulator.summary()
        compact = {
            "ok": summary["ok"],
            "active_goal_index": summary["active_goal_index"],
            "final_completed": summary.get("final_completed", summary["p8_completed"]),
            "final_label": summary.get("final_label"),
            "p8_completed": summary["p8_completed"],
            "final_hold_seen_sec": summary["final_hold_seen_sec"],
            "metric_3_6": summary["metric_3_6"],
            "metric_3_7": summary["metric_3_7"],
            "metric_3_9": summary["metric_3_9"],
            "policy_note": summary.get("policy_note"),
        }
        self.last_status_json = json.dumps(compact, sort_keys=True)
        self.status_pub.publish(self.String(data=self.last_status_json))

    def publish_markers(self):
        from geometry_msgs.msg import Vector3
        from visualization_msgs.msg import Marker, MarkerArray

        markers = []
        delete_all = Marker()
        delete_all.header.frame_id = self.frame_id
        delete_all.header.stamp = self.rospy.Time.now()
        delete_all.ns = "maritime_metrics"
        delete_all.action = Marker.DELETEALL
        markers.append(delete_all)

        marker_id = 0
        for stat in self.accumulator.stats:
            state = self.accumulator.status_for_marker(stat["index"])
            color = make_color(*self.COLORS.get(state, self.COLORS["pending"]))
            sphere = Marker()
            sphere.header.frame_id = self.frame_id
            sphere.header.stamp = self.rospy.Time.now()
            sphere.ns = "maritime_metrics_waypoints"
            sphere.id = marker_id
            marker_id += 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position = point_from_list(stat["position"])
            sphere.pose.orientation.w = 1.0
            scale = 1.25 if state != "current" else 1.75
            sphere.scale = Vector3(scale, scale, scale)
            sphere.color = color
            sphere.lifetime = self.rospy.Duration(0.0)
            markers.append(sphere)

            label = Marker()
            label.header.frame_id = self.frame_id
            label.header.stamp = self.rospy.Time.now()
            label.ns = "maritime_metrics_labels"
            label.id = marker_id
            marker_id += 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position = point_from_list([
                stat["position"][0],
                stat["position"][1],
                stat["position"][2] + 2.1 + 0.35 * (stat["index"] % 2),
            ])
            label.pose.orientation.w = 1.0
            label.scale.z = 1.35
            label.color = make_color(1.0, 1.0, 1.0, 0.95)
            label.text = self.label_text(stat)
            label.lifetime = self.rospy.Duration(0.0)
            markers.append(label)

        if self.show_global_label:
            markers.append(self.global_label(marker_id))
        self.marker_pub.publish(MarkerArray(markers=markers))

    def label_text(self, stat):
        err = "--"
        if stat["nearest_distance_m"] is not None:
            err = "%.2fm" % stat["nearest_distance_m"]
        yaw = "--"
        if stat["nearest_yaw_error_rad"] is not None:
            yaw = "%.2frad" % wrap_angle_rad(stat["nearest_yaw_error_rad"])
        m36 = "--"
        if stat["metric_3_6_error_ratio"] is not None:
            m36 = "%.4f" % stat["metric_3_6_error_ratio"]
        return "%s\nerr %s yaw %s\n3.6 %s" % (stat["label"], err, yaw, m36)

    def global_label(self, marker_id):
        from visualization_msgs.msg import Marker

        summary = self.accumulator.summary()
        m36 = summary["metric_3_6"]["mean_error_ratio"]
        m39 = summary["metric_3_9"]["final_error_ratio"]
        text = "3.6 mean %s | 3.7 %s | 3.9 %s | 3.10 FC-only" % (
            "--" if m36 is None else "%.5f" % m36,
            metric37_safety_text(summary["metric_3_7"]),
            "--" if m39 is None else "%.5f" % m39,
        )
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.rospy.Time.now()
        marker.ns = "maritime_metrics_global"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 22.0
        marker.pose.position.y = 108.0
        marker.pose.position.z = 18.0
        marker.pose.orientation.w = 1.0
        marker.scale.z = 2.2
        marker.color = make_color(0.95, 1.0, 0.95, 0.98)
        marker.text = text
        marker.lifetime = self.rospy.Duration(0.0)
        return marker


def offline_main(args):
    summary = run_offline(
        args.scene_config,
        args.trajectory_csv,
        args.output_dir,
        run_label=args.run_label,
        dynamic_mode=args.dynamic_mode,
        actual_filter=args.actual_filter,
        clearance_sample_period_sec=args.clearance_sample_period_sec,
    )
    print(json.dumps({
        "ok": summary["ok"],
            "metric_3_6": summary["metric_3_6"],
            "metric_3_7": summary["metric_3_7"],
            "metric_3_9": summary["metric_3_9"],
            "policy_note": summary.get("policy_note"),
        }, indent=2, sort_keys=True))
    return 0 if summary["ok"] else 1


def main():
    parser = argparse.ArgumentParser(description="F250 maritime quick-complex metric monitor.")
    parser.add_argument("--offline", action="store_true", help="compute metrics from an actual_trajectory.csv")
    parser.add_argument("--scene-config", default=None)
    parser.add_argument("--trajectory-csv", default=None)
    parser.add_argument("--output-dir", default="")
    parser.add_argument("--run-label", default=None)
    parser.add_argument("--dynamic-mode", default="auto")
    parser.add_argument("--actual-filter", default="armed_offboard")
    parser.add_argument("--clearance-sample-period-sec", type=float, default=0.0)
    args, _unknown = parser.parse_known_args()
    if args.offline:
        if not args.scene_config or not args.trajectory_csv:
            parser.error("--offline requires --scene-config and --trajectory-csv")
        return offline_main(args)

    import rospy
    rospy.init_node("maritime_metric_monitor")
    MetricMonitorNode()
    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
