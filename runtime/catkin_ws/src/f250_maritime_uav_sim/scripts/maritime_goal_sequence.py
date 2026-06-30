#!/usr/bin/env python3
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Header

from maritime_scene_utils import load_scene, scene_waypoints


def quaternion_from_yaw(yaw):
    return Quaternion(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def waypoint_to_pose(waypoint, frame_id):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = frame_id
    pose.pose.position.x = waypoint["position"][0]
    pose.pose.position.y = waypoint["position"][1]
    pose.pose.position.z = waypoint["position"][2]
    pose.pose.orientation = quaternion_from_yaw(waypoint["yaw"])
    return pose


class GoalSequence:
    def __init__(self):
        parser = argparse.ArgumentParser(description="Publish maritime PoseStamped goals in order.")
        parser.add_argument("--scene-config", default=None)
        args = parser.parse_known_args()[0]

        scene = load_scene(rospy.get_param("~scene_config", args.scene_config))
        self.frame_id = rospy.get_param("~frame_id", scene.get("frame_id", "world"))
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.active_goal_topic = rospy.get_param("~active_goal_topic", "/maritime/active_goal")
        self.heartbeat_topic = rospy.get_param("~heartbeat_topic", "/mission/heartbeat")
        self.odom_topic = rospy.get_param("~odom_topic", "/mavros/local_position/odom")
        self.start_paused = bool(rospy.get_param("~start_paused", False))
        self.start_topic = rospy.get_param("~start_topic", "/maritime/demo/start_waypoints")
        self.started = not self.start_paused
        acceptance = scene.get("acceptance") or {}
        self.position_tolerance = float(rospy.get_param(
            "~position_tolerance_m", acceptance.get("position_tolerance_m", 0.75)))
        self.publish_period = float(rospy.get_param(
            "~publish_period_sec", acceptance.get("goal_publish_period_sec", 2.0)))
        self.no_odom_advance_sec = float(rospy.get_param(
            "~no_odom_advance_sec", acceptance.get("no_odom_advance_sec", 0.0)))
        self.loop = bool(rospy.get_param("~loop", False))
        self.waypoints = scene_waypoints(scene)
        if not self.waypoints:
            raise RuntimeError("maritime scene has no waypoints")
        self.index = 0
        self.odom = None
        self.current_started = rospy.Time.now()
        self.reached_since = None
        self.publisher = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1, latch=True)
        self.active_goal_publisher = rospy.Publisher(self.active_goal_topic, PoseStamped, queue_size=1, latch=True)
        self.heartbeat_publisher = rospy.Publisher(self.heartbeat_topic, Header, queue_size=5)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)
        self.start_sub = None
        if self.start_paused:
            self.start_sub = rospy.Subscriber(self.start_topic, rospy.AnyMsg, self.start_callback, queue_size=1)
            rospy.loginfo(
                "maritime goal sequence waiting for demo start on %s (std_msgs/Bool true or std_msgs/Empty)",
                self.start_topic)
        self.timer = rospy.Timer(rospy.Duration(self.publish_period), self.timer_callback)
        self.heartbeat_timer = rospy.Timer(rospy.Duration(0.2), self.heartbeat_callback)
        rospy.Timer(rospy.Duration(0.25), self.initial_publish, oneshot=True)

    def initial_publish(self, _event):
        if not self.started:
            return
        self.publish_current_goal()

    def start_callback(self, msg):
        if self.started:
            return
        if not self.is_start_message(msg):
            return

        self.started = True
        self.current_started = rospy.Time.now()
        self.reached_since = None
        rospy.loginfo("maritime goal sequence started from %s", self.start_topic)
        self.publish_current_goal()

    def is_start_message(self, msg):
        connection_header = getattr(msg, "_connection_header", {}) or {}
        msg_type = connection_header.get("type", "")
        if msg_type == "std_msgs/Empty":
            return True
        if msg_type == "std_msgs/Bool":
            try:
                decoded = Bool().deserialize(msg._buff)
            except Exception as exc:
                rospy.logwarn("failed to decode demo start Bool from %s: %s", self.start_topic, exc)
                return False
            if decoded.data:
                return True
            rospy.loginfo_throttle(5.0, "maritime goal sequence still paused after Bool false on %s",
                                   self.start_topic)
            return False
        rospy.logwarn_throttle(5.0, "ignoring unsupported demo start message type %s on %s",
                               msg_type or "<unknown>", self.start_topic)
        return False

    def odom_callback(self, msg):
        self.odom = msg

    def reached_current(self):
        if self.odom is None:
            return False
        target = self.waypoints[self.index]["position"]
        current = self.odom.pose.pose.position
        distance = math.sqrt(
            (current.x - target[0]) ** 2 +
            (current.y - target[1]) ** 2 +
            (current.z - target[2]) ** 2
        )
        radius = float(self.waypoints[self.index].get("radius", self.position_tolerance))
        return distance <= radius

    def should_advance_without_odom(self):
        if self.odom is not None or self.no_odom_advance_sec <= 0.0:
            return False
        elapsed = (rospy.Time.now() - self.current_started).to_sec()
        return elapsed >= self.no_odom_advance_sec

    def should_advance_on_timeout(self):
        max_duration = float(self.waypoints[self.index].get("max_duration_sec", 0.0))
        if max_duration <= 0.0:
            return False
        elapsed = (rospy.Time.now() - self.current_started).to_sec()
        return elapsed >= max_duration

    def advance(self):
        if self.index + 1 < len(self.waypoints):
            self.index += 1
        elif self.loop:
            self.index = 0
        self.current_started = rospy.Time.now()
        self.reached_since = None

    def publish_current_goal(self):
        waypoint = self.waypoints[self.index]
        goal = waypoint_to_pose(waypoint, self.frame_id)
        self.publisher.publish(goal)
        self.active_goal_publisher.publish(goal)
        if waypoint.get("gps_position") is not None:
            rospy.loginfo("published maritime goal %d/%d: %s gps=%s local=%s",
                          self.index + 1, len(self.waypoints), waypoint["name"],
                          waypoint["gps_position"], waypoint["position"])
        else:
            rospy.loginfo("published maritime goal %d/%d: %s",
                          self.index + 1, len(self.waypoints), waypoint["name"])

    def heartbeat_callback(self, _event):
        self.heartbeat_publisher.publish(Header(stamp=rospy.Time.now(), frame_id=self.frame_id))

    def timer_callback(self, _event):
        if not self.started:
            rospy.loginfo_throttle(10.0, "maritime goal sequence demo-paused; waiting on %s", self.start_topic)
            return

        should_advance = False
        if self.reached_current():
            if self.reached_since is None:
                self.reached_since = rospy.Time.now()
            hold_time = float(self.waypoints[self.index].get("hold_time", 0.0))
            should_advance = (rospy.Time.now() - self.reached_since).to_sec() >= hold_time
        else:
            self.reached_since = None

        if should_advance or self.should_advance_without_odom() or self.should_advance_on_timeout():
            previous = self.index
            self.advance()
            if self.index != previous:
                rospy.loginfo("advanced maritime goal sequence to %s", self.waypoints[self.index]["name"])
        self.publish_current_goal()


def main():
    rospy.init_node("maritime_goal_sequence")
    GoalSequence()
    rospy.spin()


if __name__ == "__main__":
    main()
