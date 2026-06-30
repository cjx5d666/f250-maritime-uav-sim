#!/usr/bin/env python3
from collections import deque

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path


def distance_sq(a, b):
    dx = a.x - b.x
    dy = a.y - b.y
    dz = a.z - b.z
    return dx * dx + dy * dy + dz * dz


class FlightPathPublisher:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/mavros/local_position/odom")
        self.path_topic = rospy.get_param("~path_topic", "/maritime/flight_path")
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.sample_period_sec = float(rospy.get_param("~sample_period_sec", 0.2))
        self.min_distance_m = float(rospy.get_param("~min_distance_m", 0.05))
        self.max_points = int(rospy.get_param("~max_points", 1500))
        self.last_sample_time = None
        self.last_sample_position = None
        self.poses = deque(maxlen=max(1, self.max_points))
        self.publisher = rospy.Publisher(self.path_topic, Path, queue_size=1, latch=True)
        self.subscriber = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)

    def should_sample(self, msg):
        now = rospy.Time.now()
        if self.last_sample_time is None:
            return True
        if (now - self.last_sample_time).to_sec() < self.sample_period_sec:
            return False
        if self.last_sample_position is None:
            return True
        return distance_sq(msg.pose.pose.position, self.last_sample_position) >= self.min_distance_m ** 2

    def odom_callback(self, msg):
        if not self.should_sample(msg):
            return
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp if msg.header.stamp else rospy.Time.now()
        pose.header.frame_id = self.frame_id
        pose.pose = msg.pose.pose
        self.poses.append(pose)
        self.last_sample_time = rospy.Time.now()
        self.last_sample_position = msg.pose.pose.position
        self.publish_path()

    def publish_path(self):
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = self.frame_id
        path.poses = list(self.poses)
        self.publisher.publish(path)


def main():
    rospy.init_node("maritime_flight_path")
    FlightPathPublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
