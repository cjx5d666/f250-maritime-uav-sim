#!/usr/bin/env python3
"""Bridge Gazebo truth pose into MAVROS external-vision input."""

import rospy
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


PREFERRED_LINK_NAMES = (
    "iris::base_link",
    "iris_depth_camera::base_link",
    "iris_depth_camera::iris::base_link",
)


def is_base_link(name):
    return name == "base_link" or name.endswith("::base_link") or name.endswith("/base_link")


class GazeboTruthToMavrosVision:
    def __init__(self):
        self.link_states_topic = rospy.get_param("~link_states_topic", "/gazebo/link_states")
        self.vision_pose_topic = rospy.get_param("~vision_pose_topic", "/mavros/vision_pose/pose")
        self.odom_topic = rospy.get_param("~odom_topic", "/visual_truth/odom")
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.child_frame_id = rospy.get_param("~child_frame_id", "base_link")
        self.publish_odom = bool(rospy.get_param("~publish_odom", True))
        self.excluded_link_substrings = rospy.get_param(
            "~excluded_link_substrings", ["dynamic_", "obstacle"])

        self.cached_index = None
        self.cached_name = None
        self.vision_pose_pub = rospy.Publisher(self.vision_pose_topic, PoseStamped, queue_size=1)
        self.odom_pub = None
        if self.publish_odom:
            self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=1)

        rospy.Subscriber(self.link_states_topic, LinkStates, self.link_states_callback, queue_size=1)
        rospy.loginfo(
            "gazebo_truth_to_mavros_vision publishing %s from %s in frame %s",
            self.vision_pose_topic,
            self.link_states_topic,
            self.frame_id,
        )

    def candidate_base_link(self, name):
        if not is_base_link(name):
            return False
        return not any(token and token in name for token in self.excluded_link_substrings)

    def resolve_index(self, names):
        if (
            self.cached_index is not None
            and self.cached_index < len(names)
            and names[self.cached_index] == self.cached_name
        ):
            return self.cached_index

        for candidate in PREFERRED_LINK_NAMES:
            if candidate in names:
                return self.cache_index(names.index(candidate), candidate)

        iris_base_links = [
            (index, name) for index, name in enumerate(names)
            if self.candidate_base_link(name) and "iris" in name
        ]
        if iris_base_links:
            index, name = iris_base_links[0]
            return self.cache_index(index, name)

        for index, name in enumerate(names):
            if self.candidate_base_link(name):
                return self.cache_index(index, name)

        return None

    def cache_index(self, index, name):
        if self.cached_name != name:
            rospy.loginfo("using Gazebo truth link %s", name)
        self.cached_index = index
        self.cached_name = name
        return index

    def link_states_callback(self, msg):
        index = self.resolve_index(msg.name)
        if index is None:
            rospy.logwarn_throttle(
                5.0,
                "no base_link found in %s; available links include: %s",
                self.link_states_topic,
                ", ".join(msg.name[:8]),
            )
            return
        if index >= len(msg.pose):
            rospy.logwarn_throttle(5.0, "resolved link index %d has no pose", index)
            return

        stamp = rospy.Time.now()
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose = msg.pose[index]
        self.vision_pose_pub.publish(pose_msg)

        if self.odom_pub is not None:
            odom_msg = Odometry()
            odom_msg.header.stamp = stamp
            odom_msg.header.frame_id = self.frame_id
            odom_msg.child_frame_id = self.child_frame_id
            odom_msg.pose.pose = msg.pose[index]
            if index < len(msg.twist):
                odom_msg.twist.twist = msg.twist[index]
            self.odom_pub.publish(odom_msg)


def main():
    rospy.init_node("gazebo_truth_to_mavros_vision")
    GazeboTruthToMavrosVision()
    rospy.spin()


if __name__ == "__main__":
    main()
