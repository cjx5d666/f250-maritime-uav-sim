#!/usr/bin/env python3
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


class OdomToTf:
    def __init__(self):
        rospy.init_node("mavros_odom_to_tf")
        self.odom_topic = rospy.get_param("~odom_topic", "/mavros/local_position/odom")
        self.parent_frame = rospy.get_param("~parent_frame", "world")
        self.child_frame = rospy.get_param("~child_frame", "base_link")
        self.broadcaster = tf2_ros.TransformBroadcaster()
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=10)
        rospy.loginfo("mavros_odom_to_tf: %s -> %s from %s", self.parent_frame, self.child_frame, self.odom_topic)

    def odom_cb(self, msg):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp if msg.header.stamp else rospy.Time.now()
        tf_msg.header.frame_id = self.parent_frame
        tf_msg.child_frame_id = self.child_frame
        tf_msg.transform.translation.x = msg.pose.pose.position.x
        tf_msg.transform.translation.y = msg.pose.pose.position.y
        tf_msg.transform.translation.z = msg.pose.pose.position.z
        tf_msg.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(tf_msg)


if __name__ == "__main__":
    try:
        OdomToTf()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
