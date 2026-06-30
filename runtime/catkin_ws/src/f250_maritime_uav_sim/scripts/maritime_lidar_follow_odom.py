#!/usr/bin/env python3
import sys

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from nav_msgs.msg import Odometry


def quaternion_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def rotate(matrix, point):
    return (
        matrix[0][0] * point[0] + matrix[0][1] * point[1] + matrix[0][2] * point[2],
        matrix[1][0] * point[0] + matrix[1][1] * point[1] + matrix[1][2] * point[2],
        matrix[2][0] * point[0] + matrix[2][1] * point[1] + matrix[2][2] * point[2],
    )


class LidarOdomFollower:
    def __init__(self):
        rospy.init_node("maritime_lidar_follow_odom")
        self.model_name = rospy.get_param("~model_name", "maritime_mid360_lidar")
        self.odom_topic = rospy.get_param("~odom_topic", "/mavros/local_position/odom")
        self.sensor_xyz = [float(v) for v in rospy.get_param("~sensor_xyz", [0.12, 0.0, 0.04])]
        self.update_rate_hz = max(1.0, float(rospy.get_param("~update_rate_hz", 20.0)))
        self.set_model_state_service = rospy.get_param("~set_model_state_service", "/gazebo/set_model_state")
        self.odom = None
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)
        try:
            rospy.wait_for_service(self.set_model_state_service, timeout=10.0)
        except rospy.ROSException:
            rospy.logerr("Gazebo set_model_state service not available after 10s: %s", self.set_model_state_service)
            sys.exit(1)
        self.set_model_state = rospy.ServiceProxy(self.set_model_state_service, SetModelState)
        rospy.Timer(rospy.Duration(1.0 / self.update_rate_hz), self.timer_callback)
        rospy.loginfo("following %s with Gazebo LiDAR model %s", self.odom_topic, self.model_name)

    def odom_callback(self, msg):
        self.odom = msg

    def timer_callback(self, _event):
        if self.odom is None:
            return
        pose = self.odom.pose.pose
        offset = rotate(quaternion_matrix(pose.orientation), self.sensor_xyz)
        state = ModelState()
        state.model_name = self.model_name
        state.reference_frame = "world"
        state.pose.position.x = pose.position.x + offset[0]
        state.pose.position.y = pose.position.y + offset[1]
        state.pose.position.z = pose.position.z + offset[2]
        state.pose.orientation = pose.orientation
        try:
            self.set_model_state(state)
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "failed to update LiDAR model pose: %s", exc)


def main():
    LidarOdomFollower()
    rospy.spin()


if __name__ == "__main__":
    main()
