#!/usr/bin/env python3
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Quaternion

from maritime_scene_utils import dynamic_mode_enabled, dynamic_obstacle_center, dynamic_obstacle_yaw
from maritime_scene_utils import load_scene, scene_dynamic_obstacles, validate_scene


def quaternion_from_yaw(yaw):
    return Quaternion(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class DynamicObstacleController:
    def __init__(self):
        self.scene = load_scene(rospy.get_param("~scene_config", None))
        errors = validate_scene(self.scene)
        if errors:
            raise RuntimeError("invalid maritime scene: %s" % "; ".join(errors))

        self.dynamic_mode = rospy.get_param("~dynamic_mode", "auto")
        self.obstacles = scene_dynamic_obstacles(self.scene) if dynamic_mode_enabled(self.scene, self.dynamic_mode) else []
        self.service_name = rospy.get_param("~set_model_state_service", "/gazebo/set_model_state")
        self.reference_frame = rospy.get_param("~reference_frame", "map")
        self.rate_hz = max(2.0, min(20.0, float(rospy.get_param("~rate_hz", 8.0))))
        self.service = None
        self.last_service_attempt = rospy.Time(0)
        rospy.loginfo("maritime dynamic obstacles mode=%s count=%d", self.dynamic_mode, len(self.obstacles))

    def connect_service(self):
        if self.service is not None:
            return True
        now = rospy.Time.now()
        if self.last_service_attempt and (now - self.last_service_attempt).to_sec() < 2.0:
            return False
        self.last_service_attempt = now
        try:
            rospy.wait_for_service(self.service_name, timeout=0.2)
            self.service = rospy.ServiceProxy(self.service_name, SetModelState)
            rospy.loginfo("connected dynamic obstacle controller to %s", self.service_name)
            return True
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "waiting for Gazebo model-state service %s: %s", self.service_name, exc)
            return False

    def publish_obstacle_state(self, obstacle, time_sec):
        center = dynamic_obstacle_center(obstacle, time_sec)
        state = ModelState()
        state.model_name = obstacle.get("name", "dynamic_obstacle")
        state.reference_frame = self.reference_frame
        state.pose.position.x = center[0]
        state.pose.position.y = center[1]
        state.pose.position.z = center[2]
        state.pose.orientation = quaternion_from_yaw(dynamic_obstacle_yaw(obstacle, time_sec))
        self.service(state)

    def spin(self):
        if not self.obstacles:
            return

        rate = rospy.Rate(self.rate_hz)
        try:
            while not rospy.is_shutdown():
                if self.connect_service():
                    now_sec = rospy.Time.now().to_sec()
                    for obstacle in self.obstacles:
                        try:
                            self.publish_obstacle_state(obstacle, now_sec)
                        except Exception as exc:
                            rospy.logwarn_throttle(
                                5.0, "failed to update dynamic obstacle %s: %s",
                                obstacle.get("name", "dynamic_obstacle"), exc)
                            self.service = None
                            break
                rate.sleep()
        except rospy.ROSInterruptException:
            pass


def main():
    rospy.init_node("maritime_dynamic_obstacles")
    DynamicObstacleController().spin()


if __name__ == "__main__":
    main()
