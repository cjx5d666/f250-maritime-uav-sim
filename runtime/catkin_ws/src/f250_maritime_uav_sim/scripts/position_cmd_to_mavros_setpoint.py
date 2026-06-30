#!/usr/bin/env python3
import json
import math
import os
import sys

import rospy
import rostopic
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import Odometry
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from maritime_scene_utils import load_scene, scene_box, scene_waypoints


def quaternion_from_yaw(yaw):
    return Quaternion(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def wrap_angle_rad(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def nested_position(msg):
    for attr in ("position", "pos", "p"):
        if hasattr(msg, attr):
            value = getattr(msg, attr)
            if all(hasattr(value, axis) for axis in ("x", "y", "z")):
                return float(value.x), float(value.y), float(value.z)
    if hasattr(msg, "pose") and hasattr(msg.pose, "position"):
        value = msg.pose.position
        return float(value.x), float(value.y), float(value.z)
    if all(hasattr(msg, axis) for axis in ("x", "y", "z")):
        return float(msg.x), float(msg.y), float(msg.z)
    return None


def yaw_from_msg(msg, default_yaw):
    if hasattr(msg, "yaw"):
        return float(msg.yaw)
    if hasattr(msg, "heading"):
        return float(msg.heading)
    return default_yaw


class PositionCommandBridge:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/planning/pos_cmd")
        self.output_topic = rospy.get_param("~output_topic", "/mavros/setpoint_position/local")
        self.velocity_output_topic = rospy.get_param(
            "~velocity_output_topic", "/mavros/setpoint_raw/local")
        self.control_mode_topic = rospy.get_param(
            "~control_mode_topic", "/maritime/fc_control_mode")
        self.state_topic = rospy.get_param("~state_topic", "/mavros/state")
        self.odom_topic = rospy.get_param("~odom_topic", "/mavros/local_position/odom")
        self.active_goal_topic = rospy.get_param("~active_goal_topic", "/maritime/active_goal")
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.command_timeout = float(rospy.get_param("~command_timeout_sec", 1.0))
        self.rate_hz = float(rospy.get_param("~rate_hz", 20.0))
        self.max_yaw_rate_rad_s = float(rospy.get_param("~max_yaw_rate_rad_s", 0.0))
        self.max_setpoint_speed_mps = float(rospy.get_param("~max_setpoint_speed_mps", 0.0))
        self.max_setpoint_z_rate_mps = float(rospy.get_param("~max_setpoint_z_rate_mps", 0.0))
        self.max_setpoint_lead_m = float(rospy.get_param("~max_setpoint_lead_m", 0.0))
        self.ignore_command_yaw = bool(rospy.get_param("~ignore_command_yaw", False))
        self.hold_last_command_on_timeout = bool(rospy.get_param("~hold_last_command_on_timeout", True))
        self.auto_offboard_arm = bool(rospy.get_param("~auto_offboard_arm", False))
        self.arm_after_offboard = bool(rospy.get_param("~arm_after_offboard", True))
        self.require_planner_command_for_offboard = bool(
            rospy.get_param("~require_planner_command_for_offboard", True))
        self.offboard_mode = rospy.get_param("~offboard_mode", "OFFBOARD")
        self.min_setpoint_stream_sec = float(rospy.get_param("~min_setpoint_stream_sec", 2.0))
        self.require_odom_before_offboard = bool(rospy.get_param("~require_odom_before_offboard", True))
        self.min_odom_stream_sec = float(rospy.get_param("~min_odom_stream_sec", 1.0))
        self.service_retry_sec = float(rospy.get_param("~service_retry_sec", 2.0))
        self.set_mode_service_name = rospy.get_param("~set_mode_service", "/mavros/set_mode")
        self.arming_service_name = rospy.get_param("~arming_service", "/mavros/cmd/arming")
        self.command_long_service_name = rospy.get_param("~command_long_service", "/mavros/cmd/command")
        self.hover = (
            float(rospy.get_param("~hover_x", 0.0)),
            float(rospy.get_param("~hover_y", 0.0)),
            float(rospy.get_param("~hover_z", 3.0)),
            float(rospy.get_param("~hover_yaw", 0.0)),
        )
        self.scene_config = rospy.get_param("~scene_config", "")
        self.landing_enabled = bool(rospy.get_param("~landing_enabled", False))
        self.landing_status_topic = rospy.get_param("~landing_status_topic", "/maritime/landing_status")
        self.last_pose = None
        self.last_velocity_target = None
        self.control_mode = "position"
        self.last_command_time = None
        self.last_limited_yaw = None
        self.last_yaw_limit_time = None
        self.last_limited_position = None
        self.last_position_limit_time = None
        self.last_odom_time = None
        self.first_odom_time = None
        self.last_position = None
        self.last_yaw = None
        self.active_goal_position = None
        self.setpoint_count = 0
        self.started = rospy.Time.now()
        self.last_service_request = rospy.Time(0)
        self.mavros_state = None
        self.subscriber = None
        self.subscriber_class = None
        self.publisher = rospy.Publisher(self.output_topic, PoseStamped, queue_size=1)
        self.velocity_publisher = rospy.Publisher(self.velocity_output_topic, PositionTarget, queue_size=1)
        self.configure_landing()
        self.setup_offboard_services()
        self.odom_subscriber = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)
        self.active_goal_subscriber = rospy.Subscriber(
            self.active_goal_topic, PoseStamped, self.active_goal_callback, queue_size=1)
        self.control_mode_subscriber = rospy.Subscriber(
            self.control_mode_topic, String, self.control_mode_callback, queue_size=1)
        self.discovery_timer = rospy.Timer(rospy.Duration(0.5), self.discovery_callback)
        self.publish_timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.publish_callback)
        if self.landing_enabled:
            self.status_publisher = rospy.Publisher(self.landing_status_topic, String, queue_size=1)
            self.status_timer = rospy.Timer(rospy.Duration(0.5), self.status_callback)

    def setup_offboard_services(self):
        self.state_subscriber = None
        self.set_mode = None
        self.arming = None
        if not self.auto_offboard_arm and not self.landing_enabled:
            return
        try:
            from mavros_msgs.msg import State
            from mavros_msgs.srv import CommandBool, CommandLong, SetMode
            self.state_subscriber = rospy.Subscriber(self.state_topic, State, self.state_callback, queue_size=1)
            self.set_mode = rospy.ServiceProxy(self.set_mode_service_name, SetMode)
            self.arming = rospy.ServiceProxy(self.arming_service_name, CommandBool)
            self.command_long = rospy.ServiceProxy(self.command_long_service_name, CommandLong)
            if self.auto_offboard_arm:
                self.control_timer = rospy.Timer(rospy.Duration(0.5), self.control_callback)
        except Exception as exc:
            rospy.logwarn("PX4 mode/arming services unavailable: %s", exc)
            self.auto_offboard_arm = False
            self.landing_can_disarm = False

    def configure_landing(self):
        self.landing_can_disarm = True
        self.landing_state = "disabled"
        self.landing_center = None
        self.landing_size = None
        self.landing_yaw = 0.0
        self.landing_goal_position = None
        self.landing_zone_entered = None
        self.landing_descent_started = None
        self.landing_descent_start_z = None
        self.landing_touchdown_entered = None
        self.landing_disarm_requested = False
        self.landing_disarm_success = False
        self.landing_disarm_attempts = 0
        self.landing_goaround_reason = ""
        self.last_disarm_request = rospy.Time(0)
        self.landing_last_target_z = None
        if not self.landing_enabled:
            return

        scene = load_scene(self.scene_config)
        acceptance = scene.get("acceptance") or {}
        box = scene_box(scene, "landing_deck_zone", required=True)
        top_z = box["center"][2] + box["size"][2] / 2.0
        final_waypoint_z = self.hover[2]
        waypoints = scene_waypoints(scene)
        if waypoints:
            self.landing_goal_position = waypoints[-1]["position"]
            final_waypoint_z = float(self.landing_goal_position[2])

        self.landing_center = box["center"]
        self.landing_size = box["size"]
        self.landing_yaw = float(box.get("yaw", 0.0))
        self.landing_hold_sec = float(rospy.get_param(
            "~landing_hold_sec",
            acceptance.get("landing_hold_sec", acceptance.get("final_zone_hold_sec", 2.0))))
        self.landing_descent_rate_mps = float(rospy.get_param(
            "~landing_descent_rate_mps", acceptance.get("landing_descent_rate_mps", 0.35)))
        self.landing_setpoint_z_m = float(rospy.get_param(
            "~landing_setpoint_z_m", acceptance.get("landing_setpoint_z_m", top_z - 0.25)))
        self.landing_touchdown_z_m = float(rospy.get_param(
            "~landing_touchdown_z_m", acceptance.get("landing_touchdown_z_m", top_z + 0.18)))
        self.landing_touchdown_hold_sec = float(rospy.get_param(
            "~landing_touchdown_hold_sec", acceptance.get("landing_touchdown_hold_sec", 1.0)))
        self.landing_disarm_retry_sec = float(rospy.get_param(
            "~landing_disarm_retry_sec", acceptance.get("landing_disarm_retry_sec", 1.0)))
        self.landing_force_disarm = bool(rospy.get_param(
            "~landing_force_disarm", acceptance.get("landing_force_disarm", True)))
        self.landing_force_disarm_after_attempts = int(rospy.get_param(
            "~landing_force_disarm_after_attempts", acceptance.get("landing_force_disarm_after_attempts", 2)))
        self.landing_abort_margin_m = float(rospy.get_param(
            "~landing_abort_margin_m", acceptance.get("landing_abort_margin_m", 0.8)))
        self.landing_goaround_z_m = float(rospy.get_param(
            "~landing_goaround_z_m", acceptance.get("landing_goaround_z_m", max(final_waypoint_z, 5.0))))
        self.landing_goal_trigger_radius_m = float(rospy.get_param(
            "~landing_goal_trigger_radius_m", acceptance.get("landing_goal_trigger_radius_m", 0.9)))
        self.landing_state = "tracking"
        rospy.loginfo("landing enabled: center=%s setpoint_z=%.2f touchdown_z=%.2f",
                      self.landing_center, self.landing_setpoint_z_m, self.landing_touchdown_z_m)

    def subscribe_with_class(self, msg_class):
        if self.subscriber is not None and self.subscriber_class == msg_class:
            return
        if self.subscriber is not None:
            self.subscriber.unregister()
        self.subscriber = rospy.Subscriber(self.input_topic, msg_class, self.position_cmd_callback, queue_size=1)
        self.subscriber_class = msg_class
        rospy.loginfo("subscribed to %s as %s", self.input_topic, msg_class.__name__)

    def discovery_callback(self, _event):
        msg_class, real_topic, _eval_fn = rostopic.get_topic_class(self.input_topic, blocking=False)
        if msg_class is not None:
            self.subscribe_with_class(msg_class)
            return
        if self.subscriber is None:
            try:
                from quadrotor_msgs.msg import PositionCommand
                self.subscribe_with_class(PositionCommand)
            except Exception:
                pass

    def make_pose(self, x, y, z, yaw):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation = quaternion_from_yaw(yaw)
        return pose

    def make_velocity_target(self, vx, vy, z, yaw):
        target = PositionTarget()
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = self.frame_id
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        target.type_mask = (
            PositionTarget.IGNORE_PX
            | PositionTarget.IGNORE_PY
            | PositionTarget.IGNORE_VZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.FORCE
            | PositionTarget.IGNORE_YAW_RATE
        )
        target.position.z = z
        target.velocity.x = vx
        target.velocity.y = vy
        target.yaw = yaw
        return target

    def hover_pose(self):
        return self.make_pose(*self.hover)

    def limited_pose(self, target_xyz, target_yaw, now=None):
        if now is None:
            now = rospy.Time.now()
        yaw = self.limited_yaw(float(target_yaw), now)
        position = self.limited_position(target_xyz, now)
        return self.make_pose(position[0], position[1], position[2], yaw)

    def hover_pose_limited(self):
        return self.limited_pose(self.hover[:3], self.hover[3], rospy.Time.now())

    def limited_yaw(self, target_yaw, now):
        yaw = target_yaw
        valid_time = now is not None and now.to_sec() > 0.0
        if self.max_yaw_rate_rad_s > 0.0:
            reference_yaw = self.last_limited_yaw if self.last_limited_yaw is not None else self.last_yaw
            reference_time = self.last_yaw_limit_time
            if valid_time and reference_time is not None and reference_yaw is not None:
                dt = (now - reference_time).to_sec()
                if dt > 0.0:
                    max_delta = self.max_yaw_rate_rad_s * dt
                    delta = wrap_angle_rad(target_yaw - reference_yaw)
                    if abs(delta) > max_delta:
                        yaw = reference_yaw + math.copysign(max_delta, delta)
            elif reference_yaw is not None:
                yaw = reference_yaw
                delta = wrap_angle_rad(target_yaw - reference_yaw)
                max_delta = self.max_yaw_rate_rad_s / max(self.rate_hz, 1.0)
                if abs(delta) > max_delta:
                    yaw = reference_yaw + math.copysign(max_delta, delta)
            self.last_limited_yaw = yaw
            self.last_yaw_limit_time = now if valid_time else None
        return yaw

    def limited_position(self, target, now):
        position = [float(value) for value in target]
        valid_time = now is not None and now.to_sec() > 0.0
        limits_enabled = self.max_setpoint_speed_mps > 0.0 or self.max_setpoint_z_rate_mps > 0.0
        if limits_enabled:
            reference_position = self.last_limited_position if self.last_limited_position is not None else self.last_position
            reference_time = self.last_position_limit_time
            if valid_time and reference_time is not None and reference_position is not None:
                dt = (now - reference_time).to_sec()
                if dt > 0.0:
                    limited = list(reference_position)
                    dx = position[0] - limited[0]
                    dy = position[1] - limited[1]
                    dz = position[2] - limited[2]
                    if self.max_setpoint_speed_mps > 0.0:
                        horizontal_distance = math.hypot(dx, dy)
                        max_horizontal = self.max_setpoint_speed_mps * dt
                        if horizontal_distance > max_horizontal > 0.0:
                            scale = max_horizontal / horizontal_distance
                            dx *= scale
                            dy *= scale
                        limited[0] += dx
                        limited[1] += dy
                    else:
                        limited[0] = position[0]
                        limited[1] = position[1]
                    if self.max_setpoint_z_rate_mps > 0.0:
                        max_z_delta = self.max_setpoint_z_rate_mps * dt
                        if abs(dz) > max_z_delta:
                            dz = math.copysign(max_z_delta, dz)
                        limited[2] += dz
                    else:
                        limited[2] = position[2]
                    position = limited
            elif reference_position is not None:
                limited = list(reference_position)
                dx = position[0] - limited[0]
                dy = position[1] - limited[1]
                dz = position[2] - limited[2]
                if self.max_setpoint_speed_mps > 0.0:
                    horizontal_distance = math.hypot(dx, dy)
                    max_horizontal = self.max_setpoint_speed_mps / max(self.rate_hz, 1.0)
                    if horizontal_distance > max_horizontal > 0.0:
                        scale = max_horizontal / horizontal_distance
                        dx *= scale
                        dy *= scale
                    limited[0] += dx
                    limited[1] += dy
                else:
                    limited[0] = position[0]
                    limited[1] = position[1]
                if self.max_setpoint_z_rate_mps > 0.0:
                    max_z_delta = self.max_setpoint_z_rate_mps / max(self.rate_hz, 1.0)
                    if abs(dz) > max_z_delta:
                        dz = math.copysign(max_z_delta, dz)
                    limited[2] += dz
                else:
                    limited[2] = position[2]
                position = limited
        if self.max_setpoint_lead_m > 0.0 and self.last_position is not None:
            dx = position[0] - self.last_position[0]
            dy = position[1] - self.last_position[1]
            horizontal_distance = math.hypot(dx, dy)
            if horizontal_distance > self.max_setpoint_lead_m > 0.0:
                scale = self.max_setpoint_lead_m / horizontal_distance
                position[0] = self.last_position[0] + dx * scale
                position[1] = self.last_position[1] + dy * scale
        if limits_enabled:
            self.last_limited_position = position
            self.last_position_limit_time = now if valid_time else None
        return position

    def position_cmd_callback(self, msg):
        xyz = nested_position(msg)
        if xyz is None:
            rospy.logwarn_throttle(2.0, "planner command on %s has no x/y/z or position fields", self.input_topic)
            return
        now = rospy.Time.now()
        target_yaw = self.hover[3] if self.ignore_command_yaw else yaw_from_msg(msg, self.hover[3])
        yaw = self.limited_yaw(target_yaw, now)
        position = self.limited_position(xyz, now)
        self.last_pose = self.make_pose(position[0], position[1], position[2], yaw)
        if all(hasattr(msg, attr) for attr in ("velocity", "yaw")):
            self.last_velocity_target = self.make_velocity_target(
                float(msg.velocity.x),
                float(msg.velocity.y),
                float(msg.position.z),
                float(target_yaw),
            )
        self.last_command_time = now

    def control_mode_callback(self, msg):
        mode = str(msg.data).strip().lower()
        if mode not in ("position", "velocity"):
            rospy.logwarn_throttle(2.0, "ignoring unsupported control mode %r", mode)
            return
        if mode != self.control_mode:
            rospy.loginfo("setpoint bridge control mode: %s", mode)
        self.control_mode = mode

    def odom_callback(self, msg):
        now = rospy.Time.now()
        self.last_odom_time = now
        if self.first_odom_time is None:
            self.first_odom_time = now
        pos = msg.pose.pose.position
        self.last_position = [float(pos.x), float(pos.y), float(pos.z)]
        self.last_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.update_landing_state()

    def active_goal_callback(self, msg):
        pos = msg.pose.position
        self.active_goal_position = [float(pos.x), float(pos.y), float(pos.z)]

    def active_pose(self):
        landing_pose = self.landing_override_pose()
        if landing_pose is not None:
            return landing_pose
        if self.last_pose is None or self.last_command_time is None:
            return self.hover_pose_limited()
        command_age = (rospy.Time.now() - self.last_command_time).to_sec()
        if command_age > self.command_timeout and not self.hold_last_command_on_timeout:
            return self.hover_pose_limited()
        self.last_pose.header.stamp = rospy.Time.now()
        self.last_pose.header.frame_id = self.frame_id
        return self.last_pose

    def active_velocity_target(self):
        if self.last_velocity_target is None or self.last_command_time is None:
            z = self.last_position[2] if self.last_position else self.hover[2]
            yaw = self.last_yaw if self.last_yaw is not None else self.hover[3]
            return self.make_velocity_target(0.0, 0.0, z, yaw)
        command_age = (rospy.Time.now() - self.last_command_time).to_sec()
        if command_age > self.command_timeout:
            z = self.last_position[2] if self.last_position else self.hover[2]
            yaw = self.last_yaw if self.last_yaw is not None else self.hover[3]
            return self.make_velocity_target(0.0, 0.0, z, yaw)
        self.last_velocity_target.header.stamp = rospy.Time.now()
        self.last_velocity_target.header.frame_id = self.frame_id
        return self.last_velocity_target

    def landing_override_pose(self):
        if not self.landing_enabled or self.landing_center is None:
            return None
        if self.landing_state == "descending":
            now = rospy.Time.now()
            if self.landing_descent_started is None:
                self.landing_descent_started = now
                self.landing_descent_start_z = self.last_position[2] if self.last_position else self.hover[2]
            elapsed = max(0.0, (now - self.landing_descent_started).to_sec())
            target_z = max(
                self.landing_setpoint_z_m,
                float(self.landing_descent_start_z) - self.landing_descent_rate_mps * elapsed)
            self.landing_last_target_z = target_z
            return self.make_pose(self.landing_center[0], self.landing_center[1], target_z, self.landing_yaw)
        if self.landing_state in ("touchdown", "disarmed"):
            self.landing_last_target_z = self.landing_setpoint_z_m
            return self.make_pose(
                self.landing_center[0], self.landing_center[1],
                self.landing_setpoint_z_m, self.landing_yaw)
        if self.landing_state == "goaround":
            self.landing_last_target_z = self.landing_goaround_z_m
            return self.make_pose(
                self.landing_center[0], self.landing_center[1],
                self.landing_goaround_z_m, self.landing_yaw)
        return None

    def inside_landing_zone(self, margin=0.0):
        if self.last_position is None or self.landing_center is None or self.landing_size is None:
            return False
        dx = self.last_position[0] - self.landing_center[0]
        dy = self.last_position[1] - self.landing_center[1]
        cos_yaw = math.cos(self.landing_yaw)
        sin_yaw = math.sin(self.landing_yaw)
        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy
        return (
            abs(local_x) <= float(self.landing_size[0]) / 2.0 + margin and
            abs(local_y) <= float(self.landing_size[1]) / 2.0 + margin
        )

    def planner_near_landing_goal(self):
        if self.landing_goal_position is None or self.last_pose is None:
            return False
        pos = self.last_pose.pose.position
        distance = math.sqrt(
            (float(pos.x) - self.landing_goal_position[0]) ** 2 +
            (float(pos.y) - self.landing_goal_position[1]) ** 2 +
            (float(pos.z) - self.landing_goal_position[2]) ** 2
        )
        return distance <= self.landing_goal_trigger_radius_m

    def final_landing_goal_is_active(self):
        if self.landing_goal_position is None:
            return False
        if self.active_goal_position is None:
            return True
        distance = math.sqrt(
            (self.active_goal_position[0] - self.landing_goal_position[0]) ** 2 +
            (self.active_goal_position[1] - self.landing_goal_position[1]) ** 2 +
            (self.active_goal_position[2] - self.landing_goal_position[2]) ** 2
        )
        return distance <= self.landing_goal_trigger_radius_m

    def update_landing_state(self):
        if not self.landing_enabled or self.last_position is None:
            return
        now = rospy.Time.now()
        inside = self.inside_landing_zone()
        inside_abort_zone = self.inside_landing_zone(self.landing_abort_margin_m)

        if self.landing_state == "tracking":
            if inside and self.final_landing_goal_is_active() and self.planner_near_landing_goal():
                self.landing_state = "final_zone_hold"
                self.landing_zone_entered = now
            return

        if self.landing_state == "final_zone_hold":
            if not inside:
                self.landing_state = "tracking"
                self.landing_zone_entered = None
                return
            if self.landing_zone_entered and (now - self.landing_zone_entered).to_sec() >= self.landing_hold_sec:
                self.landing_state = "descending"
                self.landing_descent_started = now
                self.landing_descent_start_z = self.last_position[2]
            return

        if self.landing_state == "descending":
            if not inside_abort_zone:
                self.trigger_goaround("left landing zone during descent")
                return
            if inside and self.last_position[2] <= self.landing_touchdown_z_m:
                self.landing_state = "touchdown"
                self.landing_touchdown_entered = now
            return

        if self.landing_state == "touchdown":
            if not inside_abort_zone:
                self.trigger_goaround("left landing zone after touchdown")
                return
            if self.landing_touchdown_entered and (
                    now - self.landing_touchdown_entered).to_sec() >= self.landing_touchdown_hold_sec:
                self.request_disarm()
            if self.mavros_state is not None and not self.mavros_state.armed and self.landing_disarm_requested:
                self.landing_state = "disarmed"
            return

        if self.landing_state == "disarmed":
            return

    def trigger_goaround(self, reason):
        self.landing_state = "goaround"
        self.landing_goaround_reason = reason
        rospy.logwarn("landing go-around: %s", reason)

    def request_disarm(self):
        if self.arming is None or not self.landing_can_disarm:
            return
        now = rospy.Time.now()
        if (now - self.last_disarm_request).to_sec() < self.landing_disarm_retry_sec:
            return
        self.last_disarm_request = now
        self.landing_disarm_requested = True
        self.landing_disarm_attempts += 1
        try:
            response = self.arming(False)
            if getattr(response, "success", False):
                self.landing_disarm_success = True
                rospy.loginfo("requested PX4 disarm after landing")
                return
            if self.landing_force_disarm and self.landing_disarm_attempts >= self.landing_force_disarm_after_attempts:
                self.request_force_disarm()
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(5.0, "PX4 disarm request failed: %s", exc)
            if self.landing_force_disarm and self.landing_disarm_attempts >= self.landing_force_disarm_after_attempts:
                self.request_force_disarm()

    def request_force_disarm(self):
        if not hasattr(self, "command_long") or self.command_long is None:
            return
        try:
            # MAV_CMD_COMPONENT_ARM_DISARM with param2=21196 requests force-disarm in PX4 SITL.
            response = self.command_long(
                broadcast=False,
                command=400,
                confirmation=0,
                param1=0.0,
                param2=21196.0,
                param3=0.0,
                param4=0.0,
                param5=0.0,
                param6=0.0,
                param7=0.0,
            )
            if getattr(response, "success", False):
                self.landing_disarm_success = True
                rospy.loginfo("requested PX4 force-disarm after landing")
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(5.0, "PX4 force-disarm request failed: %s", exc)

    def landing_report(self):
        return {
            "enabled": self.landing_enabled,
            "state": self.landing_state,
            "position": self.last_position,
            "landing_center": self.landing_center,
            "landing_size": self.landing_size,
            "landing_goal_position": self.landing_goal_position,
            "target_z_m": self.landing_last_target_z,
            "setpoint_z_m": getattr(self, "landing_setpoint_z_m", None),
            "touchdown_z_m": getattr(self, "landing_touchdown_z_m", None),
            "inside_zone": self.inside_landing_zone() if self.landing_enabled else False,
            "disarm_requested": self.landing_disarm_requested,
            "disarm_attempts": self.landing_disarm_attempts,
            "disarm_success": self.landing_disarm_success,
            "goaround_reason": self.landing_goaround_reason,
        }

    def status_callback(self, _event):
        self.status_publisher.publish(String(data=json.dumps(self.landing_report(), sort_keys=True)))

    def publish_callback(self, _event):
        if self.control_mode == "velocity" and not self.landing_enabled:
            self.velocity_publisher.publish(self.active_velocity_target())
        else:
            self.publisher.publish(self.active_pose())
        self.setpoint_count += 1

    def state_callback(self, msg):
        self.mavros_state = msg

    def command_is_fresh(self):
        if self.last_command_time is None:
            return False
        return (rospy.Time.now() - self.last_command_time).to_sec() <= self.command_timeout

    def ready_for_offboard(self):
        now = rospy.Time.now()
        if self.mavros_state is None or not self.mavros_state.connected:
            return False
        if (now - self.started).to_sec() < self.min_setpoint_stream_sec:
            return False
        if self.setpoint_count < max(1, int(self.rate_hz * self.min_setpoint_stream_sec)):
            return False
        if self.require_odom_before_offboard:
            if self.last_position is None or self.last_odom_time is None or self.first_odom_time is None:
                return False
            if (now - self.last_odom_time).to_sec() > 0.5:
                return False
            if (now - self.first_odom_time).to_sec() < self.min_odom_stream_sec:
                return False
        if self.require_planner_command_for_offboard and not self.command_is_fresh():
            return False
        return True

    def service_due(self):
        return (rospy.Time.now() - self.last_service_request).to_sec() >= self.service_retry_sec

    def control_callback(self, _event):
        if not self.auto_offboard_arm or not self.ready_for_offboard() or not self.service_due():
            return
        if self.landing_enabled and self.landing_state in ("descending", "touchdown", "disarmed"):
            return
        self.last_service_request = rospy.Time.now()
        try:
            if self.mavros_state.mode != self.offboard_mode:
                response = self.set_mode(base_mode=0, custom_mode=self.offboard_mode)
                if response.mode_sent:
                    rospy.loginfo("requested PX4 mode %s", self.offboard_mode)
                return
            if self.arm_after_offboard and not self.mavros_state.armed:
                response = self.arming(True)
                if response.success:
                    rospy.loginfo("requested PX4 arm")
                else:
                    rospy.logwarn_throttle(5.0, "PX4 arm request after %s was rejected", self.offboard_mode)
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(5.0, "PX4 OFFBOARD/ARM service request failed: %s", exc)


def main():
    rospy.init_node("position_cmd_to_mavros_setpoint")
    PositionCommandBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
