#!/usr/bin/env python3
import math
import threading

import rospy
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


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


def optical_to_body(point, pitch_rad):
    # ROS optical cloud convention: x right, y down, z forward.
    body = (point[2], -point[0], -point[1])
    if abs(pitch_rad) < 1.0e-9:
        return body
    c = math.cos(pitch_rad)
    s = math.sin(pitch_rad)
    return (
        c * body[0] - s * body[2],
        body[1],
        s * body[0] + c * body[2],
    )


class SensorCloudAdapter:
    def __init__(self):
        rospy.init_node("maritime_sensor_cloud_adapter")
        self.source_type = rospy.get_param("~source_type", "depth")
        if self.source_type != "depth":
            rospy.logwarn("sensor cloud adapter expected source_type=depth, got %s", self.source_type)
        self.input_topic = rospy.get_param("~input_topic", "/maritime_depth_camera/points")
        self.output_topic = rospy.get_param("~output_topic", "/maritime/obstacles_cloud")
        self.odom_topic = rospy.get_param("~odom_topic", "/mavros/local_position/odom")
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.camera_xyz = [float(v) for v in rospy.get_param("~camera_xyz", [0.12, 0.0, 0.04])]
        if len(self.camera_xyz) != 3:
            rospy.logwarn("camera_xyz must have 3 values; using F250 depth camera mount [0.12, 0.0, 0.04]")
            self.camera_xyz = [0.12, 0.0, 0.04]
        self.camera_pitch_rad = math.radians(float(rospy.get_param("~camera_pitch_deg", 0.0)))
        self.min_z = float(rospy.get_param("~min_z", 1.0))
        self.max_z = float(rospy.get_param("~max_z", 14.0))
        self.max_range = float(rospy.get_param("~max_range", 40.0))
        self.drop_far_clip = bool(rospy.get_param("~drop_far_clip", True))
        self.far_clip_margin = max(0.0, float(rospy.get_param("~far_clip_margin", 0.2)))
        self.voxel_size = max(1.0e-6, float(rospy.get_param("~voxel_size", 0.25)))
        self.input_stride = max(1, int(rospy.get_param("~input_stride", 2)))
        self.max_points = max(1, int(rospy.get_param("~max_points", 12000)))
        self.odom = None
        self.odom_lock = threading.Lock()
        self.publisher = rospy.Publisher(self.output_topic, PointCloud2, queue_size=1)
        self.odom_subscriber = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)
        self.cloud_subscriber = rospy.Subscriber(self.input_topic, PointCloud2, self.cloud_callback, queue_size=1)
        rospy.loginfo(
            "sensor cloud adapter source=%s input=%s output=%s frame=%s max_range=%.2f drop_far_clip=%s margin=%.2f",
            self.source_type, self.input_topic, self.output_topic, self.frame_id,
            self.max_range, self.drop_far_clip, self.far_clip_margin)

    def odom_callback(self, msg):
        with self.odom_lock:
            self.odom = msg

    def world_point(self, optical_point):
        body_point = optical_to_body(optical_point, self.camera_pitch_rad)
        body_point = (
            body_point[0] + self.camera_xyz[0],
            body_point[1] + self.camera_xyz[1],
            body_point[2] + self.camera_xyz[2],
        )
        with self.odom_lock:
            pose = self.odom.pose.pose
        world_delta = rotate(quaternion_matrix(pose.orientation), body_point)
        return (
            pose.position.x + world_delta[0],
            pose.position.y + world_delta[1],
            pose.position.z + world_delta[2],
        )

    def keep_world_point(self, optical_point, point):
        # Gazebo depth clouds report z as forward optical depth.  Gate on that
        # value so camera mount offset and off-axis components do not turn the
        # configured far clip into a vehicle-origin Euclidean threshold.  The
        # depth plugin emits many no-return samples exactly at cutoffMax; keep
        # those out of the planner cloud so they do not become a moving wall.
        optical_depth = float(optical_point[2])
        if not math.isfinite(optical_depth):
            return False
        if optical_depth > self.max_range + 1.0e-6:
            return False
        if self.drop_far_clip and optical_depth >= max(0.0, self.max_range - self.far_clip_margin):
            return False
        return self.min_z <= point[2] <= self.max_z

    def voxel_key(self, point):
        scale = 1.0 / self.voxel_size
        return (int(math.floor(point[0] * scale)), int(math.floor(point[1] * scale)), int(math.floor(point[2] * scale)))

    def cloud_callback(self, msg):
        if self.odom is None:
            rospy.logwarn_throttle(5.0, "waiting for odom before adapting %s", self.input_topic)
            return
        points = []
        occupied = set()
        seen = 0
        skipped_stride = 0
        skipped_filter = 0
        for index, raw_point in enumerate(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)):
            seen += 1
            if index % self.input_stride:
                skipped_stride += 1
                continue
            world = self.world_point(raw_point)
            if not self.keep_world_point(raw_point, world):
                skipped_filter += 1
                continue
            key = self.voxel_key(world)
            if key in occupied:
                continue
            occupied.add(key)
            points.append(world)
            if len(points) >= self.max_points:
                break
        stamp = msg.header.stamp if msg.header.stamp and msg.header.stamp.to_sec() > 0.0 else rospy.Time.now()
        header = Header(stamp=stamp, frame_id=self.frame_id)
        self.publisher.publish(pc2.create_cloud_xyz32(header, points))
        rospy.loginfo_throttle(
            5.0,
            "adapted %s cloud: kept=%d seen=%d stride_skipped=%d filter_skipped=%d",
            self.source_type, len(points), seen, skipped_stride, skipped_filter)


def main():
    SensorCloudAdapter()
    rospy.spin()


if __name__ == "__main__":
    main()
