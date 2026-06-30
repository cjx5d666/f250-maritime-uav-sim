#!/usr/bin/env python3
import math
import threading

import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray


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


class LaserScanAdapter:
    def __init__(self):
        rospy.init_node("maritime_laser_scan_adapter")
        self.source_type = rospy.get_param("~source_type", "lidar")
        self.scan_topic = rospy.get_param("~scan_topic", "/maritime/lidar_scan")
        self.raw_cloud_topic = rospy.get_param("~raw_cloud_topic", "/maritime/lidar_points")
        self.output_topic = rospy.get_param("~output_topic", "/maritime/obstacles_cloud")
        self.odom_topic = rospy.get_param("~odom_topic", "/mavros/local_position/odom")
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.sensor_xyz = [float(v) for v in rospy.get_param("~sensor_xyz", [0.12, 0.0, 0.04])]
        self.sensor_pitch_rad = math.radians(float(rospy.get_param("~sensor_pitch_deg", 0.0)))
        self.min_z = float(rospy.get_param("~min_z", 1.0))
        self.max_z = float(rospy.get_param("~max_z", 14.0))
        self.voxel_size = float(rospy.get_param("~voxel_size", 0.2))
        self.max_points = max(1, int(rospy.get_param("~max_points", 12000)))
        self.delay_scan_until_odom = bool(rospy.get_param("~delay_scan_until_odom", True))
        self.enable_debug_markers = bool(rospy.get_param("~enable_debug_markers", True))
        self.debug_marker_topic = rospy.get_param("~debug_marker_topic", "/maritime/lidar_debug_markers")
        self.debug_ray_stride = max(1, int(rospy.get_param("~debug_ray_stride", 1)))
        self.debug_ray_max = max(1, int(rospy.get_param("~debug_ray_max", 120)))
        self.odom = None
        self.odom_lock = threading.Lock()
        self.scan_subscriber = None
        self.raw_publisher = rospy.Publisher(self.raw_cloud_topic, PointCloud2, queue_size=1)
        self.output_publisher = rospy.Publisher(self.output_topic, PointCloud2, queue_size=1)
        self.debug_marker_publisher = rospy.Publisher(
            self.debug_marker_topic, MarkerArray, queue_size=1
        ) if self.enable_debug_markers else None
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)
        if not self.delay_scan_until_odom:
            self.subscribe_scan()
        rospy.loginfo("laser scan adapter source=%s scan=%s raw_cloud=%s output=%s",
                      self.source_type, self.scan_topic, self.raw_cloud_topic, self.output_topic)

    def odom_callback(self, msg):
        with self.odom_lock:
            self.odom = msg
        if self.scan_subscriber is None:
            self.subscribe_scan()

    def subscribe_scan(self):
        self.scan_subscriber = rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback, queue_size=1)
        rospy.loginfo("laser scan adapter subscribed to %s", self.scan_topic)

    def sensor_to_body(self, point):
        if abs(self.sensor_pitch_rad) > 1.0e-9:
            c = math.cos(self.sensor_pitch_rad)
            s = math.sin(self.sensor_pitch_rad)
            point = (c * point[0] - s * point[2], point[1], s * point[0] + c * point[2])
        return (
            point[0] + self.sensor_xyz[0],
            point[1] + self.sensor_xyz[1],
            point[2] + self.sensor_xyz[2],
        )

    def world_point(self, sensor_point):
        with self.odom_lock:
            pose = self.odom.pose.pose
        world_delta = rotate(quaternion_matrix(pose.orientation), self.sensor_to_body(sensor_point))
        return (
            pose.position.x + world_delta[0],
            pose.position.y + world_delta[1],
            pose.position.z + world_delta[2],
        )

    def voxel_key(self, point):
        scale = 1.0 / self.voxel_size
        return (int(math.floor(point[0] * scale)), int(math.floor(point[1] * scale)), int(math.floor(point[2] * scale)))

    def publish_debug_markers(self, header, origin, rays):
        if self.debug_marker_publisher is None:
            return

        delete_all = Marker()
        delete_all.header = header
        delete_all.action = Marker.DELETEALL

        ray_marker = Marker()
        ray_marker.header = header
        ray_marker.ns = "lidar_actual_rays"
        ray_marker.id = 1
        ray_marker.type = Marker.LINE_LIST
        ray_marker.action = Marker.ADD
        ray_marker.pose.orientation.w = 1.0
        ray_marker.scale.x = 0.045
        ray_marker.color = ColorRGBA(0.0, 0.85, 1.0, 0.30)
        for start, end in rays:
            ray_marker.points.append(Point(x=start[0], y=start[1], z=start[2]))
            ray_marker.points.append(Point(x=end[0], y=end[1], z=end[2]))

        origin_marker = Marker()
        origin_marker.header = header
        origin_marker.ns = "lidar_sensor_origin"
        origin_marker.id = 2
        origin_marker.type = Marker.SPHERE
        origin_marker.action = Marker.ADD
        origin_marker.pose.position = Point(x=origin[0], y=origin[1], z=origin[2])
        origin_marker.pose.orientation.w = 1.0
        origin_marker.scale.x = 0.45
        origin_marker.scale.y = 0.45
        origin_marker.scale.z = 0.45
        origin_marker.color = ColorRGBA(0.0, 1.0, 1.0, 0.85)

        self.debug_marker_publisher.publish(MarkerArray(markers=[delete_all, ray_marker, origin_marker]))

    def scan_callback(self, msg):
        if self.odom is None:
            rospy.logwarn_throttle(5.0, "waiting for odom before adapting %s", self.scan_topic)
            return
        points = []
        rays = []
        occupied = set()
        origin_world = self.world_point((0.0, 0.0, 0.0))
        angle = msg.angle_min
        for index, value in enumerate(msg.ranges):
            if math.isfinite(value) and msg.range_min <= value <= msg.range_max:
                world = self.world_point((value * math.cos(angle), value * math.sin(angle), 0.0))
                if self.min_z <= world[2] <= self.max_z:
                    if index % self.debug_ray_stride == 0 and len(rays) < self.debug_ray_max:
                        rays.append((origin_world, world))
                    key = self.voxel_key(world)
                    if key not in occupied:
                        occupied.add(key)
                        points.append(world)
                        if len(points) >= self.max_points:
                            break
            angle += msg.angle_increment
        header = Header(stamp=msg.header.stamp, frame_id=self.frame_id)
        cloud = pc2.create_cloud_xyz32(header, points)
        self.raw_publisher.publish(cloud)
        self.output_publisher.publish(cloud)
        self.publish_debug_markers(header, origin_world, rays)
        rospy.loginfo_throttle(5.0, "adapted lidar scan: %d points", len(points))


def main():
    LaserScanAdapter()
    rospy.spin()


if __name__ == "__main__":
    main()
