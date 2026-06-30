#!/usr/bin/env python3
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from maritime_clearance import static_planner_obstacles
from maritime_scene_utils import load_scene


def make_line_marker(frame_id, marker_id, ns, points, rgba, width=0.08, stamp=None):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp if stamp is not None else marker_stamp()
    marker.ns = ns
    marker.id = marker_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = float(width)
    marker.color = rgba
    marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in points]
    marker.lifetime = rospy.Duration(0.0)
    return marker


def make_arrow_marker(frame_id, marker_id, ns, start, end, rgba, shaft=0.18, head=0.55, stamp=None):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp if stamp is not None else marker_stamp()
    marker.ns = ns
    marker.id = marker_id
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale = Vector3(x=float(shaft), y=float(head), z=float(head))
    marker.color = rgba
    marker.points = [Point(x=float(start[0]), y=float(start[1]), z=float(start[2])),
                     Point(x=float(end[0]), y=float(end[1]), z=float(end[2]))]
    marker.lifetime = rospy.Duration(0.0)
    return marker


def make_text_marker(frame_id, marker_id, ns, text, center, rgba, size=1.2, stamp=None):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp if stamp is not None else marker_stamp()
    marker.ns = ns
    marker.id = marker_id
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.pose.position = Point(x=float(center[0]), y=float(center[1]), z=float(center[2]))
    marker.pose.orientation.w = 1.0
    marker.scale.z = float(size)
    marker.color = rgba
    marker.text = text
    marker.lifetime = rospy.Duration(0.0)
    return marker


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def make_box_outline_marker(frame_id, marker_id, ns, center, size, rgba, width=0.08, stamp=None):
    cx, cy, cz = center
    sx, sy, sz = size[0] * 0.5, size[1] * 0.5, size[2] * 0.5
    corners = [
        (cx - sx, cy - sy, cz - sz), (cx + sx, cy - sy, cz - sz),
        (cx + sx, cy + sy, cz - sz), (cx - sx, cy + sy, cz - sz),
        (cx - sx, cy - sy, cz + sz), (cx + sx, cy - sy, cz + sz),
        (cx + sx, cy + sy, cz + sz), (cx - sx, cy + sy, cz + sz),
    ]
    edge_pairs = [
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    ]
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp if stamp is not None else marker_stamp()
    marker.ns = ns
    marker.id = marker_id
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = float(width)
    marker.color = rgba
    for a, b in edge_pairs:
        marker.points.append(Point(x=corners[a][0], y=corners[a][1], z=corners[a][2]))
        marker.points.append(Point(x=corners[b][0], y=corners[b][1], z=corners[b][2]))
    marker.lifetime = rospy.Duration(0.0)
    return marker


def perception_display_label(source):
    normalized = str(source or "lidar").strip().lower()
    if normalized == "depth":
        return "Depth"
    if normalized == "lidar":
        return "LiDAR"
    return normalized or "Sensor"


def sensor_context_markers(frame_id, start_id, odom, radius_m, local_range_xy, local_range_z, perception_source="lidar", stamp=None):
    if odom is None:
        return []
    pose = odom.pose.pose
    ox, oy, oz = pose.position.x, pose.position.y, pose.position.z
    yaw = yaw_from_quaternion(pose.orientation)
    z = oz + 0.08
    points = []
    for index in range(97):
        angle = 2.0 * math.pi * index / 96.0
        points.append((ox + radius_m * math.cos(angle), oy + radius_m * math.sin(angle), z))
    c = math.cos(yaw)
    s = math.sin(yaw)
    arrow_len = min(radius_m, 18.0)
    start = (ox, oy, z + 0.25)
    end = (ox + arrow_len * c, oy + arrow_len * s, z + 0.25)
    local_center = (ox, oy, oz)
    local_size = (2.0 * local_range_xy, 2.0 * local_range_xy, 2.0 * local_range_z)
    note = "%s -> EGO inflated" % perception_display_label(perception_source)
    return [
        make_line_marker(frame_id, start_id, "sensor_horizontal_coverage", points, color(0.1, 0.95, 1.0, 0.60), width=0.10, stamp=stamp),
        make_arrow_marker(frame_id, start_id + 1, "uav_forward_axis", start, end, color(0.1, 0.95, 1.0, 0.85), stamp=stamp),
        make_box_outline_marker(frame_id, start_id + 2, "ego_local_planning_window", local_center, local_size, color(0.55, 0.90, 1.0, 0.38), width=0.06, stamp=stamp),
        make_text_marker(frame_id, start_id + 3, "perception_note", note, (ox, oy, z + 2.0), color(0.75, 1.0, 1.0, 0.9), size=1.0, stamp=stamp),
    ]


def marker_stamp():
    try:
        return rospy.Time.now()
    except Exception:
        return rospy.Time(0)


def color(r, g, b, a):
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


def quat_from_yaw(yaw):
    return Quaternion(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def pose_from_obstacle(obstacle):
    pose = Pose()
    center = obstacle["center"]
    pose.position = Point(x=float(center[0]), y=float(center[1]), z=float(center[2]))
    pose.orientation = quat_from_yaw(float(obstacle.get("yaw", 0.0)))
    return pose


def obstacle_height_and_center(obstacle):
    if obstacle["shape"] == "cylinder":
        z_min = float(obstacle["z_min"])
        z_max = float(obstacle["z_max"])
        center = list(obstacle["center"])
        center[2] = 0.5 * (z_min + z_max)
        return max(0.01, z_max - z_min), center
    size = obstacle.get("size", [1.0, 1.0, 1.0])
    return max(0.01, float(size[2])), list(obstacle["center"])


def make_marker(frame_id, marker_id, obstacle, inflation_m, stamp=None):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp if stamp is not None else marker_stamp()
    marker.ns = "static_planning_proxy_envelope"
    marker.id = marker_id
    marker.action = Marker.ADD
    marker.lifetime = rospy.Duration(0.0)
    marker.text = obstacle.get("name", "planner_obstacle")

    height, center = obstacle_height_and_center(obstacle)
    inflated_height = height + 2.0 * inflation_m
    center[2] = center[2]
    pose = pose_from_obstacle(dict(obstacle, center=center))
    marker.pose = pose

    if obstacle["shape"] == "cylinder":
        marker.type = Marker.CYLINDER
        radius = float(obstacle["radius"]) + inflation_m
        marker.scale = Vector3(x=2.0 * radius, y=2.0 * radius, z=inflated_height)
    elif obstacle["shape"] == "box":
        marker.type = Marker.CUBE
        size = obstacle.get("size", [1.0, 1.0, 1.0])
        marker.scale = Vector3(
            x=max(0.01, float(size[0]) + 2.0 * inflation_m),
            y=max(0.01, float(size[1]) + 2.0 * inflation_m),
            z=max(0.01, float(size[2]) + 2.0 * inflation_m),
        )
    else:
        return None

    marker.color = color(1.0, 0.58, 0.05, 0.22)
    return marker


def build_markers(scene, frame_id, inflation_m, dynamic_mode, time_sec, stamp=None, odom=None, lidar_range_m=18.0, local_range_xy=18.0, local_range_z=9.0, perception_source="lidar"):
    stamp = stamp if stamp is not None else marker_stamp()
    markers = []
    delete_all = Marker()
    delete_all.header.frame_id = frame_id
    delete_all.header.stamp = stamp
    delete_all.action = Marker.DELETEALL
    markers.append(delete_all)

    marker_id = 0
    for obstacle in static_planner_obstacles(scene):
        marker = make_marker(frame_id, marker_id, obstacle, inflation_m, stamp=stamp)
        if marker is not None:
            markers.append(marker)
            marker_id += 1

    markers.extend(sensor_context_markers(frame_id, marker_id, odom, lidar_range_m, local_range_xy, local_range_z, perception_source=perception_source, stamp=stamp))
    return MarkerArray(markers=markers)


def main():
    rospy.init_node("maritime_inflated_obstacle_markers")
    scene = load_scene(rospy.get_param("~scene_config", None))
    frame_id = rospy.get_param("~frame_id", scene.get("frame_id", "world"))
    marker_topic = rospy.get_param("~marker_topic", "/maritime/inflated_obstacle_markers")
    dynamic_mode = rospy.get_param("~dynamic_mode", "auto")
    inflation_m = max(0.0, float(rospy.get_param("~inflation_m", 0.5)))
    rate_hz = max(0.2, min(5.0, float(rospy.get_param("~rate_hz", 2.0))))
    odom_topic = rospy.get_param("~odom_topic", "/mavros/local_position/odom")
    lidar_range_m = max(1.0, float(rospy.get_param("~lidar_range_m", 18.0)))
    local_range_xy = max(1.0, float(rospy.get_param("~local_range_xy", 18.0)))
    local_range_z = max(1.0, float(rospy.get_param("~local_range_z", 9.0)))
    perception_source = rospy.get_param("~perception_source", os.environ.get("PERCEPTION_SOURCE", "lidar"))
    latest_odom = {"msg": None}
    rospy.Subscriber(odom_topic, Odometry, lambda msg: latest_odom.__setitem__("msg", msg), queue_size=1)

    publisher = rospy.Publisher(marker_topic, MarkerArray, queue_size=1, latch=True)
    rate = rospy.Rate(rate_hz)
    rospy.loginfo("publishing inflated planner obstacle markers topic=%s inflation=%.3f",
                  marker_topic, inflation_m)
    while not rospy.is_shutdown():
        now = rospy.Time.now().to_sec()
        publisher.publish(build_markers(scene, frame_id, inflation_m, dynamic_mode, now, odom=latest_odom["msg"], lidar_range_m=lidar_range_m, local_range_xy=local_range_xy, local_range_z=local_range_z, perception_source=perception_source))
        rate.sleep()


if __name__ == "__main__":
    main()
