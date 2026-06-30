#!/usr/bin/env python3
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from maritime_scene_utils import dynamic_collision_proxy_instances, dynamic_mode_enabled
from maritime_scene_utils import dynamic_obstacle_center, dynamic_obstacle_yaw
from maritime_scene_utils import load_scene, scene_dynamic_obstacles, scene_waypoints
from maritime_scene_utils import scene_box
from maritime_scene_utils import visual_collision_proxy_instances


NS_STATIC_VISUAL_MESH = "f250_maritime_static_visual_mesh"
NS_DYNAMIC_VISUAL_MESH = "f250_maritime_dynamic_visual_mesh"
NS_PERCEPTION_COLLISION_MESH = "f250_maritime_perception_collision_mesh"
NS_ROUTE_MARKERS = "f250_maritime_route_markers"
NS_REFERENCE_PRIMITIVES = "f250_maritime_reference_primitives"
NS_RAW_PLANNER_BOXES = "f250_maritime_raw_planner_boxes"
NS_COLLISION_PROXY_PRIMITIVES = "f250_maritime_collision_proxy_primitives"
NS_DYNAMIC_PERCEPTION_PROXY_PRIMITIVES = "f250_maritime_dynamic_perception_proxy_primitives"


def color(r, g, b, a):
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


def quat_from_yaw(yaw):
    return Quaternion(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quat_from_rpy(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )


def pose_from_center(center, yaw=0.0):
    pose = Pose()
    pose.position = Point(x=center[0], y=center[1], z=center[2])
    pose.orientation = quat_from_yaw(yaw)
    return pose


def pose_from_center_rpy(center, rpy):
    pose = Pose()
    pose.position = Point(x=center[0], y=center[1], z=center[2])
    pose.orientation = quat_from_rpy(float(rpy[0]), float(rpy[1]), float(rpy[2]))
    return pose


def scale3(value):
    if isinstance(value, (list, tuple)):
        vals = [float(v) for v in value[:3]]
        if len(vals) == 1:
            return [vals[0], vals[0], vals[0]]
        if len(vals) == 2:
            return [vals[0], vals[1], 1.0]
        return vals
    v = float(value)
    return [v, v, v]


def bool_param(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def rviz_mesh_resource(mesh_uri):
    text = str(mesh_uri)
    prefix = "model://"
    if not text.startswith(prefix):
        return text
    relative = text[len(prefix):].lstrip("/")
    return "package://f250_maritime_uav_sim/models/%s" % relative


def dynamic_mesh_pose(obstacle, dynamic_time_sec):
    center = list(dynamic_obstacle_center(obstacle, dynamic_time_sec))
    yaw = dynamic_obstacle_yaw(obstacle, dynamic_time_sec)
    mesh_pose = list(obstacle.get("mesh_pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
    while len(mesh_pose) < 6:
        mesh_pose.append(0.0)
    dx, dy, dz = float(mesh_pose[0]), float(mesh_pose[1]), float(mesh_pose[2])
    c = math.cos(yaw)
    s = math.sin(yaw)
    center[0] += c * dx - s * dy
    center[1] += s * dx + c * dy
    center[2] += dz
    return pose_from_center(center, yaw + float(mesh_pose[5]))


def make_marker(frame_id, marker_id, name, marker_type, center, scale, marker_color,
                yaw=0.0, namespace=NS_REFERENCE_PRIMITIVES):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = namespace
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose = pose_from_center(center, yaw)
    marker.scale = Vector3(x=scale[0], y=scale[1], z=scale[2])
    marker.color = marker_color
    marker.lifetime = rospy.Duration(0.0)
    marker.text = name
    return marker


def make_mesh_marker(frame_id, marker_id, obstacle, dynamic_time_sec):
    mesh_uri = obstacle.get("mesh_uri")
    if not mesh_uri:
        return None
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = NS_DYNAMIC_VISUAL_MESH
    marker.id = marker_id
    marker.type = Marker.MESH_RESOURCE
    marker.action = Marker.ADD
    marker.pose = dynamic_mesh_pose(obstacle, dynamic_time_sec)
    sx, sy, sz = scale3(obstacle.get("mesh_scale", obstacle.get("scale", 1.0)))
    marker.scale = Vector3(x=sx, y=sy, z=sz)
    marker.color = color(1.0, 1.0, 1.0, 1.0)
    marker.lifetime = rospy.Duration(0.0)
    marker.mesh_resource = rviz_mesh_resource(mesh_uri)
    marker.mesh_use_embedded_materials = True
    marker.text = obstacle.get("name", "dynamic_obstacle") + "_visual_mesh"
    return marker


def static_mesh_specs(item):
    meshes = item.get("visual_meshes")
    if meshes:
        return meshes
    mesh_uri = item.get("mesh_uri")
    if not mesh_uri:
        return []
    return [{
        "name": item.get("name", "static_visual_mesh"),
        "uri": mesh_uri,
        "scale": item.get("mesh_scale", item.get("scale", 1.0)),
        "pose": item.get("mesh_pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        "material": item.get("mesh_material"),
    }]


def make_static_mesh_marker(frame_id, marker_id, item, mesh):
    return make_mesh_resource_marker(frame_id, marker_id, item, mesh, NS_STATIC_VISUAL_MESH)


def make_mesh_resource_marker(frame_id, marker_id, item, mesh, namespace,
                              color_override=None, embedded_materials=None, text_suffix="visual_mesh"):
    mesh_uri = mesh.get("uri", mesh.get("mesh_uri", item.get("mesh_uri")))
    if not mesh_uri:
        return None
    center = list(item.get("center", [0.0, 0.0, 0.0]))
    if len(center) != 3:
        return None
    rpy = list(item.get("rpy", [0.0, 0.0, float(item.get("yaw", 0.0))]))
    while len(rpy) < 3:
        rpy.append(0.0)
    mesh_pose = list(mesh.get("pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
    while len(mesh_pose) < 6:
        mesh_pose.append(0.0)

    yaw = float(rpy[2])
    dx, dy, dz = float(mesh_pose[0]), float(mesh_pose[1]), float(mesh_pose[2])
    c = math.cos(yaw)
    s = math.sin(yaw)
    center = [
        float(center[0]) + c * dx - s * dy,
        float(center[1]) + s * dx + c * dy,
        float(center[2]) + dz,
    ]
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.MESH_RESOURCE
    marker.action = Marker.ADD
    marker.pose = pose_from_center_rpy(
        center,
        [float(rpy[0]) + float(mesh_pose[3]),
         float(rpy[1]) + float(mesh_pose[4]),
         float(rpy[2]) + float(mesh_pose[5])],
    )
    sx, sy, sz = scale3(mesh.get("scale", item.get("mesh_scale", item.get("scale", 1.0))))
    marker.scale = Vector3(x=sx, y=sy, z=sz)
    mesh_material = mesh.get("material", item.get("mesh_material"))
    if color_override is not None:
        marker.color = color(*color_override)
    else:
        marker.color = color(*(mesh_material if mesh_material else [1.0, 1.0, 1.0, 1.0]))
    marker.lifetime = rospy.Duration(0.0)
    marker.mesh_resource = rviz_mesh_resource(mesh_uri)
    if embedded_materials is None:
        marker.mesh_use_embedded_materials = mesh_material is None and color_override is None
    else:
        marker.mesh_use_embedded_materials = bool(embedded_materials)
    marker.text = "%s_%s" % (item.get("name", mesh.get("name", "static_mesh")), text_suffix)
    return marker


def build_marker_array(scene, frame_id, dynamic_time_sec=0.0, include_dynamic=True,
                       show_reference_primitives=False, show_raw_planner_shapes=False,
                       show_collision_meshes=True):
    markers = []
    delete_all = Marker()
    delete_all.header.frame_id = frame_id
    delete_all.header.stamp = rospy.Time.now()
    delete_all.action = Marker.DELETEALL
    markers.append(delete_all)

    marker_id = 0
    if show_reference_primitives:
        deck = scene_box(scene, "deck", required=True)
        markers.append(make_marker(frame_id, marker_id, deck["name"], Marker.CUBE, deck["center"], deck["size"],
                                   color(0.38, 0.38, 0.34, 0.9)))
        marker_id += 1

        for item in scene.get("ship_hull", []) or []:
            markers.append(make_marker(frame_id, marker_id, item["name"], Marker.CUBE, item["center"], item["size"],
                                       color(0.17, 0.29, 0.36, 0.65)))
            marker_id += 1

        takeoff = scene_box(scene, "takeoff_deck_zone", required=True)
        markers.append(make_marker(frame_id, marker_id, takeoff["name"], Marker.CUBE, takeoff["center"], takeoff["size"],
                                   color(0.18, 0.48, 0.95, 0.9)))
        marker_id += 1

        landing = scene_box(scene, "landing_deck_zone", required=True)
        markers.append(make_marker(frame_id, marker_id, landing["name"], Marker.CUBE, landing["center"], landing["size"],
                                   color(0.15, 0.85, 0.25, 0.9)))
        marker_id += 1

        for pier in scene.get("bridge_piers", []) or []:
            markers.append(make_marker(frame_id, marker_id, pier["name"], Marker.CYLINDER, pier["center"],
                                       [2.0 * pier["radius"], 2.0 * pier["radius"], pier["height"]],
                                       color(0.72, 0.72, 0.68, 0.92)))
            marker_id += 1

        for buoy in scene.get("buoys", []) or []:
            buoy_color = buoy.get("color", [1.0, 0.18, 0.08, 1.0])
            markers.append(make_marker(frame_id, marker_id, buoy["name"], Marker.CYLINDER, buoy["center"],
                                       [2.0 * buoy["radius"], 2.0 * buoy["radius"], buoy["height"]],
                                       color(*buoy_color)))
            marker_id += 1

        for dock in scene.get("docks", []) or []:
            dock_color = dock.get("color", [0.34, 0.28, 0.20, 0.9])
            markers.append(make_marker(frame_id, marker_id, dock["name"], Marker.CUBE, dock["center"], dock["size"],
                                       color(*dock_color)))
            marker_id += 1

        for box in scene.get("visual_boxes", []) or []:
            obj_color = box.get("color", [0.72, 0.74, 0.74, 0.9])
            markers.append(make_marker(frame_id, marker_id, box["name"], Marker.CUBE,
                                       box["center"], box["size"], color(*obj_color)))
            marker_id += 1

    if show_raw_planner_shapes:
        for box in scene.get("box_obstacles", []) or []:
            obj_color = color(0.9, 0.52, 0.14, 0.88) if "container" in box["name"] else color(0.82, 0.82, 0.75, 0.85)
            markers.append(make_marker(frame_id, marker_id, box["name"], Marker.CUBE, box["center"], box["size"], obj_color,
                                       yaw=float(box.get("yaw", 0.0)),
                                       namespace=NS_RAW_PLANNER_BOXES))
            marker_id += 1
        for proxy in visual_collision_proxy_instances(scene, include_cloud_only=True):
            proxy_color = proxy.get("color", [1.0, 0.22, 0.05, 0.28])
            if len(proxy_color) >= 4:
                proxy_color = list(proxy_color)
                proxy_color[3] = min(float(proxy_color[3]), 0.35)
            shape = str(proxy.get("shape", "box")).lower()
            if shape == "box":
                markers.append(make_marker(frame_id, marker_id, proxy["name"], Marker.CUBE,
                                           proxy["center"], proxy["size"], color(*proxy_color),
                                           yaw=float(proxy.get("yaw", 0.0)),
                                           namespace=NS_COLLISION_PROXY_PRIMITIVES))
                marker_id += 1
            elif shape == "cylinder":
                radius = float(proxy["radius"])
                markers.append(make_marker(frame_id, marker_id, proxy["name"], Marker.CYLINDER,
                                           proxy["center"], [2.0 * radius, 2.0 * radius, proxy["height"]],
                                           color(*proxy_color),
                                           namespace=NS_COLLISION_PROXY_PRIMITIVES))
                marker_id += 1

    for item in scene.get("visual_vessels", []) or []:
        for mesh in static_mesh_specs(item):
            mesh_marker = make_static_mesh_marker(frame_id, marker_id, item, mesh)
            if mesh_marker is not None:
                markers.append(mesh_marker)
                marker_id += 1
            if show_collision_meshes and bool(item.get("mesh_collision", False)):
                collision_marker = make_mesh_resource_marker(
                    frame_id, marker_id, item, mesh, NS_PERCEPTION_COLLISION_MESH,
                    color_override=[1.0, 0.34, 0.04, 0.32],
                    embedded_materials=False,
                    text_suffix="mesh_collision_shell")
                if collision_marker is not None:
                    markers.append(collision_marker)
                    marker_id += 1

    if include_dynamic:
        if show_collision_meshes:
            for obstacle in scene_dynamic_obstacles(scene):
                for proxy in dynamic_collision_proxy_instances(obstacle, dynamic_time_sec):
                    proxy_color = proxy.get("color", obstacle.get("color", [0.10, 0.68, 0.92, 0.32]))
                    if len(proxy_color) >= 4:
                        proxy_color = list(proxy_color)
                        proxy_color[3] = min(float(proxy_color[3]), 0.38)
                    shape = str(proxy.get("shape", "box")).lower()
                    if shape == "box":
                        markers.append(make_marker(
                            frame_id, marker_id, proxy["name"], Marker.CUBE,
                            proxy["center"], proxy["size"], color(*proxy_color),
                            yaw=float(proxy.get("yaw", 0.0)),
                            namespace=NS_DYNAMIC_PERCEPTION_PROXY_PRIMITIVES))
                        marker_id += 1
                    elif shape == "cylinder":
                        radius = float(proxy["radius"])
                        markers.append(make_marker(
                            frame_id, marker_id, proxy["name"], Marker.CYLINDER,
                            proxy["center"], [2.0 * radius, 2.0 * radius, proxy["height"]],
                            color(*proxy_color),
                            namespace=NS_DYNAMIC_PERCEPTION_PROXY_PRIMITIVES))
                        marker_id += 1

        for obstacle in scene_dynamic_obstacles(scene):
            mesh_marker = make_mesh_marker(frame_id, marker_id, obstacle, dynamic_time_sec)
            if mesh_marker is not None:
                markers.append(mesh_marker)
                marker_id += 1

    path_marker = Marker()
    path_marker.header.frame_id = frame_id
    path_marker.header.stamp = rospy.Time.now()
    path_marker.ns = NS_ROUTE_MARKERS
    path_marker.id = marker_id
    path_marker.type = Marker.LINE_STRIP
    path_marker.action = Marker.ADD
    path_marker.pose.orientation.w = 1.0
    path_marker.scale.x = 0.08
    path_marker.color = color(0.1, 0.85, 1.0, 0.95)
    waypoints = scene_waypoints(scene)
    path_marker.points = []
    for waypoint in waypoints:
        x, y, z = waypoint["position"]
        path_marker.points.append(Point(x=x, y=y, z=z))
    markers.append(path_marker)
    marker_id += 1

    for waypoint in waypoints:
        markers.append(make_marker(frame_id, marker_id, waypoint["name"], Marker.SPHERE,
                                   waypoint["position"], [0.35, 0.35, 0.35],
                                   color(0.05, 0.95, 0.9, 0.95),
                                   namespace=NS_ROUTE_MARKERS))
        marker_id += 1

    return MarkerArray(markers=markers)


def build_path(scene, frame_id):
    path = Path()
    path.header.frame_id = frame_id
    path.header.stamp = rospy.Time.now()
    for waypoint in scene_waypoints(scene):
        pose = Pose()
        pose.position = Point(
            x=waypoint["position"][0],
            y=waypoint["position"][1],
            z=waypoint["position"][2],
        )
        pose.orientation = quat_from_yaw(waypoint["yaw"])
        stamped = PoseStamped()
        stamped.header = path.header
        stamped.pose = pose
        path.poses.append(stamped)
    return path


def main():
    rospy.init_node("maritime_scene_markers")
    scene = load_scene(rospy.get_param("~scene_config", None))
    frame_id = rospy.get_param("~frame_id", scene.get("frame_id", "world"))
    marker_topic = rospy.get_param("~marker_topic", "/maritime/scene_markers")
    path_topic = rospy.get_param("~path_topic", "/maritime/mission_path")
    dynamic_mode = rospy.get_param("~dynamic_mode", "auto")
    include_dynamic = dynamic_mode_enabled(scene, dynamic_mode)
    show_reference_primitives = bool_param(rospy.get_param("~show_reference_primitives", False))
    show_raw_planner_shapes = bool_param(rospy.get_param("~show_raw_planner_shapes", False))
    show_collision_meshes = bool_param(rospy.get_param("~show_collision_meshes", True))
    rate_hz = max(1.0, min(5.0, float(rospy.get_param("~rate_hz", scene.get("cloud_publish_rate_hz", 3.0)))))

    marker_pub = rospy.Publisher(marker_topic, MarkerArray, queue_size=1, latch=True)
    path_pub = rospy.Publisher(path_topic, Path, queue_size=1, latch=True)
    rate = rospy.Rate(rate_hz)
    rospy.loginfo(
        "publishing maritime scene markers on %s and %s reference_primitives=%s raw_planner_shapes=%s collision_meshes=%s",
        marker_topic, path_topic, show_reference_primitives, show_raw_planner_shapes, show_collision_meshes)
    try:
        while not rospy.is_shutdown():
            marker_pub.publish(build_marker_array(
                scene, frame_id, rospy.Time.now().to_sec(), include_dynamic=include_dynamic,
                show_reference_primitives=show_reference_primitives,
                show_raw_planner_shapes=show_raw_planner_shapes,
                show_collision_meshes=show_collision_meshes))
            path_pub.publish(build_path(scene, frame_id))
            rate.sleep()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
