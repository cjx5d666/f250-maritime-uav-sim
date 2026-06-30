#!/usr/bin/env python3
import argparse
import math
import os
import sys
import xml.sax.saxutils as xml_escape

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from maritime_scene_utils import dynamic_obstacle_center, dynamic_obstacle_yaw
from maritime_scene_utils import load_scene, scene_box, scene_dynamic_obstacles, validate_scene


def fmt(values):
    return " ".join("%.6g" % float(value) for value in values)


def color_value(item, fallback):
    return fmt(item.get("color", fallback))


def xml_name(value):
    return xml_escape.escape(str(value), {'"': "&quot;"})


def box_model(name, center, size, color, visual=True, yaw=0.0, collision=True):
    name = xml_name(name)
    pose = [float(center[0]), float(center[1]), float(center[2]), 0.0, 0.0, float(yaw or 0.0)]
    visual_text = ""
    if bool(visual):
        visual_text = '        <visual name="visual"><geometry><box><size>{size}</size></box></geometry><material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>\n'.format(
            size=fmt(size), color=fmt(color)
        )
    collision_text = ""
    if bool(collision):
        collision_text = '        <collision name="collision"><geometry><box><size>{size}</size></box></geometry></collision>\n'.format(
            size=fmt(size)
        )
    return """    <model name="{name}">
      <static>true</static>
      <pose>{pose}</pose>
      <link name="link">
{collision}{visual}      </link>
    </model>
""".format(name=name, pose=fmt(pose), collision=collision_text, visual=visual_text)


def visual_box_model(name, center, size, color, yaw=0.0):
    name = xml_name(name)
    pose = [float(center[0]), float(center[1]), float(center[2]), 0.0, 0.0, float(yaw or 0.0)]
    return """    <model name="{name}">
      <static>true</static>
      <pose>{pose}</pose>
      <link name="link">
        <visual name="visual"><geometry><box><size>{size}</size></box></geometry><material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>
      </link>
    </model>
""".format(name=name, pose=fmt(pose), size=fmt(size), color=fmt(color))


def cylinder_model(name, center, radius, height, color, visual=True):
    name = xml_name(name)
    radius_text = "%.6g" % float(radius)
    height_text = "%.6g" % float(height)
    visual_text = ""
    if bool(visual):
        visual_text = '        <visual name="visual"><geometry><cylinder><radius>{radius}</radius><length>{height}</length></cylinder></geometry><material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>\n'.format(
            radius=radius_text, height=height_text, color=fmt(color)
        )
    return """    <model name="{name}">
      <static>true</static>
      <pose>{pose} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><cylinder><radius>{radius}</radius><length>{height}</length></cylinder></geometry></collision>
{visual}      </link>
    </model>
""".format(name=name, pose=fmt(center), radius=radius_text, height=height_text, visual=visual_text)



def navigation_buoy_model(item):
    name = xml_name(item.get("name", "navigation_buoy"))
    center = item["center"]
    radius = float(item["radius"])
    height = float(item["height"])
    collision_height = max(height, float(item.get("lidar_collision_height", height)))
    collision_z_offset = float(item.get("lidar_collision_z_offset", 0.5 * (collision_height - height)))
    radius_text = "%.6g" % radius
    collision_height_text = "%.6g" % collision_height
    collision_z_offset_text = "%.6g" % collision_z_offset
    body_radius = "%.6g" % (radius * 0.70)
    cap_radius = "%.6g" % (radius * 0.58)
    band_radius = "%.6g" % (radius * 0.73)
    body_height = "%.6g" % (height * 0.72)
    band_height = "%.6g" % max(0.22, height * 0.055)
    color = color_value(item, [1.0, 0.5, 0.12, 1.0])
    stripe = color_value({"color": item.get("stripe_color", [0.96, 0.96, 0.90, 1.0])}, [0.96, 0.96, 0.90, 1.0])
    cap_z = "%.6g" % (height * 0.43)
    high_band_z = "%.6g" % (height * 0.20)
    low_band_z = "%.6g" % (-height * 0.18)
    return """    <model name="{name}">
      <static>true</static>
      <pose>{pose} 0 0 0</pose>
      <link name="link">
        <collision name="lidar_collision_shell"><pose>0 0 {collision_z_offset} 0 0 0</pose><geometry><cylinder><radius>{radius}</radius><length>{collision_height}</length></cylinder></geometry></collision>
        <visual name="body"><geometry><cylinder><radius>{body_radius}</radius><length>{body_height}</length></cylinder></geometry><material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>
        <visual name="upper_band"><pose>0 0 {high_band_z} 0 0 0</pose><geometry><cylinder><radius>{band_radius}</radius><length>{band_height}</length></cylinder></geometry><material><ambient>{stripe}</ambient><diffuse>{stripe}</diffuse></material></visual>
        <visual name="lower_band"><pose>0 0 {low_band_z} 0 0 0</pose><geometry><cylinder><radius>{band_radius}</radius><length>{band_height}</length></cylinder></geometry><material><ambient>{stripe}</ambient><diffuse>{stripe}</diffuse></material></visual>
        <visual name="top_cap"><pose>0 0 {cap_z} 0 0 0</pose><geometry><sphere><radius>{cap_radius}</radius></sphere></geometry><material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>
      </link>
    </model>
""".format(
        name=name,
        pose=fmt(center),
        radius=radius_text,
        collision_height=collision_height_text,
        collision_z_offset=collision_z_offset_text,
        body_radius=body_radius,
        body_height=body_height,
        band_radius=band_radius,
        band_height=band_height,
        color=color,
        stripe=stripe,
        cap_radius=cap_radius,
        cap_z=cap_z,
        high_band_z=high_band_z,
        low_band_z=low_band_z,
    )

def sdf_bool(value):
    return "true" if bool(value) else "false"


def scale3(item, fallback=1.0):
    raw = item.get("scale", fallback)
    if isinstance(raw, (int, float)):
        return [float(raw), float(raw), float(raw)]
    values = [float(value) for value in raw]
    if len(values) == 1:
        return [values[0], values[0], values[0]]
    if len(values) != 3:
        raise RuntimeError("scale must be a scalar or a 3-vector")
    return values


def scaled(values, scale):
    return [float(values[index]) * float(scale[index]) for index in range(3)]


def pose6(values, key):
    data = [float(value) for value in values]
    if len(data) == 3:
        data.extend([0.0, 0.0, 0.0])
    if len(data) != 6:
        raise RuntimeError("%s must be a 3-vector or 6-vector" % key)
    return data


def material_block(color):
    if color is None:
        return ""
    color_text = fmt(color)
    return "\n          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>".format(color=color_text)


def mesh_visuals(item, default_name, default_mesh_uri=None):
    raw_meshes = item.get("visual_meshes")
    if not raw_meshes:
        mesh_uri = item.get("mesh_uri", default_mesh_uri)
        if not mesh_uri:
            return ""
        raw_meshes = [{
            "name": default_name,
            "uri": mesh_uri,
            "scale": item.get("mesh_scale", item.get("scale", 1.0)),
            "pose": item.get("mesh_pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            "material": item.get("mesh_material"),
        }]

    visuals = []
    for index, mesh in enumerate(raw_meshes):
        mesh_uri = mesh.get("uri", mesh.get("mesh_uri", default_mesh_uri))
        if not mesh_uri:
            continue
        mesh_name = xml_name(mesh.get("name", "%s_visual_%d" % (default_name, index)))
        mesh_scale = fmt(scale3(mesh, 1.0))
        mesh_pose = fmt(pose6(mesh.get("pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), "visual_meshes[%d].pose" % index))
        visuals.append("""        <visual name="{mesh_name}">
          <pose>{mesh_pose}</pose>
          <geometry><mesh><uri>{mesh_uri}</uri><scale>{mesh_scale}</scale></mesh></geometry>{material}
        </visual>
""".format(
            mesh_name=mesh_name,
            mesh_pose=mesh_pose,
            mesh_uri=xml_escape.escape(str(mesh_uri)),
            mesh_scale=mesh_scale,
            material=material_block(mesh.get("material")),
        ))
    return "".join(visuals)


def bridge_box_element(name, pose, size, color, collision=True):
    name = xml_name(name)
    pose_text = fmt(pose)
    size_text = fmt(size)
    color_text = fmt(color)
    collision_text = ""
    if collision:
        collision_text = """        <collision name="{name}_collision">
          <pose>{pose}</pose>
          <geometry><box><size>{size}</size></box></geometry>
        </collision>
""".format(name=name, pose=pose_text, size=size_text)
    return """        <visual name="{name}_visual">
          <pose>{pose}</pose>
          <geometry><box><size>{size}</size></box></geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
        </visual>
{collision}""".format(name=name, pose=pose_text, size=size_text, color=color_text, collision=collision_text)


def bridge_diagonal_element(name, x0, z0, x1, z1, y, thickness, color):
    dx = float(x1) - float(x0)
    dz = float(z1) - float(z0)
    length = math.hypot(dx, dz)
    if length <= 1.0e-6:
        return ""
    pose = [(x0 + x1) * 0.5, y, (z0 + z1) * 0.5, 0.0, -math.atan2(dz, dx), 0.0]
    return bridge_box_element(name, pose, [length, thickness, thickness], color, collision=False)


def proxy_pose6(proxy, index):
    if "pose" in proxy:
        return pose6(proxy.get("pose"), "collision_proxies[%d].pose" % index)
    center = [float(value) for value in proxy.get("center", [0.0, 0.0, 0.0])]
    if len(center) != 3:
        raise RuntimeError("collision_proxies[%d].center must be a 3-vector" % index)
    return center + [0.0, 0.0, float(proxy.get("yaw", 0.0))]


def collision_proxy_geometry(proxy, index):
    shape = str(proxy.get("shape", "box")).lower()
    if shape == "box":
        size = proxy.get("size")
        if not isinstance(size, (list, tuple)) or len(size) != 3:
            raise RuntimeError("collision_proxies[%d].size must be a 3-vector" % index)
        return "<box><size>{size}</size></box>".format(size=fmt(size))
    if shape == "cylinder":
        return "<cylinder><radius>{radius:.6g}</radius><length>{height:.6g}</length></cylinder>".format(
            radius=float(proxy.get("radius", 0.0)),
            height=float(proxy.get("height", proxy.get("length", 0.0))),
        )
    raise RuntimeError("unsupported collision proxy shape: %s" % shape)


def collision_proxy_elements(item):
    elements = []
    for index, proxy in enumerate(item.get("collision_proxies", []) or []):
        name = xml_name(proxy.get("name", "collision_proxy_%d" % index))
        pose = fmt(proxy_pose6(proxy, index))
        geometry = collision_proxy_geometry(proxy, index)
        elements.append("""        <collision name="{name}">
          <pose>{pose}</pose>
          <geometry>{geometry}</geometry>
        </collision>
""".format(name=name, pose=pose, geometry=geometry))
    return "".join(elements)


def mesh_collision_enabled(item):
    return bool(item.get("mesh_collision", False))


def mesh_collision_elements(item):
    if not mesh_collision_enabled(item):
        return ""

    raw_meshes = item.get("visual_meshes")
    if not raw_meshes:
        mesh_uri = item.get("mesh_uri")
        if not mesh_uri:
            return ""
        raw_meshes = [{
            "name": item.get("name", "visual_mesh"),
            "uri": mesh_uri,
            "scale": item.get("mesh_scale", item.get("scale", 1.0)),
            "pose": item.get("mesh_pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        }]

    elements = []
    for index, mesh in enumerate(raw_meshes):
        mesh_uri = mesh.get("uri", mesh.get("mesh_uri", item.get("mesh_uri")))
        if not mesh_uri:
            continue
        name = xml_name("%s_mesh_collision" % mesh.get("name", "mesh_%d" % index))
        mesh_pose = fmt(pose6(mesh.get("pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), "visual_meshes[%d].pose" % index))
        mesh_scale = fmt(scale3(mesh, 1.0))
        elements.append("""        <collision name="{name}">
          <pose>{mesh_pose}</pose>
          <geometry><mesh><uri>{mesh_uri}</uri><scale>{mesh_scale}</scale></mesh></geometry>
        </collision>
""".format(
            name=name,
            mesh_pose=mesh_pose,
            mesh_uri=xml_escape.escape(str(mesh_uri)),
            mesh_scale=mesh_scale,
        ))
    return "".join(elements)


def collision_proxies_enabled(item):
    return bool(item.get("use_collision_proxies", not mesh_collision_enabled(item)))


def scalable_bridge_model(item):
    name = xml_name(item.get("name", "scalable_bridge"))
    center = [float(value) for value in item.get("center", [0.0, 0.0, 0.0])]
    if len(center) != 3:
        raise RuntimeError("bridge center must be a 3-vector")
    scale = float(item.get("scale", 1.0))
    yaw = float(item.get("yaw", 0.0))
    length = float(item.get("length", 12.0)) * scale
    width = float(item.get("width", 6.0)) * scale
    clearance = float(item.get("clearance", 4.0)) * scale
    deck_thickness = float(item.get("deck_thickness", 0.32)) * scale
    tower_height = float(item.get("tower_height", 2.8)) * scale
    tower_width = float(item.get("tower_width", 0.22)) * scale
    truss_thickness = float(item.get("truss_thickness", 0.10)) * scale
    rail_height = float(item.get("rail_height", 0.65)) * scale
    truss_segments = max(2, int(item.get("truss_segments", 4)))
    concrete = item.get("concrete_color", [0.62, 0.64, 0.61, 1.0])
    steel = item.get("steel_color", [0.18, 0.28, 0.34, 1.0])
    road = item.get("road_color", [0.28, 0.29, 0.30, 1.0])

    deck_z = clearance + deck_thickness * 0.5
    top_z = clearance + tower_height
    pylon_height = clearance + tower_height
    tower_x = [-length * 0.34, length * 0.34]
    side_y = width * 0.5
    truss_y = side_y + 0.16 * scale
    elements = []

    elements.append(bridge_box_element("deck", [0.0, 0.0, deck_z, 0.0, 0.0, 0.0],
                                       [length, width, deck_thickness], road, collision=True))
    elements.append(bridge_box_element("center_lane", [0.0, 0.0, deck_z + deck_thickness * 0.52, 0.0, 0.0, 0.0],
                                       [length * 0.94, 0.06 * scale, 0.025 * scale],
                                       [0.85, 0.82, 0.60, 1.0], collision=False))

    for side in (-1.0, 1.0):
        y = side * side_y
        elements.append(bridge_box_element("side_rail_%s" % ("port" if side > 0 else "starboard"),
                                           [0.0, y, deck_z + rail_height * 0.5, 0.0, 0.0, 0.0],
                                           [length, 0.12 * scale, rail_height], steel, collision=True))
        elements.append(bridge_box_element("upper_truss_%s" % ("port" if side > 0 else "starboard"),
                                           [0.0, side * truss_y, top_z, 0.0, 0.0, 0.0],
                                           [length * 0.92, truss_thickness, truss_thickness], steel, collision=False))

    for x in tower_x:
        elements.append(bridge_box_element("crossbeam_%+.1f" % x, [x, 0.0, top_z, 0.0, 0.0, 0.0],
                                           [0.28 * scale, width + 0.7 * scale, 0.18 * scale], steel, collision=False))
        for y in (-side_y, side_y):
            elements.append(bridge_box_element("pylon_%+.1f_%+.1f" % (x, y),
                                               [x, y, pylon_height * 0.5, 0.0, 0.0, 0.0],
                                               [tower_width, tower_width, pylon_height], concrete, collision=True))

    span = length / float(truss_segments)
    z_low = deck_z + deck_thickness * 0.8
    z_high = top_z - 0.18 * scale
    for side in (-1.0, 1.0):
        y = side * truss_y
        side_name = "port" if side > 0 else "starboard"
        for index in range(truss_segments):
            x0 = -length * 0.5 + index * span
            x1 = x0 + span
            if index % 2:
                elements.append(bridge_diagonal_element("truss_%s_%02d" % (side_name, index), x0, z_high, x1, z_low,
                                                        y, truss_thickness, steel))
            else:
                elements.append(bridge_diagonal_element("truss_%s_%02d" % (side_name, index), x0, z_low, x1, z_high,
                                                        y, truss_thickness, steel))

    return """    <model name="{name}">
      <static>true</static>
      <pose>{pose} 0 0 {yaw:.6g}</pose>
      <link name="bridge_link">
{elements}      </link>
    </model>
""".format(name=name, pose=fmt(center), yaw=yaw, elements="".join(elements))


def mesh_vessel_model(item):
    name = xml_name(item.get("name", "mesh_vessel"))
    center = [float(value) for value in item.get("center", [0.0, 0.0, 0.0])]
    if len(center) != 3:
        raise RuntimeError("vessel center must be a 3-vector")
    rpy = [float(value) for value in item.get("rpy", [0.0, 0.0, float(item.get("yaw", 0.0))])]
    if len(rpy) != 3:
        raise RuntimeError("vessel rpy must be a 3-vector")
    scale = scale3(item, 1.0)
    mesh_uri = item.get("mesh_uri", "model://f250_wamv_visual/mesh/WAM-V-Base.dae")
    static = sdf_bool(item.get("static", True))
    collision_enabled = bool(item.get("collision", True))
    collision_size = scaled(item.get("collision_size", [5.0, 2.5, 0.7]), scale)
    collision_center = scaled(item.get("collision_center", [0.0, 0.0, 0.35]), scale)
    deck_size = scaled(item.get("deck_collision_size", [1.8, 1.1, 0.18]), scale)
    deck_center = scaled(item.get("deck_collision_center", [0.0, 0.0, 1.05]), scale)
    visuals = mesh_visuals(item, "vessel_visual", default_mesh_uri=mesh_uri)
    collisions = mesh_collision_elements(item)
    if collision_proxies_enabled(item):
        collisions += collision_proxy_elements(item)
    if collision_enabled:
        collisions = """        <collision name="hull_collision">
          <pose>{collision_center} 0 0 0</pose>
          <geometry><box><size>{collision_size}</size></box></geometry>
        </collision>
        <collision name="deck_collision">
          <pose>{deck_center} 0 0 0</pose>
          <geometry><box><size>{deck_size}</size></box></geometry>
        </collision>
""".format(
            collision_center=fmt(collision_center),
            collision_size=fmt(collision_size),
            deck_center=fmt(deck_center),
            deck_size=fmt(deck_size),
        ) + collisions
    return """    <model name="{name}">
      <static>{static}</static>
      <pose>{pose}</pose>
      <link name="vessel_link">
{visuals}{collisions}      </link>
    </model>
""".format(
        name=name,
        static=static,
        pose=fmt(center + rpy),
        visuals=visuals,
        collisions=collisions,
    )


def dynamic_box_model(item):
    name = xml_name(item.get("name", "dynamic_obstacle"))
    center = dynamic_obstacle_center(item, 0.0)
    yaw = dynamic_obstacle_yaw(item, 0.0)
    size = fmt(item["size"])
    color = color_value(item, [0.95, 0.32, 0.12, 1.0])
    visuals = mesh_visuals(item, "dynamic_obstacle_visual")
    if not visuals:
        visuals = """        <visual name="visual"><geometry><box><size>{size}</size></box></geometry><material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>
""".format(size=size, color=color)
    collisions = collision_proxy_elements(item) if item.get("collision_proxies") else ""
    if not collisions:
        collisions = '        <collision name="collision"><geometry><box><size>{size}</size></box></geometry></collision>\n'.format(
            size=size)
    return """    <model name="{name}">
      <static>false</static>
      <pose>{pose} 0 0 {yaw:.6g}</pose>
      <link name="obstacle_link">
        <gravity>false</gravity>
        <self_collide>false</self_collide>
        <inertial><mass>20.0</mass><inertia><ixx>1.0</ixx><ixy>0.0</ixy><ixz>0.0</ixz><iyy>1.0</iyy><iyz>0.0</iyz><izz>1.0</izz></inertia></inertial>
{collisions}{visuals}      </link>
    </model>
""".format(name=name, pose=fmt(center), yaw=yaw, collisions=collisions, visuals=visuals)


def dynamic_cylinder_model(item):
    name = xml_name(item.get("name", "dynamic_obstacle"))
    center = dynamic_obstacle_center(item, 0.0)
    yaw = dynamic_obstacle_yaw(item, 0.0)
    color = color_value(item, [0.95, 0.32, 0.12, 1.0])
    radius = "%.6g" % float(item["radius"])
    height = "%.6g" % float(item["height"])
    return """    <model name="{name}">
      <static>false</static>
      <pose>{pose} 0 0 {yaw:.6g}</pose>
      <link name="obstacle_link">
        <gravity>false</gravity>
        <self_collide>false</self_collide>
        <inertial><mass>10.0</mass><inertia><ixx>0.5</ixx><ixy>0.0</ixy><ixz>0.0</ixz><iyy>0.5</iyy><iyz>0.0</iyz><izz>0.5</izz></inertia></inertial>
        <collision name="collision"><geometry><cylinder><radius>{radius}</radius><length>{height}</length></cylinder></geometry></collision>
        <visual name="visual"><geometry><cylinder><radius>{radius}</radius><length>{height}</length></cylinder></geometry><material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>
      </link>
    </model>
""".format(name=name, pose=fmt(center), yaw=yaw, radius=radius, height=height, color=color)


def dynamic_composite_model(item):
    raw_name = item.get("name", "dynamic_obstacle")
    name = xml_name(raw_name)
    center = dynamic_obstacle_center(item, 0.0)
    yaw = dynamic_obstacle_yaw(item, 0.0)
    collisions = collision_proxy_elements(item)
    if not collisions:
        raise RuntimeError("composite dynamic obstacle %s requires collision_proxies" % raw_name)
    visuals = mesh_visuals(item, "dynamic_obstacle_visual")
    if not visuals:
        raise RuntimeError("composite dynamic obstacle %s requires mesh_uri or visual_meshes" % raw_name)
    return """    <model name="{name}">
      <static>false</static>
      <pose>{pose} 0 0 {yaw:.6g}</pose>
      <link name="obstacle_link">
        <gravity>false</gravity>
        <self_collide>false</self_collide>
        <inertial><mass>20.0</mass><inertia><ixx>1.0</ixx><ixy>0.0</ixy><ixz>0.0</ixz><iyy>1.0</iyy><iyz>0.0</iyz><izz>1.0</izz></inertia></inertial>
{collisions}{visuals}      </link>
    </model>
""".format(name=name, pose=fmt(center), yaw=yaw, collisions=collisions, visuals=visuals)


def dynamic_model(item):
    shape = str(item.get("shape", "box")).lower()
    if shape == "box":
        return dynamic_box_model(item)
    if shape == "cylinder":
        return dynamic_cylinder_model(item)
    if shape == "composite":
        return dynamic_composite_model(item)
    raise RuntimeError("unsupported dynamic obstacle shape: %s" % shape)


def vector3_value(values, fallback):
    data = values if values is not None else fallback
    return fmt(data[:3])


def wind_plugin(scene):
    wind = scene.get("wind") or {}
    if not wind or not bool(wind.get("enabled", False)):
        return ""
    return """    <plugin name='wind_plugin' filename='libgazebo_wind_plugin.so'>
      <frameId>{frame_id}</frameId>
      <robotNamespace/>
      <windVelocityMean>{velocity_mean:.6g}</windVelocityMean>
      <windVelocityMax>{velocity_max:.6g}</windVelocityMax>
      <windVelocityVariance>{velocity_variance:.6g}</windVelocityVariance>
      <windDirectionMean>{direction_mean}</windDirectionMean>
      <windDirectionVariance>{direction_variance:.6g}</windDirectionVariance>
      <windGustStart>{gust_start:.6g}</windGustStart>
      <windGustDuration>{gust_duration:.6g}</windGustDuration>
      <windGustVelocityMean>{gust_velocity_mean:.6g}</windGustVelocityMean>
      <windGustVelocityMax>{gust_velocity_max:.6g}</windGustVelocityMax>
      <windGustVelocityVariance>{gust_velocity_variance:.6g}</windGustVelocityVariance>
      <windGustDirectionMean>{gust_direction_mean}</windGustDirectionMean>
      <windGustDirectionVariance>{gust_direction_variance:.6g}</windGustDirectionVariance>
      <windPubTopic>{wind_topic}</windPubTopic>
    </plugin>
""".format(
        frame_id=xml_name(wind.get("frame_id", "base_link")),
        velocity_mean=float(wind.get("velocity_mean", 0.0)),
        velocity_max=float(wind.get("velocity_max", max(float(wind.get("velocity_mean", 0.0)), 0.0))),
        velocity_variance=float(wind.get("velocity_variance", 0.0)),
        direction_mean=vector3_value(wind.get("direction_mean"), [1.0, 0.0, 0.0]),
        direction_variance=float(wind.get("direction_variance", 0.0)),
        gust_start=float(wind.get("gust_start", 0.0)),
        gust_duration=float(wind.get("gust_duration", 0.0)),
        gust_velocity_mean=float(wind.get("gust_velocity_mean", 0.0)),
        gust_velocity_max=float(wind.get("gust_velocity_max", 0.0)),
        gust_velocity_variance=float(wind.get("gust_velocity_variance", 0.0)),
        gust_direction_mean=vector3_value(wind.get("gust_direction_mean"), [1.0, 0.0, 0.0]),
        gust_direction_variance=float(wind.get("gust_direction_variance", 0.0)),
        wind_topic=xml_name(wind.get("topic", "world_wind")),
    )


def zone_models(scene):
    models = []
    seen = set()

    def append_zone(zone, source, color):
        key = (
            zone["name"],
            tuple(round(float(value), 6) for value in zone["center"]),
            tuple(round(float(value), 6) for value in zone["size"]),
            round(float(zone["yaw"]), 6),
        )
        if key in seen:
            return
        seen.add(key)
        models.append(box_model(zone["name"], zone["center"], zone["size"], color,
                                visual=source.get("visual", True), yaw=zone["yaw"]))

    deck = scene_box(scene, "deck", required=True)
    deck_source = scene.get("deck") or {}
    append_zone(deck, deck_source, [0.38, 0.38, 0.34, 1.0])
    takeoff = scene_box(scene, "takeoff_deck_zone", required=True)
    takeoff_source = scene.get("takeoff_deck_zone") or {}
    append_zone(takeoff, takeoff_source, [0.12, 0.28, 0.62, 1.0])
    landing = scene_box(scene, "landing_deck_zone", required=True)
    landing_source = scene.get("landing_deck_zone") or scene.get("landing_box") or {}
    append_zone(landing, landing_source, [0.15, 0.85, 0.25, 1.0])
    return models


def generate_world(scene):
    errors = validate_scene(scene)
    if errors:
        raise RuntimeError("invalid scene: %s" % "; ".join(errors))

    world_name = xml_name(scene.get("world_name", "f250_maritime_%s" % scene.get("scene_level", "scene")))
    scene_path = scene.get("_scene_path", "")
    if scene_path:
        scripts_dir = os.path.dirname(os.path.abspath(__file__))
        pkg_dir = os.path.dirname(scripts_dir)
        scene_path = os.path.relpath(scene_path, pkg_dir)
    gui_camera = scene.get("gui_camera") or {}
    gui_camera_pose = fmt(pose6(gui_camera.get("pose", [12.0, -13.0, 7.0, 0.0, 0.5, 2.35619]),
                               "gui_camera.pose"))
    parts = ["""<?xml version="1.0"?>
<!-- Generated by generate_maritime_world.py from {scene_path}. Do not hand-edit. -->
<sdf version="1.6">
  <world name="{world_name}">
    <scene>
      <ambient>0.45 0.48 0.52 1.0</ambient>
      <sky><clouds><speed>0</speed></clouds></sky>
      <shadows>true</shadows>
    </scene>
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>{gui_camera_pose}</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>
    <include><uri>model://sun</uri></include>
    <include><uri>model://ocean</uri><pose>0 0 0 0 0 0</pose><static>true</static></include>
    <physics name="default_physics" default="0" type="ode">
      <gravity>0 0 -9.8066</gravity>
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
""".format(
        scene_path=xml_escape.escape(scene_path),
        world_name=world_name,
        gui_camera_pose=gui_camera_pose,
    )]

    for item in scene.get("ship_hull", []) or []:
        parts.append(box_model(item.get("name", "ship_hull"), item["center"], item["size"],
                               item.get("color", [0.17, 0.29, 0.36, 1.0])))
    parts.extend(zone_models(scene))
    for item in scene.get("docks", []) or []:
        parts.append(box_model(item.get("name", "dock"), item["center"], item["size"],
                               item.get("color", [0.34, 0.28, 0.20, 1.0]),
                               visual=item.get("visual", True),
                               yaw=float(item.get("yaw", 0.0))))
    for item in scene.get("visual_vessels", []) or []:
        parts.append(mesh_vessel_model(item))
    for item in scene.get("bridges", []) or []:
        parts.append(scalable_bridge_model(item))
    for item in scene.get("bridge_piers", []) or []:
        parts.append(cylinder_model(item.get("name", "bridge_pier"), item["center"], item["radius"], item["height"],
                                    item.get("color", [0.72, 0.72, 0.68, 1.0]),
                                    visual=item.get("visual", True)))
    for item in scene.get("buoys", []) or []:
        if item.get("visual_style") in ("nav_marker", "navigation_buoy") and item.get("visual", True):
            parts.append(navigation_buoy_model(item))
        else:
            parts.append(cylinder_model(item.get("name", "buoy"), item["center"], item["radius"], item["height"],
                                        item.get("color", [1.0, 0.18, 0.08, 1.0]),
                                        visual=item.get("visual", True)))
    for item in scene.get("box_obstacles", []) or []:
        default_color = [0.9, 0.52, 0.14, 1.0] if "container" in item.get("name", "") else [0.82, 0.82, 0.75, 1.0]
        if item.get("visual", True) or item.get("collision", True):
            parts.append(box_model(item.get("name", "box_obstacle"), item["center"], item["size"],
                                   item.get("color", default_color),
                                   visual=item.get("visual", True),
                                   yaw=float(item.get("yaw", 0.0)),
                                   collision=item.get("collision", True)))
    for item in scene.get("visual_boxes", []) or []:
        if item.get("visual", True):
            parts.append(visual_box_model(item.get("name", "visual_box"), item["center"], item["size"],
                                          item.get("color", [0.72, 0.74, 0.74, 1.0]),
                                          yaw=float(item.get("yaw", 0.0))))

    for item in scene_dynamic_obstacles(scene):
        parts.append(dynamic_model(item))

    parts.append(wind_plugin(scene))
    parts.append("    <!-- Dynamic obstacle models are moved by maritime_dynamic_obstacles.py when DYNAMIC_MODE is enabled. -->\n")
    parts.append("  </world>\n</sdf>\n")
    return "".join(parts)


def parse_args():
    parser = argparse.ArgumentParser(description="Generate a Gazebo Classic world from a maritime scene YAML.")
    parser.add_argument("--scene-config", required=True)
    parser.add_argument("--output", required=True)
    return parser.parse_args()


def main():
    args = parse_args()
    scene = load_scene(args.scene_config)
    world = generate_world(scene)
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as handle:
        handle.write(world)
    print(args.output)


if __name__ == "__main__":
    main()
