#!/usr/bin/env python3
import math
import os
from collections import OrderedDict

import yaml


EARTH_RADIUS_M = 6378137.0
_OBJ_VERTEX_CACHE = {}


def package_root_from_file():
    return os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))


def default_scene_path():
    package_root = package_root_from_file()
    return os.path.join(package_root, "config", "scenes", "level_m_gps_assets_quick_complex.yaml")


def load_scene(path=None):
    scene_path = path or default_scene_path()
    try:
        with open(scene_path, "r", encoding="utf-8") as handle:
            scene = yaml.safe_load(handle)
    except Exception as exc:
        raise RuntimeError("failed to load scene from %s: %s" % (scene_path, exc))
    if not isinstance(scene, dict):
        raise RuntimeError("scene YAML must be a dict, got %s from %s" % (type(scene).__name__, scene_path))
    scene["_scene_path"] = os.path.abspath(scene_path)
    return scene


def scene_box(scene, key, required=False):
    item = scene.get(key)
    if not item:
        if required:
            raise ValueError("%s is required" % key)
        return None
    return {
        "name": item.get("name", key),
        "center": as_float_list(item.get("center"), 3, "%s.center" % key),
        "size": as_float_list(item.get("size"), 3, "%s.size" % key),
        "yaw": float(item.get("yaw", 0.0)),
        "include_in_cloud": bool(item.get("include_in_cloud", False)),
        "allow_outboard": bool(item.get("allow_outboard", False)),
    }


def as_float_list(value, length, key):
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError("%s must be a list of %d numbers" % (key, length))
    return [float(v) for v in value]


def as_pose6(value, key):
    if not isinstance(value, (list, tuple)):
        raise ValueError("%s must be a 3- or 6-number list" % key)
    data = [float(v) for v in value]
    if len(data) == 3:
        data.extend([0.0, 0.0, 0.0])
    if len(data) != 6:
        raise ValueError("%s must be a 3- or 6-number list" % key)
    return data


def scale3(value, fallback=1.0, key="scale"):
    raw = fallback if value is None else value
    if isinstance(raw, (int, float)):
        return [float(raw), float(raw), float(raw)]
    if not isinstance(raw, (list, tuple)):
        raise ValueError("%s must be a scalar or list" % key)
    data = [float(v) for v in raw]
    if len(data) == 1:
        return [data[0], data[0], data[0]]
    if len(data) != 3:
        raise ValueError("%s must be a scalar, 1-vector, or 3-vector" % key)
    return data


def rpy_matrix(rpy):
    roll, pitch, yaw = [float(v) for v in rpy]
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def mat_vec(matrix, vector):
    return [
        matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
        matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
        matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
    ]


def mat_mul(left, right):
    rows = []
    for row in range(3):
        rows.append(tuple(
            left[row][0] * right[0][col] +
            left[row][1] * right[1][col] +
            left[row][2] * right[2][col]
            for col in range(3)
        ))
    return tuple(rows)


def matrix_yaw(matrix):
    return math.atan2(matrix[1][0], matrix[0][0])


def model_uri_path(uri):
    text = str(uri)
    prefix = "model://"
    if not text.startswith(prefix):
        return text
    return os.path.join(package_root_from_file(), "models", text[len(prefix):].lstrip("/"))


def visual_mesh_specs(item):
    raw_meshes = item.get("visual_meshes")
    if not raw_meshes:
        mesh_uri = item.get("mesh_uri")
        if not mesh_uri:
            return []
        raw_meshes = [{
            "name": item.get("name", "visual_mesh"),
            "uri": mesh_uri,
            "scale": item.get("mesh_scale", item.get("scale", 1.0)),
            "pose": item.get("mesh_pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            "material": item.get("mesh_material"),
        }]

    specs = []
    for index, mesh in enumerate(raw_meshes):
        mesh_uri = mesh.get("uri", mesh.get("mesh_uri", item.get("mesh_uri")))
        if not mesh_uri:
            continue
        specs.append({
            "name": mesh.get("name", "%s_mesh_%d" % (item.get("name", "visual"), index)),
            "uri": mesh_uri,
            "scale": scale3(mesh.get("scale", item.get("mesh_scale", item.get("scale", 1.0))),
                            1.0, "visual_meshes[%d].scale" % index),
            "pose": as_pose6(mesh.get("pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                             "visual_meshes[%d].pose" % index),
            "material": mesh.get("material", item.get("mesh_material")),
        })
    return specs


def visual_mesh_collision_enabled(item):
    return bool(item.get("mesh_collision", False))


def visual_collision_proxies_enabled(item):
    return bool(item.get("use_collision_proxies", not visual_mesh_collision_enabled(item)))


def load_obj_vertices(uri):
    path = os.path.abspath(model_uri_path(uri))
    cached = _OBJ_VERTEX_CACHE.get(path)
    if cached is not None:
        return cached
    vertices = []
    with open(path, "r", encoding="utf-8", errors="ignore") as handle:
        for line in handle:
            if not line.startswith("v "):
                continue
            parts = line.split()
            if len(parts) < 4:
                continue
            try:
                vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
            except ValueError:
                continue
    _OBJ_VERTEX_CACHE[path] = vertices
    return vertices


def scene_geo_origin(scene):
    origin = scene.get("geo_origin")
    if not origin:
        return None

    if isinstance(origin, (list, tuple)):
        values = as_float_list(origin, 3, "geo_origin")
        return {
            "latitude_deg": values[0],
            "longitude_deg": values[1],
            "altitude_m": values[2],
            "local_position": [0.0, 0.0, 0.0],
        }

    if not isinstance(origin, dict):
        raise ValueError("geo_origin must be a mapping or [lat, lon, alt] list")

    latitude = origin.get("latitude_deg", origin.get("lat"))
    longitude = origin.get("longitude_deg", origin.get("lon"))
    altitude = origin.get("altitude_m", origin.get("alt", 0.0))
    if latitude is None or longitude is None:
        raise ValueError("geo_origin requires latitude_deg and longitude_deg")

    return {
        "latitude_deg": float(latitude),
        "longitude_deg": float(longitude),
        "altitude_m": float(altitude),
        "local_position": as_float_list(origin.get("local_position", [0.0, 0.0, 0.0]), 3,
                                        "geo_origin.local_position"),
    }


def gps_to_local_position(scene, gps_position):
    origin = scene_geo_origin(scene)
    if origin is None:
        raise ValueError("gps_position waypoints require scene geo_origin")

    lat, lon, alt = as_float_list(gps_position, 3, "waypoint.gps_position")
    origin_lat = math.radians(origin["latitude_deg"])
    d_lat = math.radians(lat - origin["latitude_deg"])
    d_lon = math.radians(lon - origin["longitude_deg"])
    east_m = d_lon * EARTH_RADIUS_M * math.cos(origin_lat)
    north_m = d_lat * EARTH_RADIUS_M
    up_m = alt - origin["altitude_m"]
    local = origin["local_position"]
    return [local[0] + east_m, local[1] + north_m, local[2] + up_m]


def positive(value, key):
    value = float(value)
    if value <= 0.0:
        raise ValueError("%s must be > 0" % key)
    return value


def frange(start, stop, step):
    start = float(start)
    stop = float(stop)
    step = positive(step, "step")
    if stop < start:
        start, stop = stop, start
    count = max(1, int(math.ceil((stop - start) / step)))
    values = []
    for index in range(count + 1):
        value = start + (stop - start) * index / count
        values.append(round(value, 4))
    return values


def quantize_point(point):
    return (round(float(point[0]), 4), round(float(point[1]), 4), round(float(point[2]), 4))


def sample_box(center, size, resolution, surface_only=True, yaw=0.0):
    cx, cy, cz = as_float_list(center, 3, "box.center")
    sx, sy, sz = as_float_list(size, 3, "box.size")
    if min(sx, sy, sz) <= 0.0:
        raise ValueError("box dimensions must be > 0")

    yaw = float(yaw or 0.0)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    xs = frange(-sx / 2.0, sx / 2.0, resolution)
    ys = frange(-sy / 2.0, sy / 2.0, resolution)
    zs = frange(cz - sz / 2.0, cz + sz / 2.0, resolution)
    points = OrderedDict()
    for x in xs:
        for y in ys:
            for z in zs:
                on_surface = (
                    x == xs[0] or x == xs[-1] or
                    y == ys[0] or y == ys[-1] or
                    z == zs[0] or z == zs[-1]
                )
                if on_surface or not surface_only:
                    world_x = cx + x * cos_yaw - y * sin_yaw
                    world_y = cy + x * sin_yaw + y * cos_yaw
                    points[quantize_point((world_x, world_y, z))] = None
    return list(points.keys())


def cylinder_z_range(cylinder):
    height = positive(cylinder.get("height", 0.0), "cylinder.height")
    if "z_min" in cylinder and "z_max" in cylinder:
        z_min = float(cylinder["z_min"])
        z_max = float(cylinder["z_max"])
        if z_max <= z_min:
            raise ValueError("cylinder.z_max must be greater than z_min")
        return z_min, z_max
    center = as_float_list(cylinder.get("center", [0, 0, 0]), 3, "cylinder.center")
    return center[2] - height / 2.0, center[2] + height / 2.0


def sample_cylinder(cylinder, resolution):
    center = as_float_list(cylinder.get("center", [0, 0, 0]), 3, "cylinder.center")
    radius = positive(cylinder.get("radius", 0.0), "cylinder.radius")
    z_min, z_max = cylinder_z_range(cylinder)
    z_values = frange(z_min, z_max, resolution)
    circumference_steps = max(16, int(math.ceil(2.0 * math.pi * radius / resolution)))
    radial_steps = max(1, int(math.ceil(radius / resolution)))
    points = OrderedDict()

    for z in z_values:
        for idx in range(circumference_steps):
            angle = 2.0 * math.pi * idx / circumference_steps
            x = center[0] + radius * math.cos(angle)
            y = center[1] + radius * math.sin(angle)
            points[quantize_point((x, y, z))] = None

    for z in (z_min, z_max):
        for ridx in range(radial_steps + 1):
            r = radius * ridx / radial_steps
            steps = max(8, int(math.ceil(2.0 * math.pi * max(r, resolution) / resolution)))
            for idx in range(steps):
                angle = 2.0 * math.pi * idx / steps
                x = center[0] + r * math.cos(angle)
                y = center[1] + r * math.sin(angle)
                points[quantize_point((x, y, z))] = None
    return list(points.keys())


def collision_proxy_local_pose(proxy, key):
    if "pose" in proxy:
        return as_pose6(proxy.get("pose"), "%s.pose" % key)
    center = as_float_list(proxy.get("center", [0.0, 0.0, 0.0]), 3, "%s.center" % key)
    return center + [0.0, 0.0, float(proxy.get("yaw", 0.0))]


def visual_item_yaw(item):
    rpy = item.get("rpy", [0.0, 0.0, float(item.get("yaw", 0.0))])
    return as_float_list(rpy, 3, "visual_vessels.rpy")[2]


def visual_collision_proxy_instances(scene, include_cloud_only=False):
    instances = []
    for item_index, item in enumerate(scene.get("visual_vessels", []) or []):
        if not visual_collision_proxies_enabled(item):
            continue
        proxies = item.get("collision_proxies", []) or []
        if not proxies:
            continue
        parent_center = as_float_list(
            item.get("center", [0.0, 0.0, 0.0]), 3,
            "visual_vessels[%d].center" % item_index)
        parent_yaw = visual_item_yaw(item)
        cos_yaw = math.cos(parent_yaw)
        sin_yaw = math.sin(parent_yaw)
        parent_name = item.get("name", "visual_vessel_%d" % item_index)
        for proxy_index, proxy in enumerate(proxies):
            if include_cloud_only and not proxy.get("include_in_cloud", True):
                continue
            local_pose = collision_proxy_local_pose(
                proxy, "visual_vessels[%d].collision_proxies[%d]" % (item_index, proxy_index))
            local_x, local_y, local_z = local_pose[:3]
            instance = dict(proxy)
            proxy_name = proxy.get("name", "collision_proxy_%d" % proxy_index)
            instance["name"] = "%s__%s" % (parent_name, proxy_name)
            instance["parent_name"] = parent_name
            instance["shape"] = str(proxy.get("shape", "box")).lower()
            instance["center"] = [
                parent_center[0] + local_x * cos_yaw - local_y * sin_yaw,
                parent_center[1] + local_x * sin_yaw + local_y * cos_yaw,
                parent_center[2] + local_z,
            ]
            instance["yaw"] = parent_yaw + float(local_pose[5])
            if "height" not in instance and "length" in instance:
                instance["height"] = instance["length"]
            instances.append(instance)
    return instances


def visual_mesh_collision_instances(scene, include_cloud_only=False):
    instances = []
    default_resolution = positive(
        scene.get("mesh_cloud_resolution", max(float(scene.get("cloud_resolution", 0.35)), 0.75)),
        "mesh_cloud_resolution")
    for item_index, item in enumerate(scene.get("visual_vessels", []) or []):
        if not visual_mesh_collision_enabled(item):
            continue
        if include_cloud_only and not item.get("mesh_collision_include_in_cloud", True):
            continue
        parent_center = as_float_list(
            item.get("center", [0.0, 0.0, 0.0]), 3,
            "visual_vessels[%d].center" % item_index)
        parent_rpy = as_float_list(
            item.get("rpy", [0.0, 0.0, float(item.get("yaw", 0.0))]), 3,
            "visual_vessels[%d].rpy" % item_index)
        parent_matrix = rpy_matrix(parent_rpy)
        parent_name = item.get("name", "visual_vessel_%d" % item_index)
        for mesh_index, mesh in enumerate(visual_mesh_specs(item)):
            mesh_pose = mesh["pose"]
            mesh_matrix = rpy_matrix(mesh_pose[3:6])
            local_offset = mat_vec(parent_matrix, mesh_pose[:3])
            world_center = [
                parent_center[0] + local_offset[0],
                parent_center[1] + local_offset[1],
                parent_center[2] + local_offset[2],
            ]
            world_matrix = mat_mul(parent_matrix, mesh_matrix)
            instance = {
                "name": "%s__%s_mesh_collision" % (parent_name, mesh.get("name", "mesh_%d" % mesh_index)),
                "parent_name": parent_name,
                "shape": "mesh",
                "uri": mesh["uri"],
                "scale": mesh["scale"],
                "pose": mesh_pose,
                "center": world_center,
                "matrix": world_matrix,
                "yaw": matrix_yaw(world_matrix),
                "cloud_resolution": float(item.get("mesh_cloud_resolution", default_resolution)),
                "max_cloud_points": int(item.get("mesh_cloud_max_points", 80000)),
            }
            instances.append(instance)
    return instances


def sample_collision_proxy(proxy, default_resolution):
    resolution = positive(proxy.get("cloud_resolution", default_resolution), "collision_proxy.cloud_resolution")
    shape = str(proxy.get("shape", "box")).lower()
    if shape == "box":
        return sample_box(proxy.get("center"), proxy.get("size"), resolution,
                          yaw=float(proxy.get("yaw", 0.0)))
    if shape == "cylinder":
        return sample_cylinder(proxy, resolution)
    raise ValueError("unsupported collision proxy shape: %s" % shape)


def sample_mesh_collision(instance, default_resolution):
    resolution = positive(instance.get("cloud_resolution", default_resolution), "mesh_collision.cloud_resolution")
    max_points = max(0, int(instance.get("max_cloud_points", 0)))
    scale = as_float_list(instance.get("scale", [1.0, 1.0, 1.0]), 3, "mesh_collision.scale")
    center = as_float_list(instance.get("center", [0.0, 0.0, 0.0]), 3, "mesh_collision.center")
    matrix = instance.get("matrix")
    if matrix is None:
        matrix = rpy_matrix([0.0, 0.0, float(instance.get("yaw", 0.0))])
    occupied = OrderedDict()
    for vertex in load_obj_vertices(instance["uri"]):
        scaled_vertex = [vertex[0] * scale[0], vertex[1] * scale[1], vertex[2] * scale[2]]
        rotated = mat_vec(matrix, scaled_vertex)
        world = [
            center[0] + rotated[0],
            center[1] + rotated[1],
            center[2] + rotated[2],
        ]
        key = (
            int(math.floor(world[0] / resolution)),
            int(math.floor(world[1] / resolution)),
            int(math.floor(world[2] / resolution)),
        )
        if key not in occupied:
            occupied[key] = quantize_point(world)

    points = list(occupied.values())
    if max_points > 0 and len(points) > max_points:
        stride = int(math.ceil(float(len(points)) / float(max_points)))
        points = points[::stride][:max_points]
    return points


def axis_vector_from_motion(motion):
    if "axis_vector" in motion:
        axis = as_float_list(motion.get("axis_vector"), 3, "motion.axis_vector")
        norm = math.sqrt(sum(value * value for value in axis))
        if norm <= 0.0:
            raise ValueError("motion.axis_vector must be non-zero")
        return [value / norm for value in axis]

    axis = str(motion.get("axis", "x")).lower()
    if axis == "x":
        return [1.0, 0.0, 0.0]
    if axis == "y":
        return [0.0, 1.0, 0.0]
    if axis == "z":
        return [0.0, 0.0, 1.0]
    raise ValueError("motion.axis must be one of x, y, z")


def dynamic_motion_offset(motion, time_sec):
    if not motion:
        return [0.0, 0.0, 0.0]

    motion_type = str(motion.get("type", "static")).lower()
    if motion_type in ("static", "none", "off"):
        return [0.0, 0.0, 0.0]
    if motion_type != "sinusoid":
        raise ValueError("unsupported dynamic obstacle motion type: %s" % motion_type)

    amplitude = float(motion.get("amplitude", 0.0))
    period_sec = positive(motion.get("period_sec", 1.0), "motion.period_sec")
    phase_rad = float(motion.get("phase_rad", 0.0))
    axis = axis_vector_from_motion(motion)
    value = amplitude * math.sin((2.0 * math.pi * float(time_sec) / period_sec) + phase_rad)
    return [axis[0] * value, axis[1] * value, axis[2] * value]


def dynamic_obstacle_center(item, time_sec=0.0):
    center = as_float_list(item.get("center"), 3, "dynamic_obstacle.center")
    offset = dynamic_motion_offset(item.get("motion") or {}, time_sec)
    return [center[0] + offset[0], center[1] + offset[1], center[2] + offset[2]]


def dynamic_obstacle_yaw(item, time_sec=0.0):
    yaw = float(item.get("yaw", 0.0))
    motion = item.get("motion") or {}
    if str(motion.get("yaw_mode", "fixed")).lower() != "track":
        return yaw

    motion_type = str(motion.get("type", "static")).lower()
    if motion_type != "sinusoid":
        return yaw
    axis = axis_vector_from_motion(motion)
    amplitude = float(motion.get("amplitude", 0.0))
    period_sec = positive(motion.get("period_sec", 1.0), "motion.period_sec")
    phase_rad = float(motion.get("phase_rad", 0.0))
    velocity_scale = amplitude * (2.0 * math.pi / period_sec) * math.cos(
        (2.0 * math.pi * float(time_sec) / period_sec) + phase_rad)
    vx = axis[0] * velocity_scale
    vy = axis[1] * velocity_scale
    if abs(vx) < 1e-6 and abs(vy) < 1e-6:
        return yaw
    return math.atan2(vy, vx)


def scene_dynamic_obstacles(scene):
    return list(scene.get("dynamic_obstacles", []) or [])


def dynamic_collision_proxy_instances(item, time_sec=0.0, include_cloud_only=False):
    if include_cloud_only and not item.get("include_in_cloud", True):
        return []
    proxies = item.get("collision_proxies", []) or []
    if not proxies:
        return []

    parent_center = dynamic_obstacle_center(item, time_sec)
    parent_yaw = dynamic_obstacle_yaw(item, time_sec)
    cos_yaw = math.cos(parent_yaw)
    sin_yaw = math.sin(parent_yaw)
    parent_name = item.get("name", "dynamic_obstacle")
    instances = []
    for proxy_index, proxy in enumerate(proxies):
        if include_cloud_only and not proxy.get("include_in_cloud", True):
            continue
        local_pose = collision_proxy_local_pose(
            proxy, "dynamic_obstacles.%s.collision_proxies[%d]" % (parent_name, proxy_index))
        local_x, local_y, local_z = local_pose[:3]
        instance = dict(proxy)
        proxy_name = proxy.get("name", "collision_proxy_%d" % proxy_index)
        instance["name"] = "%s__%s" % (parent_name, proxy_name)
        instance["parent_name"] = parent_name
        instance["shape"] = str(proxy.get("shape", "box")).lower()
        instance["center"] = [
            parent_center[0] + local_x * cos_yaw - local_y * sin_yaw,
            parent_center[1] + local_x * sin_yaw + local_y * cos_yaw,
            parent_center[2] + local_z,
        ]
        instance["yaw"] = parent_yaw + float(local_pose[5])
        if "height" not in instance and "length" in instance:
            instance["height"] = instance["length"]
        instances.append(instance)
    return instances


def scene_has_dynamic_obstacles(scene):
    return bool(scene_dynamic_obstacles(scene))


def dynamic_mode_enabled(scene, mode):
    normalized = str(mode or "auto").strip().lower()
    if normalized in ("off", "false", "0", "disabled", "disable", "none"):
        return False
    if normalized in ("on", "true", "1", "enabled", "enable"):
        return scene_has_dynamic_obstacles(scene)
    return scene_has_dynamic_obstacles(scene)


def sample_dynamic_obstacle(item, resolution, time_sec=0.0):
    shape = str(item.get("shape", "box")).lower()
    center = dynamic_obstacle_center(item, time_sec)
    if shape == "box":
        return sample_box(center, item.get("size"), resolution,
                          yaw=dynamic_obstacle_yaw(item, time_sec))
    if shape == "cylinder":
        cylinder = dict(item)
        cylinder["center"] = center
        return sample_cylinder(cylinder, resolution)
    if shape == "composite":
        raise ValueError("composite dynamic obstacle requires collision_proxies")
    raise ValueError("unsupported dynamic obstacle shape: %s" % shape)


def scene_dynamic_cloud_points(scene, dynamic_time_sec=0.0, include_labels=False):
    resolution = positive(scene.get("cloud_resolution", 0.35), "cloud_resolution")
    labeled = []
    for item in scene_dynamic_obstacles(scene):
        if not item.get("include_in_cloud", True):
            continue
        label = item.get("name", "dynamic_obstacle")
        proxies = dynamic_collision_proxy_instances(item, dynamic_time_sec, include_cloud_only=True)
        if proxies:
            for proxy in proxies:
                for point in sample_collision_proxy(proxy, resolution):
                    labeled.append((proxy.get("name", label), point))
        else:
            for point in sample_dynamic_obstacle(item, resolution, dynamic_time_sec):
                labeled.append((label, point))

    unique = OrderedDict()
    for label, point in labeled:
        unique[point] = label

    points = sorted(unique.keys())
    if include_labels:
        return [(unique[point], point) for point in points]
    return points


def scene_cloud_points(scene, include_labels=False, dynamic_time_sec=0.0, include_dynamic=True):
    resolution = positive(scene.get("cloud_resolution", 0.35), "cloud_resolution")
    labeled = []

    for key in ("deck", "landing_box", "takeoff_deck_zone", "landing_deck_zone"):
        zone = scene_box(scene, key)
        if zone and zone.get("include_in_cloud", False):
            for point in sample_box(zone.get("center"), zone.get("size"), resolution,
                                    yaw=zone.get("yaw", 0.0)):
                labeled.append((zone.get("name", key), point))

    for item in scene.get("bridge_piers", []) or []:
        for point in sample_cylinder(item, resolution):
            labeled.append((item.get("name", "bridge_pier"), point))

    for item in scene.get("buoys", []) or []:
        if not item.get("include_in_cloud", True):
            continue
        for point in sample_cylinder(item, resolution):
            labeled.append((item.get("name", "buoy"), point))

    for item in scene.get("docks", []) or []:
        if not item.get("include_in_cloud", True):
            continue
        for point in sample_box(item.get("center"), item.get("size"), resolution,
                                yaw=float(item.get("yaw", 0.0))):
            labeled.append((item.get("name", "dock"), point))

    for item in scene.get("box_obstacles", []) or []:
        if not item.get("include_in_cloud", True):
            continue
        for point in sample_box(item.get("center"), item.get("size"), resolution,
                                yaw=float(item.get("yaw", 0.0))):
            labeled.append((item.get("name", "box_obstacle"), point))

    for item in visual_mesh_collision_instances(scene, include_cloud_only=True):
        for point in sample_mesh_collision(item, resolution):
            labeled.append((item.get("name", "mesh_collision"), point))

    for item in visual_collision_proxy_instances(scene, include_cloud_only=True):
        for point in sample_collision_proxy(item, resolution):
            labeled.append((item.get("name", "collision_proxy"), point))

    if include_dynamic:
        for label, point in scene_dynamic_cloud_points(scene, dynamic_time_sec, include_labels=True):
            labeled.append((label, point))

    unique = OrderedDict()
    for label, point in labeled:
        unique[point] = label

    points = sorted(unique.keys())
    if include_labels:
        return [(unique[point], point) for point in points]
    return points


def scene_bounds(points):
    if not points:
        return None
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]
    return {
        "min": [min(xs), min(ys), min(zs)],
        "max": [max(xs), max(ys), max(zs)],
    }


def scene_waypoints(scene):
    waypoints = []
    for item in scene.get("waypoints", []) or []:
        if "position" in item:
            position = as_float_list(item.get("position"), 3, "waypoint.position")
            source = "local"
        elif "gps_position" in item:
            position = gps_to_local_position(scene, item.get("gps_position"))
            source = "gps"
        else:
            raise ValueError("waypoint requires position or gps_position")
        waypoints.append({
            "name": item.get("name", "waypoint_%d" % len(waypoints)),
            "label": item.get("label"),
            "position": position,
            "gps_position": as_float_list(item["gps_position"], 3, "waypoint.gps_position")
            if "gps_position" in item else None,
            "source": source,
            "yaw": float(item.get("yaw", 0.0)),
            "radius": float(item.get("radius", (scene.get("acceptance") or {}).get("position_tolerance_m", 0.75))),
            "hold_time": float(item.get("hold_time", 0.0)),
            "max_duration_sec": float(item.get("max_duration_sec", 0.0)),
        })
    return waypoints


def validate_scene(scene):
    errors = []
    for key in ("deck", "landing_box", "takeoff_deck_zone", "landing_deck_zone",
                "ship_hull", "docks", "bridge_piers", "buoys",
                "box_obstacles", "waypoints", "acceptance"):
        if key not in scene:
            errors.append("missing key: %s" % key)

    try:
        scene_box(scene, "deck", required=True)
    except Exception as exc:
        errors.append(str(exc))

    try:
        scene_box(scene, "landing_box", required=True)
    except Exception as exc:
        errors.append(str(exc))

    try:
        deck = scene_box(scene, "deck", required=True)
        takeoff = scene_box(scene, "takeoff_deck_zone", required=True)
        landing_zone = scene_box(scene, "landing_deck_zone", required=True)
        deck_x_min = deck["center"][0] - deck["size"][0] / 2.0
        deck_x_max = deck["center"][0] + deck["size"][0] / 2.0
        deck_y_min = deck["center"][1] - deck["size"][1] / 2.0
        deck_y_max = deck["center"][1] + deck["size"][1] / 2.0
        deck_z_top = deck["center"][2] + deck["size"][2] / 2.0
        for zone, label in ((takeoff, "takeoff_deck_zone"), (landing_zone, "landing_deck_zone")):
            if not (deck_x_min <= zone["center"][0] <= deck_x_max):
                raise ValueError("%s.center.x must lie on the ship deck" % label)
            if not zone.get("allow_outboard", False) and not (deck_y_min <= zone["center"][1] <= deck_y_max):
                raise ValueError("%s.center.y must lie on the ship deck" % label)
            if zone["center"][2] < deck_z_top - 0.05:
                raise ValueError("%s.center.z must sit on or above deck height" % label)
    except Exception as exc:
        errors.append(str(exc))

    try:
        points = scene_cloud_points(scene)
        acceptance = scene.get("acceptance") or {}
        min_points = int(acceptance.get("min_cloud_points", 1))
        max_points = int(acceptance.get("max_cloud_points", 1000000))
        if not (min_points <= len(points) <= max_points):
            errors.append("cloud point count %d outside [%d, %d]" % (len(points), min_points, max_points))
        positive(acceptance.get("position_tolerance_m", 0.0), "acceptance.position_tolerance_m")
        positive(acceptance.get("goal_publish_period_sec", 0.0), "acceptance.goal_publish_period_sec")
        positive(acceptance.get("min_water_clearance_m", 0.0), "acceptance.min_water_clearance_m")
        positive(acceptance.get("min_obstacle_distance_m", 0.0), "acceptance.min_obstacle_distance_m")
        positive(acceptance.get("final_zone_hold_sec", 0.0), "acceptance.final_zone_hold_sec")
    except Exception as exc:
        errors.append(str(exc))

    try:
        for index, item in enumerate(scene.get("ship_hull", []) or []):
            as_float_list(item.get("center"), 3, "ship_hull[%d].center" % index)
            as_float_list(item.get("size"), 3, "ship_hull[%d].size" % index)
        for index, item in enumerate(scene.get("docks", []) or []):
            as_float_list(item.get("center"), 3, "docks[%d].center" % index)
            as_float_list(item.get("size"), 3, "docks[%d].size" % index)
        for index, item in enumerate(scene.get("buoys", []) or []):
            as_float_list(item.get("center"), 3, "buoys[%d].center" % index)
            positive(item.get("radius", 0.0), "buoys[%d].radius" % index)
            positive(item.get("height", 0.0), "buoys[%d].height" % index)
        for index, item in enumerate(scene_dynamic_obstacles(scene)):
            as_float_list(item.get("center"), 3, "dynamic_obstacles[%d].center" % index)
            shape = str(item.get("shape", "box")).lower()
            proxies = item.get("collision_proxies", []) or []
            if shape == "box":
                as_float_list(item.get("size"), 3, "dynamic_obstacles[%d].size" % index)
            elif shape == "cylinder":
                positive(item.get("radius", 0.0), "dynamic_obstacles[%d].radius" % index)
                positive(item.get("height", 0.0), "dynamic_obstacles[%d].height" % index)
            elif shape == "composite":
                if not proxies:
                    raise ValueError("dynamic_obstacles[%d].composite requires collision_proxies" % index)
            else:
                raise ValueError("dynamic_obstacles[%d].shape must be box, cylinder, or composite" % index)
            dynamic_obstacle_center(item, 0.0)
            envelope = item.get("safety_envelope")
            if envelope:
                env_key = "dynamic_obstacles[%d].safety_envelope" % index
                as_float_list(envelope.get("center", [0.0, 0.0, 0.0]), 3, "%s.center" % env_key)
                env_shape = str(envelope.get("shape", shape)).lower()
                if env_shape == "box":
                    as_float_list(envelope.get("size", item.get("size")), 3, "%s.size" % env_key)
                elif env_shape == "cylinder":
                    positive(envelope.get("radius", item.get("radius", 0.0)), "%s.radius" % env_key)
                    positive(envelope.get("height", item.get("height", 0.0)), "%s.height" % env_key)
                else:
                    raise ValueError("%s.shape must be box or cylinder" % env_key)
            for proxy_index, proxy in enumerate(item.get("collision_proxies", []) or []):
                key = "dynamic_obstacles[%d].collision_proxies[%d]" % (index, proxy_index)
                collision_proxy_local_pose(proxy, key)
                proxy_shape = str(proxy.get("shape", "box")).lower()
                if proxy_shape == "box":
                    as_float_list(proxy.get("size"), 3, "%s.size" % key)
                elif proxy_shape == "cylinder":
                    positive(proxy.get("radius", 0.0), "%s.radius" % key)
                    positive(proxy.get("height", proxy.get("length", 0.0)), "%s.height" % key)
                else:
                    raise ValueError("%s.shape must be box or cylinder" % key)
                if "cloud_resolution" in proxy:
                    positive(proxy.get("cloud_resolution"), "%s.cloud_resolution" % key)
        for item_index, item in enumerate(scene.get("visual_vessels", []) or []):
            as_float_list(item.get("center", [0.0, 0.0, 0.0]), 3,
                          "visual_vessels[%d].center" % item_index)
            if "rpy" in item:
                as_float_list(item.get("rpy"), 3, "visual_vessels[%d].rpy" % item_index)
            if visual_mesh_collision_enabled(item):
                specs = visual_mesh_specs(item)
                if not specs:
                    raise ValueError("visual_vessels[%d].mesh_collision requires mesh_uri or visual_meshes" % item_index)
                if "mesh_cloud_resolution" in item:
                    positive(item.get("mesh_cloud_resolution"),
                             "visual_vessels[%d].mesh_cloud_resolution" % item_index)
                if "mesh_cloud_max_points" in item:
                    positive(item.get("mesh_cloud_max_points"),
                             "visual_vessels[%d].mesh_cloud_max_points" % item_index)
            for proxy_index, proxy in enumerate(item.get("collision_proxies", []) or []):
                key = "visual_vessels[%d].collision_proxies[%d]" % (item_index, proxy_index)
                collision_proxy_local_pose(proxy, key)
                shape = str(proxy.get("shape", "box")).lower()
                if shape == "box":
                    as_float_list(proxy.get("size"), 3, "%s.size" % key)
                elif shape == "cylinder":
                    positive(proxy.get("radius", 0.0), "%s.radius" % key)
                    positive(proxy.get("height", proxy.get("length", 0.0)), "%s.height" % key)
                else:
                    raise ValueError("%s.shape must be box or cylinder" % key)
                if "cloud_resolution" in proxy:
                    positive(proxy.get("cloud_resolution"), "%s.cloud_resolution" % key)
    except Exception as exc:
        errors.append(str(exc))

    try:
        if not scene_waypoints(scene):
            errors.append("at least one waypoint is required")
    except Exception as exc:
        errors.append(str(exc))

    return errors
