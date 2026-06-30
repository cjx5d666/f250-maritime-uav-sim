#!/usr/bin/env python3
import csv
import json
import math
import os
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import patheffects as pe
import numpy as np
import trimesh
import yaml
from PIL import Image, ImageColor, ImageDraw
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Rectangle, Polygon
from matplotlib.transforms import Affine2D


def resolve_project_root():
    env_root = os.environ.get("F250_PROJECT_ROOT")
    if env_root:
        return Path(env_root).expanduser().resolve()
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "catkin_ws/src/f250_maritime_uav_sim/models").is_dir():
            return parent
    return Path.cwd().resolve()


ROOT = resolve_project_root()
OUT_DIR = Path(__file__).resolve().parent
SOURCE_DIR = OUT_DIR / "sources"
SCENE_PATH = SOURCE_DIR / "scene.yaml"

X_RANGE = (0.0, 300.0)
Y_RANGE = (-120.0, 120.0)
PX_PER_M = 6.0
CANVAS_W = int(round((X_RANGE[1] - X_RANGE[0]) * PX_PER_M)) + 1
CANVAS_H = int(round((Y_RANGE[1] - Y_RANGE[0]) * PX_PER_M)) + 1

STANDARD_OUTPUTS = {
    "clean": "base_world.png",
    "planner": "obstacle_map.png",
    "route": "route_map.png",
    "result": "route_result.png",
}


VISUAL_STYLE = {
    "oasis": {"face": "#d5e0e7", "edge": "#4c5e68", "alpha": 0.92, "z": 8},
    "tanker": {"face": "#cbd5dc", "edge": "#46545f", "alpha": 0.92, "z": 8},
    "island": {"face": "#bfd4ae", "edge": "#708a69", "alpha": 0.86, "z": 4},
    "red_bridge": {"face": "#cf6f60", "edge": "#9f3d32", "alpha": 0.72, "z": 7},
    "white_bridge": {"face": "#e8eef2", "edge": "#98a6ae", "alpha": 0.74, "z": 7},
    "wind": {"face": "#f8faf8", "edge": "#66737a", "alpha": 0.88, "z": 9},
    "buoy": {"face": "#f6b84d", "edge": "#b06a00", "alpha": 0.9, "z": 12},
    "other": {"face": "#d7d7d7", "edge": "#666666", "alpha": 0.7, "z": 6},
}

WAMV_VISUAL_LENGTH_M = 20.0
WAMV_VISUAL_BEAM_M = 9.95

LABEL_STROKE = [pe.withStroke(linewidth=2.8, foreground="white", alpha=0.92)]

WAYPOINT_LABEL_OFFSETS = {
    "P0": (2.2, -2.2),
    "P1": (1.6, 2.4),
    "P2": (1.6, 2.2),
    "P3": (1.6, -2.0),
    "P4": (1.8, 1.8),
    "P5": (1.6, 2.2),
    "P6": (1.6, -3.0),
    "P7": (1.8, 1.6),
    "P8": (1.7, 1.5),
}

BUOY_LABEL_OFFSETS = {
    "O1": (2.2, 2.2),
    "O2": (2.2, 1.0),
    "O3": (2.1, -2.0),
    "O4": (2.1, 2.1),
    "O5": (2.1, 1.3),
}


def as_scale3(value):
    if isinstance(value, (int, float)):
        return np.array([float(value), float(value), float(value)], dtype=float)
    if isinstance(value, (list, tuple)):
        if len(value) == 3:
            return np.array([float(value[0]), float(value[1]), float(value[2])], dtype=float)
        if len(value) == 1:
            return np.array([float(value[0]), float(value[0]), float(value[0])], dtype=float)
    return np.ones(3, dtype=float)


def yaw_of(item):
    if "yaw" in item:
        return float(item.get("yaw", 0.0))
    rpy = item.get("rpy") or [0.0, 0.0, 0.0]
    return float(rpy[2]) if len(rpy) >= 3 else 0.0


def resolve_model_uri(uri):
    if not uri.startswith("model://"):
        return Path(uri)
    rel = uri[len("model://"):]
    model, rest = rel.split("/", 1)
    return ROOT / "catkin_ws/src/f250_maritime_uav_sim/models" / model / rest


def project_display_path(path):
    path = Path(path).expanduser().resolve()
    try:
        return "${F250_PROJECT_ROOT}/" + path.relative_to(ROOT).as_posix()
    except ValueError:
        return path.as_posix()


def transform_xyz(points, item):
    center = np.array(item.get("center", [0.0, 0.0, 0.0]), dtype=float)
    scale = as_scale3(item.get("scale", item.get("mesh_scale", 1.0)))
    local = points * scale
    yaw = yaw_of(item)
    c, s = math.cos(yaw), math.sin(yaw)
    rot = np.array([[c, -s], [s, c]], dtype=float)
    xy = local[:, :2] @ rot.T + center[:2]
    z = local[:, 2] + center[2]
    return np.column_stack([xy, z])


def convex_hull(points):
    pts = sorted(set((float(x), float(y)) for x, y in points))
    if len(pts) <= 1:
        return np.array(pts, dtype=float)

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return np.array(lower[:-1] + upper[:-1], dtype=float)


def classify_visual(name):
    n = name.lower()
    if "oasis" in n or "carrier" in n:
        return "oasis"
    if "tanker" in n:
        return "tanker"
    if "island" in n or "kauai" in n:
        return "island"
    if "golden" in n:
        return "red_bridge"
    if "helix" in n:
        return "white_bridge"
    if "wind" in n:
        return "wind"
    if "navigation_buoy" in n or (n.startswith("o") and ("reference" in n or "near_island" in n)):
        return "buoy"
    return "other"


def load_visual_mesh(item):
    mesh_path = resolve_model_uri(item["mesh_uri"])
    mesh = trimesh.load(mesh_path, force="mesh", process=False)
    vertices = np.asarray(mesh.vertices, dtype=float)
    faces = np.asarray(mesh.faces, dtype=int)
    world = transform_xyz(vertices, item)
    hull = convex_hull(world[:, :2])
    return {
        "name": item["name"],
        "mesh_path": project_display_path(mesh_path),
        "style": classify_visual(item["name"]),
        "yaw": yaw_of(item),
        "vertices": int(len(vertices)),
        "faces": faces,
        "world_xy": world[:, :2],
        "hull": hull,
        "center": [float(item.get("center", [0.0, 0.0, 0.0])[0]), float(item.get("center", [0.0, 0.0, 0.0])[1])],
        "bounds": {
            "min_x": float(np.min(world[:, 0])),
            "max_x": float(np.max(world[:, 0])),
            "min_y": float(np.min(world[:, 1])),
            "max_y": float(np.max(world[:, 1])),
            "min_z": float(np.min(world[:, 2])),
            "max_z": float(np.max(world[:, 2])),
        },
    }


def text_with_halo(ax, x, y, text, **kwargs):
    default = {
        "fontsize": 7.4,
        "color": "#253238",
        "weight": "bold",
        "zorder": 35,
    }
    default.update(kwargs)
    t = ax.text(x, y, text, **default)
    t.set_path_effects(LABEL_STROKE)
    return t


def world_to_pixel(points):
    pts = np.asarray(points, dtype=float)
    cols = (pts[:, 0] - X_RANGE[0]) * PX_PER_M
    rows = (Y_RANGE[1] - pts[:, 1]) * PX_PER_M
    return [(float(c), float(r)) for c, r in zip(cols, rows)]


def rasterize_visual_meshes(meshes, skip_styles=None):
    skip_styles = set(skip_styles or [])
    base = Image.new("RGBA", (CANVAS_W, CANVAS_H), (0, 0, 0, 0))
    for rec in meshes:
        if rec["style"] in skip_styles:
            continue
        style = VISUAL_STYLE[rec["style"]]
        mask = Image.new("L", (CANVAS_W, CANVAS_H), 0)
        draw = ImageDraw.Draw(mask)
        xy = rec["world_xy"]
        for face in rec["faces"]:
            pts = xy[face]
            if (
                np.max(pts[:, 0]) < X_RANGE[0]
                or np.min(pts[:, 0]) > X_RANGE[1]
                or np.max(pts[:, 1]) < Y_RANGE[0]
                or np.min(pts[:, 1]) > Y_RANGE[1]
            ):
                continue
            draw.polygon(world_to_pixel(pts), fill=255)

        mask_np = np.asarray(mask, dtype=bool)
        if rec["style"] in {"island", "oasis", "tanker"}:
            layer = Image.fromarray(textured_visual_layer(rec, style, mask_np), mode="RGBA")
        else:
            fill_rgba = ImageColor.getrgb(style["face"]) + (int(255 * style["alpha"]),)
            layer = Image.new("RGBA", (CANVAS_W, CANVAS_H), fill_rgba)
            layer.putalpha(mask)
        base = Image.alpha_composite(base, layer)

        if mask_np.any():
            eroded = np.zeros_like(mask_np, dtype=bool)
            eroded[1:-1, 1:-1] = (
                mask_np[1:-1, 1:-1]
                & mask_np[:-2, 1:-1]
                & mask_np[2:, 1:-1]
                & mask_np[1:-1, :-2]
                & mask_np[1:-1, 2:]
            )
            boundary = mask_np & ~eroded
            edge_rgba = ImageColor.getrgb(style["edge"]) + (230,)
            edge_arr = np.zeros((CANVAS_H, CANVAS_W, 4), dtype=np.uint8)
            edge_arr[boundary] = edge_rgba
            base = Image.alpha_composite(base, Image.fromarray(edge_arr, mode="RGBA"))
    return np.asarray(base)


_WORLD_GRIDS = None


def world_grids():
    global _WORLD_GRIDS
    if _WORLD_GRIDS is None:
        cols = np.arange(CANVAS_W, dtype=float)
        rows = np.arange(CANVAS_H, dtype=float)
        xs = X_RANGE[0] + cols / PX_PER_M
        ys = Y_RANGE[1] - rows / PX_PER_M
        _WORLD_GRIDS = np.meshgrid(xs, ys)
    return _WORLD_GRIDS


def principal_frame(rec):
    pts = np.asarray(rec["world_xy"], dtype=float)
    center = np.asarray(rec["center"], dtype=float)
    shifted = pts - center
    cov = np.cov(shifted.T)
    vals, vecs = np.linalg.eigh(cov)
    tangent = vecs[:, int(np.argmax(vals))]
    if tangent[0] < 0:
        tangent = -tangent
    normal = np.array([-tangent[1], tangent[0]])
    u = shifted @ tangent
    v = shifted @ normal
    half_len = max(1.0, float(np.percentile(np.abs(u), 98.5)))
    half_width = max(1.0, float(np.percentile(np.abs(v), 98.5)))
    return center, tangent, normal, half_len, half_width


def mix_rgb(a, b, t):
    t = np.asarray(t, dtype=float)
    if t.ndim == 0:
        return np.asarray(a, dtype=float) * (1.0 - float(t)) + np.asarray(b, dtype=float) * float(t)
    return np.asarray(a, dtype=float) * (1.0 - t[:, None]) + np.asarray(b, dtype=float) * t[:, None]


def textured_visual_layer(rec, style, mask_np):
    arr = np.zeros((CANVAS_H, CANVAS_W, 4), dtype=np.uint8)
    if not mask_np.any():
        return arr

    xs_grid, ys_grid = world_grids()
    xs = xs_grid[mask_np]
    ys = ys_grid[mask_np]
    base_rgb = np.array(ImageColor.getrgb(style["face"]), dtype=float)
    colors = np.tile(base_rgb, (len(xs), 1))

    if rec["style"] == "island":
        cx, cy = rec["center"]
        sx = max(1.0, rec["bounds"]["max_x"] - rec["bounds"]["min_x"])
        sy = max(1.0, rec["bounds"]["max_y"] - rec["bounds"]["min_y"])
        dx = (xs - cx) / sx
        dy = (ys - cy) / sy
        elev = (
            0.62 * np.exp(-((dx * 2.3) ** 2 + (dy * 2.0) ** 2))
            + 0.24 * np.exp(-(((dx - 0.18) * 3.1) ** 2 + ((dy + 0.12) * 2.7) ** 2))
            + 0.16 * np.exp(-(((dx + 0.24) * 3.5) ** 2 + ((dy - 0.20) * 2.9) ** 2))
        )
        elev += 0.045 * np.sin(xs * 0.21 + ys * 0.17)
        elev = np.clip(elev, 0.0, 1.0)
        low = np.array(ImageColor.getrgb("#b8d0a8"), dtype=float)
        mid = np.array(ImageColor.getrgb("#8fb27a"), dtype=float)
        high = np.array(ImageColor.getrgb("#d4cf9c"), dtype=float)
        colors = mix_rgb(low, mid, np.clip(elev * 1.25, 0.0, 1.0))
        high_mask = elev > 0.58
        if np.any(high_mask):
            colors[high_mask] = mix_rgb(colors[high_mask], high, np.clip((elev[high_mask] - 0.58) / 0.42, 0.0, 1.0))
        contour = np.abs((elev * 8.0) % 1.0 - 0.5) < 0.035
        colors[contour] *= 0.88
    elif rec["style"] in {"oasis", "tanker"}:
        center, tangent, normal, half_len, half_width = principal_frame(rec)
        shifted = np.column_stack([xs, ys]) - center
        u = shifted @ tangent / half_len
        v = shifted @ normal / half_width
        hull = np.array(ImageColor.getrgb("#c9d5dd" if rec["style"] == "oasis" else "#c4cfd6"), dtype=float)
        deck = np.array(ImageColor.getrgb("#eef5f8" if rec["style"] == "oasis" else "#e7ecef"), dtype=float)
        shadow = np.array(ImageColor.getrgb("#8fa0aa"), dtype=float)
        colors = np.tile(hull, (len(xs), 1))
        side = np.clip((np.abs(v) - 0.34) / 0.55, 0.0, 1.0)
        colors = mix_rgb(colors, shadow, side * 0.38)
        deck_mask = (np.abs(v) < 0.22) & (np.abs(u) < 0.86)
        colors[deck_mask] = mix_rgb(colors[deck_mask], deck, 0.78)
        center_line = (np.abs(v) < 0.035) & (np.abs(u) < 0.84)
        colors[center_line] = np.array(ImageColor.getrgb("#5f7581"), dtype=float)
        if rec["style"] == "oasis":
            windows = (np.abs(v) > 0.34) & (np.abs(v) < 0.47) & (np.abs(u) < 0.68)
            colors[windows] = mix_rgb(colors[windows], ImageColor.getrgb("#6f8795"), 0.70)
            pad = (u > 0.58) & (u < 0.80) & (np.abs(v) < 0.20)
            colors[pad] = mix_rgb(colors[pad], ImageColor.getrgb("#5fbf8b"), 0.70)
        else:
            hatches = (np.abs(v) < 0.26) & (np.abs(u) < 0.76) & (np.abs(np.sin((u + 1.0) * 18.0)) > 0.88)
            colors[hatches] = mix_rgb(colors[hatches], ImageColor.getrgb("#7c8a92"), 0.45)

    arr[mask_np, :3] = np.clip(colors, 0, 255).astype(np.uint8)
    arr[mask_np, 3] = int(255 * style["alpha"])
    return arr


def mesh_centerline(rec, bins=36):
    pts = np.asarray(rec["world_xy"], dtype=float)
    if len(pts) < 4:
        return pts
    center = np.mean(pts, axis=0)
    shifted = pts - center
    cov = np.cov(shifted.T)
    vals, vecs = np.linalg.eigh(cov)
    tangent = vecs[:, int(np.argmax(vals))]
    if tangent[0] < 0:
        tangent = -tangent
    normal = np.array([-tangent[1], tangent[0]])
    u = shifted @ tangent
    v = shifted @ normal
    lo, hi = np.percentile(u, [1.0, 99.0])
    edges = np.linspace(lo, hi, bins + 1)
    line = []
    for a, b in zip(edges[:-1], edges[1:]):
        mask = (u >= a) & (u <= b)
        if int(np.count_nonzero(mask)) < 6:
            continue
        um = float(np.median(u[mask]))
        vm = float(np.median(v[mask]))
        line.append(center + um * tangent + vm * normal)
    if len(line) < 2:
        hull = rec["hull"]
        return hull if len(hull) else pts
    return np.asarray(line)


def draw_bridge_icon(ax, rec):
    line = mesh_centerline(rec)
    if len(line) < 2:
        return
    if rec["style"] == "white_bridge":
        edge, face, rail = "#758b95", "#f8fcff", "#b9c7ce"
        edge_w, face_w, rail_w = 8.0, 5.8, 1.0
        z = 24
    else:
        edge, face, rail = "#974c44", "#df7668", "#b7554b"
        edge_w, face_w, rail_w = 9.2, 7.2, 1.0
        z = 23
    ax.plot(line[:, 0], line[:, 1], color=edge, linewidth=edge_w,
            alpha=0.88, zorder=z, solid_capstyle="round")
    ax.plot(line[:, 0], line[:, 1], color=face, linewidth=face_w,
            alpha=0.96, zorder=z + 1, solid_capstyle="round")
    ax.plot(line[:, 0], line[:, 1], color=rail, linewidth=rail_w,
            alpha=0.75, zorder=z + 2, solid_capstyle="round")


def draw_ship_icon(ax, rec):
    hull = rec["hull"]
    if len(hull) < 3:
        return
    center, tangent, normal, half_len, half_width = principal_frame(rec)
    edge = "#40515d" if rec["style"] == "oasis" else "#3f4a53"
    hull_face = "#dfe8ee" if rec["style"] == "oasis" else "#d4dde3"
    deck = "#f7fbfd" if rec["style"] == "oasis" else "#eef3f5"
    steel = "#9dafb9" if rec["style"] == "oasis" else "#91a0a8"

    hull_patch = Polygon(hull, closed=True, facecolor=hull_face, edgecolor=edge,
                         linewidth=1.0, alpha=0.80, zorder=20)
    ax.add_patch(hull_patch)

    def xy(points):
        pts = np.asarray(points, dtype=float)
        return center + pts[:, 0:1] * half_len * tangent + pts[:, 1:2] * half_width * normal

    def poly(points, facecolor, edgecolor=None, linewidth=0.55, alpha=0.90, zorder=21):
        patch = Polygon(xy(points), closed=True, facecolor=facecolor,
                        edgecolor=edgecolor or facecolor, linewidth=linewidth,
                        alpha=alpha, zorder=zorder)
        patch.set_clip_path(hull_patch)
        ax.add_patch(patch)
        return patch

    def line(points, color, linewidth=0.75, alpha=0.90, zorder=22, style="-"):
        pts = xy(points)
        ln, = ax.plot(pts[:, 0], pts[:, 1], color=color, linewidth=linewidth,
                      alpha=alpha, zorder=zorder, linestyle=style,
                      solid_capstyle="round")
        ln.set_clip_path(hull_patch)
        return ln

    shadow = xy([[-0.94, -0.70], [0.94, -0.70]])
    ax.plot(shadow[:, 0], shadow[:, 1], color="#71818b", linewidth=1.4,
            alpha=0.34, zorder=20.5, solid_capstyle="round")

    if rec["style"] == "oasis":
        poly([[-0.82, -0.20], [0.84, -0.20], [0.84, 0.20], [-0.82, 0.20]],
             deck, edgecolor="#8ba0aa", linewidth=0.55, alpha=0.96, zorder=21)
        poly([[-0.58, -0.08], [0.30, -0.08], [0.30, 0.08], [-0.58, 0.08]],
             "#d2e2ea", edgecolor="#8da2ad", linewidth=0.45, alpha=0.92, zorder=22)
        poly([[0.44, -0.16], [0.72, -0.16], [0.72, 0.16], [0.44, 0.16]],
             "#7fc4a0", edgecolor="#4f9273", linewidth=0.45, alpha=0.86, zorder=22)
        for side in (-1, 1):
            poly([[-0.72, side * 0.35], [0.62, side * 0.35],
                  [0.62, side * 0.48], [-0.72, side * 0.48]],
                 "#7892a0", edgecolor="#5a7280", linewidth=0.28, alpha=0.72, zorder=22)
            for u0 in np.linspace(-0.62, 0.48, 7):
                poly([[u0, side * 0.55], [u0 + 0.11, side * 0.55],
                      [u0 + 0.11, side * 0.68], [u0, side * 0.68]],
                     "#f0a640", edgecolor="#b87826", linewidth=0.22, alpha=0.70, zorder=23)
        for u0 in np.linspace(-0.45, 0.18, 4):
            poly([[u0, -0.16], [u0 + 0.12, -0.16],
                  [u0 + 0.12, 0.16], [u0, 0.16]],
                 "#f9fcfd", edgecolor="#aab8bf", linewidth=0.30, alpha=0.80, zorder=23)
        line([[-0.88, 0.0], [0.88, 0.0]], "#536a77", linewidth=0.8, alpha=0.72, zorder=24)
        line([[-0.78, -0.28], [0.76, -0.28]], "#9aadba", linewidth=0.55, alpha=0.70, zorder=23)
        line([[-0.78, 0.28], [0.76, 0.28]], "#9aadba", linewidth=0.55, alpha=0.70, zorder=23)
    else:
        poly([[-0.82, -0.24], [0.74, -0.24], [0.74, 0.24], [-0.82, 0.24]],
             deck, edgecolor="#83939b", linewidth=0.55, alpha=0.94, zorder=21)
        for u0 in np.linspace(-0.62, 0.32, 5):
            poly([[u0, -0.18], [u0 + 0.16, -0.18],
                  [u0 + 0.16, 0.18], [u0, 0.18]],
                 "#b8c5cc", edgecolor="#6d7c84", linewidth=0.40, alpha=0.86, zorder=22)
        poly([[0.52, -0.30], [0.78, -0.30], [0.78, 0.30], [0.52, 0.30]],
             "#e8edf0", edgecolor="#74848c", linewidth=0.45, alpha=0.92, zorder=22)
        poly([[0.60, -0.18], [0.72, -0.18], [0.72, 0.18], [0.60, 0.18]],
             "#b5c3ca", edgecolor="#75858d", linewidth=0.35, alpha=0.82, zorder=23)
        line([[-0.78, 0.0], [0.50, 0.0]], "#61727b", linewidth=0.75, alpha=0.78, zorder=23)
        line([[-0.72, -0.34], [0.42, -0.34]], steel, linewidth=0.55, alpha=0.70, zorder=22)
        line([[-0.72, 0.34], [0.42, 0.34]], steel, linewidth=0.55, alpha=0.70, zorder=22)
        for u0 in (-0.42, -0.05, 0.30):
            line([[u0, -0.20], [u0 + 0.18, 0.20]], "#7b8c94",
                 linewidth=0.45, alpha=0.62, zorder=23)


def draw_wind_icon(ax, rec):
    cx, cy = rec["center"]
    yaw = float(rec.get("yaw", 0.0))
    blade_len = 4.8
    mast_len = 5.2
    mast_dir = yaw + math.pi / 2.0
    ax.plot([cx - math.cos(mast_dir) * mast_len * 0.35, cx + math.cos(mast_dir) * mast_len * 0.35],
            [cy - math.sin(mast_dir) * mast_len * 0.35, cy + math.sin(mast_dir) * mast_len * 0.35],
            color="#7b878d", linewidth=1.2, alpha=0.75, zorder=26)
    for k in range(3):
        a = yaw + math.pi / 2.0 + k * 2.0 * math.pi / 3.0
        ax.plot([cx, cx + math.cos(a) * blade_len],
                [cy, cy + math.sin(a) * blade_len],
                color="#58676e", linewidth=1.35, alpha=0.96, zorder=28)
    ax.add_patch(Circle((cx, cy), 1.25, facecolor="#f9fbfa",
                        edgecolor="#40515a", linewidth=1.0, zorder=29))


def draw_readable_overlays(ax, meshes):
    bridges = [rec for rec in meshes if rec["style"] in {"red_bridge", "white_bridge"}]
    ships = [rec for rec in meshes if rec["style"] in {"oasis", "tanker"}]
    winds = [rec for rec in meshes if rec["style"] == "wind"]
    for rec in ships:
        draw_ship_icon(ax, rec)
    for rec in bridges:
        draw_bridge_icon(ax, rec)
    if winds:
        for rec in winds:
            draw_wind_icon(ax, rec)


def rotated_rect(ax, center, size, yaw, **kwargs):
    sx, sy = float(size[0]), float(size[1])
    transform = Affine2D().rotate(float(yaw)).translate(float(center[0]), float(center[1])) + ax.transData
    patch = Rectangle((-sx / 2.0, -sy / 2.0), sx, sy, transform=transform, **kwargs)
    ax.add_patch(patch)
    return patch


def rotate_point(px, py, yaw):
    c, s = math.cos(float(yaw)), math.sin(float(yaw))
    return px * c - py * s, px * s + py * c


def dynamic_visual_size(item):
    if item.get("visual_size"):
        size = item.get("visual_size")
        return float(size[0]), float(size[1])
    name = str(item.get("name", "")).lower()
    mesh_uri = str(item.get("mesh_uri", "")).lower()
    if "wamv" in name or "wamv" in mesh_uri:
        return WAMV_VISUAL_LENGTH_M, WAMV_VISUAL_BEAM_M
    size = item.get("size", [1.0, 1.0, 1.0])
    return float(size[0]), float(size[1])


def rotated_bbox(center, size, yaw):
    sx, sy = float(size[0]), float(size[1])
    hx, hy = sx / 2.0, sy / 2.0
    points = []
    for px, py in ((-hx, -hy), (-hx, hy), (hx, hy), (hx, -hy)):
        rx, ry = rotate_point(px, py, yaw)
        points.append((float(center[0]) + rx, float(center[1]) + ry))
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    return min(xs), max(xs), min(ys), max(ys)


def draw_dynamic_boat(ax, center, size, yaw, facecolor="#3aa4c7",
                      edgecolor="#176073", alpha=0.62, zorder=16):
    """Draw the dynamic obstacle as its real-size top-down boat footprint."""
    sx, sy = float(size[0]), float(size[1])
    half_x = sx / 2.0
    half_y = sy / 2.0
    bow_narrow = 0.36 * half_y
    points = [
        (-half_x, -half_y),
        (-half_x, half_y),
        (0.18 * sx, half_y),
        (0.36 * sx, bow_narrow),
        (half_x, 0.0),
        (0.36 * sx, -bow_narrow),
        (0.18 * sx, -half_y),
    ]
    cos_yaw = math.cos(float(yaw))
    sin_yaw = math.sin(float(yaw))
    cx, cy = float(center[0]), float(center[1])
    world_points = [
        (cx + px * cos_yaw - py * sin_yaw, cy + px * sin_yaw + py * cos_yaw)
        for px, py in points
    ]
    patch = Polygon(
        world_points,
        closed=True,
        facecolor=facecolor,
        edgecolor=edgecolor,
        linewidth=1.35,
        alpha=alpha,
        zorder=zorder,
    )
    ax.add_patch(patch)
    return patch


def draw_wamv_visual_icon(ax, center, size, yaw, zorder=24):
    """Draw a readable top-down WAM-V catamaran within the visual footprint."""
    sx, sy = float(size[0]), float(size[1])
    cx, cy = float(center[0]), float(center[1])

    # A faint full-footprint silhouette makes the visual size auditable.
    draw_dynamic_boat(
        ax, center, (sx, sy), yaw,
        facecolor="#7dc5d6", edgecolor="#176073", alpha=0.26, zorder=zorder)

    pontoon_len = sx * 0.92
    pontoon_width = sy * 0.16
    pontoon_offset = sy * 0.34
    beam_len = sy * 0.72
    beam_width = sx * 0.055

    for offset in (-pontoon_offset, pontoon_offset):
        ox, oy = rotate_point(0.0, offset, yaw)
        draw_dynamic_boat(
            ax,
            (cx + ox, cy + oy),
            (pontoon_len, pontoon_width),
            yaw,
            facecolor="#6fbfd2",
            edgecolor="#0b5261",
            alpha=0.92,
            zorder=zorder + 1,
        )

    for offset_x in (-sx * 0.22, sx * 0.18):
        ox, oy = rotate_point(offset_x, 0.0, yaw)
        rotated_rect(
            ax,
            (cx + ox, cy + oy),
            (beam_width, beam_len),
            yaw,
            facecolor="#d7eef4",
            edgecolor="#2b7481",
            linewidth=0.8,
            alpha=0.86,
            zorder=zorder + 2,
        )

    rotated_rect(
        ax,
        (cx, cy),
        (sx * 0.33, sy * 0.34),
        yaw,
        facecolor="#e9f4f6",
        edgecolor="#2b7481",
        linewidth=0.8,
        alpha=0.88,
        zorder=zorder + 3,
    )
    ox, oy = rotate_point(-sx * 0.12, 0.0, yaw)
    rotated_rect(
        ax,
        (cx + ox, cy + oy),
        (sx * 0.14, sy * 0.24),
        yaw,
        facecolor="#b6d7df",
        edgecolor="#2b7481",
        linewidth=0.7,
        alpha=0.88,
        zorder=zorder + 4,
    )


def setup_ax(ax):
    ax.set_facecolor("#e9f5fb")
    ax.set_xlim(*X_RANGE)
    ax.set_ylim(*Y_RANGE)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, color="#cfe2ec", linewidth=0.55, alpha=0.62)
    ax.set_xlabel("local X east (m)")
    ax.set_ylabel("local Y north (m)")


def draw_visual_layer(ax, meshes, labels=True, readable_icons=True):
    visual_rgba = rasterize_visual_meshes(meshes, skip_styles={"wind"} if readable_icons else set())
    ax.imshow(visual_rgba, extent=[X_RANGE[0], X_RANGE[1], Y_RANGE[0], Y_RANGE[1]], origin="upper", zorder=3)
    if readable_icons:
        draw_readable_overlays(ax, meshes)
    if labels:
        for rec in meshes:
            short = short_label(rec["name"])
            if not short:
                continue
            cx, cy = rec["center"]
            dy = 8.0 if rec["style"] == "wind" else 0.0
            text_with_halo(ax, cx, cy + dy, short, ha="center", va="center",
                           fontsize=7.3, color="#263238", zorder=34)


def short_label(name):
    head = name.split("_", 1)[0].upper()
    if len(head) > 1 and head[0] == "O" and head[1:].isdigit():
        return head
    mapping = {
        "oasis_of_the_seas_static_carrier": "S1",
        "tanker_ship_static_visual": "S2",
        "kauai_left_island_visual": "Island-L",
        "kauai_center_island_visual": "Island-C",
        "kauai_right_island_visual": "Island-R",
        "golden_gate_bridge_visual": "B1",
        "helix_bridge_visual": "B2",
        "task_wind_channel_1": "W1",
        "task_wind_channel_2": "W2",
        "task_wind_channel_3": "W3",
    }
    return mapping.get(name, "")


def is_buoy_visual(item):
    name = item.get("name", "").lower()
    mesh = str(item.get("mesh_uri", "")).lower()
    return "navigation_buoy" in mesh or (name.startswith("o") and ("reference" in name or "near_island" in name))


def planner_buoys(scene):
    rows = []
    seen = set()
    for item in scene.get("buoys", []) or []:
        name = item.get("name", "")
        if name:
            seen.add(name)
        rows.append(dict(item))
    for item in scene.get("visual_vessels", []) or []:
        if not is_buoy_visual(item):
            continue
        name = item.get("name", "")
        if name in seen:
            continue
        center = item.get("center", [0.0, 0.0, 0.0])
        scale = as_scale3(item.get("scale", [1.0, 1.0, 1.0]))
        rows.append({
            "name": name,
            "center": center,
            "radius": max(float(scale[0]), float(scale[1])),
            "height": float(scale[2]),
            "yaw": yaw_of(item),
            "visual": True,
            "include_in_cloud": bool(item.get("mesh_collision", item.get("include_in_cloud", True))),
        })
    return rows


def draw_planner_layer(ax, scene):
    for item in planner_buoys(scene):
        if not item.get("include_in_cloud", True):
            continue
        x, y = item["center"][:2]
        r = float(item.get("radius", 1.0))
        ax.add_patch(Circle((x, y), r, facecolor="#f6b84d", edgecolor="#b06a00",
                            linewidth=1.25, alpha=0.85, zorder=18))
        label = item["name"].split("_")[0].upper()
        dx, dy = BUOY_LABEL_OFFSETS.get(label, (2.0, 2.0))
        text_with_halo(ax, x + dx, y + dy, label, fontsize=7.2,
                       color="#7a4300", zorder=33)

    for item in scene.get("box_obstacles", []) or []:
        if not item.get("include_in_cloud", True):
            continue
        patch = rotated_rect(
            ax,
            item.get("center", [0.0, 0.0, 0.0]),
            item.get("size", [1.0, 1.0, 1.0])[:2],
            item.get("yaw", 0.0),
            facecolor="#7d8790",
            edgecolor="#40484f",
            linewidth=1.2,
            alpha=0.22,
            zorder=15,
        )
        patch.set_hatch("///")

    for item in scene.get("dynamic_obstacles", []) or []:
        center = item.get("center", [0.0, 0.0, 0.0])
        yaw = item.get("yaw", 0.0)
        visual_size = dynamic_visual_size(item)
        draw_wamv_visual_icon(ax, center, visual_size, yaw, zorder=24)
        text_with_halo(ax, center[0] + 8.0, center[1] + 3.8, "D1",
                       ha="left", va="center", fontsize=7.0,
                       color="#0b3945", zorder=33)


def draw_waypoints(ax, scene):
    points = []
    for item in scene.get("waypoints", []) or []:
        name = item["name"].split("_")[0].upper()
        x, y = item["position"][:2]
        points.append((x, y))
        ax.scatter([x], [y], color="#174ea6", edgecolor="white",
                   linewidth=0.8, s=34, zorder=31)
        dx, dy = WAYPOINT_LABEL_OFFSETS.get(name, (1.7, 1.7))
        text_with_halo(ax, x + dx, y + dy, name, fontsize=7.5,
                       color="#08306b", zorder=36)
    if points:
        ax.plot([p[0] for p in points], [p[1] for p in points],
                color="#174ea6", linewidth=2.25, alpha=0.96, zorder=30)


def save_figure(path, draw_planner=False, draw_points=False, legend_kind="none"):
    scene = yaml.safe_load(SCENE_PATH.read_text(encoding="utf-8"))
    meshes = [load_visual_mesh(item) for item in scene.get("visual_vessels", []) or [] if item.get("mesh_uri")]
    fig, ax = plt.subplots(figsize=(13.0, 8.2), dpi=180)
    setup_ax(ax)
    draw_visual_layer(ax, meshes)
    if draw_planner:
        draw_planner_layer(ax, scene)
    if draw_points:
        draw_waypoints(ax, scene)

    handles = []
    if legend_kind in {"obstacle", "route", "result"}:
        if legend_kind in {"route", "result"}:
            handles.append(Line2D([0], [0], color="#174ea6", lw=2.25, label="Planned route"))
            handles.append(Line2D([0], [0], color="#174ea6", marker="o", linestyle="", markersize=5.0,
                                  markeredgecolor="white", label="Waypoint"))
        handles.append(Line2D([0], [0], color="#b06a00", marker="o", linestyle="", markersize=5.0,
                              markerfacecolor="#f6b84d", label="Buoy obstacle"))
    if handles:
        ax.legend(handles=handles, loc="upper right", fontsize=7.5, framealpha=0.92)

    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
    return scene, meshes


def write_manifest(scene, meshes):
    visual_rows = []
    item_by_name = {item.get("name"): item for item in scene.get("visual_vessels", []) or []}
    for rec in meshes:
        item = item_by_name.get(rec["name"], {})
        center = item.get("center", [0.0, 0.0, 0.0])
        scale = as_scale3(item.get("scale", item.get("mesh_scale", 1.0)))
        row = {
            "name": rec["name"],
            "style": rec["style"],
            "mesh_path": rec["mesh_path"],
            "center_x": float(center[0]) if len(center) > 0 else 0.0,
            "center_y": float(center[1]) if len(center) > 1 else 0.0,
            "center_z": float(center[2]) if len(center) > 2 else 0.0,
            "yaw_rad": yaw_of(item),
            "scale_x": float(scale[0]),
            "scale_y": float(scale[1]),
            "scale_z": float(scale[2]),
            "vertices": rec["vertices"],
        }
        row.update(rec["bounds"])
        visual_rows.append(row)
    for item in scene.get("dynamic_obstacles", []) or []:
        name = item.get("name", "")
        if not ("wamv" in name.lower() or "wamv" in str(item.get("mesh_uri", "")).lower()):
            continue
        center = item.get("center", [0.0, 0.0, 0.0])
        yaw = yaw_of(item)
        scale = as_scale3(item.get("scale", item.get("mesh_scale", 1.0)))
        size = dynamic_visual_size(item)
        min_x, max_x, min_y, max_y = rotated_bbox(center, size, yaw)
        visual_rows.append({
            "name": name,
            "style": "wamv",
            "mesh_path": project_display_path(resolve_model_uri(item.get("mesh_uri", ""))) if item.get("mesh_uri") else "",
            "center_x": float(center[0]) if len(center) > 0 else 0.0,
            "center_y": float(center[1]) if len(center) > 1 else 0.0,
            "center_z": float(center[2]) if len(center) > 2 else 0.0,
            "yaw_rad": yaw,
            "scale_x": float(scale[0]),
            "scale_y": float(scale[1]),
            "scale_z": float(scale[2]),
            "vertices": 0,
            "min_x": min_x,
            "max_x": max_x,
            "min_y": min_y,
            "max_y": max_y,
            "min_z": float(center[2]) - 3.1 if len(center) > 2 else 0.0,
            "max_z": float(center[2]) + 3.1 if len(center) > 2 else 0.0,
        })
    visual_csv = "visual_mesh_footprints.csv"
    waypoint_csv = "route_waypoints.csv"
    obstacle_csv = "planner_obstacles.csv"
    layer_index_csv = "layer_index.csv"
    manifest_json = "map_manifest.json"

    with (SOURCE_DIR / visual_csv).open("w", encoding="utf-8", newline="") as fh:
        fields = [
            "name", "style", "mesh_path", "center_x", "center_y", "center_z",
            "yaw_rad", "scale_x", "scale_y", "scale_z", "vertices",
            "min_x", "max_x", "min_y", "max_y", "min_z", "max_z",
        ]
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        writer.writerows(visual_rows)

    with (SOURCE_DIR / waypoint_csv).open("w", encoding="utf-8", newline="") as fh:
        fields = [
            "index", "label", "name", "x", "y", "z", "yaw_rad", "radius_m",
            "hold_time_sec", "max_duration_sec",
        ]
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        for idx, item in enumerate(scene.get("waypoints", []) or []):
            pos = item.get("position", [0.0, 0.0, 0.0])
            name = item.get("name", f"p{idx}")
            writer.writerow({
                "index": idx,
                "label": name.split("_")[0].upper(),
                "name": name,
                "x": float(pos[0]),
                "y": float(pos[1]),
                "z": float(pos[2]) if len(pos) > 2 else 0.0,
                "yaw_rad": float(item.get("yaw", 0.0)),
                "radius_m": float(item.get("radius", 0.0)),
                "hold_time_sec": float(item.get("hold_time", 0.0)),
                "max_duration_sec": float(item.get("max_duration_sec", 0.0)),
            })

    obstacle_rows = []
    for item in planner_buoys(scene):
        center = item.get("center", [0.0, 0.0, 0.0])
        obstacle_rows.append({
            "type": "buoy_cylinder",
            "name": item.get("name", ""),
            "include_in_cloud": bool(item.get("include_in_cloud", True)),
            "visual": bool(item.get("visual", True)),
            "center_x": float(center[0]),
            "center_y": float(center[1]),
            "center_z": float(center[2]) if len(center) > 2 else 0.0,
            "radius_m": float(item.get("radius", 0.0)),
            "height_m": float(item.get("height", 0.0)),
            "size_x": "",
            "size_y": "",
            "size_z": "",
            "yaw_rad": float(item.get("yaw", 0.0)),
            "shape": "cylinder",
            "motion_type": "",
            "motion_axis_x": "",
            "motion_axis_y": "",
            "motion_axis_z": "",
            "motion_amplitude_m": "",
            "motion_period_sec": "",
            "motion_phase_rad": "",
        })
    for item in scene.get("box_obstacles", []) or []:
        center = item.get("center", [0.0, 0.0, 0.0])
        size = item.get("size", [0.0, 0.0, 0.0])
        obstacle_rows.append({
            "type": "static_box",
            "name": item.get("name", ""),
            "include_in_cloud": bool(item.get("include_in_cloud", True)),
            "visual": bool(item.get("visual", True)),
            "center_x": float(center[0]),
            "center_y": float(center[1]),
            "center_z": float(center[2]) if len(center) > 2 else 0.0,
            "radius_m": "",
            "height_m": "",
            "size_x": float(size[0]) if len(size) > 0 else 0.0,
            "size_y": float(size[1]) if len(size) > 1 else 0.0,
            "size_z": float(size[2]) if len(size) > 2 else 0.0,
            "yaw_rad": float(item.get("yaw", 0.0)),
            "shape": "box",
            "motion_type": "",
            "motion_axis_x": "",
            "motion_axis_y": "",
            "motion_axis_z": "",
            "motion_amplitude_m": "",
            "motion_period_sec": "",
            "motion_phase_rad": "",
        })
    for item in scene.get("dynamic_obstacles", []) or []:
        center = item.get("center", [0.0, 0.0, 0.0])
        size = item.get("size", [0.0, 0.0, 0.0])
        motion = item.get("motion", {}) or {}
        axis = motion.get("axis_vector", ["", "", ""])
        proxies = item.get("collision_proxies", []) or []
        if proxies:
            yaw = float(item.get("yaw", 0.0))
            for proxy in proxies:
                local = proxy.get("center", [0.0, 0.0, 0.0])
                offset_x, offset_y = rotate_point(float(local[0]), float(local[1]), yaw)
                proxy_size = proxy.get("size", [0.0, 0.0, 0.0])
                obstacle_rows.append({
                    "type": "dynamic_collision_proxy",
                    "name": "d1_%s" % proxy.get("name", "proxy").replace("_proxy", ""),
                    "include_in_cloud": bool(proxy.get("include_in_cloud", item.get("include_in_cloud", True))),
                    "visual": bool(item.get("visual", True)),
                    "center_x": float(center[0]) + offset_x,
                    "center_y": float(center[1]) + offset_y,
                    "center_z": float(center[2]) + (float(local[2]) if len(local) > 2 else 0.0),
                    "radius_m": "",
                    "height_m": "",
                    "size_x": float(proxy_size[0]) if len(proxy_size) > 0 else 0.0,
                    "size_y": float(proxy_size[1]) if len(proxy_size) > 1 else 0.0,
                    "size_z": float(proxy_size[2]) if len(proxy_size) > 2 else 0.0,
                    "yaw_rad": yaw,
                    "shape": proxy.get("shape", "box"),
                    "motion_type": motion.get("type", ""),
                    "motion_axis_x": axis[0] if len(axis) > 0 else "",
                    "motion_axis_y": axis[1] if len(axis) > 1 else "",
                    "motion_axis_z": axis[2] if len(axis) > 2 else "",
                    "motion_amplitude_m": motion.get("amplitude", ""),
                    "motion_period_sec": motion.get("period_sec", ""),
                    "motion_phase_rad": motion.get("phase_rad", ""),
                })
        else:
            obstacle_rows.append({
                "type": "dynamic_obstacle",
                "name": item.get("name", ""),
                "include_in_cloud": bool(item.get("include_in_cloud", True)),
                "visual": bool(item.get("visual", True)),
                "center_x": float(center[0]),
                "center_y": float(center[1]),
                "center_z": float(center[2]) if len(center) > 2 else 0.0,
                "radius_m": "",
                "height_m": "",
                "size_x": float(size[0]) if len(size) > 0 else 0.0,
                "size_y": float(size[1]) if len(size) > 1 else 0.0,
                "size_z": float(size[2]) if len(size) > 2 else 0.0,
                "yaw_rad": float(item.get("yaw", 0.0)),
                "shape": item.get("shape", "box"),
                "motion_type": motion.get("type", ""),
                "motion_axis_x": axis[0] if len(axis) > 0 else "",
                "motion_axis_y": axis[1] if len(axis) > 1 else "",
                "motion_axis_z": axis[2] if len(axis) > 2 else "",
                "motion_amplitude_m": motion.get("amplitude", ""),
                "motion_period_sec": motion.get("period_sec", ""),
                "motion_phase_rad": motion.get("phase_rad", ""),
            })
    with (SOURCE_DIR / obstacle_csv).open("w", encoding="utf-8", newline="") as fh:
        fields = [
            "type", "name", "include_in_cloud", "visual", "center_x",
            "center_y", "center_z", "radius_m", "height_m", "size_x",
            "size_y", "size_z", "yaw_rad", "shape", "motion_type",
            "motion_axis_x", "motion_axis_y", "motion_axis_z",
            "motion_amplitude_m", "motion_period_sec", "motion_phase_rad",
        ]
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        writer.writerows(obstacle_rows)

    layer_rows = [
        {
            "file": STANDARD_OUTPUTS["clean"],
            "role": "base_world",
            "has_axes": True,
            "has_title": False,
            "has_legend": False,
            "visual_mesh_csv": "sources/" + visual_csv,
            "planner_obstacle_csv": "",
            "waypoint_csv": "",
            "notes": "Clean visual base only; no planner obstacles or route points.",
        },
        {
            "file": STANDARD_OUTPUTS["planner"],
            "role": "clean_scene_obstacle_map",
            "has_axes": True,
            "has_title": False,
            "has_legend": True,
            "visual_mesh_csv": "sources/" + visual_csv,
            "planner_obstacle_csv": "sources/" + obstacle_csv,
            "waypoint_csv": "",
            "notes": "Visual base plus current human-facing buoy obstacle footprints.",
        },
        {
            "file": STANDARD_OUTPUTS["route"],
            "role": "route_map",
            "has_axes": True,
            "has_title": False,
            "has_legend": True,
            "visual_mesh_csv": "sources/" + visual_csv,
            "planner_obstacle_csv": "sources/" + obstacle_csv,
            "waypoint_csv": "sources/" + waypoint_csv,
            "notes": "P0-P8 route review on the current runtime scene geometry.",
        },
        {
            "file": STANDARD_OUTPUTS["result"],
            "role": "route_result",
            "has_axes": True,
            "has_title": False,
            "has_legend": True,
            "visual_mesh_csv": "sources/" + visual_csv,
            "planner_obstacle_csv": "sources/" + obstacle_csv,
            "waypoint_csv": "sources/" + waypoint_csv,
            "notes": "P0-P8 planned-vs-flown result using current retained trajectory evidence.",
        },
    ]
    with (SOURCE_DIR / layer_index_csv).open("w", encoding="utf-8", newline="") as fh:
        fields = [
            "file", "role", "has_axes", "has_title", "has_legend",
            "visual_mesh_csv", "planner_obstacle_csv", "waypoint_csv",
            "notes",
        ]
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        writer.writerows(layer_rows)

    meta = {
        "scene_path": project_display_path(SCENE_PATH),
        "coordinate_extent": {"x": list(X_RANGE), "y": list(Y_RANGE)},
        "source": "generated from p0p8_clean_scene sources plus transformed mesh/model geometry",
        "outputs": [
            STANDARD_OUTPUTS["clean"],
            STANDARD_OUTPUTS["planner"],
            STANDARD_OUTPUTS["route"],
            STANDARD_OUTPUTS["result"],
            "sources/" + visual_csv,
            "sources/" + waypoint_csv,
            "sources/" + obstacle_csv,
            "sources/" + layer_index_csv,
        ],
    }
    (SOURCE_DIR / manifest_json).write_text(json.dumps(meta, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    readme = [
        "# P0-P8 Clean Scene Map Authority",
        "",
        "这个目录是地图权威包。`evidence/current` 保存当前 accepted 机器证据；地图图件和最终结果图只放在这里。",
        "",
        "## 怎么重新生成",
        "",
        "- `python3 render_map.py --target base`：只画底图。",
        "- `python3 render_map.py --target obstacle`：画底图和浮标等人看地图需要显示的障碍。",
        "- `python3 render_map.py --target route`：画底图、障碍和规划路线。",
        "- `python3 render_map.py --target result`：再叠加 `evidence/current/route_p0_p8/measurements/actual_trajectory.csv` 里的实际飞行轨迹。",
        "- `python3 render_map.py --target all`：生成全部四张图。",
        "",
        "## 文件说明",
        "",
        "- `render_map.py`：唯一画图入口，同一套图层画底图、障碍图、路线图和结果图。",
        "- `README.md`：说明这个地图包怎么用、每个文件是什么。",
        "- `sources/scene.yaml`：当前真实运行场景定义，记录船、岛、桥、三台风机、浮标、D1 动态小船和障碍配置。",
        "- `sources/route_waypoints.csv`：规划路线点，记录 P0 到 P8 的位置、朝向、半径和停留时间。",
        "- `sources/visual_mesh_footprints.csv`：视觉物体占地范围，告诉画图脚本船、桥、岛、风机在哪里、占多大。",
        "- `sources/planner_obstacles.csv`：规划和安全检查看到的障碍边界，不等同于人眼看到的船外观。",
        "- `sources/layer_index.csv`：每张地图图件用到了哪些数据层。",
        "- `sources/map_manifest.json`：地图包总索引，记录地图范围、来源和输出文件。",
        "- `base_world.png`：第一层图，只画水面、岛、船、桥、风机等视觉场景。",
        "- `obstacle_map.png`：第二层图，在底图上加当前浮标障碍。",
        "- `route_map.png`：第三层图，在障碍图上加规划路线 P0-P8。",
        "- `route_result.png`：第四层图，在路线图上加实际飞行轨迹；轨迹数据从 `evidence/current` 读取，不复制到本目录。",
        "",
        "## 图层规则",
        "",
        "- 共同场景层：水面、视觉物体和 D1/W1-W3 等标号。",
        "- 共同障碍层：人看地图需要显示的浮标障碍；D1 碰撞盒保留在数据里但不画到 PNG。",
        "- 共同路线层：P0-P8 规划路线。",
        "- 结果图只额外叠加实际飞行轨迹。",
        "- 不手工改 PNG；需要变化时改来源数据后重新运行 `render_map.py`。",
    ]
    (OUT_DIR / "README.md").write_text("\n".join(readme) + "\n", encoding="utf-8")



def read_actual_trajectory(current_dir):
    path = Path(current_dir) / "route_p0_p8/measurements/actual_trajectory.csv"
    if not path.is_file():
        raise SystemExit("missing actual trajectory for result map: %s" % path)
    points = []
    with path.open("r", encoding="utf-8", newline="") as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            try:
                points.append((float(row["x"]), float(row["y"])))
            except Exception:
                continue
    if not points:
        raise SystemExit("actual trajectory has no drawable x/y points: %s" % path)
    return points


def save_result_figure(path, current_dir):
    scene = yaml.safe_load(SCENE_PATH.read_text(encoding="utf-8"))
    meshes = [load_visual_mesh(item) for item in scene.get("visual_vessels", []) or [] if item.get("mesh_uri")]
    actual = read_actual_trajectory(current_dir)
    fig, ax = plt.subplots(figsize=(13.0, 8.2), dpi=180)
    setup_ax(ax)
    draw_visual_layer(ax, meshes)
    draw_planner_layer(ax, scene)
    draw_waypoints(ax, scene)
    xs = [p[0] for p in actual]
    ys = [p[1] for p in actual]
    ax.plot(xs, ys, color="#d13f31", linewidth=1.8, alpha=0.96, zorder=42)
    handles = [
        Line2D([0], [0], color="#174ea6", lw=2.25, label="Planned route"),
        Line2D([0], [0], color="#d13f31", lw=1.8, label="Actual flight"),
        Line2D([0], [0], color="#174ea6", marker="o", linestyle="", markersize=5.0,
               markeredgecolor="white", label="Waypoint"),
        Line2D([0], [0], color="#b06a00", marker="o", linestyle="", markersize=5.0,
               markerfacecolor="#f6b84d", label="Buoy obstacle"),
    ]
    ax.legend(handles=handles, loc="upper right", fontsize=7.5, framealpha=0.92)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def selected_targets(target):
    if target == "all":
        return {"base", "obstacle", "route", "result"}
    return {target}

def main():
    import argparse
    parser = argparse.ArgumentParser(description="Render P0-P8 clean-scene map authority figures.")
    parser.add_argument("--target", choices=("base", "obstacle", "route", "result", "all"), default="all")
    parser.add_argument("--current-dir", default=str(ROOT / "evidence/current"))
    args = parser.parse_args()

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    SOURCE_DIR.mkdir(parents=True, exist_ok=True)
    targets = selected_targets(args.target)
    scene = None
    meshes = None

    if "base" in targets:
        scene, meshes = save_figure(OUT_DIR / STANDARD_OUTPUTS["clean"], legend_kind="none")
    if "obstacle" in targets:
        scene, meshes = save_figure(
            OUT_DIR / STANDARD_OUTPUTS["planner"],
            draw_planner=True,
            legend_kind="obstacle",
        )
    if "route" in targets:
        scene, meshes = save_figure(
            OUT_DIR / STANDARD_OUTPUTS["route"],
            draw_planner=True,
            draw_points=True,
            legend_kind="route",
        )
    if "result" in targets:
        save_result_figure(OUT_DIR / STANDARD_OUTPUTS["result"], args.current_dir)
    if scene is None or meshes is None:
        scene = yaml.safe_load(SCENE_PATH.read_text(encoding="utf-8"))
        meshes = [load_visual_mesh(item) for item in scene.get("visual_vessels", []) or [] if item.get("mesh_uri")]
    write_manifest(scene, meshes)
    print(OUT_DIR)


if __name__ == "__main__":
    main()
