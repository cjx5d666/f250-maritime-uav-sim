#!/usr/bin/env python3
import argparse
import math
import os
import subprocess
import sys
import tempfile
import time


def have_display():
    return bool(os.environ.get("DISPLAY"))


def screen_size():
    try:
        out = subprocess.check_output(["xrandr"], stderr=subprocess.DEVNULL, text=True)
        for line in out.splitlines():
            if " connected" in line and "+" in line:
                for token in line.split():
                    if "x" in token and "+" in token:
                        size = token.split("+")[0]
                        w, h = size.split("x", 1)
                        return int(w), int(h)
    except Exception:
        pass
    return 2358, 1248  # fallback: F250 desktop resolution when xrandr is unavailable


def layout_geometry(width, height):
    # Keep the VM desktop filled without overlapping windows. GNOME Terminal is
    # constrained by its text grid, so the metrics pane uses the nearest stable
    # size that exactly reaches the right and bottom workarea edges on the F250
    # desktop. The visual panes fill the remaining left side.
    left = 72
    top = 27
    screen_right = width
    screen_bottom = height
    metrics_w = 894
    metrics_h = screen_bottom - top
    metrics_x = screen_right - metrics_w
    metrics_y = top
    visual_w = metrics_x - left
    rviz_h = 588
    gazebo_y = top + rviz_h
    gazebo_h = screen_bottom - gazebo_y
    return {
        "rviz": (left, top, visual_w, rviz_h),
        "gazebo": (left, gazebo_y, visual_w, gazebo_h),
        "metrics": (metrics_x, metrics_y, metrics_w, metrics_h),
    }


GAZEBO_ROUTE_CAMERA = {
    "x": 289.316,
    "y": -145.615,
    "z": 12.483,
    "roll": 0.0,
    "pitch": 0.085,
    "yaw": 2.538,
}


def quat_from_rpy(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return {
        "x": sr * cp * cy - cr * sp * sy,
        "y": cr * sp * cy + sr * cp * sy,
        "z": cr * cp * sy - sr * sp * cy,
        "w": cr * cp * cy + sr * sp * sy,
    }


def gazebo_world_name():
    try:
        out = subprocess.check_output(["gz", "topic", "-l"], stderr=subprocess.DEVNULL, text=True, timeout=5)
    except Exception:
        return ""
    prefix = "/gazebo/"
    suffix = "/gui"
    for line in out.splitlines():
        if line.startswith(prefix) and line.endswith(suffix):
            return line[len(prefix):-len(suffix)]
    return ""


def apply_gazebo_camera():
    world = gazebo_world_name()
    if not world:
        print("gazebo_camera_skipped=no_world")
        return False
    pose = dict(GAZEBO_ROUTE_CAMERA)
    q = quat_from_rpy(pose["roll"], pose["pitch"], pose["yaw"])
    message = """fullscreen: false
camera {{
  name: "user_camera"
  view_controller: "orbit"
  pose {{
    position {{ x: {x:.3f} y: {y:.3f} z: {z:.3f} }}
    orientation {{ x: {qx:.6f} y: {qy:.6f} z: {qz:.6f} w: {qw:.6f} }}
  }}
}}
""".format(
        x=pose["x"], y=pose["y"], z=pose["z"],
        qx=q["x"], qy=q["y"], qz=q["z"], qw=q["w"],
    )
    topic = "/gazebo/%s/gui" % world
    camera_file = None
    try:
        with tempfile.NamedTemporaryFile("w", prefix="f250_gazebo_camera_", suffix=".pbtxt", delete=False) as fh:
            fh.write(message)
            camera_file = fh.name
        subprocess.run(
            ["gz", "topic", "-p", topic, "-f", camera_file],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=5,
        )
    except Exception as exc:
        print("gazebo_camera_skipped=%s" % exc)
        return False
    finally:
        if camera_file:
            try:
                os.unlink(camera_file)
            except OSError:
                pass
    print(
        "gazebo_camera=%s pose=(%.3f, %.3f, %.3f, %.3f, %.3f, %.3f)"
        % (world, pose["x"], pose["y"], pose["z"], pose["roll"], pose["pitch"], pose["yaw"])
    )
    return True


def x11_move_resize(xid, x, y, width, height):
    import ctypes
    libx11 = ctypes.cdll.LoadLibrary("libX11.so.6")
    libx11.XOpenDisplay.restype = ctypes.c_void_p
    libx11.XMoveResizeWindow.argtypes = [
        ctypes.c_void_p,
        ctypes.c_ulong,
        ctypes.c_int,
        ctypes.c_int,
        ctypes.c_uint,
        ctypes.c_uint,
    ]
    libx11.XFlush.argtypes = [ctypes.c_void_p]
    display = libx11.XOpenDisplay(None)
    if not display:
        return False
    libx11.XMoveResizeWindow(
        display,
        int(xid),
        int(x),
        int(y),
        int(width),
        int(height),
    )
    libx11.XFlush(display)
    return True


def wnck_layout(kind):
    import gi
    gi.require_version("Gdk", "3.0")
    gi.require_version("Wnck", "3.0")
    from gi.repository import Wnck

    screen = Wnck.Screen.get_default()
    if screen is None:
        return False
    screen.force_update()
    width, height = screen_size()
    geometry = layout_geometry(width, height)
    mask = (Wnck.WindowMoveResizeMask.X | Wnck.WindowMoveResizeMask.Y |
            Wnck.WindowMoveResizeMask.WIDTH | Wnck.WindowMoveResizeMask.HEIGHT)
    moved = []

    def target_for(name):
        if kind in ("all", "visual") and ("RViz" in name or "maritime_visual_acceptance" in name):
            return "rviz", geometry["rviz"]
        if kind in ("all", "visual") and "Gazebo" in name:
            return "gazebo", geometry["gazebo"]
        if kind in ("all", "metrics") and (
            "F250 Route Metrics" in name
            or "F250 FC 3.10 Metrics" in name
        ):
            return "metrics", geometry["metrics"]
        return None, None

    windows = []
    seen_labels = set()
    for window in screen.get_windows():
        name = window.get_name() or ""
        label, target = target_for(name)
        if target and label not in seen_labels:
            seen_labels.add(label)
            windows.append((label, name, window, target))

    if not windows:
        return False

    # Use X11 geometry for the terminal because Wnck honors GNOME Terminal's
    # size increments and can leave a visible strip at the right/bottom edges.
    for label, name, window, target in windows:
        if label != "metrics":
            continue
        try:
            window.unmaximize()
            if not x11_move_resize(window.get_xid(), *target):
                window.set_geometry(Wnck.WindowGravity.CURRENT, mask, *target)
            window.activate(int(time.time()))
            moved.append((label, name, target))
        except Exception as exc:
            print("layout_failed %s %s: %s" % (label, name, exc), file=sys.stderr)

    screen.force_update()

    mx, my, _mw, mh = geometry["metrics"]
    metrics_left = mx
    metrics_bottom = my + mh
    for label, name, window, target in windows:
        if label != "metrics":
            continue
        try:
            geo = window.get_client_window_geometry()
            metrics_left = geo[0]
        except Exception:
            pass

    for label, name, window, target in windows:
        if label == "metrics":
            continue
        try:
            rx, ry, rw, rh = geometry["rviz"]
            gx, gy, gw, gh = geometry["gazebo"]
            if label == "rviz":
                target = (rx, ry, max(0, metrics_left - rx), rh)
            elif label == "gazebo":
                target = (gx, gy, max(0, metrics_left - gx), max(0, metrics_bottom - gy))
            window.unmaximize()
            if not x11_move_resize(window.get_xid(), *target):
                window.set_geometry(Wnck.WindowGravity.CURRENT, mask, *target)
            window.activate(int(time.time()))
            moved.append((label, name, target))
        except Exception as exc:
            print("layout_failed %s %s: %s" % (label, name, exc), file=sys.stderr)

    screen.force_update()
    for label, name, target in moved:
        print("layout_%s=%s %s" % (label, name, target))
    return bool(moved)


def main():
    parser = argparse.ArgumentParser(description="Arrange F250 demo windows on a horizontal desktop.")
    parser.add_argument("--kind", choices=["all", "visual", "metrics", "gazebo-camera"], default="all")
    parser.add_argument("--wait-sec", type=float, default=0.0)
    parser.add_argument("--attempts", type=int, default=1)
    parser.add_argument("--retry-sec", type=float, default=0.5)
    args = parser.parse_args()
    if not have_display():
        print("layout_skipped=no_DISPLAY")
        return 0
    if args.wait_sec > 0:
        time.sleep(args.wait_sec)
    if args.kind == "gazebo-camera":
        apply_gazebo_camera()
    else:
        attempts = max(1, int(args.attempts))
        moved = False
        for attempt in range(attempts):
            try:
                moved = wnck_layout(args.kind)
            except Exception as exc:
                print("layout_skipped=%s" % exc)
                return 0
            if moved:
                break
            if attempt + 1 < attempts:
                time.sleep(max(0.0, float(args.retry_sec)))
        if not moved:
            print("layout_no_matching_windows")
        if args.kind in ("all", "visual"):
            apply_gazebo_camera()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
