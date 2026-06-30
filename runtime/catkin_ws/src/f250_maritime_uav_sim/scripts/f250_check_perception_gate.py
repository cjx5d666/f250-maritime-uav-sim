#!/usr/bin/env python3
import argparse
import json
import math
import os
import sys
import time

import rospy
from rosgraph.masterapi import Master
from sensor_msgs.msg import LaserScan, PointCloud2


PLANNER_TOPIC = "/maritime/obstacles_cloud"
OCCUPANCY_TOPIC = "/grid_map/occupancy_inflate"
LIDAR_SCAN_TOPIC = "/maritime/lidar_scan"
LIDAR_POINTS_TOPIC = "/maritime/lidar_points"
DEPTH_POINTS_TOPIC = "/maritime_depth_camera/points"
LIDAR_ADAPTER_NODE = "/maritime_laser_scan_adapter"
DEPTH_ADAPTER_NODE = "/maritime_sensor_cloud_adapter"


def sensor_label(source):
    return "LiDAR" if source == "lidar" else "Depth"


def expected_topics(source, topics):
    if source == "lidar":
        return [
            (topics["lidar_scan"], LaserScan, "lidar_scan"),
            (topics["lidar_points"], PointCloud2, "lidar_points"),
            (topics["planner"], PointCloud2, "planner_cloud"),
            (topics["occupancy"], PointCloud2, "occupancy_inflate"),
        ]
    if source == "depth":
        return [
            (topics["depth_points"], PointCloud2, "depth_points"),
            (topics["planner"], PointCloud2, "planner_cloud"),
            (topics["occupancy"], PointCloud2, "occupancy_inflate"),
        ]
    raise ValueError("unsupported sensor source: %s" % source)


def message_count(msg):
    if isinstance(msg, LaserScan):
        return len(msg.ranges)
    if isinstance(msg, PointCloud2):
        return int(msg.width * msg.height)
    return None


def finite_scan_count(msg):
    if not isinstance(msg, LaserScan):
        return None
    count = 0
    for value in msg.ranges:
        if math.isfinite(value):
            count += 1
    return count


def wait_for_message(topic, msg_type, deadline):
    last_error = None
    seen_any = False
    while time.time() < deadline and not rospy.is_shutdown():
        timeout = max(0.1, min(1.0, deadline - time.time()))
        try:
            msg = rospy.wait_for_message(topic, msg_type, timeout=timeout)
            seen_any = True
            count = message_count(msg)
            if count is None or count > 0:
                return {
                    "topic": topic,
                    "type": getattr(msg_type, "_type", str(msg_type)),
                    "message_seen": True,
                    "nonempty": True,
                    "count": count,
                    "finite_scan_count": finite_scan_count(msg),
                    "stamp": msg.header.stamp.to_sec() if hasattr(msg, "header") else None,
                }
            last_error = "empty message"
        except Exception as exc:
            last_error = str(exc)
    return {
        "topic": topic,
        "type": getattr(msg_type, "_type", str(msg_type)),
        "message_seen": seen_any,
        "nonempty": False,
        "count": 0,
        "finite_scan_count": None,
        "error": last_error or "timeout",
    }


def master_publishers():
    master = Master("/f250_check_perception_gate")
    pubs, _subs, _srvs = master.getSystemState()
    return {topic: list(nodes) for topic, nodes in pubs}


def topic_types():
    master = Master("/f250_check_perception_gate")
    return {topic: type_name for topic, type_name in master.getPublishedTopics("")}


def adapter_gate(source, publishers, planner_topic):
    planner_publishers = publishers.get(planner_topic, [])
    expected = LIDAR_ADAPTER_NODE if source == "lidar" else DEPTH_ADAPTER_NODE
    forbidden = DEPTH_ADAPTER_NODE if source == "lidar" else LIDAR_ADAPTER_NODE
    expected_topics = sorted(
        topic for topic, nodes in publishers.items()
        if expected in nodes
    )
    forbidden_topics = sorted(
        topic for topic, nodes in publishers.items()
        if forbidden in nodes
    )
    lidar_topics = sorted(
        topic for topic, nodes in publishers.items()
        if LIDAR_ADAPTER_NODE in nodes
    )
    depth_topics = sorted(
        topic for topic, nodes in publishers.items()
        if DEPTH_ADAPTER_NODE in nodes
    )
    expected_active = bool(expected_topics)
    forbidden_active_any = bool(forbidden_topics)
    planner_expected_active = expected in planner_publishers
    planner_forbidden_active = forbidden in planner_publishers
    return {
        "planner_topic": planner_topic,
        "planner_publishers": planner_publishers,
        "expected_publisher": expected,
        "forbidden_publisher": forbidden,
        "expected_publisher_active": planner_expected_active,
        "forbidden_publisher_inactive": not planner_forbidden_active,
        "expected_adapter_active": expected_active,
        "expected_adapter_topics": expected_topics,
        "forbidden_adapter_active_any_topic": forbidden_active_any,
        "forbidden_adapter_topics": forbidden_topics,
        "lidar_adapter_publishing_any_topic": bool(lidar_topics),
        "lidar_adapter_topics": lidar_topics,
        "depth_adapter_publishing_any_topic": bool(depth_topics),
        "depth_adapter_topics": depth_topics,
        "planner_topic_mutual_exclusion_ok": (
            planner_expected_active and not planner_forbidden_active
        ),
        "mutual_exclusion_ok": (
            planner_expected_active and expected_active and not planner_forbidden_active and not forbidden_active_any
        ),
        "policy": "expected adapter must publish planner cloud; forbidden adapter must publish no topics",
    }


def write_json(path, payload):
    if not path:
        return
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")


def main():
    parser = argparse.ArgumentParser(description="Internal F250 live perception gate for route runs.")
    parser.add_argument("--sensor", choices=("lidar", "depth"), required=True)
    parser.add_argument("--timeout-sec", type=float, default=60.0)
    parser.add_argument("--output-json", default="")
    parser.add_argument("--planner-cloud-topic", default=PLANNER_TOPIC)
    parser.add_argument("--occupancy-topic", default=OCCUPANCY_TOPIC)
    parser.add_argument("--lidar-scan-topic", default=LIDAR_SCAN_TOPIC)
    parser.add_argument("--lidar-points-topic", default=LIDAR_POINTS_TOPIC)
    parser.add_argument("--depth-points-topic", default=DEPTH_POINTS_TOPIC)
    args = parser.parse_args()
    topics_config = {
        "planner": args.planner_cloud_topic,
        "occupancy": args.occupancy_topic,
        "lidar_scan": args.lidar_scan_topic,
        "lidar_points": args.lidar_points_topic,
        "depth_points": args.depth_points_topic,
    }

    rospy.init_node("f250_check_perception_gate", anonymous=True, disable_signals=True)
    start = time.time()
    deadline = start + max(1.0, float(args.timeout_sec))
    topics = []
    for topic, msg_type, key in expected_topics(args.sensor, topics_config):
        result = wait_for_message(topic, msg_type, deadline)
        result["key"] = key
        topics.append(result)

    try:
        publishers = master_publishers()
        types = topic_types()
        adapter = adapter_gate(args.sensor, publishers, args.planner_cloud_topic)
        master_error = None
    except Exception as exc:
        publishers = {}
        types = {}
        adapter = {
            "planner_topic": args.planner_cloud_topic,
            "planner_publishers": [],
            "expected_publisher": LIDAR_ADAPTER_NODE if args.sensor == "lidar" else DEPTH_ADAPTER_NODE,
            "forbidden_publisher": DEPTH_ADAPTER_NODE if args.sensor == "lidar" else LIDAR_ADAPTER_NODE,
            "expected_publisher_active": False,
            "forbidden_publisher_inactive": False,
            "expected_adapter_active": False,
            "expected_adapter_topics": [],
            "forbidden_adapter_active_any_topic": None,
            "forbidden_adapter_topics": [],
            "lidar_adapter_publishing_any_topic": False,
            "lidar_adapter_topics": [],
            "depth_adapter_publishing_any_topic": False,
            "depth_adapter_topics": [],
            "planner_topic_mutual_exclusion_ok": False,
            "mutual_exclusion_ok": False,
            "policy": "ROS master query failed; cannot distinguish active forbidden adapter from stale/unavailable master state",
        }
        master_error = str(exc)

    for item in topics:
        item["publisher_nodes"] = publishers.get(item["topic"], [])
        item["published_type"] = types.get(item["topic"])

    ok = all(item["message_seen"] and item["nonempty"] for item in topics) and adapter["mutual_exclusion_ok"]
    payload = {
        "ok": bool(ok),
        "sensor": args.sensor,
        "sensor_label": sensor_label(args.sensor),
        "topics_config": topics_config,
        "checked_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "duration_sec": time.time() - start,
        "timeout_sec": float(args.timeout_sec),
        "topics": topics,
        "adapter_mutual_exclusion": adapter,
        "expected_adapter_active": adapter.get("expected_adapter_active"),
        "forbidden_adapter_active_any_topic": adapter.get("forbidden_adapter_active_any_topic"),
        "mutual_exclusion_ok": adapter["mutual_exclusion_ok"],
        "actual_obstacles_cloud_publishers": adapter["planner_publishers"],
        "occupancy_topic": args.occupancy_topic,
        "master_error": master_error,
    }
    write_json(args.output_json, payload)

    print("sensor=%s" % args.sensor)
    for item in topics:
        print(
            "topic=%s seen=%s count=%s publishers=%s" % (
                item["topic"],
                str(item["message_seen"]).lower(),
                item["count"],
                ",".join(item.get("publisher_nodes") or []),
            )
        )
    print("obstacles_cloud_publishers=%s" % ",".join(adapter["planner_publishers"]))
    print("expected_adapter_active=%s" % str(adapter.get("expected_adapter_active")).lower())
    print("forbidden_adapter_active_any_topic=%s" % str(adapter.get("forbidden_adapter_active_any_topic")).lower())
    print("mutual_exclusion_ok=%s" % str(adapter["mutual_exclusion_ok"]).lower())
    print("ok=%s" % str(ok).lower())
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
