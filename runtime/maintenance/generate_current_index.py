#!/usr/bin/env python3
import argparse
import json
import os
from datetime import datetime, timezone
from pathlib import Path


def utc_now():
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def resolve_project_root():
    env_root = os.environ.get("F250_PROJECT_ROOT")
    if env_root:
        return Path(env_root).expanduser().resolve()
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "catkin_ws/src/f250_maritime_uav_sim").is_dir():
            return parent
    return Path.cwd().resolve()


def current_evidence_dir(root):
    return Path(os.environ.get("F250_EVIDENCE_CURRENT_DIR", root / "evidence/current")).expanduser().resolve()


def read_json(path):
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except Exception:
        return {}


def read_env(path):
    data = {}
    try:
        for line in Path(path).read_text(encoding="utf-8").splitlines():
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, value = line.split("=", 1)
            data[key] = value.strip().strip('"').strip("'")
    except FileNotFoundError:
        pass
    return data


def rel(path, base):
    path = Path(path)
    try:
        return str(path.resolve().relative_to(base.resolve()))
    except Exception:
        return str(path)


def task_entry(current, task):
    task_dir = current / task
    manifest_path = task_dir / "manifest.json"
    status_path = task_dir / "status.env"
    manifest = read_json(manifest_path)
    status = read_env(status_path)
    return {
        "available": manifest_path.exists(),
        "manifest": rel(manifest_path, current) if manifest_path.exists() else None,
        "status": rel(status_path, current) if status_path.exists() else None,
        "inputs": rel(task_dir / "inputs", current) if (task_dir / "inputs").is_dir() else None,
        "measurements": rel(task_dir / "measurements", current) if (task_dir / "measurements").is_dir() else None,
        "metrics": rel(task_dir / "metrics", current) if (task_dir / "metrics").is_dir() else None,
        "updated_at": manifest.get("updated_at") or status.get("updated_at"),
        "source_task": manifest.get("source_task"),
        "state": manifest.get("state") or status.get("state"),
        "sensor": manifest.get("sensor") or status.get("sensor") or status.get("perception_source"),
        "outcome": manifest.get("outcome"),
    }


def main():
    parser = argparse.ArgumentParser(description="Update evidence/current/index.json for F250 retained evidence.")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()
    root = resolve_project_root()
    current = current_evidence_dir(root)
    current.mkdir(parents=True, exist_ok=True)
    payload = {
        "schema": "f250_current_evidence_index_v1",
        "updated_at": utc_now(),
        "tasks": {
            "route_p0_p8": task_entry(current, "route_p0_p8"),
            "fc_3_10": task_entry(current, "fc_3_10"),
        },
    }
    target = current / "index.json"
    target.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if not args.quiet:
        print(target)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
