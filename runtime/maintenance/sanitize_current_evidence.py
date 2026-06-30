#!/usr/bin/env python3
import argparse
import os
import re
import shutil
from pathlib import Path

TEXT_SUFFIXES = {'.json', '.env', '.yaml', '.yml', '.csv'}
DROP_DIRS = {'logs', 'live_metric_runs', '__pycache__', '.pytest_cache', 'build', 'devel', 'cache', 'launch_context'}
DROP_SUFFIXES = {'.log', '.pyc', '.png', '.jpg', '.jpeg', '.xwd', '.html', '.htm', '.md', '.txt', '.tmp'}
DROP_FILES = {
    'launch_env.sh', 'launch_in_screen.sh', 'provenance.txt', 'background_worker.env',
    'params.env', 'realtime_metric_live.txt', 'postprocess_status.env', 'route_status.env',
    'source_current_status.env', 'route_profile.json', 'perception_gate.json', 'prealign_yaw.json',
}


def resolve_project_root():
    env_root = os.environ.get('F250_PROJECT_ROOT')
    if env_root:
        return Path(env_root).expanduser().resolve()
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / 'catkin_ws/src/f250_maritime_uav_sim').is_dir():
            return parent
    return Path.cwd().resolve()


def runtime_state_dir(root):
    return Path(os.environ.get('F250_RUNTIME_STATE_DIR', root / 'runtime_state')).expanduser().resolve()


def runtime_work_dir(root):
    return Path(os.environ.get('RUN_ROOT', runtime_state_dir(root) / 'work')).expanduser().resolve()


def current_dir(root):
    return Path(os.environ.get('F250_EVIDENCE_CURRENT_DIR', root / 'evidence/current')).expanduser().resolve()


def should_drop(path):
    if path.name in DROP_FILES:
        return True
    if path.suffix.lower() in DROP_SUFFIXES:
        return True
    return False


def remove_noise(current):
    removed = []
    if not current.exists():
        return removed
    for path in sorted(current.rglob('*'), key=lambda p: len(p.parts), reverse=True):
        if path.is_dir() and path.name in DROP_DIRS:
            shutil.rmtree(path)
            removed.append(str(path))
        elif path.is_file() and should_drop(path):
            path.unlink()
            removed.append(str(path))
    return removed


def replacement_pairs(root, work, current):
    pairs = [
        (str(current), '${EVIDENCE_CURRENT_DIR}'),
        (str(work), '${RUNTIME_WORK_DIR}'),
        (str(root), '${F250_PROJECT_ROOT}'),
    ]
    px4_root = os.environ.get('F250_PX4_ROOT')
    if px4_root:
        pairs.append((str(Path(px4_root).expanduser()), '${F250_PX4_ROOT}'))
    pairs.extend([
        ('/home/adminpc/PX4-Autopilot-v1.16.0-src-main', '${F250_PX4_ROOT}'),
        ('/home/adminpc/catkin_ws', '${CATKIN_WS}'),
        ('${RUNTIME_WORK_DIR}/' + 'ACTIVE_' + 'TASK/status.env', '${RUNTIME_STATE_DIR}/active_task.env'),
        ('${RUNTIME_WORK_DIR}/' + 'ACTIVE_' + 'SENSOR.env', '${RUNTIME_STATE_DIR}/active_sensor.env'),
        ('${RUN_ROOT}', '${RUNTIME_WORK_DIR}'),
        ('${CURRENT_DIR}', '${EVIDENCE_CURRENT_DIR}'),
    ])
    return sorted(pairs, key=lambda item: len(item[0]), reverse=True)


def stabilize_runtime_names(text):
    text = re.sub(r'f250_p0_hover_[^/\s]*\d{8}_\d{6}', 'launch_run', text)
    text = re.sub(r'f250_p0_p8_route_[^/\s]*\d{8}_\d{6}', 'route_run', text)
    text = re.sub(r'f250_fc_3_10_steady_state_[^/\s]*\d{8}_\d{6}', 'flight_control_run', text)
    text = re.sub(r'stop_\d{8}_\d{6}\.log', 'stop_latest.log', text)
    return text


def sanitize_text(current, pairs):
    changed = []
    if not current.exists():
        return changed
    for path in current.rglob('*'):
        if not path.is_file() or path.suffix.lower() not in TEXT_SUFFIXES:
            continue
        try:
            text = path.read_text(encoding='utf-8')
        except UnicodeDecodeError:
            continue
        new = text
        for old, repl in pairs:
            new = new.replace(old, repl)
        new = stabilize_runtime_names(new)
        if new != text:
            path.write_text(new, encoding='utf-8')
            changed.append(str(path))
    return changed


def main():
    parser = argparse.ArgumentParser(description='Remove transient files and machine-local paths from evidence/current.')
    parser.add_argument('--quiet', action='store_true')
    args = parser.parse_args()
    root = resolve_project_root()
    work = runtime_work_dir(root)
    current = current_dir(root)
    removed = remove_noise(current)
    changed = sanitize_text(current, replacement_pairs(root, work, current))
    if not args.quiet:
        print(f'current evidence sanitized: removed={len(removed)} changed={len(changed)}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
