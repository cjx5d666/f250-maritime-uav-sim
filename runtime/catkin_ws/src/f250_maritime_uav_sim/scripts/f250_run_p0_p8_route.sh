#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/f250_paths.sh"
PROJECT_ROOT="$(f250_resolve_project_root "${SCRIPT_DIR}")"
WS="${PROJECT_ROOT}/catkin_ws"
PKG="$(f250_resolve_package_root "${SCRIPT_DIR}" "${PROJECT_ROOT}")"
SCRIPTS="${PKG}/scripts"
SCENE_LEVEL_FIXED="level_m_gps_assets_quick_complex"
SCENE_CONFIG_FIXED="${PKG}/config/scenes/${SCENE_LEVEL_FIXED}.yaml"
WORLD_FIXED="${PKG}/worlds/maritime_${SCENE_LEVEL_FIXED}.world"
MAP_AUTHORITY="${MAP_AUTHORITY:-${PROJECT_ROOT}/map_authority/p0p8_clean_scene}"
f250_apply_quick_complex_defaults "${PKG}"
QUICK_COMPLEX_PROFILE_ID="${F250_QUICK_COMPLEX_PROFILE_ID:-quick_complex_accepted}"
QUICK_COMPLEX_BASELINE="${F250_QUICK_COMPLEX_BASELINE:-${QUICK_COMPLEX_PROFILE_ID}}"
QUICK_COMPLEX_DESCRIPTION="${F250_QUICK_COMPLEX_DESCRIPTION:-F250 quick-complex accepted EGO defaults}"

RECORDER="${SCRIPTS}/f250_quick_complex_record.py"
ROUTE_HELPER="${SCRIPTS}/f250_route_profiles.py"
METRIC_MONITOR="${SCRIPTS}/maritime_metric_monitor.py"
DISPLAY_HELPER="${SCRIPTS}/f250_route_human_summary.py"
START_DEMO="${SCRIPTS}/start_demo_waypoints.sh"
CHECK_GATE="${SCRIPTS}/f250_check_perception_gate.py"
PREALIGN_HELPER="${SCRIPTS}/f250_prealign_yaw.py"
PUBLISH_CURRENT_REVIEW="${PROJECT_ROOT}/maintenance/publish_current_review.py"

RUNTIME_STATE_DIR="${F250_RUNTIME_STATE_DIR:-${PROJECT_ROOT}/runtime_state}"
RUN_ROOT="${RUN_ROOT:-${RUNTIME_STATE_DIR}/work}"
DEFAULT_CURRENT_STATUS="${F250_ACTIVE_TASK_ENV:-${RUNTIME_STATE_DIR}/active_task.env}"
CURRENT_STATUS="${CURRENT_STATUS:-}"
STAMP="$(date +%Y%m%d_%H%M%S)"
SCRIPT_PATH="$(readlink -f "$0")"
PERCEPTION_SOURCE_FROM_ENV="${PERCEPTION_SOURCE:-}"
RAW_CLOUD_TOPIC_FROM_ENV="${RAW_CLOUD_TOPIC:-}"
PERCEPTION_SOURCE="${PERCEPTION_SOURCE_FROM_ENV:-lidar}"
REQUESTED_ROUTE="${F250_ROUTE:-}"
REQUESTED_ROUTE_PROFILE="${F250_ROUTE_PROFILE:-}"

usage() {
	cat <<EOF
Usage:
  ${0} [--dry-run] [--route classic_p0_p8]
  ${0} [--foreground]
  ${0} --help

Runs the selected F250-only maritime_quick_complex route from an already active
route-start hover stack. It releases /maritime/demo/start_waypoints, records the actual
trajectory, replays route metrics offline, and prints only the current formal
human route indicators below:
  3.6 keypoint error
  3.7 route safety
  3.9 final target error

Fixed task inputs:
  vehicle: f250
  route/map/scene: current classic route over the active map authority package
  baseline/defaults: ${QUICK_COMPLEX_DESCRIPTION}
  route acceptance policy: excludes current-scope planning success rate and Metric 3.10 FC
  terminal policy: no yaw pass/fail display
  route metrics: completion, Metric 3.6 keypoint error, Metric 3.7 route safety, Metric 3.9 endpoint error

Default human mode:
  The route worker runs in a background screen/nohup worker and opens a
  separate metrics terminal or screen that follows route_terminal.log. The
  calling terminal prints only a short startup summary. Use --foreground for the
  old blocking behavior.

Useful environment overrides:
  RUN_ROOT=...                  default: ${PROJECT_ROOT}/runtime_state/work
  RUN_LABEL=...                 default: f250_p0_p8_route_<timestamp>
  RUN_DIR=...                   explicit output directory under RUN_ROOT
  CURRENT_STATUS=...            default: ${PROJECT_ROOT}/runtime_state/active_task.env
  F250_ROUTE=classic_p0_p8      optional mismatch guard against current route_id
  ROUTE_MAX_DURATION_SEC=...    default: 360
  F250_ALLOW_RUN_DIR_REUSE=true allow reusing RUN_DIR
  F250_ROUTE_ARGS='...'         extra args passed to recorder
  F250_ROUTE_PREALIGN_YAW=true|false  default: true, align P0 yaw toward P1 before recording
  F250_PREALIGN_YAW_SETTLE_SEC=...    default: 5
  F250_PREALIGN_YAW_STABLE_SEC=...    default: 3
  F250_PREALIGN_YAW_TIMEOUT_SEC=...   default: 15, max wait after settle phase
  F250_PREALIGN_YAW_TOLERANCE_DEG=... default: 1.6
  F250_PUBLISH_CURRENT_REVIEW=true|false default: true for real successful runs
  F250_ROUTE_BACKGROUND=true|false
  F250_OPEN_METRICS_TERMINAL=true|false
  F250_PROJECT_ROOT=...         override project root detected from script path
  MAP_AUTHORITY=...             override authoritative map directory
  F250_QUICK_COMPLEX_DEFAULTS=... optional defaults file override

Dry-run:
  ${0} --dry-run
  Does not require ROS master and writes synthetic successful route outputs.
EOF
}

fail() {
	echo "f250_run_p0_p8_route: $*" >&2
	exit 2
}

normalize_sensor() {
	case "$1" in
	lidar | depth)
		printf "%s\n" "$1"
		;;
	*)
		fail "unsupported perception_source '$1'; expected lidar or depth"
		;;
	esac
}

sensor_label() {
	case "$1" in
	lidar) printf "LiDAR\n" ;;
	depth) printf "Depth\n" ;;
	*) printf "%s\n" "$1" ;;
	esac
}

raw_cloud_topic_for_sensor() {
	case "$1" in
	lidar) printf "/maritime/lidar_points\n" ;;
	depth) printf "/maritime_depth_camera/points\n" ;;
	*) return 2 ;;
	esac
}

env_value() {
	local key="$1"
	local file="$2"
	[ -f "${file}" ] || return 0
	awk -F= -v key="${key}" '$1 == key {sub(/^[^=]*=/, ""); print; exit}' "${file}"
}

canonical_route_id() {
	local route="$1"
	[ -n "${route}" ] || return 0
	python3 "${ROUTE_HELPER}" --route-id "${route}" --base-scene "${SCENE_CONFIG_FIXED}" --print-shell |
		awk -F= '$1 == "route_id" {print $2; exit}'
}

require_path() {
	[ -e "$1" ] || fail "missing required path: $1"
}

write_status() {
	local state="$1"
	local runtime_active="true"
	case "$state" in
	prepared_dry_run | blocked_existing_runtime | blocked_existing_screen | screen_exited_early | perception_gate_failed | prealign_yaw_failed | recorder_failed | postprocess_failed)
		runtime_active="false"
		;;
	esac
	{
		echo "state=${state}"
		echo "task=route"
		echo "runtime_active=${runtime_active}"
		echo "updated_at=$(date -Is)"
		echo "run_dir=${RUN_DIR}"
		echo "run_label=${RUN_LABEL}"
		echo "vehicle=f250"
		echo "dry_run=${DRY_RUN}"
		echo "source_p0_status=${CURRENT_STATUS}"
		echo "source_p0_run_dir=${P0_RUN_DIR}"
		echo "base_scene_config=${SCENE_CONFIG_FIXED}"
		echo "scene_config=${SCENE_CONFIG_SELECTED}"
		echo "world=${WORLD_FIXED}"
		echo "map_authority=${MAP_AUTHORITY}"
		echo "quick_complex_profile_id=${QUICK_COMPLEX_PROFILE_ID}"
		echo "quick_complex_baseline=${QUICK_COMPLEX_BASELINE}"
		echo "route_id=${ROUTE_ID}"
		echo "route_name=${ROUTE_NAME}"
		echo "route_profile=${ROUTE_PROFILE}"
		echo "route_profile_source=${ROUTE_PROFILE_SOURCE}"
		echo "route_effective_scene=${ROUTE_EFFECTIVE_SCENE}"
		echo "route_metadata_json=${ROUTE_METADATA_JSON}"
		echo "route_waypoints_csv=${ROUTE_WAYPOINTS_CSV}"
		echo "route_waypoint_count=${ROUTE_WAYPOINT_COUNT}"
		echo "route_first_label=${ROUTE_FIRST_LABEL}"
		echo "route_final_label=${ROUTE_FINAL_LABEL}"
		echo "route_total_length_m=${ROUTE_TOTAL_LENGTH_M}"
		echo "prealign_yaw_json=${PREALIGN_JSON}"
		echo "prealign_yaw_log=${PREALIGN_LOG}"
		echo "prealign_yaw_policy=settle_sec_${F250_PREALIGN_YAW_SETTLE_SEC:-5}_stable_sec_${F250_PREALIGN_YAW_STABLE_SEC:-3}_tolerance_deg_${F250_PREALIGN_YAW_TOLERANCE_DEG:-1.6}_timeout_sec_${F250_PREALIGN_YAW_TIMEOUT_SEC:-15}_warning_only"
		echo "actual_trajectory_csv=${TRAJECTORY_CSV}"
		echo "summary_json=${SUMMARY_JSON}"
		echo "metrics_json=${METRICS_JSON}"
		echo "metric_summary_json=${METRIC_SUMMARY_JSON}"
		echo "metric_waypoints_csv=${METRIC_WAYPOINTS_CSV}"
		echo "route_terminal_log=${ROUTE_TERMINAL_LOG}"
		echo "route_status_env=${ROUTE_STATUS_ENV}"
		echo "prealign_yaw_json=${PREALIGN_JSON}"
		echo "prealign_yaw_log=${PREALIGN_LOG}"
		echo "prealign_yaw_enabled=${F250_ROUTE_PREALIGN_YAW:-true}"
		echo "route_acceptance_excludes_planning_success_rate=true"
		echo "route_acceptance_excludes_metric_3_10=true"
		echo "route_acceptance_excludes_yaw=true"
		echo "dynamic_boat_clearance_role=telemetry_only"
		echo "sensor=${PERCEPTION_SOURCE}"
		echo "sensor_label=${SENSOR_LABEL}"
		echo "perception_source=${PERCEPTION_SOURCE}"
		echo "planner_cloud_topic=${PLANNER_CLOUD_TOPIC}"
		echo "raw_cloud_topic=${RAW_CLOUD_TOPIC}"
		echo "lidar_cloud_topic=${LIDAR_CLOUD_TOPIC}"
		echo "lidar_scan_topic=${LIDAR_SCAN_TOPIC}"
		echo "depth_cloud_topic=${DEPTH_CLOUD_TOPIC}"
		echo "occupancy_topic=${OCCUPANCY_TOPIC}"
		echo "perception_gate_json=${GATE_JSON}"
		echo "perception_gate_log=${GATE_LOG}"
	} >"${STATUS_FILE}"
	if [ -n "${CURRENT_STATUS:-}" ] && [ "${CURRENT_STATUS}" != "${STATUS_FILE}" ]; then
		mkdir -p "$(dirname "${CURRENT_STATUS}")"
		cp "${STATUS_FILE}" "${CURRENT_STATUS}"
	fi
}

write_params_json() {
	python3 - "${PARAMS_JSON}" "${SCENE_CONFIG_SELECTED}" "${ROUTE_METADATA_JSON}" <<'PY'
import json
import os
import sys

path, scene_config, route_metadata_json = sys.argv[1], sys.argv[2], sys.argv[3]
route = {}
if route_metadata_json and os.path.exists(route_metadata_json):
    with open(route_metadata_json, "r", encoding="utf-8") as handle:
        route = json.load(handle)
def env_float(name, default):
    value = os.environ.get(name, str(default))
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)

profile_id = os.environ.get("F250_QUICK_COMPLEX_PROFILE_ID", "quick_complex_accepted")
baseline = os.environ.get("F250_QUICK_COMPLEX_BASELINE", profile_id)
description = os.environ.get("F250_QUICK_COMPLEX_DESCRIPTION", "F250 quick-complex accepted EGO defaults")
payload = {
  "description": description,
  "vehicle": "f250",
  "profile_id": profile_id,
  "family_id": profile_id,
  "baseline": baseline,
  "scene_level": "level_m_gps_assets_quick_complex",
  "scene_config": os.path.abspath(scene_config),
  "base_scene_config": os.environ.get("SCENE_CONFIG_FIXED", ""),
  "route": route,
  "route_id": os.environ.get("ROUTE_ID", ""),
  "route_name": os.environ.get("ROUTE_NAME", ""),
  "route_profile": os.environ.get("ROUTE_PROFILE", ""),
  "route_profile_source": os.environ.get("ROUTE_PROFILE_SOURCE", ""),
  "route_waypoint_count": int(os.environ.get("ROUTE_WAYPOINT_COUNT", "0") or 0),
  "route_final_label": os.environ.get("ROUTE_FINAL_LABEL", ""),
  "sensor": os.environ.get("PERCEPTION_SOURCE", "lidar"),
  "sensor_label": os.environ.get("SENSOR_LABEL", "LiDAR"),
  "perception_source": os.environ.get("PERCEPTION_SOURCE", "lidar"),
  "dynamic_mode": "auto",
  "topics": {
	    "planner_cloud_topic": os.environ.get("PLANNER_CLOUD_TOPIC", "/maritime/obstacles_cloud"),
	    "raw_cloud_topic": os.environ.get("RAW_CLOUD_TOPIC", "/maritime/lidar_points"),
	    "lidar_cloud_topic": os.environ.get("LIDAR_CLOUD_TOPIC", "/maritime/lidar_points"),
	    "lidar_scan_topic": os.environ.get("LIDAR_SCAN_TOPIC", "/maritime/lidar_scan"),
    "depth_cloud_topic": os.environ.get("DEPTH_CLOUD_TOPIC", "/maritime_depth_camera/points"),
    "occupancy_topic": os.environ.get("OCCUPANCY_TOPIC", "/grid_map/occupancy_inflate")
  },
  "params": {
    "map_size_x": env_float("MAP_SIZE_X", 760.0),
    "map_size_y": env_float("MAP_SIZE_Y", 320.0),
    "map_size_z": env_float("MAP_SIZE_Z", 18.0),
    "max_vel": env_float("EGO_MAX_VEL", 3.55),
    "max_acc": env_float("EGO_MAX_ACC", 4.90),
    "max_jerk": env_float("EGO_MAX_JERK", 6.3),
    "control_points_distance": env_float("EGO_CONTROL_POINTS_DISTANCE", 0.35),
    "feasibility_tolerance": env_float("EGO_FEASIBILITY_TOLERANCE", 0.0),
    "planning_horizon": env_float("EGO_PLANNING_HORIZON", 15.0),
    "local_update_range_x": env_float("EGO_LOCAL_UPDATE_RANGE_X", 40.0),
    "local_update_range_y": env_float("EGO_LOCAL_UPDATE_RANGE_Y", 40.0),
    "local_update_range_z": env_float("EGO_LOCAL_UPDATE_RANGE_Z", 9.0),
    "virtual_ceil_height": env_float("EGO_VIRTUAL_CEIL_HEIGHT", 17.0),
    "visualization_truncate_height": env_float("EGO_VISUALIZATION_TRUNCATE_HEIGHT", 18.0),
    "obstacles_inflation": env_float("EGO_OBSTACLES_INFLATION", 0.50),
    "collision_dist0": env_float("EGO_COLLISION_DIST0", 1.25),
    "lambda_smooth": env_float("EGO_LAMBDA_SMOOTH", 1.40),
    "lambda_collision": env_float("EGO_LAMBDA_COLLISION", 6.0),
    "lambda_feasibility": env_float("EGO_LAMBDA_FEASIBILITY", 0.15),
    "lambda_fitness": env_float("EGO_LAMBDA_FITNESS", 1.35),
    "grid_map_resolution": env_float("EGO_GRID_MAP_RESOLUTION", 0.35)
  },
  "route_acceptance_policy": {
    "policy_id": "current_route_acceptance_policy",
    "route_acceptance_excludes_planning_success_rate": True,
    "route_acceptance_excludes_metric_3_10": True,
    "route_acceptance_excludes_yaw": True,
    "dynamic_boat_clearance_role": "telemetry_only"
  }
}
os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
with open(path, "w", encoding="utf-8") as handle:
    json.dump(payload, handle, indent=2, sort_keys=True)
    handle.write("\n")
PY
}

ensure_ros_available() {
	if ! command -v rostopic >/dev/null 2>&1; then
		fail "rostopic is unavailable; source ROS or use --dry-run"
	fi
	if ! rostopic list >/dev/null 2>&1; then
		fail "ROS master is unavailable; start f250_start_to_p0_hover.sh first or use --dry-run"
	fi
	if ! rostopic list | grep -qx "/mavros/local_position/odom"; then
		fail "missing /mavros/local_position/odom; P0 hover stack does not look active"
	fi
	# /maritime/active_goal is only published after start_demo_waypoints.sh
	# releases the paused goal sequence, so do not block route startup on it here.
}

run_perception_gate() {
	if [ "${F250_SKIP_PERCEPTION_GATE:-false}" = "true" ]; then
		return 0
	fi
	local gate_lidar_points="${LIDAR_CLOUD_TOPIC:-/maritime/lidar_points}"
	local gate_depth_points="${DEPTH_CLOUD_TOPIC:-/maritime_depth_camera/points}"
	if [ "${PERCEPTION_SOURCE}" = "lidar" ]; then
		gate_lidar_points="${RAW_CLOUD_TOPIC}"
	elif [ "${PERCEPTION_SOURCE}" = "depth" ]; then
		gate_depth_points="${RAW_CLOUD_TOPIC}"
	fi
	write_status "perception_gate"
	set +e
	python3 "${CHECK_GATE}" \
		--sensor "${PERCEPTION_SOURCE}" \
		--timeout-sec "${F250_PERCEPTION_GATE_TIMEOUT_SEC:-60}" \
		--output-json "${GATE_JSON}" \
		--planner-cloud-topic "${PLANNER_CLOUD_TOPIC}" \
		--occupancy-topic "${OCCUPANCY_TOPIC}" \
		--lidar-scan-topic "${LIDAR_SCAN_TOPIC}" \
		--lidar-points-topic "${gate_lidar_points}" \
		--depth-points-topic "${gate_depth_points}" \
		>"${GATE_LOG}" 2>&1
	local gate_status=$?
	set -e
	if [ "${gate_status}" -ne 0 ]; then
		write_status "perception_gate_failed"
		echo "[f250-route] perception gate failed for ${PERCEPTION_SOURCE}; see ${GATE_LOG}" >&2
		exit "${gate_status}"
	fi
	write_status "perception_gate_ok"
}

run_route_prealign_yaw() {
	[ "${F250_ROUTE_PREALIGN_YAW:-true}" = "true" ] || {
		printf '{"schema":"f250_prealign_yaw_v1","status":"skipped","aligned":false,"warning":false}\n' >"${PREALIGN_JSON}"
		printf "status=skipped\n" >"${PREALIGN_LOG}"
		return 0
	}
	write_status "prealign_yaw"
	set +e
	python3 "${PREALIGN_HELPER}" \
		--route-waypoints-csv "${ROUTE_WAYPOINTS_CSV}" \
		--output-json "${PREALIGN_JSON}" \
		--log "${PREALIGN_LOG}" \
		--settle-sec "${F250_PREALIGN_YAW_SETTLE_SEC:-5}" \
		--stable-sec "${F250_PREALIGN_YAW_STABLE_SEC:-3}" \
		--timeout-sec "${F250_PREALIGN_YAW_TIMEOUT_SEC:-15}" \
		--tolerance-deg "${F250_PREALIGN_YAW_TOLERANCE_DEG:-1.6}" \
		>"${PREALIGN_STDOUT_LOG}" 2>&1
	local prealign_status=$?
	set -e
	if [ "${prealign_status}" -ne 0 ]; then
		write_status "prealign_yaw_failed"
		echo "[f250-route] prealign yaw command chain failed status=${prealign_status}; see ${PREALIGN_LOG}" >&2
		exit "${prealign_status}"
	fi
	if python3 - "${PREALIGN_JSON}" <<'PY' >/dev/null 2>&1; then
import json, sys
with open(sys.argv[1], "r", encoding="utf-8") as handle:
    data = json.load(handle)
raise SystemExit(0 if data.get("aligned") else 1)
PY
		echo "[f250-route] prealign yaw OK; continuing." >>"${ROUTE_TERMINAL_LOG}" 2>/dev/null || true
	else
		echo "[f250-route] WARNING prealign yaw outside tolerance; continuing by policy. Details: prealign_yaw.json" >>"${ROUTE_TERMINAL_LOG}" 2>/dev/null || true
	fi
	write_status "prealign_yaw_done"
}

stop_pid() {
	local pid="$1"
	[ -n "${pid}" ] || return 0
	if kill -0 "${pid}" >/dev/null 2>&1; then
		kill -TERM "${pid}" >/dev/null 2>&1 || true
		local waited=0
		while kill -0 "${pid}" >/dev/null 2>&1 && [ "${waited}" -lt 30 ]; do
			sleep 0.2
			waited=$((waited + 1))
		done
		if kill -0 "${pid}" >/dev/null 2>&1; then
			kill -KILL "${pid}" >/dev/null 2>&1 || true
		fi
	fi
	wait "${pid}" >/dev/null 2>&1 || true
}

terminal_command() {
	local logfile="$1"
	local heading="${2:-F250 Route Metrics}"
	local py_filter
	py_filter='
import re
import sys
for raw in iter(sys.stdin.readline, ""):
    line = raw.rstrip("\n")
    if re.match(r"^=+", line):
        line = "\033[1;36m%s\033[0m" % line
    elif re.match(r"^\[[^]]+\]", line):
        line = "\033[1;33m%s\033[0m" % line
    elif "FAIL" in line:
        line = line.replace("FAIL", "\033[1;31mFAIL\033[0m")
    elif "PASS" in line:
        line = line.replace("PASS", "\033[1;32mPASS\033[0m")
    elif re.search(r"不安全|未达到|failed", line):
        line = "\033[1;31m%s\033[0m" % line
    elif re.search(r"安全|已达到", line):
        line = "\033[1;32m%s\033[0m" % line
    elif re.search(r"任务状态|说明", line):
        line = "\033[1;37m%s\033[0m" % line
    print(line, flush=True)
'
	printf "printf %%b %q; touch %q; tail -n +1 -F %q | python3 -u -c %q" "\\033[1;36m${heading}\\033[0m\\n" "${logfile}" "${logfile}" "${py_filter}"
}

adopt_desktop_env() {
	[ -n "${DISPLAY:-}" ] && return 0
	local pid key value
	for pattern in "[f]250_control_panel.py" "[r]viz" "[g]zclient" "[g]nome-shell"; do
		pid="$(pgrep -n -f "${pattern}" 2>/dev/null || true)"
		[ -n "${pid}" ] || continue
		[ -r "/proc/${pid}/environ" ] || continue
		while IFS='=' read -r key value; do
			case "${key}" in
			DISPLAY | DBUS_SESSION_BUS_ADDRESS | XAUTHORITY)
				export "${key}=${value}"
				;;
			esac
		done < <(tr '\0' '\n' <"/proc/${pid}/environ")
		[ -n "${DISPLAY:-}" ] && return 0
	done
	return 0
}

append_terminal_over() {
	[ -n "${ROUTE_TERMINAL_LOG:-}" ] || return 0
	mkdir -p "$(dirname "${ROUTE_TERMINAL_LOG}")"
	if [ -f "${ROUTE_TERMINAL_LOG}" ] && [ "$(tail -n 1 "${ROUTE_TERMINAL_LOG}" 2>/dev/null || true)" = "OVER" ]; then
		return 0
	fi
	printf "\nOVER\n" >>"${ROUTE_TERMINAL_LOG}"
}

publish_current_review() {
	[ "${F250_PUBLISH_CURRENT_REVIEW:-true}" = "true" ] || return 0
	[ "${DRY_RUN}" != "true" ] || return 0
	if [ ! -f "${PUBLISH_CURRENT_REVIEW}" ]; then
		echo "publish_current_review=skipped_missing_tool:${PUBLISH_CURRENT_REVIEW}" >>"${RUN_DIR}/postprocess_status.env"
		return 0
	fi
	if [ -z "${P0_RUN_DIR:-}" ] || [ ! -d "${P0_RUN_DIR}" ]; then
		echo "publish_current_review=skipped_missing_p0_run:${P0_RUN_DIR:-}" >>"${RUN_DIR}/postprocess_status.env"
		return 0
	fi
	set +e
	python3 "${PUBLISH_CURRENT_REVIEW}" \
		--kind route \
		--run-dir "${RUN_DIR}" \
		--p0-run-dir "${P0_RUN_DIR}" \
		--quiet \
		>"${RUN_DIR}/logs/publish_current_review.log" 2>&1
	local publish_status=$?
	set -e
	if [ "${publish_status}" -eq 0 ]; then
		echo "publish_current_review=ok" >>"${RUN_DIR}/postprocess_status.env"
	else
		echo "publish_current_review=warning_status_${publish_status}" >>"${RUN_DIR}/postprocess_status.env"
		echo "[f250-route] WARNING current review publish failed; see ${RUN_DIR}/logs/publish_current_review.log" >&2
	fi
	return 0
}

open_metrics_terminal() {
	local title="$1"
	local logfile="$2"
	local screen_name="$3"
	[ "${F250_OPEN_METRICS_TERMINAL:-true}" = "true" ] || return 0
	local cmd
	cmd="$(terminal_command "${logfile}" "${title}")"
	adopt_desktop_env
	if [ -n "${DISPLAY:-}" ] && command -v gnome-terminal >/dev/null 2>&1; then
		nohup gnome-terminal --title="${title}" -- bash -lc "${cmd}" >/dev/null 2>&1 &
		echo "metrics_terminal=${title}"
	elif [ -n "${DISPLAY:-}" ] && command -v x-terminal-emulator >/dev/null 2>&1; then
		nohup x-terminal-emulator -T "${title}" -e bash -lc "${cmd}" >/dev/null 2>&1 &
		echo "metrics_terminal=${title}"
	elif [ -n "${DISPLAY:-}" ] && command -v xterm >/dev/null 2>&1; then
		nohup xterm -T "${title}" -e bash -lc "${cmd}" >/dev/null 2>&1 &
		echo "metrics_terminal=${title}"
	elif command -v screen >/dev/null 2>&1; then
		screen -dmS "${screen_name}" bash -lc "${cmd}"
		echo "metrics_screen=${screen_name}"
		echo "attach_metrics=screen -r ${screen_name}"
	else
		echo "metrics_terminal_unavailable=true"
		echo "manual_metrics=tail -n +1 -F ${logfile}"
	fi
}

layout_metrics_window() {
	[ "${F250_AUTO_LAYOUT:-true}" = "true" ] || return 0
	(DISPLAY="${DISPLAY:-:0}" "${SCRIPTS}/f250_layout_windows.py" --kind metrics --wait-sec "${F250_METRICS_LAYOUT_WAIT_SEC:-1}" --attempts "${F250_METRICS_LAYOUT_ATTEMPTS:-6}" --retry-sec "${F250_METRICS_LAYOUT_RETRY_SEC:-0.5}" >>"${RUN_DIR}/logs/window_layout.log" 2>&1 || true) &
}

launch_route_worker() {
	mkdir -p "${RUN_DIR}/logs"
	local worker_screen="f250_route_worker_${RUN_LABEL}"
	local metrics_screen="f250_route_metrics_${RUN_LABEL}"
	local worker_log="${RUN_DIR}/logs/route_worker.log"
	local metrics_info
	metrics_info="$(open_metrics_terminal "F250 Route Metrics" "${ROUTE_TERMINAL_LOG}" "${metrics_screen}")"
	layout_metrics_window
	local worker_info=""
	local worker_pid=""
	local worker_used_screen="false"
	if [ "${F250_ROUTE_WORKER_SCREEN:-false}" = "true" ] && command -v screen >/dev/null 2>&1; then
		worker_used_screen="true"
		screen -dmS "${worker_screen}" -L -Logfile "${worker_log}" env \
			F250_ROUTE_BACKGROUND=false \
			F250_OPEN_METRICS_TERMINAL=false \
			RUN_ROOT="${RUN_ROOT}" \
			RUN_DIR="${RUN_DIR}" \
			RUN_LABEL="${RUN_LABEL}" \
			CURRENT_STATUS="${CURRENT_STATUS}" \
			SOURCE_CURRENT_STATUS="${SOURCE_STATUS_COPY}" \
			ROUTE_MAX_DURATION_SEC="${ROUTE_MAX_DURATION_SEC}" \
			F250_ROUTE_ARGS="${F250_ROUTE_ARGS:-}" \
			F250_ROUTE="${ROUTE_ID}" \
			ROUTE_ID="${ROUTE_ID}" \
			ROUTE_NAME="${ROUTE_NAME}" \
			ROUTE_PROFILE="${ROUTE_PROFILE}" \
			ROUTE_PROFILE_SOURCE="${ROUTE_PROFILE_SOURCE}" \
			ROUTE_EFFECTIVE_SCENE="${ROUTE_EFFECTIVE_SCENE}" \
			ROUTE_METADATA_JSON="" \
			ROUTE_WAYPOINTS_CSV="" \
			ROUTE_WAYPOINT_COUNT="${ROUTE_WAYPOINT_COUNT}" \
			ROUTE_FIRST_LABEL="${ROUTE_FIRST_LABEL}" \
			ROUTE_FINAL_LABEL="${ROUTE_FINAL_LABEL}" \
			ROUTE_TOTAL_LENGTH_M="${ROUTE_TOTAL_LENGTH_M}" \
			F250_ROUTE_RECORD_PRESTART_SEC="${F250_ROUTE_RECORD_PRESTART_SEC:-}" \
			F250_ROUTE_PREALIGN_YAW="${F250_ROUTE_PREALIGN_YAW:-true}" \
			F250_PREALIGN_YAW_SETTLE_SEC="${F250_PREALIGN_YAW_SETTLE_SEC:-5}" \
			F250_PREALIGN_YAW_STABLE_SEC="${F250_PREALIGN_YAW_STABLE_SEC:-3}" \
			F250_PREALIGN_YAW_TIMEOUT_SEC="${F250_PREALIGN_YAW_TIMEOUT_SEC:-15}" \
			F250_PREALIGN_YAW_TOLERANCE_DEG="${F250_PREALIGN_YAW_TOLERANCE_DEG:-1.6}" \
			PERCEPTION_SOURCE="${PERCEPTION_SOURCE}" \
			SENSOR_LABEL="${SENSOR_LABEL}" \
			RAW_CLOUD_TOPIC="${RAW_CLOUD_TOPIC}" \
			PLANNER_CLOUD_TOPIC="${PLANNER_CLOUD_TOPIC}" \
			LIDAR_CLOUD_TOPIC="${LIDAR_CLOUD_TOPIC}" \
			LIDAR_SCAN_TOPIC="${LIDAR_SCAN_TOPIC}" \
			DEPTH_CLOUD_TOPIC="${DEPTH_CLOUD_TOPIC}" \
			OCCUPANCY_TOPIC="${OCCUPANCY_TOPIC}" \
			F250_ALLOW_RUN_DIR_REUSE=true \
			F250_SKIP_PERCEPTION_GATE=true \
			F250_ROUTE_TERMINAL_LOG_READY=true \
			bash "${SCRIPT_PATH}" --foreground --run-dir "${RUN_DIR}" --current-status "${CURRENT_STATUS}"
		worker_info="worker_screen=${worker_screen}"
	else
		nohup env \
			F250_ROUTE_BACKGROUND=false \
			F250_OPEN_METRICS_TERMINAL=false \
			RUN_ROOT="${RUN_ROOT}" \
			RUN_DIR="${RUN_DIR}" \
			RUN_LABEL="${RUN_LABEL}" \
			CURRENT_STATUS="${CURRENT_STATUS}" \
			SOURCE_CURRENT_STATUS="${SOURCE_STATUS_COPY}" \
			ROUTE_MAX_DURATION_SEC="${ROUTE_MAX_DURATION_SEC}" \
			F250_ROUTE_ARGS="${F250_ROUTE_ARGS:-}" \
			F250_ROUTE="${ROUTE_ID}" \
			ROUTE_ID="${ROUTE_ID}" \
			ROUTE_NAME="${ROUTE_NAME}" \
			ROUTE_PROFILE="${ROUTE_PROFILE}" \
			ROUTE_PROFILE_SOURCE="${ROUTE_PROFILE_SOURCE}" \
			ROUTE_EFFECTIVE_SCENE="${ROUTE_EFFECTIVE_SCENE}" \
			ROUTE_METADATA_JSON="" \
			ROUTE_WAYPOINTS_CSV="" \
			ROUTE_WAYPOINT_COUNT="${ROUTE_WAYPOINT_COUNT}" \
			ROUTE_FIRST_LABEL="${ROUTE_FIRST_LABEL}" \
			ROUTE_FINAL_LABEL="${ROUTE_FINAL_LABEL}" \
			ROUTE_TOTAL_LENGTH_M="${ROUTE_TOTAL_LENGTH_M}" \
			F250_ROUTE_RECORD_PRESTART_SEC="${F250_ROUTE_RECORD_PRESTART_SEC:-}" \
			F250_ROUTE_PREALIGN_YAW="${F250_ROUTE_PREALIGN_YAW:-true}" \
			F250_PREALIGN_YAW_SETTLE_SEC="${F250_PREALIGN_YAW_SETTLE_SEC:-5}" \
			F250_PREALIGN_YAW_STABLE_SEC="${F250_PREALIGN_YAW_STABLE_SEC:-3}" \
			F250_PREALIGN_YAW_TIMEOUT_SEC="${F250_PREALIGN_YAW_TIMEOUT_SEC:-15}" \
			F250_PREALIGN_YAW_TOLERANCE_DEG="${F250_PREALIGN_YAW_TOLERANCE_DEG:-1.6}" \
			PERCEPTION_SOURCE="${PERCEPTION_SOURCE}" \
			SENSOR_LABEL="${SENSOR_LABEL}" \
			RAW_CLOUD_TOPIC="${RAW_CLOUD_TOPIC}" \
			PLANNER_CLOUD_TOPIC="${PLANNER_CLOUD_TOPIC}" \
			LIDAR_CLOUD_TOPIC="${LIDAR_CLOUD_TOPIC}" \
			LIDAR_SCAN_TOPIC="${LIDAR_SCAN_TOPIC}" \
			DEPTH_CLOUD_TOPIC="${DEPTH_CLOUD_TOPIC}" \
			OCCUPANCY_TOPIC="${OCCUPANCY_TOPIC}" \
			F250_ALLOW_RUN_DIR_REUSE=true \
			F250_SKIP_PERCEPTION_GATE=true \
			F250_ROUTE_TERMINAL_LOG_READY=true \
			bash "${SCRIPT_PATH}" --foreground --run-dir "${RUN_DIR}" --current-status "${CURRENT_STATUS}" \
			>"${worker_log}" 2>&1 &
		worker_pid="$!"
		worker_info="worker_pid=${worker_pid}"
	fi
	{
		printf "%s\n" "${metrics_info}"
		echo "${worker_info}"
		echo "worker_log=${worker_log}"
		echo "run_dir=${RUN_DIR}"
		echo "route_status_env=${ROUTE_STATUS_ENV}"
		echo "route_terminal_log=${ROUTE_TERMINAL_LOG}"
		echo "perception_gate_json=${GATE_JSON}"
		echo "perception_gate_log=${GATE_LOG}"
		echo "stop_script=${SCRIPTS}/f250_stop_all.sh"
	} >"${RUN_DIR}/logs/background_worker.env"
	# Worker liveness self-check: a --foreground worker that dies immediately must
	# not leave the operator hanging at background_worker_starting with no signal.
	sleep 3
	local worker_alive="false"
	if [ "${worker_used_screen}" = "true" ]; then
		screen -ls 2>/dev/null | grep -q "${worker_screen}" && worker_alive="true"
	elif [ -n "${worker_pid}" ]; then
		kill -0 "${worker_pid}" 2>/dev/null && worker_alive="true"
	fi
	if [ "${worker_alive}" != "true" ]; then
		write_status "failed"
		{
			echo "route_worker_died_immediately=true"
			echo "worker_log=${worker_log}"
		} >>"${STATUS_FILE}"
		echo "F250 route worker exited immediately; see ${worker_log}" >&2
		return 1
	fi
	cat <<EOF
F250 route started.
Metrics window: F250 Route Metrics
Stop: use Stop All
EOF
	if printf "%s\n" "${metrics_info}" | grep -q "manual_metrics"; then
		echo "Metrics window unavailable; use Stop All or inspect current status."
	fi
}

DRY_RUN="${F250_ROUTE_DRY_RUN:-false}"
RUN_IN_BACKGROUND="${F250_ROUTE_BACKGROUND:-true}"
RUN_LABEL="${RUN_LABEL:-${RUN_LABEL_OVERRIDE:-f250_p0_p8_route_${STAMP}}}"
RUN_DIR="${RUN_DIR:-}"

while [ "$#" -gt 0 ]; do
	case "$1" in
	--help | -h)
		usage
		exit 0
		;;
	--dry-run)
		DRY_RUN="true"
		shift
		;;
	--foreground)
		RUN_IN_BACKGROUND="false"
		shift
		;;
	--run-label)
		RUN_LABEL="$2"
		shift 2
		;;
	--run-dir)
		RUN_DIR="$2"
		RUN_LABEL="$(basename "$2")"
		shift 2
		;;
	--current-status)
		CURRENT_STATUS="$2"
		shift 2
		;;
	--route)
		[ "$#" -ge 2 ] || fail "--route requires ROUTE_ID"
		REQUESTED_ROUTE="$2"
		shift 2
		;;
	--route=*)
		REQUESTED_ROUTE="${1#--route=}"
		shift
		;;
	--route-profile | --custom-route)
		[ "$#" -ge 2 ] || fail "$1 requires a YAML path"
		REQUESTED_ROUTE_PROFILE="$2"
		shift 2
		;;
	--route-profile=* | --custom-route=*)
		REQUESTED_ROUTE_PROFILE="${1#*=}"
		shift
		;;
	*)
		fail "unknown argument: $1; run ${0} --help"
		;;
	esac
done

require_path "${WS}"
require_path "${PKG}"
require_path "${SCENE_CONFIG_FIXED}"
require_path "${WORLD_FIXED}"
require_path "${MAP_AUTHORITY}"
require_path "${RECORDER}"
require_path "${ROUTE_HELPER}"
require_path "${METRIC_MONITOR}"
require_path "${DISPLAY_HELPER}"
require_path "${START_DEMO}"
require_path "${CHECK_GATE}"
require_path "${PREALIGN_HELPER}"

mkdir -p "${RUNTIME_STATE_DIR}" "${RUN_ROOT}"
RUNTIME_STATE_DIR="$(cd "${RUNTIME_STATE_DIR}" && pwd -P)"
RUN_ROOT="$(cd "${RUN_ROOT}" && pwd -P)"
DEFAULT_CURRENT_STATUS="${F250_ACTIVE_TASK_ENV:-${RUNTIME_STATE_DIR}/active_task.env}"
if [ -z "${CURRENT_STATUS:-}" ]; then
	CURRENT_STATUS="${DEFAULT_CURRENT_STATUS}"
fi

if [ -z "${RUN_DIR}" ]; then
	RUN_DIR="${RUN_ROOT}/${RUN_LABEL}"
else
	case "${RUN_DIR}" in
	/*) ;;
	*) RUN_DIR="${RUN_ROOT}/${RUN_DIR}" ;;
	esac
fi
RUN_DIR_PARENT="$(dirname "${RUN_DIR}")"
mkdir -p "${RUN_DIR_PARENT}"
RUN_DIR="${RUN_DIR_PARENT}/$(basename "${RUN_DIR}")"

case "${RUN_DIR}" in
"${RUN_ROOT}"/*) ;;
*) fail "RUN_DIR must stay under RUN_ROOT=${RUN_ROOT}: ${RUN_DIR}" ;;
esac

if [[ ! "${RUN_LABEL}" =~ ^[A-Za-z0-9_.-]+$ ]]; then
	fail "RUN_LABEL must use only letters, numbers, dot, underscore, or dash: ${RUN_LABEL}"
fi

if [ -e "${RUN_DIR}" ] && [ "${F250_ALLOW_RUN_DIR_REUSE:-false}" != "true" ]; then
	fail "run directory already exists: ${RUN_DIR}; set F250_ALLOW_RUN_DIR_REUSE=true to reuse"
fi

P0_RUN_DIR=""
P0_TASK=""
P0_VEHICLE=""
P0_STATE=""
P0_SCENE=""
P0_WORLD=""
P0_PERCEPTION_SOURCE=""
P0_RAW_CLOUD_TOPIC=""
P0_ROUTE_ID=""
P0_ROUTE_NAME=""
P0_ROUTE_PROFILE=""
P0_ROUTE_PROFILE_SOURCE=""
P0_ROUTE_EFFECTIVE_SCENE=""
P0_ROUTE_METADATA_JSON=""
P0_ROUTE_WAYPOINTS_CSV=""
P0_ROUTE_WAYPOINT_COUNT=""
P0_ROUTE_FIRST_LABEL=""
P0_ROUTE_FINAL_LABEL=""
P0_ROUTE_TOTAL_LENGTH_M=""
STATUS_INPUT_FILE="${SOURCE_CURRENT_STATUS:-${CURRENT_STATUS}}"
if [ -f "${STATUS_INPUT_FILE}" ]; then
	P0_RUN_DIR="$(env_value run_dir "${STATUS_INPUT_FILE}")"
	P0_TASK="$(env_value task "${STATUS_INPUT_FILE}")"
	P0_VEHICLE="$(env_value vehicle "${STATUS_INPUT_FILE}")"
	P0_STATE="$(env_value state "${STATUS_INPUT_FILE}")"
	P0_SCENE="$(env_value scene_config "${STATUS_INPUT_FILE}")"
	P0_WORLD="$(env_value world "${STATUS_INPUT_FILE}")"
	P0_PERCEPTION_SOURCE="$(env_value perception_source "${STATUS_INPUT_FILE}")"
	P0_RAW_CLOUD_TOPIC="$(env_value raw_cloud_topic "${STATUS_INPUT_FILE}")"
	P0_ROUTE_ID="$(env_value route_id "${STATUS_INPUT_FILE}")"
	P0_ROUTE_NAME="$(env_value route_name "${STATUS_INPUT_FILE}")"
	P0_ROUTE_PROFILE="$(env_value route_profile "${STATUS_INPUT_FILE}")"
	P0_ROUTE_PROFILE_SOURCE="$(env_value route_profile_source "${STATUS_INPUT_FILE}")"
	P0_ROUTE_EFFECTIVE_SCENE="$(env_value route_effective_scene "${STATUS_INPUT_FILE}")"
	P0_ROUTE_METADATA_JSON="$(env_value route_metadata_json "${STATUS_INPUT_FILE}")"
	P0_ROUTE_WAYPOINTS_CSV="$(env_value route_waypoints_csv "${STATUS_INPUT_FILE}")"
	P0_ROUTE_WAYPOINT_COUNT="$(env_value route_waypoint_count "${STATUS_INPUT_FILE}")"
	P0_ROUTE_FIRST_LABEL="$(env_value route_first_label "${STATUS_INPUT_FILE}")"
	P0_ROUTE_FINAL_LABEL="$(env_value route_final_label "${STATUS_INPUT_FILE}")"
	P0_ROUTE_TOTAL_LENGTH_M="$(env_value route_total_length_m "${STATUS_INPUT_FILE}")"
	[ -n "${P0_PERCEPTION_SOURCE}" ] || P0_PERCEPTION_SOURCE="$(env_value sensor "${STATUS_INPUT_FILE}")"
fi

ROUTE_ID="${P0_ROUTE_ID:-${ROUTE_ID:-classic_p0_p8}}"
ROUTE_NAME="${P0_ROUTE_NAME:-${ROUTE_NAME:-Classic P0-P8}}"
ROUTE_PROFILE="${P0_ROUTE_PROFILE:-${ROUTE_PROFILE:-${PKG}/config/routes/classic_p0_p8.yaml}}"
ROUTE_PROFILE_SOURCE="${P0_ROUTE_PROFILE_SOURCE:-${ROUTE_PROFILE_SOURCE:-builtin}}"
ROUTE_EFFECTIVE_SCENE="${P0_ROUTE_EFFECTIVE_SCENE:-${ROUTE_EFFECTIVE_SCENE:-${SCENE_CONFIG_FIXED}}}"
ROUTE_METADATA_JSON="${P0_ROUTE_METADATA_JSON:-${ROUTE_METADATA_JSON:-}}"
ROUTE_WAYPOINTS_CSV="${P0_ROUTE_WAYPOINTS_CSV:-${ROUTE_WAYPOINTS_CSV:-}}"
ROUTE_WAYPOINT_COUNT="${P0_ROUTE_WAYPOINT_COUNT:-${ROUTE_WAYPOINT_COUNT:-9}}"
ROUTE_FIRST_LABEL="${P0_ROUTE_FIRST_LABEL:-${ROUTE_FIRST_LABEL:-P0}}"
ROUTE_FINAL_LABEL="${P0_ROUTE_FINAL_LABEL:-${ROUTE_FINAL_LABEL:-P8}}"
ROUTE_TOTAL_LENGTH_M="${P0_ROUTE_TOTAL_LENGTH_M:-${ROUTE_TOTAL_LENGTH_M:-}}"
SCENE_CONFIG_SELECTED="${ROUTE_EFFECTIVE_SCENE}"

if [ -n "${REQUESTED_ROUTE}" ]; then
	REQUESTED_ROUTE_CANONICAL="$(canonical_route_id "${REQUESTED_ROUTE}")"
	[ -n "${REQUESTED_ROUTE_CANONICAL}" ] || fail "could not resolve requested route: ${REQUESTED_ROUTE}"
	if [ "${REQUESTED_ROUTE_CANONICAL}" != "${ROUTE_ID}" ]; then
		fail "selected route mismatch: current=${ROUTE_ID} requested=${REQUESTED_ROUTE} canonical=${REQUESTED_ROUTE_CANONICAL}"
	fi
fi
if [ -n "${REQUESTED_ROUTE_PROFILE}" ]; then
	fail "custom route profiles are not available in the locked Stable Baseline workflow"
fi
if [ "${ROUTE_ID}" != "classic_p0_p8" ]; then
	fail "current route '${ROUTE_ID}' is not runnable in the locked Stable Baseline workflow; relaunch P0 hover with classic_p0_p8"
fi
require_path "${SCENE_CONFIG_SELECTED}"
if [ -n "${ROUTE_METADATA_JSON}" ]; then
	require_path "${ROUTE_METADATA_JSON}"
fi
if [ -n "${ROUTE_WAYPOINTS_CSV}" ]; then
	require_path "${ROUTE_WAYPOINTS_CSV}"
fi
export ROUTE_ID ROUTE_NAME ROUTE_PROFILE ROUTE_PROFILE_SOURCE ROUTE_EFFECTIVE_SCENE
export ROUTE_METADATA_JSON ROUTE_WAYPOINTS_CSV ROUTE_WAYPOINT_COUNT ROUTE_FIRST_LABEL ROUTE_FINAL_LABEL ROUTE_TOTAL_LENGTH_M

if [ -z "${PERCEPTION_SOURCE_FROM_ENV}" ] && [ -n "${P0_PERCEPTION_SOURCE}" ]; then
	PERCEPTION_SOURCE="${P0_PERCEPTION_SOURCE}"
fi
PERCEPTION_SOURCE="$(normalize_sensor "${PERCEPTION_SOURCE}")"
SENSOR_LABEL="$(sensor_label "${PERCEPTION_SOURCE}")"
if [ -z "${RAW_CLOUD_TOPIC_FROM_ENV}" ]; then
	if [ -n "${P0_RAW_CLOUD_TOPIC}" ]; then
		RAW_CLOUD_TOPIC="${P0_RAW_CLOUD_TOPIC}"
	else
		RAW_CLOUD_TOPIC="$(raw_cloud_topic_for_sensor "${PERCEPTION_SOURCE}")"
	fi
else
	RAW_CLOUD_TOPIC="${RAW_CLOUD_TOPIC_FROM_ENV}"
fi
PLANNER_CLOUD_TOPIC="${PLANNER_CLOUD_TOPIC:-/maritime/obstacles_cloud}"
if [ "${PERCEPTION_SOURCE}" = "lidar" ]; then
	LIDAR_CLOUD_TOPIC="${LIDAR_CLOUD_TOPIC:-${RAW_CLOUD_TOPIC}}"
else
	LIDAR_CLOUD_TOPIC="${LIDAR_CLOUD_TOPIC:-/maritime/lidar_points}"
fi
LIDAR_SCAN_TOPIC="${LIDAR_SCAN_TOPIC:-/maritime/lidar_scan}"
if [ "${PERCEPTION_SOURCE}" = "depth" ]; then
	DEPTH_CLOUD_TOPIC="${DEPTH_CLOUD_TOPIC:-${RAW_CLOUD_TOPIC}}"
else
	DEPTH_CLOUD_TOPIC="${DEPTH_CLOUD_TOPIC:-/maritime_depth_camera/points}"
fi
OCCUPANCY_TOPIC="${OCCUPANCY_TOPIC:-/grid_map/occupancy_inflate}"
export PERCEPTION_SOURCE SENSOR_LABEL RAW_CLOUD_TOPIC PLANNER_CLOUD_TOPIC LIDAR_CLOUD_TOPIC LIDAR_SCAN_TOPIC DEPTH_CLOUD_TOPIC OCCUPANCY_TOPIC

if [ "${DRY_RUN}" != "true" ]; then
	[ -f "${CURRENT_STATUS}" ] || fail "missing current P0 hover status: ${CURRENT_STATUS}; run f250_start_to_p0_hover.sh first"
	[ "${P0_TASK}" = "launch" ] || fail "current status is not a fresh P0 launch: task=${P0_TASK:-<empty>} state=${P0_STATE:-<empty>}; relaunch P0 hover first"
	[ "${P0_STATE}" = "hover_ready" ] || fail "P0 hover is not ready yet: task=${P0_TASK:-<empty>} state=${P0_STATE:-<empty>}; wait for hover_ready before route"
	[ "${P0_VEHICLE}" = "f250" ] || fail "current status is not F250: ${CURRENT_STATUS} vehicle=${P0_VEHICLE:-<empty>}"
	if [ -n "${P0_PERCEPTION_SOURCE}" ] && [ "${P0_PERCEPTION_SOURCE}" != "${PERCEPTION_SOURCE}" ]; then
		fail "current status sensor mismatch: current=${P0_PERCEPTION_SOURCE} requested=${PERCEPTION_SOURCE}"
	fi
	if [ -n "${P0_SCENE}" ] && [ "${P0_SCENE}" != "${SCENE_CONFIG_SELECTED}" ]; then
		fail "current status scene mismatch: ${P0_SCENE}"
	fi
	if [ -n "${P0_WORLD}" ] && [ "${P0_WORLD}" != "${WORLD_FIXED}" ]; then
		fail "current status world mismatch: ${P0_WORLD}"
	fi
fi

ROUTE_MAX_DURATION_SEC="${ROUTE_MAX_DURATION_SEC:-360}"
mkdir -p "${RUN_DIR}" "${RUN_DIR}/logs"

STATUS_FILE="${RUN_DIR}/status.env"
ROUTE_STATUS_ENV="${RUN_DIR}/route_status.env"
PROVENANCE_FILE="${RUN_DIR}/provenance.txt"
PARAMS_JSON="${RUN_DIR}/params.json"
TRAJECTORY_CSV="${RUN_DIR}/actual_trajectory.csv"
SUMMARY_JSON="${RUN_DIR}/summary.json"
METRIC_SUMMARY_JSON="${RUN_DIR}/metric_summary.json"
METRIC_WAYPOINTS_CSV="${RUN_DIR}/metric_waypoints.csv"
METRICS_JSON="${RUN_DIR}/metrics.json"
ROUTE_TERMINAL_LOG="${RUN_DIR}/route_terminal.log"
GATE_JSON="${RUN_DIR}/perception_gate.json"
RECORDER_LOG="${RUN_DIR}/logs/recorder.log"
DISPLAY_LOG="${RUN_DIR}/logs/display_helper.log"
RELEASE_LOG="${RUN_DIR}/logs/release.log"
GATE_LOG="${RUN_DIR}/logs/perception_gate.log"
PREALIGN_JSON="${RUN_DIR}/prealign_yaw.json"
PREALIGN_LOG="${RUN_DIR}/prealign_yaw.log"
PREALIGN_STDOUT_LOG="${RUN_DIR}/logs/prealign_yaw_stdout.log"
METRIC_REPLAY_LOG="${RUN_DIR}/logs/metric_replay.log"
POSTPROCESS_LOG="${RUN_DIR}/logs/postprocess.log"
RUN_ROUTE_METADATA_JSON="${RUN_DIR}/route_profile.json"
RUN_ROUTE_EFFECTIVE_SCENE="${RUN_DIR}/route_effective_scene.yaml"
RUN_ROUTE_WAYPOINTS_CSV="${RUN_DIR}/route_waypoints.csv"

copy_if_different() {
	local src="$1"
	local dest="$2"
	[ -e "$src" ] || return 0
	[ "$src" = "$dest" ] && return 0
	if [ -e "$dest" ] && [ "$(readlink -f "$src")" = "$(readlink -f "$dest")" ]; then
		return 0
	fi
	cp "$src" "$dest"
}

copy_if_different "${SCENE_CONFIG_SELECTED}" "${RUN_ROUTE_EFFECTIVE_SCENE}"
if [ -n "${ROUTE_METADATA_JSON}" ] && [ -f "${ROUTE_METADATA_JSON}" ]; then
	copy_if_different "${ROUTE_METADATA_JSON}" "${RUN_ROUTE_METADATA_JSON}"
else
	python3 "${ROUTE_HELPER}" --route-profile "${ROUTE_PROFILE}" --base-scene "${SCENE_CONFIG_FIXED}" \
		--summary-json "${RUN_ROUTE_METADATA_JSON}" --csv-out "${RUN_ROUTE_WAYPOINTS_CSV}" >/dev/null
fi
if [ -n "${ROUTE_WAYPOINTS_CSV}" ] && [ -f "${ROUTE_WAYPOINTS_CSV}" ]; then
	copy_if_different "${ROUTE_WAYPOINTS_CSV}" "${RUN_ROUTE_WAYPOINTS_CSV}"
elif [ ! -f "${RUN_ROUTE_WAYPOINTS_CSV}" ]; then
	python3 "${ROUTE_HELPER}" --route-profile "${ROUTE_PROFILE}" --base-scene "${SCENE_CONFIG_FIXED}" \
		--csv-out "${RUN_ROUTE_WAYPOINTS_CSV}" >/dev/null
fi
SCENE_CONFIG_SELECTED="${RUN_ROUTE_EFFECTIVE_SCENE}"
ROUTE_EFFECTIVE_SCENE="${RUN_ROUTE_EFFECTIVE_SCENE}"
ROUTE_METADATA_JSON="${RUN_ROUTE_METADATA_JSON}"
ROUTE_WAYPOINTS_CSV="${RUN_ROUTE_WAYPOINTS_CSV}"
export SCENE_CONFIG_SELECTED ROUTE_EFFECTIVE_SCENE ROUTE_METADATA_JSON ROUTE_WAYPOINTS_CSV

if [ "${F250_ROUTE_TERMINAL_LOG_READY:-false}" != "true" ]; then
	: >"${ROUTE_TERMINAL_LOG}"
fi

{
	echo "created_at=$(date -Is)"
	echo "project_root=${PROJECT_ROOT}"
	echo "workspace=${WS}"
	echo "package=${PKG}"
	echo "script=${0}"
	echo "run_dir=${RUN_DIR}"
	echo "run_label=${RUN_LABEL}"
	echo "source_p0_status=${CURRENT_STATUS}"
	echo "source_p0_run_dir=${P0_RUN_DIR}"
	echo "source_p0_state=${P0_STATE}"
	echo "base_scene_config=${SCENE_CONFIG_FIXED}"
	echo "scene_config=${SCENE_CONFIG_SELECTED}"
	echo "world=${WORLD_FIXED}"
	echo "map_authority=${MAP_AUTHORITY}"
	echo "quick_complex_profile_id=${QUICK_COMPLEX_PROFILE_ID}"
	echo "quick_complex_baseline=${QUICK_COMPLEX_BASELINE}"
	echo "route_id=${ROUTE_ID}"
	echo "route_name=${ROUTE_NAME}"
	echo "route_profile=${ROUTE_PROFILE}"
	echo "route_profile_source=${ROUTE_PROFILE_SOURCE}"
	echo "route_effective_scene=${ROUTE_EFFECTIVE_SCENE}"
	echo "route_metadata_json=${ROUTE_METADATA_JSON}"
	echo "route_waypoints_csv=${ROUTE_WAYPOINTS_CSV}"
	echo "route_waypoint_count=${ROUTE_WAYPOINT_COUNT}"
	echo "route_first_label=${ROUTE_FIRST_LABEL}"
	echo "route_final_label=${ROUTE_FINAL_LABEL}"
	echo "route_total_length_m=${ROUTE_TOTAL_LENGTH_M}"
	echo "vehicle=f250"
	echo "sensor=${PERCEPTION_SOURCE}"
	echo "sensor_label=${SENSOR_LABEL}"
	echo "perception_source=${PERCEPTION_SOURCE}"
	echo "planner_cloud_topic=${PLANNER_CLOUD_TOPIC}"
	echo "raw_cloud_topic=${RAW_CLOUD_TOPIC}"
	echo "lidar_cloud_topic=${LIDAR_CLOUD_TOPIC}"
	echo "lidar_scan_topic=${LIDAR_SCAN_TOPIC}"
	echo "depth_cloud_topic=${DEPTH_CLOUD_TOPIC}"
	echo "occupancy_topic=${OCCUPANCY_TOPIC}"
	echo "route_policy=excludes_planning_success_rate_metric_3_10_and_yaw;dynamic_boat_clearance_telemetry_only"
	echo "host=$(hostname)"
	echo "user=$(id -un)"
} >"${PROVENANCE_FILE}"

write_params_json
SOURCE_STATUS_COPY="${RUN_DIR}/source_current_status.env"
if [ -f "${CURRENT_STATUS}" ]; then
	cp "${CURRENT_STATUS}" "${SOURCE_STATUS_COPY}"
fi
export SOURCE_STATUS_COPY SOURCE_CURRENT_STATUS="${SOURCE_STATUS_COPY}"
write_status "prepared"

HELPER_COMMON=(
	--run-dir "${RUN_DIR}"
	--run-label "${RUN_LABEL}"
	--scene-config "${SCENE_CONFIG_SELECTED}"
	--perception-source "${PERCEPTION_SOURCE}"
	--dynamic-mode auto
	--max-duration-sec "${ROUTE_MAX_DURATION_SEC}"
	--terminal-log "${ROUTE_TERMINAL_LOG}"
	--route-status-env "${ROUTE_STATUS_ENV}"
	--status-env "${STATUS_FILE}"
)

if [ "${DRY_RUN}" = "true" ]; then
	python3 "${DISPLAY_HELPER}" dry-run "${HELPER_COMMON[@]}"
	exit $?
fi

# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
if [ -f "${WS}/devel/setup.bash" ]; then
	# shellcheck disable=SC1090
	source "${WS}/devel/setup.bash"
else
	export ROS_PACKAGE_PATH="${WS}/src:${ROS_PACKAGE_PATH:-}"
fi

export MARITIME_VEHICLE="f250"
export SCENE_LEVEL="${SCENE_LEVEL_FIXED}"
export SCENE_CONFIG="${SCENE_CONFIG_SELECTED}"
export WORLD="${WORLD_FIXED}"
export PERCEPTION_SOURCE
export DYNAMIC_MODE="auto"
export MARITIME_START_TOPIC="${MARITIME_START_TOPIC:-/maritime/demo/start_waypoints}"

ensure_ros_available
run_perception_gate

if [ "${RUN_IN_BACKGROUND}" = "true" ]; then
	write_status "background_worker_starting"
	if launch_route_worker; then
		exit 0
	fi
	# launch_route_worker already wrote state=failed and logged the cause.
	exit 2
fi

RECORDER_PID=""
DISPLAY_PID=""
cleanup() {
	stop_pid "${DISPLAY_PID}"
	stop_pid "${RECORDER_PID}"
	append_terminal_over
}
trap cleanup EXIT

open_metrics_terminal "F250 Route Metrics" "${ROUTE_TERMINAL_LOG}" "f250_route_metrics_${RUN_LABEL}" >/dev/null || true
layout_metrics_window

run_route_prealign_yaw

write_status "recording"

# shellcheck disable=SC2086
python3 "${RECORDER}" \
	--scene-config "${SCENE_CONFIG_SELECTED}" \
	--output-csv "${TRAJECTORY_CSV}" \
	--summary-json "${SUMMARY_JSON}" \
	--max-duration-sec "${ROUTE_MAX_DURATION_SEC}" \
	--perception-source "${PERCEPTION_SOURCE}" \
	--planner-cloud-topic "${PLANNER_CLOUD_TOPIC}" \
	--raw-cloud-topic "${RAW_CLOUD_TOPIC}" \
	--lidar-scan-topic "${LIDAR_SCAN_TOPIC}" \
	--depth-cloud-topic "${DEPTH_CLOUD_TOPIC}" \
	--occupancy-topic "${OCCUPANCY_TOPIC}" \
	${F250_ROUTE_ARGS:-} \
	>"${RECORDER_LOG}" 2>&1 &
RECORDER_PID="$!"

python3 "${DISPLAY_HELPER}" live-monitor "${HELPER_COMMON[@]}" \
	2>"${DISPLAY_LOG}" &
DISPLAY_PID="$!"

sleep "${F250_ROUTE_RECORD_PRESTART_SEC:-1.0}"

"${START_DEMO}" "${MARITIME_START_TOPIC}" >"${RELEASE_LOG}" 2>&1

set +e
wait "${RECORDER_PID}"
RECORDER_STATUS=$?
RECORDER_PID=""
set -e

stop_pid "${DISPLAY_PID}"
DISPLAY_PID=""

if [ "${RECORDER_STATUS}" -ne 0 ]; then
	write_status "recorder_failed"
	echo "[f250-route] recorder failed status=${RECORDER_STATUS}; see ${RECORDER_LOG}" >&2
	exit "${RECORDER_STATUS}"
fi

write_status "finalizing"

set +e
python3 "${METRIC_MONITOR}" --offline \
	--scene-config "${SCENE_CONFIG_SELECTED}" \
	--trajectory-csv "${TRAJECTORY_CSV}" \
	--output-dir "${RUN_DIR}" \
	--run-label "${RUN_LABEL}" \
	--dynamic-mode auto \
	>"${METRIC_REPLAY_LOG}" 2>&1
METRIC_STATUS=$?

if [ "${METRIC_STATUS}" -eq 0 ]; then
	python3 "${DISPLAY_HELPER}" finalize "${HELPER_COMMON[@]}" --print-final \
		>"${POSTPROCESS_LOG}" 2>&1
	FINAL_STATUS=$?
else
	FINAL_STATUS=2
fi
set -e

{
	echo "metric_replay_status=${METRIC_STATUS}"
	echo "postprocess_status=${FINAL_STATUS}"
	echo "terminal_finalize_status=${FINAL_STATUS}"
	echo "perception_gate_json=${GATE_JSON}"
	echo "perception_gate_log=${GATE_LOG}"
	echo "metric_replay_log=${METRIC_REPLAY_LOG}"
	echo "postprocess_log=${POSTPROCESS_LOG}"
} >"${RUN_DIR}/postprocess_status.env"

POSTPROCESS_REQUIRED_FAILURES=()
if [ "${METRIC_STATUS}" -ne 0 ]; then
	POSTPROCESS_REQUIRED_FAILURES+=("metric_replay_status=${METRIC_STATUS}")
fi
if [ "${FINAL_STATUS}" -ne 0 ]; then
	POSTPROCESS_REQUIRED_FAILURES+=("terminal_finalize_status=${FINAL_STATUS}")
fi
ROUTE_ACCEPTANCE_JSON="${RUN_DIR}/route_acceptance_summary.json"
for required_artifact in \
	"${SUMMARY_JSON}" \
	"${METRIC_SUMMARY_JSON}" \
	"${METRICS_JSON}" \
	"${ROUTE_ACCEPTANCE_JSON}"; do
	if [ ! -s "${required_artifact}" ]; then
		POSTPROCESS_REQUIRED_FAILURES+=("missing_required_artifact=${required_artifact}")
	elif ! python3 - "${required_artifact}" <<'PY' >/dev/null 2>&1; then
import json
import sys
with open(sys.argv[1], "r", encoding="utf-8") as handle:
    json.load(handle)
PY
		POSTPROCESS_REQUIRED_FAILURES+=("invalid_required_json=${required_artifact}")
	fi
done
if [ ! -s "${ROUTE_TERMINAL_LOG}" ]; then
	POSTPROCESS_REQUIRED_FAILURES+=("missing_required_artifact=${ROUTE_TERMINAL_LOG}")
fi
if [ "${#POSTPROCESS_REQUIRED_FAILURES[@]}" -gt 0 ]; then
	{
		echo "required_artifact_check=failed"
		for item in "${POSTPROCESS_REQUIRED_FAILURES[@]}"; do
			echo "${item}"
		done
	} >>"${RUN_DIR}/postprocess_status.env"
	write_status "postprocess_failed"
	append_terminal_over
	printf "[f250-route] postprocess required artifact check failed; see %s\n" "${RUN_DIR}/postprocess_status.env" >&2
	exit 2
fi
echo "required_artifact_check=ok" >>"${RUN_DIR}/postprocess_status.env"

if [ ! -s "${ROUTE_ACCEPTANCE_JSON}" ]; then
	echo "missing_route_acceptance_summary=${ROUTE_ACCEPTANCE_JSON}" >>"${RUN_DIR}/postprocess_status.env"
	write_status "postprocess_failed"
	exit 2
elif ! python3 - "${ROUTE_ACCEPTANCE_JSON}" <<'PY' >/dev/null 2>&1; then
import json
import sys
with open(sys.argv[1], "r", encoding="utf-8") as handle:
    data = json.load(handle)
if "ok" not in data or "components" not in data:
    raise SystemExit(2)
PY
	echo "invalid_route_acceptance_summary=${ROUTE_ACCEPTANCE_JSON}" >>"${RUN_DIR}/postprocess_status.env"
	write_status "postprocess_failed"
	exit 2
fi

if [ "${FINAL_STATUS}" -eq 0 ]; then
	publish_current_review
	write_status "complete"
fi

exit "${FINAL_STATUS}"
