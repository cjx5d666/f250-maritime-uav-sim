#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/f250_paths.sh"
PROJECT_ROOT="$(f250_resolve_project_root "${SCRIPT_DIR}")"
WS="${PROJECT_ROOT}/catkin_ws"
PKG="$(f250_resolve_package_root "${SCRIPT_DIR}" "${PROJECT_ROOT}")"
START_SCRIPT="${PKG}/scripts/start_maritime_sim.sh"
STOP_SCRIPT="${PKG}/scripts/f250_stop_all.sh"
LAYOUT_SCRIPT="${PKG}/scripts/f250_layout_windows.py"
ROUTE_HELPER="${PKG}/scripts/f250_route_profiles.py"
CHECK_GATE="${PKG}/scripts/f250_check_perception_gate.py"
SCENE_LEVEL_FIXED="level_m_gps_assets_quick_complex"
SCENE_CONFIG_FIXED="${PKG}/config/scenes/${SCENE_LEVEL_FIXED}.yaml"
WORLD_FIXED="${PKG}/worlds/maritime_${SCENE_LEVEL_FIXED}.world"
MAP_AUTHORITY="${MAP_AUTHORITY:-${PROJECT_ROOT}/map_authority/p0p8_clean_scene}"
f250_apply_quick_complex_defaults "${PKG}"
QUICK_COMPLEX_PROFILE_ID="${F250_QUICK_COMPLEX_PROFILE_ID:-quick_complex_accepted}"
QUICK_COMPLEX_BASELINE="${F250_QUICK_COMPLEX_BASELINE:-${QUICK_COMPLEX_PROFILE_ID}}"
QUICK_COMPLEX_DESCRIPTION="${F250_QUICK_COMPLEX_DESCRIPTION:-F250 quick-complex accepted EGO defaults}"

fail() {
	echo "f250_start_to_p0_hover: $*" >&2
	exit 2
}

usage() {
	cat <<EOF
Usage:
  ${0} [--sensor lidar|depth] [--route classic_p0_p8] [--dry-run]
  ${0} --help

Starts the F250 quick-complex stack and holds at the selected route start. The waypoint sequence is
left paused; a later script can publish /maritime/demo/start_waypoints to run
the selected route.

Fixed task inputs:
  vehicle: f250
  scene: ${SCENE_LEVEL_FIXED}
  perception: --sensor > F250_SENSOR/PERCEPTION_SOURCE > runtime_state/active_sensor.env > lidar
  dynamic obstacles: auto
  default route: classic_p0_p8, locked Stable Baseline compatibility
  hover target: first selected route waypoint
  route defaults: ${QUICK_COMPLEX_DESCRIPTION}

Useful environment overrides:
  RUN_ROOT=...                  default: ${PROJECT_ROOT}/runtime_state/work
  F250_SKIP_STARTUP_HEALTH_GATE=true skip final gzserver/perception readiness gate
  F250_STARTUP_HEALTH_TIMEOUT_SEC=... default: 45
  RUN_LABEL=...                 default: f250_p0_hover_<sensor>_<timestamp>
  SCREEN_NAME=...               default: f250_p0_hover_<sensor>_<timestamp>
  ENABLE_RVIZ=true|false        default: true
  PX4_GUI=true|false            default: true
  PX4_NO_FOLLOW_MODE=1          default: 1, keep Gazebo on the fixed route camera
  F250_PX4_ROOT=...             override PX4-Autopilot root
  DISPLAY=:0                    default: current DISPLAY or :0
  F250_PRESTOP=true             run f250_stop_all.sh before starting
  F250_ALLOW_EXISTING_RUNTIME=true
  F250_ALLOW_RUN_DIR_REUSE=true
  F250_SENSOR=lidar|depth       env sensor override, also refreshes runtime_state/active_sensor.env
  PERCEPTION_SOURCE=lidar|depth legacy env sensor override
  F250_ROUTE=classic_p0_p8       compatibility aliases resolve to classic_p0_p8
  F250_PROJECT_ROOT=...         override project root detected from script path
  MAP_AUTHORITY=...             override authoritative map directory

Stop:
  ${STOP_SCRIPT}

Release route later:
  ${PKG}/scripts/start_demo_waypoints.sh
EOF
}

normalize_sensor() {
	case "$1" in
	lidar | depth)
		printf "%s\n" "$1"
		;;
	*)
		fail "unsupported sensor '$1'; expected lidar or depth"
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

env_file_value() {
	local key="$1"
	local file="$2"
	[ -f "${file}" ] || return 0
	awk -F= -v key="${key}" '$1 == key {sub(/^[^=]*=/, ""); print; exit}' "${file}"
}

read_sensor_preference() {
	local file="$1"
	local value=""
	[ -f "${file}" ] || return 0
	value="$(env_file_value PERCEPTION_SOURCE "${file}")"
	[ -n "${value}" ] || value="$(env_file_value F250_SENSOR "${file}")"
	[ -n "${value}" ] || value="$(env_file_value sensor "${file}")"
	[ -n "${value}" ] || return 0
	normalize_sensor "${value}"
}

write_sensor_preference() {
	local file="$1"
	local sensor="$2"
	local selected_by="$3"
	local tmp="${file}.$$"
	{
		echo "PERCEPTION_SOURCE=${sensor}"
		echo "F250_SENSOR=${sensor}"
		echo "sensor=${sensor}"
		echo "selected_by=${selected_by}"
		echo "updated_at=$(date -Is)"
	} >"${tmp}"
	mv "${tmp}" "${file}"
}

DRY_RUN="${F250_DRY_RUN:-false}"
REQUESTED_SENSOR=""
SENSOR_ARG_SET="false"
REQUESTED_ROUTE="${F250_ROUTE:-}"
REQUESTED_ROUTE_PROFILE="${F250_ROUTE_PROFILE:-}"
ROUTE_ARG_SET="false"
ROUTE_PROFILE_ARG_SET="false"
while [ "$#" -gt 0 ]; do
	case "$1" in
	--help | -h)
		usage
		exit 0
		;;
	--sensor)
		[ "$#" -ge 2 ] || fail "--sensor requires lidar or depth"
		REQUESTED_SENSOR="$2"
		SENSOR_ARG_SET="true"
		shift 2
		;;
	--sensor=*)
		REQUESTED_SENSOR="${1#--sensor=}"
		SENSOR_ARG_SET="true"
		shift
		;;
	--dry-run)
		DRY_RUN="true"
		shift
		;;
	--route)
		[ "$#" -ge 2 ] || fail "--route requires a route id"
		REQUESTED_ROUTE="$2"
		ROUTE_ARG_SET="true"
		shift 2
		;;
	--route=*)
		REQUESTED_ROUTE="${1#--route=}"
		ROUTE_ARG_SET="true"
		shift
		;;
	--route-profile | --custom-route)
		[ "$#" -ge 2 ] || fail "$1 requires a YAML path"
		REQUESTED_ROUTE_PROFILE="$2"
		ROUTE_PROFILE_ARG_SET="true"
		shift 2
		;;
	--route-profile=* | --custom-route=*)
		REQUESTED_ROUTE_PROFILE="${1#*=}"
		ROUTE_PROFILE_ARG_SET="true"
		shift
		;;
	*)
		echo "Unknown argument: $1" >&2
		echo "Run ${0} --help" >&2
		exit 2
		;;
	esac
done

if [ -n "${REQUESTED_ROUTE_PROFILE}" ]; then
	fail "custom route profiles are not available in the locked Stable Baseline workflow"
fi
if [ -n "${REQUESTED_ROUTE}" ] && [ "${REQUESTED_ROUTE}" != "classic_p0_p8" ]; then
	REQUESTED_ROUTE_CANONICAL="$(python3 "${ROUTE_HELPER}" --route-id "${REQUESTED_ROUTE}" --base-scene "${SCENE_CONFIG_FIXED}" --print-shell |
		awk -F= '$1 == "route_id" {print $2; exit}')"
	[ "${REQUESTED_ROUTE_CANONICAL}" = "classic_p0_p8" ] || fail "only classic_p0_p8 is available in the locked Stable Baseline workflow"
	REQUESTED_ROUTE="classic_p0_p8"
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
RUNTIME_STATE_DIR="${F250_RUNTIME_STATE_DIR:-${PROJECT_ROOT}/runtime_state}"
RUN_ROOT="${RUN_ROOT:-${RUNTIME_STATE_DIR}/work}"
ACTIVE_STATUS="${F250_ACTIVE_TASK_ENV:-${RUNTIME_STATE_DIR}/active_task.env}"
ACTIVE_SENSOR_ENV="${F250_ACTIVE_SENSOR_ENV:-${RUNTIME_STATE_DIR}/active_sensor.env}"
if [ "${F250_CLEAN_RUNS_BEFORE_LAUNCH:-true}" = "true" ] && [ "${DRY_RUN}" != "true" ]; then
	F250_KEEP_ACTIVE_TASK_DIR=false F250_CLEANUP_QUIET="${F250_CLEANUP_QUIET:-true}" RUN_ROOT="${RUN_ROOT}" "${SCRIPT_DIR}/f250_cleanup_runs.sh" || true
fi
RUN_DIR_INPUT="${RUN_DIR:-}"

require_path() {
	[ -e "$1" ] || fail "missing required path: $1"
}

mkdir -p "${RUNTIME_STATE_DIR}" "${RUN_ROOT}" "$(dirname "${ACTIVE_STATUS}")" "$(dirname "${ACTIVE_SENSOR_ENV}")"
RUNTIME_STATE_DIR="$(cd "${RUNTIME_STATE_DIR}" && pwd -P)"
RUN_ROOT="$(cd "${RUN_ROOT}" && pwd -P)"
ACTIVE_STATUS="$(cd "$(dirname "${ACTIVE_STATUS}")" && pwd -P)/$(basename "${ACTIVE_STATUS}")"
ACTIVE_SENSOR_ENV="$(cd "$(dirname "${ACTIVE_SENSOR_ENV}")" && pwd -P)/$(basename "${ACTIVE_SENSOR_ENV}")"
SENSOR_PREFERENCE_FILE="${ACTIVE_SENSOR_ENV}"
PERSISTED_SENSOR="$(read_sensor_preference "${SENSOR_PREFERENCE_FILE}")"
ENV_SENSOR="${F250_SENSOR:-${PERCEPTION_SOURCE:-}}"
SENSOR_SELECTED_BY="default"
if [ "${SENSOR_ARG_SET}" = "true" ]; then
	SENSOR_SELECTED_BY="cli"
	REQUESTED_SENSOR_RAW="${REQUESTED_SENSOR}"
elif [ -n "${ENV_SENSOR}" ]; then
	SENSOR_SELECTED_BY="environment"
	REQUESTED_SENSOR_RAW="${ENV_SENSOR}"
elif [ -n "${PERSISTED_SENSOR}" ]; then
	SENSOR_SELECTED_BY="preference"
	REQUESTED_SENSOR_RAW="${PERSISTED_SENSOR}"
else
	REQUESTED_SENSOR_RAW="lidar"
fi
PERCEPTION_SOURCE="$(normalize_sensor "${REQUESTED_SENSOR_RAW}")"
SENSOR_LABEL="$(sensor_label "${PERCEPTION_SOURCE}")"
export PERCEPTION_SOURCE SENSOR_LABEL
write_sensor_preference "${SENSOR_PREFERENCE_FILE}" "${PERCEPTION_SOURCE}" "${SENSOR_SELECTED_BY}"

RUN_LABEL="${RUN_LABEL:-f250_p0_hover_${PERCEPTION_SOURCE}_${STAMP}}"
SCREEN_NAME="${SCREEN_NAME:-f250_p0_hover_${PERCEPTION_SOURCE}_${STAMP}}"
if [ -z "${RUN_DIR_INPUT}" ]; then
	RUN_DIR="${RUN_ROOT}/${RUN_LABEL}"
else
	RUN_DIR="${RUN_DIR_INPUT}"
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
*) fail "RUN_DIR must be inside RUN_ROOT=${RUN_ROOT}: ${RUN_DIR}" ;;
esac
LOG_DIR="${RUN_DIR}/logs"
METRIC_DIR="${RUN_DIR}/live_metric_runs"
STATUS_FILE="${RUN_DIR}/status.env"
PROVENANCE_FILE="${RUN_DIR}/provenance.txt"
PARAMS_FILE="${RUN_DIR}/params.env"
LAUNCH_ENV="${RUN_DIR}/launch_env.sh"
LAUNCH_CMD="${RUN_DIR}/launch_in_screen.sh"
START_LOG="${LOG_DIR}/start_maritime_sim.log"
PREFLIGHT_LOG="${LOG_DIR}/preflight_processes.txt"
METRIC_DISPLAY_LOG="${RUN_DIR}/realtime_metric_live.txt"
STARTUP_HEALTH_JSON="${RUN_DIR}/startup_health_gate.json"
STARTUP_HEALTH_LOG="${LOG_DIR}/startup_health_gate.log"
ROUTE_METADATA_JSON="${RUN_DIR}/route_profile.json"
ROUTE_STATUS_ENV="${RUN_DIR}/route.env"
ROUTE_EFFECTIVE_SCENE="${RUN_DIR}/route_effective_scene.yaml"
ROUTE_WAYPOINTS_CSV="${RUN_DIR}/route_waypoints.csv"

if [[ ! "${RUN_LABEL}" =~ ^[A-Za-z0-9_.-]+$ ]]; then
	fail "RUN_LABEL must use only letters, numbers, dot, underscore, or dash: ${RUN_LABEL}"
fi

if [[ ! "${SCREEN_NAME}" =~ ^[A-Za-z0-9_.-]+$ ]]; then
	fail "SCREEN_NAME must use only letters, numbers, dot, underscore, or dash: ${SCREEN_NAME}"
fi

require_path "${WS}"
require_path "${PKG}"
require_path "${START_SCRIPT}"
require_path "${STOP_SCRIPT}"
require_path "${LAYOUT_SCRIPT}"
require_path "${ROUTE_HELPER}"
require_path "${CHECK_GATE}"
require_path "${SCENE_CONFIG_FIXED}"
require_path "${WORLD_FIXED}"
require_path "${MAP_AUTHORITY}"

PX4_ROOT="$(f250_resolve_px4_root "${PROJECT_ROOT}")"
SITL_GAZEBO="${PX4_ROOT}/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
PX4_BUILD="${PX4_ROOT}/build/px4_sitl_default"

if [ -e "${RUN_DIR}" ] && [ "${F250_ALLOW_RUN_DIR_REUSE:-false}" != "true" ]; then
	fail "run directory already exists: ${RUN_DIR}; set F250_ALLOW_RUN_DIR_REUSE=true to reuse"
fi

if [ "${DRY_RUN}" != "true" ] && ! command -v screen >/dev/null 2>&1; then
	fail "screen is required to start a detachable human-facing session"
fi

mkdir -p "${LOG_DIR}" "${METRIC_DIR}"
: >"${PREFLIGHT_LOG}"
: >"${METRIC_DISPLAY_LOG}"

ROUTE_SELECTED_BY="default"
if [ "${ROUTE_ARG_SET}" = "true" ]; then
	ROUTE_SELECTED_BY="cli"
elif [ -n "${REQUESTED_ROUTE}" ]; then
	ROUTE_SELECTED_BY="environment"
fi

ROUTE_HELPER_ARGS=(
	--base-scene "${SCENE_CONFIG_FIXED}"
	--effective-scene "${ROUTE_EFFECTIVE_SCENE}"
	--summary-json "${ROUTE_METADATA_JSON}"
	--env-out "${ROUTE_STATUS_ENV}"
	--csv-out "${ROUTE_WAYPOINTS_CSV}"
	--selected-by "${ROUTE_SELECTED_BY}"
)
ROUTE_HELPER_ARGS+=(--route-id "${REQUESTED_ROUTE:-classic_p0_p8}")
python3 "${ROUTE_HELPER}" "${ROUTE_HELPER_ARGS[@]}" >/dev/null
# shellcheck source=/dev/null
source "${ROUTE_STATUS_ENV}"

DISPLAY_VALUE="${DISPLAY:-}"
if [ -z "${DISPLAY_VALUE//[[:space:]]/}" ]; then
	export DISPLAY=":0"
else
	export DISPLAY
fi
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"
export DISABLE_ROS1_EOL_WARNINGS="${DISABLE_ROS1_EOL_WARNINGS:-1}"
export F250_PROJECT_ROOT="${PROJECT_ROOT}"
export F250_PX4_ROOT="${PX4_ROOT}"
export ROS_PACKAGE_PATH="${PX4_ROOT}:${SITL_GAZEBO}:${ROS_PACKAGE_PATH:-}"
export GAZEBO_MODEL_PATH="${PKG}/models:${SITL_GAZEBO}/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_PLUGIN_PATH="/opt/ros/noetic/lib:${PX4_BUILD}/build_gazebo-classic:${GAZEBO_PLUGIN_PATH:-}"
export LD_LIBRARY_PATH="${PX4_BUILD}/build_gazebo-classic:${LD_LIBRARY_PATH:-}"

export MARITIME_VEHICLE="f250"
export SCENE_LEVEL="${SCENE_LEVEL_FIXED}"
export SCENE_CONFIG="${ROUTE_EFFECTIVE_SCENE}"
export WORLD="${WORLD_FIXED}"
export DYNAMIC_MODE="auto"
export LANDING_MODE="false"
export AUTO_OFFBOARD_ARM="${AUTO_OFFBOARD_ARM:-true}"
export REQUIRE_PLANNER_COMMAND_FOR_OFFBOARD="false"
export MARITIME_START_PAUSED="true"
export MARITIME_START_TOPIC="${MARITIME_START_TOPIC:-/maritime/demo/start_waypoints}"
export ENABLE_RVIZ="${ENABLE_RVIZ:-true}"
export PX4_GUI="${PX4_GUI:-true}"
export PX4_NO_FOLLOW_MODE="${PX4_NO_FOLLOW_MODE:-1}"
if [ -z "${RAW_CLOUD_TOPIC:-}" ]; then
	RAW_CLOUD_TOPIC="$(raw_cloud_topic_for_sensor "${PERCEPTION_SOURCE}")"
fi
export RAW_CLOUD_TOPIC
export PLANNER_CLOUD_TOPIC="${PLANNER_CLOUD_TOPIC:-/maritime/obstacles_cloud}"
if [ "${PERCEPTION_SOURCE}" = "lidar" ]; then
	export LIDAR_CLOUD_TOPIC="${LIDAR_CLOUD_TOPIC:-${RAW_CLOUD_TOPIC}}"
else
	export LIDAR_CLOUD_TOPIC="${LIDAR_CLOUD_TOPIC:-/maritime/lidar_points}"
fi
export LIDAR_SCAN_TOPIC="${LIDAR_SCAN_TOPIC:-/maritime/lidar_scan}"
if [ "${PERCEPTION_SOURCE}" = "depth" ]; then
	export DEPTH_CLOUD_TOPIC="${DEPTH_CLOUD_TOPIC:-${RAW_CLOUD_TOPIC}}"
else
	export DEPTH_CLOUD_TOPIC="${DEPTH_CLOUD_TOPIC:-/maritime_depth_camera/points}"
fi
export OCCUPANCY_TOPIC="${OCCUPANCY_TOPIC:-/grid_map/occupancy_inflate}"
if [ "${PERCEPTION_SOURCE}" = "lidar" ]; then
	export MARITIME_ENABLE_LIDAR_DEBUG_MARKERS="${MARITIME_ENABLE_LIDAR_DEBUG_MARKERS:-true}"
else
	export MARITIME_ENABLE_LIDAR_DEBUG_MARKERS="${MARITIME_ENABLE_LIDAR_DEBUG_MARKERS:-false}"
fi
export MARITIME_LIDAR_DEBUG_RAY_STRIDE="${MARITIME_LIDAR_DEBUG_RAY_STRIDE:-2}"
export MARITIME_LIDAR_DEBUG_RAY_MAX="${MARITIME_LIDAR_DEBUG_RAY_MAX:-90}"
export MARITIME_LIDAR_VIS_RANGE_M="${MARITIME_LIDAR_VIS_RANGE_M:-40.0}"

export F250_ROUTE="${ROUTE_ID}"
export F250_ROUTE_PROFILE="${ROUTE_PROFILE}"
export F250_ROUTE_EFFECTIVE_SCENE="${ROUTE_EFFECTIVE_SCENE}"
export ROUTE_ID ROUTE_NAME ROUTE_PROFILE ROUTE_PROFILE_SOURCE ROUTE_SELECTED_BY
export ROUTE_EFFECTIVE_SCENE ROUTE_METADATA_JSON ROUTE_WAYPOINTS_CSV
export ROUTE_WAYPOINT_COUNT ROUTE_FIRST_LABEL ROUTE_FINAL_LABEL ROUTE_TOTAL_LENGTH_M
export ROUTE_LOCKED_BASELINE_COMPATIBILITY
export PX4_SPAWN_X="${ROUTE_SPAWN_X}"
export PX4_SPAWN_Y="${ROUTE_SPAWN_Y}"
export PX4_SPAWN_Z="${ROUTE_SPAWN_Z}"
export PX4_SPAWN_YAW="${ROUTE_SPAWN_YAW}"
export HOVER_X="${ROUTE_HOVER_X}"
export HOVER_Y="${ROUTE_HOVER_Y}"
export HOVER_Z="${ROUTE_HOVER_Z}"
export HOVER_YAW="${ROUTE_HOVER_YAW}"

f250_apply_quick_complex_defaults "${PKG}"

export MARITIME_ENABLE_METRIC_MONITOR="${MARITIME_ENABLE_METRIC_MONITOR:-true}"
export MARITIME_METRIC_OUTPUT_DIR="${METRIC_DIR}"
export MARITIME_METRIC_RUN_LABEL="${RUN_LABEL}"
export MARITIME_METRIC_DISPLAY_LOG="${METRIC_DISPLAY_LOG}"

runtime_patterns=(
	"[r]oscore"
	"[r]osmaster"
	"[r]osout"
	"[r]oslaunch f250_maritime_uav_sim maritime_visual_acceptance.launch"
	"[r]oslaunch f250_maritime_uav_sim maritime_px4_sitl.launch"
	"[r]oslaunch f250_maritime_uav_sim maritime_ego_planner.launch"
	"[g]zclient"
	"[g]zserver"
	"[r]viz .*maritime_visual_acceptance.rviz"
	"[p]x4 .*/build/px4_sitl_default/bin/px4"
	"[m]avros_node"
	"[g]azebo_truth_to_mavros_vision.py"
	"[p]osition_cmd_to_mavros_setpoint.py"
	"[m]aritime_dynamic_obstacles.py"
	"[m]aritime_laser_scan_adapter.py"
	"[m]aritime_lidar_follow_odom.py"
	"[m]aritime_sensor_cloud_adapter.py"
	"[m]aritime_goal_sequence.py"
	"[m]aritime_metric_monitor.py"
	"[m]aritime_scene_markers.py"
	"[m]aritime_flight_path.py"
	"[e]go_planner_node"
	"[t]raj_server"
	"[w]aypoint_generator"
)

screen_session_exists() {
	screen -ls 2>/dev/null | awk '{print $1}' | grep -Eq "(^|[.])${SCREEN_NAME}([[:space:]]|$)"
}

write_startup_health_failure_json() {
	local reason="$1"
	python3 - "${STARTUP_HEALTH_JSON}" "${PERCEPTION_SOURCE}" "${SENSOR_LABEL}" "${reason}" <<'PY'
import json
import sys
from datetime import datetime, timezone
path, sensor, label, reason = sys.argv[1:5]
payload = {
    "schema": "f250_startup_health_gate_v1",
    "ok": False,
    "sensor": sensor,
    "sensor_label": label,
    "reason": reason,
    "checked_at": datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z"),
}
with open(path, "w", encoding="utf-8") as handle:
    json.dump(payload, handle, indent=2, sort_keys=True)
    handle.write("\n")
PY
}

run_startup_health_gate() {
	[ "${F250_SKIP_STARTUP_HEALTH_GATE:-false}" != "true" ] || return 0
	write_status "startup_health"
	mkdir -p "${LOG_DIR}"
	: >"${STARTUP_HEALTH_LOG}"
	{
		echo "sensor=${PERCEPTION_SOURCE}"
		echo "expected_raw_cloud_topic=${RAW_CLOUD_TOPIC}"
		echo "planner_cloud_topic=${PLANNER_CLOUD_TOPIC}"
		echo "occupancy_topic=${OCCUPANCY_TOPIC}"
		echo "check=gzserver"
	} >>"${STARTUP_HEALTH_LOG}"
	if ! pgrep -x gzserver >/dev/null 2>&1; then
		echo "gzserver_running=false" >>"${STARTUP_HEALTH_LOG}"
		write_startup_health_failure_json "gzserver_not_running"
		write_status "startup_health_failed"
		echo "F250 startup health failed: gzserver is not running. Check ${START_LOG} and ${STARTUP_HEALTH_LOG}" >&2
		exit 2
	fi
	echo "gzserver_running=true" >>"${STARTUP_HEALTH_LOG}"

	# shellcheck disable=SC1091
	source /opt/ros/noetic/setup.bash
	if [ -f "${WS}/devel/setup.bash" ]; then
		# shellcheck disable=SC1090
		source "${WS}/devel/setup.bash"
	else
		export ROS_PACKAGE_PATH="${WS}/src:${ROS_PACKAGE_PATH:-}"
	fi

	local gate_lidar_points="${LIDAR_CLOUD_TOPIC:-/maritime/lidar_points}"
	local gate_depth_points="${DEPTH_CLOUD_TOPIC:-/maritime_depth_camera/points}"
	if [ "${PERCEPTION_SOURCE}" = "lidar" ]; then
		gate_lidar_points="${RAW_CLOUD_TOPIC}"
	elif [ "${PERCEPTION_SOURCE}" = "depth" ]; then
		gate_depth_points="${RAW_CLOUD_TOPIC}"
	fi

	set +e
	python3 "${CHECK_GATE}" \
		--sensor "${PERCEPTION_SOURCE}" \
		--timeout-sec "${F250_STARTUP_HEALTH_TIMEOUT_SEC:-45}" \
		--output-json "${STARTUP_HEALTH_JSON}" \
		--planner-cloud-topic "${PLANNER_CLOUD_TOPIC}" \
		--occupancy-topic "${OCCUPANCY_TOPIC}" \
		--lidar-scan-topic "${LIDAR_SCAN_TOPIC}" \
		--lidar-points-topic "${gate_lidar_points}" \
		--depth-points-topic "${gate_depth_points}" \
		>>"${STARTUP_HEALTH_LOG}" 2>&1
	local gate_status=$?
	set -e
	if [ "${gate_status}" -ne 0 ]; then
		write_status "startup_health_failed"
		echo "F250 startup health failed for ${SENSOR_LABEL}. Check ${STARTUP_HEALTH_LOG}" >&2
		exit "${gate_status}"
	fi
	write_status "startup_health_ok"
}

runtime_busy() {
	local busy=0
	: >"${PREFLIGHT_LOG}"
	for pat in "${runtime_patterns[@]}"; do
		if pgrep -af "${pat}" >>"${PREFLIGHT_LOG}" 2>/dev/null; then
			busy=1
		fi
	done
	if command -v screen >/dev/null 2>&1; then
		screen -ls 2>/dev/null | awk '{print $1}' | grep -E '(^|[.])f250_(p0_hover|human|metrics|terminal_showcase)' >>"${PREFLIGHT_LOG}" || true
		if [ -s "${PREFLIGHT_LOG}" ]; then
			busy=1
		fi
	fi
	[ "${busy}" -ne 0 ]
}

write_status() {
	local state="$1"
	local runtime_active="true"
	case "$state" in
	blocked_existing_runtime | blocked_existing_screen | screen_exited_early | prepared_dry_run | startup_health_failed)
		runtime_active="false"
		;;
	esac
	{
		echo "state=${state}"
		echo "task=launch"
		echo "runtime_active=${runtime_active}"
		echo "updated_at=$(date -Is)"
		echo "run_dir=${RUN_DIR}"
		echo "screen_name=${SCREEN_NAME}"
		echo "start_log=${START_LOG}"
		echo "metric_output_dir=${METRIC_DIR}"
		echo "metric_display_log=${METRIC_DISPLAY_LOG}"
		echo "startup_health_json=${STARTUP_HEALTH_JSON}"
		echo "startup_health_log=${STARTUP_HEALTH_LOG}"
		echo "base_scene_config=${SCENE_CONFIG_FIXED}"
		echo "scene_config=${SCENE_CONFIG}"
		echo "world=${WORLD}"
		echo "vehicle=${MARITIME_VEHICLE}"
		echo "route_id=${ROUTE_ID}"
		echo "route_name=${ROUTE_NAME}"
		echo "route_profile=${ROUTE_PROFILE}"
		echo "route_profile_source=${ROUTE_PROFILE_SOURCE}"
		echo "route_selected_by=${ROUTE_SELECTED_BY}"
		echo "route_effective_scene=${ROUTE_EFFECTIVE_SCENE}"
		echo "route_metadata_json=${ROUTE_METADATA_JSON}"
		echo "route_waypoints_csv=${ROUTE_WAYPOINTS_CSV}"
		echo "route_waypoint_count=${ROUTE_WAYPOINT_COUNT}"
		echo "route_first_label=${ROUTE_FIRST_LABEL}"
		echo "route_final_label=${ROUTE_FINAL_LABEL}"
		echo "route_total_length_m=${ROUTE_TOTAL_LENGTH_M}"
		echo "quick_complex_profile_id=${QUICK_COMPLEX_PROFILE_ID}"
		echo "quick_complex_baseline=${QUICK_COMPLEX_BASELINE}"
		echo "params_file=${PARAMS_FILE}"
		echo "route_locked_baseline_compatibility=${ROUTE_LOCKED_BASELINE_COMPATIBILITY}"
		echo "sensor=${PERCEPTION_SOURCE}"
		echo "sensor_label=${SENSOR_LABEL}"
		echo "perception_source=${PERCEPTION_SOURCE}"
		echo "sensor_selected_by=${SENSOR_SELECTED_BY}"
		echo "sensor_preference_file=${SENSOR_PREFERENCE_FILE}"
		echo "planner_cloud_topic=${PLANNER_CLOUD_TOPIC}"
		echo "raw_cloud_topic=${RAW_CLOUD_TOPIC}"
		echo "lidar_cloud_topic=${LIDAR_CLOUD_TOPIC}"
		echo "lidar_scan_topic=${LIDAR_SCAN_TOPIC}"
		echo "depth_cloud_topic=${DEPTH_CLOUD_TOPIC}"
		echo "occupancy_topic=${OCCUPANCY_TOPIC}"
		echo "start_paused=${MARITIME_START_PAUSED}"
		echo "hover_target=${HOVER_X},${HOVER_Y},${HOVER_Z},${HOVER_YAW}"
		echo "spawn_target=${PX4_SPAWN_X},${PX4_SPAWN_Y},${PX4_SPAWN_Z},${PX4_SPAWN_YAW}"
		echo "stop_script=${STOP_SCRIPT}"
	} >"${STATUS_FILE}"
	mkdir -p "$(dirname "${ACTIVE_STATUS}")"
	cp "${STATUS_FILE}" "${ACTIVE_STATUS}"
}

write_export() {
	printf 'export %s=%q\n' "$1" "$2" >>"${LAUNCH_ENV}"
}

write_snapshots() {
	{
		echo "created_at=$(date -Is)"
		echo "project_root=${PROJECT_ROOT}"
		echo "workspace=${WS}"
		echo "package=${PKG}"
		echo "start_script=${START_SCRIPT}"
		echo "stop_script=${STOP_SCRIPT}"
		echo "map_authority=${MAP_AUTHORITY}"
		echo "quick_complex_profile_id=${QUICK_COMPLEX_PROFILE_ID}"
		echo "quick_complex_baseline=${QUICK_COMPLEX_BASELINE}"
		echo "quick_complex_description=${QUICK_COMPLEX_DESCRIPTION}"
		echo "run_dir=${RUN_DIR}"
		echo "screen_name=${SCREEN_NAME}"
		echo "route_id=${ROUTE_ID}"
		echo "route_name=${ROUTE_NAME}"
		echo "route_profile=${ROUTE_PROFILE}"
		echo "route_profile_source=${ROUTE_PROFILE_SOURCE}"
		echo "route_effective_scene=${ROUTE_EFFECTIVE_SCENE}"
		echo "route_metadata_json=${ROUTE_METADATA_JSON}"
		echo "route_waypoints_csv=${ROUTE_WAYPOINTS_CSV}"
		echo "route_waypoint_count=${ROUTE_WAYPOINT_COUNT}"
		echo "route_final_label=${ROUTE_FINAL_LABEL}"
		echo "sensor=${PERCEPTION_SOURCE}"
		echo "sensor_label=${SENSOR_LABEL}"
		echo "sensor_selected_by=${SENSOR_SELECTED_BY}"
		echo "sensor_preference_file=${SENSOR_PREFERENCE_FILE}"
		echo "host=$(hostname)"
		echo "user=$(id -un)"
	} >"${PROVENANCE_FILE}"

	{
		echo "MARITIME_VEHICLE=${MARITIME_VEHICLE}"
		echo "SCENE_LEVEL=${SCENE_LEVEL}"
		echo "BASE_SCENE_CONFIG=${SCENE_CONFIG_FIXED}"
		echo "SCENE_CONFIG=${SCENE_CONFIG}"
		echo "WORLD=${WORLD}"
		echo "ROUTE_ID=${ROUTE_ID}"
		echo "ROUTE_NAME=${ROUTE_NAME}"
		echo "ROUTE_PROFILE=${ROUTE_PROFILE}"
		echo "ROUTE_PROFILE_SOURCE=${ROUTE_PROFILE_SOURCE}"
		echo "ROUTE_SELECTED_BY=${ROUTE_SELECTED_BY}"
		echo "ROUTE_EFFECTIVE_SCENE=${ROUTE_EFFECTIVE_SCENE}"
		echo "ROUTE_METADATA_JSON=${ROUTE_METADATA_JSON}"
		echo "ROUTE_WAYPOINTS_CSV=${ROUTE_WAYPOINTS_CSV}"
		echo "ROUTE_WAYPOINT_COUNT=${ROUTE_WAYPOINT_COUNT}"
		echo "ROUTE_FIRST_LABEL=${ROUTE_FIRST_LABEL}"
		echo "ROUTE_FINAL_LABEL=${ROUTE_FINAL_LABEL}"
		echo "ROUTE_TOTAL_LENGTH_M=${ROUTE_TOTAL_LENGTH_M}"
		echo "ROUTE_LOCKED_BASELINE_COMPATIBILITY=${ROUTE_LOCKED_BASELINE_COMPATIBILITY}"
		echo "SENSOR=${PERCEPTION_SOURCE}"
		echo "SENSOR_LABEL=${SENSOR_LABEL}"
		echo "SENSOR_SELECTED_BY=${SENSOR_SELECTED_BY}"
		echo "SENSOR_PREFERENCE_FILE=${SENSOR_PREFERENCE_FILE}"
		echo "ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}"
		echo "GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}"
		echo "GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}"
		echo "LD_LIBRARY_PATH=${LD_LIBRARY_PATH}"
		echo "PERCEPTION_SOURCE=${PERCEPTION_SOURCE}"
		echo "PLANNER_CLOUD_TOPIC=${PLANNER_CLOUD_TOPIC}"
		echo "LIDAR_CLOUD_TOPIC=${LIDAR_CLOUD_TOPIC}"
		echo "LIDAR_SCAN_TOPIC=${LIDAR_SCAN_TOPIC}"
		echo "DEPTH_CLOUD_TOPIC=${DEPTH_CLOUD_TOPIC}"
		echo "OCCUPANCY_TOPIC=${OCCUPANCY_TOPIC}"
		echo "DYNAMIC_MODE=${DYNAMIC_MODE}"
		echo "LANDING_MODE=${LANDING_MODE}"
		echo "MARITIME_START_PAUSED=${MARITIME_START_PAUSED}"
		echo "MARITIME_START_TOPIC=${MARITIME_START_TOPIC}"
		echo "AUTO_OFFBOARD_ARM=${AUTO_OFFBOARD_ARM}"
		echo "REQUIRE_PLANNER_COMMAND_FOR_OFFBOARD=${REQUIRE_PLANNER_COMMAND_FOR_OFFBOARD}"
		echo "ENABLE_RVIZ=${ENABLE_RVIZ}"
		echo "PX4_GUI=${PX4_GUI}"
		echo "PX4_NO_FOLLOW_MODE=${PX4_NO_FOLLOW_MODE}"
		echo "F250_PROJECT_ROOT=${F250_PROJECT_ROOT}"
		echo "F250_PX4_ROOT=${F250_PX4_ROOT}"
		echo "RAW_CLOUD_TOPIC=${RAW_CLOUD_TOPIC}"
		echo "MARITIME_ENABLE_LIDAR_DEBUG_MARKERS=${MARITIME_ENABLE_LIDAR_DEBUG_MARKERS}"
		echo "MARITIME_LIDAR_DEBUG_RAY_STRIDE=${MARITIME_LIDAR_DEBUG_RAY_STRIDE}"
		echo "MARITIME_LIDAR_DEBUG_RAY_MAX=${MARITIME_LIDAR_DEBUG_RAY_MAX}"
		echo "MARITIME_LIDAR_VIS_RANGE_M=${MARITIME_LIDAR_VIS_RANGE_M}"
		echo "PX4_SPAWN_X=${PX4_SPAWN_X}"
		echo "PX4_SPAWN_Y=${PX4_SPAWN_Y}"
		echo "PX4_SPAWN_Z=${PX4_SPAWN_Z}"
		echo "PX4_SPAWN_YAW=${PX4_SPAWN_YAW}"
		echo "HOVER_X=${HOVER_X}"
		echo "HOVER_Y=${HOVER_Y}"
		echo "F250_QUICK_COMPLEX_PROFILE_ID=${QUICK_COMPLEX_PROFILE_ID}"
		echo "F250_QUICK_COMPLEX_BASELINE=${QUICK_COMPLEX_BASELINE}"
		echo "F250_QUICK_COMPLEX_DESCRIPTION=${QUICK_COMPLEX_DESCRIPTION}"
		echo "HOVER_Z=${HOVER_Z}"
		echo "HOVER_YAW=${HOVER_YAW}"
		echo "MAP_SIZE_X=${MAP_SIZE_X}"
		echo "MAP_SIZE_Y=${MAP_SIZE_Y}"
		echo "MAP_SIZE_Z=${MAP_SIZE_Z}"
		echo "EGO_FEASIBILITY_TOLERANCE=${EGO_FEASIBILITY_TOLERANCE}"
		echo "EGO_GRID_MAP_RESOLUTION=${EGO_GRID_MAP_RESOLUTION}"
		echo "EGO_MAX_VEL=${EGO_MAX_VEL}"
		echo "EGO_MAX_ACC=${EGO_MAX_ACC}"
		echo "EGO_MAX_JERK=${EGO_MAX_JERK}"
		echo "EGO_CONTROL_POINTS_DISTANCE=${EGO_CONTROL_POINTS_DISTANCE}"
		echo "EGO_PLANNING_HORIZON=${EGO_PLANNING_HORIZON}"
		echo "EGO_LOCAL_UPDATE_RANGE_X=${EGO_LOCAL_UPDATE_RANGE_X}"
		echo "EGO_LOCAL_UPDATE_RANGE_Y=${EGO_LOCAL_UPDATE_RANGE_Y}"
		echo "EGO_LOCAL_UPDATE_RANGE_Z=${EGO_LOCAL_UPDATE_RANGE_Z}"
		echo "EGO_VIRTUAL_CEIL_HEIGHT=${EGO_VIRTUAL_CEIL_HEIGHT}"
		echo "EGO_VISUALIZATION_TRUNCATE_HEIGHT=${EGO_VISUALIZATION_TRUNCATE_HEIGHT}"
		echo "EGO_OBSTACLES_INFLATION=${EGO_OBSTACLES_INFLATION}"
		echo "EGO_COLLISION_DIST0=${EGO_COLLISION_DIST0}"
		echo "EGO_LAMBDA_SMOOTH=${EGO_LAMBDA_SMOOTH}"
		echo "EGO_LAMBDA_COLLISION=${EGO_LAMBDA_COLLISION}"
		echo "EGO_LAMBDA_FEASIBILITY=${EGO_LAMBDA_FEASIBILITY}"
		echo "EGO_LAMBDA_FITNESS=${EGO_LAMBDA_FITNESS}"
		echo "MARITIME_ENABLE_METRIC_MONITOR=${MARITIME_ENABLE_METRIC_MONITOR}"
		echo "MARITIME_METRIC_OUTPUT_DIR=${MARITIME_METRIC_OUTPUT_DIR}"
		echo "MARITIME_METRIC_RUN_LABEL=${MARITIME_METRIC_RUN_LABEL}"
		echo "MARITIME_METRIC_DISPLAY_LOG=${MARITIME_METRIC_DISPLAY_LOG}"
	} >"${PARAMS_FILE}"

	: >"${LAUNCH_ENV}"
	for name in \
		DISPLAY LIBGL_ALWAYS_SOFTWARE QT_X11_NO_MITSHM DISABLE_ROS1_EOL_WARNINGS \
		F250_PROJECT_ROOT F250_PX4_ROOT ROS_PACKAGE_PATH GAZEBO_MODEL_PATH \
		GAZEBO_PLUGIN_PATH LD_LIBRARY_PATH \
		MARITIME_VEHICLE SCENE_LEVEL SCENE_CONFIG WORLD PERCEPTION_SOURCE DYNAMIC_MODE \
		F250_ROUTE F250_ROUTE_PROFILE F250_ROUTE_EFFECTIVE_SCENE \
		ROUTE_ID ROUTE_NAME ROUTE_PROFILE ROUTE_PROFILE_SOURCE ROUTE_SELECTED_BY \
		ROUTE_EFFECTIVE_SCENE ROUTE_METADATA_JSON ROUTE_WAYPOINTS_CSV \
		ROUTE_WAYPOINT_COUNT ROUTE_FIRST_LABEL ROUTE_FINAL_LABEL ROUTE_TOTAL_LENGTH_M \
		F250_QUICK_COMPLEX_PROFILE_ID F250_QUICK_COMPLEX_BASELINE F250_QUICK_COMPLEX_DESCRIPTION \
		ROUTE_LOCKED_BASELINE_COMPATIBILITY \
		SENSOR_LABEL SENSOR_SELECTED_BY SENSOR_PREFERENCE_FILE \
		PLANNER_CLOUD_TOPIC LIDAR_CLOUD_TOPIC LIDAR_SCAN_TOPIC DEPTH_CLOUD_TOPIC OCCUPANCY_TOPIC \
		LANDING_MODE AUTO_OFFBOARD_ARM REQUIRE_PLANNER_COMMAND_FOR_OFFBOARD \
		MARITIME_START_PAUSED MARITIME_START_TOPIC ENABLE_RVIZ PX4_GUI PX4_NO_FOLLOW_MODE RAW_CLOUD_TOPIC \
		MARITIME_ENABLE_LIDAR_DEBUG_MARKERS MARITIME_LIDAR_DEBUG_RAY_STRIDE \
		MARITIME_LIDAR_DEBUG_RAY_MAX MARITIME_LIDAR_VIS_RANGE_M \
		PX4_SPAWN_X PX4_SPAWN_Y PX4_SPAWN_Z PX4_SPAWN_YAW HOVER_X HOVER_Y HOVER_Z HOVER_YAW \
		MAP_SIZE_X MAP_SIZE_Y MAP_SIZE_Z EGO_FEASIBILITY_TOLERANCE EGO_GRID_MAP_RESOLUTION \
		EGO_MAX_VEL EGO_MAX_ACC EGO_MAX_JERK EGO_CONTROL_POINTS_DISTANCE EGO_PLANNING_HORIZON \
		EGO_LOCAL_UPDATE_RANGE_X EGO_LOCAL_UPDATE_RANGE_Y EGO_LOCAL_UPDATE_RANGE_Z \
		EGO_VIRTUAL_CEIL_HEIGHT EGO_VISUALIZATION_TRUNCATE_HEIGHT EGO_OBSTACLES_INFLATION \
		EGO_COLLISION_DIST0 EGO_LAMBDA_SMOOTH EGO_LAMBDA_COLLISION EGO_LAMBDA_FEASIBILITY \
		EGO_LAMBDA_FITNESS MARITIME_ENABLE_METRIC_MONITOR \
		MARITIME_METRIC_OUTPUT_DIR MARITIME_METRIC_RUN_LABEL MARITIME_METRIC_DISPLAY_LOG; do
		write_export "${name}" "${!name}"
	done
	chmod 0644 "${LAUNCH_ENV}"

	cat >"${LAUNCH_CMD}" <<EOF
#!/usr/bin/env bash
set -euo pipefail
source "${LAUNCH_ENV}"
exec > >(tee -a "${START_LOG}") 2>&1
echo "[f250-p0-hover] started at \$(date -Is)"
echo "[f250-p0-hover] run_dir=${RUN_DIR}"
echo "[f250-p0-hover] sensor=${PERCEPTION_SOURCE}"
echo "[f250-p0-hover] route=${ROUTE_NAME} (${ROUTE_ID})"
echo "[f250-p0-hover] route_profile=${ROUTE_PROFILE}"
echo "[f250-p0-hover] screen_name=${SCREEN_NAME}"
echo "[f250-p0-hover] stop=${STOP_SCRIPT}"
echo "[f250-p0-hover] route release=${PKG}/scripts/start_demo_waypoints.sh"
exec "${START_SCRIPT}" quick-complex
EOF
	chmod 0755 "${LAUNCH_CMD}"
}

write_snapshots
write_status "prepared"

if [ "${DRY_RUN}" = "true" ]; then
	write_status "prepared_dry_run"
	cat <<EOF
F250 environment dry run prepared.
Sensor: ${SENSOR_LABEL}
Route: ${ROUTE_NAME}
No runtime started.
EOF
	exit 0
fi

if [ "${F250_PRESTOP:-false}" = "true" ]; then
	"${STOP_SCRIPT}"
	mkdir -p "${LOG_DIR}" "${METRIC_DIR}"
	: >"${PREFLIGHT_LOG}"
	: >"${METRIC_DISPLAY_LOG}"
	python3 "${ROUTE_HELPER}" "${ROUTE_HELPER_ARGS[@]}" >/dev/null
	# shellcheck source=/dev/null
	source "${ROUTE_STATUS_ENV}"
	write_snapshots
	write_status "prepared"
elif runtime_busy && [ "${F250_ALLOW_EXISTING_RUNTIME:-false}" != "true" ]; then
	write_status "blocked_existing_runtime"
	echo "Existing maritime runtime was detected. Details: ${PREFLIGHT_LOG}" >&2
	echo "Run ${STOP_SCRIPT} first, or set F250_PRESTOP=true / F250_ALLOW_EXISTING_RUNTIME=true." >&2
	exit 2
fi

if screen_session_exists; then
	write_status "blocked_existing_screen"
	fail "screen session already exists: ${SCREEN_NAME}"
fi

screen -dmS "${SCREEN_NAME}" "${LAUNCH_CMD}"
sleep "${F250_SCREEN_CONFIRM_SEC:-2}"

if ! screen_session_exists; then
	write_status "screen_exited_early"
	echo "screen session exited early. Check ${START_LOG}" >&2
	exit 2
fi

write_status "screen_started"
if [ "${F250_AUTO_LAYOUT:-true}" = "true" ]; then
	(DISPLAY="${DISPLAY:-:0}" "${LAYOUT_SCRIPT}" --kind visual --wait-sec "${F250_LAYOUT_WAIT_SEC:-8}" >>"${RUN_DIR}/logs/window_layout.log" 2>&1 || true) &
fi

sleep "${F250_P0_READY_DELAY_SEC:-10}"
if ! screen_session_exists; then
	write_status "screen_exited_early"
	echo "screen session exited before P0 hover ready. Check ${START_LOG}" >&2
	exit 2
fi
run_startup_health_gate
write_status "hover_ready"

cat <<EOF
F250 environment ready.
Sensor: ${SENSOR_LABEL}
Route: ${ROUTE_NAME}
View: Gazebo + RViz + PX4 GUI
Next: Run Route or Run FC
Stop: use Stop All
EOF
