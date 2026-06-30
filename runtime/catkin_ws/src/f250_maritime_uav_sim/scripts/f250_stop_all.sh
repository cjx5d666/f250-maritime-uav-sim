#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/f250_paths.sh"
PROJECT_ROOT="$(f250_resolve_project_root "${SCRIPT_DIR}")"
RUNTIME_STATE_DIR="${F250_RUNTIME_STATE_DIR:-${PROJECT_ROOT}/runtime_state}"
RUN_ROOT="${RUN_ROOT:-${RUNTIME_STATE_DIR}/work}"
ACTIVE_STATUS="${F250_ACTIVE_TASK_ENV:-${RUNTIME_STATE_DIR}/active_task.env}"
STAMP="$(date +%Y%m%d_%H%M%S)"
STOP_LOG="${STOP_LOG:-${RUN_ROOT}/stop_latest.log}"

usage() {
	cat <<EOF
Usage:
  ${0} [--dry-run]
  ${0} --help

Stops the F250 maritime quick-complex simulation chain:
  ROS master/launch, Gazebo, PX4 SITL, MAVROS, EGO-Planner, RViz,
  metric monitor, and F250 human-script screen sessions.

It preserves run/result directories and intentionally does not match unrelated tools,
ssh/sshd, or general workspace shells.

Useful environment overrides:
  RUN_ROOT=...                  default: ${PROJECT_ROOT}/runtime_state/work
  STOP_LOG=...                  default: <RUN_ROOT>/stop_latest.log
  F250_STOP_TERM_WAIT_SEC=2
  F250_STOP_FORCE_KILL=true
  F250_STOP_SCREENS=true
  F250_PROJECT_ROOT=...         override project root detected from script path
EOF
}

DRY_RUN="${F250_STOP_DRY_RUN:-false}"
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
	*)
		echo "Unknown argument: $1" >&2
		echo "Run ${0} --help" >&2
		exit 2
		;;
	esac
done

mkdir -p "$(dirname "${STOP_LOG}")"
: >"${STOP_LOG}"

TERM_WAIT="${F250_STOP_TERM_WAIT_SEC:-2}"
FORCE_KILL="${F250_STOP_FORCE_KILL:-true}"
STOP_SCREENS="${F250_STOP_SCREENS:-true}"
QUIET="${F250_STOP_QUIET:-true}"

patterns=(
	"[r]oscore"
	"[r]osmaster"
	"[r]osout"
	"[r]oslaunch f250_maritime_uav_sim maritime_visual_acceptance.launch"
	"[r]oslaunch f250_maritime_uav_sim maritime_px4_sitl.launch"
	"[r]oslaunch f250_maritime_uav_sim maritime_ego_planner.launch"
	"[r]oslaunch f250_maritime_uav_sim maritime_obstacles.launch"
	"[g]zclient"
	"[g]zserver"
	"[r]viz .*maritime_visual_acceptance.rviz"
	"[p]x4 .*/build/px4_sitl_default/bin/px4"
	"[m]avros_node"
	"[g]azebo_truth_to_mavros_vision.py"
	"[m]avros_odom_to_tf.py"
	"[p]osition_cmd_to_mavros_setpoint.py"
	"[m]aritime_dynamic_obstacles.py"
	"[m]aritime_laser_scan_adapter.py"
	"[m]aritime_lidar_follow_odom.py"
	"[m]aritime_sensor_cloud_adapter.py"
	"[m]aritime_goal_sequence.py"
	"[m]aritime_metric_monitor.py"
	"[m]aritime_scene_markers.py"
	"[m]aritime_inflated_obstacle_markers.py"
	"[m]aritime_flight_path.py"
	"[f]250_check_perception_gate.py"
	"[f]250_quick_complex_record.py"
	"[f]250_route_human_summary.py"
	"[f]250_fc_3_10_steady_state.py"
	"[e]go_planner_node"
	"[t]raj_server"
	"[w]aypoint_generator"
	"[t]ail -n [+]1 -f ${RUN_ROOT}/.*/realtime_metric_live\\.txt"
	"[t]ail -f ${RUN_ROOT}/.*/realtime_metric_live\\.txt"
	"[t]ail -n [+]1 -F ${RUN_ROOT}/.*/realtime_metric_live\\.txt"
	"[t]ail -F ${RUN_ROOT}/.*/realtime_metric_live\\.txt"
	"[t]ail -n [+]1 -F ${RUN_ROOT}/.*/route_terminal\\.log"
	"[t]ail -F ${RUN_ROOT}/.*/route_terminal\\.log"
	"[t]ail -n [+]1 -F ${RUN_ROOT}/.*/fc_3_10_terminal\\.log"
	"[t]ail -F ${RUN_ROOT}/.*/fc_3_10_terminal\\.log"
	"[f]250_run_p0_p8_route.sh"
	"[f]250_run_fc_3_10_steady_state.sh"
)

log() {
	echo "$*" >>"${STOP_LOG}"
	if [ "${QUIET}" != "true" ]; then
		echo "$*"
	fi
}

list_matches() {
	local pat="$1"
	pgrep -af "${pat}" 2>/dev/null || true
}

terminate_pattern() {
	local pat="$1"
	local matches
	matches="$(list_matches "${pat}")"
	if [ -z "${matches}" ]; then
		return 0
	fi

	log "pattern: ${pat}"
	log "${matches}"
	if [ "${DRY_RUN}" = "true" ]; then
		return 0
	fi
	pkill -TERM -f "${pat}" >/dev/null 2>&1 || true
}

kill_pattern() {
	local pat="$1"
	local matches
	matches="$(list_matches "${pat}")"
	if [ -z "${matches}" ]; then
		return 0
	fi

	log "force_pattern: ${pat}"
	log "${matches}"
	if [ "${DRY_RUN}" = "true" ]; then
		return 0
	fi
	pkill -KILL -f "${pat}" >/dev/null 2>&1 || true
}

stop_screen_sessions() {
	[ "${STOP_SCREENS}" = "true" ] || return 0
	command -v screen >/dev/null 2>&1 || return 0

	local sessions
	sessions="$(screen -ls 2>/dev/null | awk '
    /^[[:space:]]*[0-9]+[.]/ {
      session = $1
      name = session
      sub(/^[0-9]+[.]/, "", name)
      if (name ~ /^f250_(p0_hover|human|metrics|terminal_showcase|route_worker|route_metrics|fc310_worker|fc310_metrics)/) {
        print session
      }
    }' || true)"

	if [ -z "${sessions}" ]; then
		return 0
	fi

	log "screen_sessions:"
	log "${sessions}"
	if [ "${DRY_RUN}" = "true" ]; then
		return 0
	fi

	while IFS= read -r session; do
		[ -n "${session}" ] || continue
		screen -S "${session}" -X quit >/dev/null 2>&1 || true
	done <<<"${sessions}"
}

clean_python_caches() {
	[ "${F250_CLEAN_PYCACHE_AFTER_STOP:-true}" = "true" ] || return 0
	[ -d "${PROJECT_ROOT}" ] || return 0
	find "${PROJECT_ROOT}" -type d -name __pycache__ -prune -exec rm -rf -- {} + 2>/dev/null || true
	find "${PROJECT_ROOT}" -type f -name '*.pyc' -exec rm -f -- {} + 2>/dev/null || true
}

echo "F250 stop requested."
log "f250_stop_all start $(date -Is)"
log "dry_run=${DRY_RUN}"
log "run_root=${RUN_ROOT}"

stop_screen_sessions

for pat in "${patterns[@]}"; do
	terminate_pattern "${pat}"
done

if [ "${DRY_RUN}" != "true" ]; then
	sleep "${TERM_WAIT}"
fi

if [ "${FORCE_KILL}" = "true" ]; then
	for pat in "${patterns[@]}"; do
		kill_pattern "${pat}"
	done
fi

log "f250_stop_all end $(date -Is)"

current_status="${ACTIVE_STATUS}"
if [ "${DRY_RUN}" != "true" ] && [ -e "${current_status}" ]; then
	{
		echo "state=stop_requested"
		echo "updated_at=$(date -Is)"
		echo "stop_log=${STOP_LOG}"
	} >>"${current_status}"
fi

if [ "${DRY_RUN}" = "true" ]; then
	echo "F250 stop dry run complete."
else
	echo "F250 simulation stopped."
fi

if [ "${DRY_RUN}" != "true" ] && [ "${F250_CLEAN_RUNS_AFTER_STOP:-true}" = "true" ]; then
	F250_KEEP_ACTIVE_TASK_DIR=false F250_CLEANUP_QUIET="${F250_CLEANUP_QUIET:-true}" "${SCRIPT_DIR}/f250_cleanup_runs.sh" || true
fi

if [ "${DRY_RUN}" != "true" ]; then
	mkdir -p "$(dirname "${ACTIVE_STATUS}")"
	{
		echo "state=stopped"
		echo "task=stopped"
		echo "runtime_active=false"
		echo "updated_at=$(date -Is)"
		echo "note=no_active_runtime"
		if [ -f "${STOP_LOG}" ]; then
			echo "stop_log=${STOP_LOG}"
			echo "stop_log_status=retained"
		else
			echo "stop_log_status=cleaned"
		fi
	} >"${ACTIVE_STATUS}"
	clean_python_caches
fi
if [ -f "${STOP_LOG}" ]; then
	echo "Stop log: ${STOP_LOG}"
else
	echo "Stop log cleaned."
fi
