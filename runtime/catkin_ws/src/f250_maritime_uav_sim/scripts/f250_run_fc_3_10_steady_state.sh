#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/f250_paths.sh"
PROJECT_ROOT="$(f250_resolve_project_root "${SCRIPT_DIR}")"
WS="${PROJECT_ROOT}/catkin_ws"
PKG="$(f250_resolve_package_root "${SCRIPT_DIR}" "${PROJECT_ROOT}")"
HELPER="${PKG}/scripts/f250_fc_3_10_steady_state.py"
PUBLISH_CURRENT_REVIEW="${PROJECT_ROOT}/maintenance/publish_current_review.py"
RUNTIME_STATE_DIR="${F250_RUNTIME_STATE_DIR:-${PROJECT_ROOT}/runtime_state}"
RUN_ROOT="${RUN_ROOT:-${RUNTIME_STATE_DIR}/work}"
DEFAULT_CURRENT_STATUS="${F250_ACTIVE_TASK_ENV:-${RUNTIME_STATE_DIR}/active_task.env}"
CURRENT_STATUS="${CURRENT_STATUS:-}"
STAMP="$(date +%Y%m%d_%H%M%S)"
SCRIPT_PATH="$(readlink -f "$0")"

usage() {
	cat <<EOF
Usage:
  ${0} [--dry-run]
  ${0} [--geometry-check]
  ${0} [--speed-only] [--disable-velocity-prealign]
  ${0} [--foreground]
  ${0} --help

Runs the F250-only Metric 3.10 FC steady-state test from an already running P0
hover stack. It does not start PX4, Gazebo, ROS, RViz, or route waypoints. It
derives safe FC geometry from the authoritative map package, sends position and
yaw targets through the existing /planning/pos_cmd ->
/mavros/setpoint_position/local chain, switches the bridge to raw-local
XY-velocity / Z-position-hold mode for velocity windows, samples actual state from
/mavros/local_position/odom, evaluates only steady-state windows, then commands
P0 hover again.

Default tasks:
  geometry: P0 from route_waypoints.csv, z fixed at 10 m, u is opposite the
            ship-to-three-islands direction.
  velocity: 2 m/s, 10 windows = 5 AB round trips. Default geometry uses
            A=P0, B=P0+60u. AB endpoints/segments are audited against
            authoritative planner obstacles, and L can be adjusted for
            settle/eval time or clearance.
  position: 10 decagon vertices around C=P0+10u. The first target is the next
            planned vertex after P0, and the last target returns to P0. Error
            uses 3D steady-position deviation over the planned 3D target step.
  yaw: +90/+180/+270/+360, -90/-180/-270/-360, +180, 0.
  Metric 3.10 is FC-only steady-state evidence, not route/planner acceptance.

Default human mode:
  The FC test worker runs in a background screen/nohup worker and opens a
  separate metrics terminal or screen that tails fc_3_10_terminal.log. The
  calling terminal prints only a short startup summary. Use --foreground for the
  old blocking behavior.

Outputs:
  fc_3_10_summary.json
  fc_3_10_samples.csv
  fc_3_10_phases.csv
  fc_3_10_geometry_audit.json
  fc_3_10_decagon_points.csv
  fc_3_10_terminal.log
  status.env

Useful environment overrides:
  RUN_ROOT=...                  default: ${PROJECT_ROOT}/runtime_state/work
  RUN_LABEL=...                 default: f250_fc_3_10_steady_state_<timestamp>
  RUN_DIR=...                   explicit output directory under RUN_ROOT
  CURRENT_STATUS=...            default: ${PROJECT_ROOT}/runtime_state/active_task.env
  HOVER_TARGET=x,y,z,yaw        default: read from runtime_state/active_task.env, else P0
  F250_ALLOW_RUN_DIR_REUSE=true allow reusing RUN_DIR
  F250_FC_3_10_ARGS='...'       extra helper args
  F250_PUBLISH_CURRENT_REVIEW=true|false default: true for real successful runs
  F250_FC_SPEED_ONLY=true|false run only the 10 velocity windows
  F250_DISABLE_VELOCITY_PREALIGN=true|false disable AB/BA prealign holds; default false
  F250_FC_BACKGROUND=true|false
  F250_OPEN_METRICS_TERMINAL=true|false
  F250_PROJECT_ROOT=...         override project root detected from script path
  MAP_AUTHORITY=...             override authoritative map directory for helper
  ROUTE_WAYPOINTS_CSV=...       current route CSV; first waypoint is FC start

Metric formulas written to JSON:
  Each phase ignores transient time, searches for a stationary window, then
  computes error only on the following eval window. A phase with no stationary
  window is not_settled and is not a formal steady-state metric.
  E_pos_i = norm3d(mean(eval actual_xyz) - target_xyz) / previous_to_current_step_3d * 100
  E_vel_i = abs(mean(eval v_parallel) - target_speed) / target_speed * 100
  e_att_i = abs(mean(eval wrap(actual_yaw - target_yaw))) / yaw_denominator * 100
  e_pos, e_vel, and e_att are the means over their 10 formal windows.
  E3.10_selected = max(mean e_pos, mean e_vel, mean e_att)
  E3.10_2mps is retained for compatibility.
  MAVROS odom twist is rotated body -> world with current yaw before velocity comparison.
  Position denominator policy: XYZ error is normalized by the planned 3D
  distance from the previous position target to the current position target.

Dry-run:
  ${0} --dry-run
  Does not require ROS master and writes synthetic demo outputs.

Geometry check:
  ${0} --geometry-check
  Does not require ROS master; writes authoritative geometry audit only plus empty metric CSV/JSON.
EOF
}

fail() {
	echo "f250_run_fc_3_10_steady_state: $*" >&2
	exit 2
}

env_value() {
	local key="$1"
	local file="$2"
	[ -f "${file}" ] || return 0
	awk -F= -v key="${key}" '$1 == key {sub(/^[^=]*=/, ""); print; exit}' "${file}"
}

write_status() {
	local state="$1"
	local runtime_active="true"
	case "$state" in
	prepared_dry_run | dry_run | geometry_check)
		runtime_active="false"
		;;
	esac
	{
		echo "state=${state}"
		echo "task=flight_control"
		echo "runtime_active=${runtime_active}"
		echo "updated_at=$(date -Is)"
		echo "run_dir=${RUN_DIR}"
		echo "run_label=${RUN_LABEL}"
		echo "vehicle=f250"
		echo "metric=3.10_fc_only_steady_state"
		echo "dry_run=${DRY_RUN}"
		echo "geometry_check=${GEOMETRY_CHECK}"
		echo "speed_only=${SPEED_ONLY}"
		echo "disable_velocity_prealign=${DISABLE_VELOCITY_PREALIGN}"
		echo "source_p0_status=${P0_STATUS_FOR_HELPER:-${CURRENT_STATUS}}"
		echo "source_p0_run_dir=${P0_RUN_DIR}"
		echo "source_p0_sensor=${P0_PERCEPTION_SOURCE}"
		echo "source_p0_raw_cloud_topic=${P0_RAW_CLOUD_TOPIC}"
		echo "route_id=${P0_ROUTE_ID}"
		echo "route_name=${P0_ROUTE_NAME}"
		echo "route_profile=${P0_ROUTE_PROFILE}"
		echo "route_final_label=${P0_ROUTE_FINAL_LABEL}"
		echo "route_waypoints_csv=${ROUTE_WAYPOINTS_CSV}"
		echo "hover_target=${HOVER_TARGET_RESOLVED}"
		echo "summary_json=${SUMMARY_JSON}"
		echo "samples_csv=${SAMPLES_CSV}"
		echo "phase_csv=${PHASE_CSV}"
		echo "geometry_audit_json=${GEOMETRY_JSON}"
		echo "decagon_csv=${DECAGON_CSV}"
		echo "terminal_display_log=${DISPLAY_LOG}"
		echo "route_acceptance_written=false"
	} >"${STATUS_FILE}"
	if [ -n "${CURRENT_STATUS:-}" ] && [ "${CURRENT_STATUS}" != "${STATUS_FILE}" ]; then
		mkdir -p "$(dirname "${CURRENT_STATUS}")"
		cp "${STATUS_FILE}" "${CURRENT_STATUS}"
	fi
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
	if ! rostopic list | grep -qx "/planning/pos_cmd"; then
		fail "missing /planning/pos_cmd; setpoint chain does not look active"
	fi
	if ! rostopic list | grep -qx "/mavros/setpoint_raw/local"; then
		fail "missing /mavros/setpoint_raw/local; raw velocity setpoint chain does not look active"
	fi
}

DRY_RUN="${F250_FC_3_10_DRY_RUN:-false}"
GEOMETRY_CHECK="${F250_FC_3_10_GEOMETRY_CHECK:-false}"
RUN_IN_BACKGROUND="${F250_FC_BACKGROUND:-true}"
SPEED_ONLY="${F250_FC_SPEED_ONLY:-false}"
DISABLE_VELOCITY_PREALIGN="${F250_DISABLE_VELOCITY_PREALIGN:-false}"

terminal_command() {
	local logfile="$1"
	local heading="${2:-F250 FC 3.10 Metrics}"
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

append_terminal_over() {
	[ -n "${DISPLAY_LOG:-}" ] || return 0
	mkdir -p "$(dirname "${DISPLAY_LOG}")"
	if [ -f "${DISPLAY_LOG}" ] && [ "$(tail -n 1 "${DISPLAY_LOG}" 2>/dev/null || true)" = "OVER" ]; then
		return 0
	fi
	printf "\nOVER\n" >>"${DISPLAY_LOG}"
}

publish_current_review() {
	[ "${F250_PUBLISH_CURRENT_REVIEW:-true}" = "true" ] || return 0
	[ "${SPEED_ONLY}" != "true" ] || return 0
	[ "${DRY_RUN}" != "true" ] || return 0
	[ "${GEOMETRY_CHECK}" != "true" ] || return 0
	if [ ! -f "${PUBLISH_CURRENT_REVIEW}" ]; then
		echo "publish_current_review=skipped_missing_tool:${PUBLISH_CURRENT_REVIEW}" >>"${STATUS_FILE}"
		return 0
	fi
	if [ -z "${P0_RUN_DIR:-}" ] || [ ! -d "${P0_RUN_DIR}" ]; then
		echo "publish_current_review=skipped_missing_p0_run:${P0_RUN_DIR:-}" >>"${STATUS_FILE}"
		return 0
	fi
	set +e
	python3 "${PUBLISH_CURRENT_REVIEW}" \
		--kind flight_control \
		--run-dir "${RUN_DIR}" \
		--p0-run-dir "${P0_RUN_DIR}" \
		--quiet \
		>"${RUN_DIR}/logs/publish_current_review.log" 2>&1
	local publish_status=$?
	set -e
	if [ "${publish_status}" -eq 0 ]; then
		echo "publish_current_review=ok" >>"${STATUS_FILE}"
	else
		echo "publish_current_review=warning_status_${publish_status}" >>"${STATUS_FILE}"
		echo "[f250-fc] WARNING current review publish failed; see ${RUN_DIR}/logs/publish_current_review.log" >&2
	fi
	return 0
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
		echo "manual_metrics=tail -F ${logfile}"
	fi
}

layout_metrics_window() {
	[ "${F250_AUTO_LAYOUT:-true}" = "true" ] || return 0
	(DISPLAY="${DISPLAY:-:0}" "${PKG}/scripts/f250_layout_windows.py" --kind metrics --wait-sec "${F250_METRICS_LAYOUT_WAIT_SEC:-1}" --attempts "${F250_METRICS_LAYOUT_ATTEMPTS:-6}" --retry-sec "${F250_METRICS_LAYOUT_RETRY_SEC:-0.5}" >>"${RUN_DIR}/logs/window_layout.log" 2>&1 || true) &
}

launch_fc_worker() {
	local worker_screen="f250_fc310_worker_${RUN_LABEL}"
	local metrics_screen="f250_fc310_metrics_${RUN_LABEL}"
	local worker_log="${RUN_DIR}/fc_3_10_worker.log"
	local metrics_info
	metrics_info="$(open_metrics_terminal "F250 FC 3.10 Metrics" "${DISPLAY_LOG}" "${metrics_screen}")"
	layout_metrics_window
	local worker_info=""
	local worker_pid=""
	local worker_used_screen="false"
	if command -v screen >/dev/null 2>&1; then
		worker_used_screen="true"
		screen -dmS "${worker_screen}" -L -Logfile "${worker_log}" env \
			F250_FC_BACKGROUND=false \
			F250_FC_PRECHECKED=true \
			F250_OPEN_METRICS_TERMINAL=false \
			RUN_ROOT="${RUN_ROOT}" \
			RUN_DIR="${RUN_DIR}" \
			RUN_LABEL="${RUN_LABEL}" \
			CURRENT_STATUS="${CURRENT_STATUS}" \
			F250_FC_P0_RUN_DIR="${P0_RUN_DIR}" \
			F250_FC_P0_STATUS="${P0_STATUS_FOR_HELPER:-${CURRENT_STATUS}}" \
			F250_FC_P0_PERCEPTION_SOURCE="${P0_PERCEPTION_SOURCE}" \
			F250_FC_P0_RAW_CLOUD_TOPIC="${P0_RAW_CLOUD_TOPIC}" \
			ROUTE_WAYPOINTS_CSV="${ROUTE_WAYPOINTS_CSV}" \
			HOVER_TARGET="${HOVER_TARGET_RESOLVED}" \
			F250_FC_3_10_ARGS="${F250_FC_3_10_ARGS:-}" \
			F250_FC_SPEED_ONLY="${SPEED_ONLY}" \
			F250_DISABLE_VELOCITY_PREALIGN="${DISABLE_VELOCITY_PREALIGN}" \
			F250_PUBLISH_CURRENT_REVIEW="${F250_PUBLISH_CURRENT_REVIEW:-true}" \
			F250_ALLOW_RUN_DIR_REUSE=true \
			F250_FC_TERMINAL_LOG_READY=true \
			bash "${SCRIPT_PATH}" --foreground --run-dir "${RUN_DIR}" --current-status "${CURRENT_STATUS}"
		worker_info="worker_screen=${worker_screen}"
	else
		nohup env \
			F250_FC_BACKGROUND=false \
			F250_FC_PRECHECKED=true \
			F250_OPEN_METRICS_TERMINAL=false \
			RUN_ROOT="${RUN_ROOT}" \
			RUN_DIR="${RUN_DIR}" \
			RUN_LABEL="${RUN_LABEL}" \
			CURRENT_STATUS="${CURRENT_STATUS}" \
			F250_FC_P0_RUN_DIR="${P0_RUN_DIR}" \
			F250_FC_P0_STATUS="${P0_STATUS_FOR_HELPER:-${CURRENT_STATUS}}" \
			F250_FC_P0_PERCEPTION_SOURCE="${P0_PERCEPTION_SOURCE}" \
			F250_FC_P0_RAW_CLOUD_TOPIC="${P0_RAW_CLOUD_TOPIC}" \
			ROUTE_WAYPOINTS_CSV="${ROUTE_WAYPOINTS_CSV}" \
			HOVER_TARGET="${HOVER_TARGET_RESOLVED}" \
			F250_FC_3_10_ARGS="${F250_FC_3_10_ARGS:-}" \
			F250_FC_SPEED_ONLY="${SPEED_ONLY}" \
			F250_DISABLE_VELOCITY_PREALIGN="${DISABLE_VELOCITY_PREALIGN}" \
			F250_PUBLISH_CURRENT_REVIEW="${F250_PUBLISH_CURRENT_REVIEW:-true}" \
			F250_ALLOW_RUN_DIR_REUSE=true \
			F250_FC_TERMINAL_LOG_READY=true \
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
		echo "status_env=${STATUS_FILE}"
		echo "terminal_display_log=${DISPLAY_LOG}"
		echo "stop_script=${PKG}/scripts/f250_stop_all.sh"
	} >"${RUN_DIR}/background_worker.env"
	# Worker liveness self-check: a --foreground worker that dies instantly (e.g. a
	# failed precondition) used to leave the operator stuck at background_worker_starting
	# with no log. Confirm the worker is actually alive; if it vanished within a few
	# seconds, surface a failure with a pointer to the captured worker log. This must
	# cover BOTH spawn paths (screen and the nohup fallback), so it is not guarded by
	# the presence of screen.
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
			echo "fc_worker_died_immediately=true"
			echo "worker_screen=${worker_screen}"
			echo "worker_log=${worker_log}"
		} >>"${RUN_DIR}/status.env"
		echo "F250 FC 3.10 worker exited immediately; see ${worker_log}" >&2
		return 1
	fi
	cat <<EOF
F250 FC check started.
Metrics window: F250 FC 3.10 Metrics
Stop: use Stop All
EOF
	if printf "%s\n" "${metrics_info}" | grep -q "manual_metrics"; then
		echo "Metrics window unavailable; use Stop All or inspect current status."
	fi
}

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
	--geometry-check)
		GEOMETRY_CHECK="true"
		RUN_IN_BACKGROUND="false"
		shift
		;;
	--foreground)
		RUN_IN_BACKGROUND="false"
		shift
		;;
	--speed-only)
		SPEED_ONLY="true"
		RUN_IN_BACKGROUND="false"
		F250_PUBLISH_CURRENT_REVIEW="false"
		shift
		;;
	--disable-velocity-prealign)
		DISABLE_VELOCITY_PREALIGN="true"
		shift
		;;
	--run-label)
		RUN_LABEL="$2"
		shift 2
		;;
	--run-dir)
		RUN_DIR="$2"
		shift 2
		;;
	--current-status)
		CURRENT_STATUS="$2"
		shift 2
		;;
	*)
		fail "unknown argument: $1; run ${0} --help"
		;;
	esac
done

[ -d "${WS}" ] || fail "missing workspace: ${WS}"
[ -d "${PKG}" ] || fail "missing package: ${PKG}"
[ -x "${HELPER}" ] || [ -f "${HELPER}" ] || fail "missing helper: ${HELPER}"

mkdir -p "${RUNTIME_STATE_DIR}" "${RUN_ROOT}"
RUNTIME_STATE_DIR="$(cd "${RUNTIME_STATE_DIR}" && pwd -P)"
RUN_ROOT="$(cd "${RUN_ROOT}" && pwd -P)"
DEFAULT_CURRENT_STATUS="${F250_ACTIVE_TASK_ENV:-${RUNTIME_STATE_DIR}/active_task.env}"
if [ -z "${CURRENT_STATUS:-}" ]; then
	CURRENT_STATUS="${DEFAULT_CURRENT_STATUS}"
fi

RUN_LABEL="${RUN_LABEL:-${RUN_LABEL_OVERRIDE:-f250_fc_3_10_steady_state_${STAMP}}}"
if [ -z "${RUN_DIR:-}" ]; then
	RUN_DIR="${RUN_ROOT}/${RUN_LABEL}"
else
	case "${RUN_DIR}" in
	/*) ;;
	*) RUN_DIR="${RUN_ROOT}/${RUN_DIR}" ;;
	esac
	RUN_LABEL="$(basename "${RUN_DIR}")"
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
P0_HOVER_TARGET=""
P0_PERCEPTION_SOURCE=""
P0_RAW_CLOUD_TOPIC=""
P0_ROUTE_ID=""
P0_ROUTE_NAME=""
P0_ROUTE_PROFILE=""
P0_ROUTE_FINAL_LABEL=""
P0_ROUTE_WAYPOINTS_CSV=""
# If a background worker is launched, it must read the original P0 source
# snapshot. CURRENT_STATUS is also the active output status and is overwritten by
# this script, so it is not reliable as P0 provenance after dispatch.
P0_STATUS_INPUT="${F250_FC_P0_STATUS:-${CURRENT_STATUS}}"
if [ -n "${F250_FC_P0_RUN_DIR:-}" ]; then
	P0_RUN_DIR="${F250_FC_P0_RUN_DIR}"
fi
if [ -f "${P0_STATUS_INPUT}" ]; then
	[ -n "${P0_RUN_DIR}" ] || P0_RUN_DIR="$(env_value run_dir "${P0_STATUS_INPUT}")"
	P0_TASK="$(env_value task "${P0_STATUS_INPUT}")"
	P0_VEHICLE="$(env_value vehicle "${P0_STATUS_INPUT}")"
	P0_STATE="$(env_value state "${P0_STATUS_INPUT}")"
	P0_HOVER_TARGET="$(env_value hover_target "${P0_STATUS_INPUT}")"
	P0_PERCEPTION_SOURCE="$(env_value perception_source "${P0_STATUS_INPUT}")"
	P0_RAW_CLOUD_TOPIC="$(env_value raw_cloud_topic "${P0_STATUS_INPUT}")"
	P0_ROUTE_ID="$(env_value route_id "${P0_STATUS_INPUT}")"
	P0_ROUTE_NAME="$(env_value route_name "${P0_STATUS_INPUT}")"
	P0_ROUTE_PROFILE="$(env_value route_profile "${P0_STATUS_INPUT}")"
	P0_ROUTE_FINAL_LABEL="$(env_value route_final_label "${P0_STATUS_INPUT}")"
	P0_ROUTE_WAYPOINTS_CSV="$(env_value route_waypoints_csv "${P0_STATUS_INPUT}")"
	[ -n "${P0_PERCEPTION_SOURCE}" ] || P0_PERCEPTION_SOURCE="$(env_value sensor "${P0_STATUS_INPUT}")"
	[ -n "${P0_PERCEPTION_SOURCE}" ] || P0_PERCEPTION_SOURCE="$(env_value source_p0_sensor "${P0_STATUS_INPUT}")"
	[ -n "${P0_RAW_CLOUD_TOPIC}" ] || P0_RAW_CLOUD_TOPIC="$(env_value source_p0_raw_cloud_topic "${P0_STATUS_INPUT}")"
fi
[ -n "${P0_PERCEPTION_SOURCE}" ] || P0_PERCEPTION_SOURCE="${F250_FC_P0_PERCEPTION_SOURCE:-}"
[ -n "${P0_RAW_CLOUD_TOPIC}" ] || P0_RAW_CLOUD_TOPIC="${F250_FC_P0_RAW_CLOUD_TOPIC:-}"

ROUTE_WAYPOINTS_CSV="${ROUTE_WAYPOINTS_CSV:-${P0_ROUTE_WAYPOINTS_CSV}}"
if [ -z "${ROUTE_WAYPOINTS_CSV}" ] && [ -n "${P0_RUN_DIR}" ] && [ -f "${P0_RUN_DIR}/route_waypoints.csv" ]; then
	ROUTE_WAYPOINTS_CSV="${P0_RUN_DIR}/route_waypoints.csv"
fi

if [ -n "${P0_VEHICLE}" ] && [ "${P0_VEHICLE}" != "f250" ]; then
	fail "current status is not F250: ${CURRENT_STATUS} vehicle=${P0_VEHICLE}"
fi

# H1: Guard against running from anything except a fresh P0 hover-ready launch.
# The dispatcher validates CURRENT_STATUS before it overwrites that file with
# task=flight_control/background_worker_starting. The --foreground worker uses
# F250_FC_PRECHECKED=true plus the copied P0 status snapshot from the dispatcher.
if [ "${F250_FC_PRECHECKED:-false}" != "true" ]; then
	[ -f "${CURRENT_STATUS}" ] || fail "missing current P0 hover status: ${CURRENT_STATUS}; run f250_start_to_p0_hover.sh first"
	_src_task="$(env_value task "${CURRENT_STATUS}")"
	_src_state="$(env_value state "${CURRENT_STATUS}")"
	if [ "${_src_task}" != "launch" ] || [ "${_src_state}" != "hover_ready" ]; then
		fail "请先运行 f250_start_to_p0_hover.sh 重建 P0 hover 环境，并等待 hover_ready 后再运行 FC (task=${_src_task:-<empty>} state=${_src_state:-<empty>})"
	fi
fi

# H2: Do not fall back to a hardcoded magic number; require an explicit hover target.
if [ -n "${HOVER_TARGET:-}" ]; then
	HOVER_TARGET_RESOLVED="${HOVER_TARGET}"
elif [ -n "${P0_HOVER_TARGET:-}" ]; then
	HOVER_TARGET_RESOLVED="${P0_HOVER_TARGET}"
else
	fail "无法解析有效的 P0 hover target：HOVER_TARGET 和 P0_HOVER_TARGET 均为空，请先运行 f250_start_to_p0_hover.sh 重建 P0 hover 环境"
fi

mkdir -p "${RUN_DIR}" "${RUN_DIR}/logs"
STATUS_FILE="${RUN_DIR}/status.env"
SUMMARY_JSON="${RUN_DIR}/fc_3_10_summary.json"
SAMPLES_CSV="${RUN_DIR}/fc_3_10_samples.csv"
PHASE_CSV="${RUN_DIR}/fc_3_10_phases.csv"
GEOMETRY_JSON="${RUN_DIR}/fc_3_10_geometry_audit.json"
DECAGON_CSV="${RUN_DIR}/fc_3_10_decagon_points.csv"
DISPLAY_LOG="${RUN_DIR}/fc_3_10_terminal.log"
PROVENANCE_FILE="${RUN_DIR}/provenance.txt"

if [ "${F250_FC_TERMINAL_LOG_READY:-false}" != "true" ]; then
	: >"${DISPLAY_LOG}"
fi

P0_STATUS_FOR_HELPER="${F250_FC_P0_STATUS:-}"
if [ -z "${P0_STATUS_FOR_HELPER}" ] && [ -f "${P0_STATUS_INPUT:-${CURRENT_STATUS}}" ]; then
	P0_STATUS_FOR_HELPER="${RUN_DIR}/source_p0_status.env"
	cp "${P0_STATUS_INPUT:-${CURRENT_STATUS}}" "${P0_STATUS_FOR_HELPER}"
fi
[ -n "${P0_STATUS_FOR_HELPER}" ] || P0_STATUS_FOR_HELPER="${CURRENT_STATUS}"

{
	echo "created_at=$(date -Is)"
	echo "project_root=${PROJECT_ROOT}"
	echo "workspace=${WS}"
	echo "package=${PKG}"
	echo "script=${0}"
	echo "helper=${HELPER}"
	echo "run_dir=${RUN_DIR}"
	echo "run_label=${RUN_LABEL}"
	echo "source_p0_status=${P0_STATUS_FOR_HELPER:-${CURRENT_STATUS}}"
	echo "source_p0_run_dir=${P0_RUN_DIR}"
	echo "source_p0_state=${P0_STATE}"
	echo "source_p0_sensor=${P0_PERCEPTION_SOURCE}"
	echo "source_p0_raw_cloud_topic=${P0_RAW_CLOUD_TOPIC}"
	echo "route_id=${P0_ROUTE_ID}"
	echo "route_name=${P0_ROUTE_NAME}"
	echo "route_profile=${P0_ROUTE_PROFILE}"
	echo "route_final_label=${P0_ROUTE_FINAL_LABEL}"
	echo "route_waypoints_csv=${ROUTE_WAYPOINTS_CSV}"
	echo "hover_target=${HOVER_TARGET_RESOLVED}"
	echo "vehicle=f250"
	echo "metric_policy=3.10 independent FC-only steady-state test; no route acceptance writes"
	echo "geometry_policy=current route first waypoint when available; authoritative P0 fallback; no PNG coordinate truth"
	echo "velocity_policy=settled standard, speed 2 mps, 10 windows, 5 AB round trips"
	echo "speed_only=${SPEED_ONLY}"
	echo "velocity_prealign_enabled=$([ "${DISABLE_VELOCITY_PREALIGN}" = "true" ] && echo false || echo true)"
	echo "position_policy=3D steady-position error divided by planned 3D previous-target to current-target step distance"
	echo "yaw_policy=+90,+180,+270,+360,-90,-180,-270,-360,+180,0"
	echo "host=$(hostname)"
	echo "user=$(id -un)"
} >"${PROVENANCE_FILE}"

write_status "prepared"

HELPER_ARGS=(
	--run-dir "${RUN_DIR}"
	--run-label "${RUN_LABEL}"
	--summary-json "${SUMMARY_JSON}"
	--samples-csv "${SAMPLES_CSV}"
	--phase-csv "${PHASE_CSV}"
	--geometry-audit-json "${GEOMETRY_JSON}"
	--decagon-csv "${DECAGON_CSV}"
	--display-log "${DISPLAY_LOG}"
	--p0-status "${P0_STATUS_FOR_HELPER}"
	--p0-run-dir "${P0_RUN_DIR}"
	--route-waypoints-csv "${ROUTE_WAYPOINTS_CSV}"
	--hover-target "${HOVER_TARGET_RESOLVED}"
)
if [ -n "${MAP_AUTHORITY:-}" ]; then
	HELPER_ARGS+=(--map-authority "${MAP_AUTHORITY}")
fi
if [ "${SPEED_ONLY}" = "true" ]; then
	HELPER_ARGS+=(--speed-only)
fi
if [ "${DISABLE_VELOCITY_PREALIGN}" = "true" ]; then
	HELPER_ARGS+=(--velocity-prealign-sec 0)
fi

if [ "${GEOMETRY_CHECK}" = "true" ]; then
	HELPER_ARGS+=(--geometry-check)
elif [ "${DRY_RUN}" = "true" ]; then
	HELPER_ARGS+=(--dry-run)
else
	source /opt/ros/noetic/setup.bash
	if [ -f "${WS}/devel/setup.bash" ]; then
		source "${WS}/devel/setup.bash"
	else
		export ROS_PACKAGE_PATH="${WS}/src:${ROS_PACKAGE_PATH:-}"
	fi
	ensure_ros_available
fi

if [ -n "${F250_FC_3_10_ARGS:-}" ]; then
	# shellcheck disable=SC2206
	EXTRA_ARGS=(${F250_FC_3_10_ARGS})
	HELPER_ARGS+=("${EXTRA_ARGS[@]}")
fi

if [ "${DRY_RUN}" != "true" ] && [ "${GEOMETRY_CHECK}" != "true" ] && [ "${RUN_IN_BACKGROUND}" = "true" ]; then
	write_status "background_worker_starting"
	if launch_fc_worker; then
		exit 0
	fi
	# launch_fc_worker already wrote state=failed and logged the cause.
	exit 2
fi

if [ "${DRY_RUN}" != "true" ] && [ "${GEOMETRY_CHECK}" != "true" ]; then
	trap append_terminal_over EXIT
	open_metrics_terminal "F250 FC 3.10 Metrics" "${DISPLAY_LOG}" "f250_fc310_metrics_${RUN_LABEL}" >/dev/null || true
	layout_metrics_window
fi

set +e
python3 "${HELPER}" "${HELPER_ARGS[@]}"
STATUS=$?
set -e

if [ "${STATUS}" -eq 0 ]; then
	write_status "complete"
	publish_current_review
else
	write_status "failed"
	echo "F250 FC 3.10 failed with status ${STATUS}; see ${RUN_DIR}" >&2
fi
exit "${STATUS}"
