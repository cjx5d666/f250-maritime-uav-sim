#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/f250_paths.sh"
PROJECT_ROOT="$(f250_resolve_project_root "${SCRIPT_DIR}")"
WS="${PROJECT_ROOT}/catkin_ws"
PKG="$(f250_resolve_package_root "${SCRIPT_DIR}" "${PROJECT_ROOT}")"

MODE="${1:-quick-complex}"
if [ "${MODE}" = "--help" ] || [ "${MODE}" = "-h" ]; then
	cat <<'EOF'
Usage:
  ./start_maritime_sim.sh [quick-complex|quick-complex-depth|quick-complex-lidar]

Default mode:
  quick-complex  F250 integrated quick-complex route with Gazebo LiDAR perception.

Operator-facing mode:
  quick-complex  integrated quick-complex route; default perception is LiDAR
  quick-complex-lidar  same route with Gazebo lidar planner cloud
  quick-complex-depth  same route with Gazebo depth planner cloud

Developer/debug hooks:

Stop the visible simulation:
  ./f250_stop_all.sh

Common environment overrides:
  F250_PROJECT_ROOT=...
  F250_PX4_ROOT=...
  PX4_BOOT_WAIT_SEC=12
  AUTO_OFFBOARD_ARM=true
  REQUIRE_PLANNER_COMMAND_FOR_OFFBOARD=false
  PX4_NO_FOLLOW_MODE=1
EOF
	exit 0
fi

if [ ! -d "${WS}" ] || [ ! -d "${PKG}" ]; then
	echo "Missing workspace: ${WS}" >&2
	exit 2
fi
PX4_ROOT="$(f250_resolve_px4_root "${PROJECT_ROOT}")"
SITL_GAZEBO="${PX4_ROOT}/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
PX4_BUILD="${PX4_ROOT}/build/px4_sitl_default"

cd "${WS}"
export ROS_DISTRO="${ROS_DISTRO:-noetic}"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
if [ -f devel/setup.bash ]; then
	# shellcheck disable=SC1091
	source devel/setup.bash
else
	echo "Missing catkin devel setup. Run catkin_make in ${WS} first." >&2
	exit 2
fi

export F250_PX4_ROOT="${PX4_ROOT}"
export ROS_PACKAGE_PATH="${PX4_ROOT}:${SITL_GAZEBO}:${ROS_PACKAGE_PATH:-}"
export GAZEBO_MODEL_PATH="${PKG}/models:${SITL_GAZEBO}/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_PLUGIN_PATH="/opt/ros/noetic/lib:${PX4_BUILD}/build_gazebo-classic:${GAZEBO_PLUGIN_PATH:-}"
export LD_LIBRARY_PATH="${PX4_BUILD}/build_gazebo-classic:${LD_LIBRARY_PATH:-}"
DISPLAY_VALUE="${DISPLAY:-}"
if [ -z "${DISPLAY_VALUE//[[:space:]]/}" ]; then
	export DISPLAY=":0"
else
	export DISPLAY
fi
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"
export DISABLE_ROS1_EOL_WARNINGS="${DISABLE_ROS1_EOL_WARNINGS:-1}"
export AUTO_OFFBOARD_ARM="${AUTO_OFFBOARD_ARM:-true}"
export MARITIME_VEHICLE="f250"
export ENABLE_RVIZ="${ENABLE_RVIZ:-true}"
export PX4_NO_FOLLOW_MODE="${PX4_NO_FOLLOW_MODE:-1}"

normalize_perception_source() {
	case "$1" in
	lidar | depth)
		printf "%s\n" "$1"
		;;
	*)
		echo "Unsupported PERCEPTION_SOURCE: $1" >&2
		echo "Expected one of: lidar, depth" >&2
		return 2
		;;
	esac
}

rviz_config_for_sensor() {
	case "$1" in
	depth) printf "%s
" "${PKG}/rviz/maritime_visual_acceptance_depth.rviz" ;;
	*) printf "%s
" "${PKG}/rviz/maritime_visual_acceptance.rviz" ;;
	esac
}

launch_visible() {
	local scene_level="$1"
	local perception_source
	perception_source="$(normalize_perception_source "$2")"
	local dynamic_mode="$3"
	local landing_mode="$4"
	local scene_config="${SCENE_CONFIG:-${PKG}/config/scenes/${scene_level}.yaml}"
	local world="${PKG}/worlds/maritime_${scene_level}.world"
	local rviz_config="${RVIZ_CONFIG:-$(rviz_config_for_sensor "${perception_source}")}"
	echo "[maritime] visible mode: ${MODE}"
	echo "[maritime] scene=${scene_level} perception=${perception_source} dynamic=${dynamic_mode} landing=${landing_mode}"
	echo "[maritime] rviz_config=${rviz_config}"
	echo "[maritime] close with: ${PKG}/scripts/f250_stop_all.sh"
	exec roslaunch f250_maritime_uav_sim maritime_visual_acceptance.launch \
		vehicle:="${MARITIME_VEHICLE}" \
		scene_level:="${scene_level}" \
		scene_config:="${scene_config}" \
		perception_source:="${perception_source}" \
		dynamic_mode:="${dynamic_mode}" \
		world:="${world}" \
		rviz_config:="${rviz_config}" \
		px4_gui:="${PX4_GUI:-true}" \
		auto_offboard_arm:="${AUTO_OFFBOARD_ARM}" \
		require_planner_command_for_offboard:="${REQUIRE_PLANNER_COMMAND_FOR_OFFBOARD:-true}" \
		landing_enabled:="${landing_mode}" \
		enable_rviz:="${ENABLE_RVIZ}"
}

use_quick_map() {
	export MAP_SIZE_X="${MAP_SIZE_X:-760.0}"
	export MAP_SIZE_Y="${MAP_SIZE_Y:-320.0}"
	export MAP_SIZE_Z="${MAP_SIZE_Z:-18.0}"
}

use_quick_complex_ego_map() {
	use_quick_map
	f250_apply_quick_complex_defaults "${PKG}"
}

use_quick_spawn() {
	export PX4_SPAWN_X="${PX4_SPAWN_X:-$1}"
	export PX4_SPAWN_Y="${PX4_SPAWN_Y:-$2}"
	export PX4_SPAWN_Z="${PX4_SPAWN_Z:-${3:-1.10}}"
	export PX4_SPAWN_YAW="${PX4_SPAWN_YAW:-${4:-0.0}}"
}

use_f250_quick_metrics() {
	local metric_root="${RUN_ROOT:-${PROJECT_ROOT}/runtime_state/work}"
	export MARITIME_ENABLE_METRIC_MONITOR="${MARITIME_ENABLE_METRIC_MONITOR:-true}"
	export MARITIME_METRIC_OUTPUT_DIR="${MARITIME_METRIC_OUTPUT_DIR:-${metric_root}/live_metric_runs}"
	export MARITIME_METRIC_RUN_LABEL="${MARITIME_METRIC_RUN_LABEL:-f250_quick_complex_$(date +%Y%m%d_%H%M%S)}"
}

case "${MODE}" in
quick-complex | complex-quick | quick_complex | default)
	use_quick_complex_ego_map
	use_quick_spawn 55.0 16.0 4.82 0.469929
	use_f250_quick_metrics
	export MARITIME_START_PAUSED="${MARITIME_START_PAUSED:-false}"
	launch_visible "${SCENE_LEVEL:-level_m_gps_assets_quick_complex}" "${PERCEPTION_SOURCE:-lidar}" "${DYNAMIC_MODE:-auto}" "${LANDING_MODE:-false}"
	;;
quick-complex-depth | complex-depth | quick_complex_depth)
	use_quick_complex_ego_map
	use_quick_spawn 55.0 16.0 4.82 0.469929
	use_f250_quick_metrics
	export MARITIME_START_PAUSED="${MARITIME_START_PAUSED:-false}"
	launch_visible "${SCENE_LEVEL:-level_m_gps_assets_quick_complex}" "${PERCEPTION_SOURCE:-depth}" "${DYNAMIC_MODE:-auto}" "${LANDING_MODE:-false}"
	;;
quick-complex-lidar | complex-lidar | quick_complex_lidar)
	use_quick_complex_ego_map
	use_quick_spawn 55.0 16.0 4.82 0.469929
	use_f250_quick_metrics
	export MARITIME_START_PAUSED="${MARITIME_START_PAUSED:-false}"
	launch_visible "${SCENE_LEVEL:-level_m_gps_assets_quick_complex}" "${PERCEPTION_SOURCE:-lidar}" "${DYNAMIC_MODE:-auto}" "${LANDING_MODE:-false}"
	;;
*)
	echo "Unknown mode: ${MODE}" >&2
	echo "Run: ${0} --help" >&2
	exit 2
	;;
esac
