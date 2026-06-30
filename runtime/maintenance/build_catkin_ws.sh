#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
WS="${ROOT}/catkin_ws"
ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
DRY_RUN="false"
DEPENDENCY_SETUPS=()

usage() {
	cat <<EOF
Usage:
  ${0} [--dry-run] [--dependency-setup /path/to/devel/setup.bash]
  ${0} --help

Builds this repository's catkin workspace only:
  ${WS}

Build order:
  1. source ROS setup: ${ROS_SETUP}
  2. source each dependency setup passed by --dependency-setup or F250_DEPENDENCY_SETUP
  3. run catkin_make inside ${WS}

Dependency setup:
  --dependency-setup may be repeated.
  F250_DEPENDENCY_SETUP may contain one setup file or a colon-separated list.

Required dependencies:
  - MAVROS message/runtime packages
  - EGO-Planner dependencies that provide quadrotor_msgs and planner nodes

If no dependency setup is supplied, EGO-Planner and quadrotor_msgs must already
be discoverable after sourcing ROS. If they live in another catkin workspace,
pass that workspace's devel/setup.bash with --dependency-setup.
EOF
}

if [ -n "${F250_DEPENDENCY_SETUP:-}" ]; then
	IFS=':' read -r -a env_dependency_setups <<<"${F250_DEPENDENCY_SETUP}"
	for setup_file in "${env_dependency_setups[@]}"; do
		[ -n "${setup_file}" ] && DEPENDENCY_SETUPS+=("${setup_file}")
	done
fi

while [ "$#" -gt 0 ]; do
	case "$1" in
	--dry-run)
		DRY_RUN="true"
		shift
		;;
	--dependency-setup)
		[ "$#" -ge 2 ] || {
			echo "build_catkin_ws: --dependency-setup requires a path" >&2
			exit 2
		}
		DEPENDENCY_SETUPS+=("$2")
		shift 2
		;;
	--dependency-setup=*)
		DEPENDENCY_SETUPS+=("${1#*=}")
		shift
		;;
	--help | -h)
		usage
		exit 0
		;;
	*)
		echo "build_catkin_ws: unknown argument: $1" >&2
		echo "Run ${0} --help" >&2
		exit 2
		;;
	esac
done

[ -d "${WS}/src/f250_maritime_uav_sim" ] || {
	echo "build_catkin_ws: missing package under ${WS}/src" >&2
	exit 2
}
[ -f "${ROS_SETUP}" ] || {
	echo "build_catkin_ws: missing ROS setup file: ${ROS_SETUP}" >&2
	exit 2
}
for setup_file in "${DEPENDENCY_SETUPS[@]}"; do
	[ -f "${setup_file}" ] || {
		echo "build_catkin_ws: missing dependency setup file: ${setup_file}" >&2
		exit 2
	}
done

print_dependency_note() {
	if [ "${#DEPENDENCY_SETUPS[@]}" -eq 0 ]; then
		echo "No dependency setup supplied."
		echo "EGO-Planner and quadrotor_msgs must already be discoverable after sourcing ROS."
		echo "If they live in another catkin workspace, pass --dependency-setup /path/to/devel/setup.bash."
	else
		echo "Dependency setup files:"
		for setup_file in "${DEPENDENCY_SETUPS[@]}"; do
			echo "  ${setup_file}"
		done
	fi
}

if [ "${DRY_RUN}" = "true" ]; then
	print_dependency_note
	echo "Would run:"
	echo "  cd ${WS}"
	echo "  source ${ROS_SETUP}"
	for setup_file in "${DEPENDENCY_SETUPS[@]}"; do
		echo "  source ${setup_file}"
	done
	echo "  catkin_make"
	exit 0
fi

cd "${WS}"
print_dependency_note
echo "build_catkin_ws: sourcing ROS setup: ${ROS_SETUP}"
# shellcheck source=/dev/null
source "${ROS_SETUP}"
for setup_file in "${DEPENDENCY_SETUPS[@]}"; do
	echo "build_catkin_ws: sourcing dependency setup: ${setup_file}"
	# shellcheck source=/dev/null
	source "${setup_file}"
done
echo "build_catkin_ws: running catkin_make in ${WS}"
catkin_make
