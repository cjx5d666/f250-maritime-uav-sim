#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/f250_paths.sh"
PROJECT_ROOT="$(f250_resolve_project_root "${SCRIPT_DIR}")"
WS="${PROJECT_ROOT}/catkin_ws"

if [ "${1:-}" = "--help" ] || [ "${1:-}" = "-h" ]; then
	cat <<'EOF'
Usage:
  catkin_ws/src/f250_maritime_uav_sim/scripts/start_demo_waypoints.sh [topic]

Publishes a one-shot std_msgs/Bool true to the demo waypoint start topic.
Default topic: /maritime/demo/start_waypoints

Optional:
  MARITIME_START_MSG_TYPE=empty  publish std_msgs/Empty instead of Bool
EOF
	exit 0
fi

TOPIC="${1:-${MARITIME_START_TOPIC:-/maritime/demo/start_waypoints}}"
MSG_TYPE="${MARITIME_START_MSG_TYPE:-bool}"

source /opt/ros/noetic/setup.bash
if [ -f "${WS}/devel/setup.bash" ]; then
	source "${WS}/devel/setup.bash"
fi

case "${MSG_TYPE}" in
bool | Bool | std_msgs/Bool)
	echo "[demo-start] publishing std_msgs/Bool true on ${TOPIC}"
	exec rostopic pub -1 "${TOPIC}" std_msgs/Bool "data: true"
	;;
empty | Empty | std_msgs/Empty)
	echo "[demo-start] publishing std_msgs/Empty on ${TOPIC}"
	exec rostopic pub -1 "${TOPIC}" std_msgs/Empty "{}"
	;;
*)
	echo "Unsupported MARITIME_START_MSG_TYPE=${MSG_TYPE}; use bool or empty." >&2
	exit 2
	;;
esac
