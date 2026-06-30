#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/f250_paths.sh"
PROJECT_ROOT="$(f250_resolve_project_root "${SCRIPT_DIR}")"
MAP_AUTHORITY="${MAP_AUTHORITY:-${PROJECT_ROOT}/map_authority/p0p8_clean_scene}"
RENDERER="${MAP_AUTHORITY}/render_map.py"
TARGET_PNG="${MAP_AUTHORITY}/route_map.png"

[ -x "${RENDERER}" ] || {
	echo "missing map renderer: ${RENDERER}" >&2
	exit 2
}
PYTHONDONTWRITEBYTECODE=1 python3 "${RENDERER}" --target route >/dev/null
if [ "${F250_OPEN_PLOT:-true}" = "true" ] && [ -n "${DISPLAY:-}" ] && command -v xdg-open >/dev/null 2>&1; then
	nohup xdg-open "${TARGET_PNG}" >/dev/null 2>&1 &
fi
echo "Route map: ${TARGET_PNG}"
