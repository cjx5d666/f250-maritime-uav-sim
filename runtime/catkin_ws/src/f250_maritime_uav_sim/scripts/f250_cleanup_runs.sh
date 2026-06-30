#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/f250_paths.sh"
PROJECT_ROOT="$(f250_resolve_project_root "${SCRIPT_DIR}")"
RUNTIME_STATE_DIR="${F250_RUNTIME_STATE_DIR:-${PROJECT_ROOT}/runtime_state}"
RUN_ROOT="${RUN_ROOT:-${RUNTIME_STATE_DIR}/work}"
ACTIVE_STATUS="${F250_ACTIVE_TASK_ENV:-${RUNTIME_STATE_DIR}/active_task.env}"
KEEP_ACTIVE="${F250_KEEP_ACTIVE_TASK_DIR:-true}"
DRY_RUN="${F250_CLEANUP_DRY_RUN:-false}"
QUIET="${F250_CLEANUP_QUIET:-true}"

log() {
	[ "${QUIET}" = "true" ] || echo "$*"
}

safe_rm() {
	local path="$1"
	case "${path}" in
	"${RUN_ROOT}"/f250_p0_hover_* | "${RUN_ROOT}"/f250_p0_p8_route_* | "${RUN_ROOT}"/f250_fc_3_10_steady_state_* | "${RUN_ROOT}"/stop_*.log)
		;;
	*)
		echo "refusing to remove outside known F250 run pattern: ${path}" >&2
		return 2
		;;
	esac
	if [ "${DRY_RUN}" = "true" ]; then
		log "would_remove=${path}"
	else
		rm -rf -- "${path}"
		log "removed=${path}"
	fi
}

[ -d "${RUN_ROOT}" ] || exit 0
RUN_ROOT="$(cd "${RUN_ROOT}" && pwd -P)"

ACTIVE_TARGET=""
if [ -f "${ACTIVE_STATUS}" ]; then
	ACTIVE_TARGET="$(awk -F= '$1 == "run_dir" {sub(/^[^=]*=/, ""); print; exit}' "${ACTIVE_STATUS}" 2>/dev/null || true)"
	[ -n "${ACTIVE_TARGET}" ] && ACTIVE_TARGET="$(readlink -f "${ACTIVE_TARGET}" 2>/dev/null || true)"
fi

shopt -s nullglob
for path in "${RUN_ROOT}"/stop_*.log "${RUN_ROOT}"/f250_p0_p8_route_* "${RUN_ROOT}"/f250_fc_3_10_steady_state_* "${RUN_ROOT}"/f250_p0_hover_*; do
	[ -e "${path}" ] || continue
	resolved="$(readlink -f "${path}" 2>/dev/null || true)"
	if [ "${KEEP_ACTIVE}" = "true" ] && [ -n "${ACTIVE_TARGET}" ] && [ "${resolved}" = "${ACTIVE_TARGET}" ]; then
		log "kept_active=${path}"
		continue
	fi
	safe_rm "${path}"
done
