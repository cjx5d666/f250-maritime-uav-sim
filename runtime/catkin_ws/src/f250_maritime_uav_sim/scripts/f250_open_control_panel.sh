#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/f250_paths.sh"
PROJECT_ROOT="$(f250_resolve_project_root "${SCRIPT_DIR}")"
PKG="$(f250_resolve_package_root "${SCRIPT_DIR}" "${PROJECT_ROOT}")"
HELPER="${PKG}/scripts/f250_control_panel.py"
PYTHON_BIN="${PYTHON:-python3}"

[ -f "${HELPER}" ] || {
	echo "f250_open_control_panel: missing helper: ${HELPER}" >&2
	exit 2
}

exec "${PYTHON_BIN}" "${HELPER}" "$@"
