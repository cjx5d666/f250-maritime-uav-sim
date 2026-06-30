#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
PKG_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd -P)"

APP_ID="f250-maritime-control-panel"
APP_NAME="F250 Maritime Control Panel"
OPEN_SCRIPT="${PKG_ROOT}/scripts/f250_open_control_panel.sh"
ICON_SOURCE="${PKG_ROOT}/resources/f250_control_panel_icon.svg"
ICON_DIR="${HOME}/.local/share/icons/hicolor/scalable/apps"
APP_DIR="${HOME}/.local/share/applications"
DESKTOP_DIR="${HOME}/Desktop"
ICON_TARGET="${ICON_DIR}/${APP_ID}.svg"
APP_FILE="${APP_DIR}/${APP_ID}.desktop"
DESKTOP_FILE="${DESKTOP_DIR}/${APP_NAME}.desktop"

if [[ ! -x "${OPEN_SCRIPT}" ]]; then
	echo "missing or non-executable UI launcher: ${OPEN_SCRIPT}" >&2
	exit 2
fi

if [[ ! -f "${ICON_SOURCE}" ]]; then
	echo "missing icon: ${ICON_SOURCE}" >&2
	exit 2
fi

install -d -m 0755 "${ICON_DIR}" "${APP_DIR}" "${DESKTOP_DIR}"
install -m 0644 "${ICON_SOURCE}" "${ICON_TARGET}"

tmp_app="$(mktemp "${APP_DIR}/${APP_ID}.XXXXXX.desktop")"
cat >"${tmp_app}" <<EOF
[Desktop Entry]
Type=Application
Name=${APP_NAME}
Comment=Open the F250 maritime UAV control panel
Exec=${OPEN_SCRIPT}
Path=${PKG_ROOT}
Icon=${ICON_TARGET}
Terminal=false
StartupNotify=true
Categories=Utility;
EOF

if command -v desktop-file-validate >/dev/null 2>&1; then
	desktop-file-validate "${tmp_app}"
fi

mv "${tmp_app}" "${APP_FILE}"
chmod 0644 "${APP_FILE}"
cp "${APP_FILE}" "${DESKTOP_FILE}"
chmod 0755 "${DESKTOP_FILE}"

if command -v gio >/dev/null 2>&1; then
	gio set "${DESKTOP_FILE}" metadata::trusted true >/dev/null 2>&1 || true
fi

if command -v update-desktop-database >/dev/null 2>&1; then
	update-desktop-database "${APP_DIR}" >/dev/null 2>&1 || true
fi

echo "installed: ${DESKTOP_FILE}"
