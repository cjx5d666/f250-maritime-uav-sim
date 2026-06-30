#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<EOF
Usage:
  ./apply_px4_f250_patch.sh /path/to/PX4-Autopilot

Copies this repository's F250 airframe and Gazebo Classic model into a PX4 v1.16.0 source tree.
EOF
}

if [ "${1:-}" = "--help" ] || [ "${1:-}" = "-h" ]; then
  usage
  exit 0
fi

PX4_ROOT="${1:-${F250_PX4_ROOT:-}}"
[ -n "${PX4_ROOT}" ] || { usage >&2; exit 2; }
PX4_ROOT="$(cd "${PX4_ROOT}" && pwd -P)" || { echo "PX4 root not found: ${PX4_ROOT}" >&2; exit 2; }

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
PATCH_ROOT="${REPO_ROOT}/px4_f250_patch"
AIRFRAME_SRC="${PATCH_ROOT}/ROMFS/px4fmu_common/init.d-posix/airframes/10020_gazebo-classic_f250"
MODEL_SRC="${PATCH_ROOT}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/f250"
AIRFRAME_DST="${PX4_ROOT}/ROMFS/px4fmu_common/init.d-posix/airframes/10020_gazebo-classic_f250"
MODEL_DST_PARENT="${PX4_ROOT}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models"
MODEL_DST="${MODEL_DST_PARENT}/f250"
EXPECTED_SDF_SHA256="f57e4f06c849d8c7cd350399b8500ea3f47ca7d929e014acd5caaa99d7fcca98"

[ -f "${PX4_ROOT}/launch/mavros_posix_sitl.launch" ] || {
  echo "This does not look like the expected PX4 v1.16.0 tree: ${PX4_ROOT}" >&2
  exit 2
}
[ -f "${AIRFRAME_SRC}" ] || { echo "missing patch airframe: ${AIRFRAME_SRC}" >&2; exit 2; }
[ -f "${MODEL_SRC}/f250.sdf" ] || { echo "missing patch model: ${MODEL_SRC}" >&2; exit 2; }

actual="$(sha256sum "${MODEL_SRC}/f250.sdf" | awk '{print $1}')"
[ "${actual}" = "${EXPECTED_SDF_SHA256}" ] || {
  echo "patch f250.sdf hash mismatch: ${actual}" >&2
  exit 2
}

install -D -m 755 "${AIRFRAME_SRC}" "${AIRFRAME_DST}"
mkdir -p "${MODEL_DST_PARENT}"
rm -rf "${MODEL_DST}"
cp -a "${MODEL_SRC}" "${MODEL_DST_PARENT}/"

BUILD_AIRFRAME_DIR="${PX4_ROOT}/build/px4_sitl_default/etc/init.d-posix/airframes"
if [ -d "${BUILD_AIRFRAME_DIR}" ]; then
  install -D -m 755 "${AIRFRAME_SRC}" "${BUILD_AIRFRAME_DIR}/10020_gazebo-classic_f250"
fi

printf 'Installed F250 PX4 patch into: %s
' "${PX4_ROOT}"
printf 'Next, rebuild PX4 SITL if needed:
  cd %s && make px4_sitl gazebo-classic
' "${PX4_ROOT}"
printf 'For this simulation, set if PX4 is outside common home-directory paths:
  export F250_PX4_ROOT=%s
' "${PX4_ROOT}"
