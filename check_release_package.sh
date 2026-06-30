#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
RUNTIME="${ROOT}/runtime"
PATCH="${ROOT}/px4_f250_patch"
EXPECTED_F250_SDF_SHA256="f57e4f06c849d8c7cd350399b8500ea3f47ca7d929e014acd5caaa99d7fcca98"

fail() { echo "check_release_package: $*" >&2; exit 2; }

for required in README.md .gitignore .gitattributes apply_px4_f250_patch.sh runtime px4_f250_patch; do
  [ -e "${ROOT}/${required}" ] || fail "missing root entry: ${required}"
done
[ -x "${ROOT}/apply_px4_f250_patch.sh" ] || fail "apply_px4_f250_patch.sh is not executable"
[ ! -e "${ROOT}/docs" ] || fail "root docs/ should not exist in final layout"
[ ! -e "${ROOT}/install" ] || fail "root install/ should not exist in final layout"

unexpected_root="$(find "${ROOT}" -mindepth 1 -maxdepth 1   ! -name README.md   ! -name .gitignore   ! -name .gitattributes   ! -name apply_px4_f250_patch.sh   ! -name check_release_package.sh   ! -name runtime   ! -name px4_f250_patch   ! -name .git   -print)"
[ -z "${unexpected_root}" ] || { printf '%s
' "${unexpected_root}" >&2; fail "unexpected root entry"; }

[ -f "${PATCH}/ROMFS/px4fmu_common/init.d-posix/airframes/10020_gazebo-classic_f250" ] || fail "missing F250 airframe patch"
[ -f "${PATCH}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/f250/f250.sdf" ] || fail "missing F250 model patch"
actual_sdf_sha="$(sha256sum "${PATCH}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/f250/f250.sdf" | awk '{print $1}')"
[ "${actual_sdf_sha}" = "${EXPECTED_F250_SDF_SHA256}" ] || fail "F250 patch SDF hash mismatch: ${actual_sdf_sha}"

for excluded in   "${RUNTIME}/.gitignore"   "${RUNTIME}/.gitattributes"   "${RUNTIME}/runtime_state"   "${RUNTIME}/catkin_ws/build"   "${RUNTIME}/catkin_ws/devel"   "${RUNTIME}/catkin_ws/install"   "${RUNTIME}/cache"   "${RUNTIME}/runs"   "${ROOT}/delivery_package"   "${ROOT}/.check_pycache"; do
  [ ! -e "${excluded}" ] || fail "generated/local path should not be in release: ${excluded}"
done

if find "${ROOT}" -path "${ROOT}/.git" -prune -o -type d \( -name __pycache__ -o -name .pytest_cache -o -name .vscode -o -name '.check_pycache.*' \) -print | grep -q .; then
  find "${ROOT}" -path "${ROOT}/.git" -prune -o -type d \( -name __pycache__ -o -name .pytest_cache -o -name .vscode -o -name '.check_pycache.*' \) -print >&2
  fail "excluded cache/editor directory found"
fi
if find "${ROOT}" -path "${ROOT}/.git" -prune -o -type f \( -name '*.pyc' -o -name '*.log' -o -name '*.zip' -o -name '*.bag' -o -name '*.xwd' \) -print | grep -q .; then
  find "${ROOT}" -path "${ROOT}/.git" -prune -o -type f \( -name '*.pyc' -o -name '*.log' -o -name '*.zip' -o -name '*.bag' -o -name '*.xwd' \) -print >&2
  fail "excluded cache/log/archive file found"
fi

[ -f "${RUNTIME}/README.md" ] || fail "missing runtime README"
[ -f "${RUNTIME}/env.example" ] || fail "missing runtime env.example"
[ -d "${RUNTIME}/catkin_ws/src/f250_maritime_uav_sim" ] || fail "missing F250 ROS package"
[ -d "${RUNTIME}/catkin_ws/src/ego-planner" ] || fail "missing EGO-Planner source"
[ -f "${RUNTIME}/catkin_ws/src/CMakeLists.txt" ] || fail "missing catkin toplevel CMakeLists"
[ -f "${RUNTIME}/catkin_ws/src/ego-planner/src/uav_simulator/local_sensing/CATKIN_IGNORE" ] || fail "EGO local_sensing should be ignored for public build"
[ -x "${RUNTIME}/maintenance/check_package.sh" ] || fail "missing runtime package checker"

bash -n "${ROOT}/apply_px4_f250_patch.sh" "${ROOT}/check_release_package.sh"
"${RUNTIME}/maintenance/check_package.sh"

echo "F250 GitHub release package check passed: ${ROOT}"
