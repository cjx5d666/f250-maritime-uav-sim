#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
REPO_ROOT="$(cd "${ROOT}/.." && pwd -P)"
PKG="${ROOT}/catkin_ws/src/f250_maritime_uav_sim"
EGO="${ROOT}/catkin_ws/src/ego-planner"
MAP="${ROOT}/map_authority/p0p8_clean_scene"
EVIDENCE="${ROOT}/evidence/current"

fail() { echo "runtime/check_package: $*" >&2; exit 2; }

[ -d "${PKG}" ] || fail "missing ROS package: ${PKG}"
[ -d "${EGO}/src" ] || fail "missing EGO-Planner source"
[ -f "${ROOT}/catkin_ws/src/CMakeLists.txt" ] || fail "missing catkin_ws/src/CMakeLists.txt"
[ -d "${MAP}/sources" ] || fail "missing map authority sources"
[ -f "${ROOT}/README.md" ] || fail "missing runtime README.md"
[ -f "${ROOT}/env.example" ] || fail "missing runtime env.example"
[ -f "${ROOT}/RUNTIME_INDEX.md" ] || fail "missing runtime source index"
[ ! -e "${ROOT}/.gitignore" ] || fail "runtime-local .gitignore should not exist in final GitHub layout"
[ ! -e "${ROOT}/.gitattributes" ] || fail "runtime-local .gitattributes should not exist in final GitHub layout"

unexpected_root="$(find "${ROOT}" -mindepth 1 -maxdepth 1   ! -name catkin_ws   ! -name evidence   ! -name map_authority   ! -name maintenance   ! -name env.example   ! -name README.md   ! -name RUNTIME_INDEX.md   -print)"
[ -z "${unexpected_root}" ] || { printf '%s
' "${unexpected_root}" >&2; fail "unexpected runtime root entry"; }

for excluded in   "${ROOT}/runtime_state"   "${ROOT}/catkin_ws/build"   "${ROOT}/catkin_ws/devel"   "${ROOT}/catkin_ws/install"   "${ROOT}/cache"   "${ROOT}/runs"   "${ROOT}/maintenance/venvs"; do
  [ ! -e "${excluded}" ] || fail "generated/local path should not be in runtime release: ${excluded}"
done

for required_map in   render_map.py   sources/scene.yaml   sources/route_waypoints.csv   sources/visual_mesh_footprints.csv   sources/planner_obstacles.csv   sources/layer_index.csv   sources/map_manifest.json   base_world.png   obstacle_map.png   route_map.png   route_result.png; do
  [ -e "${MAP}/${required_map}" ] || fail "missing map authority file: ${required_map}"
done
cmp -s "${MAP}/sources/scene.yaml" "${PKG}/config/scenes/level_m_gps_assets_quick_complex.yaml" || fail "map authority scene.yaml differs from package scene config"

for required_evidence in   index.json   route_p0_p8/manifest.json   route_p0_p8/status.env   route_p0_p8/inputs/route_waypoints.csv   route_p0_p8/inputs/effective_scene.yaml   route_p0_p8/inputs/params.json   route_p0_p8/measurements/actual_trajectory.csv   route_p0_p8/metrics/acceptance_summary.json   route_p0_p8/metrics/metric_summary.json   route_p0_p8/metrics/metrics_full.json   route_p0_p8/metrics/waypoint_errors.csv   fc_3_10/manifest.json   fc_3_10/status.env   fc_3_10/inputs/decagon_points.csv   fc_3_10/measurements/samples.csv   fc_3_10/measurements/phases.csv   fc_3_10/metrics/summary.json   fc_3_10/metrics/geometry_audit.json; do
  [ -e "${EVIDENCE}/${required_evidence}" ] || fail "missing reference evidence: ${required_evidence}"
done

if find "${EVIDENCE}" -type f \( -name '*.log' -o -name '*.png' -o -name '*.jpg' -o -name '*.jpeg' -o -name '*.xwd' -o -name '*.html' -o -name '*.htm' -o -name '*.md' -o -name '*.txt' -o -name '*.sh' \) | grep -q .; then
  find "${EVIDENCE}" -type f \( -name '*.log' -o -name '*.png' -o -name '*.jpg' -o -name '*.jpeg' -o -name '*.xwd' -o -name '*.html' -o -name '*.htm' -o -name '*.md' -o -name '*.txt' -o -name '*.sh' \) >&2
  fail "evidence/current contains non-machine evidence files"
fi
if grep -R --binary-files=without-match --line-number '/home/adminpc/' "${EVIDENCE}" >&2; then
  fail "evidence/current contains machine-local absolute paths"
fi

[ -x "${PKG}/scripts/f250_open_control_panel.sh" ] || fail "missing control panel launcher"
[ -x "${PKG}/scripts/f250_start_to_p0_hover.sh" ] || fail "missing P0 startup launcher"
[ -x "${PKG}/scripts/f250_run_p0_p8_route.sh" ] || fail "missing route launcher"
[ -x "${PKG}/scripts/f250_run_fc_3_10_steady_state.sh" ] || fail "missing FC launcher"
[ -x "${PKG}/scripts/f250_stop_all.sh" ] || fail "missing stop script"
[ -x "${PKG}/scripts/f250_install_desktop_shortcut.sh" ] || fail "missing desktop shortcut installer"
[ -f "${PKG}/resources/f250_control_panel_icon.svg" ] || fail "missing desktop icon"
grep -Fq '$(find f250_maritime_uav_sim)/launch/f250_ego_advanced_param_px4_native_pose.xml' "${PKG}/launch/maritime_ego_planner.launch" || fail "EGO launch wrapper is not repo-local"
[ -f "${EGO}/src/uav_simulator/local_sensing/CATKIN_IGNORE" ] || fail "local_sensing should be ignored in the public build"

missing_cmake=()
while IFS= read -r relpath; do
  [ -e "${PKG}/${relpath}" ] || missing_cmake+=("${relpath}")
done < <(grep -Eo 'scripts/[A-Za-z0-9_./-]+' "${PKG}/CMakeLists.txt" | sort -u)
if [ "${#missing_cmake[@]}" -gt 0 ]; then
  printf 'missing CMake script reference: %s
' "${missing_cmake[@]}" >&2
  fail "CMake references missing script files"
fi

if find "${ROOT}" -type d \( -name __pycache__ -o -name .pytest_cache -o -name .vscode -o -name '.check_pycache.*' \) -print | grep -q .; then
  find "${ROOT}" -type d \( -name __pycache__ -o -name .pytest_cache -o -name .vscode -o -name '.check_pycache.*' \) -print >&2
  fail "excluded cache/editor directory found"
fi
if find "${ROOT}" -type f \( -name '*.pyc' -o -name '*.log' -o -name '*.zip' -o -name '*.bag' -o -name '*.xwd' \) -print | grep -q .; then
  find "${ROOT}" -type f \( -name '*.pyc' -o -name '*.log' -o -name '*.zip' -o -name '*.bag' -o -name '*.xwd' \) -print >&2
  fail "excluded cache/log/archive file found"
fi

mapfile -d '' shell_files < <(find "${ROOT}" -type f -name '*.sh' -print0 | sort -z)
[ "${#shell_files[@]}" -eq 0 ] || bash -n "${shell_files[@]}"
PYCACHE_ROOT="$(mktemp -d "${REPO_ROOT}/.release_pycache.XXXXXX")"
cleanup() { rm -rf "${PYCACHE_ROOT}"; }
trap cleanup EXIT
mapfile -d '' python_files < <(find "${PKG}" "${ROOT}/maintenance" "${MAP}" -type f -name '*.py' -print0 | sort -z)
[ "${#python_files[@]}" -eq 0 ] || PYTHONPYCACHEPREFIX="${PYCACHE_ROOT}" python3 -m py_compile "${python_files[@]}"
PYTHONPYCACHEPREFIX="${PYCACHE_ROOT}" python3 "${PKG}/scripts/f250_control_panel.py" --dry-run-self-test >/dev/null
python3 - "${ROOT}" <<'PY'
import json
import sys
from pathlib import Path
root = Path(sys.argv[1])
idx = json.loads((root / 'evidence/current/index.json').read_text(encoding='utf-8'))
for key in ('route_p0_p8', 'fc_3_10'):
    entry = idx.get('tasks', {}).get(key, {})
    if not entry.get('available'):
        raise SystemExit(f'missing available index task: {key}')
panel = (root / 'catkin_ws/src/f250_maritime_uav_sim/scripts/f250_control_panel.py').read_text(encoding='utf-8')
if 'return "lidar"' not in panel or 'def write_active_sensor' not in panel:
    raise SystemExit('control-panel fallback sensor is not LiDAR')
manifest = json.loads((root / 'map_authority/p0p8_clean_scene/sources/map_manifest.json').read_text(encoding='utf-8'))
if 'route_result.png' not in set(manifest.get('outputs', [])):
    raise SystemExit('map manifest missing route_result.png')
PY

echo "F250 runtime package check passed: ${ROOT}"
