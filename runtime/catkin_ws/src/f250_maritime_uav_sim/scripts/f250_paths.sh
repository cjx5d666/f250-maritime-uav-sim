#!/usr/bin/env bash
# Shared path resolution for the F250 runtime entry scripts.

: "${PYTHONDONTWRITEBYTECODE:=1}"
export PYTHONDONTWRITEBYTECODE

f250_abs_dir() {
	local path="$1"
	cd "${path}" 2>/dev/null && pwd -P
}

f250_resolve_project_root() {
	local script_dir="$1"
	local candidate

	if [ -n "${F250_PROJECT_ROOT:-}" ]; then
		candidate="$(f250_abs_dir "${F250_PROJECT_ROOT}")" || {
			echo "F250_PROJECT_ROOT does not exist: ${F250_PROJECT_ROOT}" >&2
			return 2
		}
		echo "${candidate}"
		return 0
	fi

	for candidate in \
		"${script_dir}/../../../.." \
		"${script_dir}/../../../../.." \
		"$(pwd -P)"; do
		candidate="$(f250_abs_dir "${candidate}")" || continue
		if [ -d "${candidate}/catkin_ws/src/f250_maritime_uav_sim" ]; then
			echo "${candidate}"
			return 0
		fi
	done

	echo "Unable to resolve F250 project root; set F250_PROJECT_ROOT." >&2
	return 2
}

f250_resolve_package_root() {
	local script_dir="$1"
	local project_root="$2"
	local candidate

	for candidate in \
		"${script_dir}/.." \
		"${project_root}/catkin_ws/src/f250_maritime_uav_sim"; do
		candidate="$(f250_abs_dir "${candidate}")" || continue
		if [ -f "${candidate}/package.xml" ] && grep -q '<name>f250_maritime_uav_sim</name>' "${candidate}/package.xml"; then
			echo "${candidate}"
			return 0
		fi
	done

	if command -v rospack >/dev/null 2>&1; then
		candidate="$(rospack find f250_maritime_uav_sim 2>/dev/null || true)"
		if [ -n "${candidate}" ] && [ -d "${candidate}" ]; then
			f250_abs_dir "${candidate}"
			return 0
		fi
	fi

	echo "Unable to resolve f250_maritime_uav_sim package root." >&2
	return 2
}

f250_resolve_px4_root() {
	local project_root="$1"
	local candidate

	if [ -n "${F250_PX4_ROOT:-}" ]; then
		candidate="$(f250_abs_dir "${F250_PX4_ROOT}")" || {
			echo "F250_PX4_ROOT does not exist: ${F250_PX4_ROOT}" >&2
			return 2
		}
		echo "${candidate}"
		return 0
	fi

	for candidate in \
		"${project_root}/PX4-Autopilot" \
		"${project_root}/PX4-Autopilot-src-main" \
		"${HOME:-}/PX4-Autopilot" \
		"${HOME:-}"/PX4-Autopilot*; do
		[ -n "${candidate}" ] || continue
		candidate="$(f250_abs_dir "${candidate}")" || continue
		if [ -f "${candidate}/launch/mavros_posix_sitl.launch" ]; then
			echo "${candidate}"
			return 0
		fi
	done

	echo "Unable to resolve PX4 root; set F250_PX4_ROOT." >&2
	return 2
}

f250_first_existing_or_first() {
	local first=""
	local candidate
	for candidate in "$@"; do
		[ -n "${first}" ] || first="${candidate}"
		if [ -e "${candidate}" ]; then
			echo "${candidate}"
			return 0
		fi
	done
	echo "${first}"
}


f250_runtime_state_dir() {
	local project_root="$1"
	local path="${F250_RUNTIME_STATE_DIR:-${project_root}/runtime_state}"
	mkdir -p "${path}"
	f250_abs_dir "${path}"
}

f250_runtime_work_dir() {
	local project_root="$1"
	local runtime_state
	runtime_state="$(f250_runtime_state_dir "${project_root}")" || return 2
	local path="${RUN_ROOT:-${runtime_state}/work}"
	mkdir -p "${path}"
	f250_abs_dir "${path}"
}

f250_active_sensor_env() {
	local project_root="$1"
	local runtime_state
	runtime_state="$(f250_runtime_state_dir "${project_root}")" || return 2
	echo "${F250_ACTIVE_SENSOR_ENV:-${runtime_state}/active_sensor.env}"
}

f250_active_task_env() {
	local project_root="$1"
	local runtime_state
	runtime_state="$(f250_runtime_state_dir "${project_root}")" || return 2
	echo "${F250_ACTIVE_TASK_ENV:-${runtime_state}/active_task.env}"
}

f250_current_evidence_dir() {
	local project_root="$1"
	local path="${F250_EVIDENCE_CURRENT_DIR:-${project_root}/evidence/current}"
	mkdir -p "${path}"
	f250_abs_dir "${path}"
}

f250_apply_env_defaults_file() {
	local defaults_file="$1"
	local line key value current

	[ -f "${defaults_file}" ] || {
		echo "missing defaults file: ${defaults_file}" >&2
		return 2
	}
	while IFS= read -r line || [ -n "${line}" ]; do
		line="${line%$'\r'}"
		case "${line}" in
		"" | \#*) continue ;;
		esac
		case "${line}" in
		*=*) ;;
		*) continue ;;
		esac
		key="${line%%=*}"
		value="${line#*=}"
		if ! [[ "${key}" =~ ^[A-Za-z_][A-Za-z0-9_]*$ ]]; then
			echo "invalid defaults key in ${defaults_file}: ${key}" >&2
			return 2
		fi
		current="${!key-}"
		if [ -z "${current}" ]; then
			export "${key}=${value}"
		else
			export "${key}"
		fi
	done <"${defaults_file}"
}

f250_apply_quick_complex_defaults() {
	local package_root="$1"
	local defaults_file="${F250_QUICK_COMPLEX_DEFAULTS:-${package_root}/config/quick_complex_defaults.env}"
	f250_apply_env_defaults_file "${defaults_file}"
}
