#!/usr/bin/env bash

# If you're changing this file, make sure to read the systems/software/scripts docs
#
# Pixi activation hook (see pixi.toml's [activation] scripts). Defends against
# a parent shell that has already sourced a native/system ROS install (eg
# `/opt/ros/jazzy/setup.bash`) or a foreign colcon workspace (eg
# ~/turtlebot3_ws) before `pixi shell`/`pixi run` activates this env. Pixi's
# own activation is additive -- it does not reset vars a different toolchain's
# activation script already set in the same shell -- so without this, those
# stray entries leak into the Pixi env and can cause ABI-mismatch crashes
# (see ERRORS.md: "gz sim segfaults from a contaminated GZ_CONFIG_PATH").
#
# Pixi runs activation scripts with bash on linux. This script must be safe
# under `set -u`, must not print anything to stdout (that would corrupt
# activation), and must not fail activation on a clean/uncontaminated shell.

set -uo pipefail

# Path-like (colon-separated) variables to strip foreign entries from.
_PIXI_SANITIZE_PATH_VARS=(
  PATH
  LD_LIBRARY_PATH
  AMENT_PREFIX_PATH
  CMAKE_PREFIX_PATH
  PYTHONPATH
  COLCON_PREFIX_PATH
  GZ_CONFIG_PATH
  GZ_SIM_RESOURCE_PATH
)

# Path fragments that mark an entry as foreign (system ROS install, or a
# colcon workspace outside this repo).
_PIXI_SANITIZE_FOREIGN_PATTERNS=(
  "/opt/ros/"
  "${HOME:-}/turtlebot3_ws"
)

_pixi_sanitize_is_foreign() {
  local entry="$1" pattern
  for pattern in "${_PIXI_SANITIZE_FOREIGN_PATTERNS[@]}"; do
    [[ -n $pattern && $entry == *"$pattern"* ]] && return 0
  done
  return 1
}

_pixi_sanitize_strip_var() {
  local var_name="$1" old_value entry
  local -a kept=()

  old_value="${!var_name:-}"
  [[ -z $old_value ]] && return 0

  local IFS=":"
  read -r -a _pixi_sanitize_entries <<<"$old_value"
  for entry in "${_pixi_sanitize_entries[@]}"; do
    [[ -z $entry ]] && continue
    if ! _pixi_sanitize_is_foreign "$entry"; then
      kept+=("$entry")
    fi
  done

  local new_value
  new_value="$(
    IFS=:
    echo "${kept[*]:-}"
  )"
  export "${var_name}=${new_value}"
}

for _pixi_sanitize_var in "${_PIXI_SANITIZE_PATH_VARS[@]}"; do
  _pixi_sanitize_strip_var "$_pixi_sanitize_var"
done

unset -f _pixi_sanitize_is_foreign _pixi_sanitize_strip_var
unset _pixi_sanitize_var _pixi_sanitize_entries
unset _PIXI_SANITIZE_PATH_VARS _PIXI_SANITIZE_FOREIGN_PATTERNS

true
