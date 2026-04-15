#!/usr/bin/env bash
set -eo pipefail

: "${ROS_DISTRO:=jazzy}"
if [ -z "${AMENT_PREFIX_PATH:-}" ]; then
  # shellcheck disable=SC1091
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi
if [ -f "${HOME}/space_ros_ws/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${HOME}/space_ros_ws/install/setup.bash"
fi

PKG_CW_SHARE="${HOME}/space_ros_ws/install/gz_cw_dynamics/share/gz_cw_dynamics"
WORLD="${PKG_CW_SHARE}/worlds/gco_verify.sdf"
SCRIPT="${HOME}/space_ros_ws/install/gz_cw_dynamics/lib/gz_cw_dynamics/test_chief.py"
[ -f "${SCRIPT}" ] || SCRIPT="${HOME}/space_ros_ws/src/gz_cw_dynamics/scripts/test_chief.py"

export GZ_SIM_SYSTEM_PLUGIN_PATH="${HOME}/space_ros_ws/install/gz_cw_dynamics/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"

gz sim -s -r --verbose 2 "${WORLD}" > /tmp/gz_chief_test.log 2>&1 &
GZ_PID=$!
cleanup() { kill -INT "${GZ_PID}" 2>/dev/null || true; sleep 1; kill -9 "${GZ_PID}" 2>/dev/null || true; }
trap cleanup EXIT INT TERM
sleep 2

python3 "${SCRIPT}"
