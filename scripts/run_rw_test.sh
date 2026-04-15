#!/usr/bin/env bash
# Reaction wheel test - uses mission.sdf, requires chief_propagator_node.
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

PKG="${HOME}/space_ros_ws/install/gz_cw_dynamics"
WORLD="${PKG}/share/gz_cw_dynamics/worlds/mission.sdf"
LIB="${PKG}/lib/gz_cw_dynamics"

export GZ_SIM_RESOURCE_PATH="${HOME}/space_ros_ws/install/orbit_sim/share/orbit_sim/models${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${PKG}/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"

gz sim -s -r --verbose 2 "${WORLD}" > /tmp/gz_rw_test.log 2>&1 &
GZ_PID=$!
cleanup() {
  kill -INT "${GZ_PID}" 2>/dev/null || true
  sleep 1
  kill -9 "${GZ_PID}" 2>/dev/null || true
  if [ -n "${CHIEF_PID:-}" ]; then
    kill -INT "${CHIEF_PID}" 2>/dev/null || true
    kill -9  "${CHIEF_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM
sleep 2

python3 "${LIB}/chief_propagator_node.py" \
    --ros-args -p world_name:=mission > /tmp/chief_rw.log 2>&1 &
CHIEF_PID=$!
sleep 2

python3 "${LIB}/test_reaction_wheel.py"
exit $?
