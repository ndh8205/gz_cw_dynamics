#!/usr/bin/env bash
# Run headless thruster unit test:
#   - launches gz sim server on thruster_test.sdf
#   - runs test_thrusters.py
#   - stops gz sim

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
WORLD="${PKG_CW_SHARE}/worlds/thruster_test.sdf"
SCRIPT_INSTALLED="${HOME}/space_ros_ws/install/gz_cw_dynamics/lib/gz_cw_dynamics/test_thrusters.py"
SCRIPT_SRC="${HOME}/space_ros_ws/src/gz_cw_dynamics/scripts/test_thrusters.py"
SCRIPT="${SCRIPT_INSTALLED}"
[ -f "${SCRIPT}" ] || SCRIPT="${SCRIPT_SRC}"

export GZ_SIM_SYSTEM_PLUGIN_PATH="${HOME}/space_ros_ws/install/gz_cw_dynamics/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"

echo "[run_thruster_test] world=${WORLD}"

gz sim -s -r --verbose 2 "${WORLD}" > /tmp/gz_thruster_test.log 2>&1 &
GZ_PID=$!
echo "[run_thruster_test] gz sim server PID=${GZ_PID}"

cleanup() {
  if kill -0 "${GZ_PID}" 2>/dev/null; then
    kill -INT "${GZ_PID}" 2>/dev/null || true
    sleep 1
    kill -9 "${GZ_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

sleep 2

python3 "${SCRIPT}"
TEST_RC=$?

echo "[run_thruster_test] exit code: ${TEST_RC}"
echo "[run_thruster_test] gz server log: /tmp/gz_thruster_test.log"
exit "${TEST_RC}"
