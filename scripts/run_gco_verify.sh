#!/usr/bin/env bash
# Headless GCO verification runner.
# Starts gz sim server-only on gco_verify.sdf, runs verify_gco.py for N
# orbits, then stops the server. CSV written to $CSV_PATH.
#
# Usage:
#   run_gco_verify.sh [orbits=3] [csv=/tmp/gco_verify.csv]

set -eo pipefail   # no -u: ROS setup.bash references unset COLCON_TRACE

ORBITS="${1:-3}"
CSV_PATH="${2:-/tmp/gco_verify.csv}"

# Source ROS / workspace if not already.
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
SCRIPT="${PKG_CW_SHARE}/../../lib/gz_cw_dynamics/verify_gco.py"
if [ ! -f "${SCRIPT}" ]; then
  SCRIPT="${HOME}/space_ros_ws/src/gz_cw_dynamics/scripts/verify_gco.py"
fi

export GZ_SIM_SYSTEM_PLUGIN_PATH="${HOME}/space_ros_ws/install/gz_cw_dynamics/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"

echo "[run_gco_verify] world=${WORLD}"
echo "[run_gco_verify] orbits=${ORBITS}, csv=${CSV_PATH}"

# Start gz sim headless in background.
gz sim -s -r --verbose 2 "${WORLD}" > /tmp/gz_gco_verify.log 2>&1 &
GZ_PID=$!
echo "[run_gco_verify] gz sim server PID=${GZ_PID}"

# Cleanup on exit.
cleanup() {
  if kill -0 "${GZ_PID}" 2>/dev/null; then
    kill -INT "${GZ_PID}" 2>/dev/null || true
    sleep 1
    kill -9 "${GZ_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

# Give gz sim a moment to bring up transport.
sleep 2

# Run the verifier (blocks until orbits complete).
python3 "${SCRIPT}" --orbits "${ORBITS}" --csv "${CSV_PATH}" --world gco_test

echo "[run_gco_verify] done. CSV at ${CSV_PATH}"
