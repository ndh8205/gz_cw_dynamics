#!/usr/bin/env bash
# run_full_system_test.sh — master verification for the entire sim stack.
#
# Runs every individual test in sequence, captures exit codes, prints a
# summary table at the end. Returns non-zero if any test failed.
#
# Expected total runtime: ~5 minutes.

: "${ROS_DISTRO:=jazzy}"
if [ -z "${AMENT_PREFIX_PATH:-}" ]; then
  # shellcheck disable=SC1091
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi
if [ -f "${HOME}/space_ros_ws/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${HOME}/space_ros_ws/install/setup.bash"
fi

LIB="${HOME}/space_ros_ws/install/gz_cw_dynamics/lib/gz_cw_dynamics"

tests=(
  "CW GCO 3-orbit          |run_gco_verify.sh"
  "Thrusters x6            |run_thruster_test.sh"
  "IMU (truth + LVLH + noise)|run_imu_test.sh"
  "Chief propagator        |run_chief_test.sh"
  "Reaction wheels x3      |run_rw_test.sh"
  "All sensors (7 checks)  |run_all_sensors_test.sh"
)

results=()
t_global_start=$(date +%s)

for entry in "${tests[@]}"; do
  name="${entry%%|*}"
  script="${entry##*|}"
  path="${LIB}/${script}"
  echo ""
  echo "==========================================================="
  echo "[TEST] ${name}"
  echo "==========================================================="

  if [ ! -x "${path}" ]; then
    echo "  !! MISSING: ${path}"
    results+=("FAIL|${name}|missing script")
    continue
  fi

  # Always clean any leftover simulation between tests.
  bash "${HOME}/kill_sim.sh" > /dev/null 2>&1 || true
  sleep 1

  t_start=$(date +%s)
  bash "${path}" 2>&1 | tail -20
  rc=${PIPESTATUS[0]}
  t_dur=$(( $(date +%s) - t_start ))

  if [ "${rc}" -eq 0 ]; then
    results+=("PASS|${name}|${t_dur}s")
  else
    results+=("FAIL|${name}|exit=${rc}, ${t_dur}s")
  fi
done

bash "${HOME}/kill_sim.sh" > /dev/null 2>&1 || true

t_global=$(( $(date +%s) - t_global_start ))

echo ""
echo "==========================================================="
echo "                       SUMMARY"
echo "==========================================================="
n_pass=0; n_fail=0
for r in "${results[@]}"; do
  st="${r%%|*}"
  rest="${r#*|}"
  name="${rest%%|*}"
  detail="${rest#*|}"
  printf "  [%-4s] %-30s %s\n" "${st}" "${name}" "${detail}"
  if [ "${st}" = "PASS" ]; then n_pass=$((n_pass+1)); else n_fail=$((n_fail+1)); fi
done
echo "-----------------------------------------------------------"
echo "  ${n_pass} / ${#results[@]} passed  (total ${t_global} s)"
echo "==========================================================="

if [ "${n_fail}" -gt 0 ]; then
  exit 1
fi
exit 0
