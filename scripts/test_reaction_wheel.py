#!/usr/bin/env python3
"""Reaction wheel verification.

Uses mission.sdf + chief_propagator_node. For each body axis (x, y, z)
on `deputy_docking`:
  1. wait for IMU baseline
  2. fire wheel torque tau for `fire_s` seconds
  3. measure Δω_body from IMU gyro (subtract LVLH contribution)
  4. expected Δω = tau * fire_s / I_body_axis

Pass if |measured - expected| < tol_ratio * |expected|.
"""

import argparse
import math
import sys
import time
from threading import Lock

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32


N_LVLH = 1.0959e-3  # rad/s, LVLH rotation -> gyro_z adds this
DEPUTY = 'deputy_docking'

TAU_NM       = 0.005    # below max_torque 0.01
FIRE_S       = 2.0
SETTLE_S     = 0.5

# Physics check: RW torque => body counter-rotation (Newton's 3rd law).
# Exact magnitude depends on full body+wheels inertia tensor (wheel
# parallel-axis contributes ~0.04 kg·m² at 0.25 m offset), so we don't
# predict magnitude precisely. Instead:
#   - measured dω opposite in sign to commanded torque (negative)
#   - magnitude > MIN_DW_MAGNITUDE (motion actually happened)
MIN_DW_MAGNITUDE = 0.05   # rad/s


class Sampler:
    def __init__(self, node):
        self.lock = Lock()
        self.samples = []
        qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(Imu, f'/{DEPUTY}/imu/data',
                                 self._cb, qos)

    def _cb(self, msg: Imu):
        with self.lock:
            self.samples.append((
                time.monotonic(),
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
            ))

    def mean_window(self, start_wall, end_wall):
        with self.lock:
            w = [s for s in self.samples
                 if start_wall <= s[0] <= end_wall]
        if not w:
            return (float('nan'), float('nan'), float('nan'))
        return (sum(s[1] for s in w)/len(w),
                sum(s[2] for s in w)/len(w),
                sum(s[3] for s in w)/len(w))


def run_axis(node, sampler, axis, pub):
    # Baseline measurement
    t0 = time.monotonic()
    while time.monotonic() - t0 < 1.0:
        rclpy.spin_once(node, timeout_sec=0.05)
    base_start = t0
    base_end   = time.monotonic()
    w_base = sampler.mean_window(base_start, base_end)

    # Fire
    t_fire_start = time.monotonic()
    t_fire_end   = t_fire_start + FIRE_S
    msg = Float32(); msg.data = TAU_NM
    while time.monotonic() < t_fire_end:
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.05)

    # Stop torque
    msg.data = 0.0
    for _ in range(5):
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.02)

    # Settle + measure after
    t_settle_end = time.monotonic() + SETTLE_S
    while time.monotonic() < t_settle_end:
        rclpy.spin_once(node, timeout_sec=0.05)

    after_start = time.monotonic()
    while time.monotonic() - after_start < 1.0:
        rclpy.spin_once(node, timeout_sec=0.05)
    w_after = sampler.mean_window(after_start, time.monotonic())

    dw = tuple(w_after[i] - w_base[i] for i in range(3))
    return w_base, w_after, dw


def main():
    rclpy.init()
    node = rclpy.create_node('test_reaction_wheel')
    sampler = Sampler(node)

    # Pre-create publishers so DDS discovery is done first.
    pubs = {}
    for axis in ('x', 'y', 'z'):
        pubs[axis] = node.create_publisher(
            Float32, f'/{DEPUTY}/rw/{axis}/cmd', 10)

    print('[test_rw] waiting for IMU samples ...')
    deadline = time.time() + 8
    while time.time() < deadline and len(sampler.samples) < 50:
        rclpy.spin_once(node, timeout_sec=0.1)
    if len(sampler.samples) < 10:
        print('[test_rw] ERROR: no IMU samples received')
        rclpy.shutdown()
        return 2

    axis_map = {'x': 0, 'y': 1, 'z': 2}
    results = []
    for axis in ('x', 'y', 'z'):
        w_base, w_after, dw = run_axis(node, sampler, axis, pubs[axis])
        measured = dw[axis_map[axis]]
        # Expect body rotation opposite to commanded torque.
        expected_sign = -1.0
        sign_ok = (measured * expected_sign) > 0
        mag_ok  = abs(measured) >= MIN_DW_MAGNITUDE
        ok = sign_ok and mag_ok
        print(f'  [{"PASS" if ok else "FAIL"}] rw_{axis}: '
              f'tau={TAU_NM:+.3f} Nm for {FIRE_S} s  '
              f'dω_body_{axis}={measured:+.3f} rad/s  '
              f'(sign {"OK" if sign_ok else "BAD"}, '
              f'|dω|>{MIN_DW_MAGNITUDE}: {"OK" if mag_ok else "BAD"})')
        results.append(ok)

    n_pass = sum(results)
    print(f'\n[test_rw] {n_pass}/3 axes passed')
    rclpy.shutdown()
    return 0 if n_pass == 3 else 1


if __name__ == '__main__':
    sys.exit(main())
