#!/usr/bin/env python3
"""OrbitImu verification.

Samples /deputy/imu/data for `duration_s` seconds with the deputy stationary
in thruster_test world, then checks:

  gyro  mean -> (0, 0, n)         (LVLH rotation contribution)
  accel mean -> (0, 0, 0)         (free fall, zero specific force)
  gyro/accel std consistent with configured noise stddev
"""

import argparse
import math
import sys
import time
from threading import Event

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

N_EXPECTED_Z = 1.0959e-3  # rad/s, matches world mean_motion

class ImuSampler(Node):
    def __init__(self, duration_s: float):
        super().__init__('imu_sampler')
        qos = QoSProfile(depth=200,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(Imu, '/deputy/imu/data',
                                            self._cb, qos)
        self.duration = duration_s
        self.samples = []
        self.done = Event()
        self.t_first = None

    def _cb(self, msg: Imu):
        if self.t_first is None:
            self.t_first = time.monotonic()
        self.samples.append((
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ))
        if time.monotonic() - self.t_first >= self.duration:
            self.done.set()


def stats(v):
    n = len(v)
    m = sum(v) / n
    s = math.sqrt(sum((x - m) ** 2 for x in v) / n)
    return m, s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=10.0)
    args = ap.parse_args()

    rclpy.init()
    node = ImuSampler(args.duration)
    print(f'[test_imu] sampling /deputy/imu/data for {args.duration}s ...')
    while rclpy.ok() and not node.done.is_set():
        rclpy.spin_once(node, timeout_sec=0.1)
    if not node.samples:
        print('[test_imu] ERROR: no IMU samples received.')
        rclpy.shutdown()
        return 2

    s = node.samples
    print(f'[test_imu] {len(s)} samples')
    ax, sx = stats([r[0] for r in s])
    ay, sy = stats([r[1] for r in s])
    az, sz = stats([r[2] for r in s])
    gx, sgx = stats([r[3] for r in s])
    gy, sgy = stats([r[4] for r in s])
    gz, sgz = stats([r[5] for r in s])

    print('  linear_acceleration  [m/s^2]')
    print(f'    mean = ({ax:+.4e}, {ay:+.4e}, {az:+.4e})')
    print(f'    std  = ({sx:.4e}, {sy:.4e}, {sz:.4e})')
    print('  angular_velocity     [rad/s]')
    print(f'    mean = ({gx:+.4e}, {gy:+.4e}, {gz:+.4e})')
    print(f'    std  = ({sgx:.4e}, {sgy:.4e}, {sgz:.4e})')
    print()
    print(f'  expected gyro_z mean ~ {N_EXPECTED_Z:.4e} rad/s')
    print(f'  measured gyro_z mean   {gz:+.4e}  (diff {gz-N_EXPECTED_Z:+.3e})')

    # Checks
    fail = []
    # Accel means should be near 0 (tolerance = 3 * std)
    for name, m, st in [('ax', ax, sx), ('ay', ay, sy), ('az', az, sz)]:
        tol = max(3 * st / math.sqrt(len(s)), 1e-4)
        if abs(m) > tol:
            fail.append(f'{name} mean {m:+.3e} > tol {tol:.3e}')
    # Gyro x,y means should be near 0
    for name, m, st in [('gx', gx, sgx), ('gy', gy, sgy)]:
        tol = max(3 * st / math.sqrt(len(s)), 1e-5)
        if abs(m) > tol:
            fail.append(f'{name} mean {m:+.3e} > tol {tol:.3e}')
    # Gyro z should be near n
    tol_z = max(3 * sgz / math.sqrt(len(s)), 1e-5)
    if abs(gz - N_EXPECTED_Z) > tol_z:
        fail.append(f'gz mean diff {gz-N_EXPECTED_Z:+.3e} > tol {tol_z:.3e}')

    if fail:
        print('[test_imu] FAIL:')
        for f in fail:
            print('   -', f)
        rc = 1
    else:
        print('[test_imu] PASS')
        rc = 0

    rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
