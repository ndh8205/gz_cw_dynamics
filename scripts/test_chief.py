#!/usr/bin/env python3
"""Verify ChiefPropagator output.

Subscribes to /chief/eci_state and /chief/sun_vector_lvlh for a few seconds,
then sanity-checks the values:
  |r_eci| ~ SMA  (= 6923.137 km)
  |v_eci| ~ sqrt(mu/a)  (~7.594 km/s)
  r . v  ~ 0   (circular orbit)
  |sun_lvlh| ~ 1
"""

import math
import sys
import time
from threading import Event

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped


SMA_KM = 6923.137
MU     = 3.986004418e14  # SI
V_EXPECTED = math.sqrt(MU / (SMA_KM * 1000)) / 1000.0  # km/s


class Listener(Node):
    def __init__(self, duration: float):
        super().__init__('chief_tester')
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.eci_samples = []
        self.sun_samples = []
        self.create_subscription(Odometry, '/chief/eci_state',
                                 self._on_eci, qos)
        self.create_subscription(Vector3Stamped, '/chief/sun_vector_lvlh',
                                 self._on_sun, qos)
        self.duration = duration
        self.start = None
        self.done = Event()

    def _on_eci(self, msg: Odometry):
        if self.start is None: self.start = time.monotonic()
        self.eci_samples.append((
            msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
            msg.twist.twist.linear.x,  msg.twist.twist.linear.y,  msg.twist.twist.linear.z))
        if time.monotonic() - self.start >= self.duration:
            self.done.set()

    def _on_sun(self, msg: Vector3Stamped):
        self.sun_samples.append((msg.vector.x, msg.vector.y, msg.vector.z))


def main():
    rclpy.init()
    node = Listener(duration=3.0)
    print('[test_chief] listening for 3 s...')
    while rclpy.ok() and not node.done.is_set():
        rclpy.spin_once(node, timeout_sec=0.1)

    if not node.eci_samples:
        print('[test_chief] no eci_state messages. Is gz sim running?')
        rclpy.shutdown()
        return 2

    print(f'[test_chief] {len(node.eci_samples)} ECI samples, '
          f'{len(node.sun_samples)} sun samples')

    fail = []
    # Check all ECI samples.
    for (x, y, z, vx, vy, vz) in node.eci_samples[:5]:
        r = math.sqrt(x*x + y*y + z*z) / 1000.0   # km
        v = math.sqrt(vx*vx + vy*vy + vz*vz) / 1000.0  # km/s
        rdv = (x*vx + y*vy + z*vz) / (r*v*1e6) if r*v > 0 else 0.0
        print(f'  |r|={r:9.3f} km (expect {SMA_KM})  '
              f'|v|={v:7.4f} km/s (expect {V_EXPECTED:.4f})  '
              f'r.v/(|r||v|)={rdv:+.2e}')
        if abs(r - SMA_KM) > 1e-3:
            fail.append(f'|r|={r} != {SMA_KM}')
        if abs(v - V_EXPECTED) > 1e-4:
            fail.append(f'|v|={v} != {V_EXPECTED}')

    if node.sun_samples:
        sx, sy, sz = node.sun_samples[0]
        sm = math.sqrt(sx*sx + sy*sy + sz*sz)
        print(f'  sun_lvlh = ({sx:+.4f}, {sy:+.4f}, {sz:+.4f})  '
              f'|sun|={sm:.4f}')
        if abs(sm - 1.0) > 1e-3:
            fail.append(f'|sun_lvlh|={sm} != 1')

    if fail:
        print('[test_chief] FAIL:')
        for f in fail: print('  -', f)
        rclpy.shutdown()
        return 1
    print('[test_chief] PASS')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
