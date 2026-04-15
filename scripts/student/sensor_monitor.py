#!/usr/bin/env python3
"""Sensor monitor — starter.

Subscribes to all sensors of one deputy and prints a one-line status at
~2 Hz.  Intended as a learning skeleton: modify to log CSV, run filters,
etc.

Example:
    ros2 run gz_cw_dynamics sensor_monitor.py --deputy deputy_formation
"""

import argparse
import math
import sys
import time
from threading import Lock

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped
from nav_msgs.msg import Odometry


class Monitor:
    def __init__(self, deputy: str):
        self.lock = Lock()
        self.imu = None
        self.star = None
        self.gps = None
        self.sun_lvlh = None
        self.tle = None
        self.deputy = deputy

    def imu_cb(self, msg: Imu):
        with self.lock:
            self.imu = msg

    def star_cb(self, msg: QuaternionStamped):
        with self.lock:
            self.star = msg

    def gps_cb(self, msg: Odometry):
        with self.lock:
            self.gps = msg

    def sun_cb(self, msg: Vector3Stamped):
        with self.lock:
            self.sun_lvlh = msg

    def tle_cb(self, msg: Odometry):
        with self.lock:
            self.tle = msg

    def print_line(self):
        with self.lock:
            lines = []
            if self.imu:
                g = self.imu.angular_velocity
                a = self.imu.linear_acceleration
                lines.append(
                    f'  IMU  gyro=({g.x:+.2e},{g.y:+.2e},{g.z:+.2e}) rad/s  '
                    f'accel=({a.x:+.2e},{a.y:+.2e},{a.z:+.2e}) m/s2')
            if self.star:
                q = self.star.quaternion
                lines.append(
                    f'  ST   q_body/ECI=({q.x:+.4f},{q.y:+.4f},'
                    f'{q.z:+.4f},{q.w:+.4f})')
            if self.gps:
                p = self.gps.pose.pose.position
                v = self.gps.twist.twist.linear
                lines.append(
                    f'  GPS  r_ECI=({p.x:+.1f},{p.y:+.1f},{p.z:+.1f}) m  '
                    f'|v|={math.sqrt(v.x**2+v.y**2+v.z**2):.3f} m/s')
            if self.sun_lvlh:
                s = self.sun_lvlh.vector
                lines.append(
                    f'  SUN  lvlh=({s.x:+.4f},{s.y:+.4f},{s.z:+.4f})')
            if self.tle:
                p = self.tle.pose.pose.position
                lines.append(
                    f'  TLE  chief r_ECI=({p.x:+.1f},{p.y:+.1f},{p.z:+.1f})')
        if not lines:
            print('  (waiting for topics...)')
        else:
            print(f'\n=== {self.deputy} sensor snapshot ===')
            for l in lines: print(l)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--deputy', default='deputy_formation',
                    choices=('deputy_formation', 'deputy_docking'))
    ap.add_argument('--rate',   type=float, default=2.0)
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('sensor_monitor')
    qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)

    mon = Monitor(args.deputy)
    d = args.deputy
    node.create_subscription(Imu,
        f'/{d}/imu/data', mon.imu_cb, qos)
    node.create_subscription(QuaternionStamped,
        f'/{d}/star_tracker/attitude', mon.star_cb, qos)
    node.create_subscription(Odometry,
        f'/{d}/gps/odometry', mon.gps_cb, qos)
    node.create_subscription(Vector3Stamped,
        '/chief/sun_vector_lvlh', mon.sun_cb, qos)
    node.create_subscription(Odometry,
        '/chief/eci_state', mon.tle_cb, qos)

    period = 1.0 / max(args.rate, 0.1)
    try:
        while rclpy.ok():
            t_end = time.time() + period
            while time.time() < t_end:
                rclpy.spin_once(node, timeout_sec=period)
            mon.print_line()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main() or 0)
