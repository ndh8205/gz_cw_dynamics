#!/usr/bin/env python3
"""Thruster commander — starter.

Publishes std_msgs/Float32 throttle [0, 1] to one thruster topic.

Example:
    ros2 run gz_cw_dynamics thruster_commander.py \
        --deputy deputy_docking --axis fy_plus --throttle 0.5 --duration 2.0
"""

import argparse
import sys
import time
import rclpy
from std_msgs.msg import Float32


AXES = ('fx_plus', 'fx_minus', 'fy_plus', 'fy_minus', 'fz_plus', 'fz_minus')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--deputy',   default='deputy_docking')
    ap.add_argument('--axis',     choices=AXES, default='fy_plus')
    ap.add_argument('--throttle', type=float, default=1.0)
    ap.add_argument('--duration', type=float, default=1.0)
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('thruster_commander')
    topic = f'/{args.deputy}/thruster/{args.axis}/cmd'
    pub = node.create_publisher(Float32, topic, 10)
    print(f'[fire] {topic} throttle={args.throttle:.2f} for {args.duration}s')

    t_end = time.time() + args.duration
    msg = Float32(); msg.data = float(args.throttle)
    while time.time() < t_end:
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.05)

    msg.data = 0.0
    for _ in range(5):
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.02)

    print('[fire] stopped')
    rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main() or 0)
