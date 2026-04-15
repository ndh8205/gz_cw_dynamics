#!/usr/bin/env python3
"""Reaction wheel commander — starter.

Publishes std_msgs/Float32 torque [N·m] to one reaction wheel joint.
By Newton's third law the wheel torque produces the opposite body torque,
which is what actually rotates the spacecraft.

Example:
    ros2 run gz_cw_dynamics rw_commander.py \
        --deputy deputy_docking --axis z --torque 0.002 --duration 3.0
"""

import argparse
import sys
import time
import rclpy
from std_msgs.msg import Float32


AXES = ('x', 'y', 'z')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--deputy',   default='deputy_docking')
    ap.add_argument('--axis',     choices=AXES, default='z')
    ap.add_argument('--torque',   type=float, default=0.001,
        help='torque [N·m], clamped by plugin to max_torque')
    ap.add_argument('--duration', type=float, default=1.0)
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('rw_commander')
    topic = f'/{args.deputy}/rw/{args.axis}/cmd'
    pub = node.create_publisher(Float32, topic, 10)
    print(f'[rw] {topic} torque={args.torque:.4f} Nm for {args.duration}s')

    t_end = time.time() + args.duration
    msg = Float32(); msg.data = float(args.torque)
    while time.time() < t_end:
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.05)

    msg.data = 0.0
    for _ in range(5):
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.02)

    print('[rw] stopped')
    rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main() or 0)
