#!/usr/bin/env python3
"""Automated unit test for the 6 thrusters in thruster_test.sdf.

For each thruster in turn:
  1. sample deputy pose for `baseline_s` seconds -> v_before (finite diff)
  2. publish throttle = 1.0 for `fire_s` seconds
  3. publish throttle = 0.0, settle for `settle_s` seconds
  4. sample pose again -> v_after (finite diff)
  5. compare Delta_v with expected = (F_max / mass) * fire_s * dir_body

Checks:
  - |Delta_v| within `vel_tol` of expected magnitude
  - Delta_v aligned with commanded direction (off-axis component < `off_tol`)
  - Angular velocity change < `omega_tol` (no induced rotation)

Requires gz sim running on world thruster_test (see thruster_test.launch.py).
"""

import math
import sys
import time
from dataclasses import dataclass
from threading import Lock

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.clock_pb2 import Clock
from gz.transport13 import Node as GzNode


MASS_KG   = 2.6
F_MAX_N   = 1.0
ACCEL     = F_MAX_N / MASS_KG   # ~0.3846 m/s^2

BASELINE_S = 1.0
FIRE_S     = 2.0
SETTLE_S   = 0.5

VEL_TOL    = 0.05   # m/s: allowed |dv_mag - expected|
OFF_TOL    = 0.02   # m/s: allowed transverse dv
OMEGA_TOL  = 0.02   # rad/s: allowed spin change


@dataclass
class Thruster:
    name: str
    topic: str
    direction: tuple  # body frame unit


THRUSTERS = [
    Thruster('fx_plus',  '/deputy/thruster/fx_plus/cmd',  ( 1,  0,  0)),
    Thruster('fx_minus', '/deputy/thruster/fx_minus/cmd', (-1,  0,  0)),
    Thruster('fy_plus',  '/deputy/thruster/fy_plus/cmd',  ( 0,  1,  0)),
    Thruster('fy_minus', '/deputy/thruster/fy_minus/cmd', ( 0, -1,  0)),
    Thruster('fz_plus',  '/deputy/thruster/fz_plus/cmd',  ( 0,  0,  1)),
    Thruster('fz_minus', '/deputy/thruster/fz_minus/cmd', ( 0,  0, -1)),
]


class PoseRecorder:
    """Collects (sim_t, x, y, z, qx, qy, qz, qw) samples for entity 'deputy'."""
    def __init__(self, world='thruster_test'):
        self.gz = GzNode()
        self.lock = Lock()
        self.sim_time = 0.0
        self.samples  = []
        if not self.gz.subscribe(Clock, '/clock', self._on_clock):
            raise RuntimeError('gz subscribe /clock failed')
        topic = f'/world/{world}/pose/info'
        if not self.gz.subscribe(Pose_V, topic, self._on_pose_v):
            raise RuntimeError(f'gz subscribe {topic} failed')

    def _on_clock(self, msg):
        with self.lock:
            self.sim_time = msg.sim.sec + msg.sim.nsec * 1e-9

    def _on_pose_v(self, msg):
        with self.lock:
            t = self.sim_time
        for p in msg.pose:
            if p.name == 'deputy':
                with self.lock:
                    self.samples.append((
                        t,
                        p.position.x, p.position.y, p.position.z,
                        p.orientation.x, p.orientation.y,
                        p.orientation.z, p.orientation.w,
                    ))
                break

    def window(self, t_center, half_window=0.3):
        with self.lock:
            lo = t_center - half_window
            hi = t_center + half_window
            return [s for s in self.samples if lo <= s[0] <= hi]


def estimate_velocity(window):
    """Linear regression (x,y,z) vs t -> (vx,vy,vz)."""
    if len(window) < 2:
        return (float('nan'),) * 3
    ts = [s[0] for s in window]
    t0 = ts[0]
    ts = [t - t0 for t in ts]
    def lin(idx):
        xs = [s[idx] for s in window]
        n = len(ts)
        mt = sum(ts) / n
        mx = sum(xs) / n
        num = sum((ts[i] - mt) * (xs[i] - mx) for i in range(n))
        den = sum((ts[i] - mt) ** 2 for i in range(n))
        return num / den if den > 0 else float('nan')
    return (lin(1), lin(2), lin(3))


def estimate_omega(window):
    """Approximate body-frame angular rate from quaternion differences.

    Returns magnitude |omega| in rad/s averaged over the window.
    """
    if len(window) < 2:
        return float('nan')
    rates = []
    for i in range(1, len(window)):
        dt = window[i][0] - window[i-1][0]
        if dt <= 0:
            continue
        q1 = window[i-1][4:8]    # (qx, qy, qz, qw)
        q2 = window[i  ][4:8]
        # dq ~ q2 * conj(q1); small-angle approx: omega ~ 2*dq_vec / dt.
        q1c = (-q1[0], -q1[1], -q1[2], q1[3])
        dq = quat_mul(q2, q1c)
        w = (2.0 * dq[0] / dt, 2.0 * dq[1] / dt, 2.0 * dq[2] / dt)
        rates.append(math.sqrt(w[0]**2 + w[1]**2 + w[2]**2))
    if not rates:
        return float('nan')
    return sum(rates) / len(rates)


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
        aw*bw - ax*bx - ay*by - az*bz,
    )


def dot(a, b):
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]


def sub(a, b):
    return (a[0]-b[0], a[1]-b[1], a[2]-b[2])


def norm(a):
    return math.sqrt(a[0]**2 + a[1]**2 + a[2]**2)


def project(v, unit):
    s = dot(v, unit)
    parallel = (unit[0]*s, unit[1]*s, unit[2]*s)
    orth = sub(v, parallel)
    return s, norm(orth)


def run():
    rclpy.init()
    ros_node = rclpy.create_node('thruster_tester')
    publishers = {
        t.topic: ros_node.create_publisher(Float32, t.topic, 10)
        for t in THRUSTERS
    }

    rec = PoseRecorder(world='thruster_test')

    # Let transport stabilize and wait for first samples.
    print('[test] waiting for pose samples ...')
    deadline = time.time() + 10.0
    while time.time() < deadline and len(rec.samples) < 10:
        rclpy.spin_once(ros_node, timeout_sec=0.1)
    if len(rec.samples) < 10:
        print('[test] ERROR: no deputy pose samples. Is gz sim running?')
        rclpy.shutdown()
        return 2

    expected_dv = ACCEL * FIRE_S   # magnitude per thruster

    print(f'[test] mass={MASS_KG} kg, F_max={F_MAX_N} N '
          f'-> a={ACCEL:.4f} m/s^2, fire={FIRE_S}s -> |dv|={expected_dv:.4f} m/s')
    print(f'[test] tolerance: vel_mag={VEL_TOL} m/s, off-axis={OFF_TOL} m/s, '
          f'omega={OMEGA_TOL} rad/s')
    print()
    print(f'{"thruster":<10} {"|dv|":>8} {"expect":>8} '
          f'{"along":>9} {"off":>8} {"|omega|":>9}  {"result":<6}')

    def sim_now():
        with rec.lock:
            return rec.sim_time

    def publish(topic, value):
        msg = Float32()
        msg.data = float(value)
        publishers[topic].publish(msg)

    def sleep_sim(duration):
        """Wait for `duration` seconds of sim time."""
        end = sim_now() + duration
        while sim_now() < end:
            rclpy.spin_once(ros_node, timeout_sec=0.05)

    # Start: stop all thrusters.
    for t in THRUSTERS:
        publish(t.topic, 0.0)
    sleep_sim(0.3)

    results = []
    for th in THRUSTERS:
        # Baseline
        t_before_mid = sim_now() + BASELINE_S / 2
        sleep_sim(BASELINE_S)
        before_win = rec.window(t_before_mid, BASELINE_S / 2 - 0.05)
        v_before = estimate_velocity(before_win)
        w_before = estimate_omega(before_win)

        # Fire (keep publishing at 10 Hz to avoid timeout).
        t_fire_start = sim_now()
        fire_end = t_fire_start + FIRE_S
        while sim_now() < fire_end:
            publish(th.topic, 1.0)
            rclpy.spin_once(ros_node, timeout_sec=0.05)
        publish(th.topic, 0.0)

        # Settle
        t_settle_start = sim_now()
        sleep_sim(SETTLE_S)
        t_after_mid = sim_now() + BASELINE_S / 2
        sleep_sim(BASELINE_S)

        after_win = rec.window(t_after_mid, BASELINE_S / 2 - 0.05)
        v_after = estimate_velocity(after_win)
        w_after = estimate_omega(after_win)

        dv = sub(v_after, v_before)
        along, off = project(dv, th.direction)
        dv_mag = norm(dv)
        d_omega = abs(w_after - w_before) if not math.isnan(w_before) \
                                          and not math.isnan(w_after) \
                                          else float('nan')

        ok_mag   = abs(dv_mag - expected_dv) <= VEL_TOL
        ok_along = (along - expected_dv) >= -VEL_TOL \
                   and abs(along - expected_dv) <= VEL_TOL
        ok_off   = off <= OFF_TOL
        ok_omega = math.isnan(d_omega) or d_omega <= OMEGA_TOL
        ok = ok_mag and ok_along and ok_off and ok_omega

        mark = 'PASS' if ok else 'FAIL'
        print(f'{th.name:<10} {dv_mag:>8.4f} {expected_dv:>8.4f} '
              f'{along:>+9.4f} {off:>8.4f} {d_omega:>9.4f}  {mark}')
        results.append((th.name, ok, dv_mag, along, off, d_omega))

    print()
    n_pass = sum(1 for r in results if r[1])
    print(f'[test] summary: {n_pass} / {len(results)} thrusters passed')

    rclpy.shutdown()
    return 0 if n_pass == len(results) else 1


if __name__ == '__main__':
    sys.exit(run())
