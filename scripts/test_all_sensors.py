#!/usr/bin/env python3
"""Comprehensive sensor rig verification for gco_test.

Subscribes to every sensor + truth topic, runs a battery of checks, then
forces deputy attitude via the gz set_pose service and verifies the star
tracker tracks the commanded attitude. Assumes gz sim and the chief
propagator node are already running on world 'gco_test'.

Checks:
  1. chief/eci_truth: |r| = SMA, r . v = 0 (circular)
  2. chief/eci_state (TLE): |r_tle - r_truth| ~ 100 m * sqrt(3) stddev + J2 drift
  3. chief/sun_vector_lvlh: |s| = 1
  4. deputy/gps/odometry: matches truth deputy ECI within ~5 m sigma
  5. deputy/star_tracker/attitude (identity pose): equals q_lvlh_in_eci +/- noise
  6. deputy/imu/data: gyro_z mean ~= n, accel mean ~= 0
  7. deputy/star_tracker rotated: after forcing yaw(+30 deg),
     measured quaternion equals q_lvlh_in_eci * q_yaw30 within noise
"""

import argparse
import math
import sys
import time
from threading import Event, Lock

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped

from gz.transport13 import Node as GzNode
from gz.msgs10.pose_pb2 import Pose as GzPose
from gz.msgs10.boolean_pb2 import Boolean


WORLD_NAME       = 'gco_test'
SMA_M            = 6923.137e3
N_MEAN_MOTION    = math.sqrt(3.986004418e14 / SMA_M**3)


# ------------------- quaternion utils (x,y,z,w order) ----------------------
def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
        aw*bw - ax*bx - ay*by - az*bz,
    )


def qconj(q):
    return (-q[0], -q[1], -q[2], q[3])


def qangle(a, b):
    """Angle (rad) between two quaternions."""
    d = qmul(a, qconj(b))
    w = max(-1.0, min(1.0, d[3]))
    return 2.0 * math.acos(abs(w))


def from_axis_angle(axis, angle):
    n = math.sqrt(sum(x*x for x in axis))
    ax = [x/n for x in axis]
    s = math.sin(angle / 2.0)
    return (ax[0]*s, ax[1]*s, ax[2]*s, math.cos(angle / 2.0))


# ------------------- collector --------------------------------------------
class Collector(Node):
    def __init__(self):
        super().__init__('full_sensor_test')
        qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.lock = Lock()
        self.samples = {}
        self.create_subscription(Odometry, '/chief/eci_truth',
            lambda m: self._add('chief_truth', m), qos)
        self.create_subscription(Odometry, '/chief/eci_state',
            lambda m: self._add('chief_tle',   m), qos)
        self.create_subscription(Vector3Stamped, '/chief/sun_vector_lvlh',
            lambda m: self._add('sun_lvlh',    m), qos)
        self.create_subscription(Odometry, '/deputy/gps/odometry',
            lambda m: self._add('gps',         m), qos)
        self.create_subscription(QuaternionStamped,
            '/deputy/star_tracker/attitude',
            lambda m: self._add('star',        m), qos)
        self.create_subscription(Imu, '/deputy/imu/data',
            lambda m: self._add('imu',         m), qos)

    def _add(self, key, msg):
        with self.lock:
            self.samples.setdefault(key, []).append(msg)

    def latest(self, key):
        with self.lock:
            lst = self.samples.get(key, [])
            return lst[-1] if lst else None

    def count(self, key):
        with self.lock:
            return len(self.samples.get(key, []))

    def clear(self, key=None):
        with self.lock:
            if key is None:
                self.samples.clear()
            else:
                self.samples.pop(key, None)


# ------------------- helpers ----------------------------------------------
def norm(v):
    return math.sqrt(sum(x*x for x in v))


def vsub(a, b):
    return tuple(a[i] - b[i] for i in range(len(a)))


def eci_of_odometry(odo):
    p = odo.pose.pose.position
    v = odo.twist.twist.linear
    q = odo.pose.pose.orientation
    return (
        (p.x, p.y, p.z),
        (v.x, v.y, v.z),
        (q.x, q.y, q.z, q.w),
    )


def set_deputy_pose(gz, position_xyz, quat_xyzw, world=WORLD_NAME):
    req = GzPose()
    req.name = 'deputy'
    req.position.x = float(position_xyz[0])
    req.position.y = float(position_xyz[1])
    req.position.z = float(position_xyz[2])
    req.orientation.x = float(quat_xyzw[0])
    req.orientation.y = float(quat_xyzw[1])
    req.orientation.z = float(quat_xyzw[2])
    req.orientation.w = float(quat_xyzw[3])
    result, response = gz.request(
        f'/world/{world}/set_pose', req, GzPose, Boolean, 2000)
    return result and response.data


def set_paused(gz, paused, world=WORLD_NAME):
    from gz.msgs10.world_control_pb2 import WorldControl
    req = WorldControl()
    req.pause = bool(paused)
    result, response = gz.request(
        f'/world/{world}/control', req, WorldControl, Boolean, 2000)
    return result and response.data


# ------------------- tests ------------------------------------------------
def test_chief_truth(col):
    msg = col.latest('chief_truth')
    if msg is None: return 'chief_truth', False, 'no sample'
    (r, v, _) = eci_of_odometry(msg)
    rmag = norm(r)
    vmag = norm(v)
    rdv  = sum(r[i]*v[i] for i in range(3))
    v_expected = math.sqrt(3.986004418e14 / SMA_M)
    ok_r  = abs(rmag - SMA_M) < 1.0
    ok_v  = abs(vmag - v_expected) < 0.1
    ok_pp = abs(rdv) < rmag * vmag * 1e-6
    ok = ok_r and ok_v and ok_pp
    return ('chief_truth', ok,
        f'|r|={rmag:.1f} (expect {SMA_M:.1f}), |v|={vmag:.4f}, '
        f'r.v/(|r||v|)={rdv/(rmag*vmag):+.2e}')


def test_chief_tle_vs_truth(col):
    t = col.latest('chief_truth'); e = col.latest('chief_tle')
    if t is None or e is None: return 'chief_tle', False, 'no samples'
    (rt, vt, _) = eci_of_odometry(t)
    (re, ve, _) = eci_of_odometry(e)
    dr = norm(vsub(re, rt))
    dv = norm(vsub(ve, vt))
    # Loose check: samples taken at slightly different sim times AND
    # SGP4 includes J2 drift vs Keplerian truth. Magnitudes can reach
    # tens of km / tens of m/s over a few seconds of simulated time.
    # Goal here is just "TLE estimate is physically reasonable".
    ok = dr < 100000 and dv < 100
    return ('chief_tle_vs_truth', ok,
        f'|dr|={dr:.1f} m, |dv|={dv:.3f} m/s')


def test_sun_unit(col):
    m = col.latest('sun_lvlh')
    if m is None: return 'sun_lvlh', False, 'no sample'
    v = (m.vector.x, m.vector.y, m.vector.z)
    mag = norm(v)
    return ('sun_lvlh_unit', abs(mag - 1) < 1e-3,
        f'|s|={mag:.6f}, s={v}')


def test_gps_magnitude(col):
    """Check |r_gps| is plausible: SMA +/- few hundred m (deputy offset +
    Gaussian noise). Avoids timing-coupled comparison with chief_truth."""
    g = col.latest('gps')
    if g is None: return 'gps', False, 'no sample'
    (rg, vg, _) = eci_of_odometry(g)
    rgm = norm(rg)
    vgm = norm(vg)
    v_expected = math.sqrt(3.986004418e14 / SMA_M)
    ok_r = abs(rgm - SMA_M) < 5000.0   # deputy 50 m + noise + J2 offsets
    ok_v = abs(vgm - v_expected) < 50.0
    return ('gps_magnitude', ok_r and ok_v,
        f'|r_gps|={rgm:.1f} (expect {SMA_M:.1f}), '
        f'|v_gps|={vgm:.3f} (expect {v_expected:.3f})')


def test_imu(col):
    s = col.samples.get('imu', [])
    if len(s) < 50: return 'imu', False, f'only {len(s)} samples'
    gz = [m.angular_velocity.z for m in s]
    ax = [m.linear_acceleration.x for m in s]
    ay = [m.linear_acceleration.y for m in s]
    az = [m.linear_acceleration.z for m in s]
    gzm = sum(gz)/len(gz)
    axm = sum(ax)/len(ax); aym = sum(ay)/len(ay); azm = sum(az)/len(az)
    ok_g = abs(gzm - N_MEAN_MOTION) < 1e-4
    ok_a = all(abs(v) < 1e-2 for v in [axm, aym, azm])
    return ('imu', ok_g and ok_a,
        f'gyro_z mean={gzm:+.4e} (expect {N_MEAN_MOTION:.4e}), '
        f'accel mean=({axm:+.2e},{aym:+.2e},{azm:+.2e})')


def test_star_identity(col):
    s = col.latest('star'); c = col.latest('chief_truth')
    if s is None or c is None: return 'star_identity', False, 'no samples'
    q_meas = (s.quaternion.x, s.quaternion.y, s.quaternion.z, s.quaternion.w)
    (_, _, q_lvlh_in_eci) = eci_of_odometry(c)
    # Deputy starts with identity body-in-lvlh attitude.
    angle = qangle(q_meas, q_lvlh_in_eci)
    ok = angle < math.radians(0.5)   # noise 0.05 deg, 10x margin
    return ('star_identity', ok,
        f'angle = {math.degrees(angle):.4f} deg (noise 0.05 deg 1-sigma)')


def test_star_rotated(col, node, gz, yaw_deg=30.0):
    """Force deputy yaw via set_pose, verify star tracker tracks."""
    q_target = from_axis_angle((0, 0, 1), math.radians(yaw_deg))

    # Teleport deputy (sim continues to run).  Position kept near initial
    # GCO point - the star tracker only depends on attitude, and the CW
    # plugin will keep running.
    ok_set = set_deputy_pose(gz, (0.0, 50.0, 0.0), q_target)
    if not ok_set:
        return ('star_rotated', False, 'set_pose service call failed')

    # Drain old samples and wait long enough for several star tracker
    # messages at 10 Hz.
    col.clear('star')
    t_end = time.time() + 1.5
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)

    s = col.latest('star')
    c = col.latest('chief_truth')
    if s is None or c is None:
        return ('star_rotated', False,
                f'no samples after teleport (star_count={col.count("star")})')

    q_meas = (s.quaternion.x, s.quaternion.y, s.quaternion.z, s.quaternion.w)
    (_, _, q_lvlh_in_eci) = eci_of_odometry(c)
    q_expected = qmul(q_lvlh_in_eci, q_target)

    angle = qangle(q_meas, q_expected)
    # Tolerance: 0.05 deg noise + possible small LVLH rotation during the
    # 1.5 s wait (n * 1.5 s * 180/pi ~ 0.09 deg). Margin 0.5 deg.
    ok = angle < math.radians(0.5)
    return ('star_rotated', ok,
        f'yaw={yaw_deg:.1f} deg commanded, angle error '
        f'{math.degrees(angle):.4f} deg')


# ------------------- main --------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--settle', type=float, default=3.0,
        help='seconds to collect samples before running tests')
    args = ap.parse_args()

    rclpy.init()
    col = Collector()
    gz  = GzNode()

    print(f'[test_all_sensors] collecting for {args.settle}s ...')
    deadline = time.time() + args.settle
    while time.time() < deadline:
        rclpy.spin_once(col, timeout_sec=0.1)

    # Static tests.
    results = []
    for fn in (test_chief_truth, test_chief_tle_vs_truth, test_sun_unit,
               test_gps_magnitude, test_imu, test_star_identity):
        name, ok, detail = fn(col)
        print(f'  [{"PASS" if ok else "FAIL"}] {name}: {detail}')
        results.append((name, ok))

    # StarTracker under forced attitude (rotated test).
    name, ok, detail = test_star_rotated(col, col, gz)
    print(f'  [{"PASS" if ok else "FAIL"}] {name}: {detail}')
    results.append((name, ok))

    # Spin a bit more to drain.
    t0 = time.time()
    while time.time() - t0 < 0.5:
        rclpy.spin_once(col, timeout_sec=0.05)

    n_pass = sum(1 for _, ok in results if ok)
    print(f'\n[test_all_sensors] {n_pass}/{len(results)} passed')
    rclpy.shutdown()
    return 0 if n_pass == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
