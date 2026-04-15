#!/usr/bin/env python3
"""Chief orbit propagator as a standalone ROS 2 node.

Subscribes to Gazebo /clock (via gz-transport) and publishes chief ECI state
and sun vector in LVLH (ROS 2). Equivalent to the C++ ChiefPropagator plugin
but decoupled from gz-sim to avoid GUI rendering issues observed when a
rclcpp executor is started from inside a world-level plugin on WSL.

Also moves the sun_visual model's pose each tick via the
/world/<name>/set_pose gz-transport service.

Parameters (ROS):
  sma_km, ecc, inc_deg, raan_deg, aop_deg, ta_deg, mu
  sun_eci                     (list of 3 floats, unit vector)
  chief_state_topic, sun_lvlh_topic
  update_rate_hz
  sun_visual_distance, sun_visual_name, world_name
"""

import math
import time
from threading import Lock, Event

import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from gz.msgs10.clock_pb2 import Clock
from gz.transport13 import Node as GzNode

from sgp4.api import Satrec, WGS72


def lvlh_basis_in_eci(r_eci, v_eci):
    """Return LVLH axes expressed in ECI as (x, y, z)."""
    import numpy as np
    r = np.array(r_eci); v = np.array(v_eci)
    x = r / np.linalg.norm(r)
    z = np.cross(r, v); z = z / np.linalg.norm(z)
    y = np.cross(z, x)
    return x, y, z


def rotate_eci_to_lvlh(sun_eci, r_eci, v_eci):
    import numpy as np
    x, y, z = lvlh_basis_in_eci(r_eci, v_eci)
    R = np.stack([x, y, z])     # rows = LVLH axes in ECI
    return R @ np.array(sun_eci)


def quat_lvlh_in_eci(r_eci, v_eci):
    """Build quaternion (x, y, z, w) representing q_lvlh/eci: rotating an
    LVLH-frame vector by this quaternion yields its ECI expression.
    Equivalently, the columns of R are the LVLH basis vectors in ECI.
    """
    import numpy as np
    xh, yh, zh = lvlh_basis_in_eci(r_eci, v_eci)
    R = np.stack([xh, yh, zh], axis=1)   # columns = LVLH axes in ECI
    # Shepperd's method for stable quaternion from DCM.
    tr = R[0,0] + R[1,1] + R[2,2]
    if tr > 0:
        S = 2 * np.sqrt(1 + tr)
        qw = 0.25 * S
        qx = (R[2,1] - R[1,2]) / S
        qy = (R[0,2] - R[2,0]) / S
        qz = (R[1,0] - R[0,1]) / S
    elif R[0,0] >= R[1,1] and R[0,0] >= R[2,2]:
        S = 2 * np.sqrt(1 + R[0,0] - R[1,1] - R[2,2])
        qw = (R[2,1] - R[1,2]) / S
        qx = 0.25 * S
        qy = (R[0,1] + R[1,0]) / S
        qz = (R[0,2] + R[2,0]) / S
    elif R[1,1] >= R[2,2]:
        S = 2 * np.sqrt(1 + R[1,1] - R[0,0] - R[2,2])
        qw = (R[0,2] - R[2,0]) / S
        qx = (R[0,1] + R[1,0]) / S
        qy = 0.25 * S
        qz = (R[1,2] + R[2,1]) / S
    else:
        S = 2 * np.sqrt(1 + R[2,2] - R[0,0] - R[1,1])
        qw = (R[1,0] - R[0,1]) / S
        qx = (R[0,2] + R[2,0]) / S
        qy = (R[1,2] + R[2,1]) / S
        qz = 0.25 * S
    return (qx, qy, qz, qw)


class ChiefPropagator(Node):
    def __init__(self):
        super().__init__('chief_propagator')
        # ---- Parameters ----
        d = self.declare_parameter
        d('sma_km',              6923.137)
        d('ecc',                 0.0)
        d('inc_deg',             97.5736787997506)
        d('raan_deg',            226.932075641711)
        d('aop_deg',             171.130489033293)
        d('ta_deg',              0.0)
        d('mu',                  3.986004418e14)
        d('sun_eci',             [0.891, 0.416, 0.181])
        d('truth_topic',         '/chief/eci_truth')
        d('tle_topic',           '/chief/eci_state')      # published to students
        d('sun_lvlh_topic',      '/chief/sun_vector_lvlh')
        d('update_rate_hz',      10.0)
        d('world_name',          'gco_test')
        # TLE-like noise (3-sigma ~= 1.5 km position, 1.5 m/s velocity)
        # TLE realistic: 3-sigma ~300 m / 0.3 m/s (fresh TLE accuracy).
        # Combined with J2 drift between Keplerian-truth and SGP4-TLE this
        # gives total chief uncertainty of roughly 0.5 - 2 km over a 2-h
        # sim - about right to make VBN useful inside ~500 m.
        d('tle_pos_sigma_m',     100.0)
        d('tle_vel_sigma_mps',   0.1)
        # Epoch for SGP4 (days since 1949-12-31 00:00 UT).
        d('epoch_days_1949',     0.0)   # 0 => use 2026-04-17 default

        p = self.get_parameter
        self.a_m      = p('sma_km').value * 1000.0
        self.ecc      = p('ecc').value
        self.inc      = math.radians(p('inc_deg').value)
        self.raan     = math.radians(p('raan_deg').value)
        self.aop      = math.radians(p('aop_deg').value)
        self.ta0      = math.radians(p('ta_deg').value)
        self.mu       = p('mu').value
        self.sun_eci  = list(p('sun_eci').value)
        n0 = math.sqrt(sum(s*s for s in self.sun_eci))
        self.sun_eci  = [s / n0 for s in self.sun_eci]
        self.update_period = 1.0 / p('update_rate_hz').value
        self.world_name          = p('world_name').value

        self.tle_pos_sigma = p('tle_pos_sigma_m').value
        self.tle_vel_sigma = p('tle_vel_sigma_mps').value

        self.n  = math.sqrt(self.mu / self.a_m**3)
        self.v0 = math.sqrt(self.mu / self.a_m)

        # --- Build synthetic SGP4 satellite from our orbital elements ----
        # Epoch days since 1949-12-31 for 2026-04-17 00:00 UT = 28047.
        epoch_days = p('epoch_days_1949').value
        if epoch_days <= 0:
            from datetime import datetime
            ref = datetime(1949, 12, 31)
            ep  = datetime(2026, 4, 17)
            epoch_days = (ep - ref).total_seconds() / 86400.0
        self.sat = Satrec()
        no_kozai_rad_per_min = self.n * 60.0   # mean motion [rad/min]
        self.sat.sgp4init(
            WGS72, 'i', 99999,
            epoch_days,
            0.0,                   # bstar (drag)
            0.0,                   # ndot
            0.0,                   # nddot
            self.ecc,
            self.aop,
            self.inc,
            self.ta0,              # mean anomaly (= TA for e=0)
            no_kozai_rad_per_min,
            self.raan,
        )
        self.rng = np.random.default_rng(0)

        self.truth_pub = self.create_publisher(
            Odometry, p('truth_topic').value, 10)
        self.tle_pub   = self.create_publisher(
            Odometry, p('tle_topic').value, 10)
        self.sun_pub   = self.create_publisher(
            Vector3Stamped, p('sun_lvlh_topic').value, 10)

        # gz-transport: subscribe /clock.
        self.gz = GzNode()
        self.sim_time = 0.0
        self.lock = Lock()
        ok = self.gz.subscribe(Clock, '/clock', self._on_clock)
        if not ok:
            self.get_logger().error('failed to subscribe /clock')
        self.last_pub = 0.0

        self.get_logger().info(
            f'ChiefPropagator: a={self.a_m/1000:.3f} km, '
            f'n={self.n:.6e} rad/s, T={2*math.pi/self.n:.2f} s. '
            f'TLE noise: pos_sigma={self.tle_pos_sigma} m, '
            f'vel_sigma={self.tle_vel_sigma} m/s '
            f'(3-sigma: {3*self.tle_pos_sigma/1000:.1f} km, '
            f'{3*self.tle_vel_sigma:.2f} m/s)')

        self.timer = self.create_timer(self.update_period, self._tick)

    def _on_clock(self, msg: Clock) -> None:
        t = msg.sim.sec + msg.sim.nsec * 1e-9
        with self.lock:
            self.sim_time = t

    def _compute(self, t):
        nu = self.ta0 + self.n * t
        u  = self.aop + nu
        cu, su = math.cos(u), math.sin(u)
        cR, sR = math.cos(self.raan), math.sin(self.raan)
        ci, si = math.cos(self.inc), math.sin(self.inc)

        r_eci = (
            self.a_m * ( cu*cR - su*ci*sR),
            self.a_m * ( cu*sR + su*ci*cR),
            self.a_m * ( su*si))
        v_eci = (
            self.v0 * (-su*cR - cu*ci*sR),
            self.v0 * (-su*sR + cu*ci*cR),
            self.v0 * ( cu*si))
        sun_lvlh = rotate_eci_to_lvlh(self.sun_eci, r_eci, v_eci)
        return r_eci, v_eci, tuple(sun_lvlh)

    def _build_odo(self, t, r, v, frame='eci', child='chief'):
        odo = Odometry()
        odo.header.frame_id = frame
        odo.child_frame_id  = child
        odo.header.stamp.sec     = int(t)
        odo.header.stamp.nanosec = int((t - int(t)) * 1e9)
        odo.pose.pose.position.x = r[0]
        odo.pose.pose.position.y = r[1]
        odo.pose.pose.position.z = r[2]
        qx, qy, qz, qw = quat_lvlh_in_eci(r, v)
        odo.pose.pose.orientation.x = qx
        odo.pose.pose.orientation.y = qy
        odo.pose.pose.orientation.z = qz
        odo.pose.pose.orientation.w = qw
        odo.twist.twist.linear.x = v[0]
        odo.twist.twist.linear.y = v[1]
        odo.twist.twist.linear.z = v[2]
        _, _, zh = lvlh_basis_in_eci(r, v)
        odo.twist.twist.angular.x = self.n * zh[0]
        odo.twist.twist.angular.y = self.n * zh[1]
        odo.twist.twist.angular.z = self.n * zh[2]
        return odo

    def _sgp4_state(self, t):
        """Propagate synthetic TLE with SGP4 at elapsed sim time t (s)."""
        fr = self.sat.jdsatepochF + t / 86400.0
        e, r_km, v_kmps = self.sat.sgp4(self.sat.jdsatepoch, fr)
        if e != 0:
            return None, None
        return (np.array(r_km) * 1000.0, np.array(v_kmps) * 1000.0)

    def _tick(self):
        with self.lock:
            t = self.sim_time

        # Truth (analytical Keplerian, ECC=0).
        r_truth, v_truth, sun_lvlh = self._compute(t)

        # TLE estimate (SGP4 + Gaussian noise).
        r_sgp4, v_sgp4 = self._sgp4_state(t)
        if r_sgp4 is None:
            # SGP4 error (shouldn't happen for our elements) - skip.
            return
        r_tle = r_sgp4 + self.rng.normal(0.0, self.tle_pos_sigma, 3)
        v_tle = v_sgp4 + self.rng.normal(0.0, self.tle_vel_sigma, 3)

        # Publish both.
        self.truth_pub.publish(self._build_odo(t, r_truth, v_truth))
        self.tle_pub.publish(  self._build_odo(t, r_tle,   v_tle  ))

        # Sun vector (in LVLH, from truth — attitude reference is inertial).
        sun = Vector3Stamped()
        sun.header.stamp.sec     = int(t)
        sun.header.stamp.nanosec = int((t - int(t)) * 1e9)
        sun.header.frame_id = 'lvlh'
        sun.vector.x, sun.vector.y, sun.vector.z = sun_lvlh
        self.sun_pub.publish(sun)


def main():
    rclpy.init()
    node = ChiefPropagator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
