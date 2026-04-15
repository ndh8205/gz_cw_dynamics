#!/usr/bin/env python3
"""Verify 50 m GCO formation against the closed-form CW solution.

Subscribes to /world/gco_test/pose/info and /clock, logs deputy pose +
sim-time into a CSV, and after `orbits` orbits prints statistics vs the
analytical GCO trajectory:

    x(t) = 25      sin(n t)
    y(t) = 50      cos(n t)
    z(t) = 25*sqrt(3) sin(n t)
    |r|  = 50 m   (constant)

Run with gz sim already active on world "gco_test".

    python3 verify_gco.py --orbits 3 --csv /tmp/gco.csv
"""

import argparse
import csv
import math
import signal
import sys
import time
from threading import Event, Lock

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.clock_pb2 import Clock
from gz.transport13 import Node


# --- CW / chief constants (match world SDF) ---------------------------------
MU_EARTH_KM = 3.986004418e5
A_KM        = 6923.137            # SMA of chief, km (545 km SSO)
N_RAD_S     = math.sqrt(MU_EARTH_KM / A_KM**3)   # mean motion
T_ORBIT_S   = 2.0 * math.pi / N_RAD_S
R_B_M       = 50.0                # GCO radius

# Closed-form GCO with phase=0 (matches MATLAB lvlh_check_ver1.m):
def analytic(t):
    s, c = math.sin(N_RAD_S * t), math.cos(N_RAD_S * t)
    return ( (R_B_M / 2.0) * s,
              R_B_M        * c,
             (R_B_M * math.sqrt(3.0) / 2.0) * s )


class Verifier:
    def __init__(self, orbits: float, csv_path: str, world: str):
        self.orbits   = orbits
        self.t_end    = orbits * T_ORBIT_S
        self.csv_path = csv_path
        self.world    = world

        self.node     = Node()
        self.sim_time = 0.0
        self.samples  = []        # (sim_t, x, y, z)
        self.lock     = Lock()
        self.done     = Event()

        ok = self.node.subscribe(Clock, '/clock', self._on_clock)
        if not ok:
            raise RuntimeError('Failed to subscribe /clock')
        topic = f'/world/{world}/pose/info'
        ok = self.node.subscribe(Pose_V, topic, self._on_poses)
        if not ok:
            raise RuntimeError(f'Failed to subscribe {topic}')

    def _on_clock(self, msg: Clock) -> None:
        t = msg.sim.sec + msg.sim.nsec * 1e-9
        with self.lock:
            self.sim_time = t
            if t >= self.t_end:
                self.done.set()

    def _on_poses(self, msg: Pose_V) -> None:
        with self.lock:
            t = self.sim_time
        for p in msg.pose:
            if p.name == 'deputy':
                self.samples.append((t,
                                     p.position.x,
                                     p.position.y,
                                     p.position.z))
                break

    def run(self):
        print(f'[verify_gco] n={N_RAD_S:.6e} rad/s, T={T_ORBIT_S:.2f} s'
              f' ({T_ORBIT_S/60:.2f} min), target {self.orbits} orbits ='
              f' {self.t_end:.1f} s sim-time')
        print(f'[verify_gco] logging to {self.csv_path} ...')
        self.done.wait()
        self._report()

    def _report(self):
        if not self.samples:
            print('[verify_gco] no samples captured — is sim running?')
            return

        # Write CSV.
        with open(self.csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'x', 'y', 'z', 'r',
                        'x_ref', 'y_ref', 'z_ref',
                        'err_x', 'err_y', 'err_z', 'err_r'])
            for (t, x, y, z) in self.samples:
                xr, yr, zr = analytic(t)
                r  = math.sqrt(x*x + y*y + z*z)
                w.writerow([f'{t:.6f}',
                            f'{x:.6f}', f'{y:.6f}', f'{z:.6f}', f'{r:.6f}',
                            f'{xr:.6f}', f'{yr:.6f}', f'{zr:.6f}',
                            f'{x-xr:.6f}', f'{y-yr:.6f}', f'{z-zr:.6f}',
                            f'{r-R_B_M:.6f}'])

        # Statistics.
        r_vals   = [math.sqrt(x*x + y*y + z*z) for (_, x, y, z) in self.samples]
        err_r    = [r - R_B_M for r in r_vals]
        pos_err  = []
        for (t, x, y, z) in self.samples:
            xr, yr, zr = analytic(t)
            pos_err.append(math.sqrt((x-xr)**2 + (y-yr)**2 + (z-zr)**2))

        def stats(name, v):
            n = len(v); m = sum(v)/n
            mx = max(v); mn = min(v)
            sd = math.sqrt(sum((x-m)**2 for x in v)/n)
            print(f'  {name:<18} mean={m:+.4e}  std={sd:.4e}  '
                  f'min={mn:+.4e}  max={mx:+.4e}')

        print(f'[verify_gco] samples: {len(self.samples)}')
        stats('|r| - 50 [m]',      err_r)
        stats('pos err vs CW [m]', pos_err)

        # Per-orbit in-track drift.
        print('[verify_gco] per-orbit drift (LVLH y at orbit boundaries):')
        for k in range(int(self.orbits) + 1):
            tk = k * T_ORBIT_S
            sample = min(self.samples, key=lambda s: abs(s[0]-tk))
            print(f'  orbit {k}: t={sample[0]:>8.1f} s  '
                  f'y={sample[2]:+.4f} m  r={math.sqrt(sample[1]**2+sample[2]**2+sample[3]**2):.4f} m')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--orbits', type=float, default=3.0)
    ap.add_argument('--csv',    default='/tmp/gco_verify.csv')
    ap.add_argument('--world',  default='gco_test')
    args = ap.parse_args()

    v = Verifier(args.orbits, args.csv, args.world)

    def on_sigint(*_):
        v.done.set()
    signal.signal(signal.SIGINT, on_sigint)

    try:
        v.run()
    except KeyboardInterrupt:
        v.done.set()


if __name__ == '__main__':
    main()
