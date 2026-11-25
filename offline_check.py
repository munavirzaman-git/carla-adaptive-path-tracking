"""CARLA-free sanity check of controller signs and adaptation.
Kinematic bicycle with steering lag and an UNKNOWN steering-gain loss
(effective wheelbase m*L, i.e. understeer).   python offline_check.py"""
import math

import numpy as np

from controllers import make_controller
from reference_path import smooth_and_resample
from vehicle_state import Geometry, State, with_pose, wrap


def test_path():
    pts, x, y, h = [], 0.0, 0.0, 0.0
    def straight(L):
        nonlocal x, y
        for _ in range(int(L)):
            x += math.cos(h); y += math.sin(h); pts.append((x, y, 0.0))
    def arc(R, ang):
        nonlocal x, y, h
        n = int(abs(R * ang))
        for _ in range(n):
            h += ang / n; x += math.cos(h) * abs(R * ang) / n; y += math.sin(h) * abs(R * ang) / n
            pts.append((x, y, 0.0))
    straight(50); arc(30, math.pi / 2); straight(30); arc(20, -2 * math.pi / 3); straight(40)
    arc(25, math.pi / 3); straight(50)
    return smooth_and_resample(pts)


def simulate(name, path, gain_loss=1.0, v=10.0, tau=0.15, dt=0.05, L=2.875):
    geom = Geometry(L, -L / 2, L / 2, 70.0)
    ctrl = make_controller(name, L, math.radians(35))
    xr, yr, psi, d_act = path.x[0] + 0.5 * math.cos(path.psi[0] + 1.57), path.y[0], path.psi[0], 0.0
    idx, ey, ep, theta = 0, [], [], []
    for k in range(int(400 / dt)):
        st = with_pose(State(k * dt, xr + L / 2 * math.cos(psi), yr + L / 2 * math.sin(psi), psi, v, 0, 0),
                       xr + L / 2 * math.cos(psi), yr + L / 2 * math.sin(psi), psi, geom)
        idx, _ = path.project(st.xr, st.yr, idx)
        d_cmd, info = ctrl.compute(st, path, idx, dt)
        d_act += (d_cmd - d_act) * dt / tau
        xr += v * math.cos(psi) * dt; yr += v * math.sin(psi) * dt
        psi += v * math.tan(d_act) / (gain_loss * L) * dt
        i, e = path.project(st.x, st.y, idx)
        ey.append(e); ep.append(wrap(psi - path.psi[i])); theta.append(info.get("theta", np.nan))
        if idx >= path.n - 8:
            break
    ey = np.array(ey)
    return np.sqrt(np.mean(ey ** 2)), np.max(np.abs(ey)), math.degrees(np.sqrt(np.mean(np.square(ep)))), theta[-1]


if __name__ == "__main__":
    p = test_path()
    print(f"test path {p.length:.0f} m")
    for gl in (1.0, 1.6):
        print(f"\ngain loss x{gl} (true theta = {gl*2.875:.2f} m)")
        for c in ("pure_pursuit", "stanley", "adaptive_mrac"):
            r, mx, e, th = simulate(c, p, gl)
            print(f"  {c:14s} RMS e_y {r:.3f} m  max {mx:.3f} m  RMS e_psi {e:.2f} deg  theta_hat {th:.2f}")
