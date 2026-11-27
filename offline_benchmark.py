"""CARLA-free benchmark of every controller on a kinematic bicycle with an UNKNOWN steering gain,
first-order steering lag, and optional transport delay and sensor noise.

    python offline_benchmark.py

theta_true is the car's effective steering-gain parameter (1/b); controllers are calibrated for
4.3 m, so 3.3 / 5.5 m are plants that moved away from calibration.
"""
import collections
import math

import numpy as np

import offline_check as oc
from controllers import make_controller
from vehicle_state import Geometry, State, with_pose

L = 2.99
GEOM = Geometry(L, -1.41, 1.58, 70)
PATH = oc.test_path()


def sim(name, theta_true, v=11.1, tau=0.15, delay=0, pos_sd=0.0, yaw_sd=0.0, seed=0, gains=None):
    rng = np.random.default_rng(seed)
    c = make_controller(name, L, math.radians(40), **(gains or {}))
    p = PATH
    xr, yr, psi, d, idx = p.x[0], p.y[0], p.psi[0], 0.0, 0
    buf = collections.deque([0.0] * delay)
    ey, ds, ms, info = [], [], [], {}
    for _ in range(5000):
        x, y = xr + 1.41 * math.cos(psi), yr + 1.41 * math.sin(psi)
        st = with_pose(State(0, x, y, psi, v, v * math.tan(d) / theta_true, 0), x, y, psi, GEOM)
        m = with_pose(st, x + rng.normal(0, pos_sd) if pos_sd else x, y + rng.normal(0, pos_sd) if pos_sd else y,
                      psi + rng.normal(0, math.radians(yaw_sd)) if yaw_sd else psi, GEOM)
        idx, _ = p.project(m.xr, m.yr, idx)
        dc, info = c.compute(m, p, idx, 0.05)
        buf.append(dc); da = buf.popleft(); ds.append(da); ms.append(info.get("solve_ms", 0.0))
        d += (da - d) * 0.05 / tau
        xr += v * math.cos(psi) * 0.05; yr += v * math.sin(psi) * 0.05
        psi += v * math.tan(d) / theta_true * 0.05
        ey.append(p.frame(x, y, idx)[1])
        if abs(ey[-1]) > 5:
            return dict(status="off")
        if idx >= p.n - 8:
            break
    ey = np.array(ey)
    return dict(status="ok", rms=float(np.sqrt(np.mean(ey ** 2))), mx=float(np.abs(ey).max()),
                rate=float(np.degrees(np.sqrt(np.mean((np.diff(ds) / 0.05) ** 2)))),
                theta=info.get("theta", float("nan")), ms=float(np.mean(ms)))


CONDITIONS = {"clean θ=3.3": dict(theta_true=3.3), "clean θ=4.3": dict(theta_true=4.3),
              "clean θ=5.5": dict(theta_true=5.5), "delay θ=5.5": dict(theta_true=5.5, delay=3),
              "noise+delay θ=4.3": dict(theta_true=4.3, delay=3, pos_sd=0.1, yaw_sd=0.5),
              "noise+delay θ=5.5": dict(theta_true=5.5, delay=3, pos_sd=0.1, yaw_sd=0.5)}
CONTROLLERS = ["stanley", "adaptive_mrac_rob", "adaptive_mrac_rob_fixed", "mpc_geom", "mpc_cal", "ampc_cal"]

if __name__ == "__main__":
    print("RMS cross-track error [m] / final theta_hat [m]   (delay = 3 steps = 150 ms; noise 0.1 m, 0.5 deg)\n")
    print("controller".ljust(26) + "".join(k.rjust(19) for k in CONDITIONS))
    for c in CONTROLLERS:
        row = ""
        for cond in CONDITIONS.values():
            r = sim(c, **cond)
            cell = "off road" if r["status"] != "ok" else (
                f"{r['rms']:.3f}" + (f" / {r['theta']:.2f}" if r["theta"] == r["theta"] else ""))
            row += cell.rjust(19)
        print(c.ljust(26) + row)
    r = sim("ampc_cal", theta_true=5.5)
    print(f"\nadaptive MPC mean QP solve time: {r['ms']:.2f} ms per step (OSQP, N = 15)")
