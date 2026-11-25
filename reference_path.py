"""Reference trajectory: Town03 route -> smoothed path (x, y, psi, kappa) + speed profile."""
import math
import os
import sys

import numpy as np

import config as C
from vehicle_state import wrap


def moving_average(a, w):
    w = max(1, int(w) | 1)  # odd window
    pad = w // 2
    return np.convolve(np.pad(a, pad, mode="edge"), np.ones(w) / w, mode="valid")


class ReferencePath:
    def __init__(self, x, y, z):
        self.x, self.y, self.z = (np.asarray(v, dtype=float) for v in (x, y, z))
        self.n = len(self.x)
        d = np.hypot(np.diff(self.x), np.diff(self.y))
        self.s = np.concatenate([[0.0], np.cumsum(d)])
        dx, dy = np.gradient(self.x, self.s), np.gradient(self.y, self.s)
        ddx, ddy = np.gradient(dx, self.s), np.gradient(dy, self.s)
        self.psi = np.arctan2(dy, dx)
        k = (dx * ddy - dy * ddx) / np.maximum((dx * dx + dy * dy) ** 1.5, 1e-9)
        self.kappa_raw = moving_average(k, 7)   # light smoothing: geometry / feasibility check
        self.kappa = moving_average(k, C.KAPPA_SMOOTH_M / max(self.s[1] - self.s[0], 1e-3))

    @property
    def length(self):
        return float(self.s[-1])

    def project(self, px, py, hint=0, back=20, fwd=200):
        """Nearest path index (windowed search around hint) and signed lateral error (left +)."""
        i0, i1 = max(0, hint - back), min(self.n, hint + fwd)
        d2 = (self.x[i0:i1] - px) ** 2 + (self.y[i0:i1] - py) ** 2
        i = i0 + int(np.argmin(d2))
        e_y = -math.sin(self.psi[i]) * (px - self.x[i]) + math.cos(self.psi[i]) * (py - self.y[i])
        return i, e_y

    def frame(self, px, py, hint=0):
        """Continuous projection onto the polyline segment: (index, e_y, psi_ref, kappa_ref).
        Avoids the step in psi_ref (~kappa*ds) every time the nearest vertex changes."""
        i, _ = self.project(px, py, hint)
        a = min(i, self.n - 2)
        if i > 0 and ((px - self.x[i]) * math.cos(self.psi[i]) + (py - self.y[i]) * math.sin(self.psi[i])) < 0:
            a = i - 1
        dx, dy = self.x[a + 1] - self.x[a], self.y[a + 1] - self.y[a]
        u = ((px - self.x[a]) * dx + (py - self.y[a]) * dy) / max(dx * dx + dy * dy, 1e-9)
        u = min(max(u, 0.0), 1.0)
        qx, qy = self.x[a] + u * dx, self.y[a] + u * dy
        psi = self.psi[a] + u * wrap(self.psi[a + 1] - self.psi[a])
        kap = self.kappa[a] + u * (self.kappa[a + 1] - self.kappa[a])
        e_y = -math.sin(psi) * (px - qx) + math.cos(psi) * (py - qy)
        return i, e_y, psi, kap

    def index_ahead(self, idx, dist):
        return min(int(np.searchsorted(self.s, self.s[idx] + dist)), self.n - 1)

    def speed_profile(self, v_max, a_lat, a_lon=C.A_LON_MAX, v_start=C.V_START, v_end=C.V_END):
        """Curvature-limited speed with forward (accel) and backward (decel) passes."""
        v = np.minimum(v_max, np.sqrt(a_lat / np.maximum(np.abs(self.kappa), 1e-4)))
        ds = np.diff(self.s)
        v[0] = min(v[0], v_start)
        for i in range(1, self.n):
            v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2 * a_lon * ds[i - 1]))
        v[-1] = min(v[-1], v_end)
        for i in range(self.n - 2, -1, -1):
            v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2 * a_lon * ds[i]))
        return v

    def save(self, fname):
        os.makedirs(os.path.dirname(fname) or ".", exist_ok=True)
        np.savez(fname, x=self.x, y=self.y, z=self.z)

    @classmethod
    def load(cls, fname):
        d = np.load(fname)
        return cls(d["x"], d["y"], d["z"])


def smooth_and_resample(pts, ds=C.PATH_DS, window_m=C.SMOOTH_WINDOW_M):
    pts = np.asarray(pts, dtype=float)
    keep = [0]
    for i in range(1, len(pts)):
        if np.hypot(*(pts[i, :2] - pts[keep[-1], :2])) > 0.1:
            keep.append(i)
    pts = pts[keep]
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(pts[:, :2], axis=0).T))])
    s_new = np.arange(0.0, s[-1], ds)
    x, y, z = (np.interp(s_new, s, pts[:, k]) for k in range(3))
    w = window_m / ds
    return ReferencePath(moving_average(x, w), moving_average(y, w), z)


# ---------------------------------------------------------------- CARLA side
def _import_grp():
    root = os.environ.get("CARLA_ROOT")
    if root:
        sys.path.append(os.path.join(root, "PythonAPI", "carla"))
    from agents.navigation.global_route_planner import GlobalRoutePlanner
    return GlobalRoutePlanner


def route_points(route):
    """CARLA waypoints -> right-handed (x, -y, z)."""
    return np.array([[wp.transform.location.x, -wp.transform.location.y, wp.transform.location.z]
                     for wp, _ in route])


def build_reference(world_map, seed=C.ROUTE_SEED, trials=C.ROUTE_TRIALS,
                    length_range=C.ROUTE_LENGTH_RANGE, start_idx=None, goal_idx=None):
    """Pick the 'curviest' route of acceptable length between Town03 spawn points."""
    GRP = _import_grp()
    grp = GRP(world_map, C.ROUTE_RESOLUTION)
    sps = world_map.get_spawn_points()

    if start_idx is not None and goal_idx is not None:
        route = grp.trace_route(sps[start_idx].location, sps[goal_idx].location)
        return smooth_and_resample(route_points(route)), (start_idx, goal_idx)

    rng = np.random.default_rng(seed)
    best, best_score, best_pair = None, -1.0, None
    for _ in range(trials):
        a, b = (int(v) for v in rng.choice(len(sps), 2, replace=False))
        route = grp.trace_route(sps[a].location, sps[b].location)
        if len(route) < 20:
            continue
        pts = route_points(route)
        length = float(np.sum(np.hypot(*np.diff(pts[:, :2], axis=0).T)))
        if not (length_range[0] <= length <= length_range[1]):
            continue
        if np.max(np.abs(smooth_and_resample(pts).kappa_raw)) > C.KAPPA_MAX:
            continue
        yaw = np.array([-math.radians(wp.transform.rotation.yaw) for wp, _ in route])
        turning = float(np.sum(np.abs([wrap(d) for d in np.diff(yaw)])))
        if turning > best_score:
            best, best_score, best_pair = pts, turning, (a, b)
    if best is None:
        raise RuntimeError("No route in the requested length range; widen ROUTE_LENGTH_RANGE.")
    print(f"[route] spawn {best_pair[0]} -> {best_pair[1]}, total turning {math.degrees(best_score):.0f} deg")
    return smooth_and_resample(best), best_pair
