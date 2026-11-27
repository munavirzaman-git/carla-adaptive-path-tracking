"""Lateral controllers (Pure Pursuit, Stanley, direct MRAC) + longitudinal PI.

All lateral controllers share:   delta, info = ctrl.compute(state, path, idx, dt)
  state : vehicle_state.State (right-handed frame, delta > 0 = left)
  idx   : path index nearest to the REAR axle (hint for windowed projection)
  delta : road-wheel steering angle [rad], already saturated to +-delta_max
"""
import math
import time

import numpy as np

from vehicle_state import wrap


def clip(v, lo, hi):
    return max(lo, min(hi, v))


class PurePursuit:
    name = "pure_pursuit"

    def __init__(self, wheelbase, delta_max, k_ld=0.35, ld_min=4.0, ld_max=30.0):
        self.L, self.dmax = wheelbase, delta_max
        self.k_ld, self.ld_min, self.ld_max = k_ld, ld_min, ld_max

    def compute(self, st, path, idx, dt):
        ld = clip(self.k_ld * st.v + self.ld_min, self.ld_min, self.ld_max)
        j = path.index_ahead(idx, ld)
        dx, dy = path.x[j] - st.xr, path.y[j] - st.yr
        alpha = wrap(math.atan2(dy, dx) - st.psi)
        dist = max(math.hypot(dx, dy), 1e-3)
        delta = math.atan2(2.0 * self.L * math.sin(alpha), dist)
        return clip(delta, -self.dmax, self.dmax), {"lookahead": ld}


class Stanley:
    """Classic kinematic Stanley at the front axle. Leave curvature_ff off for the kinematic
    form: at steady state the heading term already supplies the turn angle, and adding
    atan(L kappa) double-counts it (vehicle cuts inside the curve)."""
    name = "stanley"

    def __init__(self, wheelbase, delta_max, k=1.2, k_soft=1.0, curvature_ff=False, k_yaw=0.0):
        self.L, self.dmax = wheelbase, delta_max
        self.k, self.k_soft, self.ff, self.k_yaw = k, k_soft, curvature_ff, k_yaw

    def compute(self, st, path, idx, dt):
        j, e_fa, psi_r, _ = path.frame(st.xf, st.yf, idx)
        e_psi = wrap(st.psi - psi_r)
        delta = -e_psi - math.atan2(self.k * e_fa, self.k_soft + st.v)
        if self.ff:
            delta += math.atan(self.L * path.kappa[j])
        if self.k_yaw:  # Hoffmann et al. yaw-rate damping
            delta += self.k_yaw * (path.kappa[j] * st.v - st.r)
        return clip(delta, -self.dmax, self.dmax), {"e_front": e_fa}


class DirectMRAC:
    """Direct MRAC / adaptive feedback linearisation of the heading-error loop.

    Error model at the rear axle (kinematic bicycle, UNKNOWN effective steering gain b):
        e_y'   = v sin(e_psi)
        e_psi' = b v tan(delta) - v kappa          (nominal b = 1/L; true b shrinks with
                                                     understeer, low mu, load, steer-map error)
    Sliding variable  s = e_psi + atan(lam e_y)  ->  s' = b v w + f,   w = tan(delta)
        f = -v kappa + lam v sin(e_psi) / (1 + (lam e_y)^2)            (known)
    Reference model   s_m' = -k s_m,  s_m = 0.
    Control   nu = -k s - f,   w = theta_hat nu / v,   theta = 1/b  (theta0 = L)
    Closed loop   s' = -k s + b theta_tilde nu
    Lyapunov  V = s^2/2 + b theta_tilde^2 / (2 gamma)   (needs only sign(b) > 0)
    Adaptive law  theta_hat' = -gamma s nu   =>  V' = -k s^2 <= 0
    Robustness: projection on theta_hat, sigma-modification, dead-zone on |s|,
    adaptation frozen under steering saturation and at low speed.

    Optional (all default off = the original high-gain design):
      k_e       Stanley-type surface, lam = k_e / (k_s + v): lateral rate ~k_e at every speed
      k_max     cap on the heading gain k = k0 + k1 v (delay robustness)
      adapt_tau low-pass filter on s and nu used by the adaptive law (noise / delay robustness)
      preview_t curvature taken v * preview_t ahead in the feedforward
      theta_init initial theta_hat and sigma-modification anchor (calibration)
    See PRESETS for the configurations used in the study.
    """
    name = "adaptive_mrac"

    def __init__(self, wheelbase, delta_max, lam=0.35, k0=1.0, k1=0.15, gamma=8.0,
                 sigma=0.02, theta_bounds=(0.5, 4.0), dead_zone=0.01,
                 v_floor=2.0, v_adapt_min=3.0, theta_init=None, preview_t=0.0,
                 k_e=None, k_s=1.0, k_max=None, adapt_tau=0.0):
        self.L, self.dmax = wheelbase, delta_max
        self.lam, self.k0, self.k1 = lam, k0, k1
        self.gamma, self.sigma, self.dz = gamma, sigma, dead_zone
        self.theta0 = theta_init if theta_init else wheelbase
        self.preview_t = preview_t
        self.k_e, self.k_s, self.k_max, self.adapt_tau = k_e, k_s, k_max, adapt_tau
        self.s_f = self.nu_f = 0.0
        self.tmin, self.tmax = theta_bounds[0] * wheelbase, theta_bounds[1] * wheelbase
        self.v_floor, self.v_adapt_min = v_floor, v_adapt_min
        self.theta = self.theta0

    def compute(self, st, path, idx, dt):
        j, e_y, psi_r, kap_r = path.frame(st.xr, st.yr, idx)
        e_psi = wrap(st.psi - psi_r)
        v = max(st.v, self.v_floor)
        # Stanley-type surface: lam = k_e/(k_s+v) keeps lateral convergence rate ~k_e at all speeds
        lam = self.k_e / (self.k_s + v) if self.k_e else self.lam
        s = e_psi + math.atan(lam * e_y)
        kap = path.kappa[path.index_ahead(j, v * self.preview_t)] if self.preview_t > 0 else kap_r
        f = -v * kap + lam * v * math.sin(e_psi) / (1.0 + (lam * e_y) ** 2)
        k = self.k0 + self.k1 * v
        if self.k_max:
            k = min(k, self.k_max)
        nu = -k * s - f
        delta_cmd = math.atan(self.theta * nu / v)
        saturated = abs(delta_cmd) >= self.dmax
        if self.adapt_tau > 0:   # low-pass the adaptation signals: rejects noise / delay-band content
            a = dt / (self.adapt_tau + dt)
            self.s_f += a * (s - self.s_f)
            self.nu_f += a * (nu - self.nu_f)
            sa, na = self.s_f, self.nu_f
        else:
            sa, na = s, nu
        if st.v > self.v_adapt_min and not saturated and abs(sa) > self.dz:
            dtheta = -self.gamma * sa * na - self.sigma * (self.theta - self.theta0)
            self.theta = clip(self.theta + dtheta * dt, self.tmin, self.tmax)
        return clip(delta_cmd, -self.dmax, self.dmax), {"theta": self.theta, "sigma": s}


class MPC:
    """Linear MPC on the rear-axle error model, optionally with RLS adaptation (adaptive MPC).

    Model (same kinematic bicycle error model as DirectMRAC, effective steering gain b = 1/theta):
        e_y'   = v sin(e_psi)               ~ v e_psi
        e_psi' = b v u - v kappa,           u = tan(delta)
    Exact zero-order-hold discretisation over dt_mpc with v held over the horizon:
        x+ = A x + B u + d_k,  A = [[1, v dt], [0, 1]],  B = [0.5 b v^2 dt^2, b v dt],
        d_k = [-0.5 v^2 kappa_k dt^2, -v kappa_k dt]     (kappa_k previewed along the path)
    Cost  sum x'Qx (terminal x 5) + R u^2 + Rd (du)^2,  s.t. |u| <= tan(delta_max), |du| <= rate * dt.
    Condensed QP in N inputs, solved with OSQP (falls back to unconstrained solve + clipping).

    adapt="rls": indirect adaptation. Scalar recursive least squares on the measured yaw rate,
        r = b * (v tan(delta)),  both sides low-pass filtered (tau_f) to tolerate small delays,
        forgetting factor lam, updated only with enough excitation (|v tan delta| > phi_min).
    """
    name = "mpc"

    def __init__(self, wheelbase, delta_max, theta_init=None, N=15, dt_mpc=0.1,
                 q_y=1.0, q_psi=1.0, r_u=0.05, r_du=4.0, rate_max_deg=60.0,
                 adapt="none", lam=0.995, tau_f=0.3, phi_min=0.04, p0=1e-3, v_floor=2.0, v_adapt_min=3.0,
                 delay_s=0.0, ctrl_dt=0.05, rls_ref=None):
        self.L, self.dmax = wheelbase, delta_max
        self.N, self.dt = N, dt_mpc
        self.Q = np.diag([q_y, q_psi]); self.r_u, self.r_du = r_u, r_du
        self.du_max = math.tan(math.radians(rate_max_deg) * dt_mpc)
        self.umax = math.tan(delta_max)
        self.b = 1.0 / (theta_init if theta_init else wheelbase)
        # rls_ref: the theta that RLS identifies from yaw rate on the calibration road. If given, RLS
        # tracks the RELATIVE change of the yaw gain and scales the calibrated tracking model by it.
        self.b_cal = self.b
        self.b_ref = 1.0 / rls_ref if rls_ref else None
        self.b_id = self.b_ref if rls_ref else self.b          # identified yaw-rate gain
        self.b_lo, self.b_hi = 1.0 / (4.0 * wheelbase), 2.0 / wheelbase
        self.adapt, self.lam, self.tau_f, self.phi_min, self.P = adapt, lam, tau_f, phi_min, p0
        self.v_floor, self.v_adapt_min = v_floor, v_adapt_min
        self.u_prev = 0.0
        self.r_f = self.phi_f = 0.0
        # delay compensation: commands already sent but not yet acting on the car
        self.n_delay = int(round(delay_s / ctrl_dt))
        self.pending = [0.0] * self.n_delay
        try:
            import osqp  # noqa: F401
            from scipy import sparse  # noqa: F401
            self._osqp = True
        except ImportError:
            self._osqp = False
            print("[mpc] osqp/scipy not installed: using unconstrained solve + clipping")
        D = np.eye(N) - np.eye(N, k=-1)           # rate operator: (D U)_k = u_k - u_{k-1}
        self.D = D

    def _rls(self, st, dt):
        a = dt / (self.tau_f + dt)
        self.r_f += a * (st.r - self.r_f)
        u_act = self.pending[0] if self.n_delay else self.u_prev   # command acting on the car now
        self.phi_f += a * (st.v * u_act - self.phi_f)
        if st.v < self.v_adapt_min or abs(self.phi_f) < self.phi_min:
            return
        k = self.P * self.phi_f / (self.lam + self.phi_f ** 2 * self.P)
        self.b_id = clip(self.b_id + k * (self.r_f - self.phi_f * self.b_id), self.b_lo, self.b_hi)
        self.P = clip((self.P - k * self.phi_f * self.P) / self.lam, 1e-6, 1e-1)
        self.b = self.b_cal * self.b_id / self.b_ref if self.b_ref else self.b_id

    def compute(self, st, path, idx, dt):
        if self.adapt == "rls":
            self._rls(st, dt)
        j, e_y, psi_r, _ = path.frame(st.xr, st.yr, idx)
        e_psi = wrap(st.psi - psi_r)
        v, N, h, b = max(st.v, self.v_floor), self.N, self.dt, self.b
        s0 = path.s[j]
        if self.n_delay:   # predict the error state forward through the commands still in the pipe
            dtc = 0.05                                 # control period
            for u_p in self.pending:
                k0 = float(np.interp(s0, path.s, path.kappa))
                e_y += v * math.sin(e_psi) * dtc
                e_psi += (b * v * u_p - v * k0) * dtc
                s0 += v * dtc
        kap = np.interp(s0 + v * h * np.arange(N), path.s, path.kappa)          # curvature preview
        A = np.array([[1.0, v * h], [0.0, 1.0]])
        B = np.array([0.5 * b * v * v * h * h, b * v * h])
        d = np.stack([-0.5 * v * v * kap * h * h, -v * kap * h], axis=1)        # N x 2
        # condensed prediction X = Phi x0 + Gam U + Lam   (X = [x_1 .. x_N])
        Phi = np.zeros((2 * N, 2)); Gam = np.zeros((2 * N, N)); Lam = np.zeros(2 * N)
        Ak = np.eye(2); xd = np.zeros(2)
        for k in range(N):
            Ak = A @ Ak
            Phi[2 * k:2 * k + 2] = Ak
            xd = A @ xd + d[k]
            Lam[2 * k:2 * k + 2] = xd
            for i in range(k + 1):
                Gam[2 * k:2 * k + 2, i] = np.linalg.matrix_power(A, k - i) @ B
        Qb = np.kron(np.eye(N), self.Q); Qb[-2:, -2:] *= 5.0
        x0 = np.array([e_y, e_psi])
        c = np.zeros(N); c[0] = self.u_prev
        H = Gam.T @ Qb @ Gam + self.r_u * np.eye(N) + self.r_du * self.D.T @ self.D
        g = Gam.T @ Qb @ (Phi @ x0 + Lam) - self.r_du * self.D.T @ c
        H = 0.5 * (H + H.T)
        t0 = time.perf_counter()
        U = None
        if self._osqp:
            import osqp
            from scipy import sparse
            Ac = sparse.csc_matrix(np.vstack([np.eye(N), self.D]))
            lo = np.concatenate([-self.umax * np.ones(N), c - self.du_max])
            hi = np.concatenate([self.umax * np.ones(N), c + self.du_max])
            prob = osqp.OSQP()
            prob.setup(sparse.csc_matrix(np.triu(H)), g, Ac, lo, hi, verbose=False,
                       eps_abs=1e-5, eps_rel=1e-5, max_iter=4000, polish=False)
            res = prob.solve()
            if res.x is not None and np.all(np.isfinite(res.x)):
                U = res.x
        if U is None:
            U = np.clip(np.linalg.solve(H, -g), -self.umax, self.umax)
        solve_ms = 1e3 * (time.perf_counter() - t0)
        self.u_prev = float(U[0])
        if self.n_delay:
            self.pending = self.pending[1:] + [self.u_prev]
        delta = clip(math.atan(self.u_prev), -self.dmax, self.dmax)
        return delta, {"theta": 1.0 / self.b, "sigma": 1.0 / self.b_id, "solve_ms": solve_ms}


class SpeedPI:
    def __init__(self, kp=0.35, ki=0.08, i_max=6.0, thr_max=0.8, brake_max=1.0):
        self.kp, self.ki, self.i_max, self.thr_max = kp, ki, i_max, thr_max
        self.brake_max = brake_max
        self.i = 0.0

    def step(self, v_ref, v, dt):
        e = v_ref - v
        self.i = clip(self.i + e * dt, -self.i_max, self.i_max)
        u = self.kp * e + self.ki * self.i
        return (min(u, self.thr_max), 0.0) if u >= 0 else (0.0, min(-u, self.brake_max))


CONTROLLERS = {c.name: c for c in (PurePursuit, Stanley, DirectMRAC, MPC)}

# Named configurations used in the study.  name -> (base controller, gains)
# theta_init = 4.3 m is the effective steering-gain parameter learned on dry road in CARLA
# (about 45 % above the Model 3's 2.99 m wheelbase).
_ROBUST = {"theta_init": 4.3, "k_e": 2.0, "k_max": 2.0, "adapt_tau": 0.5, "dead_zone": 0.02}
_MPC = {"r_du": 10.0, "delay_s": 0.1}   # assumed nominal actuator delay, compensated by forward prediction
PRESETS = {
    "adaptive_mrac_cal":            ("adaptive_mrac", {"theta_init": 4.3}),               # high-gain MRAC
    "adaptive_mrac_cal_fixed":      ("adaptive_mrac", {"theta_init": 4.3, "gamma": 0.0}),
    "adaptive_mrac_rob":            ("adaptive_mrac", dict(_ROBUST)),                      # MRAC-R
    "adaptive_mrac_rob_fixed":      ("adaptive_mrac", {**_ROBUST, "gamma": 0.0}),
    "adaptive_mrac_rob_prev":       ("adaptive_mrac", {**_ROBUST, "preview_t": 0.2}),     # MRAC-RP
    "adaptive_mrac_rob_prev_fixed": ("adaptive_mrac", {**_ROBUST, "preview_t": 0.2, "gamma": 0.0}),
    "mpc_geom":                     ("mpc", dict(_MPC)),                                    # theta = wheelbase
    "mpc_cal":                      ("mpc", {**_MPC, "theta_init": 4.3}),                   # calibrated, fixed
    "ampc_cal":                     ("mpc", {**_MPC, "theta_init": 4.3, "adapt": "rls"}),   # adaptive MPC (RLS)
    "ampc_ratio":                   ("mpc", {**_MPC, "theta_init": 4.3, "adapt": "rls", "rls_ref": 3.36}),
}
ALL_NAMES = list(CONTROLLERS) + list(PRESETS)


def make_controller(name, wheelbase, delta_max, **gains):
    """name = a base controller or a preset; explicit gains override the preset's."""
    if name in PRESETS:
        base, preset = PRESETS[name]
        return CONTROLLERS[base](wheelbase, delta_max, **{**preset, **gains})
    return CONTROLLERS[name](wheelbase, delta_max, **gains)
