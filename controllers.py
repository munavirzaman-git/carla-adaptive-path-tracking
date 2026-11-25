"""Lateral controllers (Pure Pursuit, Stanley, direct MRAC) + longitudinal PI.

All lateral controllers share:   delta, info = ctrl.compute(state, path, idx, dt)
  state : vehicle_state.State (right-handed frame, delta > 0 = left)
  idx   : path index nearest to the REAR axle (hint for windowed projection)
  delta : road-wheel steering angle [rad], already saturated to +-delta_max
"""
import math

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


CONTROLLERS = {c.name: c for c in (PurePursuit, Stanley, DirectMRAC)}

# Named configurations used in the study.  name -> (base controller, gains)
# theta_init = 4.3 m is the effective steering-gain parameter learned on dry road in CARLA
# (about 45 % above the Model 3's 2.99 m wheelbase).
_ROBUST = {"theta_init": 4.3, "k_e": 2.0, "k_max": 2.0, "adapt_tau": 0.5, "dead_zone": 0.02}
PRESETS = {
    "adaptive_mrac_cal":            ("adaptive_mrac", {"theta_init": 4.3}),               # high-gain MRAC
    "adaptive_mrac_cal_fixed":      ("adaptive_mrac", {"theta_init": 4.3, "gamma": 0.0}),
    "adaptive_mrac_rob":            ("adaptive_mrac", dict(_ROBUST)),                      # MRAC-R
    "adaptive_mrac_rob_fixed":      ("adaptive_mrac", {**_ROBUST, "gamma": 0.0}),
    "adaptive_mrac_rob_prev":       ("adaptive_mrac", {**_ROBUST, "preview_t": 0.2}),     # MRAC-RP
    "adaptive_mrac_rob_prev_fixed": ("adaptive_mrac", {**_ROBUST, "preview_t": 0.2, "gamma": 0.0}),
}
ALL_NAMES = list(CONTROLLERS) + list(PRESETS)


def make_controller(name, wheelbase, delta_max, **gains):
    """name = a base controller or a preset; explicit gains override the preset's."""
    if name in PRESETS:
        base, preset = PRESETS[name]
        return CONTROLLERS[base](wheelbase, delta_max, **{**preset, **gains})
    return CONTROLLERS[name](wheelbase, delta_max, **gains)
