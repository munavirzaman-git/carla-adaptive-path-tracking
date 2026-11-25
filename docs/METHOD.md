# Method

## Frames and sign conventions

CARLA uses a left-handed frame (x forward, y right, yaw clockwise). `sim_utils.py` is the only file
that touches CARLA coordinates; it converts once to a right-handed frame:

| Quantity | Right-handed (used everywhere else) | From CARLA |
| --- | --- | --- |
| position | (x, y) | (x, −y) |
| heading ψ | counter-clockwise | −yaw |
| yaw rate r | counter-clockwise | −ω_z |
| steering δ | > 0 turns left | CARLA steer = −δ / max_wheel_steer |

Cross-track error e_y > 0 means the reference point is left of the path; heading error
e_ψ = wrap(ψ − ψ_ref). Path heading and curvature are interpolated along the segment
(`ReferencePath.frame`), which removes a step of κ·Δs in ψ_ref every time the nearest vertex
changes. That step had roughly doubled the steering rate of Stanley and MRAC.

## Baseline controllers

**Pure Pursuit** (rear axle):

$$\delta = \arctan\frac{2L\sin\alpha}{L_d}, \qquad L_d = \mathrm{clip}(0.35\,v + 4,\ 4,\ 30)\ \mathrm{m}$$

**Stanley** (front axle, kinematic form, no curvature feedforward):

$$\delta = -e_\psi - \arctan\frac{k\,e_f}{k_s + v}, \qquad k = 1.2,\ k_s = 1.0$$

In the kinematic form the heading term already supplies the steady-state turn angle; adding
atan(Lκ) double-counts it and the car cuts corners.

## Direct MRAC

Rear-axle error dynamics of a kinematic bicycle with an unknown effective steering gain b
(nominally 1/L; it shrinks with understeer, reduced grip, load or a steering-map error):

$$\dot e_y = v\sin e_\psi, \qquad \dot e_\psi = b\,v\tan\delta - v\kappa$$

Sliding variable, with w = tan δ:

$$s = e_\psi + \arctan(\lambda e_y), \qquad \dot s = b\,v\,w + f, \qquad
f = -v\kappa + \frac{\lambda v\sin e_\psi}{1 + \lambda^2 e_y^2}$$

With θ = 1/b and its estimate θ̂:

$$\nu = -k s - f, \qquad w = \frac{\hat\theta\,\nu}{v}, \qquad
\dot s = -k s + b\,\tilde\theta\,\nu, \quad \tilde\theta = \hat\theta - \theta$$

Lyapunov function and adaptive law:

$$V = \tfrac12 s^2 + \frac{b}{2\gamma}\tilde\theta^2, \qquad
\dot{\hat\theta} = -\gamma\,s\,\nu \ \Rightarrow\ \dot V = -k s^2 \le 0$$

Only sign(b) > 0 is assumed. On s = 0 the error obeys e_ψ = −atan(λ e_y), so e_y → 0.
A time-varying gain k(t) > 0 keeps V̇ = −k(t)s² ≤ 0.

Practical modifications: θ̂ is projected to [0.5L, 4L]; σ-modification (σ = 0.02) pulls it
toward its initial value; a dead zone on |s|; adaptation is frozen under steering saturation and
below 3 m/s.

**Calibration.** On dry road CARLA's Model 3 settles at θ̂ ≈ 4.3 m, about 45 % above its 2.99 m
measured wheelbase (understeer plus a non-geometric steering map). All calibrated presets start
at θ̂ = 4.3 m.

## Robust redesign (MRAC-R)

The first calibrated design (λ = 0.35, k = 1 + 0.15 v) left the road with 150 ms actuator delay
and sensor noise, with θ̂ drifting to 10.1 m. Two causes:

1. **Gain too high for the delay.** Cancelling λ v sin e_ψ adds θ̂λ ≈ 1.5 to the heading gain,
   about 2.5·e_ψ in total against Stanley's 1.0·e_ψ, and the lateral rate λv grows with speed.
2. **Adaptation drift.** Noise and delay correlate s with ν, so the gradient law pushes θ̂ upward
   (a Rohrs-type robustness failure of unmodified MRAC).

Changes, none of which alter the Lyapunov argument:

- Stanley-type surface λ = k_e / (k_s + v), k_e = 2.0: lateral convergence ≈ k_e at every speed.
- Heading gain cap k ≤ k_max = 2.0.
- Adaptation driven by low-pass-filtered s and ν (τ = 0.5 s), dead zone 0.02.

Offline sweeps (kinematic bicycle, 0.1 m / 0.5° noise, 150 ms delay) showed a sharp stability
cliff between k = 2.0 (stable) and 2.5 (oscillating).

**MRAC-RP** adds a 0.2 s curvature preview in the feedforward (κ taken v·0.2 s ahead).

## Skid-pad grip calibration

`skidpad.py` generates a flat 300 × 120 m OpenDRIVE pad, holds 14 m/s and ramps steering slowly;
the plateau of v·r / g is the lateral grip μ_lat.

| friction_scale | tire_friction | μ_lat |
| --- | --- | --- |
| 1.0 (CARLA default) | 3.50 | 2.24 |
| 0.30 | 1.05 | 1.34 |
| 0.15 | 0.52 | 0.81 |
| 0.08 | 0.28 | 0.42 |
| 0.04 | 0.14 | 0.21 |

Below tire_friction ≈ 1, μ_lat ≈ 1.5 × tire_friction.
