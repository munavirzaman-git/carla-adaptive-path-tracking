"""Diagnose one run: where the peak error happens, and what drives steering activity.

    python diag.py <scenario> [controller_label]      e.g. python diag.py combined adaptive_mrac_rob
"""
import sys

import numpy as np
import pandas as pd

import config as C
from reference_path import ReferencePath

sc = sys.argv[1] if len(sys.argv) > 1 else "nominal"
name = sys.argv[2] if len(sys.argv) > 2 else "adaptive_mrac_rob_prev"
p = ReferencePath.load(C.REF_FILE)
d = pd.read_csv(f"{C.RESULTS_DIR}/{sc}/{name}.csv")
k = np.interp(d.s, p.s, p.kappa)
theta = d.theta.fillna(4.3)              # non-adaptive runs: use the calibration
ff = np.degrees(np.arctan(theta * k))            # curvature-feedforward part of delta
fb = d.delta_deg - ff                             # everything else
rate = lambda x: np.sqrt(np.mean((np.diff(np.asarray(x)) / C.DT) ** 2))
print(f"steer-rate RMS [deg/s]: total {rate(d.delta_deg):.1f} | curvature part {rate(ff):.1f} | feedback part {rate(fb):.1f}")
i = d.e_y.abs().idxmax()
print(f"max |e_y| {abs(d.e_y[i]):.2f} m at s={d.s[i]:.1f}  kappa={k[i]:+.3f}  delta={d.delta_deg[i]:+.1f}  "
      f"v={d.v[i]*3.6:.1f} km/h  theta={d.theta[i]:.2f}")
x = np.asarray(fb - fb.mean()); F = np.abs(np.fft.rfft(x)); f = np.fft.rfftfreq(len(x), C.DT)
print(f"dominant feedback-steering frequency: {f[1:][F[1:].argmax()]:.2f} Hz")
if d.sigma.notna().any():
    print(f"sigma sign changes per second: {np.sum(np.diff(np.sign(d.sigma)) != 0) / d.t.iloc[-1]:.2f}")
    print(f"theta_hat: min {d.theta.min():.2f}  max {d.theta.max():.2f}  final {d.theta.iloc[-1]:.2f}")
