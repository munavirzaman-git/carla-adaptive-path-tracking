"""Tables and plots from results/: python analyze.py"""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd

import config as C

OUT = os.path.join(C.RESULTS_DIR, "plots")
COLORS = {"pure_pursuit": "tab:blue", "stanley": "tab:orange", "adaptive_mrac": "tab:green"}


def main():
    os.makedirs(OUT, exist_ok=True)
    summ = pd.read_csv(os.path.join(C.RESULTS_DIR, "summary.csv"))
    pd.set_option("display.width", 160)
    for metric in ["rms_ey", "max_ey", "rms_epsi_deg", "rms_steer_rate_dps", "completion_pct"]:
        print(f"\n{metric}\n", summ.pivot(index="scenario", columns="controller", values=metric).round(3))
    summ.round(4).to_csv(os.path.join(OUT, "summary_rounded.csv"), index=False)

    # grouped bars
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.5))
    for ax, metric, lab in zip(axes, ["rms_ey", "max_ey", "rms_epsi_deg"],
                               ["RMS cross-track [m]", "Max |cross-track| [m]", "RMS heading error [deg]"]):
        summ.pivot(index="scenario", columns="controller", values=metric).plot.bar(
            ax=ax, color=[COLORS.get(c, f"C{i+3}") for i, c in enumerate(sorted(summ.controller.unique()))], rot=30)
        ax.set_ylabel(lab); ax.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "metrics_bar.png"), dpi=150); plt.close(fig)

    # per-scenario time histories
    for sc in summ.scenario.unique():
        fig, ax = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
        for ctrl in summ.controller.unique():
            f = os.path.join(C.RESULTS_DIR, sc, f"{ctrl}.csv")
            if not os.path.exists(f):
                continue
            d = pd.read_csv(f)
            kw = dict(label=ctrl, color=COLORS.get(ctrl), lw=1)
            ax[0].plot(d.s, d.e_y, **kw); ax[1].plot(d.s, d.e_psi_deg, **kw)
            ax[2].plot(d.s, d.delta_deg, **kw); ax[3].plot(d.s, d.v * 3.6, **kw)
        ax[3].plot(d.s, d.v_ref * 3.6, "k--", lw=0.8, label="v_ref")
        for a, lab in zip(ax, ["e_y [m]", "e_psi [deg]", "delta [deg]", "v [km/h]"]):
            a.set_ylabel(lab); a.grid(alpha=0.3)
        ax[0].legend(ncol=4); ax[-1].set_xlabel("path station s [m]"); ax[0].set_title(sc)
        fig.tight_layout(); fig.savefig(os.path.join(OUT, f"{sc}_traces.png"), dpi=150); plt.close(fig)

    # adaptation: theta_hat across scenarios
    fig, ax = plt.subplots(figsize=(12, 4))
    for sc in summ.scenario.unique():
        f = os.path.join(C.RESULTS_DIR, sc, "adaptive_mrac.csv")
        if os.path.exists(f):
            d = pd.read_csv(f); ax.plot(d.s, d.theta, label=sc, lw=1)
    ax.set_xlabel("s [m]"); ax.set_ylabel(r"$\hat\theta$ = 1/$\hat b$ [m]"); ax.grid(alpha=0.3); ax.legend(ncol=4)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "theta_hat.png"), dpi=150); plt.close(fig)
    print(f"\nplots -> {OUT}")


if __name__ == "__main__":
    main()
