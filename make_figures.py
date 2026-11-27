"""Final figures + tables for the Town03 adaptive path-tracking study.

    python make_figures.py          -> results/final/*.png, results/final/tables.md
"""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

RES = "results"
OUT = os.path.join(RES, "final")
THETA_CAL = 4.3

CTRL = {  # label in files -> display name, colour
    "pure_pursuit": ("Pure Pursuit", "#4C72B0"),
    "stanley": ("Stanley", "#DD8452"),
    "adaptive_mrac_rob": ("MRAC-R", "#55A868"),
    "adaptive_mrac_rob_prev": ("MRAC-RP", "#2E7D32"),
    "adaptive_mrac_cal": ("MRAC high-gain", "#C44E52"),
    "mpc_cal": ("MPC", "#8172B3"),
    "ampc_ratio": ("Adaptive MPC", "#DA8BC3"),
}
MAIN = ["pure_pursuit", "stanley", "adaptive_mrac_rob", "adaptive_mrac_rob_prev", "mpc_cal", "ampc_ratio"]
SCEN = ["nominal", "wet_matched", "icy_matched", "heavy_load", "wet", "icy", "high_speed", "dry_push",
        "noisy_delayed", "steer_map_error", "icy_push", "combined", "dry_real", "dry_real_push"]
PAIRS = [("adaptive_mrac_rob", "adaptive_mrac_rob_fixed", "MRAC-R", "o"),
         ("adaptive_mrac_rob_prev", "adaptive_mrac_rob_prev_fixed", "MRAC-RP", "s"),
         ("ampc_ratio", "mpc_cal", "Adaptive MPC", "^")]

plt.rcParams.update({"font.size": 10, "axes.grid": True, "grid.alpha": 0.3, "savefig.dpi": 200,
                     "savefig.bbox": "tight"})


def trace(sc, ctrl):
    f = os.path.join(RES, sc, f"{ctrl}.csv")
    return pd.read_csv(f) if os.path.exists(f) else None


def fig_rms_bars(d):
    scen = [s for s in SCEN if s in set(d.scenario)]
    fig, axes = plt.subplots(2, 1, figsize=(14, 7.5), sharex=True)
    w = 0.8 / len(MAIN)
    x = np.arange(len(scen))
    for ax, metric, lab in zip(axes, ["rms_ey", "max_ey"], ["RMS cross-track error [m]", "Max |cross-track error| [m]"]):
        for k, c in enumerate(MAIN):
            name, col = CTRL[c]
            vals, off = [], []
            for s in scen:
                r = d[(d.scenario == s) & (d.controller == c)]
                vals.append(r[metric].iloc[0] if len(r) else np.nan)
                off.append(len(r) and r.status.iloc[0] != "completed")
            vals = np.array(vals, dtype=float)
            shown = np.minimum(vals, 1.0 if metric == "rms_ey" else 3.0)
            bars = ax.bar(x + (k - (len(MAIN) - 1) / 2) * w, shown, w, label=name, color=col)
            for b, o in zip(bars, off):
                if o:
                    b.set_hatch("xx"); b.set_edgecolor("k")
                    ax.text(b.get_x() + b.get_width() / 2, b.get_height(), "off", ha="center", va="bottom", fontsize=7)
        ax.set_ylabel(lab)
    axes[0].legend(ncol=len(MAIN), loc="upper left", fontsize=8)
    axes[1].set_xticks(x)
    axes[1].set_xticklabels(scen, rotation=30, ha="right")
    axes[0].set_title("Tracking error by scenario (hatched = left the road)")
    fig.savefig(os.path.join(OUT, "fig1_error_by_scenario.png")); plt.close(fig)


def fig_adaptation_value(d):
    fig, ax = plt.subplots(figsize=(7.5, 5))
    for a, b, name, mk in PAIRS:
        A = d[d.controller == a].set_index("scenario"); B = d[d.controller == b].set_index("scenario")
        j = A.join(B, rsuffix="_f", how="inner")
        j = j[(j.status == "completed") & (j.status_f == "completed")]
        dth, dr = j.final_theta - THETA_CAL, 100 * (j.rms_ey - j.rms_ey_f) / j.rms_ey_f
        r = np.corrcoef(dth, dr)[0, 1] if len(j) > 2 else np.nan
        ax.scatter(dth, dr, marker=mk, s=45, label=f"{name}  (r = {r:+.2f})")
        for s, xv, yv in zip(j.index, dth, dr):
            ax.annotate(s, (xv, yv), fontsize=7, xytext=(3, 3), textcoords="offset points")
    ax.axhline(0, color="k", lw=0.8); ax.axvline(0, color="k", lw=0.8)
    ax.set_xlabel(r"final $\hat\theta$ − calibrated $\theta$ (4.3 m)  [m]")
    ax.set_ylabel("RMS e_y change from adapting [%]\n(negative = adaptation helps)")
    ax.set_title("Value of online adaptation vs. how far the plant moved from calibration")
    ax.legend()
    fig.savefig(os.path.join(OUT, "fig2_adaptation_value.png")); plt.close(fig)


def fig_theta(d):
    fig, ax = plt.subplots(figsize=(11, 4))
    for sc in ["nominal", "high_speed", "steer_map_error", "noisy_delayed", "icy_push", "combined"]:
        t = trace(sc, "adaptive_mrac_rob")
        if t is not None:
            ax.plot(t.s, t.theta, lw=1.2, label=sc)
    t = trace("dry_real", "adaptive_mrac_rob_prev")
    if t is not None:
        ax.plot(t.s, t.theta, "k--", lw=1.2, label="dry_real (MRAC-RP)")
    ax.axhline(THETA_CAL, color="grey", ls=":", label="calibration 4.3 m")
    ax.set_xlabel("path station s [m]"); ax.set_ylabel(r"$\hat\theta = 1/\hat b$ [m]")
    ax.set_title("Adapted steering-gain parameter along the route (MRAC-R)")
    ax.legend(ncol=4, fontsize=8)
    fig.savefig(os.path.join(OUT, "fig3_theta_hat.png")); plt.close(fig)


def fig_traces(sc, ctrls, fname, title):
    fig, ax = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    for c in ctrls:
        t = trace(sc, c)
        if t is None:
            continue
        name, col = CTRL[c]
        ax[0].plot(t.s, t.e_y, color=col, lw=1, label=name)
        ax[1].plot(t.s, t.delta_deg, color=col, lw=1, label=name)
        ax[2].plot(t.s, t.v * 3.6, color=col, lw=1, label=name)
    t = trace(sc, ctrls[0])
    if t is not None:
        ax[2].plot(t.s, t.v_ref * 3.6, "k--", lw=0.8, label="v_ref")
    for a, lab in zip(ax, ["e_y [m]", "road-wheel angle δ [deg]", "speed [km/h]"]):
        a.set_ylabel(lab)
    ax[0].legend(ncol=5, fontsize=8); ax[0].set_title(title); ax[-1].set_xlabel("path station s [m]")
    fig.savefig(os.path.join(OUT, fname)); plt.close(fig)


def tables(d):
    lines = ["# Results tables\n"]
    scen = [s for s in SCEN if s in set(d.scenario)]
    for metric, lab in [("rms_ey", "RMS e_y [m]"), ("max_ey", "max |e_y| [m]"),
                        ("rms_epsi_deg", "RMS e_psi at rear axle [deg]"), ("rms_steer_rate_dps", "RMS steering rate [deg/s]")]:
        cols = MAIN + ["adaptive_mrac_rob_fixed", "adaptive_mrac_rob_prev_fixed"]
        p = d[d.controller.isin(cols)].pivot(index="scenario", columns="controller", values=metric)
        p = p.reindex(index=scen, columns=[c for c in cols if c in p.columns])
        st = d[d.controller.isin(cols)].pivot(index="scenario", columns="controller", values="status").reindex(index=scen)
        p = p.round(3).astype(object)
        for s in p.index:
            for c in p.columns:
                if c in st.columns and st.loc[s, c] not in ("completed", np.nan) and isinstance(st.loc[s, c], str):
                    p.loc[s, c] = f"{p.loc[s, c]} ({st.loc[s, c]})"
        lines += [f"\n## {lab}\n", p.fillna("–").to_markdown()]
    # noise seeds
    lines.append("\n## Noise-seed spread (seeds 0,1,2): mean ± std of RMS e_y [m]\n")
    rows = []
    for sc in ["noisy_delayed", "combined"]:
        for base in ["pure_pursuit", "stanley", "adaptive_mrac_rob_prev"]:
            v = d[(d.scenario == sc) & d.controller.isin([base, f"{base}_s1", f"{base}_s2"])].rms_ey
            if len(v):
                rows.append({"scenario": sc, "controller": CTRL[base][0], "n": len(v),
                             "rms_ey": f"{v.mean():.3f} ± {v.std(ddof=1) if len(v) > 1 else 0:.3f}"})
    lines.append(pd.DataFrame(rows).to_markdown(index=False))
    with open(os.path.join(OUT, "tables.md"), "w") as f:
        f.write("\n".join(lines) + "\n")


def main():
    os.makedirs(OUT, exist_ok=True)
    d = pd.read_csv(os.path.join(RES, "summary.csv"))
    d = d[~d.scenario.str.contains("wet_real|snow_real")]
    fig_rms_bars(d)
    fig_adaptation_value(d)
    fig_theta(d)
    fig_traces("combined", ["pure_pursuit", "stanley", "adaptive_mrac_rob_prev"], "fig4_traces_combined.png",
               "combined: reduced grip + load + steering-map error + noise + 2-step delay")
    fig_traces("steer_map_error", ["pure_pursuit", "stanley", "adaptive_mrac_rob_prev"], "fig5_traces_steer_map.png",
               "steer_map_error: speed-dependent steering reduction (unknown actuator gain)")
    fig_traces("noisy_delayed", ["stanley", "adaptive_mrac_cal", "adaptive_mrac_rob"], "fig6_delay_robustness.png",
               "noisy_delayed: high-gain MRAC vs delay-robust MRAC-R vs Stanley")
    fig_traces("dry_real_push", ["pure_pursuit", "stanley", "adaptive_mrac_rob_prev"], "fig7_limit_failure.png",
               "dry_real_push: demand near the friction limit (MRAC leaves the road)")
    try:
        tables(d)
    except ImportError:
        print("pip install tabulate  (needed for tables.md)")
    print(f"figures and tables -> {OUT}")


if __name__ == "__main__":
    main()
