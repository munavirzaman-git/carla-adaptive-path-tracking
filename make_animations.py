"""Data-driven media for the README (no CARLA needed): banner, route map, scoreboard heatmap
and animated GIFs built from the logged runs in results/.

    python make_animations.py            -> media/*.png, media/gifs/*.gif
"""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.collections import LineCollection

import config as C
from reference_path import ReferencePath

OUT = "media"
GIFS = os.path.join(OUT, "gifs")
BG, PANEL, GRID, TXT, SUB = "#0d1117", "#161b22", "#30363d", "#e6edf3", "#8b949e"
COL = {"pure_pursuit": "#4C9BE8", "stanley": "#FF9F1C", "adaptive_mrac_rob": "#2EC4B6",
       "adaptive_mrac_rob_prev": "#7BD389", "adaptive_mrac_cal": "#FF5E78"}
NAME = {"pure_pursuit": "Pure Pursuit", "stanley": "Stanley", "adaptive_mrac_rob": "MRAC-R",
        "adaptive_mrac_rob_prev": "MRAC-RP", "adaptive_mrac_cal": "high-gain MRAC"}
SCEN = ["nominal", "wet_matched", "icy_matched", "heavy_load", "wet", "icy", "high_speed", "dry_push",
        "noisy_delayed", "steer_map_error", "icy_push", "combined", "dry_real", "dry_real_push"]
MAIN = ["pure_pursuit", "stanley", "adaptive_mrac_rob", "adaptive_mrac_rob_prev"]


def run(sc, ctrl):
    f = os.path.join(C.RESULTS_DIR, sc, f"{ctrl}.csv")
    return pd.read_csv(f) if os.path.exists(f) else None


def dark(ax, grid=True):
    ax.set_facecolor(PANEL)
    for sp in ax.spines.values():
        sp.set_color(GRID)
    ax.tick_params(colors=SUB, labelsize=9)
    ax.xaxis.label.set_color(SUB); ax.yaxis.label.set_color(SUB); ax.title.set_color(TXT)
    if grid:
        ax.grid(color=GRID, lw=0.6, alpha=0.7)


def colored_route(ax, path, values, cmap, lw=5, vmin=None, vmax=None):
    pts = np.column_stack([path.x, path.y]).reshape(-1, 1, 2)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    lc = LineCollection(segs, cmap=cmap, linewidths=lw, capstyle="round")
    lc.set_array(values[:-1]); lc.set_clim(vmin if vmin is not None else values.min(),
                                           vmax if vmax is not None else values.max())
    ax.add_collection(lc)
    ax.set_xlim(path.x.min() - 15, path.x.max() + 15); ax.set_ylim(path.y.min() - 15, path.y.max() + 15)
    ax.set_aspect("equal")
    return lc


# ----------------------------------------------------------------------------- static images
def banner(path):
    v = path.speed_profile(40 / 3.6, 3.0) * 3.6
    fig = plt.figure(figsize=(14, 3.6), facecolor=BG)
    axt = fig.add_axes([0.02, 0.0, 0.58, 1.0]); axt.axis("off")
    axt.text(0.02, 0.70, "Adaptive Vehicle Path Tracking", color=TXT, fontsize=30, weight="bold")
    axt.text(0.02, 0.50, "in CARLA", color="#7BD389", fontsize=30, weight="bold")
    axt.text(0.02, 0.30, "Pure Pursuit  ·  Stanley  ·  Delay-robust direct MRAC", color=SUB, fontsize=14)
    chips = [("CARLA 0.9.16", "#4C9BE8"), ("Town03", "#FF9F1C"), ("Tesla Model 3", "#FF5E78"),
             ("14 conditions", "#2EC4B6"), ("Lyapunov MRAC", "#7BD389")]
    x = 0.02
    renderer = fig.canvas.get_renderer()
    for label, col in chips:
        t = axt.text(x, 0.10, f" {label} ", color=BG, fontsize=11, weight="bold",
                     bbox=dict(boxstyle="round,pad=0.35", fc=col, ec="none"))
        bb = t.get_window_extent(renderer).transformed(axt.transAxes.inverted())
        x = bb.x1 + 0.022
    axr = fig.add_axes([0.60, 0.04, 0.39, 0.92]); axr.axis("off"); axr.set_facecolor(BG)
    colored_route(axr, path, v, "plasma", lw=6)
    axr.plot(path.x[0], path.y[0], "o", ms=11, color="#7BD389", mec="white", mew=1.5)
    axr.plot(path.x[-1], path.y[-1], "s", ms=10, color="#FF5E78", mec="white", mew=1.5)
    fig.savefig(os.path.join(OUT, "banner.png"), dpi=110, facecolor=BG)
    plt.close(fig)


def route_map(path):
    v = path.speed_profile(40 / 3.6, 3.0) * 3.6
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.2), facecolor=BG)
    for ax, vals, cmap, lab in [(axes[0], np.abs(path.kappa), "viridis", "|curvature| [1/m]"),
                                (axes[1], v, "plasma", "speed profile, nominal [km/h]")]:
        dark(ax, grid=False); ax.set_xticks([]); ax.set_yticks([])
        lc = colored_route(ax, path, vals, cmap)
        cb = fig.colorbar(lc, ax=ax, fraction=0.04, pad=0.02)
        cb.ax.tick_params(colors=SUB); cb.set_label(lab, color=SUB); cb.outline.set_edgecolor(GRID)
        ax.plot(path.x[0], path.y[0], "o", ms=10, color="#7BD389", mec="white", mew=1.5, label="start")
        ax.plot(path.x[-1], path.y[-1], "s", ms=9, color="#FF5E78", mec="white", mew=1.5, label="finish")
        ax.legend(facecolor=PANEL, edgecolor=GRID, labelcolor=TXT, loc="lower right", fontsize=9)
    fig.suptitle(f"Town03 reference route: {path.length:.0f} m, {path.n} points", color=TXT, fontsize=14)
    fig.savefig(os.path.join(OUT, "route_map.png"), dpi=110, facecolor=BG, bbox_inches="tight")
    plt.close(fig)


def scoreboard():
    d = pd.read_csv(os.path.join(C.RESULTS_DIR, "summary.csv"))
    scen = [s for s in SCEN if s in set(d.scenario)]
    val = np.full((len(scen), len(MAIN)), np.nan); stat = np.full(val.shape, "", dtype=object)
    for i, s in enumerate(scen):
        for j, c in enumerate(MAIN):
            r = d[(d.scenario == s) & (d.controller == c)]
            if len(r):
                val[i, j] = r.rms_ey.iloc[0]; stat[i, j] = r.status.iloc[0]
    ratio = val / np.nanmin(np.where(stat == "completed", val, np.nan), axis=1, keepdims=True)
    ratio = np.where(stat == "completed", ratio, 4.0)
    ratio = np.where(np.isnan(val), np.nan, ratio)
    fig, ax = plt.subplots(figsize=(9, 8.5), facecolor=BG)
    dark(ax, grid=False)
    cmap = plt.get_cmap("RdYlGn_r").copy(); cmap.set_bad(PANEL)
    im = ax.imshow(np.ma.masked_invalid(np.clip(ratio, 1, 4)), cmap=cmap, vmin=1, vmax=3.5, aspect="auto")
    for i in range(len(scen)):
        for j in range(len(MAIN)):
            if np.isnan(val[i, j]):
                txt, col = "–", SUB
            elif stat[i, j] != "completed":
                txt, col = "left road", "white"
            else:
                best = ratio[i, j] <= 1.0001
                txt = f"★ {val[i, j]:.3f}" if best else f"{val[i, j]:.3f}"
                col = "white" if (ratio[i, j] < 1.35 or ratio[i, j] > 2.9) else "black"
            ax.text(j, i, txt, ha="center", va="center", fontsize=10, color=col,
                    weight="bold" if txt.startswith("★") else "normal")
    ax.set_xticks(range(len(MAIN)), [NAME[c] for c in MAIN], color=TXT, fontsize=11)
    ax.set_yticks(range(len(scen)), scen, color=TXT, fontsize=10)
    ax.xaxis.tick_top()
    cb = fig.colorbar(im, ax=ax, fraction=0.04, pad=0.02)
    cb.ax.tick_params(colors=SUB); cb.outline.set_edgecolor(GRID)
    cb.set_label("RMS cross-track error / best in scenario", color=SUB)
    ax.set_title("Scoreboard: RMS cross-track error [m]   (★ = best)", color=TXT, fontsize=13, pad=40)
    fig.savefig(os.path.join(OUT, "scoreboard.png"), dpi=110, facecolor=BG, bbox_inches="tight")
    plt.close(fig)


# ----------------------------------------------------------------------------- animations
def tracking_gif(path, sc, ctrls, fname, title, frames=140, fps=12):
    runs = {c: run(sc, c) for c in ctrls}
    runs = {c: d for c, d in runs.items() if d is not None and len(d) > 2}
    if not runs:
        print(f"  skip {fname}: no data"); return
    tmax = max(d.t.iloc[-1] for d in runs.values())
    times = np.linspace(0, tmax, frames)
    fig = plt.figure(figsize=(12.5, 5.8), facecolor=BG)
    gs = fig.add_gridspec(2, 2, width_ratios=[1, 1.3], hspace=0.35, wspace=0.12,
                          left=0.03, right=0.98, top=0.86, bottom=0.1)
    axm, axe, axd = fig.add_subplot(gs[:, 0]), fig.add_subplot(gs[0, 1]), fig.add_subplot(gs[1, 1])
    dark(axm, grid=False); axm.set_xticks([]); axm.set_yticks([])
    dark(axe); dark(axd)
    axm.plot(path.x, path.y, color="#2b313b", lw=9, solid_capstyle="round")
    axm.plot(path.x, path.y, color=SUB, lw=0.8, ls="--")
    axm.set_aspect("equal")
    allc = pd.concat(runs.values())
    ylim = min(max(0.3, np.nanpercentile(np.abs(allc.e_y), 99.5) * 1.15), 5.0)
    dlim = min(max(5.0, np.nanpercentile(np.abs(allc.delta_deg), 99.5) * 1.15), 45.0)
    axe.set_xlim(0, path.length); axe.set_ylim(-ylim, ylim); axe.set_ylabel("cross-track e_y [m]")
    axd.set_xlim(0, path.length); axd.set_ylim(-dlim, dlim); axd.set_ylabel("steering δ [deg]")
    axd.set_xlabel("distance along route s [m]")
    axe.axhline(0, color=SUB, lw=0.8)
    art = {}
    for c, d in runs.items():
        col = COL.get(c, "white")
        art[c] = (axm.plot([], [], lw=4, color=col, alpha=0.85)[0],
                  axm.plot([], [], "o", ms=11, color=col, mec="white", mew=1.6, zorder=5)[0],
                  axe.plot([], [], lw=1.6, color=col, label=NAME.get(c, c))[0],
                  axd.plot([], [], lw=1.2, color=col)[0])
    axe.legend(loc="upper left", ncol=len(runs), facecolor=PANEL, edgecolor=GRID, labelcolor=TXT, fontsize=9)
    fig.suptitle(title, color=TXT, fontsize=14, weight="bold")
    clock = axm.text(0.03, 0.97, "", transform=axm.transAxes, color=TXT, va="top", fontsize=12,
                     family="monospace", bbox=dict(fc=BG, ec=GRID, boxstyle="round,pad=0.3"))

    def update(i):
        t = times[i]
        for c, d in runs.items():
            k = min(int(np.searchsorted(d.t.values, t)), len(d) - 1)
            trail, dot, el, dl = art[c]
            k0 = max(0, k - 60)
            trail.set_data(d.x.values[k0:k + 1], d.y.values[k0:k + 1])
            dot.set_data([d.x.values[k]], [d.y.values[k]])
            el.set_data(d.s.values[:k + 1], d.e_y.values[:k + 1])
            dl.set_data(d.s.values[:k + 1], d.delta_deg.values[:k + 1])
        clock.set_text(f"t = {t:5.1f} s")

    FuncAnimation(fig, update, frames=len(times)).save(fname, writer=PillowWriter(fps=fps), dpi=72,
                                                        savefig_kwargs={"facecolor": BG})
    plt.close(fig)
    print(f"  {fname}  ({os.path.getsize(fname) / 1e6:.1f} MB)")


def adaptation_gif(path, fname, frames=110, fps=12):
    scen = [("nominal", "#8b949e"), ("high_speed", "#4C9BE8"), ("noisy_delayed", "#B392F0"),
            ("steer_map_error", "#FF9F1C"), ("icy_push", "#2EC4B6"), ("combined", "#FF5E78")]
    runs = [(s, col, run(s, "adaptive_mrac_rob")) for s, col in scen]
    runs = [(s, col, d) for s, col, d in runs if d is not None]
    if not runs:
        print(f"  skip {fname}: no data"); return
    fig, ax = plt.subplots(figsize=(11, 4.8), facecolor=BG)
    fig.subplots_adjust(left=0.08, right=0.98, top=0.86, bottom=0.14)
    dark(ax)
    ax.axhline(4.3, color="white", lw=1, ls=":", alpha=0.8)
    ax.text(5, 4.32, "dry-road calibration 4.3 m", color=SUB, fontsize=9, va="bottom")
    ax.set_xlim(0, path.length)
    lo = min(d.theta.min() for _, _, d in runs); hi = max(d.theta.max() for _, _, d in runs)
    ax.set_ylim(lo - 0.15, hi + 0.15)
    ax.set_xlabel("distance along route s [m]"); ax.set_ylabel("adapted gain θ̂ = 1/b̂ [m]")
    lines = [(ax.plot([], [], lw=2.2, color=col, label=s)[0], ax.plot([], [], "o", ms=7, color=col,
              mec="white", mew=1)[0], d) for s, col, d in runs]
    ax.legend(loc="upper left", ncol=3, facecolor=PANEL, edgecolor=GRID, labelcolor=TXT, fontsize=9)
    fig.suptitle("MRAC-R learns how the car responds to steering, scenario by scenario",
                 color=TXT, fontsize=13, weight="bold")
    smax = path.length

    def update(i):
        s_now = smax * (i + 1) / frames
        for ln, dot, d in lines:
            k = max(1, int(np.searchsorted(d.s.values, s_now)))
            ln.set_data(d.s.values[:k], d.theta.values[:k])
            dot.set_data([d.s.values[k - 1]], [d.theta.values[k - 1]])

    FuncAnimation(fig, update, frames=frames).save(fname, writer=PillowWriter(fps=fps), dpi=72,
                                                    savefig_kwargs={"facecolor": BG})
    plt.close(fig)
    print(f"  {fname}  ({os.path.getsize(fname) / 1e6:.1f} MB)")


def main():
    os.makedirs(GIFS, exist_ok=True)
    path = ReferencePath.load(C.REF_FILE)
    print("static images"); banner(path); route_map(path); scoreboard()
    print("animations")
    tracking_gif(path, "combined", ["pure_pursuit", "stanley", "adaptive_mrac_rob_prev"],
                 os.path.join(GIFS, "tracking_combined.gif"),
                 "combined: low grip + load + steering-map error + noise + delay")
    tracking_gif(path, "steer_map_error", ["stanley", "adaptive_mrac_rob"],
                 os.path.join(GIFS, "tracking_steer_map.gif"),
                 "steer_map_error: unknown steering gain, Stanley vs MRAC-R")
    tracking_gif(path, "noisy_delayed", ["stanley", "adaptive_mrac_cal", "adaptive_mrac_rob"],
                 os.path.join(GIFS, "delay_robustness.gif"),
                 "150 ms delay + sensor noise: high-gain MRAC vs delay-robust MRAC-R")
    tracking_gif(path, "dry_real_push", ["pure_pursuit", "stanley", "adaptive_mrac_rob_prev"],
                 os.path.join(GIFS, "friction_limit.gif"),
                 "dry_real_push: at the friction limit, MRAC leaves the road")
    adaptation_gif(path, os.path.join(GIFS, "adaptation.gif"))
    print(f"done -> {OUT}/")


if __name__ == "__main__":
    main()
