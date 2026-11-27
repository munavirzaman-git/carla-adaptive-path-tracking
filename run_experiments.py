"""Run every (scenario x controller) on the same Town03 reference and log results.

  python run_experiments.py                                   # baselines + MRAC presets, all scenarios
  python run_experiments.py -c stanley adaptive_mrac_rob_prev -s combined --spectator
  python run_experiments.py -c adaptive_mrac --gains '{"k_e": 2.0}' --tag _test   # ad-hoc gains
  python run_experiments.py -c stanley -s noisy_delayed --noise-seed 1 --tag _s1   # another noise seed
  python run_experiments.py --new-route --seed 3                # regenerate the reference route

Controller names: pure_pursuit, stanley, adaptive_mrac, or a preset from controllers.PRESETS.
Each run writes results/<scenario>/<controller><tag>.csv and merges one row into results/summary.csv.
"""
import argparse
import json
import csv
import math
import os
from collections import deque

import numpy as np
import carla

import config as C
from controllers import ALL_NAMES, SpeedPI, make_controller
from reference_path import ReferencePath, build_reference
from scenarios import SCENARIOS, STUDY_SCENARIOS
from sim_utils import (configure_physics, connect, delta_to_carla_steer, draw_path, enable_sync,
                       follow_spectator, load_town, measure_geometry, read_state, spawn_vehicle)
from vehicle_state import with_pose, wrap

FIELDS = ["t", "s", "x", "y", "v", "v_ref", "e_y", "e_y_rear", "e_psi_deg",
          "delta_deg", "steer", "ay", "theta", "sigma", "solve_ms"]


def get_reference(world, args):
    if os.path.exists(C.REF_FILE) and not args.new_route:
        path = ReferencePath.load(C.REF_FILE)
        print(f"[ref] loaded {C.REF_FILE}: {path.length:.0f} m, {path.n} pts")
    else:
        path, pair = build_reference(world.get_map(), seed=args.seed,
                                     start_idx=args.start, goal_idx=args.goal)
        path.save(C.REF_FILE)
        print(f"[ref] built route {pair}: {path.length:.0f} m -> {C.REF_FILE}")
    return path


def run_one(world, path, ctrl_name, sc, rng, spectator=False, gains=None, hook=None):
    """One run. hook(vehicle, state, row, info) is called every control step (used by
    record_demo.py to attach a camera); returning True stops the run early."""
    world.set_weather(getattr(carla.WeatherParameters, sc.weather))
    vprof = path.speed_profile(sc.v_max_kmh / 3.6, sc.a_lat_max)
    veh = spawn_vehicle(world, C.VEHICLE_BP, path.x[0], path.y[0], path.z[0], path.psi[0])
    rows, status = [], "timeout"
    try:
        for _ in range(30):  # let it drop and settle
            veh.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
            world.tick()
        configure_physics(veh, sc.friction_scale, sc.mass_scale, sc.flat_steering_curve)
        world.tick()
        geom = measure_geometry(veh)
        dmax = math.radians(min(C.MAX_STEER_DEG, geom.max_wheel_steer_deg))
        ctrl = make_controller(ctrl_name, geom.wheelbase, dmax, **(gains or {}))  # nominal model only
        pi = SpeedPI(thr_max=sc.thr_max, brake_max=sc.brake_max)
        delay = deque([0.0] * sc.steer_delay_steps)
        idx_m = idx_t = 0
        t, yaw_sd = 0.0, math.radians(sc.yaw_noise_deg)

        while t < C.T_MAX:
            world.tick()
            t += C.DT
            st = read_state(veh, geom, t)

            # measurement seen by the controller (noise)
            meas = st
            if sc.pos_noise or yaw_sd:
                meas = with_pose(st, st.x + rng.normal(0, sc.pos_noise), st.y + rng.normal(0, sc.pos_noise),
                                 st.psi + rng.normal(0, yaw_sd), geom)
            idx_m, _ = path.project(meas.xr, meas.yr, idx_m)

            delta, info = ctrl.compute(meas, path, idx_m, C.DT)
            delay.append(delta)
            delta_applied = delay.popleft()   # pure transport delay (0 steps -> passthrough)
            thr, brk = pi.step(vprof[idx_m], meas.v, C.DT)
            steer = delta_to_carla_steer(delta_applied, geom)
            veh.apply_control(carla.VehicleControl(throttle=float(thr), steer=float(steer), brake=float(brk)))

            # cross-track at the vehicle centre; heading error at the rear axle, where heading and
            # velocity coincide in the kinematic model (no sideslip offset in corners)
            idx_t, e_y, _, _ = path.frame(st.x, st.y, idx_t)
            _, e_y_rear, psi_rear, _ = path.frame(st.xr, st.yr, idx_t)
            e_psi = wrap(st.psi - psi_rear)
            rows.append(dict(t=t, s=path.s[idx_t], x=st.x, y=st.y, v=st.v, v_ref=vprof[idx_m],
                             e_y=e_y, e_y_rear=e_y_rear, e_psi_deg=math.degrees(e_psi),
                             delta_deg=math.degrees(delta_applied), steer=steer, ay=st.ay,
                             theta=info.get("theta", np.nan), sigma=info.get("sigma", np.nan),
                             solve_ms=info.get("solve_ms", np.nan)))
            if spectator:
                follow_spectator(world, veh)
            if hook is not None and hook(veh, st, rows[-1], info):
                status = "stopped"
                break
            if idx_t >= path.n - 8:
                status = "completed"
                break
            if abs(e_y) > C.OFFTRACK_LIMIT:
                status = "off_track"
                break
    finally:
        veh.destroy()
        world.tick()
    return rows, status


def summarize(rows, status, path, ctrl_name, sc):
    a = {k: np.array([r[k] for r in rows]) for k in FIELDS}
    rate = np.diff(a["delta_deg"]) / C.DT if len(rows) > 1 else np.zeros(1)
    return dict(
        scenario=sc.name, controller=ctrl_name, status=status,
        completion_pct=round(100 * a["s"][-1] / path.length, 1),
        time_s=round(a["t"][-1], 2),
        rms_ey=np.sqrt(np.mean(a["e_y"] ** 2)), mean_abs_ey=np.mean(np.abs(a["e_y"])),
        p95_ey=np.percentile(np.abs(a["e_y"]), 95), max_ey=np.max(np.abs(a["e_y"])),
        rms_epsi_deg=np.sqrt(np.mean(a["e_psi_deg"] ** 2)), max_epsi_deg=np.max(np.abs(a["e_psi_deg"])),
        rms_steer_rate_dps=np.sqrt(np.mean(rate ** 2)),
        rms_speed_err=np.sqrt(np.mean((a["v"] - a["v_ref"]) ** 2)),
        max_abs_ay=np.max(np.abs(a["ay"])),
        final_theta=a["theta"][-1],
        mean_solve_ms=np.nanmean(a["solve_ms"]) if np.isfinite(a["solve_ms"]).any() else np.nan,
        max_solve_ms=np.nanmax(a["solve_ms"]) if np.isfinite(a["solve_ms"]).any() else np.nan,
    )


def write_csv(fname, rows, fields=None):
    os.makedirs(os.path.dirname(fname), exist_ok=True)
    with open(fname, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields or list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)


def merge_summary(m):
    import pandas as pd
    f = os.path.join(C.RESULTS_DIR, "summary.csv")
    df = pd.read_csv(f) if os.path.exists(f) else pd.DataFrame()
    if len(df):
        df = df[~((df.scenario == m["scenario"]) & (df.controller == m["controller"]))]
    pd.concat([df, pd.DataFrame([m])], ignore_index=True).to_csv(f, index=False)


DEFAULT_CONTROLLERS = ["pure_pursuit", "stanley", "adaptive_mrac_rob", "adaptive_mrac_rob_prev"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-c", "--controllers", nargs="+", default=DEFAULT_CONTROLLERS, choices=ALL_NAMES, metavar="NAME",
                    help="controllers or presets: " + ", ".join(ALL_NAMES))
    ap.add_argument("-s", "--scenarios", nargs="+", default=STUDY_SCENARIOS, choices=list(SCENARIOS), metavar="NAME",
                    help="scenarios: " + ", ".join(SCENARIOS))
    ap.add_argument("--spectator", action="store_true")
    ap.add_argument("--new-route", action="store_true")
    ap.add_argument("--seed", type=int, default=C.ROUTE_SEED)
    ap.add_argument("--start", type=int, default=None, help="spawn-point index (with --goal)")
    ap.add_argument("--goal", type=int, default=None)
    ap.add_argument("--gains", default="{}", help="JSON gain overrides, e.g. '{\"gamma\": 0}'")
    ap.add_argument("--tag", default="", help="suffix for result names, e.g. _s1")
    ap.add_argument("--noise-seed", type=int, default=0)
    args = ap.parse_args()

    client = connect(C.HOST, C.PORT)
    world = load_town(client, C.TOWN)
    original = enable_sync(world, C.DT)
    try:
        world.tick()
        path = get_reference(world, args)
        draw_path(world, path)
        for sc_name in args.scenarios:
            sc = SCENARIOS[sc_name]
            for ctrl_name in args.controllers:
                label = ctrl_name + args.tag
                print(f"\n=== {sc_name} | {label} ===")
                rng = np.random.default_rng(args.noise_seed)  # identical noise sequence for every controller
                rows, status = run_one(world, path, ctrl_name, sc, rng, args.spectator, json.loads(args.gains))
                write_csv(os.path.join(C.RESULTS_DIR, sc_name, f"{label}.csv"), rows, FIELDS)
                m = summarize(rows, status, path, label, sc)
                print(f"  {status}, {m['completion_pct']}% | RMS e_y {m['rms_ey']:.3f} m | "
                      f"max e_y {m['max_ey']:.3f} m | RMS e_psi {m['rms_epsi_deg']:.2f} deg")
                merge_summary(m)
    finally:
        world.apply_settings(original)
        print("\n[sim] restored original world settings")


if __name__ == "__main__":
    main()
