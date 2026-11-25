"""Skid-pad calibration: what lateral grip does each tire_friction scale really give?

Generates a flat 300 m x 120 m OpenDRIVE pad (no walls), then for each friction scale:
drive at constant speed, slowly ramp steering (right turn), and record lateral
acceleration a_y = v * r. The plateau of a_y / g is the achievable lateral grip mu_lat.

    python skidpad.py                      # scales 1.0 0.55 0.3
    python skidpad.py --scales 1.0 0.3 --speed 12
Afterwards run_experiments.py reloads Town03 automatically.
"""
import argparse
import math

import numpy as np
import carla

import config as C
from controllers import SpeedPI
from sim_utils import configure_physics, connect, enable_sync, measure_geometry, spawn_vehicle

G = 9.81
N_LANES, LANE_W, LENGTH = 15, 4.0, 300.0   # 15 lanes per side x 4 m = 120 m wide


def pad_xodr():
    lane = ('<lane id="{i}" type="driving" level="false"><link/>'
            f'<width sOffset="0" a="{LANE_W}" b="0" c="0" d="0"/>'
            '<roadMark sOffset="0" type="none" weight="standard" color="standard" width="0"/></lane>')
    left = "".join(lane.format(i=i) for i in range(N_LANES, 0, -1))
    right = "".join(lane.format(i=-i) for i in range(1, N_LANES + 1))
    return f"""<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
<header revMajor="1" revMinor="4" name="skidpad" version="1" north="0" south="0" east="0" west="0"/>
<road name="pad" length="{LENGTH}" id="1" junction="-1">
<link/>
<planView><geometry s="0" x="0" y="0" hdg="0" length="{LENGTH}"><line/></geometry></planView>
<elevationProfile><elevation s="0" a="0" b="0" c="0" d="0"/></elevationProfile>
<lateralProfile/>
<lanes><laneSection s="0">
<left>{left}</left>
<center><lane id="0" type="none" level="false"><roadMark sOffset="0" type="none" weight="standard" color="standard" width="0"/></lane></center>
<right>{right}</right>
</laneSection></lanes>
</road>
</OpenDRIVE>"""


def load_pad(client):
    params = carla.OpendriveGenerationParameters(
        vertex_distance=2.0, max_road_length=500.0, wall_height=0.0,
        additional_width=0.6, smooth_junctions=True, enable_mesh_visibility=True)
    print("[pad] generating flat OpenDRIVE skid-pad ...")
    return client.generate_opendrive_world(pad_xodr(), params)


def run_scale(world, scale, v_set, steer0=0.06, ramp=0.01, settle=8.0, t_max=70.0):
    # CARLA start (150, -50), heading +x, right turn -> circle lies on the +y side, inside the pad
    veh = spawn_vehicle(world, C.VEHICLE_BP, 150.0, 50.0, 0.0, 0.0)
    try:
        for _ in range(30):
            veh.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
            world.tick()
        configure_physics(veh, friction_scale=scale, flat_steering_curve=True)
        world.tick()
        geom = measure_geometry(veh)
        tire_mu = veh.get_physics_control().wheels[0].tire_friction
        pi = SpeedPI(kp=0.5, ki=0.15)
        t, reason = 0.0, "steer ramp finished"
        log = []
        while t < t_max:
            world.tick()
            t += C.DT
            tf, vel, w = veh.get_transform(), veh.get_velocity(), veh.get_angular_velocity()
            f = tf.get_forward_vector()
            v = vel.x * f.x + vel.y * f.y
            r = math.radians(w.z)
            steer = steer0 if t < settle else min(steer0 + ramp * (t - settle), 0.6)
            thr, brk = pi.step(v_set, v, C.DT)
            veh.apply_control(carla.VehicleControl(throttle=float(thr), brake=float(brk), steer=float(steer)))
            log.append((t, v, abs(v * r), steer))
            x, y = tf.location.x, tf.location.y
            if not (5.0 < x < LENGTH - 5.0 and abs(y) < N_LANES * LANE_W - 2.0):
                reason = "left the pad"
                break
    finally:
        veh.destroy()
        world.tick()

    a = np.array(log)
    ok = a[:, 0] > settle                                   # ignore the speed-up phase
    win = max(1, int(1.0 / C.DT))                           # 1 s moving average
    ay_f = np.convolve(a[:, 2], np.ones(win) / win, mode="same")
    i = int(np.argmax(np.where(ok, ay_f, 0.0)))
    return dict(scale=scale, tire_friction=tire_mu, mu_lat=ay_f[i] / G,
                v_at_peak=a[i, 1], steer_at_peak=a[i, 3],
                radius_at_peak=a[i, 1] ** 2 / max(ay_f[i], 1e-3), stop=reason)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scales", nargs="+", type=float, default=[1.0, 0.55, 0.3])
    ap.add_argument("--speed", type=float, default=10.0, help="test speed [m/s]")
    args = ap.parse_args()

    client = connect(C.HOST, C.PORT)
    world = load_pad(client)
    original = enable_sync(world, C.DT)
    rows = []
    try:
        world.tick()
        for sc in args.scales:
            print(f"\n=== friction scale {sc} ===")
            r = run_scale(world, sc, args.speed)
            rows.append(r)
            print(f"  tire_friction {r['tire_friction']:.2f} -> mu_lat ~ {r['mu_lat']:.2f} "
                  f"(v {r['v_at_peak']:.1f} m/s, R {r['radius_at_peak']:.1f} m, "
                  f"steer cmd {r['steer_at_peak']:.2f}, {r['stop']})")
    finally:
        world.apply_settings(original)
    print("\nscale  tire_friction  mu_lat  R_at_peak  stop")
    for r in rows:
        print(f"{r['scale']:5.2f}  {r['tire_friction']:13.2f}  {r['mu_lat']:6.2f}  {r['radius_at_peak']:9.1f}  {r['stop']}")
    print("\nIf mu_lat is the same for every scale, the test never reached the grip limit:\n"
          "rerun with a higher --speed (e.g. 14).")


if __name__ == "__main__":
    main()
