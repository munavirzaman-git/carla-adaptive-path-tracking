"""Record CARLA demo videos with a live HUD.

For each controller it re-runs the scenario with a camera attached to the car (chase or
top-down view) and writes one MP4. It then stitches the runs side by side into one MP4 and
a README-sized GIF. Runs here are NOT added to results/summary.csv.

    python record_demo.py -s combined -c stanley adaptive_mrac_rob_prev
    python record_demo.py -s dry_real_push -c stanley adaptive_mrac_rob_prev --view top --t-end 25
    python record_demo.py -s noisy_delayed -c adaptive_mrac_cal adaptive_mrac_rob --t-end 40
"""
import argparse
import math
import os
import queue

import carla
import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

import config as C
from controllers import ALL_NAMES
from reference_path import ReferencePath
from run_experiments import run_one
from scenarios import SCENARIOS
from sim_utils import connect, draw_path, enable_sync, load_town

NAME = {"pure_pursuit": "Pure Pursuit", "stanley": "Stanley", "adaptive_mrac": "MRAC (nominal)",
        "adaptive_mrac_cal": "High-gain MRAC", "adaptive_mrac_cal_fixed": "High-gain MRAC, fixed",
        "adaptive_mrac_rob": "MRAC-R", "adaptive_mrac_rob_fixed": "MRAC-R, fixed gain",
        "adaptive_mrac_rob_prev": "MRAC-RP", "adaptive_mrac_rob_prev_fixed": "MRAC-RP, fixed gain"}
COLOR = {"pure_pursuit": (76, 155, 232), "stanley": (255, 159, 28), "adaptive_mrac_cal": (255, 94, 120),
         "adaptive_mrac_rob": (46, 196, 182), "adaptive_mrac_rob_prev": (123, 211, 137)}
VIEWS = {"chase": carla.Transform(carla.Location(x=-7.5, z=3.0), carla.Rotation(pitch=-12.0)),
         "top": carla.Transform(carla.Location(z=32.0), carla.Rotation(pitch=-90.0))}


def font(size):
    for f in ("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
              "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
        if os.path.exists(f):
            return ImageFont.truetype(f, size)
    try:
        return ImageFont.load_default(size=size)
    except TypeError:
        return ImageFont.load_default()


def err_color(e):
    """green (0 m) -> yellow (0.3 m) -> red (>= 0.8 m)"""
    a = min(abs(e) / 0.8, 1.0)
    return (int(80 + 175 * min(1.0, 2 * a)), int(220 - 170 * max(0.0, 2 * a - 1)), 90)


class Recorder:
    """Callable hook for run_one: attaches a camera, draws a HUD, writes an MP4."""

    def __init__(self, world, mp4, ctrl, scenario, view, w, h, t_start, t_end, fps):
        self.world, self.ctrl, self.scenario, self.view = world, ctrl, scenario, view
        self.w, self.h, self.t_start, self.t_end = w, h, t_start, t_end
        self.q, self.cam, self.n = queue.Queue(), None, 0
        self.writer = imageio.get_writer(mp4, fps=fps, codec="libx264", quality=None, output_params=["-crf", "28", "-preset", "slow"],
                                         macro_block_size=1, ffmpeg_log_level="error")
        self.big, self.mid, self.small = font(30), font(21), font(17)
        self.color = COLOR.get(ctrl, (230, 237, 243))

    def __call__(self, veh, st, row, info):
        if self.cam is None:                          # first step: spawn camera on this vehicle
            bp = self.world.get_blueprint_library().find("sensor.camera.rgb")
            bp.set_attribute("image_size_x", str(self.w)); bp.set_attribute("image_size_y", str(self.h))
            bp.set_attribute("fov", "90")
            self.cam = self.world.spawn_actor(bp, VIEWS[self.view], attach_to=veh)
            self.cam.listen(self.q.put)
            return False
        try:
            img = self.q.get(timeout=5.0)
        except queue.Empty:
            return False
        if row["t"] < self.t_start:
            return False
        if self.t_end and row["t"] > self.t_end:
            return True
        a = np.frombuffer(img.raw_data, dtype=np.uint8).reshape(img.height, img.width, 4)
        frame = self.hud(Image.fromarray(np.ascontiguousarray(a[:, :, 2::-1])), row)
        self.writer.append_data(np.asarray(frame))
        self.n += 1
        return False

    def hud(self, im, row):
        im = im.convert("RGBA")
        ov = Image.new("RGBA", im.size, (0, 0, 0, 0))
        d = ImageDraw.Draw(ov)
        d.rounded_rectangle([16, 16, 400, 236], radius=14, fill=(13, 17, 23, 200))
        d.rectangle([16, 16, 24, 236], fill=self.color + (255,))
        d.text((38, 26), NAME.get(self.ctrl, self.ctrl), font=self.big, fill=self.color + (255,))
        d.text((38, 66), f"scenario: {self.scenario}", font=self.small, fill=(139, 148, 158, 255))
        e = row["e_y"]
        d.text((38, 96), f"speed  {row['v'] * 3.6:5.1f} km/h", font=self.mid, fill=(230, 237, 243, 255))
        d.text((38, 126), f"e_y    {e:+5.2f} m", font=self.mid, fill=err_color(e) + (255,))
        d.text((38, 156), f"steer  {row['delta_deg']:+5.1f} deg", font=self.mid, fill=(230, 237, 243, 255))
        th = row.get("theta", float("nan"))
        if th == th:   # not NaN
            d.text((38, 186), f"theta  {th:5.2f} m", font=self.mid, fill=(123, 211, 137, 255))
        # lateral-error bar at the bottom: +-1 m full scale
        W, H = im.size
        cx, y0, half = W // 2, H - 46, 260
        d.rounded_rectangle([cx - half - 10, y0 - 22, cx + half + 10, y0 + 26], radius=10, fill=(13, 17, 23, 190))
        d.line([cx - half, y0, cx + half, y0], fill=(80, 88, 99, 255), width=6)
        d.line([cx, y0 - 12, cx, y0 + 12], fill=(230, 237, 243, 255), width=2)
        x = cx - int(max(-1.0, min(1.0, e)) * half)       # e_y > 0 = left of path -> left on screen
        d.ellipse([x - 11, y0 - 11, x + 11, y0 + 11], fill=err_color(e) + (255,), outline=(255, 255, 255, 255), width=2)
        d.text((cx - half, y0 - 20), "left 1 m", font=self.small, fill=(139, 148, 158, 255))
        d.text((cx + half - 84, y0 - 20), "right 1 m", font=self.small, fill=(139, 148, 158, 255))
        d.text((W - 150, 24), f"t = {row['t']:5.1f} s", font=self.mid, fill=(230, 237, 243, 255))
        return Image.alpha_composite(im, ov).convert("RGB")

    def finish(self):
        if self.cam is not None:
            self.cam.stop(); self.cam.destroy(); self.cam = None
        self.writer.close()


def side_by_side(mp4s, out_mp4, out_gif, fps, gif_seconds, gif_width):
    """Stream the per-controller MP4s frame by frame (no full video in memory); a run that
    ended early keeps showing its last frame."""
    readers = [imageio.get_reader(f) for f in mp4s]
    iters = [iter(r) for r in readers]
    last = [None] * len(readers)
    pw = 1280 // len(readers)
    gif_frames = []
    w = imageio.get_writer(out_mp4, fps=fps, codec="libx264", quality=None, output_params=["-crf", "28", "-preset", "slow"], macro_block_size=1, ffmpeg_log_level="error")
    i = 0
    while True:
        alive = False
        for k, it in enumerate(iters):
            f = next(it, None)
            if f is not None:
                last[k], alive = f, True
        if not alive:
            break
        panels = []
        for f in last:
            im = Image.fromarray(f)
            panels.append(im.resize((pw, int(im.height * pw / im.width))))
        canvas = Image.new("RGB", (pw * len(panels), panels[0].height), (13, 17, 23))
        for k, p in enumerate(panels):
            canvas.paste(p, (k * pw, 0))
        w.append_data(np.asarray(canvas))
        if i < gif_seconds * fps and i % 2 == 0:          # 10 fps GIF of the first seconds
            g = canvas.resize((gif_width, int(canvas.height * gif_width / canvas.width)))
            gif_frames.append(g.quantize(colors=160, method=Image.Quantize.MEDIANCUT))
        i += 1
    w.close()
    for r in readers:
        r.close()
    if gif_frames:
        gif_frames[0].save(out_gif, save_all=True, append_images=gif_frames[1:], duration=100, loop=0, optimize=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-s", "--scenario", default="combined", choices=list(SCENARIOS))
    ap.add_argument("-c", "--controllers", nargs="+", default=["stanley", "adaptive_mrac_rob_prev"],
                    choices=ALL_NAMES, metavar="NAME")
    ap.add_argument("--view", default="chase", choices=list(VIEWS))
    ap.add_argument("--t-start", type=float, default=0.0, help="start recording at this run time [s]")
    ap.add_argument("--t-end", type=float, default=40.0, help="stop the run at this time [s] (0 = full lap)")
    ap.add_argument("--width", type=int, default=960)
    ap.add_argument("--height", type=int, default=540)
    ap.add_argument("--gif-seconds", type=float, default=8.0)
    ap.add_argument("--gif-width", type=int, default=720)
    ap.add_argument("--out", default="media")
    args = ap.parse_args()

    vdir, gdir = os.path.join(args.out, "videos"), os.path.join(args.out, "gifs")
    os.makedirs(vdir, exist_ok=True); os.makedirs(gdir, exist_ok=True)
    fps = int(round(1.0 / C.DT))
    sc = SCENARIOS[args.scenario]
    tag = f"{args.scenario}_{args.view}"

    client = connect(C.HOST, C.PORT)
    world = load_town(client, C.TOWN)
    original = enable_sync(world, C.DT)
    mp4s = []
    try:
        world.tick()
        path = ReferencePath.load(C.REF_FILE)
        draw_path(world, path, every=4)
        for ctrl in args.controllers:
            mp4 = os.path.join(vdir, f"{tag}_{ctrl}.mp4")
            print(f"=== recording {args.scenario} | {ctrl} ({args.view}) -> {mp4}")
            rec = Recorder(world, mp4, ctrl, args.scenario, args.view, args.width, args.height,
                           args.t_start, args.t_end, fps)
            try:
                _, status = run_one(world, path, ctrl, sc, np.random.default_rng(0), hook=rec)
            finally:
                rec.finish()
            print(f"    {status}, {rec.n} frames ({rec.n / fps:.1f} s)")
            mp4s.append(mp4)
    finally:
        world.apply_settings(original)
    if len(mp4s) > 1:
        out_mp4 = os.path.join(vdir, f"{tag}_side_by_side.mp4")
        out_gif = os.path.join(gdir, f"{tag}_side_by_side.gif")
        side_by_side(mp4s, out_mp4, out_gif, fps, args.gif_seconds, args.gif_width)
        for f in (out_mp4, out_gif):
            print(f"  {f}  ({os.path.getsize(f) / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
