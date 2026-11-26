"""Re-encode recorded videos to GitHub-friendly sizes and rebuild the side-by-side GIFs."""
import glob, os, subprocess
import imageio_ffmpeg

FF = imageio_ffmpeg.get_ffmpeg_exe()
run = lambda *a: subprocess.run([FF, "-y", "-loglevel", "error", *a], check=True)

for f in sorted(glob.glob("media/videos/*.mp4")):
    before = os.path.getsize(f) / 1e6
    tmp = f[:-4] + ".tmp.mp4"
    scale = "scale=1280:-2" if "side_by_side" in f else "scale=960:-2"
    run("-i", f, "-vf", scale, "-c:v", "libx264", "-preset", "slow", "-crf", "28",
        "-pix_fmt", "yuv420p", "-movflags", "+faststart", "-an", tmp)
    os.replace(tmp, f)
    print(f"{f}: {before:.1f} -> {os.path.getsize(f) / 1e6:.1f} MB")

for f in sorted(glob.glob("media/videos/*_side_by_side.mp4")):
    gif = os.path.join("media/gifs", os.path.basename(f)[:-4] + ".gif")
    vf = "fps=10,scale=760:-1:flags=lanczos"
    run("-t", "9", "-i", f, "-vf", vf + ",palettegen=max_colors=128:stats_mode=diff", "/tmp/palette.png")
    run("-t", "9", "-i", f, "-i", "/tmp/palette.png", "-lavfi",
        vf + " [x]; [x][1:v] paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle", gif)
    print(f"{gif}: {os.path.getsize(gif) / 1e6:.1f} MB")
