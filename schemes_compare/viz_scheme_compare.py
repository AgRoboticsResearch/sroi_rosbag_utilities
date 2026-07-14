#!/usr/bin/env python3
"""多方案对比视频：每 episode 横排 N 个 RGB 面板(每方案一个)，叠该方案的投影指尖路径
(绿→红 + 绿当前点 + 蓝终点)，动画化，按 episode 序号拼成 ALL_compare.mp4。
需先 transform 各方案轨迹(投影按 camera_link 系)。
用法: python3 viz_scheme_compare.py <schemes_dir> [--schemes raw maskhalf maskgripper]
"""
import sys, argparse, subprocess
from pathlib import Path
import numpy as np
import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "visualization"))
import visualize_traj_video as vtv   # 复用 load_poses/load_K/project_future/TIP_KIN/_green_red_gradient


def render_episode(ep_dir, schemes, fps, codec, out_mp4):
    color = sorted([*ep_dir.glob("color_*.png"), *ep_dir.glob("color_*.jpg")])
    K = vtv.load_K(ep_dir / "camera_info_color.json")
    poses_per = {}
    for sch in schemes:
        sd = ep_dir / sch
        t = sd / "CameraTrajectoryTransformed.txt"
        if not t.exists():
            t = sd / "CameraTrajectory.txt"
        p = vtv.load_poses(t)
        if p is None or K is None:
            return False
        poses_per[sch] = p
    n = min(len(color), *[len(poses_per[s]) for s in schemes])
    if n < 2:
        return False
    color = color[:n]
    for s in schemes:
        poses_per[s] = poses_per[s][:n]

    first = cv2.cvtColor(cv2.imread(str(color[0])), cv2.COLOR_BGR2RGB)
    H, W = first.shape[:2]
    nS = len(schemes)
    fig, axes = plt.subplots(1, nS, figsize=(4.4 * nS, 4.3), dpi=100)
    if nS == 1:
        axes = [axes]
    im_objs, fut_lcs, start_dots, stop_dots = [], [], [], []
    for ax, sch in zip(axes, schemes):
        im_objs.append(ax.imshow(first)); ax.set_xticks([]); ax.set_yticks([])
        ax.set_title(sch, fontsize=12, fontweight="bold")
        lc = LineCollection([], linewidths=2.0, capstyle="round", zorder=4)
        ax.add_collection(lc); fut_lcs.append(lc)
        start_dots.append(ax.scatter([], [], c=["#00FF00"], s=45, zorder=6, edgecolors="black", linewidths=0.4))
        stop_dots.append(ax.scatter([], [], c=["#0000FF"], s=45, zorder=6, edgecolors="black", linewidths=0.4))
        ax.set_xlim(0, W); ax.set_ylim(H, 0)
    fig.subplots_adjust(left=0.01, right=0.99, top=0.86, bottom=0.02, wspace=0.05)
    fig.suptitle(f"{ep_dir.parent.name}/{ep_dir.name}", fontsize=11, y=0.97)

    cw, ch = fig.canvas.get_width_height()
    cmd = ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
           "-s", f"{cw}x{ch}", "-r", str(fps), "-i", "-",
           "-c:v", codec, "-pix_fmt", "yuv420p", "-r", str(fps), str(out_mp4)]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    try:
        for t in range(n):
            img = cv2.cvtColor(cv2.imread(str(color[t])), cv2.COLOR_BGR2RGB)
            for i, sch in enumerate(schemes):
                im_objs[i].set_data(img)
                px, py = vtv.project_future(poses_per[sch], t, K)
                pts = np.column_stack([px, py])
                segs = np.stack([pts[:-1], pts[1:]], axis=1) if len(pts) >= 2 else np.empty((0, 2, 2))
                fut_lcs[i].set_segments(segs); fut_lcs[i].set_colors(vtv._green_red_gradient(len(segs)))
                start_dots[i].set_offsets([pts[0]] if len(pts) else [[np.nan, np.nan]])
                spt = pts[-1] if len(pts) and np.isfinite(pts[-1]).all() else np.array([np.nan, np.nan])
                stop_dots[i].set_offsets([spt])
            fig.canvas.draw()
            proc.stdin.write(np.asarray(fig.canvas.buffer_rgba())[..., :3].tobytes())
    except Exception:
        proc.kill(); plt.close(fig); raise
    out, err = proc.communicate(); plt.close(fig)
    return proc.returncode == 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("schemes_dir")
    ap.add_argument("--schemes", nargs="+", default=["raw", "maskhalf", "maskgripper"])
    ap.add_argument("--fps", type=int, default=20)
    ap.add_argument("--codec", default="libx264")
    args = ap.parse_args()
    sd = Path(args.schemes_dir)
    outdir = sd / "_compare"   # 放进 schemes 里，整洁
    outdir.mkdir(exist_ok=True)
    eps = sorted(d for d in sd.iterdir() if d.is_dir() and d.name.startswith("episode_"))
    print(f"{sd.name}: {len(eps)} episodes × {args.schemes} → {outdir}", flush=True)
    ok = 0
    for i, ep in enumerate(eps, 1):
        mp4 = outdir / f"{ep.name}.mp4"
        if mp4.exists() and mp4.stat().st_size > 10000:
            print(f"[{i}/{len(eps)}] {ep.name} skip", flush=True); continue
        if render_episode(ep, args.schemes, args.fps, args.codec, mp4):
            ok += 1; print(f"[{i}/{len(eps)}] {ep.name} ok", flush=True)
        else:
            print(f"[{i}/{len(eps)}] {ep.name} skip(no data)", flush=True)
    print("concatenating...", flush=True)
    present = [e for e in eps if (outdir / f"{e.name}.mp4").exists()]
    lf = outdir / "_concat.txt"
    with open(lf, "w") as f:
        for e in present:
            f.write(f"file '{(outdir / f'{e.name}.mp4').resolve()}'\n")
    final = outdir / "ALL_compare.mp4"
    subprocess.run(["ffmpeg", "-y", "-f", "concat", "-safe", "0", "-i", str(lf),
                    "-c", "copy", str(final)], capture_output=True)
    print(f"DONE: {final} ({ok} episodes)", flush=True)


if __name__ == "__main__":
    main()
