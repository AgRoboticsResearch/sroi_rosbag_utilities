#!/usr/bin/env python3
"""对 schemes 结构里的某个方案子文件夹批量跑 ORB-SLAM3。
读 <schemes>/episode_NNN/<scheme>/ 的 left_/right_(+symlink 的 yaml/times)，跑 stereo_kitti，
轨迹存到该方案子文件夹。已有 CameraTrajectory.txt 则跳过。
用法: python3 run_orb_scheme.py <schemes_dir> <scheme_name>   例: ... 144418-schemes maskgripper
"""
import sys, subprocess
from pathlib import Path

ORB = Path.home() / "code" / "ORB_SLAM3"
STEREO = ORB / "Examples" / "Stereo" / "stereo_kitti"
VOCAB = ORB / "Vocabulary" / "ORBvoc.txt"
schemes = Path(sys.argv[1]); scheme = sys.argv[2]

eps = sorted(d for d in schemes.iterdir() if d.is_dir() and d.name.startswith("episode_"))
ok = skip = fail = 0
for i, ep in enumerate(eps, 1):
    sd = ep / scheme
    if not sd.exists() or not (sd / "left_000000.png").exists():
        print(f"[{i}/{len(eps)}] {ep.name}/{scheme}: 无图像，跳过", flush=True); skip += 1; continue
    traj = sd / "CameraTrajectory.txt"
    if traj.exists():
        print(f"[{i}/{len(eps)}] {ep.name}/{scheme}: 已有轨迹，跳过", flush=True); skip += 1; continue
    cfg = sd / "orb_slam_realsense_d405.yaml"
    r = subprocess.run([str(STEREO), str(VOCAB), str(cfg), str(sd), "false"],
                       capture_output=True, text=True)
    if traj.exists():
        ok += 1; print(f"[{i}/{len(eps)}] {ep.name}/{scheme}: done", flush=True)
    else:
        fail += 1; print(f"[{i}/{len(eps)}] {ep.name}/{scheme}: FAIL {r.stderr[-120:]}", flush=True)
print(f"=== {scheme}: ok={ok} skip={skip} fail={fail} / {len(eps)} ===")
