#!/usr/bin/env python3
"""
ORB-SLAM 轨迹 vs Vive 动捕真值：对齐 + 量尺度。

每个 episode：
  - CameraTrajectory.txt  (ORB-SLAM3 KITTI 输出, 在 -png 目录)   → 相机估计位姿
  - vive_poses.jsonl      (Vive 动捕, ~240Hz, 在 -mp4 目录)       → 真值位姿
  - times.txt             (相机帧 UNIX 时间戳, 在 -mp4 目录)

做法：
  1. 把 Vive 按时间插值到相机帧时刻 → 得到与相机帧一一对应的 Vive 真值位置。
  2. ORB 帧数 M 与相机帧数 N 若不等(ORB 丢帧)，按"归一化进度"重采样两者到等长，
     再做 Umeyama Sim3 对齐（只对位置）。s = 把 ORB 缩放到匹配 Vive 的因子。
       s≈1 → 尺度一致；s>1 → ORB 偏小(基线偏小)；s<1 → ORB 偏大。
  3. 另算一个不依赖对应的 bbox 对角线之比，交叉验证 s。

输出（写到 <png_session>_orbviz/）：
  - 每个 episode 一个交互式 HTML：Vive(红) + ORB刚体对齐(蓝,不缩放) + ORB缩放对齐(绿)
  - summary.csv：每行一个 episode 的 s / RMSE / 路径长 / bbox 比 / 帧数

用法：
  python orb_vs_vive.py <png_session目录>
  (自动把路径里的 -png 换成 -mp4 找 Vive/times)
"""
import sys, json, csv
from pathlib import Path
import numpy as np


def load_kitti_positions(p):
    """KITTI 轨迹每行 12 个数 = 3x4 [R|t] 行优先；取第 4/8/12 列即平移。"""
    a = np.atleast_2d(np.loadtxt(p))
    if a.shape[1] == 13:
        a = a[:, 1:]
    a = a.reshape(-1, 3, 4)
    return a[:, :3, 3]              # (M,3)


def load_vive(p):
    times, pos = [], []
    with open(p) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            d = json.loads(line)
            times.append(d["time"])
            pos.append(d["pos"])
    return np.array(times), np.array(pos)   # (K,) (K,3)


def load_times(p):
    return np.loadtxt(p).reshape(-1)        # (N,)


def umeyama(model, data, with_scale=True):
    """对齐 model→data：返回 (s,R,t)，使 data ≈ (s·R·model + t)。"""
    mu_m = model.mean(0); mu_d = data.mean(0)
    mz = model - mu_m; dz = data - mu_d
    n = len(model)
    H = mz.T @ dz / n
    U, S, Vt = np.linalg.svd(H)
    D = np.eye(3)
    if np.linalg.det(U @ Vt) < 0:
        D[2, 2] = -1
    R = Vt.T @ D @ U.T
    if with_scale:
        var_m = np.sum(np.var(model, axis=0))
        s = np.trace(np.diag(S) @ D) / var_m if var_m > 1e-12 else 1.0
    else:
        s = 1.0
    t = mu_d - s * R @ mu_m
    return s, R, t


def resample_to_length(P, L):
    """把 (n,3) 轨迹按弧长(或索引)重采样到 L 个点，做点对点对应用。"""
    n = len(P)
    if n == L:
        return P
    idx = np.linspace(0, n - 1, L)
    return np.column_stack([np.interp(idx, np.arange(n), P[:, d]) for d in range(3)])


def bbox_diag(P):
    return float(np.linalg.norm(P.max(0) - P.min(0)))


def path_len(P):
    return float(np.sum(np.linalg.norm(np.diff(P, axis=0), axis=1)))


def process_episode(orb_txt, vive_jsonl, times_txt, out_html, name):
    P_orb = load_kitti_positions(orb_txt)                 # (M,3)
    T_vive, P_vive_full = load_vive(vive_jsonl)           # (K,),(K,3)
    T_cam = load_times(times_txt)                         # (N,)

    # Vive 插值到相机帧时刻（越界 → nan）
    Pv_at_cam = np.column_stack([
        np.interp(T_cam, T_vive, P_vive_full[:, d],
                  left=np.nan, right=np.nan) for d in range(3)])  # (N,3)
    good = np.isfinite(Pv_at_cam).all(1)
    Pv = Pv_at_cam[good]
    # ORB 与相机帧数对应：若相等直接配对；不等则按等长重采样（假设都覆盖整段）
    if len(P_orb) == len(Pv):
        A, B = P_orb, Pv
    else:
        L = min(len(P_orb), len(Pv))
        A = resample_to_length(P_orb, L)
        B = resample_to_length(Pv, L)

    # 刚体对齐(不缩放) + Sim3(缩放)
    s1, R1, t1 = umeyama(A, B, with_scale=False)
    s, R, t = umeyama(A, B, with_scale=True)
    A_rigid = (R1 @ A.T).T + t1                 # 蓝色：只对齐姿态，保留 ORB 原尺度
    A_scaled = (s * (R @ A.T).T) + t            # 绿色：缩放后应与 Vive 重合
    rmse = float(np.sqrt(np.mean(np.sum((A_scaled - B) ** 2, axis=1))))

    # 不依赖对应的尺度交叉验证
    diag_orb = bbox_diag(P_orb); diag_vive = bbox_diag(Pv)
    bbox_ratio = diag_orb / diag_vive if diag_vive > 1e-9 else float("nan")

    # ---- 交互式 HTML ----
    if out_html is not None:
        try:
            import plotly.graph_objects as go
            fig = go.Figure()
            fig.add_trace(go.Scatter3d(x=B[:, 0], y=B[:, 1], z=B[:, 2], mode="lines+markers",
                                       line=dict(width=4, color="red"), marker=dict(size=3),
                                       name=f"Vive 真值 ({len(B)})"))
            fig.add_trace(go.Scatter3d(x=A_rigid[:, 0], y=A_rigid[:, 1], z=A_rigid[:, 2], mode="lines",
                                       line=dict(width=3, color="blue"), name="ORB 刚体对齐(不缩放)"))
            fig.add_trace(go.Scatter3d(x=A_scaled[:, 0], y=A_scaled[:, 1], z=A_scaled[:, 2], mode="lines",
                                       line=dict(width=3, color="green"), name=f"ORB 缩放对齐 (s={s:.3f})"))
            fig.update_layout(
                title=(f"{name} | M_ORB={len(P_orb)} N_cam={len(T_cam)} | "
                       f"Sim3 s={s:.3f}  RMSE={rmse*1000:.1f}mm | bbox比={bbox_ratio:.2f}"),
                scene=dict(xaxis_title="X", yaxis_title="Y", zaxis_title="Z", aspectmode="data"),
                width=980, height=760)
            fig.write_html(out_html)
        except Exception as e:
            print(f"  (HTML 跳过 {name}: {e})")

    return dict(episode=name, M_orb=len(P_orb), N_cam=len(T_cam), n_pairs=len(A),
                sim3_scale=s, rmse_mm=rmse * 1000, path_orb=path_len(P_orb),
                path_vive=path_len(Pv), bbox_ratio=bbox_ratio)


def main():
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(1)
    args = [a for a in sys.argv[1:] if a != "--no-html"]
    png_session = Path(args[0]).resolve()
    if len(args) > 1:
        mp4_session = Path(args[1]).resolve()              # 显式指定 -mp4（掩膜衍生目录用）
    else:
        mp4_session = Path(str(png_session).replace("-png", "-mp4"))
    if not mp4_session.exists():
        print(f"-mp4 目录不存在: {mp4_session}"); sys.exit(1)

    outdir = png_session.parent / (png_session.name.replace("-png", "-orbviz"))
    outdir.mkdir(exist_ok=True)
    no_html = "--no-html" in sys.argv

    eps = sorted(d for d in png_session.iterdir() if d.is_dir() and d.name.startswith("episode_"))
    print(f"{png_session.name}: {len(eps)} episodes  →  {outdir}\n")

    rows = []
    for ep in eps:
        orb_txt = ep / "CameraTrajectory.txt"
        vive = mp4_session / ep.name / "vive_poses.jsonl"
        times = mp4_session / ep.name / "times.txt"
        if not orb_txt.exists():
            print(f"  {ep.name}: 无 CameraTrajectory.txt (ORB 没跑/失败)，跳过")
            continue
        if not vive.exists() or not times.exists():
            print(f"  {ep.name}: 缺 Vive/times，跳过")
            continue
        html = None if no_html else outdir / f"{ep.name}.html"
        try:
            r = process_episode(orb_txt, vive, times, html, ep.name)
            rows.append(r)
            print(f"  {ep.name}: s={r['sim3_scale']:.3f}  RMSE={r['rmse_mm']:.1f}mm  "
                  f"bbox比={r['bbox_ratio']:.2f}  (M={r['M_orb']} N={r['N_cam']})")
        except Exception as e:
            print(f"  {ep.name}: 出错 {e}")

    if rows:
        csvp = outdir / "summary.csv"
        with open(csvp, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=rows[0].keys())
            w.writeheader(); w.writerows(rows)
        s = np.array([r["sim3_scale"] for r in rows])
        br = np.array([r["bbox_ratio"] for r in rows])
        print("\n================ 汇总 ================")
        print(f"episodes: {len(rows)}")
        print(f"Sim3 尺度因子 s  : mean={s.mean():.3f}  median={np.median(s):.3f}  std={s.std():.3f}")
        print(f"  (s>1 → ORB偏小/基线偏小;  s<1 → ORB偏大;  s≈1 → 一致)")
        print(f"bbox 对角线比     : mean={br.mean():.3f}  median={np.median(br):.3f}")
        print(f"CSV: {csvp}")


if __name__ == "__main__":
    main()
