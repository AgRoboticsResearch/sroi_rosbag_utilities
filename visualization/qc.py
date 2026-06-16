#!/usr/bin/env python3
"""
QC tool for ORB-SLAM + gripper estimation outputs.

Scans a session tree (default: <root>/*-png/episode_*/) and produces:
  - <out>.csv : per-episode metrics (path length, gripper std, category, ...)
  - <out>.pdf : one summary page + one page per session with side-by-side
                trajectory and gripper plots, color-coded by category.

Designed to be re-runnable as data updates. Idempotent — overwrites outputs.

CLI usage:
    # Default: scan all *-png sessions, write qc.csv + qc.pdf in cwd
    python visualization/qc.py /path/to/20260611

    # Custom output prefix (writes <prefix>.csv and <prefix>.pdf)
    python visualization/qc.py /path/to/20260611 -o /tmp/qc

    # Only one kind of output
    python visualization/qc.py /path/to/20260611 --csv-only
    python visualization/qc.py /path/to/20260611 --pdf-only

    # Glob for sessions with a different pattern
    python visualization/qc.py /path/to/data -p '*-jpeg'

API usage:
    from visualization.qc import scan_sessions, write_csv, render_pdf
    sessions = scan_sessions(Path("/path/to/data"))
    write_csv(sessions, Path("qc.csv"))
    render_pdf(sessions, Path("qc.pdf"))

Categories (per episode):
    ok                 - non-static motion + valid gripper signal
    static             - < 1 cm trajectory path
    flat_gripper       - motion present, gripper std < 0.05 (no actuation)
    no_gripper         - gripper_distances.txt missing or empty
    no_traj            - trajectory file missing/empty
    all_nan_gripper    - all gripper values are NaN
"""

from __future__ import annotations

import argparse
import csv
import sys
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np


# ============================================================================
# Data classes
# ============================================================================

CATEGORY_COLORS = {
    "ok": "#2e7d32",           # green
    "static": "#c62828",       # red
    "flat_gripper": "#ef6c00", # orange
    "no_gripper": "#757575",   # gray
    "no_traj": "#757575",      # gray
    "all_nan_gripper": "#c62828",
}

# Distinct colors for the two plots WITHIN an episode panel
TRAJ_COLOR = "#1565c0"   # dark blue
GRIP_COLOR = "#e65100"   # dark orange

# Background tint for each episode panel (grouping cue)
PANEL_BG = "#f5f7fa"     # very light blue-gray

# Categories considered "bad" — episodes with these get sorted to the top of pages
PROBLEM_CATEGORIES = {"no_traj", "all_nan_gripper", "no_gripper", "static", "flat_gripper"}
CATEGORY_SORT_ORDER = {"no_traj": 0, "all_nan_gripper": 1, "no_gripper": 2,
                        "static": 3, "flat_gripper": 4, "ok": 5}

# Thresholds — exposed as module-level so callers can tune
STATIC_PATH_M = 0.01           # < 1 cm = static
FLAT_GRIPPER_STD = 0.05        # gripper std below this = no actuation


@dataclass
class EpisodeQC:
    session: str
    episode: str
    path: str
    category: str
    n_traj_frames: int = 0
    path_length_cm: float = 0.0
    xy_span_cm: float = 0.0
    z_span_cm: float = 0.0
    n_grip_frames: int = 0
    n_nan_gripper: int = 0
    gripper_std: float = 0.0
    gripper_range: float = 0.0
    # Raw arrays (only kept if keep_arrays=True on scan)
    pos: np.ndarray | None = field(default=None, repr=False)
    grip: np.ndarray | None = field(default=None, repr=False)


# ============================================================================
# Loading + classification
# ============================================================================

def _load_trajectory(traj_path: Path) -> np.ndarray | None:
    """Load CameraTrajectoryTransformed.txt -> (N, 3) xyz positions, or None."""
    if not traj_path.exists():
        return None
    try:
        raw = np.loadtxt(traj_path, comments="#")
    except Exception:
        return None
    if raw.ndim == 1:
        raw = raw.reshape(1, -1)
    if raw.shape[1] < 12:
        return None
    poses = raw[:, :12].reshape(-1, 3, 4)
    return poses[:, :3, 3]


def _load_gripper(grip_path: Path) -> np.ndarray | None:
    """Load gripper_distances.txt -> (N,) array, or None."""
    if not grip_path.exists():
        return None
    try:
        v = np.loadtxt(grip_path)
    except Exception:
        return None
    if v.size == 0:
        return None
    return np.atleast_1d(v)


def _path_length(pos: np.ndarray | None) -> float:
    if pos is None or len(pos) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(pos, axis=0), axis=1)))


def classify(pos: np.ndarray | None, grip: np.ndarray | None) -> str:
    if pos is None or len(pos) < 2:
        return "no_traj"
    if _path_length(pos) < STATIC_PATH_M:
        return "static"
    if grip is None:
        return "no_gripper"
    finite = grip[np.isfinite(grip)]
    if len(finite) == 0:
        return "all_nan_gripper"
    if finite.std() < FLAT_GRIPPER_STD:
        return "flat_gripper"
    return "ok"


def scan_episode(episode_dir: Path, keep_arrays: bool = True) -> EpisodeQC:
    """Compute QC metrics for a single episode directory."""
    pos = _load_trajectory(episode_dir / "CameraTrajectoryTransformed.txt")
    grip = _load_gripper(episode_dir / "gripper_distances.txt")
    cat = classify(pos, grip)

    e = EpisodeQC(
        session=episode_dir.parent.name,
        episode=episode_dir.name,
        path=str(episode_dir),
        category=cat,
        n_traj_frames=int(len(pos)) if pos is not None else 0,
        path_length_cm=_path_length(pos) * 100,
        xy_span_cm=float(np.ptp(pos[:, :2], axis=0).max()) * 100 if pos is not None and len(pos) else 0.0,
        z_span_cm=float(np.ptp(pos[:, 2])) * 100 if pos is not None and len(pos) else 0.0,
        n_grip_frames=int(len(grip)) if grip is not None else 0,
        n_nan_gripper=int((~np.isfinite(grip)).sum()) if grip is not None else 0,
    )
    if grip is not None and len(grip):
        finite = grip[np.isfinite(grip)]
        if len(finite):
            e.gripper_std = float(finite.std())
            e.gripper_range = float(finite.max() - finite.min())
    if keep_arrays:
        e.pos = pos
        e.grip = grip
    return e


def scan_sessions(root: Path, pattern: str = "*-png", keep_arrays: bool = True) -> dict[str, list[EpisodeQC]]:
    """Scan a session tree, returning {session_name: [EpisodeQC, ...]}."""
    sessions = sorted([d for d in root.glob(pattern) if d.is_dir()])
    out: dict[str, list[EpisodeQC]] = {}
    for s in sessions:
        eps = []
        for ep_dir in sorted(s.glob("episode_*")):
            if ep_dir.is_dir():
                eps.append(scan_episode(ep_dir, keep_arrays=keep_arrays))
        if eps:
            out[s.name] = eps
    return out


# ============================================================================
# CSV output
# ============================================================================

CSV_FIELDS = ["session", "episode", "category",
              "path_length_cm", "xy_span_cm", "z_span_cm", "n_traj_frames",
              "n_nan_gripper", "n_grip_frames", "gripper_std", "gripper_range",
              "path"]


def write_csv(session_data: dict[str, list[EpisodeQC]], output: Path) -> None:
    """Write per-episode metrics to CSV."""
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        w.writeheader()
        for session_name in sorted(session_data):
            for e in session_data[session_name]:
                w.writerow({k: getattr(e, k) for k in CSV_FIELDS})


# ============================================================================
# PDF output
# ============================================================================

def _plot_episode(axes, e: EpisodeQC):
    ax_traj, ax_grip = axes
    cat_color = CATEGORY_COLORS.get(e.category, "#999")

    # Match panel background so the pair reads as one group
    ax_traj.set_facecolor(PANEL_BG)
    ax_grip.set_facecolor(PANEL_BG)

    # Trajectory — always blue inside the panel
    if e.pos is not None and len(e.pos) >= 2:
        xy = e.pos[:, :2] - e.pos[0, :2]
        ax_traj.plot(xy[:, 0] * 100, xy[:, 1] * 100, "-", linewidth=1.2, color=TRAJ_COLOR)
        ax_traj.scatter([xy[0, 0] * 100], [xy[0, 1] * 100], c="green", s=25, zorder=3)
        ax_traj.scatter([xy[-1, 0] * 100], [xy[-1, 1] * 100], c="red", s=25, zorder=3, marker="x")
        ax_traj.set_aspect("equal", adjustable="datalim")
    else:
        ax_traj.text(0.5, 0.5, "no traj", ha="center", va="center",
                     transform=ax_traj.transAxes, fontsize=9)
    ax_traj.set_title(f"{e.episode}  ({e.path_length_cm:.1f} cm, {e.category})",
                      fontsize=9, color=cat_color)
    ax_traj.tick_params(labelsize=7)
    ax_traj.grid(True, alpha=0.3)
    ax_traj.set_xlabel("x (cm)", fontsize=8)
    ax_traj.set_ylabel("y (cm)", fontsize=8)

    # Gripper — always orange inside the panel
    if e.grip is not None and len(e.grip) > 0:
        t = np.arange(len(e.grip))
        mask = np.isfinite(e.grip)
        ax_grip.plot(t[mask], e.grip[mask], "-", linewidth=1.0, color=GRIP_COLOR)
        if (~mask).any():
            ax_grip.plot(t[~mask], np.zeros((~mask).sum()), "rx", markersize=4)
        ax_grip.set_ylim(-0.05, 1.05)
    else:
        ax_grip.text(0.5, 0.5, "no grip", ha="center", va="center",
                     transform=ax_grip.transAxes, fontsize=9)
    ax_grip.tick_params(labelsize=7)
    ax_grip.grid(True, alpha=0.3)
    ax_grip.set_xlabel("frame", fontsize=8)
    ax_grip.set_ylabel("gripper", fontsize=8)


def _render_summary_page(pdf: PdfPages, session_data: dict[str, list[EpisodeQC]]):
    sessions = sorted(session_data.keys())
    n = len(sessions)
    fig, ax = plt.subplots(figsize=(11, 0.4 * n + 1.5))
    ax.axis("off")

    rows = []
    for s in sessions:
        eps = session_data[s]
        cats = [e.category for e in eps]
        paths = [e.path_length_cm for e in eps]
        n_ok = sum(1 for c in cats if c == "ok")
        n_static = sum(1 for c in cats if c == "static")
        n_other = len(cats) - n_ok - n_static
        rows.append([
            s,
            f"{len(eps)}",
            f"{n_ok}",
            f"{n_static}",
            f"{n_other}",
            f"{np.mean(paths):.1f}" if paths else "0",
            f"{max(paths):.1f}" if paths else "0",
        ])

    cols = ["session", "eps", "ok", "static", "other", "avg_path_cm", "max_path_cm"]
    table = ax.table(cellText=rows, colLabels=cols, loc="center", cellLoc="left")
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1, 1.4)
    for j in range(len(cols)):
        table[(0, j)].set_facecolor("#e0e0e0")
        table[(0, j)].set_text_props(weight="bold")
    for i, s in enumerate(sessions, start=1):
        cats = [e.category for e in session_data[s]]
        if all(c == "ok" for c in cats):
            color = "#c8e6c9"
        elif any(c in ("static", "no_traj", "all_nan_gripper") for c in cats):
            color = "#ffcdd2"
        else:
            color = "#ffe0b2"
        table[(i, 0)].set_facecolor(color)

    total_eps = sum(len(v) for v in session_data.values())
    ax.set_title(f"QC Summary — {n} sessions, {total_eps} episodes",
                 fontsize=13, pad=20)
    pdf.savefig(fig, bbox_inches="tight")
    plt.close(fig)


def _render_session_page(pdf: PdfPages, session_name: str, episodes: list[EpisodeQC]):
    from matplotlib import gridspec

    episodes = sorted(episodes, key=lambda e: (CATEGORY_SORT_ORDER.get(e.category, 9),
                                                -e.path_length_cm))
    n = len(episodes)
    ncols = 3
    nrows = int(np.ceil(n / ncols))
    fig = plt.figure(figsize=(11, 2.4 * nrows))

    # Outer grid: one cell per episode. Generous wspace/hspace to make panels
    # read as distinct groups.
    outer = gridspec.GridSpec(
        nrows, ncols, figure=fig,
        hspace=0.55, wspace=0.30,
        left=0.04, right=0.98, top=0.95, bottom=0.04,
    )

    cat_counts = defaultdict(int)
    for e in episodes:
        cat_counts[e.category] += 1
    cat_summary = "  ".join(f"{c}:{n}" for c, n in sorted(cat_counts.items()))
    fig.suptitle(f"{session_name}  —  {n} episodes  —  {cat_summary}",
                 fontsize=12, y=0.985)

    for i, e in enumerate(episodes):
        row, col = divmod(i, ncols)
        # Inner 1x2 grid: traj | gripper, tighter spacing inside the group
        inner = gridspec.GridSpecFromSubplotSpec(
            1, 2, subplot_spec=outer[row, col], wspace=0.30,
        )
        ax_traj = fig.add_subplot(inner[0, 0])
        ax_grip = fig.add_subplot(inner[0, 1])
        _plot_episode((ax_traj, ax_grip), e)

    # Hide unused cells
    for i in range(n, nrows * ncols):
        row, col = divmod(i, ncols)
        ax = fig.add_subplot(outer[row, col])
        ax.axis("off")

    pdf.savefig(fig, bbox_inches="tight")
    plt.close(fig)


def render_pdf(session_data: dict[str, list[EpisodeQC]], output: Path) -> None:
    """Render multi-page PDF: summary + one page per session."""
    output.parent.mkdir(parents=True, exist_ok=True)
    with PdfPages(output) as pdf:
        _render_summary_page(pdf, session_data)
        # Sessions with most problems first
        def issue_score(item):
            eps = item[1]
            return sum(1 for e in eps if e.category in PROBLEM_CATEGORIES)
        for name, eps in sorted(session_data.items(), key=lambda kv: (-issue_score(kv), kv[0])):
            _render_session_page(pdf, name, eps)


# ============================================================================
# CLI
# ============================================================================

def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("root", type=Path, help="Directory containing session folders")
    p.add_argument("-o", "--output", type=Path, default=Path("qc"),
                   help="Output prefix; writes <prefix>.csv and <prefix>.pdf (default: ./qc)")
    p.add_argument("-p", "--pattern", default="*-png",
                   help="Session glob pattern (default: *-png)")
    p.add_argument("--csv-only", action="store_true", help="Skip PDF generation")
    p.add_argument("--pdf-only", action="store_true", help="Skip CSV generation")
    args = p.parse_args(argv)

    if not args.root.exists():
        print(f"error: {args.root} does not exist", file=sys.stderr)
        return 1

    keep_arrays = not args.csv_only  # only need arrays for PDF
    session_data = scan_sessions(args.root, pattern=args.pattern, keep_arrays=keep_arrays)
    if not session_data:
        print(f"error: no sessions matching {args.pattern} under {args.root}", file=sys.stderr)
        return 1

    total_eps = sum(len(v) for v in session_data.values())
    print(f"Scanned {len(session_data)} session(s), {total_eps} episode(s)")

    if not args.pdf_only:
        csv_path = args.output.with_suffix(".csv")
        write_csv(session_data, csv_path)
        print(f"  wrote {csv_path}")

    if not args.csv_only:
        # Ensure we have arrays if user passed --pdf-only without prior scan
        if not keep_arrays:
            session_data = scan_sessions(args.root, pattern=args.pattern, keep_arrays=True)
        pdf_path = args.output.with_suffix(".pdf")
        render_pdf(session_data, pdf_path)
        print(f"  wrote {pdf_path}")

    # Print category breakdown
    counts = defaultdict(int)
    for eps in session_data.values():
        for e in eps:
            counts[e.category] += 1
    print("Category counts:")
    for c, n in sorted(counts.items()):
        print(f"  {c:18s} {n}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
