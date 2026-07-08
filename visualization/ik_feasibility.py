#!/usr/bin/env python3
"""
IK trajectory-feasibility checker for the Piper arm (EXPERIMENTAL, standalone).

Runs the same placo inverse kinematics the real Piper deploy uses
(`examples/umi_relative_ee/deploy_umi_relative_ee_piper.py`) over each recorded
SROI camera trajectory and reports whether the arm could physically track it.
Emits a per-dataset CSV + multi-page PDF, mirroring `visualization/qc.py`.

Feasibility model
-----------------
"Starting from the deploy's ready pose (START_POSE_DEG — the pose the arm moves to
before control engages when running without --warm_start), can the gripper camera
track the recorded relative SE(3) motion within tolerance?" The unknown
robot-base-in-world calibration is absorbed by anchoring frame 0 of the trajectory
to the forward-kinematics pose of the seed (the same trick
`lerobot.datasets.ee_to_joint_converter.convert_episode` uses). The IK target frame
is `camera_link` (the deploy's `deploy_frame`), so the SROI camera trajectory is the
IK target directly — no separate gripper<->camera transform is needed.

Per frame: solve IK (seed = previous solution, reset_state=True), verify with FK
(position + rotation residual), and record joint positions. Per episode: classify
into ik_ok / ik_unreachable / ik_joint_limit / ik_branch_jump / ik_no_traj.

Assumptions / limitations
-------------------------
- Warm-start anchor is NECESSARY, NOT SUFFICIENT: a pass means "the deploy IK would
  accept this trajectory from HOME_POSE_DEG within tolerance". It does not guarantee
  dynamic executability at 30 Hz or that the real base placement matches. Verdicts
  are seed-dependent (choose another seed via --seed).
- camera_link is the IK target; the SROI trajectory is treated as camera_link motion
  (same `CameraTrajectoryTransformed.txt` file qc.py + sroi_to_lerobot consume). A
  convention mismatch surfaces as inflated rot_err, not as a false pass on reachability.
- placo does NOT clamp to joint limits, so violations are detected post-solve against
  URDF limits (±margin).
- placo lives only in the py310 conda env. Run with:
    /home/zfei/anaconda3/envs/py310/bin/python visualization/ik_feasibility.py ...

Not wired into sroi_to_lerobot.py or the production pipeline.

Usage
-----
    # Single session (CSV + PDF)
    python visualization/ik_feasibility.py /path/to/session-png -o /tmp/ik_report
    # Multi-session parent
    python visualization/ik_feasibility.py /path/to/day -o /tmp/ik_report
    # CSV only (no matplotlib PDF)
    python visualization/ik_feasibility.py /path/to/day --csv-only -o /tmp/ik_report
"""

import argparse
import csv
import json
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import gridspec
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np
from scipy.spatial.transform import Rotation


# ============================================================================
# Config — all tunable values live in configs/ik_filters.json
# ============================================================================

DEFAULT_FILTERS_PATH = Path(__file__).resolve().parent.parent / "configs" / "ik_filters.json"

URDF_PATH: str = ""
URDF_FALLBACK_PATH: str = ""
LEROBOT_SRC_PATH: str = ""
TARGET_FRAME_NAME: str = "camera_link"
JOINT_NAMES: list[str] = []
SEED_POSES: dict[str, list[float]] = {}
DEFAULT_SEED_NAME: str = "home"
POSITION_WEIGHT: float = 1.0
ORIENTATION_WEIGHT: float = 0.1
POSITION_TOL_M: float = 0.02
ROTATION_TOL_RAD: float = 0.3
TRAJ_FILENAME: str = "CameraTrajectoryTransformed.txt"
MAX_JOINT_STEP_DEG: float = 30.0
MAX_EE_STEP_M: float = 0.05
JOINT_LIMIT_MARGIN_DEG: float = 1.0
EE_BOUNDS_MIN: np.ndarray = np.array([-0.5, -0.5, -0.1])
EE_BOUNDS_MAX: np.ndarray = np.array([0.5, 0.5, 0.6])
MIN_TRAJ_FRAMES: int = 30
MAX_FAILED_RATIO: float = 0.1
CATEGORY_COLORS: dict[str, str] = {}
CATEGORY_DEFAULT_COLOR: str = "#999999"
PROBLEM_CATEGORIES: set[str] = set()
CATEGORY_SORT_ORDER: dict[str, int] = {}
PANEL_BG: str = "#f5f7fa"
EPISODES_PER_ROW: int = 1
FIGSIZE_SESSION: tuple[float, float] = (11.0, 3.4)
FIGSIZE_SUMMARY: tuple[float, float] = (11.0, 0.42)
FIGSIZE_SUMMARY_BASE_HEIGHT: float = 1.6
OUTER_HSPACE: float = 0.55
OUTER_MARGINS: tuple[float, float, float, float] = (0.06, 0.98, 0.96, 0.05)
ERR_COLOR: str = "#1565c0"
ROT_COLOR: str = "#00838f"
JOINT_PALETTE: list[str] = []
LIMIT_COLOR: str = "#c62828"


def load_filters(path: Path | None = None) -> None:
    """Load IK filter rules from JSON into module-level config vars."""
    global URDF_PATH, URDF_FALLBACK_PATH, LEROBOT_SRC_PATH, TARGET_FRAME_NAME, JOINT_NAMES
    global SEED_POSES, DEFAULT_SEED_NAME
    global POSITION_WEIGHT, ORIENTATION_WEIGHT, POSITION_TOL_M, ROTATION_TOL_RAD
    global TRAJ_FILENAME, MAX_JOINT_STEP_DEG, MAX_EE_STEP_M, JOINT_LIMIT_MARGIN_DEG
    global EE_BOUNDS_MIN, EE_BOUNDS_MAX, MIN_TRAJ_FRAMES, MAX_FAILED_RATIO
    global CATEGORY_COLORS, CATEGORY_DEFAULT_COLOR, PROBLEM_CATEGORIES, CATEGORY_SORT_ORDER
    global PANEL_BG, EPISODES_PER_ROW, FIGSIZE_SESSION, FIGSIZE_SUMMARY, FIGSIZE_SUMMARY_BASE_HEIGHT
    global OUTER_HSPACE, OUTER_MARGINS, ERR_COLOR, ROT_COLOR, JOINT_PALETTE, LIMIT_COLOR

    filters_path = path or DEFAULT_FILTERS_PATH
    if not filters_path.exists():
        raise FileNotFoundError(
            f"IK filters file not found: {filters_path}\n"
            f"Expected configs/ik_filters.json next to visualization/ik_feasibility.py."
        )
    with filters_path.open() as f:
        cfg = json.load(f)

    r = cfg["robot"]
    URDF_PATH = r["urdf_path"]
    URDF_FALLBACK_PATH = r.get("urdf_fallback_path", "")
    LEROBOT_SRC_PATH = r["lerobot_src_path"]
    TARGET_FRAME_NAME = r["target_frame_name"]
    JOINT_NAMES = list(r["joint_names"])

    sp = {k: list(v) for k, v in cfg["seed_poses"].items() if not k.startswith("_")}
    SEED_POSES = {k: v for k, v in sp.items() if isinstance(v, list)}
    DEFAULT_SEED_NAME = cfg.get("default_seed_name", "home")

    ik = cfg["ik"]
    POSITION_WEIGHT = float(ik["position_weight"])
    ORIENTATION_WEIGHT = float(ik["orientation_weight"])
    POSITION_TOL_M = float(ik["position_tol_m"])
    ROTATION_TOL_RAD = float(ik["rotation_tol_rad"])

    t = cfg["trajectory"]
    TRAJ_FILENAME = t.get("traj_filename", "CameraTrajectoryTransformed.txt")
    MAX_JOINT_STEP_DEG = float(t.get("max_joint_step_deg", 30.0))
    MAX_EE_STEP_M = float(t.get("max_ee_step_m", 0.05))
    JOINT_LIMIT_MARGIN_DEG = float(t.get("joint_limit_margin_deg", 1.0))
    EE_BOUNDS_MIN = np.array(t.get("ee_bounds_min", [-0.5, -0.5, -0.1]), dtype=float)
    EE_BOUNDS_MAX = np.array(t.get("ee_bounds_max", [0.5, 0.5, 0.6]), dtype=float)

    th = cfg["thresholds"]
    MIN_TRAJ_FRAMES = int(th.get("min_traj_frames", 30))
    MAX_FAILED_RATIO = float(th.get("max_failed_ratio", 0.1))

    cats = {k: v for k, v in cfg["categories"].items() if isinstance(v, dict)}
    CATEGORY_COLORS = {k: v["color"] for k, v in cats.items()}
    PROBLEM_CATEGORIES = {k for k, v in cats.items() if v.get("problem")}
    CATEGORY_SORT_ORDER = {k: v["priority"] for k, v in cats.items()}

    d = cfg["display"]
    CATEGORY_DEFAULT_COLOR = d.get("category_default_color", "#999999")
    PANEL_BG = d.get("panel_background", "#f5f7fa")
    EPISODES_PER_ROW = int(d.get("episodes_per_row", 1))
    FIGSIZE_SESSION = tuple(d.get("figsize_session", [11.0, 3.4]))
    FIGSIZE_SUMMARY = tuple(d.get("figsize_summary", [11.0, 0.42]))
    FIGSIZE_SUMMARY_BASE_HEIGHT = float(d.get("figsize_summary_base_height", 1.6))
    OUTER_HSPACE = float(d.get("outer_hspace", 0.55))
    OUTER_MARGINS = tuple(d.get("outer_margins", [0.06, 0.98, 0.96, 0.05]))
    ERR_COLOR = d.get("err_color", "#1565c0")
    ROT_COLOR = d.get("rot_color", "#00838f")
    JOINT_PALETTE = list(d.get("joint_palette", []))
    LIMIT_COLOR = d.get("limit_color", "#c62828")


# Auto-load at import so the importable API works without an explicit call.
load_filters()


# ============================================================================
# Data class
# ============================================================================


@dataclass
class EpisodeIK:
    session: str
    episode: str
    path: str
    category: str
    seed_name: str = ""
    n_frames: int = 0
    n_feasible: int = 0
    n_pos_fail: int = 0
    n_rot_fail: int = 0
    n_joint_limit_viol: int = 0
    n_branch_jumps: int = 0
    n_ee_oob: int = 0
    n_ee_jumps: int = 0
    max_pos_err_mm: float = 0.0
    mean_pos_err_mm: float = 0.0
    max_rot_err_deg: float = 0.0
    mean_rot_err_deg: float = 0.0
    max_joint_step_deg: float = 0.0
    max_ee_step_mm: float = 0.0
    joint_limit_joints: str = ""
    path_length_cm: float = 0.0
    # Raw arrays (only kept if keep_arrays=True on scan)
    pos_err_mm: np.ndarray | None = field(default=None, repr=False)
    rot_err_deg: np.ndarray | None = field(default=None, repr=False)
    joints_deg: np.ndarray | None = field(default=None, repr=False)
    ee_pos_m: np.ndarray | None = field(default=None, repr=False)
    ee_rpy_deg: np.ndarray | None = field(default=None, repr=False)
    reached_pos_m: np.ndarray | None = field(default=None, repr=False)
    reached_rpy_deg: np.ndarray | None = field(default=None, repr=False)
    feasible_mask: np.ndarray | None = field(default=None, repr=False)


# ============================================================================
# Lazy IK import + kinematics construction
# ============================================================================


def _ensure_lerobot_on_path(lerobot_src_path: str) -> None:
    p = str(Path(lerobot_src_path).expanduser().resolve())
    if not (Path(p) / "lerobot" / "model" / "kinematics.py").exists():
        raise FileNotFoundError(
            f"lerobot src not found at {p} (expected .../lerobot/model/kinematics.py).\n"
            f"Set 'lerobot_src_path' in configs/ik_filters.json or pass --lerobot-src."
        )
    if p not in sys.path:
        sys.path.insert(0, p)


def _import_robot_kinematics():
    """Lazy-import lerobot.model.kinematics.RobotKinematics with an actionable error."""
    try:
        import placo  # noqa: F401
    except ImportError as e:
        raise ImportError(
            "placo is required for ik_feasibility.py but is not installed in this environment.\n"
            "placo lives only in the py310 conda env. Run with:\n"
            "  /home/zfei/anaconda3/envs/py310/bin/python visualization/ik_feasibility.py ...\n"
            f"Original error: {e}"
        ) from e
    from lerobot.model.kinematics import RobotKinematics
    return RobotKinematics


def _resolve_urdf(urdf_path: str | None = None) -> str:
    p = urdf_path or URDF_PATH
    if p and Path(p).exists():
        return p
    if URDF_FALLBACK_PATH and Path(URDF_FALLBACK_PATH).exists():
        print(f"WARNING: URDF not found at {p}; falling back to {URDF_FALLBACK_PATH}")
        return URDF_FALLBACK_PATH
    raise FileNotFoundError(
        f"Piper URDF not found. Tried {p}" + (f" and fallback {URDF_FALLBACK_PATH}" if URDF_FALLBACK_PATH else "")
        + ".\nPass --urdf or set 'urdf_path'/'urdf_fallback_path' in configs/ik_filters.json."
    )


def make_kinematics(urdf_path: str | None = None, target_frame: str | None = None,
                    joint_names: list[str] | None = None):
    """Construct a RobotKinematics lazily (the heavy placo import happens here).

    Suppresses placo's self-collision stderr warnings during construction (the deploy
    avoids them via placo Flags, which RobotKinematics doesn't expose). Pure cosmetic.
    """
    _ensure_lerobot_on_path(LEROBOT_SRC_PATH)
    RobotKinematics = _import_robot_kinematics()
    import os
    saved_fd = os.dup(2)
    devnull = os.open(os.devnull, os.O_WRONLY)
    try:
        os.dup2(devnull, 2)
        kin = RobotKinematics(
            urdf_path=_resolve_urdf(urdf_path),
            target_frame_name=target_frame or TARGET_FRAME_NAME,
            joint_names=joint_names or JOINT_NAMES,
        )
    finally:
        os.dup2(saved_fd, 2)
        os.close(devnull)
        os.close(saved_fd)
    return kin


# ============================================================================
# Trajectory + URDF helpers
# ============================================================================


def load_camera_poses(path: Path) -> np.ndarray | None:
    """Load CameraTrajectoryTransformed.txt (13-col: timestamp + 3x4) or the raw
    CameraTrajectory.txt (12-col: 3x4) -> (N, 4, 4) world<-camera poses.

    Returns None on missing/empty/unparseable files.
    """
    if not path.exists():
        return None
    try:
        raw = np.loadtxt(path, comments="#")
    except Exception:
        return None
    if raw.size == 0:
        return None
    if raw.ndim == 1:
        raw = raw.reshape(1, -1)
    # 13 cols: drop leading timestamp. 12 cols: use as-is.
    if raw.shape[1] == 13:
        vals = raw[:, 1:]
    elif raw.shape[1] == 12:
        vals = raw
    else:
        return None
    poses34 = vals.reshape(-1, 3, 4)
    bottom = np.tile(np.array([0.0, 0.0, 0.0, 1.0]), (len(poses34), 1, 1))
    return np.concatenate([poses34, bottom], axis=1)  # (N, 4, 4)


def parse_joint_limits(urdf_path: str, joint_names: list[str]) -> dict[str, tuple[float, float]]:
    """Parse revolute joint limits (rad->deg) for the requested joints from a URDF."""
    root = ET.parse(urdf_path).getroot()
    out: dict[str, tuple[float, float]] = {}
    for j in root.findall("joint"):
        name = j.attrib.get("name")
        if name not in joint_names or j.attrib.get("type") != "revolute":
            continue
        lim = j.find("limit")
        if lim is None:
            continue
        lo = float(lim.attrib["lower"])
        hi = float(lim.attrib["upper"])
        out[name] = (float(np.rad2deg(lo)), float(np.rad2deg(hi)))
    missing = [n for n in joint_names if n not in out]
    if missing:
        raise ValueError(f"Joint limits not found in {urdf_path} for: {missing}")
    return out


# ============================================================================
# Core IK loop
# ============================================================================


def solve_episode_ik(
    poses: np.ndarray,
    kinematics,
    seed_deg: np.ndarray,
    position_weight: float = POSITION_WEIGHT,
    orientation_weight: float = ORIENTATION_WEIGHT,
    position_tol_m: float = POSITION_TOL_M,
    rotation_tol_rad: float = ROTATION_TOL_RAD,
) -> dict:
    """Run the feasibility IK loop over (N,4,4) camera poses.

    Anchors frame 0 to FK(seed) (absorbs the unknown base placement), then solves
    IK per frame seeded with the last valid solution (gate-updated to prevent drift,
    matching convert_episode). Returns per-frame arrays.
    """
    n = len(poses)
    fk_seed = kinematics.forward_kinematics(seed_deg)
    alignment = fk_seed @ np.linalg.inv(poses[0])

    joints = np.zeros((n, len(seed_deg)), dtype=float)
    pos_err = np.zeros(n, dtype=float)
    rot_err = np.zeros(n, dtype=float)
    ee_pos = np.zeros((n, 3), dtype=float)        # desired EE position (m)
    ee_rpy = np.zeros((n, 3), dtype=float)        # desired EE roll/pitch/yaw (deg)
    reached_pos = np.zeros((n, 3), dtype=float)   # FK(solution) EE position (m)
    reached_rpy = np.zeros((n, 3), dtype=float)   # FK(solution) EE roll/pitch/yaw (deg)
    feasible = np.zeros(n, dtype=bool)

    last_valid = seed_deg.astype(float).copy()
    for t in range(n):
        t_des = alignment @ poses[t]
        q = kinematics.inverse_kinematics(
            current_joint_pos=last_valid,
            desired_ee_pose=t_des,
            position_weight=position_weight,
            orientation_weight=orientation_weight,
            reset_state=True,
        )
        fk_q = kinematics.forward_kinematics(q)
        pe = float(np.linalg.norm(fk_q[:3, 3] - t_des[:3, 3]))
        r_rel = t_des[:3, :3].T @ fk_q[:3, :3]
        re = float(np.linalg.norm(Rotation.from_matrix(r_rel).as_rotvec()))
        ok = (pe < position_tol_m) and (re < rotation_tol_rad)
        joints[t] = q
        pos_err[t] = pe
        rot_err[t] = re
        ee_pos[t] = t_des[:3, 3]
        ee_rpy[t] = np.rad2deg(Rotation.from_matrix(t_des[:3, :3]).as_euler("xyz"))
        reached_pos[t] = fk_q[:3, 3]
        reached_rpy[t] = np.rad2deg(Rotation.from_matrix(fk_q[:3, :3]).as_euler("xyz"))
        feasible[t] = ok
        if ok:
            last_valid = q.astype(float)
    return {
        "joints_deg": joints,
        "pos_err_m": pos_err,
        "rot_err_rad": rot_err,
        "ee_pos_m": ee_pos,
        "ee_rpy_deg": ee_rpy,
        "reached_pos_m": reached_pos,
        "reached_rpy_deg": reached_rpy,
        "feasible_mask": feasible,
    }


# ============================================================================
# Classification
# ============================================================================


def classify_ik(
    poses: np.ndarray | None,
    result: dict | None,
    joint_limits: dict[str, tuple[float, float]],
    joint_names: list[str],
) -> tuple[str, dict[str, int], set[str]]:
    """Return (category, int_counts, joint_limit_joints) following the priority chain."""
    counts: dict[str, int] = dict(n_frames=0, n_feasible=0, n_pos_fail=0, n_rot_fail=0,
                                  n_joint_limit_viol=0, n_branch_jumps=0, n_ee_oob=0, n_ee_jumps=0)
    jl_joints: set[str] = set()
    if poses is None or result is None or len(poses) < MIN_TRAJ_FRAMES:
        return "ik_no_traj", counts, jl_joints

    joints = result["joints_deg"]
    pos_err = result["pos_err_m"]
    rot_err = result["rot_err_rad"]
    ee_pos = result["ee_pos_m"]
    feas = result["feasible_mask"]
    n = len(poses)
    counts["n_frames"] = n
    counts["n_feasible"] = int(feas.sum())
    counts["n_pos_fail"] = int((pos_err >= POSITION_TOL_M).sum())
    counts["n_rot_fail"] = int((rot_err >= ROTATION_TOL_RAD).sum())

    # Joint-limit violations (per frame, any joint outside limit +/- margin)
    margin = JOINT_LIMIT_MARGIN_DEG
    viol_frames = np.zeros(n, dtype=bool)
    for i, name in enumerate(joint_names):
        lo, hi = joint_limits[name]
        bad = (joints[:, i] < lo - margin) | (joints[:, i] > hi + margin)
        if bad.any():
            jl_joints.add(name)
        viol_frames |= bad
    counts["n_joint_limit_viol"] = int(viol_frames.sum())

    # Branch jumps: per-frame max per-joint step > threshold
    if n >= 2:
        steps = np.abs(np.diff(joints, axis=0))  # (n-1, njoints)
        jump_frames = (steps.max(axis=1) > MAX_JOINT_STEP_DEG)
        counts["n_branch_jumps"] = int(jump_frames.sum())

    # EE bounds + step
    in_box = np.all((ee_pos >= EE_BOUNDS_MIN) & (ee_pos <= EE_BOUNDS_MAX), axis=1)
    counts["n_ee_oob"] = int((~in_box).sum())
    if n >= 2:
        ee_steps = np.linalg.norm(np.diff(ee_pos, axis=0), axis=1)
        counts["n_ee_jumps"] = int((ee_steps > MAX_EE_STEP_M).sum())

    # Priority chain
    if counts["n_joint_limit_viol"] > 0:
        return "ik_joint_limit", counts, jl_joints
    if counts["n_branch_jumps"] > 0:
        return "ik_branch_jump", counts, jl_joints
    failed_ratio = (n - counts["n_feasible"]) / n
    if failed_ratio > MAX_FAILED_RATIO:
        return "ik_unreachable", counts, jl_joints
    return "ik_ok", counts, jl_joints


# ============================================================================
# Scan
# ============================================================================


def _build_episode_ik(session: str, episode_dir: Path, kinematics, joint_limits,
                      joint_names, seed_name: str, keep_arrays: bool) -> EpisodeIK:
    episode = episode_dir.name
    traj_path = episode_dir / TRAJ_FILENAME
    poses = load_camera_poses(traj_path)
    seed_deg = np.array(SEED_POSES[seed_name], dtype=float)

    category = "ik_no_traj"
    result = None
    counts: dict[str, int] = dict(n_frames=0, n_feasible=0, n_pos_fail=0, n_rot_fail=0,
                                  n_joint_limit_viol=0, n_branch_jumps=0, n_ee_oob=0, n_ee_jumps=0)
    jl_joints: set[str] = set()
    max_pos_err_mm = mean_pos_err_mm = 0.0
    max_rot_err_deg = mean_rot_err_deg = 0.0
    max_joint_step_deg = max_ee_step_mm = 0.0
    path_length_cm = 0.0

    if poses is not None and len(poses) >= MIN_TRAJ_FRAMES:
        try:
            result = solve_episode_ik(poses, kinematics, seed_deg)
            category, counts, jl_joints = classify_ik(poses, result, joint_limits, joint_names)
            pos_err_mm = result["pos_err_m"] * 1000.0
            rot_err_deg = np.rad2deg(result["rot_err_rad"])
            max_pos_err_mm = float(pos_err_mm.max())
            mean_pos_err_mm = float(pos_err_mm.mean())
            max_rot_err_deg = float(rot_err_deg.max())
            mean_rot_err_deg = float(rot_err_deg.mean())
            ee = result["ee_pos_m"]
            if len(poses) >= 2:
                path_length_cm = float(np.sum(np.linalg.norm(np.diff(ee, axis=0), axis=1)) * 100)
                max_joint_step_deg = float(np.abs(np.diff(result["joints_deg"], axis=0)).max())
                max_ee_step_mm = float(np.linalg.norm(np.diff(ee, axis=0), axis=1).max() * 1000)
        except Exception as e:
            print(f"  WARNING: IK failed for {session}/{episode}: {e}")
            category = "ik_no_traj"

    e = EpisodeIK(
        session=session, episode=episode, path=str(traj_path), category=category,
        seed_name=seed_name, n_frames=counts["n_frames"], n_feasible=counts["n_feasible"],
        n_pos_fail=counts["n_pos_fail"], n_rot_fail=counts["n_rot_fail"],
        n_joint_limit_viol=counts["n_joint_limit_viol"], n_branch_jumps=counts["n_branch_jumps"],
        n_ee_oob=counts["n_ee_oob"], n_ee_jumps=counts["n_ee_jumps"],
        joint_limit_joints=",".join(j for j in joint_names if j in jl_joints),
        max_pos_err_mm=max_pos_err_mm, mean_pos_err_mm=mean_pos_err_mm,
        max_rot_err_deg=max_rot_err_deg, mean_rot_err_deg=mean_rot_err_deg,
        max_joint_step_deg=max_joint_step_deg, max_ee_step_mm=max_ee_step_mm,
        path_length_cm=path_length_cm,
    )
    if keep_arrays and result is not None:
        e.pos_err_mm = result["pos_err_m"] * 1000.0
        e.rot_err_deg = np.rad2deg(result["rot_err_rad"])
        e.joints_deg = result["joints_deg"]
        e.ee_pos_m = result["ee_pos_m"]
        e.ee_rpy_deg = result["ee_rpy_deg"]
        e.reached_pos_m = result["reached_pos_m"]
        e.reached_rpy_deg = result["reached_rpy_deg"]
        e.feasible_mask = result["feasible_mask"]
    return e


def scan_sessions_ik(root: Path, pattern: str = "*-png", seed_name: str | None = None,
                     keep_arrays: bool = True, urdf_path: str | None = None) -> dict[str, list[EpisodeIK]]:
    """Scan a session tree, returning {session_name: [EpisodeIK, ...]}.

    Constructs kinematics + parses joint limits ONCE (placo init is expensive).
    """
    seed_name = seed_name or DEFAULT_SEED_NAME
    if seed_name not in SEED_POSES:
        raise ValueError(f"Seed '{seed_name}' not in seed_poses {sorted(SEED_POSES)}")

    resolved_urdf = _resolve_urdf(urdf_path)
    print(f"URDF: {resolved_urdf}")
    print(f"Target frame: {TARGET_FRAME_NAME}, joints: {JOINT_NAMES}, seed: {seed_name}")
    kinematics = make_kinematics(urdf_path=resolved_urdf)
    joint_limits = parse_joint_limits(resolved_urdf, JOINT_NAMES)
    print(f"Joint limits (deg): " + ", ".join(f"{k}[{lo:.0f},{hi:.0f}]" for k, (lo, hi) in joint_limits.items()))

    out: dict[str, list[EpisodeIK]] = {}
    # Auto-detect: if root itself contains episode_* dirs, treat root as a single session;
    # otherwise glob for session dirs matching `pattern` (e.g. *-png) under root.
    if any(root.glob("episode_*")):
        sessions = [root]
    else:
        sessions = sorted([d for d in root.glob(pattern) if d.is_dir()])
    for s in sessions:
        eps = []
        for ep_dir in sorted(s.glob("episode_*")):
            if not ep_dir.is_dir():
                continue
            eps.append(_build_episode_ik(s.name, ep_dir, kinematics, joint_limits, JOINT_NAMES,
                                         seed_name, keep_arrays))
        if eps:
            out[s.name] = eps
    return out


# ============================================================================
# CSV output
# ============================================================================

CSV_FIELDS_IK = [
    "session", "episode", "category", "seed_name",
    "n_frames", "n_feasible", "n_pos_fail", "n_rot_fail",
    "n_joint_limit_viol", "n_branch_jumps", "n_ee_oob", "n_ee_jumps",
    "max_pos_err_mm", "mean_pos_err_mm", "max_rot_err_deg", "mean_rot_err_deg",
    "max_joint_step_deg", "max_ee_step_mm", "joint_limit_joints", "path_length_cm", "path",
]


def write_csv_ik(session_data: dict[str, list[EpisodeIK]], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=CSV_FIELDS_IK)
        w.writeheader()
        for session in sorted(session_data):
            for e in session_data[session]:
                row = {k: getattr(e, k) for k in CSV_FIELDS_IK}
                w.writerow(row)


# ============================================================================
# PDF output
# ============================================================================


def _cat_color(category: str) -> str:
    return CATEGORY_COLORS.get(category, CATEGORY_DEFAULT_COLOR)


def _render_summary_page_ik(pdf: PdfPages, session_data: dict[str, list[EpisodeIK]]):
    sessions = sorted(session_data.keys())
    n = len(sessions)
    fig, ax = plt.subplots(figsize=(FIGSIZE_SUMMARY[0], FIGSIZE_SUMMARY[1] * n + FIGSIZE_SUMMARY_BASE_HEIGHT))
    ax.axis("off")

    cat_keys = [c for c in CATEGORY_SORT_ORDER.keys() if c != "ik_ok"] + ["ik_ok"]
    rows = []
    for s in sessions:
        eps = session_data[s]
        cats = [e.category for e in eps]
        row = [s, f"{len(eps)}"]
        for c in cat_keys:
            row.append(f"{sum(1 for x in cats if x == c)}")
        posmax = [e.max_pos_err_mm for e in eps]
        row.append(f"{np.mean(posmax):.1f}" if posmax else "0")
        rows.append(row)

    cols = ["session", "eps"] + [c.replace("ik_", "") for c in cat_keys] + ["avg_max_pos_mm"]
    table = ax.table(cellText=rows, colLabels=cols, loc="center", cellLoc="left")
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1, 1.4)
    for j in range(len(cols)):
        table[(0, j)].set_facecolor("#e0e0e0")
        table[(0, j)].set_text_props(weight="bold")
    for i, s in enumerate(sessions, start=1):
        cats = [e.category for e in session_data[s]]
        if all(c == "ik_ok" for c in cats):
            color = "#c8e6c9"
        elif any(c in PROBLEM_CATEGORIES for c in cats):
            color = "#ffcdd2"
        else:
            color = "#ffe0b2"
        table[(i, 0)].set_facecolor(color)

    total_eps = sum(len(v) for v in session_data.values())
    ax.set_title(f"IK Feasibility Summary — {n} sessions, {total_eps} episodes (seed={DEFAULT_SEED_NAME})",
                 fontsize=13, pad=20)
    pdf.savefig(fig, bbox_inches="tight")
    plt.close(fig)


# Per-axis colors for desired-vs-reached pose panels (x/r, y/g, z/b).
_XYZ_PALETTE = ["#c62828", "#2e7d32", "#1565c0"]
_XYZ_LABELS = ["x", "y", "z"]
_RPY_LABELS = ["roll", "pitch", "yaw"]


def _plot_episode_panels(axes, e: EpisodeIK, joint_limits: dict[str, tuple[float, float]]):
    ax_joints, ax_xyz, ax_rpy, ax_err = axes
    color = _cat_color(e.category)
    title = f"{e.episode}  [{e.category}]  feas {e.n_feasible}/{e.n_frames}  " \
            f"max_pos {e.max_pos_err_mm:.1f}mm  max_rot {e.max_rot_err_deg:.1f}deg"
    if e.joints_deg is None:
        for ax in axes:
            ax.axis("off")
        ax_joints.set_title(title, color=color, fontsize=9)
        return

    frames = np.arange(e.n_frames)

    # Populated jointly with joints_deg above (all set in the keep_arrays block).
    assert e.ee_pos_m is not None and e.reached_pos_m is not None
    assert e.ee_rpy_deg is not None and e.reached_rpy_deg is not None

    # Panel 1 (tallest): per-joint traces (deg) + URDF limit lines
    for i, name in enumerate(JOINT_NAMES):
        c = JOINT_PALETTE[i % len(JOINT_PALETTE)] if JOINT_PALETTE else f"C{i}"
        ax_joints.plot(frames, e.joints_deg[:, i], color=c, lw=1.1, label=name)
        lo, hi = joint_limits[name]
        ax_joints.axhline(lo, color=LIMIT_COLOR, ls=":", lw=0.7)
        ax_joints.axhline(hi, color=LIMIT_COLOR, ls=":", lw=0.7)
    ax_joints.set_ylabel("joints (deg)", fontsize=8)
    ax_joints.set_title(title, color=color, fontsize=9)
    ax_joints.grid(True, alpha=0.3)
    ax_joints.tick_params(labelsize=7)
    ax_joints.legend(fontsize=6, loc="upper right", ncol=6)

    # Panel 2: desired (solid) vs reached (dashed) EE position (m), x/y/z
    for i in range(3):
        c = _XYZ_PALETTE[i]
        ax_xyz.plot(frames, e.ee_pos_m[:, i], color=c, lw=1.2, label=_XYZ_LABELS[i])
        ax_xyz.plot(frames, e.reached_pos_m[:, i], color=c, lw=1.0, ls="--")
    ax_xyz.set_ylabel("xyz (m)\nsolid=des, dash=reach", fontsize=7)
    ax_xyz.grid(True, alpha=0.3)
    ax_xyz.tick_params(labelsize=7)
    ax_xyz.legend(fontsize=6, loc="upper right", ncol=3)

    # Panel 3: desired (solid) vs reached (dashed) EE orientation (deg), roll/pitch/yaw
    for i in range(3):
        c = _XYZ_PALETTE[i]
        ax_rpy.plot(frames, e.ee_rpy_deg[:, i], color=c, lw=1.2, label=_RPY_LABELS[i])
        ax_rpy.plot(frames, e.reached_rpy_deg[:, i], color=c, lw=1.0, ls="--")
    ax_rpy.set_ylabel("rpy (deg)\nsolid=des, dash=reach", fontsize=7)
    ax_rpy.grid(True, alpha=0.3)
    ax_rpy.tick_params(labelsize=7)
    ax_rpy.legend(fontsize=6, loc="upper right", ncol=3)

    # Panel 4: position error (mm) + rotation error (deg) + thresholds
    ax_err.plot(frames, e.pos_err_mm, color=ERR_COLOR, lw=1.0, label="pos err (mm)")
    ax_err.axhline(POSITION_TOL_M * 1000.0, color=ERR_COLOR, ls="--", lw=0.8,
                   label=f"pos tol {POSITION_TOL_M*1000:.0f}mm")
    ax_err.set_ylabel("pos err (mm)", fontsize=8, color=ERR_COLOR)
    ax_err.tick_params(axis="y", labelsize=7, colors=ERR_COLOR)
    ax_err.set_xlabel("frame", fontsize=8)
    ax_err.grid(True, alpha=0.3)
    ax_err.legend(fontsize=6, loc="upper left")
    ax_err.tick_params(axis="x", labelsize=7)
    ax_rot = ax_err.twinx()
    ax_rot.plot(frames, e.rot_err_deg, color=ROT_COLOR, lw=1.0, label="rot err (deg)")
    ax_rot.axhline(np.rad2deg(ROTATION_TOL_RAD), color=ROT_COLOR, ls="--", lw=0.8,
                   label=f"rot tol {np.rad2deg(ROTATION_TOL_RAD):.0f}deg")
    ax_rot.set_ylabel("rot err (deg)", fontsize=8, color=ROT_COLOR)
    ax_rot.tick_params(axis="y", labelsize=7, colors=ROT_COLOR)
    ax_rot.legend(fontsize=6, loc="upper right")


def _render_episode_page_ik(pdf: PdfPages, session_name: str, e: EpisodeIK,
                            joint_limits: dict[str, tuple[float, float]], idx: int, n_total: int):
    """Render ONE episode as its own PDF page: 4 stacked, generously-sized panels.

    One episode per page keeps each page a normal size (a per-session grid of tall
    episodes produced 100"+-tall pages). 4 panels: joints (tallest), desired/reached
    xyz, desired/reached rpy, errors.
    """
    fig = plt.figure(figsize=(FIGSIZE_SESSION[0], FIGSIZE_SESSION[1]))
    inner = gridspec.GridSpec(
        4, 1, figure=fig, hspace=0.5, height_ratios=[2.2, 1.5, 1.5, 1.3],
        left=OUTER_MARGINS[0], right=OUTER_MARGINS[1], top=0.93, bottom=OUTER_MARGINS[3],
    )
    axes = [fig.add_subplot(inner[k, 0]) for k in range(4)]
    _plot_episode_panels(axes, e, joint_limits)
    cat = _cat_color(e.category)
    fig.suptitle(
        f"{session_name} / {e.episode}   [{e.category}]   "
        f"feas {e.n_feasible}/{e.n_frames}   ({idx}/{n_total}, seed={e.seed_name or DEFAULT_SEED_NAME})",
        fontsize=12, y=0.985, color=cat,
    )
    pdf.savefig(fig, bbox_inches="tight")
    plt.close(fig)


def render_pdf_ik(session_data: dict[str, list[EpisodeIK]], output: Path,
                  joint_limits: dict[str, tuple[float, float]]) -> None:
    """Multi-page PDF: one summary page, then ONE PAGE PER EPISODE (problem-first)."""
    output.parent.mkdir(parents=True, exist_ok=True)
    with PdfPages(output) as pdf:
        _render_summary_page_ik(pdf, session_data)

        def issue_score(item):
            return sum(1 for e in item[1] if e.category in PROBLEM_CATEGORIES)

        for name, eps in sorted(session_data.items(), key=lambda kv: (-issue_score(kv), kv[0])):
            eps_sorted = sorted(eps, key=lambda e: (CATEGORY_SORT_ORDER.get(e.category, 9), -e.max_pos_err_mm))
            n_total = len(eps_sorted)
            for i, e in enumerate(eps_sorted, start=1):
                _render_episode_page_ik(pdf, name, e, joint_limits, i, n_total)


# ============================================================================
# CLI
# ============================================================================


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("root", type=Path, help="Directory containing session folders (or a single session).")
    p.add_argument("-o", "--output", type=Path, default=Path("ik_feasibility"),
                   help="Output prefix; writes <prefix>.csv and <prefix>.pdf (default: ./ik_feasibility)")
    p.add_argument("-p", "--pattern", default="*-png", help="Session glob pattern (default: *-png)")
    p.add_argument("--csv-only", action="store_true", help="Skip PDF generation")
    p.add_argument("--pdf-only", action="store_true", help="Skip CSV generation")
    p.add_argument("--filters", type=Path, default=None, help="Override configs/ik_filters.json")
    p.add_argument("--seed", default=None, help="Seed pose name from seed_poses (default: home)")
    p.add_argument("--lerobot-src", default=None, help="Override lerobot src path")
    p.add_argument("--urdf", default=None, help="Override Piper URDF path")
    p.add_argument("--traj-file", default=None, help="Override trajectory filename")
    args = p.parse_args(argv)

    if args.filters is not None:
        load_filters(args.filters)
    if args.lerobot_src:
        global LEROBOT_SRC_PATH
        LEROBOT_SRC_PATH = args.lerobot_src
    if args.traj_file:
        global TRAJ_FILENAME
        TRAJ_FILENAME = args.traj_file

    if not args.root.exists():
        print(f"error: {args.root} does not exist", file=sys.stderr)
        return 1

    keep_arrays = not args.csv_only
    session_data = scan_sessions_ik(args.root, pattern=args.pattern, seed_name=args.seed,
                                    keep_arrays=keep_arrays, urdf_path=args.urdf)
    if not session_data:
        print(f"No sessions matching {args.pattern} under {args.root}", file=sys.stderr)
        return 1

    total_eps = sum(len(v) for v in session_data.values())
    print(f"Scanned {len(session_data)} session(s), {total_eps} episode(s)")

    if not args.pdf_only:
        csv_path = args.output.with_suffix(".csv")
        write_csv_ik(session_data, csv_path)
        print(f"  wrote {csv_path}")

    if not args.csv_only:
        if not keep_arrays:
            session_data = scan_sessions_ik(args.root, pattern=args.pattern, seed_name=args.seed,
                                            keep_arrays=True, urdf_path=args.urdf)
        joint_limits = parse_joint_limits(_resolve_urdf(args.urdf), JOINT_NAMES)
        pdf_path = args.output.with_suffix(".pdf")
        render_pdf_ik(session_data, pdf_path, joint_limits)
        print(f"  wrote {pdf_path}")

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
