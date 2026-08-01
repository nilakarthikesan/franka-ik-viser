"""Preview the two shipped circle-drawing surfaces (design artifact for the notes).

Renders, side by side:
  - TABLE: horizontal circle (XY plane), gripper points straight down INTO it.
  - WALL:  vertical circle (YZ plane), gripper points forward (+x) INTO it.

For each: the arm skeleton at one pose, the reference circle, and the gripper
"pen" axis (short arrows) at points around the loop. If the arrows stab through
the disk, it looks like drawing; if they lie flat in the disk, it looks like
waving. Also reports IK reachability and the tool-axis vs plane-normal angle.

Run: PYTHONPATH=src .venv/bin/python scripts/preview_orientations.py
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

import sys
REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src"))

from franka_ik.ik import DLSSolver
from franka_ik.robot_model import Q_HOME, FrankaModel

R_HOME = None  # gripper-down orientation, filled in main


def table_pose(t: float, period=8.0, radius=0.12):
    """Horizontal circle on a 'table'; gripper points down (home orientation)."""
    ang = 2 * np.pi * t / period
    c = np.array([0.45, 0.0, 0.35])
    p = c + np.array([radius * np.cos(ang), radius * np.sin(ang), 0.0])
    T = np.eye(4); T[:3, :3] = R_HOME; T[:3, 3] = p
    return T


# gripper approach axis (local +z) aligned with world +x -> points into a wall
R_WALL = np.array([[0.0, 0.0, 1.0],
                   [0.0, -1.0, 0.0],
                   [1.0, 0.0, 0.0]])


def wall_pose(t: float, period=8.0, radius=0.10):
    """Vertical circle facing the robot; gripper points forward (+x) into it.

    Center/radius match the shipped 'wall' surface (franka_ik.trajectory), chosen
    so every sample is comfortably reachable by the FR3.
    """
    ang = 2 * np.pi * t / period
    c = np.array([0.5, 0.0, 0.5])
    p = c + np.array([0.0, radius * np.cos(ang), radius * np.sin(ang)])
    T = np.eye(4); T[:3, :3] = R_WALL; T[:3, 3] = p
    return T


def solve_loop(model, solver, pose_fn, n=24):
    """IK to n points around the loop; return arm skeletons, tcp pts, tool axes, errors."""
    q = solver.solve(Q_HOME, pose_fn(0.0)).q
    skels, tcps, axes, perrs = [], [], [], []
    for k in range(n):
        T = pose_fn(8.0 * k / n)
        res = solver.solve(q, T)  # warm-started static solve
        q = res.q
        fk = model.fk_all(q)
        skel = np.vstack([[0, 0, 0], fk.joint_positions, fk.T_tcp[:3, 3]])
        skels.append(skel)
        tcps.append(fk.T_tcp[:3, 3])
        axes.append(fk.T_tcp[:3, 2])  # gripper approach axis (local +z)
        perrs.append(res.pos_err)
    return np.array(skels), np.array(tcps), np.array(axes), np.array(perrs)


def draw(ax, title, tcps, axes, skel, perr_mm):
    ax.plot(tcps[:, 0], tcps[:, 1], tcps[:, 2], color="#4aa0ff", lw=2.5, label="reference circle")
    ax.plot(skel[:, 0], skel[:, 1], skel[:, 2], "-o", color="#333", ms=3, lw=2, label="arm @ one pose")
    ax.scatter([0], [0], [0], color="k", s=40)
    for p, a in zip(tcps, axes):
        e = p + 0.06 * a  # 6 cm "pen" arrow
        ax.plot([p[0], e[0]], [p[1], e[1]], [p[2], e[2]], color="#ff7828", lw=1.8)
    ax.set_title(f"{title}\nIK max err {perr_mm:.3f} mm; orange = gripper 'pen' axis")
    ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("z")
    ax.set_xlim(-0.1, 0.6); ax.set_ylim(-0.35, 0.35); ax.set_zlim(0, 0.7)
    ax.legend(loc="upper left", fontsize=8)
    ax.view_init(elev=18, azim=-60)


def main():
    global R_HOME
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    model = FrankaModel()
    R_HOME = model.fk(Q_HOME)[:3, :3]
    solver = DLSSolver(model)

    fig = plt.figure(figsize=(15, 7))
    for i, (name, fn) in enumerate([("TABLE  (horizontal, pen down)", table_pose),
                                    ("WALL  (vertical, pen forward)", wall_pose)]):
        skels, tcps, axes, perrs = solve_loop(model, solver, fn)
        # plane normal from three points; tool-vs-normal angle
        n = np.cross(tcps[6] - tcps[0], tcps[12] - tcps[0]); n /= np.linalg.norm(n)
        ang = np.degrees(np.arccos(abs(np.dot(axes[0], n))))
        ax = fig.add_subplot(1, 2, i + 1, projection="3d")
        draw(ax, name, tcps, axes, skels[6], perrs.max() * 1e3)
        print(f"{name}: reachable={np.all(perrs < 1e-3)}, max err={perrs.max()*1e3:.3f} mm, "
              f"tool-vs-plane-normal={ang:.1f} deg (0=pen into disk, 90=flat)")

    out = REPO / "outputs" / "orientation_preview.png"
    out.parent.mkdir(exist_ok=True)
    fig.tight_layout()
    fig.savefig(out, dpi=120)
    print("saved", out)


if __name__ == "__main__":
    main()
