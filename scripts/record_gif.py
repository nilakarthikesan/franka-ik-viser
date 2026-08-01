"""Render a headless GIF of the arm tracking a trajectory on both surfaces.

This drives the *exact* live pipeline (franka_ik.trajectory.pose + DLSSolver.solve_step),
so the animation reflects true tracking, then draws the arm skeleton, the reference
curve, the growing end-effector trace, and the gripper "pen" axis in a matplotlib 3D
panel per surface. Frames are stitched into a GIF with imageio.

Usage:
  PYTHONPATH=src .venv/bin/python scripts/record_gif.py \
      --shape circle --surfaces table wall --frames 90 --fps 18
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src"))

from franka_ik.ik import DLSSolver, IKConfig
from franka_ik.robot_model import Q_HOME, FrankaModel
from franka_ik.trajectory import period_of, pose, reference_points

VIEW = dict(elev=18, azim=-60)
XLIM, YLIM, ZLIM = (-0.1, 0.6), (-0.35, 0.35), (0.0, 0.75)


def simulate(model, shape, surface, frames):
    """One loop of solve_step tracking; return skeletons, tcp trace, pen axes, errors."""
    solver = DLSSolver(model, IKConfig())
    period = period_of(shape)
    q = solver.solve(Q_HOME, pose(shape, surface, 0.0)).q  # seed on-path (as app converges)
    skels, trace, pens, errs = [], [], [], []
    for k in range(frames):
        t = period * k / frames
        T = pose(shape, surface, t)
        q = solver.solve_step(q, T)
        fk = model.fk_all(q)
        tcp = fk.T_tcp[:3, 3]
        skels.append(np.vstack([[0, 0, 0], fk.joint_positions, tcp]))
        trace.append(tcp)
        pens.append(fk.T_tcp[:3, 2])
        errs.append(solver.error(q, T)[0])
    return np.array(skels), np.array(trace), np.array(pens), np.array(errs)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--shape", default="circle")
    ap.add_argument("--surfaces", nargs="+", default=["table", "wall"])
    ap.add_argument("--frames", type=int, default=90)
    ap.add_argument("--fps", type=int, default=18)
    ap.add_argument("--out", default="docs/images/tracking.gif")
    args = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import imageio.v2 as imageio

    model = FrankaModel()
    sims = {s: simulate(model, args.shape, s, args.frames) for s in args.surfaces}
    refs = {s: reference_points(args.shape, s) for s in args.surfaces}

    n = len(args.surfaces)
    fig = plt.figure(figsize=(5.2 * n, 4.6))
    axes = [fig.add_subplot(1, n, i + 1, projection="3d") for i in range(n)]
    titles = {"table": "table — draw flat, gripper down",
              "wall": "wall — draw upright, gripper forward"}

    out = REPO / args.out
    out.parent.mkdir(parents=True, exist_ok=True)
    frame_imgs = []
    for k in range(args.frames):
        for ax, s in zip(axes, args.surfaces):
            skels, trace, pens, errs = sims[s]
            ref = refs[s]
            ax.clear()
            ax.plot(ref[:, 0], ref[:, 1], ref[:, 2], color="#4aa0ff", lw=2, label="reference")
            tr = trace[: k + 1]
            ax.plot(tr[:, 0], tr[:, 1], tr[:, 2], color="#ff7828", lw=2.5, label="EE trace")
            sk = skels[k]
            ax.plot(sk[:, 0], sk[:, 1], sk[:, 2], "-o", color="#333", ms=3, lw=2, label="arm")
            ax.scatter([0], [0], [0], color="k", s=30)
            tip, pen = trace[k], pens[k]
            e = tip + 0.06 * pen
            ax.plot([tip[0], e[0]], [tip[1], e[1]], [tip[2], e[2]], color="#d62728", lw=2)
            ax.set_xlim(*XLIM); ax.set_ylim(*YLIM); ax.set_zlim(*ZLIM)
            ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("z")
            ax.view_init(**VIEW)
            ax.set_title(f"{titles.get(s, s)}\nerr {errs[k]*1e3:.3f} mm", fontsize=9)
            if k == 0:
                ax.legend(loc="upper left", fontsize=7)
        fig.suptitle(f"FR3 tracking a {args.shape} (DLS IK, ~50 Hz)", fontsize=12)
        fig.tight_layout()
        fig.canvas.draw()
        frame_imgs.append(np.asarray(fig.canvas.buffer_rgba())[..., :3].copy())

    plt.close(fig)
    imageio.mimsave(out, frame_imgs, fps=args.fps, loop=0)
    size_kb = out.stat().st_size // 1024
    print(f"saved {out} ({args.frames} frames @ {args.fps} fps, {size_kb} KB)")


if __name__ == "__main__":
    main()
