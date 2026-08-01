"""Headless trajectory tracking recorder -> CSV + tracking-error plots.

Design and rationale: notes/stage-4-notes.md §4. Runs the *same* solver and
trajectory code as the live app (no separate benchmark path), so the recorded
numbers are exactly what the demo shows. Produces the quantitative artifact for
the README/report (M6).

Run:  python -m franka_ik.record --trajectory circle --periods 2
Output: outputs/tracking_<trajectory>.csv and outputs/tracking_<trajectory>.png
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np

from .ik import DLSSolver, IKConfig
from .robot_model import Q_HOME, FrankaModel
from .trajectory import SHAPES, SURFACES, period_of, pose

HZ = 50.0
OUTPUT_DIR = Path(__file__).resolve().parents[2] / "outputs"


def run(shape: str, surface: str = "table", periods: float = 2.0,
        config: IKConfig | None = None):
    model = FrankaModel()
    solver = DLSSolver(model, config or IKConfig())
    period = period_of(shape)
    n_frames = int(periods * period * HZ)

    # Static solve onto frame 0, then one step per frame (as the live app does).
    q = solver.solve(Q_HOME, pose(shape, surface, 0.0)).q
    rows = []
    for k in range(n_frames + 1):
        t = k / HZ
        T = pose(shape, surface, t)
        q = solver.solve_step(q, T)
        T_cur = model.fk(q)
        pos, rot = solver.error(q, T)
        rows.append((t, *T[:3, 3], *T_cur[:3, 3], pos * 1e3, np.rad2deg(rot)))
    return np.array(rows)


def write_csv(rows: np.ndarray, path: Path) -> None:
    header = ["t", "tx", "ty", "tz", "ax", "ay", "az", "pos_err_mm", "rot_err_deg"]
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows.tolist())


def plot(rows: np.ndarray, label: str, path: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = rows[:, 0]
    tgt, ach = rows[:, 1:4], rows[:, 4:7]
    pos_mm, rot_deg = rows[:, 7], rows[:, 8]

    # Project onto the two axes the path actually spans (XY for table, YZ for wall).
    names = "xyz"
    i, j = np.argsort(np.ptp(tgt, axis=0))[-2:]
    i, j = sorted((int(i), int(j)))

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    ax1.plot(tgt[:, i], tgt[:, j], "-", color="#4aa0ff", lw=2, label="reference")
    ax1.plot(ach[:, i], ach[:, j], "--", color="#ff7828", lw=1.5, label="achieved")
    ax1.set_aspect("equal")
    ax1.set_xlabel(f"{names[i]} (m)")
    ax1.set_ylabel(f"{names[j]} (m)")
    ax1.set_title(f"{label}: reference vs achieved ({names[i]}{names[j]})")
    ax1.legend()
    ax1.grid(alpha=0.3)

    ax2.plot(t, pos_mm, color="#d62728", label="position (mm)")
    ax2.set_xlabel("time (s)")
    ax2.set_ylabel("position error (mm)", color="#d62728")
    ax2.tick_params(axis="y", labelcolor="#d62728")
    ax2b = ax2.twinx()
    ax2b.plot(t, rot_deg, color="#1f77b4", alpha=0.7, label="rotation (deg)")
    ax2b.set_ylabel("rotation error (deg)", color="#1f77b4")
    ax2b.tick_params(axis="y", labelcolor="#1f77b4")
    ax2.set_title(f"{label}: tracking error "
                  f"(max {pos_mm.max():.3f} mm / {rot_deg.max():.3f} deg)")
    ax2.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--shape", choices=list(SHAPES), default="circle")
    parser.add_argument("--surface", choices=list(SURFACES), default="table")
    parser.add_argument("--periods", type=float, default=2.0)
    parser.add_argument("--all", action="store_true",
                        help="record every shape x surface combination")
    args = parser.parse_args()

    OUTPUT_DIR.mkdir(exist_ok=True)
    combos = ([(sh, su) for su in SURFACES for sh in SHAPES] if args.all
              else [(args.shape, args.surface)])
    for shape, surface in combos:
        rows = run(shape, surface, args.periods)
        label = f"{shape}-{surface}"
        csv_path = OUTPUT_DIR / f"tracking_{label}.csv"
        png_path = OUTPUT_DIR / f"tracking_{label}.png"
        write_csv(rows, csv_path)
        plot(rows, label, png_path)
        print(f"{label}: max {rows[:, 7].max():.4f} mm / {rows[:, 8].max():.4f} deg "
              f"-> {csv_path.name}, {png_path.name}")


if __name__ == "__main__":
    main()
