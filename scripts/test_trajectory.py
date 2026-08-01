"""M4/M5 gate: test trajectories and end-to-end tracking (stage-4 notes §5).

  Isolated (trajectory correctness)
    TR1 every sampled pose is valid SE(3) (R orthonormal, det +1)
    TR2 continuity: consecutive 50 Hz samples close in position and rotation
    TR3 periodicity: pose(0) == pose(period)
    TR4 reachability: static solve converges to samples along each path

  Pipeline (tracking + integration)
    TP1 track each trajectory one step/frame @ 50 Hz; max/RMS < 1 mm / 0.5 deg
    TP2 smoothness: per-frame |dq| <= max_step; all configs in-limits
    TP3 recorder produces CSV + PNG with sane contents

Usage:  .venv/bin/python scripts/test_trajectory.py
Exit code is nonzero on any failure, so this can gate CI.
"""

import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from franka_ik import record
from franka_ik.ik import DLSSolver, IKConfig
from franka_ik.robot_model import Q_HOME, FrankaModel
from franka_ik.trajectory import TRAJECTORIES

HZ = 50.0
BAR_POS = 1e-3
BAR_ROT = np.deg2rad(0.5)

results: list[tuple[str, str, bool]] = []


def record_result(name: str, detail: str, passed: bool) -> None:
    results.append((name, detail, passed))
    print(f"  [{'PASS' if passed else 'FAIL'}] {name}: {detail}")


def rotvec_between(Ra: np.ndarray, Rb: np.ndarray) -> float:
    from scipy.spatial.transform import Rotation
    return (Rotation.from_matrix(Ra).inv() * Rotation.from_matrix(Rb)).magnitude()


def periods() -> dict[str, float]:
    return {name: fn.__defaults__[0] for name, fn in TRAJECTORIES.items()}


# ------------------------------------------------------------------ isolated ---

def tr1_valid_se3() -> None:
    worst = 0.0
    for name, fn in TRAJECTORIES.items():
        p = fn.__defaults__[0]
        for t in np.linspace(0, p, 50):
            R = fn(t)[:3, :3]
            worst = max(worst, float(np.abs(R @ R.T - np.eye(3)).max()),
                        abs(np.linalg.det(R) - 1.0))
    record_result("TR1 valid SE(3)", f"max orthonormality/det error = {worst:.2e}", worst < 1e-9)


def tr2_continuity() -> None:
    dt = 1.0 / HZ
    worst_p, worst_r = 0.0, 0.0
    for name, fn in TRAJECTORIES.items():
        p = fn.__defaults__[0]
        for t in np.arange(0, p, dt):
            A, B = fn(t), fn(t + dt)
            worst_p = max(worst_p, float(np.linalg.norm(B[:3, 3] - A[:3, 3])))
            worst_r = max(worst_r, rotvec_between(A[:3, :3], B[:3, :3]))
    # At 50 Hz a smooth path in a 0.12 m region moves << 1 cm and << 5 deg/frame.
    record_result("TR2 continuity",
                  f"max per-frame {worst_p * 1e3:.2f} mm / {np.rad2deg(worst_r):.3f} deg",
                  worst_p < 0.01 and worst_r < np.deg2rad(5))


def tr3_periodicity() -> None:
    worst = 0.0
    for name, fn in TRAJECTORIES.items():
        p = fn.__defaults__[0]
        A, B = fn(0.0), fn(p)
        worst = max(worst, float(np.linalg.norm(A[:3, 3] - B[:3, 3])),
                    rotvec_between(A[:3, :3], B[:3, :3]))
    record_result("TR3 periodicity", f"max |pose(0) - pose(period)| = {worst:.2e}", worst < 1e-9)


def tr4_reachability(model: FrankaModel) -> None:
    solver = DLSSolver(model)
    all_ok = True
    details = []
    for name, fn in TRAJECTORIES.items():
        p = fn.__defaults__[0]
        ok = 0
        samples = [fn(p * k / 12) for k in range(12)]
        for T in samples:
            r = solver.solve(Q_HOME, T)
            if r.pos_err < BAR_POS and r.rot_err < BAR_ROT:
                ok += 1
        all_ok &= ok == len(samples)
        details.append(f"{name} {ok}/{len(samples)}")
    record_result("TR4 reachability", "; ".join(details), all_ok)


# ------------------------------------------------------------------ pipeline ---

def tp1_tp2_tracking(model: FrankaModel) -> None:
    for name, fn in TRAJECTORIES.items():
        p = fn.__defaults__[0]
        solver = DLSSolver(model, IKConfig())
        q = solver.solve(Q_HOME, fn(0.0)).q
        qs, pos_errs, rot_errs = [q.copy()], [], []
        for k in range(1, int(2 * p * HZ) + 1):
            T = fn(k / HZ)
            q = solver.solve_step(q, T)
            pe, re = solver.error(q, T)
            qs.append(q.copy())
            pos_errs.append(pe)
            rot_errs.append(re)
        qs = np.array(qs)
        pos_errs, rot_errs = np.array(pos_errs), np.array(rot_errs)
        ok = pos_errs.max() < BAR_POS and rot_errs.max() < BAR_ROT
        record_result(f"TP1 track {name}",
                      f"max {pos_errs.max() * 1e3:.3f} mm / {np.rad2deg(rot_errs.max()):.3f} deg, "
                      f"RMS {np.sqrt((pos_errs ** 2).mean()) * 1e3:.3f} mm", ok)

        deltas = np.abs(np.diff(qs, axis=0)).max()
        legal = bool(np.all(qs >= model.lower - 1e-12) and np.all(qs <= model.upper + 1e-12))
        record_result(f"TP2 smoothness {name}",
                      f"max per-frame |dq| = {deltas:.4f} rad, in-limits={legal}",
                      deltas <= IKConfig().max_step + 1e-12 and legal)


def tp3_recorder() -> None:
    rows = record.run("circle", periods=1.0)
    tmp = REPO_ROOT / "outputs"
    tmp.mkdir(exist_ok=True)
    csv_path, png_path = tmp / "tracking_circle.csv", tmp / "tracking_circle.png"
    record.write_csv(rows, csv_path)
    record.plot(rows, "circle", png_path)
    ok = (csv_path.exists() and png_path.exists() and rows.shape[1] == 9
          and png_path.stat().st_size > 1000)
    record_result("TP3 recorder", f"CSV {rows.shape[0]} rows, PNG {png_path.stat().st_size // 1000} KB", ok)


def main() -> None:
    model = FrankaModel()
    print("Isolated trajectory tests")
    tr1_valid_se3()
    tr2_continuity()
    tr3_periodicity()
    tr4_reachability(model)

    print("\nPipeline tests")
    tp1_tp2_tracking(model)
    tp3_recorder()

    print("\n" + "=" * 72)
    n_pass = sum(1 for r in results if r[2])
    for name, _, ok in results:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
    print(f"{n_pass}/{len(results)} checks passed")
    if n_pass != len(results):
        sys.exit(1)


if __name__ == "__main__":
    main()
