"""M3 gate: test the DLS solver in isolation and inside the pipeline.

Test plan and rationale: notes/stage-3-notes.md §7.

  Isolated (does the algorithm behave?)
    T1 fixed point at the solution          T5 singularity robustness
    T2 static convergence (< 1 mm, < 0.5)   T6 nullspace posture is task-neutral
    T3 joint limits always respected        T7 unreachable target degrades gracefully
    T4 per-step magnitude bounded           T8 determinism

  Pipeline (does it work with the rest of the design?)
    P1 tracking a circular SE(3) path at 50 Hz (M4 preview)
    P2 smoothness of the tracked joint sequence
    P4 tuning sweep over damping x step scale
  (P3 = re-running the Stage 1/2 scripts, driven from the shell.)

Usage:  .venv/bin/python scripts/test_ik.py
Exit code is nonzero if any check fails, so this can gate CI.
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from franka_ik.ik import DLSSolver, IKConfig
from franka_ik.robot_model import Q_HOME, FrankaModel

RNG = np.random.default_rng(0)
N_TARGETS = 50
HZ = 50.0

# Acceptance bars from DESIGN.md §1.
BAR_POS = 1e-3      # m   (1 mm)
BAR_ROT = np.deg2rad(0.5)

results: list[tuple[str, str, bool]] = []


def record(name: str, detail: str, passed: bool) -> None:
    results.append((name, detail, passed))
    print(f"  [{'PASS' if passed else 'FAIL'}] {name}: {detail}")


def pose(R: np.ndarray, p: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def random_reachable_targets(model: FrankaModel, n: int) -> tuple[np.ndarray, np.ndarray]:
    """Targets generated as fk(q) so reachability is guaranteed by construction."""
    qs = RNG.uniform(model.lower, model.upper, size=(n, 7))
    return qs, np.array([model.fk(q) for q in qs])


# ------------------------------------------------------------------ isolated ---

def t1_fixed_point(model: FrankaModel) -> None:
    solver = DLSSolver(model, IKConfig(posture_gain=0.0))
    peaks = []
    for q in RNG.uniform(model.lower, model.upper, size=(20, 7)):
        peaks.append(np.max(np.abs(solver.step_delta(q, model.fk(q)))))
    record("T1 fixed point", f"max |dq| at solution = {max(peaks):.2e} rad", max(peaks) < 1e-9)


def t2_static_convergence(model: FrankaModel) -> None:
    solver = DLSSolver(model)
    _, targets = random_reachable_targets(model, N_TARGETS)

    for label, starts in (
        ("from q_home", [Q_HOME] * N_TARGETS),
        ("from random q", list(RNG.uniform(model.lower, model.upper, size=(N_TARGETS, 7)))),
    ):
        ok, iters, pos_errs, rot_errs = 0, [], [], []
        for q0, T in zip(starts, targets):
            r = solver.solve(q0, T)
            if r.pos_err < BAR_POS and r.rot_err < BAR_ROT:
                ok += 1
                iters.append(r.iters)
            pos_errs.append(r.pos_err)
            rot_errs.append(r.rot_err)
        detail = (f"{ok}/{N_TARGETS} within 1 mm / 0.5 deg; "
                  f"median {int(np.median(iters))} iters; "
                  f"median err {np.median(pos_errs) * 1e3:.2e} mm / "
                  f"{np.rad2deg(np.median(rot_errs)):.2e} deg")
        record(f"T2 static convergence ({label})", detail, ok == N_TARGETS)


def t3_t4_limits_and_step_bound(model: FrankaModel) -> None:
    cfg = IKConfig()
    solver = DLSSolver(model, cfg)
    _, targets = random_reachable_targets(model, 20)
    worst_violation, worst_step = 0.0, 0.0
    for T in targets:
        q = Q_HOME.copy()
        for _ in range(cfg.max_iters):
            dq = solver.step_delta(q, T)
            worst_step = max(worst_step, float(np.max(np.abs(dq))))
            q = model.clamp(q + dq)
            worst_violation = max(
                worst_violation,
                float(np.max(np.maximum(model.lower - q, 0.0))),
                float(np.max(np.maximum(q - model.upper, 0.0))),
            )
    record("T3 joint limits", f"max violation = {worst_violation:.2e} rad", worst_violation == 0.0)
    record("T4 step bound", f"max |dq| = {worst_step:.4f} <= max_step {cfg.max_step}",
           worst_step <= cfg.max_step + 1e-12)


def t5_singularity(model: FrankaModel) -> None:
    """Start nearly straight (elbow at its limit) and demand motion outward:
    the direction the arm cannot move is exactly what damping must survive."""
    q_stretched = model.clamp(np.zeros(7))
    T_cur = model.fk(q_stretched)
    T_far = pose(T_cur[:3, :3], T_cur[:3, 3] + np.array([0.4, 0.0, 0.0]))

    rows = []
    all_finite = True
    for lam in (1e-4, 1e-2, 1e-1):
        solver = DLSSolver(model, IKConfig(damping=lam, posture_gain=0.0))
        q, peak = q_stretched.copy(), 0.0
        for _ in range(200):
            dq = solver.step_delta(q, T_far)
            peak = max(peak, float(np.max(np.abs(dq))))
            q = model.clamp(q + dq)
        finite = bool(np.all(np.isfinite(q)))
        all_finite &= finite
        pos, _ = solver.error(q, T_far)
        rows.append(f"lam={lam:g}: peak |dq|={peak:.3f}, finite={finite}, residual={pos * 1e3:.0f} mm")
    record("T5 singularity robustness", "; ".join(rows), all_finite)


def t6_nullspace_task_neutral(model: FrankaModel) -> None:
    """With the task converged, the posture term must pull q toward q_home
    while leaving the end-effector pose within tolerance."""
    solver_task = DLSSolver(model, IKConfig(posture_gain=0.0))
    _, targets = random_reachable_targets(model, 10)
    drifted, held = 0, 0
    for T in targets:
        r = solver_task.solve(Q_HOME, T)
        if not r.converged:
            continue
        q = r.q.copy()
        d0 = float(np.linalg.norm(q - Q_HOME))
        solver_null = DLSSolver(model, IKConfig(posture_gain=0.05))
        for _ in range(200):
            q = solver_null.solve_step(q, T)
        d1 = float(np.linalg.norm(q - Q_HOME))
        pos, rot = solver_null.error(q, T)
        if d1 < d0:
            drifted += 1
        if pos < BAR_POS and rot < BAR_ROT:
            held += 1
    record("T6 nullspace posture", f"{drifted}/10 moved toward q_home, "
           f"{held}/10 kept pose within 1 mm / 0.5 deg", drifted >= 9 and held == 10)


def t7_unreachable(model: FrankaModel) -> None:
    solver = DLSSolver(model, IKConfig(max_iters=300))
    T_far = pose(model.fk(Q_HOME)[:3, :3], np.array([2.0, 0.0, 0.5]))
    r = solver.solve(Q_HOME, T_far)
    legal = bool(np.all(r.q >= model.lower - 1e-12) and np.all(r.q <= model.upper + 1e-12))
    finite = bool(np.all(np.isfinite(r.q)))
    record("T7 unreachable target", f"no convergence (expected: {not r.converged}), "
           f"residual {r.pos_err:.3f} m, finite={finite}, in-limits={legal}",
           (not r.converged) and finite and legal)


def t8_determinism(model: FrankaModel) -> None:
    solver = DLSSolver(model)
    _, targets = random_reachable_targets(model, 5)
    same = all(np.array_equal(solver.solve(Q_HOME, T).q, solver.solve(Q_HOME, T).q)
               for T in targets)
    record("T8 determinism", "repeated solves identical" if same else "outputs differ", same)


# ------------------------------------------------------------------ pipeline ---

def circle_path(model: FrankaModel, t: float, period: float = 8.0,
                radius: float = 0.12) -> np.ndarray:
    """Circle in the YZ plane, centred in front of the base (DESIGN.md §3.3);
    orientation fixed to the home pose's. M4 formalizes this in trajectory.py."""
    center = np.array([0.45, 0.0, 0.45])
    ang = 2.0 * np.pi * t / period
    p = center + np.array([0.0, radius * np.cos(ang), radius * np.sin(ang)])
    return pose(model.fk(Q_HOME)[:3, :3], p)


def track(model: FrankaModel, cfg: IKConfig, seconds: float = 8.0):
    """One solver step per frame at 50 Hz, after a static solve onto frame 0."""
    solver = DLSSolver(model, cfg)
    q = solver.solve(Q_HOME, circle_path(model, 0.0)).q
    qs, pos_errs, rot_errs = [q.copy()], [], []
    for k in range(1, int(seconds * HZ) + 1):
        T = circle_path(model, k / HZ)
        q = solver.solve_step(q, T)
        pos, rot = solver.error(q, T)
        qs.append(q.copy())
        pos_errs.append(pos)
        rot_errs.append(rot)
    return np.array(qs), np.array(pos_errs), np.array(rot_errs)


def p1_p2_tracking(model: FrankaModel) -> None:
    cfg = IKConfig()
    qs, pos_errs, rot_errs = track(model, cfg)
    detail = (f"max {pos_errs.max() * 1e3:.3f} mm / {np.rad2deg(rot_errs.max()):.3f} deg, "
              f"RMS {np.sqrt((pos_errs ** 2).mean()) * 1e3:.3f} mm / "
              f"{np.rad2deg(np.sqrt((rot_errs ** 2).mean())):.3f} deg")
    record("P1 circle tracking (8 s @ 50 Hz)", detail,
           pos_errs.max() < BAR_POS and rot_errs.max() < BAR_ROT)

    deltas = np.abs(np.diff(qs, axis=0)).max(axis=1)
    legal = bool(np.all(qs >= model.lower - 1e-12) and np.all(qs <= model.upper + 1e-12))
    record("P2 smoothness", f"max per-frame |dq| = {deltas.max():.4f} rad "
           f"(<= max_step {cfg.max_step}), in-limits={legal}",
           deltas.max() <= cfg.max_step + 1e-12 and legal)


def p4_tuning_sweep(model: FrankaModel) -> None:
    print("\nP4 tuning sweep (max tracking error on the circle task):")
    print(f"    {'lambda':>8} {'alpha':>6} {'max pos':>12} {'max rot':>12}")
    best = None
    for lam in (1e-3, 1e-2, 5e-2, 1e-1, 3e-1):
        for alpha in (0.5, 1.0):
            _, pos_errs, rot_errs = track(model, IKConfig(damping=lam, step_scale=alpha))
            mp, mr = pos_errs.max(), rot_errs.max()
            print(f"    {lam:>8g} {alpha:>6g} {mp * 1e3:>9.4f} mm {np.rad2deg(mr):>9.4f} deg")
            if best is None or mp < best[0]:
                best = (mp, lam, alpha)
    record("P4 defaults justified",
           f"best lambda={best[1]:g}, alpha={best[2]:g} at {best[0] * 1e3:.4f} mm; "
           f"defaults (1e-2, 1.0) within an order of magnitude",
           best[0] < BAR_POS)


def main() -> None:
    model = FrankaModel()
    print("Isolated solver tests")
    t1_fixed_point(model)
    t2_static_convergence(model)
    t3_t4_limits_and_step_bound(model)
    t5_singularity(model)
    t6_nullspace_task_neutral(model)
    t7_unreachable(model)
    t8_determinism(model)

    print("\nPipeline tests")
    p1_p2_tracking(model)
    p4_tuning_sweep(model)

    print("\n" + "=" * 72)
    n_pass = sum(1 for r in results if r[2])
    for name, detail, ok in results:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
    print(f"{n_pass}/{len(results)} checks passed")
    if n_pass != len(results):
        sys.exit(1)


if __name__ == "__main__":
    main()
