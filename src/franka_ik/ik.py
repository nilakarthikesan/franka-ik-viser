"""Damped least-squares (DLS) differential inverse kinematics.

Derivation and tuning discussion: notes/stage-3-notes.md.

One iteration, given the current configuration q and a desired pose T_des:

    e   = pose_error(fk(q), T_des)              # 6D world-frame error
    Jw  = W J(q),  ew = W e                     # row weights: position vs rotation
    dq  = Jw^T (Jw Jw^T + lambda^2 I)^-1 ew     # damped least squares
    dq += (I - Jw^+ Jw) k_posture (q_home - q)  # nullspace posture task (7-DoF slack)
    dq  = alpha * dq, clipped to max_step
    q   = clamp(q + dq)                         # joint limits

Two refinements beyond the textbook update, both motivated by measured failures
(notes/stage-3-notes.md §8):
  - joints that would leave their limits are clamped to the boundary and locked,
    then the step is re-solved for the remaining joints against the residual
    error. Plain clipping instead lets one saturated joint stall the whole solve.
  - the nullspace projector uses the true (SVD) pseudoinverse, not the damped
    one, so the posture task does not leak into the end-effector task.

Error convention (must match the Jacobian's — see stage-2 notes §3): world-frame
and decoupled, e = [p_des - p_cur ; rotvec(R_des R_cur^T)], NOT the body-frame
SE(3) log6. Mixing the two is the classic differential-IK bug.
"""

from dataclasses import dataclass

import numpy as np

from .robot_model import FrankaModel


@dataclass
class IKConfig:
    damping: float = 1e-2        # lambda: singularity robustness vs. tracking lag
    step_scale: float = 1.0      # alpha: convergence speed vs. overshoot
    pos_weight: float = 1.0      # w_p (metres)
    rot_weight: float = 1.0      # w_r (radians)
    posture_gain: float = 0.05   # nullspace pull toward q_home (0 disables)
    max_step: float = 0.1        # rad, per-joint cap on |dq| (0 disables)
    pos_tol: float = 1e-4        # m, convergence bar (0.1 mm; success bar is 1 mm)
    rot_tol: float = 1e-3        # rad, convergence bar (0.057 deg; bar is 0.5 deg)
    max_iters: int = 200
    restarts: int = 12           # static solve only: retries from random seeds
    seed: int = 0                # restart RNG seed (keeps solves reproducible)


@dataclass
class SolveResult:
    q: np.ndarray
    converged: bool
    iters: int
    pos_err: float   # m
    rot_err: float   # rad


class DLSSolver:
    def __init__(self, model: FrankaModel, config: IKConfig | None = None):
        self.model = model
        self.config = config or IKConfig()

    # -- helpers -------------------------------------------------------------

    def _weights(self) -> np.ndarray:
        c = self.config
        return np.concatenate([np.full(3, c.pos_weight), np.full(3, c.rot_weight)])

    def error(self, q: np.ndarray, T_des: np.ndarray) -> tuple[float, float]:
        """(position error in m, rotation error in rad) at configuration q."""
        e = self.model.pose_error(self.model.fk(q), T_des)
        return float(np.linalg.norm(e[:3])), float(np.linalg.norm(e[3:]))

    def converged(self, q: np.ndarray, T_des: np.ndarray) -> bool:
        pos, rot = self.error(q, T_des)
        return pos < self.config.pos_tol and rot < self.config.rot_tol

    # -- the solver ----------------------------------------------------------

    def step_delta(self, q: np.ndarray, T_des: np.ndarray) -> np.ndarray:
        """The joint-space increment dq for one iteration (before integration).

        Exposed separately so tests can inspect step magnitudes directly.
        """
        c = self.config
        q = np.asarray(q, dtype=float)

        e = self.model.pose_error(self.model.fk(q), T_des)
        W = self._weights()
        J = self.model.jacobian(q) * W[:, None]
        ew = e * W
        lam2 = c.damping ** 2

        free = np.ones(7, dtype=bool)   # joints still allowed to move
        dq = np.zeros(7)

        # Clamped DLS: solve, clamp any joint that would leave its limits, lock
        # it, and re-solve the remainder against what error is left.
        for _ in range(7):
            Jf = J[:, free]
            residual = ew - J[:, ~free] @ dq[~free]
            A = Jf @ Jf.T + lam2 * np.eye(6)
            step = Jf.T @ np.linalg.solve(A, residual)

            if c.posture_gain != 0.0:
                # True (SVD) pseudoinverse: the damped one leaks the posture
                # task into the end-effector task (measured, stage-3 notes §8).
                projector = np.eye(free.sum()) - np.linalg.pinv(Jf, rcond=1e-6) @ Jf
                bias = c.posture_gain * (self.model.q_home - q)[free]
                step = step + projector @ bias

            dq[free] = c.step_scale * step

            # Bound joint velocity: keeps the linearization valid when the
            # target is far away or jumps (the interactive gizmo can teleport).
            if c.max_step > 0.0:
                peak = np.max(np.abs(dq))
                if peak > c.max_step:
                    dq = dq * (c.max_step / peak)

            q_next = q + dq
            violating = free & ((q_next < self.model.lower) | (q_next > self.model.upper))
            if not violating.any():
                break
            # Take the saturated joints exactly to their boundary and lock them.
            dq[violating] = (np.clip(q_next, self.model.lower, self.model.upper) - q)[violating]
            free &= ~violating
            if not free.any():
                break
        return dq

    def solve_step(self, q: np.ndarray, T_des: np.ndarray) -> np.ndarray:
        """One DLS iteration — used per frame when tracking a moving target."""
        return self.model.clamp(np.asarray(q, dtype=float) + self.step_delta(q, T_des))

    def _solve_from(self, q0: np.ndarray, T_des: np.ndarray) -> SolveResult:
        c = self.config
        q = np.array(q0, dtype=float, copy=True)
        for i in range(c.max_iters):
            pos, rot = self.error(q, T_des)
            if pos < c.pos_tol and rot < c.rot_tol:
                return SolveResult(q, True, i, pos, rot)
            q = self.solve_step(q, T_des)
        pos, rot = self.error(q, T_des)
        return SolveResult(q, pos < c.pos_tol and rot < c.rot_tol, c.max_iters, pos, rot)

    def solve(self, q0: np.ndarray, T_des: np.ndarray) -> SolveResult:
        """Iterate to convergence for a static target.

        Differential IK is a local method, so a start configuration can sit in a
        basin that no descent direction escapes (typically with a joint pinned at
        a limit). For static targets we simply retry from random configurations;
        the RNG is seeded per call, so results stay reproducible. Tracking never
        restarts — it must stay continuous.
        """
        best = self._solve_from(q0, T_des)
        if best.converged:
            return best
        rng = np.random.default_rng(self.config.seed)
        total = best.iters
        for _ in range(self.config.restarts):
            attempt = self._solve_from(rng.uniform(self.model.lower, self.model.upper), T_des)
            total += attempt.iters
            if attempt.converged:
                return SolveResult(attempt.q, True, total, attempt.pos_err, attempt.rot_err)
            if attempt.pos_err + attempt.rot_err < best.pos_err + best.rot_err:
                best = attempt
        return SolveResult(best.q, False, total, best.pos_err, best.rot_err)
