"""Damped least-squares differential IK.

Core update (see DESIGN.md §1):
    e  = log6(T_ee^-1 * T_des)                       # 6D pose error twist
    dq = J^T (J J^T + lambda^2 I)^-1 e               # DLS step
    dq += (I - J^+ J) k_posture (q_home - q)         # optional nullspace posture
    q  = clip(q + alpha * dq, q_lower, q_upper)
"""

from dataclasses import dataclass

import numpy as np

from .robot_model import FrankaModel, Q_HOME


@dataclass
class IKConfig:
    damping: float = 1e-2        # lambda
    step_scale: float = 1.0      # alpha
    pos_weight: float = 1.0
    rot_weight: float = 1.0
    posture_gain: float = 0.05   # nullspace pull toward Q_HOME (0 disables)
    tol: float = 1e-4            # converged when ||e|| below this
    max_iters: int = 200


class DLSSolver:
    def __init__(self, model: FrankaModel, config: IKConfig | None = None):
        self.model = model
        self.config = config or IKConfig()

    def solve_step(self, q: np.ndarray, T_des) -> np.ndarray:
        """One DLS iteration — used per-frame when tracking a moving target."""
        raise NotImplementedError  # TODO(M3)

    def solve(self, q0: np.ndarray, T_des) -> tuple[np.ndarray, bool]:
        """Iterate solve_step until convergence for a static target.

        Returns (q_solution, converged).
        """
        raise NotImplementedError  # TODO(M3)
