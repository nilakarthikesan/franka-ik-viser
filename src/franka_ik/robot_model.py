"""FR3 kinematic model wrapper around Pinocchio.

Loads the URDF generated from frankarobotics/franka_description, locks the
gripper finger joints so the model is exactly the 7-DoF arm, and exposes the
quantities the IK solver needs.
"""

from dataclasses import dataclass
from pathlib import Path

import numpy as np

# TODO(M2): import pinocchio as pin

DEFAULT_URDF = Path(__file__).parents[2] / "assets" / "franka_description" / "urdfs" / "fr3_franka_hand.urdf"
EE_FRAME = "fr3_hand_tcp"

# Comfortable home configuration used as the nullspace posture target.
Q_HOME = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])


@dataclass
class FrankaModel:
    """7-DoF FR3 arm kinematics.

    TODO(M2):
      - build pin model from URDF, lock finger joints
      - fk(q) -> 4x4 EE pose (or pin.SE3)
      - jacobian(q) -> (6, 7) geometric Jacobian at EE frame
      - joint limits (lower, upper) arrays
      - pose_error(T_current, T_target) -> 6D twist via pin.log6
    """

    urdf_path: Path = DEFAULT_URDF

    def fk(self, q: np.ndarray):
        raise NotImplementedError

    def jacobian(self, q: np.ndarray) -> np.ndarray:
        raise NotImplementedError
