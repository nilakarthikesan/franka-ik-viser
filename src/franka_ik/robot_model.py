"""FR3 kinematic model: from-scratch FK and geometric Jacobian (M2).

URDF *parsing* is delegated to yourdfpy (trusted, validated in Stage 1); all
runtime *math* — forward kinematics, the geometric Jacobian, rotation logs,
pose error — is plain numpy implemented here. Derivations and the validation
plan live in notes/stage-2-notes.md.

Conventions (the IK solver's error term must match — DESIGN.md §2):
  - world-frame (spatial) geometric Jacobian, rows ordered [linear; angular]
  - pose_error(T_cur, T_des) = [p_des - p_cur; rotvec(R_des @ R_cur.T)]

At construction the base→TCP chain is compiled down to, per arm joint i, a
constant 4x4 prefix A_i (all fixed transforms since the previous joint,
including joint i's <origin>) and a unit axis a_i, plus one constant suffix
(joint-7 flange → fr3_hand_tcp through three fixed joints). FK is then seven
4x4 products; the Jacobian reads off the same chain walk.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yourdfpy

DEFAULT_URDF = Path(__file__).parents[2] / "assets" / "franka_description" / "urdfs" / "fr3_franka_hand.urdf"
EE_FRAME = "fr3_hand_tcp"

# Comfortable home configuration used as the nullspace posture target.
Q_HOME = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])


# ----------------------------------------------------------------------------
# Rotation helpers (pure functions, individually validated in check E)
# ----------------------------------------------------------------------------

def skew(v: np.ndarray) -> np.ndarray:
    """3x3 cross-product matrix: skew(v) @ u == np.cross(v, u)."""
    return np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0],
    ])


def rotation_about_axis(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues' formula: 3x3 rotation by `angle` about unit vector `axis`."""
    K = skew(axis)
    return np.eye(3) + np.sin(angle) * K + (1.0 - np.cos(angle)) * (K @ K)


def rotvec_from_matrix(R: np.ndarray) -> np.ndarray:
    """Rotation vector (axis * angle) of a rotation matrix — the SO(3) log map.

    The naive formula theta/(2 sin theta) * vee(R - R^T) is ill-conditioned at
    both ends of theta = arccos((tr R - 1) / 2), and the bad window around pi
    is wide (catastrophic cancellation in R - R^T as sin theta -> 0). Branches:
      - theta -> 0: vee(R - R^T)/2 is the rotvec to double precision.
      - theta > 3 rad: recover the axis from the *symmetric* part,
        (R + R^T)/2 - cos(theta) I = (1 - cos theta) aa^T, which stays
        well-conditioned near pi; recover the angle as pi - arcsin(|vee|),
        which is well-conditioned exactly where arccos is not.
      - otherwise: the standard formula (sin theta >= 0.14, safe).
    """
    cos_theta = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    theta = np.arccos(cos_theta)
    vee = 0.5 * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])

    if theta < 1e-7:
        # rotvec = vee * (1 + theta^2/6 + ...) ~= vee to double precision here.
        return vee
    if theta > 3.0:
        # aa^T, exact up to roundoff for any theta bounded away from 0.
        M = (0.5 * (R + R.T) - cos_theta * np.eye(3)) / (1.0 - cos_theta)
        k = int(np.argmax(np.diag(M)))
        axis = M[:, k] / np.sqrt(max(M[k, k], 1e-300))
        axis /= np.linalg.norm(axis)
        # sin(theta) = |vee| >= 0 for theta in (0, pi]: sign the axis by vee.
        # At exactly pi, vee = 0 and either sign is a correct log.
        if np.dot(axis, vee) < 0.0:
            axis = -axis
        theta = np.pi - np.arcsin(np.clip(np.linalg.norm(vee), 0.0, 1.0))
        return theta * axis
    return (theta / np.sin(theta)) * vee


# ----------------------------------------------------------------------------
# Robot model
# ----------------------------------------------------------------------------

@dataclass
class FKResult:
    """FK output plus the per-joint world-frame quantities the Jacobian needs."""

    T_tcp: np.ndarray          # (4,4) world -> TCP pose
    joint_positions: np.ndarray  # (7,3) world position p_i of each joint origin
    joint_axes: np.ndarray       # (7,3) world direction z_i of each joint axis


class FrankaModel:
    """7-DoF FR3 arm kinematics compiled from the URDF at construction."""

    def __init__(self, urdf_path: Path = DEFAULT_URDF):
        self.urdf_path = Path(urdf_path)
        urdf = yourdfpy.URDF.load(self.urdf_path, load_meshes=False)

        # Walk child -> parent from the EE frame up to the root, then reverse
        # to get the base -> TCP joint chain in order.
        joints_by_child = {j.child: j for j in urdf.robot.joints}
        chain = []
        link = EE_FRAME
        while link in joints_by_child:
            joint = joints_by_child[link]
            chain.append(joint)
            link = joint.parent
        chain.reverse()
        assert link == urdf.base_link, f"chain root {link} != base {urdf.base_link}"

        prefixes: list[np.ndarray] = []
        axes: list[np.ndarray] = []
        names: list[str] = []
        lower: list[float] = []
        upper: list[float] = []

        A = np.eye(4)
        for joint in chain:
            origin = joint.origin if joint.origin is not None else np.eye(4)
            A = A @ origin
            if joint.type == "fixed":
                continue
            if joint.type != "revolute":
                raise ValueError(f"unexpected joint type on base->TCP path: {joint.type}")
            axis = np.asarray(joint.axis, dtype=float) if joint.axis is not None else np.array([1.0, 0.0, 0.0])
            prefixes.append(A)
            axes.append(axis / np.linalg.norm(axis))
            names.append(joint.name)
            lower.append(float(joint.limit.lower))
            upper.append(float(joint.limit.upper))
            A = np.eye(4)

        assert len(prefixes) == 7, f"expected 7 revolute joints on chain, got {len(prefixes)}"
        self.prefixes = prefixes
        self.axes = np.array(axes)
        self.suffix = A  # joint-7 flange -> TCP (three fixed joints folded in)
        self.joint_names = names
        self.lower = np.array(lower)
        self.upper = np.array(upper)
        self.q_home = Q_HOME.copy()

    # -- kinematics ----------------------------------------------------------

    def fk_all(self, q: np.ndarray) -> FKResult:
        """One chain walk: TCP pose plus each joint's world position and axis."""
        q = np.asarray(q, dtype=float)
        assert q.shape == (7,)
        T = np.eye(4)
        ps = np.empty((7, 3))
        zs = np.empty((7, 3))
        for i in range(7):
            T = T @ self.prefixes[i]
            # Joint position/axis are unaffected by the joint's own rotation
            # (a rotation about an axis fixes that axis), so read them here.
            ps[i] = T[:3, 3]
            zs[i] = T[:3, :3] @ self.axes[i]
            R_joint = np.eye(4)
            R_joint[:3, :3] = rotation_about_axis(self.axes[i], q[i])
            T = T @ R_joint
        T = T @ self.suffix
        return FKResult(T_tcp=T, joint_positions=ps, joint_axes=zs)

    def fk(self, q: np.ndarray) -> np.ndarray:
        """World -> TCP pose as a 4x4 homogeneous transform."""
        return self.fk_all(q).T_tcp

    def jacobian(self, q: np.ndarray) -> np.ndarray:
        """(6,7) world-frame geometric Jacobian at the TCP.

        Column i (revolute): [ z_i x (p_tcp - p_i) ; z_i ].
        """
        res = self.fk_all(q)
        p_tcp = res.T_tcp[:3, 3]
        J = np.empty((6, 7))
        for i in range(7):
            J[:3, i] = np.cross(res.joint_axes[i], p_tcp - res.joint_positions[i])
            J[3:, i] = res.joint_axes[i]
        return J

    # -- error / limits ------------------------------------------------------

    @staticmethod
    def pose_error(T_cur: np.ndarray, T_des: np.ndarray) -> np.ndarray:
        """6D world-frame error twist [linear; angular] from current to desired.

        Pairs with the world-frame geometric Jacobian above; see the
        convention discussion in notes/stage-2-notes.md §3.
        """
        e = np.empty(6)
        e[:3] = T_des[:3, 3] - T_cur[:3, 3]
        e[3:] = rotvec_from_matrix(T_des[:3, :3] @ T_cur[:3, :3].T)
        return e

    def clamp(self, q: np.ndarray) -> np.ndarray:
        """Clip a configuration to the URDF joint limits."""
        return np.clip(q, self.lower, self.upper)
