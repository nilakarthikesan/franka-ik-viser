"""M2 gate: prove the from-scratch FK and Jacobian correct (stage-2 notes §4).

Five independent checks, each attacking a different failure mode:

  E  rotation helpers vs scipy.spatial.transform.Rotation
     (edge cases: identity, tiny angles, near/at pi, random)
  A  our fk() vs yourdfpy get_transform      — our chain-walk math
     (same URDF parse as Stage 1's verify_setup.py: integration with Stage 1)
  B  our fk() vs PyRoKi forward_kinematics   — independent parser AND math
  C  our jacobian() vs central finite differences of our own fk()
  D  our jacobian() vs jax.jacfwd through PyRoKi FK (independent analytic)
  F  pose_error vs jacobian first-order consistency (the IK solver's contract)

Configurations tested: N random samples uniform within joint limits, plus the
structured edge set {q_home, lower limits, upper limits, mid-range, and
random limit corners} where the arm is folded/stretched near singularities.

Exit code is nonzero if any check exceeds tolerance, so this can gate CI.
Usage:  .venv/bin/python scripts/validate_kinematics.py
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import jax

jax.config.update("jax_enable_x64", True)  # float64 to make 1e-6 tolerances meaningful

import jax.numpy as jnp
import pyroki as pk
import yourdfpy
from scipy.spatial.transform import Rotation

from franka_ik.robot_model import (
    DEFAULT_URDF,
    EE_FRAME,
    Q_HOME,
    FrankaModel,
    rotation_about_axis,
    rotvec_from_matrix,
)

N_RANDOM = 100
RNG = np.random.default_rng(0)

TOL_POS = 1e-6   # m       (checks A, B)
TOL_ROT = 1e-6   # rad     (checks A, B)
TOL_JAC = 1e-5   # (checks C, D)
FD_STEP = 1e-6

results: list[tuple[str, float, float, bool]] = []  # (name, max_err, tol, passed)


def record(name: str, max_err: float, tol: float) -> None:
    results.append((name, max_err, tol, max_err < tol))
    status = "PASS" if max_err < tol else "FAIL"
    print(f"  [{status}] {name}: max err {max_err:.3e} (tol {tol:.0e})")


def geodesic(R_a: np.ndarray, R_b: np.ndarray) -> float:
    """Rotation distance in radians, immune to quaternion sign issues."""
    return float(np.linalg.norm(rotvec_from_matrix(R_a.T @ R_b)))


def test_configs(model: FrankaModel) -> np.ndarray:
    lo, hi = model.lower, model.upper
    qs = [Q_HOME, lo, hi, (lo + hi) / 2.0]
    for _ in range(8):  # random limit corners: folded/stretched near singular
        mask = RNG.integers(0, 2, size=7).astype(bool)
        qs.append(np.where(mask, lo, hi))
    qs += list(RNG.uniform(lo, hi, size=(N_RANDOM, 7)))
    return np.array(qs)


# ---------------------------------------------------------------- check E ---

def check_e_rotation_helpers() -> None:
    print("\nCheck E: rotation helpers vs scipy")
    # Rodrigues formula vs scipy, random axes/angles including huge ones.
    errs = []
    for _ in range(200):
        axis = RNG.normal(size=3)
        axis /= np.linalg.norm(axis)
        angle = RNG.uniform(-2 * np.pi, 2 * np.pi)
        R_ours = rotation_about_axis(axis, angle)
        R_ref = Rotation.from_rotvec(axis * angle).as_matrix()
        errs.append(np.abs(R_ours - R_ref).max())
    record("E1 rotation_about_axis vs scipy", max(errs), 1e-12)

    # Log map vs scipy across the full range plus degenerate edges.
    cases = [np.zeros(3)]  # identity
    for eps in (1e-12, 1e-9, 1e-7, 1e-4):  # tiny angles (Taylor branch)
        cases.append(np.array([eps, 0.0, 0.0]))
    for ax in (np.eye(3)):  # exactly pi about coordinate axes (pi branch)
        cases.append(np.pi * ax)
    for _ in range(20):  # near pi about random axes
        axis = RNG.normal(size=3)
        axis /= np.linalg.norm(axis)
        cases.append(axis * (np.pi - 10.0 ** RNG.uniform(-9, -2)))
    for _ in range(200):  # generic
        cases.append(Rotation.random(rng=RNG).as_rotvec())
    errs = []
    for rv in cases:
        R = Rotation.from_rotvec(rv).as_matrix()
        ours = rotvec_from_matrix(R)
        ref = Rotation.from_matrix(R).as_rotvec()
        # Compare as rotations (at exactly pi, +axis and -axis are both right).
        d = Rotation.from_rotvec(ours).inv() * Rotation.from_rotvec(ref)
        errs.append(d.magnitude())
    record("E2 rotvec_from_matrix vs scipy", max(errs), 1e-9)


# ---------------------------------------------------------------- check A ---

def check_a_fk_vs_yourdfpy(model: FrankaModel, qs: np.ndarray) -> None:
    print("\nCheck A: our FK vs yourdfpy get_transform (Stage 1 integration)")
    urdf = yourdfpy.URDF.load(DEFAULT_URDF, load_meshes=False)
    pos_errs, rot_errs = [], []
    for q in qs:
        urdf.update_cfg({name: q[i] for i, name in enumerate(model.joint_names)})
        T_ref = urdf.get_transform(EE_FRAME, urdf.base_link)
        T_ours = model.fk(q)
        pos_errs.append(np.linalg.norm(T_ours[:3, 3] - T_ref[:3, 3]))
        rot_errs.append(geodesic(T_ours[:3, :3], T_ref[:3, :3]))
    record("A position", max(pos_errs), TOL_POS)
    record("A rotation", max(rot_errs), TOL_ROT)


# ---------------------------------------------------------------- check B ---

def _wxyz_xyz_to_T(wxyz_xyz: np.ndarray) -> np.ndarray:
    w, x, y, z = wxyz_xyz[:4]
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([x, y, z, w]).as_matrix()  # scipy wants xyzw
    T[:3, 3] = wxyz_xyz[4:]
    return T


def make_pyroki(model: FrankaModel):
    urdf = yourdfpy.URDF.load(DEFAULT_URDF, load_meshes=False)
    robot = pk.Robot.from_urdf(urdf)
    actuated = list(robot.joints.actuated_names)
    print(f"  PyRoKi actuated joints ({len(actuated)}): {actuated}")
    # Map our 7 arm values into PyRoKi's actuated vector (finger stays 0).
    arm_idx = np.array([actuated.index(n) for n in model.joint_names])
    link_idx = list(robot.links.names).index(EE_FRAME)

    def cfg_of(q: np.ndarray) -> jnp.ndarray:
        cfg = np.zeros(len(actuated))
        cfg[arm_idx] = q
        return jnp.asarray(cfg)

    return robot, cfg_of, link_idx, arm_idx


def check_b_fk_vs_pyroki(model: FrankaModel, qs: np.ndarray, robot, cfg_of, link_idx) -> None:
    print("\nCheck B: our FK vs PyRoKi (independent parser + math)")
    cfgs = jnp.stack([cfg_of(q) for q in qs])
    poses = np.asarray(robot.forward_kinematics(cfgs))[:, link_idx, :]
    pos_errs, rot_errs = [], []
    for q, pose in zip(qs, poses):
        T_ref = _wxyz_xyz_to_T(pose)
        T_ours = model.fk(q)
        pos_errs.append(np.linalg.norm(T_ours[:3, 3] - T_ref[:3, 3]))
        rot_errs.append(geodesic(T_ours[:3, :3], T_ref[:3, :3]))
    record("B position", max(pos_errs), TOL_POS)
    record("B rotation", max(rot_errs), TOL_ROT)


# ---------------------------------------------------------------- check C ---

def check_c_jacobian_vs_fd(model: FrankaModel, qs: np.ndarray) -> None:
    print("\nCheck C: our Jacobian vs central finite differences of our FK")
    errs = []
    for q in qs:
        J = model.jacobian(q)
        J_fd = np.empty((6, 7))
        for i in range(7):
            dq = np.zeros(7)
            dq[i] = FD_STEP
            T_p, T_m = model.fk(q + dq), model.fk(q - dq)
            J_fd[:3, i] = (T_p[:3, 3] - T_m[:3, 3]) / (2 * FD_STEP)
            J_fd[3:, i] = rotvec_from_matrix(T_p[:3, :3] @ T_m[:3, :3].T) / (2 * FD_STEP)
        errs.append(np.abs(J - J_fd).max())
    record("C jacobian", max(errs), TOL_JAC)


# ---------------------------------------------------------------- check D ---

def _quat_mul_wxyz(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def check_d_jacobian_vs_pyroki(model: FrankaModel, qs: np.ndarray, robot, cfg_of, link_idx, arm_idx) -> None:
    print("\nCheck D: our Jacobian vs jax.jacfwd through PyRoKi FK")

    def tcp_pose(cfg: jnp.ndarray) -> jnp.ndarray:
        return robot.forward_kinematics(cfg)[link_idx]

    jac_fn = jax.jit(jax.jacfwd(tcp_pose))
    errs = []
    for q in qs[:20]:  # jacfwd is slower; a subset suffices for a bonus check
        cfg = cfg_of(q)
        dpose = np.asarray(jac_fn(cfg))            # (7, n_actuated): d(wxyz_xyz)/dq
        pose = np.asarray(tcp_pose(cfg))
        quat, quat_conj = pose[:4], pose[:4] * np.array([1.0, -1.0, -1.0, -1.0])
        J_ref = np.empty((6, 7))
        for col, j in enumerate(arm_idx):
            J_ref[:3, col] = dpose[4:, j]
            # omega = 2 * Im(dq/dq_i (x) q^-1), world frame, wxyz convention
            J_ref[3:, col] = 2.0 * _quat_mul_wxyz(dpose[:4, j], quat_conj)[1:]
        errs.append(np.abs(model.jacobian(q) - J_ref).max())
    record("D jacobian", max(errs), TOL_JAC)


# ---------------------------------------------------------------- check F ---

def check_f_error_jacobian_consistency(model: FrankaModel, qs: np.ndarray) -> None:
    """pose_error and jacobian must speak the same frame convention:
    e(fk(q), fk(q + d)) ~= J(q) d to first order. This is the exact contract
    the DLS solver relies on (stage-2 notes §3, 'the convention trap')."""
    print("\nCheck F: pose_error / Jacobian first-order consistency")
    errs = []
    delta_mag = 1e-5
    for q in qs:
        d = RNG.normal(size=7)
        d *= delta_mag / np.linalg.norm(d)
        e = FrankaModel.pose_error(model.fk(q), model.fk(q + d))
        errs.append(np.linalg.norm(e - model.jacobian(q) @ d) / delta_mag)
    record("F error-vs-J (relative)", max(errs), 1e-4)


# -------------------------------------------------------------------- main ---

def main() -> None:
    model = FrankaModel()
    qs = test_configs(model)
    print(f"Validating on {len(qs)} configurations "
          f"({N_RANDOM} random in-limit + {len(qs) - N_RANDOM} edge/structured)")

    check_e_rotation_helpers()
    check_a_fk_vs_yourdfpy(model, qs)
    robot, cfg_of, link_idx, arm_idx = make_pyroki(model)
    check_b_fk_vs_pyroki(model, qs, robot, cfg_of, link_idx)
    check_c_jacobian_vs_fd(model, qs)
    check_d_jacobian_vs_pyroki(model, qs, robot, cfg_of, link_idx, arm_idx)
    check_f_error_jacobian_consistency(model, qs)

    print("\n" + "=" * 60)
    n_pass = sum(1 for r in results if r[3])
    for name, err, tol, ok in results:
        print(f"  {'PASS' if ok else 'FAIL'}  {name:40s} {err:.3e} < {tol:.0e}")
    print(f"{n_pass}/{len(results)} checks passed")
    if n_pass != len(results):
        sys.exit(1)


if __name__ == "__main__":
    main()
