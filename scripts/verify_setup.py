"""Verify the M0 setup: the generated FR3 URDF contains exactly the kinematic
data the design assumes (DESIGN.md SS1, stage-1 notes SS2-3).

Checks:
  1. URDF parses with yourdfpy and its meshes resolve (relative paths work).
  2. Exactly 7 revolute arm joints (the q in R^7 from the problem statement),
     plus prismatic finger joints we will exclude from IK.
  3. Every arm joint has limits (needed for the solver's clamping step).
  4. The end-effector frame `fr3_hand_tcp` exists in the link tree.
  5. FK at the home configuration puts the hand in a sane spot (within the
     ~0.85 m reach envelope, above the table).

Usage:  .venv/bin/python scripts/verify_setup.py
"""

from pathlib import Path

import numpy as np
import yourdfpy

REPO_ROOT = Path(__file__).resolve().parents[1]
URDF_PATH = REPO_ROOT / "assets" / "franka_description" / "urdfs" / "fr3_franka_hand.urdf"
EE_FRAME = "fr3_hand_tcp"
Q_HOME = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])


def main() -> None:
    print(f"URDF: {URDF_PATH}")
    # mesh_dir anchors the URDF's relative mesh paths to its own folder,
    # independent of the caller's working directory.
    urdf = yourdfpy.URDF.load(URDF_PATH, mesh_dir=str(URDF_PATH.parent))
    n_geom = len(urdf.scene.geometry)
    assert n_geom > 0, "no mesh geometry loaded"
    print(f"[1] parsed OK: {len(urdf.link_map)} links, {len(urdf.joint_map)} joints, "
          f"{n_geom} mesh geometries resolved")

    revolute = [j for j in urdf.robot.joints if j.type == "revolute"]
    prismatic = [j for j in urdf.robot.joints if j.type == "prismatic"]
    arm_joints = [j for j in revolute if "finger" not in j.name]
    assert len(arm_joints) == 7, f"expected 7 arm joints, got {len(arm_joints)}"
    print(f"[2] 7 revolute arm joints: {[j.name for j in arm_joints]}")
    print(f"    (+{len(prismatic)} prismatic finger joints, excluded from IK)")

    print("[3] joint limits (rad):")
    for j in arm_joints:
        assert j.limit is not None and j.limit.lower is not None
        print(f"    {j.name}: [{j.limit.lower:+.4f}, {j.limit.upper:+.4f}]")

    assert EE_FRAME in urdf.link_map, f"{EE_FRAME} not in link tree"
    print(f"[4] EE frame '{EE_FRAME}' present")

    cfg = {j.name: Q_HOME[i] for i, j in enumerate(arm_joints)}
    urdf.update_cfg(cfg)
    T = urdf.get_transform(EE_FRAME, urdf.base_link)
    p = T[:3, 3]
    reach = np.linalg.norm(p)
    assert 0.1 < reach < 0.855, f"home EE position {p} outside sane reach"
    assert p[2] > 0.0, "home EE below the base plane"
    print(f"[5] FK at q_home via yourdfpy: EE at {np.round(p, 4)} (|p|={reach:.3f} m)")

    print("\nAll setup checks passed.")


if __name__ == "__main__":
    main()
