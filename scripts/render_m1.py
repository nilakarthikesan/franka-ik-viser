"""M1 milestone check: render the FR3 in a Viser browser tab with joint sliders.

This exercises the full setup chain established in Stage 1:
generated URDF -> yourdfpy parse (relative meshes) -> ViserUrdf render,
with one slider per arm joint (bounded by the URDF's joint limits) plus the
finger joints fixed open. Later stages replace the sliders with the IK loop.

Usage:  .venv/bin/python scripts/render_m1.py   then open http://localhost:8080
"""

import time
from pathlib import Path

import numpy as np
import viser
import yourdfpy
from viser.extras import ViserUrdf

REPO_ROOT = Path(__file__).resolve().parents[1]
URDF_PATH = REPO_ROOT / "assets" / "franka_description" / "urdfs" / "fr3_franka_hand.urdf"
Q_HOME = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
FINGER_OPEN = 0.04  # m, both prismatic finger joints


def main() -> None:
    urdf = yourdfpy.URDF.load(URDF_PATH, mesh_dir=str(URDF_PATH.parent))

    server = viser.ViserServer()
    server.initial_camera.position = (1.1, -0.9, 0.8)
    server.initial_camera.look_at = (0.3, 0.0, 0.4)
    server.scene.add_grid("/ground", width=2.0, height=2.0)
    robot = ViserUrdf(server, urdf_or_path=urdf, root_node_name="/robot")

    # ViserUrdf.update_cfg expects values in the order of the URDF's actuated joints.
    actuated = urdf.actuated_joints
    initial = {j.name: Q_HOME[i] for i, j in enumerate(j for j in actuated if j.type == "revolute")}

    sliders = []
    with server.gui.add_folder("Joints"):
        for joint in actuated:
            if joint.type == "revolute":
                init = initial[joint.name]
                slider = server.gui.add_slider(
                    joint.name, min=float(joint.limit.lower), max=float(joint.limit.upper),
                    step=0.01, initial_value=float(init),
                )
            else:  # finger joints: fixed open, no slider
                slider = None
            sliders.append(slider)

    def current_cfg() -> np.ndarray:
        return np.array([s.value if s is not None else FINGER_OPEN for s in sliders])

    robot.update_cfg(current_cfg())
    for s in sliders:
        if s is not None:
            s.on_update(lambda _: robot.update_cfg(current_cfg()))

    print(f"Viser running at http://localhost:{server.get_port()}")
    while True:
        time.sleep(1.0)


if __name__ == "__main__":
    main()
