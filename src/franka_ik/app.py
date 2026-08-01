"""Main entry point: ~50 Hz loop tying trajectory -> IK -> Viser together.

Design and rationale: notes/stage-4-notes.md §3 and §9.

Each frame:
  target = trajectory(shape, surface, clock)   (playback)  or  gizmo pose (interactive)
  q      = solver.solve_step(q, target)                    one bounded step -> smooth
  scene.set_joint_config(q); update EE trace + mm/deg error readout

Run:  PYTHONPATH=src python -m franka_ik.app
"""

from __future__ import annotations

import time

import numpy as np

from .ik import DLSSolver, IKConfig
from .robot_model import Q_HOME, FrankaModel
from .trajectory import pose, reference_points
from .visualizer import Scene

HZ = 50.0
DT = 1.0 / HZ


def main() -> None:
    model = FrankaModel()
    config = IKConfig()
    solver = DLSSolver(model, config)
    scene = Scene(model, config)

    q = Q_HOME.copy()
    scene.set_joint_config(q)

    state = {"clock": 0.0, "paused": False, "mode": "playback",
             "shape": "circle", "surface": "table"}
    scene.draw_reference(reference_points(state["shape"], state["surface"]))

    def restart_path() -> None:
        state["clock"] = 0.0
        scene.clear_trace()
        scene.draw_reference(reference_points(state["shape"], state["surface"]))

    @scene.gui.pause.on_click
    def _(_):
        state["paused"] = not state["paused"]

    @scene.gui.reset.on_click
    def _(_):
        nonlocal q
        q = Q_HOME.copy()
        state["clock"] = 0.0
        scene.clear_trace()
        scene.set_joint_config(q)

    @scene.gui.trajectory.on_update
    def _(_):
        state["shape"] = scene.gui.trajectory.value
        restart_path()

    @scene.gui.surface.on_update
    def _(_):
        state["surface"] = scene.gui.surface.value
        restart_path()

    @scene.gui.mode.on_update
    def _(_):
        new_mode = scene.gui.mode.value
        if new_mode == "interactive":
            # Snap the gizmo to where the arm already is, so nothing jumps.
            scene.set_target(model.fk(q))
        state["mode"] = new_mode
        scene.clear_trace()

    print(f"Viser running at http://localhost:{scene.server.get_port()}")
    while True:
        frame_start = time.time()

        if state["mode"] == "playback":
            if not state["paused"]:
                state["clock"] += scene.gui.speed.value * DT
            T_des = pose(state["shape"], state["surface"], state["clock"])
            scene.set_target(T_des)
        else:
            T_des = scene.target_pose()

        q = solver.solve_step(q, T_des)
        scene.set_joint_config(q)

        pos, rot = solver.error(q, T_des)
        scene.set_error(pos * 1e3, np.rad2deg(rot))
        scene.append_ee_trace(model.fk(q)[:3, 3])

        elapsed = time.time() - frame_start
        if elapsed < DT:
            time.sleep(DT - elapsed)


if __name__ == "__main__":
    main()
