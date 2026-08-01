"""Viser scene: robot, target gizmo, traces, and GUI controls.

Design and rationale: notes/stage-4-notes.md §2. This module hides every Viser
call behind a small Scene interface that the app loop drives. Quaternion
conventions (Viser wxyz vs scipy xyzw) are confined to the two helpers below so
the convention lives in exactly one place.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import viser
import yourdfpy
from scipy.spatial.transform import Rotation
from viser.extras import ViserUrdf

from .ik import IKConfig
from .robot_model import DEFAULT_URDF, FrankaModel

FINGER_OPEN = 0.04  # m, prismatic finger joints held open
TRACE_MAX = 400     # rolling EE-trace length (comet tail)


def mat_to_wxyz(R: np.ndarray) -> np.ndarray:
    """3x3 rotation -> Viser wxyz quaternion (scipy returns xyzw)."""
    x, y, z, w = Rotation.from_matrix(R).as_quat()
    return np.array([w, x, y, z])


def wxyz_to_mat(wxyz: np.ndarray) -> np.ndarray:
    """Viser wxyz quaternion -> 3x3 rotation."""
    w, x, y, z = wxyz
    return Rotation.from_quat([x, y, z, w]).as_matrix()


@dataclass
class GuiHandles:
    mode: object
    trajectory: object
    surface: object
    speed: object
    pause: object
    reset: object
    damping: object
    step_scale: object
    posture_gain: object
    max_step: object
    pos_err: object
    rot_err: object


class Scene:
    def __init__(self, model: FrankaModel, config: IKConfig, urdf_path: Path = DEFAULT_URDF):
        self.model = model
        self.config = config
        self._urdf = yourdfpy.URDF.load(urdf_path, mesh_dir=str(Path(urdf_path).parent))

        self.server = viser.ViserServer()
        self.server.initial_camera.position = (1.1, -0.9, 0.8)
        self.server.initial_camera.look_at = (0.3, 0.0, 0.4)
        self.server.scene.add_grid("/ground", width=2.0, height=2.0)
        self.robot = ViserUrdf(self.server, urdf_or_path=self._urdf, root_node_name="/robot")

        # Map our 7 arm joints (by name) into ViserUrdf's actuated ordering; the
        # finger joint is held open. Mapping by name is robust to ordering.
        self._actuated = list(self.robot.get_actuated_joint_names())
        self._arm_slots = [self._actuated.index(n) for n in self.model.joint_names]

        self.target = self.server.scene.add_transform_controls(
            "/target", scale=0.15, disable_sliders=True,
        )
        self._ee_frame = self.server.scene.add_frame(
            "/ee", show_axes=True, axes_length=0.08, axes_radius=0.004,
        )
        self._trace: list[np.ndarray] = []
        self._trace_handle = None

        self.gui = self._build_gui()

    # -- GUI ----------------------------------------------------------------

    def _build_gui(self) -> GuiHandles:
        g = self.server.gui
        from .trajectory import SHAPES, SURFACES

        with g.add_folder("Mode"):
            mode = g.add_dropdown("Mode", ("playback", "interactive"), initial_value="playback")
            trajectory = g.add_dropdown("Trajectory", SHAPES, initial_value="circle")
            surface = g.add_dropdown("Surface", SURFACES, initial_value="table",
                                     hint="table = draw flat (pen down); wall = draw upright (pen forward)")
            speed = g.add_slider("Speed", min=0.0, max=3.0, step=0.05, initial_value=1.0)
            pause = g.add_button("Pause / Resume")
            reset = g.add_button("Reset")

        with g.add_folder("Solver gains"):
            damping = g.add_slider("Damping (lambda)", min=1e-3, max=0.5, step=1e-3,
                                   initial_value=self.config.damping)
            step_scale = g.add_slider("Step scale (alpha)", min=0.1, max=1.0, step=0.05,
                                      initial_value=self.config.step_scale)
            posture_gain = g.add_slider("Posture gain", min=0.0, max=0.3, step=0.01,
                                        initial_value=self.config.posture_gain)
            max_step = g.add_slider("Max step (rad)", min=0.02, max=0.5, step=0.01,
                                    initial_value=self.config.max_step)

        with g.add_folder("Tracking error"):
            # step sets display precision; 1e-4 so sub-millimetre tracking is visible.
            pos_err = g.add_number("Position (mm)", initial_value=0.0, step=1e-4, disabled=True)
            rot_err = g.add_number("Rotation (deg)", initial_value=0.0, step=1e-4, disabled=True)

        # Wire gain sliders straight into the live IKConfig.
        damping.on_update(lambda _: setattr(self.config, "damping", damping.value))
        step_scale.on_update(lambda _: setattr(self.config, "step_scale", step_scale.value))
        posture_gain.on_update(lambda _: setattr(self.config, "posture_gain", posture_gain.value))
        max_step.on_update(lambda _: setattr(self.config, "max_step", max_step.value))

        return GuiHandles(mode, trajectory, surface, speed, pause, reset, damping,
                          step_scale, posture_gain, max_step, pos_err, rot_err)

    # -- robot / target -----------------------------------------------------

    def set_joint_config(self, q: np.ndarray) -> None:
        cfg = np.full(len(self._actuated), FINGER_OPEN)
        for slot, value in zip(self._arm_slots, q):
            cfg[slot] = value
        self.robot.update_cfg(cfg)
        T = self.model.fk(q)
        self._ee_frame.position = T[:3, 3]
        self._ee_frame.wxyz = mat_to_wxyz(T[:3, :3])

    def target_pose(self) -> np.ndarray:
        T = np.eye(4)
        T[:3, :3] = wxyz_to_mat(np.asarray(self.target.wxyz))
        T[:3, 3] = np.asarray(self.target.position)
        return T

    def set_target(self, T: np.ndarray) -> None:
        self.target.position = T[:3, 3]
        self.target.wxyz = mat_to_wxyz(T[:3, :3])

    # -- traces -------------------------------------------------------------

    def draw_reference(self, points: np.ndarray) -> None:
        self.server.scene.add_spline_catmull_rom(
            "/reference", np.asarray(points, dtype=np.float32), closed=True,
            line_width=2.0, color=(80, 160, 255),
        )

    def clear_trace(self) -> None:
        self._trace = []
        if self._trace_handle is not None:
            self._trace_handle.remove()
            self._trace_handle = None

    def append_ee_trace(self, point: np.ndarray) -> None:
        self._trace.append(np.asarray(point, dtype=np.float32))
        if len(self._trace) > TRACE_MAX:
            self._trace = self._trace[-TRACE_MAX:]
        if len(self._trace) < 2:
            return
        pts = np.array(self._trace)
        segments = np.stack([pts[:-1], pts[1:]], axis=1)  # (N-1, 2, 3)
        if self._trace_handle is not None:
            self._trace_handle.remove()
        self._trace_handle = self.server.scene.add_line_segments(
            "/ee_trace", segments, colors=(255, 120, 40), line_width=3.0,
        )

    def set_error(self, pos_mm: float, rot_deg: float) -> None:
        self.gui.pos_err.value = round(float(pos_mm), 4)
        self.gui.rot_err.value = round(float(rot_deg), 4)
