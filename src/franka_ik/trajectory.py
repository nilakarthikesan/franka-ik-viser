"""SE(3) reference trajectories for the end effector, on selectable surfaces.

Design and rationale: notes/stage-4-notes.md §1 and §9.

A trajectory is a moving target pose T_des(t). Each is built from two choices:

  shape   in-plane curve: circle, figure-8 (Gerono lemniscate), or lissajous (3D)
  surface where/how it is drawn, which sets the plane AND the tool orientation so
          the gripper "pen" points INTO the drawing plane (not flat along it):
            - table : horizontal circle (XY), gripper points straight down
            - wall  : vertical circle (YZ) facing the robot, gripper points forward

Both surfaces use centers/radii chosen so every sample is comfortably reachable by
the FR3 (verified in scripts/test_trajectory.py; explored in
scripts/preview_orientations.py). Only lissajous varies orientation (a small slerp
wobble about the surface pose) so it still exercises rotation tracking.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from .robot_model import DEFAULT_URDF, EE_FRAME, Q_HOME

SHAPES = ("circle", "figure-8", "lissajous")
_PERIODS = {"circle": 8.0, "figure-8": 12.0, "lissajous": 15.0}

# Gripper approach axis (tool local +z) aligned with world +x -> points forward.
_R_FORWARD = np.array([[0.0, 0.0, 1.0],
                       [0.0, -1.0, 0.0],
                       [1.0, 0.0, 0.0]])

# lissajous orientation-wobble keyframes (rad), revisited so the cycle is periodic.
_WOBBLE_TIMES = np.array([0.0, 0.25, 0.5, 0.75, 1.0])
_WOBBLE_ROTVECS = np.array([[0.0, 0.0, 0.0], [0.35, 0.0, 0.0], [0.0, 0.35, 0.0],
                            [-0.35, 0.0, 0.0], [0.0, 0.0, 0.0]])


def _home_orientation() -> np.ndarray:
    """End-effector orientation at q_home (cached): the gripper-straight-down pose."""
    global _R_DOWN
    try:
        return _R_DOWN
    except NameError:
        pass
    import yourdfpy

    from .robot_model import FrankaModel

    urdf = yourdfpy.URDF.load(DEFAULT_URDF, load_meshes=False)
    model = FrankaModel()
    urdf.update_cfg({n: Q_HOME[i] for i, n in enumerate(model.joint_names)})
    _R_DOWN = urdf.get_transform(EE_FRAME, urdf.base_link)[:3, :3].copy()
    return _R_DOWN


@dataclass(frozen=True)
class Surface:
    """A drawing surface: where the plane sits and how the tool is held."""

    center: np.ndarray
    radius: float
    axes: np.ndarray       # 3x3, columns map in-plane (u, v) and depth (w) to world
    orientation: np.ndarray  # 3x3 constant tool orientation (pen INTO the plane)

    def place(self, uvw: np.ndarray) -> np.ndarray:
        return self.center + self.axes @ uvw

    @property
    def normal(self) -> np.ndarray:
        return self.axes[:, 2]


def _surfaces() -> dict[str, Surface]:
    global _SURFACES
    try:
        return _SURFACES
    except NameError:
        pass
    R_down = _home_orientation()
    _SURFACES = {
        # u->x, v->y, w->z : horizontal disk on a 'table', pen points down (-z)
        "table": Surface(center=np.array([0.45, 0.0, 0.35]), radius=0.12,
                         axes=np.eye(3), orientation=R_down),
        # u->y, v->z, w->x : vertical disk facing the robot, pen points forward (+x)
        "wall": Surface(center=np.array([0.5, 0.0, 0.5]), radius=0.10,
                        axes=np.array([[0.0, 0.0, 1.0],
                                       [1.0, 0.0, 0.0],
                                       [0.0, 1.0, 0.0]]),
                        orientation=_R_FORWARD),
    }
    return _SURFACES


SURFACES = ("table", "wall")


def period_of(shape: str) -> float:
    return _PERIODS[shape]


def _shape_uvw(shape: str, t: float, period: float, radius: float) -> np.ndarray:
    """In-plane (u, v) and depth (w) offsets for a shape at time t."""
    ang = 2.0 * np.pi * t / period
    if shape == "circle":
        return np.array([radius * np.cos(ang), radius * np.sin(ang), 0.0])
    if shape == "figure-8":
        return np.array([radius * np.cos(ang), 0.5 * radius * np.sin(2.0 * ang), 0.0])
    if shape == "lissajous":
        return np.array([0.5 * radius * np.sin(ang),
                         radius * np.sin(2.0 * ang),
                         0.6 * radius * np.sin(3.0 * ang)])
    raise ValueError(f"unknown shape {shape!r}")


def _orientation(shape: str, surface: Surface, t: float, period: float) -> np.ndarray:
    if shape != "lissajous":
        return surface.orientation
    base = Rotation.from_matrix(surface.orientation)
    keys = Rotation.from_rotvec(_WOBBLE_ROTVECS) * base
    slerp = Slerp(_WOBBLE_TIMES, keys)
    return slerp([(t / period) % 1.0])[0].as_matrix()


def pose(shape: str, surface: str, t: float, period: float | None = None) -> np.ndarray:
    """World -> EE target pose (4x4) for a shape drawn on a surface at time t."""
    s = _surfaces()[surface]
    period = _PERIODS[shape] if period is None else period
    T = np.eye(4)
    T[:3, :3] = _orientation(shape, s, t, period)
    T[:3, 3] = s.place(_shape_uvw(shape, t, period, s.radius))
    return T


def reference_points(shape: str, surface: str, n: int = 200) -> np.ndarray:
    """(n+1, 3) sampled positions of one full period, for drawing the reference."""
    period = _PERIODS[shape]
    return np.array([pose(shape, surface, period * k / n)[:3, 3] for k in range(n + 1)])
