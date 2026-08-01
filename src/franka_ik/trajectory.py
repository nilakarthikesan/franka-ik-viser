"""SE(3) reference trajectory sampling for the end effector.

Design and rationale: notes/stage-4-notes.md §1.

Each trajectory is a pure function (t, period, radius) -> 4x4 world->EE pose,
periodic in t so playback loops seamlessly. All paths live in a reachable
region in front of the FR3 base (centered ~[0.45, 0, 0.45] m, radius <= 0.12 m,
well inside the ~0.85 m reach and away from the singular stretched boundary).

Default orientation is the home end-effector orientation (gripper down/forward),
so frame 0 of every path is trivially reachable from q_home. Only `lissajous`
varies orientation, slerping between keyframes to exercise rotation tracking.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from .robot_model import DEFAULT_URDF, EE_FRAME, Q_HOME

CENTER = np.array([0.45, 0.0, 0.45])


def _home_orientation() -> np.ndarray:
    """Orientation of the end effector at q_home (cached), as a 3x3 matrix.

    Imported lazily and cached so trajectories carry no import-time cost and no
    hard dependency on a loaded model for callers that only want positions.
    """
    global _HOME_R
    try:
        return _HOME_R
    except NameError:
        pass
    import yourdfpy

    urdf = yourdfpy.URDF.load(DEFAULT_URDF, load_meshes=False)
    from .robot_model import FrankaModel

    model = FrankaModel()
    urdf.update_cfg({n: Q_HOME[i] for i, n in enumerate(model.joint_names)})
    _HOME_R = urdf.get_transform(EE_FRAME, urdf.base_link)[:3, :3].copy()
    return _HOME_R


def _pose(p: np.ndarray, R: np.ndarray | None = None) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = _home_orientation() if R is None else R
    T[:3, 3] = p
    return T


def circle(t: float, period: float = 8.0, radius: float = 0.12) -> np.ndarray:
    """Circle in the YZ plane, centered in front of the base. Fixed orientation."""
    ang = 2.0 * np.pi * t / period
    p = CENTER + np.array([0.0, radius * np.cos(ang), radius * np.sin(ang)])
    return _pose(p)


def figure_eight(t: float, period: float = 12.0, radius: float = 0.12) -> np.ndarray:
    """Gerono lemniscate (figure-8) in the YZ plane. Fixed orientation.

    y = r cos(theta), z = (r/2) sin(2 theta): closed, self-crossing, with
    sign-changing curvature and non-uniform speed -- a harder path than a circle.
    """
    ang = 2.0 * np.pi * t / period
    p = CENTER + np.array([0.0, radius * np.cos(ang), 0.5 * radius * np.sin(2.0 * ang)])
    return _pose(p)


# Orientation keyframes for the lissajous path: home orientation plus small
# tilts about x and y, revisited so the cycle is periodic (last == first).
_LISSAJOUS_KEY_TIMES = np.array([0.0, 0.25, 0.5, 0.75, 1.0])
_LISSAJOUS_KEY_ROTVECS = np.array([
    [0.0, 0.0, 0.0],
    [0.4, 0.0, 0.0],
    [0.0, 0.4, 0.0],
    [-0.4, 0.0, 0.0],
    [0.0, 0.0, 0.0],
])


def lissajous(t: float, period: float = 15.0, radius: float = 0.12) -> np.ndarray:
    """3D Lissajous path with orientation slerped between keyframes.

    Position: x, y, z sinusoids at 1:2:3 frequencies (a bounded 3D curve).
    Orientation: shortest-arc slerp through small tilt keyframes, so this is the
    one path that exercises rotation tracking, not just translation.
    """
    ang = 2.0 * np.pi * t / period
    p = CENTER + np.array([
        0.5 * radius * np.sin(ang),
        radius * np.sin(2.0 * ang),
        0.6 * radius * np.sin(3.0 * ang),
    ])

    key_rots = Rotation.from_rotvec(_LISSAJOUS_KEY_ROTVECS) * Rotation.from_matrix(_home_orientation())
    slerp = Slerp(_LISSAJOUS_KEY_TIMES, key_rots)
    phase = (t / period) % 1.0
    R = slerp([phase])[0].as_matrix()
    return _pose(p, R)


TRAJECTORIES = {
    "circle": circle,
    "figure-8": figure_eight,
    "lissajous": lissajous,
}
