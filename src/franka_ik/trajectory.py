"""SE(3) reference trajectory sampling for the end effector.

All trajectories live in a reachable region in front of the FR3 base
(centered ~[0.45, 0.0, 0.45] m, radius <= 0.15 m) with a default
gripper-pointing-down orientation. Orientation keyframes are slerped.
"""

import numpy as np

CENTER = np.array([0.45, 0.0, 0.45])


def circle(t: float, period: float = 8.0, radius: float = 0.12):
    """Circular EE path in the YZ plane. Returns 4x4 pose. TODO(M4)."""
    raise NotImplementedError


def figure_eight(t: float, period: float = 12.0, radius: float = 0.12):
    """Figure-8 (lemniscate) path. TODO(M4)."""
    raise NotImplementedError


def lissajous(t: float, period: float = 15.0):
    """3D lissajous path with slerped orientation keyframes. TODO(M4/stretch)."""
    raise NotImplementedError


TRAJECTORIES = {
    "circle": circle,
    "figure-8": figure_eight,
    "lissajous": lissajous,
}
