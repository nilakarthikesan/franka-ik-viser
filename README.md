# franka-ik-viser

Inverse kinematics on a Franka FR3 arm, implemented from scratch (damped least squares)
and visualized live in [Viser](https://viser.studio). The end effector tracks sampled
SE(3) reference trajectories, or an interactive drag gizmo.

See [DESIGN.md](DESIGN.md) for the full system design.

## Quickstart

```bash
python3.12 -m venv .venv && source .venv/bin/activate   # any Python >= 3.10
pip install -r requirements.txt

# Get the official Franka robot models and generate the FR3 URDF.
# generate_urdf.py wraps franka_description's create_urdf.py (shims its ROS
# dependency) and rewrites mesh paths to be relative to the URDF.
git clone https://github.com/frankarobotics/franka_description assets/franka_description
python scripts/generate_urdf.py

# Check the setup (URDF data, joints, limits, EE frame, meshes)
python scripts/verify_setup.py

# M1: render the arm with joint sliders (opens a Viser tab in your browser)
python scripts/render_m1.py

# Run the IK demo (coming with M3+)
python -m franka_ik.app
```

## Status

- [x] M0 environment + URDF generated (`scripts/generate_urdf.py`, `scripts/verify_setup.py`)
- [x] M1 robot renders in Viser with joint sliders (`scripts/render_m1.py`)
- [ ] M2 FK/Jacobian implemented + validated vs. PyRoKi
- [ ] M3 static-target IK converges
- [ ] M4 trajectory tracking (circle / figure-8)
- [ ] M5 interactive gizmo mode
- [ ] M6 recording + writeup
