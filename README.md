# franka-ik-viser

Inverse kinematics on a Franka FR3 arm, implemented from scratch (damped least squares)
and visualized live in [Viser](https://viser.studio). The end effector tracks sampled
SE(3) reference trajectories, or an interactive drag gizmo.

See [DESIGN.md](DESIGN.md) for the full system design.

## Quickstart

```bash
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt

# Get the official Franka robot models and generate the FR3 URDF
git clone https://github.com/frankarobotics/franka_description assets/franka_description
cd assets/franka_description && python scripts/create_urdf.py fr3 --robot-ee franka_hand --abs-path && cd ../..

# Run the demo (opens a Viser tab in your browser)
python -m franka_ik.app
```

## Status

- [ ] M0 environment + URDF generated
- [ ] M1 robot renders in Viser
- [ ] M2 FK sanity check
- [ ] M3 static-target IK converges
- [ ] M4 trajectory tracking (circle / figure-8)
- [ ] M5 interactive gizmo mode
- [ ] M6 recording + writeup
