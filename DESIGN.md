# Design Doc — Franka Inverse Kinematics with Viser Visualization

**Task (as assigned):** Implement inverse kinematics for a Franka robot arm using the
kinematic model from [`frankarobotics/franka_description`](https://github.com/frankarobotics/franka_description),
and visualize it in [Viser](https://viser.studio). Sample an end-effector pose trajectory
and drive the arm with IK so the end effector follows the reference trajectory — either
interactively (drag a target gizmo) or with offline sampled trajectories.

**Deliverable:** A browser-based Viser demo where the FR3 arm tracks a reference
end-effector pose trajectory, plus a short recording/GIF and this writeup.

---

## 1. Background: differential (Jacobian-based) IK

Forward kinematics maps joint angles to the end-effector pose: \( T_{ee} = f(q) \), where
\( q \in \mathbb{R}^7 \) for the Franka arm. IK inverts this: given a desired pose
\( T_{des} \in SE(3) \), find \( q \).

We solve it iteratively with **damped least squares (DLS)**, a.k.a. Levenberg–Marquardt:

1. Compute the 6D pose error twist \( e = \log(T_{ee}^{-1} T_{des}) \) (position + orientation error).
2. Compute the geometric Jacobian \( J(q) \in \mathbb{R}^{6 \times 7} \) relating joint
   velocities to end-effector twist.
3. Update joints: \( \Delta q = J^\top (J J^\top + \lambda^2 I)^{-1} e \).
4. Clamp to joint limits, integrate \( q \leftarrow q + \alpha \Delta q \), repeat until
   \( \|e\| \) is below tolerance or max iterations.

The damping term \( \lambda \) keeps the solve stable near singularities (where plain
pseudoinverse blows up). Because the FR3 has 7 DoF (one redundant), we can optionally add a
**nullspace posture task** that biases the arm toward a comfortable "home" configuration
without disturbing the end-effector task:
\( \Delta q_{null} = (I - J^+ J)\, k_{posture} (q_{home} - q) \).

We use [Pinocchio](https://github.com/stack-of-tasks/pinocchio) (`pip install pin`) for FK,
Jacobians, and SE(3) log/exp — the IK loop itself is implemented by us (that is the point of
the exercise); Pinocchio only supplies the standard kinematics calculations, which is
explicitly allowed ("there are set libraries that do these calculations").

## 2. System overview

```
                 ┌─────────────────────────────────────────────┐
                 │                  app.py                      │
                 │        (main loop @ ~50 Hz, mode switch)     │
                 └──────┬──────────────┬───────────────┬────────┘
                        │              │               │
             ┌──────────▼───┐  ┌───────▼────────┐  ┌───▼──────────────┐
             │ trajectory.py │  │    ik.py       │  │ visualizer.py    │
             │ reference SE3 │─▶│ DLS solver     │─▶│ Viser scene:     │
             │ pose sampler  │  │ q_{t+1} given  │  │  - ViserUrdf robot│
             │ (circle, fig8,│  │ target pose    │  │  - target gizmo  │
             │  lissajous,   │  └───────┬────────┘  │  - EE trace line │
             │  waypoints)   │          │           │  - GUI controls  │
             └───────────────┘  ┌───────▼────────┐  └──────────────────┘
                                │ robot_model.py │
                                │ Pinocchio model│
                                │ from FR3 URDF  │
                                │ (FK, Jacobian, │
                                │  joint limits) │
                                └────────────────┘
```

Two run modes, selectable in the Viser GUI:

- **Playback mode:** a parametric reference trajectory (circle / figure-8 / lissajous in
  SE(3), orientation via slerp between keyframes) is sampled over time; each frame the IK
  solver tracks the moving target. The reference path and the achieved end-effector path are
  both drawn so tracking quality is visible.
- **Interactive mode:** a Viser `transform_controls` gizmo is the target; the user drags it
  and the arm follows in real time.

## 3. Components

### 3.1 Robot model (`robot_model.py`)
- Source of truth: `frankarobotics/franka_description`, vendored as a git submodule or a
  pinned clone in `assets/`. Generate the FR3 URDF with the repo's
  `create_urdf.py fr3 --robot-ee franka_hand --abs-path` script (URDFs are script-generated
  in that repo, not checked in).
- Load into Pinocchio (`pin.buildModelFromUrdf`). Lock the two finger joints (they are
  prismatic gripper joints, irrelevant to arm IK) so the model is exactly 7 DoF.
- Expose: `fk(q) -> SE3`, `jacobian(q) -> 6x7`, `joint_limits`, `q_home`, `frame_id` of the
  end-effector frame (`fr3_hand_tcp`).
- Fallback if URDF generation is painful: the `robot_descriptions` pip package ships a
  ready-made Panda/FR3 URDF loader. Note the fallback in the README if used.

### 3.2 IK solver (`ik.py`)
- `solve_step(q, T_des) -> q_next`: one DLS iteration as in §1 (used for tracking a moving
  target — one step per render frame is smooth and stable).
- `solve(q0, T_des, tol, max_iters) -> q*`: full solve for static targets.
- Tunables exposed in the GUI: damping `λ`, step scale `α`, position/orientation error
  weights, nullspace posture gain.
- Joint limits enforced by clamping after each step.

### 3.3 Trajectory generator (`trajectory.py`)
- `sample(t) -> SE3` for parametric paths: circle, figure-8, lissajous, centered in a
  reachable region of the FR3 workspace (~0.4–0.6 m in front of the base).
- Orientation profile: fixed "gripper pointing down" default, plus an option that slerps
  between two keyframe orientations over the cycle.
- All paths verified reachable (inside the ~0.85 m FR3 reach envelope).

### 3.4 Visualizer (`visualizer.py`)
- `viser.ViserServer` + `viser.extras.ViserUrdf` to render the URDF (meshes come from
  franka_description).
- Scene elements: reference trajectory line (`add_spline` / line segments), live EE trace,
  target frame axes, error text readout.
- GUI: mode dropdown, trajectory dropdown, speed slider, IK gain sliders, pause/reset
  buttons, current tracking error (mm / deg).

### 3.5 App loop (`app.py`)
- ~50 Hz loop: get target pose (from trajectory clock or gizmo) → one IK step → update
  robot joint config in Viser → update traces and error readout.
- Headless-friendly: also a `record.py` script that runs a full trajectory offline and dumps
  per-step tracking error to CSV + a summary plot for the writeup.

## 4. Repository layout

```
franka-ik-viser/
├── DESIGN.md               ← this document
├── README.md               ← quickstart
├── requirements.txt
├── assets/                 ← generated FR3 URDF + meshes (from franka_description)
└── src/franka_ik/
    ├── __init__.py
    ├── robot_model.py      ← URDF → Pinocchio wrapper (FK, Jacobian, limits)
    ├── ik.py               ← DLS solver (+ nullspace posture task)
    ├── trajectory.py       ← SE(3) reference trajectory sampling
    ├── visualizer.py       ← Viser scene + GUI
    ├── app.py              ← main entry point (interactive + playback modes)
    └── record.py           ← offline run → tracking-error CSV/plot
```

## 5. Milestones

| # | Milestone | Definition of done |
|---|-----------|--------------------|
| M0 | Environment | `pip install viser pin yourdfpy numpy` works; franka_description cloned, FR3 URDF generated |
| M1 | Render | FR3 renders in a Viser browser tab with joint sliders |
| M2 | FK sanity | Pinocchio FK matches Viser-rendered EE frame for several joint configs |
| M3 | Static IK | Solver converges to random reachable target poses (< 1 mm, < 0.5°) |
| M4 | Tracking | Arm follows circle/figure-8 trajectory; reference vs actual traces plotted |
| M5 | Interactive | Drag gizmo, arm follows in real time; GUI polish |
| M6 | Deliverable | Screen recording + tracking-error plot + README writeup |

## 6. Risks & mitigations

- **URDF generation friction** (franka_description builds URDFs via script, sometimes with
  ROS-style `package://` mesh paths): use `--abs-path`, or rewrite mesh paths to relative;
  fallback to `robot_descriptions` package.
- **Pinocchio install on Apple Silicon:** `pip install pin` has arm64 wheels; if broken,
  conda-forge `pinocchio` is the fallback.
- **Singularity/limit weirdness during tracking:** DLS damping + keeping trajectories in a
  comfortable workspace region; nullspace posture task keeps elbow sane.
- **Gripper mimic joints confusing the model:** lock finger joints at model-build time.

## 7. Stretch goals (only if time remains)

- Orientation-weighting toggle (position-only IK vs full 6D).
- Manipulability ellipsoid visualization at the EE.
- Side-by-side comparison with an off-the-shelf solver (e.g. PyRoKi) to sanity-check ours.
