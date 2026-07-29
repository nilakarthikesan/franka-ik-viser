# Stage 1 Notes — Understanding the Problem Statement

Companion notes to [DESIGN.md §1](../DESIGN.md). Goal: be able to explain every word of
the problem statement, why the inputs/outputs/success criteria are what they are, and
how we choose tools — before writing any code.

---

## 1. The task in plain language

> "Implement inverse kinematics for a Franka arm using the kinematic model from
> franka_description, and visualize it in Viser. Sample an end-effector pose trajectory
> and drive the arm with IK."

Translation: we have a simulated 7-joint robot arm. Someone hands us a moving target —
"the hand should be *here*, oriented *like this*, at time *t*" — and our job is to
compute, every fraction of a second, the seven joint angles that put the hand there.
Then we show it working, live, in a browser.

The assignment is really testing three things:

1. Do you understand the **math** of arm kinematics (not just how to call a library)?
2. Can you build a **working system** around it (real-time loop, visualization, UX)?
3. Can you **communicate** it (demo, evidence it works, writeup)?

---

## 2. Glossary — every term in the problem statement

**End effector (EE).** The "business end" of the arm — for the FR3, the gripper/hand.
When we say "the robot's pose," we almost always mean the pose of this one frame,
called `fr3_hand_tcp` (TCP = tool center point, the point between the fingertips).

**Pose.** Position *and* orientation together. A position is 3 numbers (x, y, z). An
orientation is a 3D rotation. A pose is the combination — where something is and which
way it's facing. Fully describing a rigid body in space takes exactly these 6 degrees
of freedom.

**SE(3).** The mathematical name for "the set of all poses" (Special Euclidean group in
3D). A pose is written as a 4×4 homogeneous transform matrix:

```
T = | R  p |     R: 3x3 rotation matrix
    | 0  1 |     p: 3x1 position vector
```

Why matrices? Because composing motions becomes matrix multiplication: "frame B relative
to A, times C relative to B, gives C relative to A." That one property is what makes all
of kinematics mechanical to compute.

**Degrees of freedom (DoF).** The number of independent ways something can move. A free
rigid body has 6 (3 translation + 3 rotation). The FR3 arm has **7 joints**, each
contributing 1 DoF. Seven knobs to set, six numbers to achieve — that mismatch is
important (see *redundancy*).

**Joint space vs. task space.** Two coordinate systems for describing the same robot:

- *Joint space:* the vector \( q \in \mathbb{R}^7 \) of joint angles. This is what we
  actually control.
- *Task space:* the SE(3) pose of the end effector. This is what we actually care about.

The whole problem is translating between them.

**Kinematic model / kinematic chain.** The arm is a chain: base → link1 → link2 → … →
hand. Each joint connects two links and rotates about a fixed axis. The "kinematic
model" is the complete geometric description: where each joint sits relative to the
previous link, which axis it spins about, and its angle limits. Note what it *excludes*:
masses, motor torques, friction — that would be *dynamics*. We only need kinematics.

**URDF (Unified Robot Description Format).** The XML file format that stores the
kinematic model (plus mesh files for rendering). This is our "data" — the one external
input to the whole project. Franka publishes theirs in `franka_description`; using the
official file means our robot's geometry is exactly the real robot's geometry.

**Forward kinematics (FK).** Joint angles → hand pose. Easy direction: start at the
base, and for each joint multiply on (a) the fixed transform to the joint (from the
URDF) and (b) a rotation by the current joint angle about the joint's axis. Seven matrix
products later you have the hand pose. One answer, always, closed form.

**Inverse kinematics (IK).** Hand pose → joint angles. The hard direction, because FK
is nonlinear (all those chained rotations produce sines and cosines everywhere) and
you can't just algebraically invert it. Three things make it genuinely hard:

- *Non-uniqueness:* many joint configurations reach the same pose (think of touching
  your nose with your elbow up vs. tucked in).
- *No solution:* poses beyond ~0.85 m from the FR3's base are simply unreachable.
- *Constraints:* even valid solutions must respect joint limits and, for a live demo,
  connect smoothly to the previous instant's solution.

**Redundancy.** 7 joint DoF − 6 pose DoF = 1 spare. For any reachable hand pose there's
a whole one-parameter family of arm shapes (the elbow can orbit). This is a *feature*:
we get to spend the spare DoF on a secondary objective — keeping the arm near a
comfortable "home" posture — without disturbing the hand. That's the "nullspace posture
task" in the design.

**Singularity.** A configuration where the arm locally loses the ability to move in some
direction (e.g., fully stretched out — no joint motion can move the hand further out).
Naive IK math divides by ~zero there and commands huge joint velocities. Our solver's
"damping" exists precisely to stay well-behaved near these.

**Trajectory.** A pose as a function of time, \( T_{des}(t) \) — e.g., a circle traced
in front of the robot over 8 seconds. "Sample a trajectory" = evaluate it at the current
clock time each frame.

**Differential IK / tracking.** Instead of solving each pose from scratch, exploit that
the target barely moves between frames: take *one small corrective step* per frame from
the current configuration. 50 steps/second of small corrections = smooth tracking. This
is the method we implement (details are Stage 2 material; the design doc §2 has the math).

---

## 3. Inputs and outputs, unpacked

### Input: the target pose \( T_{des}(t) \), from one of two sources

| Mode | Source | Why it exists |
|---|---|---|
| **Playback** | Parametric trajectory (circle, figure-8) evaluated at time t | The assignment's literal ask: "sample an EE pose trajectory." Repeatable → measurable → we can plot tracking error |
| **Interactive** | A drag gizmo in the Viser browser UI | Makes the demo compelling and proves the solver handles *arbitrary* targets, not just curated paths |

Both modes produce the identical thing — a 4×4 pose — so everything downstream of the
target is one code path. That's a deliberate architecture choice: the solver never knows
or cares where the target came from.

Design choices hidden inside the input: trajectories are centered ~[0.45, 0, 0.45] m in
front of the base with radius ≤ 0.15 m. Why? The FR3 reaches ~0.85 m; staying in the
middle of the workspace keeps us away from unreachable poses and singularities, so
demos are smooth and failures are real bugs rather than physics.

### Output: the joint vector \( q(t) \in \mathbb{R}^7 \)

Three requirements, in increasing order of subtlety:

1. **Correct:** FK(q) matches the target pose (within tolerance).
2. **Legal:** every angle inside the FR3's published joint limits (they differ per
   joint; e.g. joint 4 can only bend between about −3.0 and −0.15 rad).
3. **Smooth:** consecutive outputs close together. A solver that teleports between
   equally-"correct" but distant configurations would be useless on a real robot and
   ugly in the demo. Differential IK gives us this almost for free, since each output
   is one small step from the last.

### What is *not* an input: data

There is no dataset, no training, no learned model. The only external artifact is the
robot description (URDF + meshes). Everything else is computed from geometry at runtime.
If someone asks "what data do you need?" — the answer is: *the official kinematic
description of the robot, pinned to a known version for reproducibility.*

---

## 4. Success criteria, unpacked

Two layers, because "it works" needs both a human check and a number:

**Qualitative:** open the browser, watch the arm ride the reference path with no
stutter, no wild elbow swings, no drift. This is what "she" will actually see first.

**Quantitative:**

- *Static solves:* from a random start, converge to a random reachable target within
  **< 1 mm position and < 0.5° orientation** error. Why these numbers? They're far
  tighter than anything visible on screen and comparable to real manipulator
  repeatability (~0.1 mm) scales — tight enough to prove the math is right, loose enough
  to be achievable in a bounded number of iterations.
- *Tracking:* during playback, error stays at the mm / sub-degree level throughout.
  Tracking error is *expected* to be nonzero — we take one solver step per frame while
  the target moves, so we're always chasing slightly. The plot of error-over-time is
  the honest evidence: it should be small, bounded, and periodic with the trajectory.

There's also an implicit criterion worth naming: **reproducibility**. `pip install -r
requirements.txt && python -m franka_ik.app` must work on a fresh machine. Several
tooling decisions below trace back to this.

---

## 5. The deliverable, unpacked

Three artifacts, each answering a different reviewer question:

| Artifact | Question it answers | Form |
|---|---|---|
| Live demo + GIF | "Does it work?" | Viser demo; 10-sec GIF at top of README |
| Tracking-error plots | "How well, measurably?" | `record.py` headless run → CSV → matplotlib figure (reference vs. achieved, error in mm/deg over time) |
| Report | "Do you understand what you built?" | DESIGN.md evolved: problem → method → results → references |

The repo itself is the presentation: README opens with the GIF, quickstart reproduces
it in ~3 commands, the report explains it, git history shows the build progression.

---

## 6. Tooling — what we use and how we decided

### The decision framework

Every tool choice in this project was run through four questions:

1. **Does it respect the assignment's intent?** The IK solver must be ours. Libraries
   are fair game only for standard, well-defined calculations (rendering, URDF parsing,
   linear algebra) — the assignment explicitly permits this.
2. **Can anyone install it?** (reproducibility) — prefer pure `pip install`, no conda,
   no Docker, no ROS.
3. **Does it strengthen or weaken the story?** — "I wrote FK myself and proved it
   correct" beats "I called a library" for a skills-demonstration project.
4. **What's the risk, and is it retired early?** — anything we hand-roll needs a
   validation plan before the rest of the system builds on it.

### The chosen stack

| Tool | Role | Why it won |
|---|---|---|
| **numpy** | All the math: transforms, Jacobian, DLS linear solve | The `(J Jᵀ + λ²I)⁻¹ e` step is one `np.linalg.solve`; nothing more is needed |
| **Viser** (v1.0.27) | Browser 3D visualization: robot rendering (`ViserUrdf`), drag gizmo, GUI sliders, traces | Named in the assignment. Web-based (demo = a URL), pure pip, actively maintained, citable (arXiv:2507.22885) |
| **franka_description** | The kinematic model (URDF + meshes), pinned in `assets/` | Named in the assignment; it's the official source of the robot's geometry |
| **yourdfpy** | URDF parsing (joint origins, axes, limits) | Already a Viser dependency — we get the parsed kinematic tree for free |
| **PyRoKi** | *Validation only*: independent FK/Jacobian to cross-check ours | Pip-installable, from the Viser team, IROS 2025 paper (arXiv:2505.03728). Gives numerical proof our from-scratch kinematics is correct |
| **matplotlib** | Tracking-error plots for the artifact | Standard |

### The one contested decision (D1): who computes FK and the Jacobian?

This was the fork in the road, worth understanding because it shows the framework in action:

- **Pinocchio** (the standard robotics kinematics library) was the original plan — it's
  what a research lab would reach for. Exploration killed it on criterion 2: as of 2026
  its pip wheels are **Linux-only**; on our Mac it requires a conda environment, which
  makes the repo harder for anyone else to reproduce.
- **Writing FK/Jacobian ourselves** wins criteria 2 and 3 (pure pip; stronger story)
  but loses on 4 — we could get it wrong. For a 7-joint serial chain the code is small
  (~100 lines: a loop of matrix products, plus a closed-form Jacobian column per joint),
  so the risk is bounded.
- **Resolution: do both** — write it ourselves, and retire the risk with a validation
  script comparing our FK/Jacobian to PyRoKi's on random configurations (plus
  finite-difference checks on the Jacobian). Milestone M2 exists solely to close this
  risk before the IK solver is built on top.

The general lesson: tooling decisions weren't taste — each traces to a criterion, and
the risky choice comes bundled with the plan that de-risks it.

---

## 7. Where this stage hands off

With the problem understood, the pipeline ahead is:

1. **Stage 1 (this):** problem, terms, I/O, success criteria, tooling rationale ✓
2. **Stage 2:** the math of the solver — pose error via the SE(3) log map, the geometric
   Jacobian column formula, why damped least squares, the nullspace trick (DESIGN.md §2)
3. **Stage 3:** setup — environment, URDF generation, repo creation (milestone M0)
4. **Stages 4+:** build outward along milestones M1→M6: render → verify kinematics →
   static IK → tracking → interactivity → artifact

Each stage only starts when the previous one's risk is retired — that's the reason the
milestones are ordered the way they are.
