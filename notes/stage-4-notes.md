# Stage 4 Notes — Trajectories, the Viser App, and Live Tracking (M4 + M5)

Companion notes to [DESIGN.md §3.3–3.5](../DESIGN.md). This is the stage where the
project becomes *visible*: the validated solver (M3) is wired to a moving target and a
browser renderer, so the FR3 physically traces reference paths. Stages 1–3 built and
proved every piece; here we assemble them and add the two things the assignment asks
for by name — sampled pose trajectories and Viser visualization — plus the offline
recorder that produces the artifact.

Because M3 already proved `solve_step` tracks a circle at 0.006 mm, the risk in this
stage is *not* the IK math. It is (a) building trajectories that stay reachable, (b)
wiring Viser correctly (joint ordering, quaternion conventions, gizmo feedback loops),
and (c) keeping a real-time loop smooth. The tests target exactly those.

---

## 1. Trajectories: what to sample and why

A trajectory is a function \( T_{des}(t) \in SE(3) \) — a moving pose the hand should
follow. All three paths live in a reachable region in front of the base
(center \( c = [0.45, 0, 0.45] \) m, radius \( \le 0.12 \) m, well inside the ~0.85 m
reach envelope and away from the singular stretched-out boundary — the same region the
M3 tracking test already validated).

- **circle** — center + \( [0, r\cos\theta, r\sin\theta] \), \( \theta = 2\pi t/T \):
  a circle in the YZ plane (the vertical plane facing the robot). Simplest path;
  constant curvature; the M3 prototype (`circle_path` in `test_ik.py`) already tracks
  it at micrometre error. Position-only motion, orientation held constant.
- **figure_eight** — a Gerono lemniscate: \( y = r\cos\theta \), \( z = \tfrac{r}{2}\sin 2\theta \).
  Self-crossing path with sign-changing curvature and a speed that varies around the
  loop — a harder tracking test than the circle, still smooth and closed.
- **lissajous** — a 3D path (\( x, y, z \) each a sinusoid at a different frequency)
  **with orientation that slerps** between keyframes over the cycle. This is the only
  path that exercises *rotation* tracking; the others hold orientation fixed. Slerp
  (spherical linear interpolation, via scipy's `Slerp`) gives a constant-angular-speed,
  shortest-arc interpolation between orientation keyframes — the correct way to move
  through SO(3), as opposed to interpolating Euler angles or matrix entries.

Default orientation is "gripper pointing down/forward", taken as the orientation of
`fk(q_home)` so frame 0 of every path is trivially reachable from home. Each path is
periodic with period \( T \) (seconds), so playback loops seamlessly.

Interface: each is a pure function `(t, period, radius) -> (4,4)` and they are
collected in a `TRAJECTORIES` dict keyed by the GUI dropdown labels.

## 2. The Scene wrapper (visualizer.py)

A `Scene` class hides all Viser calls behind a small interface the app loop uses:

- `set_joint_config(q)` — maps our 7 arm angles by joint *name* into the actuated
  vector ViserUrdf expects (finger joint appended, fixed open), then `update_cfg`.
  Mapping by name, not position, is deliberate: it is robust to ViserUrdf ordering the
  actuated joints differently than our chain (a bug class we avoided in M2 too).
- `target_pose() -> (4,4)` — reads the `transform_controls` gizmo (position + wxyz
  quaternion) in interactive mode.
- `set_target(T)` — moves the gizmo to a pose; used both to show the playback target
  and, critically, to *snap the gizmo to the current EE pose* when switching into
  interactive mode so the arm never jumps (see §3).
- `draw_reference(points)` — the reference path as a Catmull–Rom spline.
- `append_ee_trace(point)` / rolling buffer — the achieved EE path as live line
  segments, capped to a fixed length so it stays a "comet tail" and memory is bounded.
- `set_error(pos_mm, rot_deg)` — writes the two disabled number readouts.
- GUI handles exposed as attributes: mode dropdown, trajectory dropdown, speed slider,
  pause/reset buttons, and the four solver sliders (damping, step scale, posture gain,
  max_step) wired to mutate the live `IKConfig`.

Quaternion convention: Viser uses wxyz; scipy uses xyzw. All matrix<->quaternion
conversion goes through two tiny helpers (`mat_to_wxyz`, `wxyz_to_mat`) so the
convention lives in exactly one place — the same discipline that kept M2's comparisons
honest.

## 3. The app loop (app.py)

```mermaid
flowchart TB
    subgraph loop [50 Hz loop]
      target["target = trajectory(t) if playback else scene.target_pose()"]
      step["q = solver.solve_step(q, target)"]
      draw["scene.set_joint_config(q); update trace + error"]
      target --> step --> draw --> target
    end
    gui["GUI callbacks: mode / trajectory / speed / pause / reset / gains"] -.mutate state.-> loop
```

Design points:

- **Fixed ~50 Hz tick** via sleeping the remainder of each 20 ms frame; the trajectory
  clock advances by `speed * dt` so the speed slider changes path velocity without
  changing the frame rate.
- **One `solve_step` per frame** — never a full `solve` — this is what makes motion
  smooth (M3 §4: consecutive configs are one bounded step apart).
- **Mode switch without a jump.** On entering interactive mode, snap the gizmo to the
  current EE pose so `target_pose()` returns where the arm already is; on entering
  playback, resume from the trajectory clock. Either way the target never
  discontinuously jumps, so the arm never lurches.
- **Reset** restores `q_home` and clears the trace. **Pause** freezes the clock but
  keeps the server responsive.
- Runs headless-friendly (`python -m franka_ik.app`), printing the localhost URL.

## 4. The recorder (record.py)

Headless (no browser): runs a chosen trajectory for N periods at 50 Hz through the same
solver, logging per frame `t`, target position, achieved position, and the position/
rotation errors. Outputs to `outputs/`:

- `tracking_<traj>.csv` — the raw log (reproducible source for any later plot).
- `tracking_<traj>.png` — two panels: (i) reference vs. achieved path in 3D/2D, (ii)
  position (mm) and rotation (deg) error vs. time. This is the quantitative artifact
  for the README and report (M6).

Sharing the exact solver + trajectory code with the live app means the recorded numbers
are the same ones the demo shows — no separate "benchmark" path to drift out of sync.

## 5. Test plan

Same two-layer pattern as M2/M3, in [scripts/test_trajectory.py](../scripts/test_trajectory.py).

**Isolated (trajectory correctness):**

| # | Test | Why |
|---|------|-----|
| TR1 | every sampled pose is a valid SE(3): R orthonormal, det +1 | a malformed target would poison the solver |
| TR2 | continuity: consecutive samples at 50 Hz are close in position and rotation | discontinuities would demand infinite joint velocity |
| TR3 | periodicity: \( T_{des}(0) = T_{des}(\text{period}) \) | playback must loop seamlessly |
| TR4 | reachability: a static `solve` converges to samples spanning each path | proves the path lives in the workspace, not asserted |

**Pipeline (tracking + integration):**

| # | Test | Why |
|---|------|-----|
| TP1 | track each trajectory one step/frame @ 50 Hz; max/RMS error under 1 mm / 0.5° | the assignment's core deliverable, per path |
| TP2 | smoothness: per-frame \|Δq\| ≤ max_step; all configs in-limits | quantifies "smooth" |
| TP3 | recorder produces CSV + PNG with sane contents | the artifact actually generates |
| TP4 | regression: M2 (`validate_kinematics`) and M3 (`test_ik`) gates still green | no cross-stage breakage |

Browser verification (manual, screenshotted): robot tracking the circle in playback,
the drag gizmo moving the arm in interactive mode, and the full GUI present.

## 6. Results

Built the three stubs plus the recorder: `trajectory.py`, `visualizer.py`, `app.py`,
`record.py`. All Stage 4 tests pass and the M2/M3 gates still pass (no regressions).

### Isolated + pipeline tests (`scripts/test_trajectory.py`, 11/11)

> Note: this is the original single-surface run. §9 (drawing surfaces) supersedes it —
> the suite now loops **both** `table` and `wall` for every check and adds TR5
> (pen-into-plane geometry), for **18/18** total. The numbers below still hold for the
> `table` surface.

```
TR1 valid SE(3):   max orthonormality/det error = 8.88e-16
TR2 continuity:    max per-frame 2.75 mm / 0.172 deg   (smooth at 50 Hz)
TR3 periodicity:   max |pose(0) - pose(period)| = 4.34e-16
TR4 reachability:  circle 12/12; figure-8 12/12; lissajous 12/12
TP1 track circle:     max 0.006 mm / 0.000 deg,  RMS 0.005 mm
TP1 track figure-8:   max 0.004 mm / 0.000 deg,  RMS 0.002 mm
TP1 track lissajous:  max 0.011 mm / 0.001 deg,  RMS 0.006 mm
TP2 smoothness:    max per-frame |dq| <= 0.009 rad, all configs in-limits
TP3 recorder:      CSV 401 rows + PNG produced
```

Every path is tracked ~2-3 orders of magnitude under the 1 mm / 0.5 deg bar. The
lissajous path — the only one with changing orientation — confirms rotation tracking
(0.001 deg), not just translation. This matches the M3 finding that with defaults
(lambda=0.01, alpha=1.0) consecutive-target tracking is essentially exact.

### Recorded artifact (`python -m franka_ik.record --all --periods 2`)

| trajectory | max pos err | max rot err | files |
|---|---|---|---|
| circle | 0.0065 mm | 0.0001 deg | `outputs/tracking_circle.{csv,png}` |
| figure-8 | 0.0036 mm | 0.0001 deg | `outputs/tracking_figure-8.{csv,png}` |
| lissajous | 0.0109 mm | 0.0010 deg | `outputs/tracking_lissajous.{csv,png}` |

The figure-8 plot (reference vs. achieved overlap on the left, periodic error-vs-time on
the right) is representative (CSVs regenerate into gitignored `outputs/`; a copy of each
plot is committed under `docs/images/`):

![figure-8 tracking artifact](../docs/images/tracking_figure-8.png)

### Live app (browser, `PYTHONPATH=src python -m franka_ik.app`)

Playback mode tracking the circle, with the blue reference spline, the orange live EE
trace, the target gizmo, and the full GUI (mode/trajectory dropdowns, speed, pause/reset,
the four solver sliders, and the mm/deg readouts showing ~0.003 mm live):

![app circle playback](../docs/images/app_circle.png)

Selecting figure-8 redraws the reference and the arm traces the lemniscate:

![app figure-8](../docs/images/app_figure8.png)

Interactive mode: switching modes snaps the gizmo to the current EE pose (no arm jump),
and the gizmo's translate/rotate handles become the IK target:

![app interactive gizmo](../docs/images/app_interactive.png)

### Regression gates (still green)

- M2 `scripts/validate_kinematics.py`: 9/9 (FK vs yourdfpy/PyRoKi, Jacobian vs finite
  differences and JAX, all < 1e-5).
- M3 `scripts/test_ik.py`: 12/12 (T1-T8 isolated + P1/P2/P4 pipeline).

## 7. Issues found and fixes

- **Packaging / running as `-m`.** This repo lives under an iCloud-synced `~/Desktop`
  path. The setuptools *editable* install (`pip install -e .`) writes an
  `__editable__*.pth`, but `site` **silently skips a `.pth` it can't open**
  (`addpackage` swallows `OSError`), and iCloud intermittently makes the file
  unreadable — so `python -m franka_ik.app` failed nondeterministically. Fix: run the
  package modules with `PYTHONPATH=src` (the same `src`-on-path convention the
  `scripts/*.py` already use with `sys.path.insert`), which is deterministic and needs
  no install. `pyproject.toml` is kept for normal (non-synced) environments where
  `pip install -e .` works fine.
- **Error readout displayed `0`.** Viser's `add_number` infers display precision from
  the initial value, so `0.0` rendered sub-millimetre errors as `0`. Fix: pass
  `step=1e-4` to the two readouts so the true ~0.003 mm value is visible.
- **Trace/reference bookkeeping on mode/trajectory switch.** Changing trajectory or
  mode now clears the rolling EE trace and (for trajectory) redraws the reference, so
  stale geometry never lingers.

## 8. Frame-by-frame visual walkthrough (what to look for)

Whether the design is *actually solving the task* is visible in six things:

1. **Orange EE trace sits on the blue reference spline.** Blue = commanded path;
   orange = where the hand actually went. Overlap = the IK is tracking. Any blue
   peeking out = tracking error you can see.
2. **The RGB target gizmo and the fingertip's own axes coincide** as the target sweeps.
3. **The `Position (mm)` / `Rotation (deg)` readouts stay tiny** (~0.003 mm / 0.0001°).
4. **Motion is continuous** — the arm reconfigures smoothly, never teleports (the
   one-bounded-step-per-frame guarantee from M3).
5. **Joint limits respected** — no joint slams to a stop.
6. **Interactive mode:** switching snaps the gizmo to the fingertip (no jump), then the
   gizmo drives the arm.

### The sequence: one full circle loop on the `table` surface (default gains)

These six frames are rendered by `scripts/record_gif.py`, which drives the **exact**
live pipeline (`trajectory.pose` → `solve_step` at 50 Hz) and draws the arm skeleton,
the blue reference, the growing orange EE trace, and the red gripper "pen" axis. Using
the headless renderer (rather than browser grabs) gives an unoccluded 3D view of the
horizontal `table` loop; for the real app render see the live screenshots in §9.

| frame | what the robot is doing |
|---|---|
| ![f0](../docs/images/frames_table/frame00.png) **0 — start** | Arm reaching down to the start of the horizontal loop; the red pen axis points straight **down into** the drawing plane. Trace empty, err ≈ 0.000 mm — the solver is seeded on-path (in the app this convergence-from-home takes a fraction of a second, step-bounded and smooth). |
| ![f1](../docs/images/frames_table/frame01.png) **1** | Target has advanced ~⅕ of the way round; the orange trace begins laying an arc directly over the blue reference. |
| ![f2](../docs/images/frames_table/frame02.png) **2** | ~⅖ of the loop drawn, still glued to the blue circle. Shoulder and elbow visibly reposition to keep the fingertip on the path while the pen stays vertical. |
| ![f3](../docs/images/frames_table/frame03.png) **3** | Most of the ring laid down and coincident with the reference; err ≈ 0.007 mm. |
| ![f4](../docs/images/frames_table/frame04.png) **4** | Far side of the loop — the arm has adopted a distinctly different posture yet the fingertip is still exactly on the circle. This is the redundant 7th DOF keeping the *task* satisfied while the *posture* changes. |
| ![f5](../docs/images/frames_table/frame05.png) **5 — loop closed** | Orange trace overlays the entire blue circle. One clean period completed, pen still pointing into the plane throughout. |

Across the whole loop the position error stays in the single-digit-micrometre band
(max 0.008 mm for `circle · table`, per `record.py`), the numeric confirmation of the
visual overlap. The animated version (both surfaces) is the demo GIF in the README.

*Real-app money shot (live Viser render + `Position (mm)` readout) lives in §9:*
[`docs/images/surface_table.png`](../docs/images/surface_table.png).

### Live stress test: does the tuning actually do what M3 claims?

To prove the GUI sliders are wired to the solver and that DLS behaves as the theory
predicts, I dragged **Damping (λ) from 0.01 → 0.5** while the circle kept playing:

| λ (damping) | live position error | live rotation error |
|---|---|---|
| 0.01 (default) | ~0.004 mm | 0.0001° |
| **0.5 (max)** | **~5.9–6.9 mm** | **~0.11°** |
| back to 0.01 | ~0.004 mm | 0.0001° |

![high damping](../docs/images/frames/frame08_highdamping.png)

*λ = 0.5: the error jumps ~1500×, and you can see the gizmo (target) pull ahead of the
fingertip — heavy damping shrinks each Δq step, so the arm lags the moving target.
(Captured live on the earlier vertical circle; the damping trade-off is
geometry-independent, so the demonstration still stands.)*

This is exactly the accuracy-vs-conditioning trade-off from M3 §tuning, now visible in
real time: damping buys robustness near singularities at the cost of tracking lag, and
the chosen default (λ = 0.01) sits in the sweet spot. Restoring the slider snaps the
error straight back to micrometres, confirming the whole path
`slider → IKConfig → solve_step → rendered arm → readout` is live and correct.

## 9. Drawing surfaces: `table` vs `wall`

### The observation that motivated this

Watching the frame-by-frame loop in §8, the numbers were perfect (< 0.006 mm) but the
motion did **not** read as *drawing*. The reason was geometric, not a solver bug: the
original circle lived in a vertical plane while the gripper kept the home orientation
(pointing straight **down**). So the tool's approach axis lay **flat in** the drawing
plane instead of pointing **into** it — the fingertip skated around a ring edge-on, like
waving a pen sideways rather than pressing it to paper.

To localize this precisely I compared the tool approach axis (FK `T_tcp`'s local +z)
against the plane normal. Flat-in-plane ⇒ ~90°; pen-into-plane ⇒ ~0°. This is now a
permanent regression check, **TR5** in `scripts/test_trajectory.py`, so the geometry can
never silently regress again. It also answers "which stage owns this?" — it's the
**trajectory** stage (plane + tool orientation), not the FK, Jacobian, or solver, all of
which were already validated.

### The fix: pick the plane and the tool orientation together

A trajectory is now `shape × surface`. The **shape** is the in-plane curve
(`circle`, `figure-8`, `lissajous`); the **surface** fixes both *where* the plane sits
and *how the tool is held* so the pen always stabs into the disk:

| surface | plane | gripper "pen" points | center | radius | reads as |
|---|---|---|---|---|---|
| **table** | horizontal (XY) | straight **down** (−z) | (0.45, 0, 0.35) | 0.12 m | drawing on a tabletop |
| **wall** | vertical (YZ), faces robot | **forward** (+x) | (0.50, 0, 0.50) | 0.10 m | drawing on a whiteboard |

Both centers/radii were chosen empirically for full reachability across all three shapes
(the earlier "wall" at (0.45,0,0.45)/0.12 m poked past the FR3's wrist limits — max
11.8 mm error; the shipped (0.50,0,0.50)/0.10 m sits at < 0.1 mm everywhere). `circle`
and `figure-8` hold a constant tool orientation; `lissajous` adds a small ±0.35 rad slerp
wobble about the surface pose so it still exercises **rotation** tracking.

### Design preview (both surfaces, pen axis drawn)

`scripts/preview_orientations.py` renders the arm skeleton, the reference circle, and the
gripper "pen" arrows at points around each loop. Orange arrows stabbing *through* the
blue disk = drawing; lying flat = waving. Both are fully reachable and the pen sits at
**0.0° to the plane normal** (into the disk) on both surfaces:

![orientation preview](../docs/images/orientation_preview.png)

```
TABLE (horizontal, pen down):  reachable=True, max err 0.006 mm, tool-vs-normal 0.0°
WALL  (vertical, pen forward): reachable=True, max err 0.067 mm, tool-vs-normal 0.0°
```

### Live in the app (new `Surface` dropdown)

Both versions ship in the live demo. A **Surface** dropdown (next to **Trajectory**) lets
whoever is interacting switch between them on the fly; changing it clears the trace and
redraws the reference, so playback restarts cleanly on the new surface. Interactive-gizmo
mode is unaffected (the target is whatever the user drags).

| `table` — draw flat, gripper down | `wall` — draw upright, gripper forward |
|---|---|
| ![table](../docs/images/surface_table.png) | ![wall](../docs/images/surface_wall.png) |

Left: the arm reaches down and the fingertip traces a horizontal loop, tool pointing into
the tabletop. Right: the arm reaches forward and traces the vertical loop (orange arc),
tool pointing into the wall. Live tracking readout stayed at ~0.003–0.006 mm while
switching.

### Tracking holds on both surfaces

`PYTHONPATH=src python -m franka_ik.record --all` records every `shape × surface` combo.
All twelve stay far under the 1 mm / 0.5° bar:

| combo | max pos err | combo | max pos err |
|---|---|---|---|
| circle · table | 0.008 mm | circle · wall | 0.031 mm |
| figure-8 · table | 0.006 mm | figure-8 · wall | 0.031 mm |
| lissajous · table | 0.011 mm | lissajous · wall | 0.013 mm |

`scripts/test_trajectory.py` now loops both surfaces for every isolated and pipeline check
(**18/18 pass**), including TR5 (pen-into-plane) and TR4 (reachability 12/12 per combo).
M2 (9/9) and M3 (12/12) regression gates still green.

### Verdict against the task

The assignment asks for an IK solver driving a Franka to **track sampled SE(3)
trajectories**, visualized live, with an **interactive** target. All of that is
demonstrated above: three trajectories tracked to < 0.011 mm / 0.001° (§6), the live
browser demo with playback + interactive gizmo modes, tuning that provably affects the
result, and the recorded CSV/plot artifacts. Design objective met.
