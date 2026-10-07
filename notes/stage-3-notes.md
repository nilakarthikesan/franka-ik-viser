# Stage 3 Notes — The DLS Solver (M3)

Companion notes to [DESIGN.md §2](../DESIGN.md). This is the stage the assignment is
actually about: everything before it (URDF, FK, Jacobian) was scaffolding so that this
one algorithm could be written and trusted. Stage 2 ended by numerically checking `fk`, `jacobian`,
and `pose_error` correct, which changes the character of the work here — bad behavior in
this stage is a **tuning** problem, not a correctness hunt.

---

## 1. What the solver is solving

We have `fk(q)`: seven joint angles in, hand pose out. We need the inverse: given a
desired pose \( T_{des} \), find \( q \). There is no formula to invert — FK is a
product of seven rotation matrices, so \( q \) is buried inside nested sines and
cosines. So we don't invert it. We **sneak up on the answer**, using the Jacobian as
a local linear stand-in for the nonlinear FK.

For a small joint change \( \Delta q \), the hand moves by approximately
\( J(q)\,\Delta q \). So "reduce the pose error \( e \)" becomes a *linear* request:

\[ J\,\Delta q \approx e, \qquad e = \texttt{pose\_error}(T_{cur}, T_{des}) \]

Solve for \( \Delta q \), step, recompute \( J \) and \( e \) at the new configuration,
repeat. Because the linearization is re-made at every iteration, its error shrinks as
we converge. This is Newton's method in robot clothing.

## 2. Why *damped* least squares

\( J \) is 6×7 — wider than tall — so \( J\Delta q = e \) has no unique solution and
must be solved in a least-squares sense (the pseudoinverse \( J^{+} \)). The
pseudoinverse alone has a dangerous failure mode: near a **singularity** (arm
stretched straight, wrist axes aligned) some direction of hand motion becomes
unreachable, \( J \) loses rank, and \( J^{+} \) divides by nearly zero. The commanded
\( \Delta q \) explodes and the arm whips.

DLS changes the objective from "achieve \( e \)" to "achieve \( e \) **while staying
small**":

\[ \min_{\Delta q}\; \|J\Delta q - e\|^2 + \lambda^2\|\Delta q\|^2
\quad\Longrightarrow\quad
\Delta q = J^\top\big(J J^\top + \lambda^2 I\big)^{-1} e \]

The \( \lambda^2 I \) term keeps the 6×6 matrix invertible no matter what \( J \) does.
In directions the arm moves easily, damping is negligible; in directions it nearly
cannot move, damping gracefully gives up instead of dividing by zero. (Identical in
form to Levenberg–Marquardt regularization; the classic robotics reference is Buss
2004.)

Implementation detail: we never form an inverse for the step itself —
`dq = J.T @ np.linalg.solve(J @ J.T + lam2 * I, e)`. Solving a 6×6 system is cheaper
and better conditioned than inverting one.

### Weighting position vs. rotation

\( e \) mixes metres and radians, which are not comparable. We scale rows:
\( J_w = W J \), \( e_w = W e \) with \( W = \mathrm{diag}(w_p, w_p, w_p, w_r, w_r, w_r) \),
then run DLS on the weighted pair. Setting \( w_r = 0 \) yields position-only IK for
free (a stretch goal in DESIGN.md §8), and the ratio \( w_p : w_r \) decides which
objective yields when both cannot be satisfied.

## 3. Redundancy: spending the spare joint deliberately

A pose pins down 6 numbers; the FR3 has 7 joints. The leftover freedom is a
one-parameter family of arm shapes reaching the *same* hand pose — the elbow can orbit
while the hand sits still. Left alone, long tracking runs let the elbow drift into
contorted configurations. So we add a secondary objective, **projected into the
nullspace** so the task never feels it:

\[ \Delta q_{null} = \big(I - J_w^{+} J_w\big)\, k_{posture}\,(q_{home} - q) \]

\( (I - J^{+}J) \) is the nullspace projector: it filters any desired joint motion down
to the component that leaves the end effector stationary. Important subtlety learned
the hard way (§8): the projector must use the **true (SVD) pseudoinverse**, not the
damped one — the damped "projector" is not a projector, and it leaks the posture pull
into the end-effector task.

## 4. The loop, and its two operating modes

`solve_step(q, T_des)` is one iteration:
error → Jacobian → weighted DLS step → nullspace posture term → scale by \( \alpha \)
→ **clip step magnitude** → integrate → **joint-limit handling**.

Two practical refinements beyond the textbook formula, both earned by measured
failures (§8):

- **Step-magnitude clip** (`max_step`, per-joint radians): bounds joint velocity when
  the target is far away or jumps (the interactive gizmo can teleport), keeping the
  demo smooth and the linearization inside its region of validity.
- **Clamped DLS at joint limits**: a joint that would leave its limits is taken
  exactly to the boundary and *locked*, and the step is re-solved for the remaining
  joints against the residual error. Naive clipping instead lets one saturated joint
  stall the entire solve (this was the dominant T2 failure mode).

Two modes consume the same step:

- **Static solve** (`solve`): iterate until \( \|e_{pos}\| \) and \( \|e_{rot}\| \) fall
  below tolerance, or `max_iters` runs out. Differential IK is a *local* method, so a
  start can sit in a basin no descent direction escapes; for static targets we retry
  from seeded-random configurations (`restarts`). Tracking never restarts — it must
  stay continuous.
- **Tracking** (M4/M5): exactly **one** step per rendered frame at ~50 Hz while the
  target moves. Legitimate because the target barely moves between frames, and it is
  *why* the output is smooth: consecutive configurations are one bounded step apart by
  construction.

## 5. Tuning: the four knobs and what they trade

The math fixes the *form* of the update; it says nothing about these values. Behavior
does.

| Knob | Symbol | Too small | Too large |
|---|---|---|---|
| Damping | \( \lambda \) | jitter/whip near singularities, huge steps | sluggish, tracking lags the target, residual error |
| Step scale | \( \alpha \) | slow convergence | overshoot, oscillation around the target |
| Pos/rot weights | \( w_p, w_r \) | that objective is ignored | that objective dominates the other |
| Posture gain | \( k_{posture} \) | elbow drifts over long runs | posture pull leaks into the task, degrades tracking |

Defaults (validated by the P4 sweep in §8): \( \lambda = 10^{-2} \), \( \alpha = 1.0 \),
\( w_p = w_r = 1 \), \( k_{posture} = 0.05 \), `max_step` = 0.1 rad. All exposed as
Viser GUI sliders in M5 — tuning is inherently interactive, and dragging \( \lambda \)
while watching the arm is both the fastest way to build intuition and a good demo.

## 6. Convention alignment (carried over from Stage 2)

DESIGN.md §1 originally sketched the error as the body-frame SE(3) log
\( \log_6(T_{ee}^{-1}T_{des}) \). Stage 2 deliberately chose the **world-frame
decoupled** convention instead — \( e = [\,p_{des}-p_{cur};\ \mathrm{rotvec}(R_{des}R_{cur}^\top)\,] \)
— because it pairs with our world-frame geometric Jacobian, needs only an SO(3) log,
and allows independent position/rotation weighting. Stage 2's check F already verified numerically
the pairing consistent to first order. Both conventions are valid; *mixing* them is
the classic differential-IK bug, so the solver docstrings state the choice explicitly.

## 7. Test plan

Two layers, run by [scripts/test_ik.py](../scripts/test_ik.py).

**Isolated solver tests** (does the algorithm behave?):

| # | Test | Why it matters |
|---|------|----------------|
| T1 | Fixed point: at the solution, \( \|\Delta q\| \approx 0 \) | a solver that drifts when already correct is broken |
| T2 | Static convergence: random reachable targets (generated as `fk(q_target)` so reachability is guaranteed) from random starts; require < 1 mm / < 0.5° | the M3 acceptance bar from DESIGN.md §1 |
| T3 | Joint limits respected at every iteration | output must be physically legal |
| T4 | Step bound: \( \max_i|\Delta q_i| \le \) `max_step` always | per-step numerical joint-update bound |
| T5 | Singularity robustness: start stretched/aligned, target far; no NaN/Inf, steps stay bounded | the reason damping exists |
| T6 | Nullspace behaviour: with the task converged, posture pull reduces \( \|q - q_{home}\| \) while pose error stays below tolerance | proves the projector really is task-neutral |
| T7 | Unreachable target (2 m away): error plateaus, no divergence, limits respected | graceful failure |
| T8 | Determinism: same seed and inputs give identical output | reproducibility for the writeup |

**Pipeline tests** (does it work with the rest of the design?):

| # | Test | Why it matters |
|---|------|----------------|
| P1 | Tracking a circular SE(3) path (M4 preview): one step per frame at 50 Hz, report max/RMS position and rotation error | the assignment's core ask |
| P2 | Smoothness of the tracking run: per-frame joint deltas bounded, no discontinuities | what "smooth output" means quantitatively |
| P3 | Consumes only Stage 2's validated primitives; Stage 1/2 checks (`verify_setup.py`, `validate_kinematics.py`) still pass afterwards | no regressions across stages |
| P4 | Tuning sweep over \( \lambda \times \alpha \) on the tracking task, reported as a table | justifies the default gains with data rather than assertion |

## 8. Results: two textbook-formula failures found and fixed

The first run of the suite failed three tests — all traced to the *textbook* update,
rather than to the kinematics implementation in the sampled validation checks. Further kinematics checks would still be appropriate if later evidence suggested a discrepancy.

**Failure 1 — naive limit clipping stalls solves (T2: 41/50 from home, 21/50 from
random).** Diagnosis: failed solves sat pinned at a joint limit (typically joint 4,
the elbow, or joint 7, the wrist) with *more iterations not helping* — plain
`clip(q + dq)` lets one saturated joint invalidate the whole step direction. Fix:
**clamped DLS** — saturated joints are taken exactly to their boundary, locked, and
the step re-solved for the free joints against the residual error.

**Failure 2 — the damped "projector" leaks (T6: only 6/10 held pose; posture ON made
T2 *worse*, 30/50 vs 37/50).** With \( J^{+} \) approximated by the damped inverse,
\( (I - J^{+}J) \) is not a projector, and the posture pull bleeds into task space as
a standing pose error. Fix: use the true SVD pseudoinverse (`np.linalg.pinv`,
rcond=1e-6) for the projector only; the task step keeps its damping.

**Residual failures — genuine local minima (T2 at 49/50 and 40/50 after the fixes).**
More iterations never helped; these targets need a different solution branch than the
start's basin reaches. Fix: seeded random **restarts** for static solves only.
Measured: restarts=12 → **50/50 from q_home and 50/50 from random starts**; only
failed first attempts pay the cost (median iterations unchanged).

Final suite (all 12 pass):

```
T1 fixed point            max |dq| at solution = 9.4e-17 rad
T2 static convergence     50/50 from q_home, 50/50 from random starts
                          (< 1 mm / < 0.5 deg; median ~30 / ~80 iters)
T3 joint limits           zero violations across all iterations
T4 step bound             max |dq| = 0.100 = max_step, never exceeded
T5 singularity            stretched start, unreachable-direction target:
                          finite for lambda in {1e-4, 1e-2, 1e-1}, steps bounded
T6 nullspace posture      9/10 moved toward q_home, 10/10 held pose in tolerance
T7 unreachable target     error plateaus at 1.21 m, finite, in-limits, no divergence
T8 determinism            repeated solves bit-identical
P1 circle tracking        max 0.006 mm / 0.0001 deg, RMS 0.005 mm  (bar: 1 mm / 0.5 deg)
P2 smoothness             max per-frame |dq| = 0.0048 rad, all configs in-limits
P4 sweep                  see below
```

P4 tuning sweep (max tracking error, 8 s circle @ 50 Hz):

| \( \lambda \) | \( \alpha=0.5 \) | \( \alpha=1.0 \) |
|---|---|---|
| 0.001 | 1.89 mm | **0.005 mm** |
| 0.01 (default) | 1.89 mm | **0.007 mm** |
| 0.05 | 2.01 mm | 0.074 mm |
| 0.1 | 2.39 mm | 0.287 mm |
| 0.3 | 6.90 mm | 2.59 mm |

Reading the sweep: \( \alpha = 1 \) beats 0.5 across the board (half-steps make
tracking lag a moving target); damping costs accuracy monotonically, exactly as §5
predicted — but even \( \lambda = 0.1 \) stays under the 1 mm bar, so the default
\( 10^{-2} \) buys singularity insurance nearly for free. This table is the "tuning,
not correctness" claim, quantified.

P3 regression: after all changes, `validate_kinematics.py` (9/9) and
`verify_setup.py` (5/5) still pass.

M3 is done: the solver converges to random reachable targets well inside the
< 1 mm / < 0.5° bar, tracks a moving trajectory at micrometre-level error, respects
limits and step bounds by construction, and fails gracefully when asked for the
impossible. Handoff to M4/M5: `trajectory.py` formalizes the paths (the tracking
harness in `test_ik.py` is its prototype), and the Viser app wires `solve_step` to
the render loop with the §5 knobs on sliders.
