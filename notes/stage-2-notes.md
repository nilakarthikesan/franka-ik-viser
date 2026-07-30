# Stage 2 Notes — FK and the Jacobian, from scratch and proven correct (M2)

Companion notes to [DESIGN.md §2/§4-D1](../DESIGN.md). Stage 1 established *what* the
problem is; this stage designs the mathematical foundation everything else stands on:
forward kinematics and the geometric Jacobian, implemented by us in numpy inside
[robot_model.py](../src/franka_ik/robot_model.py), and **proven correct** before the IK
solver is allowed to build on them. If these two functions are right, differential IK
is a 30-line loop. If they're subtly wrong, nothing downstream will ever work and the
bug will be miserable to find — which is why validation is half of this stage.

---

## 1. The data we walk: the FR3 chain as it actually is

From the generated URDF (verified in Stage 1), base to hand looks like this:

```
fr3_link0 ──J1── fr3_link1 ──J2── ... ──J7── fr3_link7 ──fixed── fr3_link8
                                                     (fr3_joint8)    │
                                                                   fixed (fr3_hand_joint)
                                                                     │
                                                                  fr3_hand
                                                                     │
                                                                   fixed (fr3_hand_tcp_joint)
                                                                     │
                                                                 fr3_hand_tcp   ← EE frame
```

Facts that shape the implementation (all confirmed against the file, not assumed):

- **All 7 arm joints are revolute about their local z-axis** (`<axis xyz="0 0 1"/>`),
  each preceded by a fixed `<origin xyz rpy>` transform. We still implement rotation
  about a *general* axis — free correctness, and the code stays robot-agnostic.
- **Three fixed joints sit between joint 7 and the TCP** (`fr3_joint8`,
  `fr3_hand_joint`, `fr3_hand_tcp_joint`). Their transforms are constants — fold them
  into one suffix matrix at parse time.
- **The fingers are prismatic and excluded**: `fr3_finger_joint2` even `<mimic>`s
  joint 1. Our model is exactly the 7-DoF arm; fingers only matter for rendering.
- Other fixed frames (accelerometers, etc.) hang off the chain but don't affect the
  base→TCP path.

So at parse time the chain compiles down to, per arm joint \( i \):
a constant transform \( A_i \) (everything fixed since the previous joint, including
that joint's `<origin>`) and an axis \( \hat a_i \); plus one constant suffix
\( A_8 \) (joint-7 flange → TCP).

## 2. Forward kinematics from scratch

One revolute joint contributes \( A_i \cdot R(\hat a_i, q_i) \), where
\( R(\hat a, \theta) \) is the rotation about axis \( \hat a \) by angle \( \theta \)
(Rodrigues' formula):

\[ R = I + \sin\theta\, [\hat a]_\times + (1 - \cos\theta)\, [\hat a]_\times^2 \]

with \( [\hat a]_\times \) the 3×3 skew-symmetric cross-product matrix. The full FK is
just the chain product:

\[ T_{world \to tcp}(q) = A_1 R_1(q_1)\, A_2 R_2(q_2) \cdots A_7 R_7(q_7)\, A_8 \]

Seven 4×4 multiplies (plus constants). Implementation notes:

- Parse once in `__init__` (via yourdfpy's joint map — we already trust its *parsing*
  from Stage 1; what we're replacing is the *math*): store `A_i`, `axis_i`, limits,
  and the suffix. `fk(q)` is then pure numpy with no URDF in sight.
- The URDF's `rpy` is fixed-axis XYZ convention: \( R = R_z(y)R_y(p)R_x(r) \).
- **Keep the intermediate products.** The walk that computes FK also passes through
  every joint's world frame — exactly what the Jacobian needs. One function
  `fk_all(q)` returns the TCP pose *and* per-joint world position \( p_i \) + world
  axis \( \hat z_i \); `fk` and `jacobian` are thin wrappers.

## 3. The geometric Jacobian

\( J(q) \in \mathbb{R}^{6\times 7} \) maps joint velocities to the EE's **world-frame
twist** (linear velocity on top, angular below). The intuition for one column: crank
joint \( i \) alone and the whole arm beyond it rigidly rotates about that joint's
world axis \( \hat z_i \) through its world position \( p_i \). A point rigidly
rotating about an axis moves at \( \omega \times r \), so:

\[ J_{\cdot i} = \begin{bmatrix} \hat z_i \times (p_{tcp} - p_i) \\ \hat z_i \end{bmatrix},
\qquad \hat z_i = R_{world \to i}\, \hat a_i \]

No calculus needed at runtime — it's closed-form geometry read off the FK
intermediates. This is the classic "geometric Jacobian for revolute serial chains"
(Siciliano & Khatib, *Springer Handbook of Robotics*, ch. 2; Buss 2004 tutorial).

### The convention trap (read this twice)

A Jacobian is only meaningful *paired with an error convention*. Ours is the
**world-frame (spatial) convention**, so the IK error in M3 must be:

\[ e = \begin{bmatrix} p_{des} - p_{tcp}(q) \\ \mathrm{rotvec}\!\left(R_{des} R_{tcp}(q)^\top\right) \end{bmatrix} \]

— position error in world coordinates, orientation error as the world-frame rotation
vector (axis·angle) that carries current orientation to desired. Pinocchio, for
comparison, defaults to body-frame (`LOCAL`) twists and the full SE(3) `log6`; both
conventions work, but *mixing* them produces an IK that spirals or converges to the
wrong orientation — the single most common differential-IK bug. Decision: decoupled
world-frame error, because it pairs with the geometric Jacobian above, needs only a
rotation log (no SE(3) V-inverse), and lets position/rotation be weighted
independently in the solver.

### The rotation log (the one nontrivial formula)

\( \mathrm{rotvec}(R) \): with \( \theta = \arccos\big((\mathrm{tr}\,R - 1)/2\big) \),

\[ \mathrm{rotvec}(R) = \frac{\theta}{2\sin\theta} \begin{bmatrix} R_{32}-R_{23} \\ R_{13}-R_{31} \\ R_{21}-R_{12} \end{bmatrix} \]

Numerical care at the two ends: as \( \theta \to 0 \) use the Taylor limit (the
prefactor → 1/2); near \( \theta = \pi \) the formula degenerates and the axis must be
recovered from the diagonal of \( R \). We implement the small-angle branch and clamp
the arccos argument; the \( \pi \) branch matters little for tracking (errors are
small every frame) but the static solver can start far away, so it's included.

## 4. Proving it correct — the actual M2 gate

Four independent checks in `scripts/validate_kinematics.py` (design D1's promise),
each attacking a different failure mode, over N = 100 random configurations sampled
uniformly *within joint limits*:

| # | Check | What it catches | Tolerance |
|---|-------|-----------------|-----------|
| A | our `fk(q)` vs yourdfpy `get_transform` | our chain-walk math (same parse, independent math) | 1e-8 m / 1e-8 rad |
| B | our `fk(q)` vs PyRoKi `forward_kinematics` | parse *and* math (fully independent implementation: its own URDF parser + jaxlie twist exponentials) | 1e-6 m / 1e-6 rad |
| C | our `jacobian(q)` vs central finite differences of our own `fk` | the Jacobian formula and its frame conventions | 1e-5 |
| D | (bonus) our `jacobian(q)` vs `jax.jacfwd` through PyRoKi FK | independent analytic Jacobian | 1e-5 |

Details that make these checks honest:

- **Check B API facts** (read from the installed PyRoKi source, not docs):
  `pk.Robot.from_urdf(urdf)` then `robot.forward_kinematics(cfg)` returns
  `(num_links, 7)` poses as `wxyz_xyz`, ordered by `robot.links.names` — index
  `fr3_hand_tcp` there. Because of the finger mimic, PyRoKi's actuated config has
  **8 entries** (7 arm + 1 finger), so `cfg = [q, 0.0]` — get this wrong and shapes
  still match nothing or, worse, silently permute.
- **Compare rotations geodesically**, never elementwise: quaternions double-cover
  rotations (\( \mathbf{q} \equiv -\mathbf{q} \)), so check
  \( \|\mathrm{rotvec}(R_{ours}^\top R_{theirs})\| \) instead of comparing numbers.
- **Check C mechanics:** column \( i \) ≈ \( \big[\,(p(q+h e_i)-p(q-h e_i))/2h;\;
  \mathrm{rotvec}\big(R(q+h e_i)R(q-h e_i)^\top\big)/2h\,\big] \) with
  \( h = 10^{-6} \) (central differences: truncation error \( O(h^2) \) balanced
  against float64 roundoff \( \sim 10^{-16}/h \)). If the linear rows match but
  angular rows are off by a rotation, the bug is frame convention (§3), not algebra.
- **Check D mapping:** differentiating a quaternion-valued FK gives
  \( \partial \mathbf{q}/\partial q_i \), not angular velocity; convert via
  \( \omega = 2\,\mathrm{vec}\big((\partial \mathbf{q}/\partial q_i) \otimes \mathbf{q}^{-1}\big) \).
  This check is optional — C already pins the Jacobian to our validated FK — so we
  implement it only if it doesn't fight us.

Passing bar for M2: all A/B/C max-errors under tolerance across all 100 samples,
printed as a table the writeup can quote. This *retires* design risk "our FK/Jacobian
is wrong" — after M2, any IK misbehavior in M3 is in the solver, by construction.

## 5. Resulting `robot_model.py` shape

Fills in the existing stub, keeping its interface:

```python
class FrankaModel:
    # parsed once from the URDF:
    #   A: list of 7 constant 4x4 prefixes, axes: (7,3), suffix: 4x4
    #   lower, upper: (7,) joint limits;  q_home: (7,)
    def fk(self, q) -> np.ndarray            # 4x4 world->TCP
    def fk_all(self, q) -> FKResult          # TCP pose + per-joint p_i, z_i
    def jacobian(self, q) -> np.ndarray      # (6,7), world-frame geometric
    def pose_error(self, T_cur, T_des) -> np.ndarray  # (6,), convention of §3
    def clamp(self, q) -> np.ndarray         # joint-limit clip
```

Plus module-level helpers with doctests-in-spirit: `rotation_about_axis`, `rpy_to_matrix`,
`rotvec_from_matrix` — small, pure, individually testable.

## 6. Gotchas checklist (things that will bite if forgotten)

- Radians everywhere; never degrees.
- Quaternion sign (wxyz vs xyzw *and* q ≡ −q) whenever PyRoKi/Viser quaternions
  appear — compare via geodesic distance only.
- PyRoKi cfg has 8 entries for this URDF (mimic finger), ours has 7.
- Joint *order*: ours follows the URDF chain (joint1..joint7); confirm PyRoKi's
  actuated ordering matches before comparing (print `robot.joints.actuated_names`).
- `mesh_dir` is a rendering concern only — kinematics validation needs no meshes
  (load with `load_meshes=False` for speed in scripts).
- Keep FK in float64 (numpy default) — the FD check's 1e-5 tolerance assumes it.

## 7. Definition of done → handoff to Stage 3 (M3, the DLS solver)

M2 is done when `scripts/validate_kinematics.py` prints all-green tables for checks
A-C (D optional) and `robot_model.py` has no remaining `NotImplementedError`. The
solver stage then consumes exactly three guarantees established here: `fk` is truth,
`jacobian` matches `fk` differentially, and `pose_error` speaks the same frame
convention as `jacobian`. Everything hard about M3 will be tuning, not correctness.

---

## 8. Results: built, tested, and one real bug caught

M2 is implemented ([robot_model.py](../src/franka_ik/robot_model.py)) and validated
([scripts/validate_kinematics.py](../scripts/validate_kinematics.py)). Two checks were
added beyond the plan: **E** (rotation helpers vs. scipy on structured edge cases —
identity, angles down to 1e-12, near/at π) and **F** (first-order consistency
`e(fk(q), fk(q+δ)) ≈ J(q)δ` — a direct test of the §3 convention contract the IK
solver will rely on).

Final table, over 112 configurations (100 random in-limit + home, both limit corners,
mid-range, and 8 random limit-corner combinations near singular folds):

```
PASS  E1 rotation_about_axis vs scipy          1.221e-15 < 1e-12
PASS  E2 rotvec_from_matrix vs scipy           2.124e-14 < 1e-09
PASS  A  position   (vs yourdfpy)              4.475e-16 < 1e-06
PASS  A  rotation   (vs yourdfpy)              3.416e-16 < 1e-06
PASS  B  position   (vs PyRoKi)                6.693e-16 < 1e-06
PASS  B  rotation   (vs PyRoKi)                1.161e-15 < 1e-06
PASS  C  jacobian   (vs finite differences)    2.752e-10 < 1e-05
PASS  D  jacobian   (vs PyRoKi jax.jacfwd)     8.882e-16 < 1e-05
PASS  F  error-vs-J first-order (relative)     5.827e-06 < 1e-04
9/9 checks passed
```

FK agrees with both independent references at machine precision (~1e-15/1e-16), i.e.
bit-for-bit up to float64 roundoff — stronger than the 1e-6 bar we set.

**The validation earned its keep: check E2 failed on the first run** (max error
8.1e-5 rad). Probing showed the worst case at θ = π − 1.08e-6 — *just outside* the
1e-6 cutoff of the original near-π branch, in the main formula's territory. Lesson:
the ill-conditioned window of θ/(2 sin θ)·vee(R−Rᵀ) around π is *wide* (catastrophic
cancellation in R−Rᵀ as sin θ → 0), not a point. Fix (in `rotvec_from_matrix`):
switch branches at θ > 3 rad, and make the near-π branch well-conditioned end to end —
axis from the symmetric part, (R+Rᵀ)/2 − cos θ·I = (1−cos θ)aaᵀ, and angle from
θ = π − arcsin(|vee|) (arcsin is well-conditioned exactly where arccos is not).
After the fix: 2.1e-14 across the same cases. Had E2 not existed, this bug would
have surfaced — untraceably — as an IK solver that misconverges only for targets
requiring a near-π reorientation.

Other implementation notes for the record:

- PyRoKi confirmed our two API predictions: `actuated_names` is 8 long
  (`fr3_joint1..7` + `fr3_finger_joint1`, the mimic folded away), and joint order
  matches ours, mapped by name anyway rather than by position.
- JAX must be switched to float64 (`jax.config.update("jax_enable_x64", True)`)
  before importing PyRoKi, or checks B/D would be capped at float32 noise (~1e-6).
- Check D's quaternion-derivative-to-angular-velocity map
  ω = 2·Im(∂q/∂qᵢ ⊗ q⁻¹) worked as derived — agreement at 1e-16.
- Integration with Stage 1 re-verified after the change: `verify_setup.py` still
  passes, the M1 render server still serves, and our `fk(q_home)` reproduces
  Stage 1's yourdfpy number exactly ([0.307, 0, 0.4869] m).

Design risk "our FK/Jacobian is wrong" is retired. M3 (the DLS solver) can now treat
`fk`, `jacobian`, and `pose_error` as ground truth.
