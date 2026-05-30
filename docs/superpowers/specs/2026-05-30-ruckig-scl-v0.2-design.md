# Ruckig SCL v0.2 — Design Spec

**Scope:** Single-axis, time-optimal jerk-limited trajectory generation with
**arbitrary initial and target states** (`v0,a0 ≠ 0` and `vT,aT ≠ 0`), online
recomputation via `pass_to_input` chaining, and a minimal brake pre-phase for an
out-of-limits first enable. Builds directly on the v0.1 FB/FC/UDT layer.

---

## 1. Context and Motivation

v0.1 delivered a single-axis, **rest-to-rest** OTG (`targetVel = targetAcc = 0`,
start from rest), validated against Ruckig to ~1e-15. The driving real-world use
case — online camera tracking of a **moving** target on the electric MLA arm
(Kalman estimate at 100 Hz) — needs two capabilities v0.1 lacks:

- **Non-zero target velocity/acceleration**, to follow a moving target without lag.
- **Arbitrary initial state**, to recompute mid-motion (when the target moves)
  from the generator's current commanded state, without a velocity/acceleration
  discontinuity.

v0.2 adds both, i.e. the full single-DoF time-optimal problem.

## 2. Goals and Non-Goals

### Goals
- Time-optimal single-DoF profile for arbitrary `(p0,v0,a0) → (pT,vT,aT)` within
  symmetric limits `±vMax, ±aMax, ±jMax`.
- Strict numerical parity with the official Ruckig solver (target ~1e-9, to be
  calibrated).
- Online `pass_to_input` chaining inside the FB → C2 continuity on retarget.
- Minimal brake pre-phase for an out-of-limits initial state (first enable in motion).

### Non-Goals (deferred)
- Multi-axis and synchronization (v0.3–v0.5).
- Velocity interface (v0.6).
- Exhaustive brake / degenerate-case handling (v0.7); v0.2 covers only the
  first-enable-in-motion case.
- Asymmetric limits and Ruckig Pro features (waypoints, per-DoF targets, bounds).

## 3. Decisions Settled During Brainstorming

1. **Scope** — arbitrary initial *and* target states (not target-only).
2. **Optimality** — time-optimal with strict parity (port Ruckig's real
   single-DoF solver, not a simplified/non-optimal scheme).
3. **Initial-state source** — `pass_to_input`: the FB chains from its own last
   commanded setpoint; the user supplies only the target (position + predicted
   velocity) and limits.
4. **Edge cases** — include a *minimal* brake pre-phase for an out-of-limits
   first enable; full brake machinery stays in v0.7.
5. **Implementation approach** — **B: structured re-derivation**. Re-implement
   Ruckig's time-optimal logic as idiomatic pure SCL FCs (one per profile type),
   using the Ruckig source as the equation reference and the parity bench as the
   correctness oracle. (Rejected: A literal C++ port — too large/fragile without
   templates; C non-optimal decomposition — incompatible with strict parity.)

## 4. Architecture

The v0.2 general solver **replaces** the two rest-to-rest-specific FCs of v0.1
(`ComputeMinDuration` + `ComputeFinalProfile`); the symmetric profile becomes a
special case the general solver reproduces.

### 4.1 New FCs (`src/blocks/`)

| FC | Role |
|----|------|
| `SolveCubic` | Real roots of `a·x³+b·x²+c·x+d` (Cardano; degrades to quadratic/linear). |
| `SolveQuartic` | Real roots of a quartic (Ferrari via resolvent cubic; degrades to cubic). |
| `ComputeProfile1Dof` | Core solver: enumerate profile types × jerk families, validate, select min-duration; fill `typeProfile` + return duration/status. |
| `IntegrateProfileStates` | Fill `a[8]/v[8]/p[8]` from `t[]/j[]` + `(p0,v0,a0)` (generalised v0.1 integration). |
| `ComputeBrakeProfile` | Brake sub-profile when `|v0|>vMax` or `|a0|>aMax`; stored in `profile.brake`. |

### 4.2 Reused / extended

- `StateAtTime` — extended to evaluate the `brake` phases first, then the main profile.
- `AdvanceTime` — unchanged.
- `ValidateInput` — accepts non-zero current/target vel/acc (consistency checks kept).
- `RuckigOtg` (FB) — `pass_to_input` chaining, brake pre-phase, change detection
  extended to `targetVelocity/targetAcceleration` (see §7).

## 5. Solver Internal Structure (`ComputeProfile1Dof`)

Reproduces Ruckig Step 1 as **enumerate-then-select**:

1. **Two jerk-sign families** (+ mirror for negative motion, via `jMax` sign /
   `direction`), encoded in `profile.controlSigns`:
   - UDDU: `+,0,−,0,−,0,+`
   - UDUD: `+,0,−,0,+,0,−`
2. **Per family, the profile types** (which segments saturate), Ruckig's set:
   `ACC0_ACC1_VEL · ACC1_VEL · ACC0_VEL · VEL · ACC0_ACC1 · ACC1 · ACC0 · NONE`.
   Each is a pure FC computing the free phase durations analytically
   `(valid, t[7], duration)`; non-trivial cases reduce to a quadratic→quartic
   solved by the root solver.
3. **Validation** (`CheckProfile`): integrate the candidate via
   `IntegrateProfileStates`; accept iff all `t ≥ −EPS`, the final `(p,v,a)`
   reaches the target within EPS, and peaks `|v| ≤ vMax+EPS`, `|a| ≤ aMax+EPS`.
4. **Selection**: keep the **minimum-duration valid** candidate over all
   types × families → the time-optimal profile. Fill `typeProfile`
   (`t/j/direction/controlSigns`); return duration + status (`WORKING`, or
   `ERR_SOLVER` if none valid — should not occur on feasible input).

## 6. Root Solver

Closed-form (non-iterative) for deterministic scan time and no convergence risk:

- `SolveCubic` — depress to `t³+p·t+q`, Cardano with the three discriminant cases.
- `SolveQuartic` — depress to `x⁴+p·x²+q·x+r`, Ferrari (resolvent cubic via
  `SolveCubic`, then two quadratics).
- **Interface** (`StateAtTime` pattern): `FUNCTION : Void` with
  `VAR_OUTPUT roots : Array[0..3] of LReal` + `count : Int`. Returns all real
  roots; physical-root selection (real, `≥ −EPS`, smallest feasible time) lives
  in the profile-type FCs.
- **Robustness**: discriminant-sign thresholds; reproduce Ruckig's numerically
  stable `Roots` forms. Tested in isolation against polynomials with known roots.

## 7. FB Lifecycle (`RuckigOtg`)

New persistent state (VAR static): `chainPos / chainVel / chainAcc : LReal` —
the last commanded setpoint (internal `pass_to_input`).

1. **Disabled** — force outputs false, status FINISHED, `firstCall := true`,
   `trajectory.isValid := false`, return (as v0.1).
2. **Validation** — `ValidateInput`; propagate DA014 error codes.
3. **First enable (`firstCall`)** — seed the chain from the real axis state:
   `chainPos/Vel/Acc := input.current{Position,Velocity,Acceleration}[0]`
   (all 0 when starting from rest).
4. **Change detection** —
   `recompute := firstCall OR reset OR NOT isValid OR Δ(targetPos,targetVel,targetAcc) > EPS OR Δ(limits) > EPS`.
5. **Recompute** (replaces the two v0.1 FCs):
   - **Brake pre-phase** if `|chainVel|>vMax` or `|chainAcc|>aMax` →
     `ComputeBrakeProfile`; main profile starts from the post-brake state.
     (With `pass_to_input` the chained state is always within limits, so this
     only triggers at a first enable in motion.)
   - `ComputeProfile1Dof(p0,v0,a0 := initial state; pT,vT,aT := target; limits)`.
   - `trajectory.duration := brakeDuration + mainDuration`; `currentTime := 0`;
     `isValid := true`; `prevInput := input`; `firstCall := false`.
6. **Advance + Evaluate** — `AdvanceTime`; `StateAtTime` (brake phases first,
   then main) → `output`.
7. **Status** — set valid/busy/done/error + DA014 code; done when
   `currentTime ≥ duration − EPS_TIME`.
8. **Chain update (end of cycle)** —
   `chainPos/Vel/Acc := output.new{Position,Velocity,Acceleration}[0]` → initial
   state for the next recompute, ensuring C2 continuity on retarget.

## 8. Data Model

No UDT changes — the v0.1 scaffolding already supports v0.2:

- `typeProfile`: `t[7]/j[7]/a[8]/v[8]/p[8]`, `direction`, nested `brake`,
  `controlSigns` (now encodes the jerk-sign family).
- `typeBrakeProfile`: `t[2]/j[2]/a[3]/v[3]/p[3]` — ≤ 2 brake segments (sufficient).
- `typeRuckigInput/Output`: `current*` / `target*` velocity & acceleration and the
  `new*` outputs already present.
- Symmetric limits assumed (`±vMax/±aMax/±jMax`).

## 9. Validation Strategy

- **SCL unit tests (TDD)** per brick: `SolveCubic`/`SolveQuartic` (known roots),
  each profile-type FC (hand-computed cases), `CheckProfile`, `ComputeProfile1Dof`,
  `ComputeBrakeProfile`, `IntegrateProfileStates`, and the extended FB.
- **Parity bench** (`tests/parity/`, reused): +10 scenarios — non-rest initial,
  moving target (`targetVel,Acc ≠ 0`), and brake-at-first-enable. The runners
  already pass `target_velocity/acceleration` to both Ruckig (online
  `update` + `pass_to_input`) and the FB, so traces compare directly.
- **Tolerance**: aim strict (~1e-9), calibrate as in v0.1 (~1e-15 measured);
  relax locally only if root-solver numerics require it, and log the value.
- **Stretch**: extend the bench to a time-varying target (true tracking);
  otherwise constant non-rest targets.

## 10. Out of Scope (explicit)

Multi-axis / synchronization; velocity interface; exhaustive brake & degenerate
cases (v0.2 covers only first-enable-in-motion); asymmetric limits; Ruckig Pro
features.

## 11. Risks

1. **Root-solver robustness** near-degenerate (double roots, near-zero leading
   coefficients) → thresholds + dedicated unit tests.
2. **Tie-breaking vs Ruckig** (which candidate when durations tie) → min-duration
   selection + parity bench; may need to mirror Ruckig's ordering.
3. **Effort** — substantially larger than the original 2-week roadmap estimate
   (porting the exact solver is the bulk of the work).

## 12. Success Criteria

- v0.1 **rest-to-rest behaviour is preserved**: the parity scenarios with
  `targetVel = 0` still pass via the general solver, and the FB unit tests
  (`test_ruckig_otg`) still pass. The FC-level tests of the *replaced*
  `ComputeMinDuration` / `ComputeFinalProfile` are superseded by the new solver's
  unit + parity tests (the integration half lives on as `IntegrateProfileStates`,
  with its own tests).
- New per-FC unit tests pass; `ComputeProfile1Dof` produces time-optimal profiles
  for arbitrary feasible boundary states.
- +10 v0.2 parity scenarios pass within the calibrated tolerance.
- The FB tracks a moving (non-zero-velocity) target and recomputes mid-motion
  with C2 continuity (no setpoint jump).
