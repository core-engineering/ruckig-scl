# Ruckig SCL v0.9 — step1 Two-Step Fallbacks — Design Spec

**Date** : 2026-06-04
**Owner** : Camille Martin
**Project** : Port of Ruckig (C++ MIT) to Structured Text for Siemens S7-1500
**Status** : Design approved, ready for implementation planning
**Builds on** : v0.8.0 (multi-axis brake)

---

## 1. Context and Motivation

The single-axis time-optimal solver (step 1) ports Ruckig's three main profile
families (`time_all_vel`, `time_acc0_acc1`, `time_all_none_acc0_acc1`) via
`SolveDirection`. Ruckig additionally has **four closed-form "two-step" fallback
families** (`time_none_two_step`, `time_acc0_two_step`, `time_vel_two_step`,
`time_acc1_vel_two_step`) that it runs **only when the three main families find no
profile**. ruckig-scl does not port these, so for ~3% of arbitrary
initial/target state combinations the solver returns `RESULT_ERR_SOLVER` where
Ruckig succeeds.

This is the long-standing "step1 ~3% gap" tracked since v0.2. v0.9 closes it by
porting the four two-step fallbacks. Because step 1 underlies every solve path —
single-axis, the multi-axis Block (`ComputeBlock1Axis`), and (since v0.8) the
post-brake Block — the fix benefits all of them.

### Scope decision (settled during brainstorming)

- **step1 two-step fallbacks only.** The separate step2 short-move-stretched
  divergence (a *valid-but-different* shape, flagged by the v0.8 final review) is
  an independent chantier deferred to a later version.
- **Zero-limits special case out of scope.** Ruckig's `time_all_single_step`
  handles `jMax == 0` / `aMax == 0`; `ValidateInput` already rejects non-positive
  `maxJerk` / `maxAcceleration` in the position interface, so that path is
  unreachable here.

---

## 2. Goals and Non-Goals

### Goals

- Port Ruckig's four step1 two-step fallback families as a last-resort stage,
  invoked only when the three main families find no profile.
- Eliminate the ~3% `RESULT_ERR_SOLVER` failures on arbitrary single-axis states,
  matching the Ruckig oracle to the floating-point floor on those states.
- Benefit all callers (single-axis `ComputeProfile1Axis`, multi-axis
  `ComputeBlock1Axis`, and the v0.8 post-brake Block) with no change to their
  existing behaviour.
- No regression: the existing 245 tests stay green (the fallback runs only when
  the main families found nothing).

### Non-Goals

- step2 completeness / the short-move-stretched divergence (separate version).
- Zero-limits `time_all_single_step` (unreachable — validation rejects it).
- The structural `ReachedLimits` disambiguation guards Ruckig uses to assign a
  duration to a family for Block-interval bookkeeping — not needed for a
  last-resort feasibility fallback (see §4).

---

## 3. Architecture

### 3.1 New FC `SolveTwoStepDir`

A new pure FC, **mirroring the `SolveDirection` signature exactly** so the two
callers invoke it identically and reuse their existing finalize / Block-assembly
code:

```
FUNCTION "SolveTwoStepDir" : Void
    VAR_IN_OUT
        profile      : _.typeProfile;
        bestT        : Array[0..6] of LReal;
        bestDuration : LReal;
        found        : Bool;
        bestSjMax    : LReal;
        allDur       : Array[0..7] of LReal;
        allCount     : Int;
    VAR_INPUT
        p0, v0, a0, pT, vT, aT : LReal;
        svMax, svMin, saMax, saMin, sjMax : LReal;   // signed, per direction
```

Behaviour, for one direction, mirroring the oracle dispatch order
(`time_none_two_step` → `time_acc0_two_step` → `time_vel_two_step` →
`time_acc1_vel_two_step`), **return after the first valid profile**:

1. Self-skip: `IF #found THEN RETURN; END_IF`.
2. Precompute `pd = pT - p0` and the power terms used by the formulas
   (`a0_a0, af_af, a0_p3, af_p3, a0_p4, af_p4, v0_v0, vf_vf, jMax_jMax`, with
   `af := aT`, `vf := vT`). Set the UDDU jerk pattern
   `j = [sjMax, 0, -sjMax, 0, -sjMax, 0, sjMax]`.
3. Run the four families' sub-cases in order; each sets `profile.t[0..6]`, calls
   `CheckProfile(... tf := -1.0)`; on the first that validates, commit:
   `bestT := profile.t`, `bestDuration := Σ t`, `found := true`,
   `bestSjMax := sjMax`, append `bestDuration` to `allDur` (bounds-guarded
   `allCount <= 7`), and `RETURN`.

### 3.2 Invocation (fallback, both directions)

Both callers, after their two `SolveDirection` calls (positive then mirrored
direction), add a guarded fallback **before** finalizing/erroring:

- `ComputeProfile1Axis` — after the mirror-direction `SolveDirection`, before
  `REGION Finalise winner or fail`:
  ```scl
  IF NOT #found THEN
      "SolveTwoStepDir"(… svMax := #vMax, svMin := -#vMax, saMax := #aMax, saMin := -#aMax, sjMax := #jMax);
      "SolveTwoStepDir"(… svMax := -#vMax, svMin := #vMax, saMax := -#aMax, saMin := #aMax, sjMax := -#jMax);
  END_IF;
  ```
  The existing finalize then sets the UDDU jerk pattern from `bestSjMax` and
  integrates `bestT`, unchanged.

- `ComputeBlock1Axis` — after the second `SolveDirection`, before `REGION Fail if
  no profile`, the same `IF NOT #found THEN … END_IF`. The existing sort / dedup /
  `calculate_block` then produces `tMin` from the single fallback duration (no
  blocked interval), and `RESULT_ERR_SOLVER` now fires only when the fallbacks
  also fail (genuinely infeasible input).

No change to `SolveDirection`, `CheckProfile`, `IntegrateProfileStates`, the
finalize regions, or the Block assembly.

---

## 4. The four two-step families

All closed-form (only `SQRT`), all `ControlSigns::UDDU`. Faithful ports of
`position_third_step1.cpp` (lines 287-444). `pd = pT - p0`, `af = aT`, `vf = vT`.

- **`time_none_two_step`** — 2 sub-cases: two-step
  `h0 = sqrt((a0²+af²)/2 + jMax·(vf−v0))·sgn(jMax)`, `t0 = (h0−a0)/jMax`,
  `t2 = (h0−af)/jMax`; single-step `t0 = (af−a0)/jMax`. (`NONE` check.)
- **`time_acc0_two_step`** — 4 sub-cases: two-step; three-step "removed pf";
  three-step "removed aMax" (with an `h1` discriminant `sqrt`); three-step
  `t = (aMax−aMin)/jMax`. (`ACC0` check.)
- **`time_vel_two_step`** — 2 sub-cases (four-step), shared
  `h1 = sqrt(af²/(2·jMax²) + (vMax−vf)/jMax)`. (`VEL` check.)
- **`time_acc1_vel_two_step`** — 1 sub-case (six-step), using `af_p4`/`a0_p3`
  power terms. (`ACC1_VEL` check.)

**Validity.** Each sub-case is accepted by the existing `CheckProfile` (integrate
from the boundary; phase times ≥ 0; `|v|≤vMax`, `|a|≤aMax`; interior velocity
extrema; reaches the target `(pT,vT,aT)`). Ruckig additionally applies a
structural `ReachedLimits` guard (e.g. `ACC0` requires `t[1] ≥ eps`); this is used
to assign a duration to a *named* family for Block-interval bookkeeping. For a
last-resort fallback that only needs to find **one feasible profile** (the main
families produced none, so there is no blocked-interval ambiguity), `CheckProfile`
alone is the correct acceptance test. If a sub-case yields a profile that reaches
the target within limits, it is a valid trajectory and is accepted — matching
Ruckig's outcome (Ruckig also returns the first two-step profile that passes its
check).

**Transpiler constraints** (`docs/PLC_CODE_LIMITATIONS.md`): no division by a
parenthesized product — every compound denominator (`2·a0·jMax`, `2·aMax·jMax`,
`jMax·h0`, `3·jMax²·vMax`, …) is precomputed into a scalar temp; one `:=` per
line; `END_IF`/`END_FOR` on their own line; identifiers avoid `IF`-prefix and
`of`/`Dof`-suffix.

---

## 5. Error handling

Unchanged. `ComputeProfile1Axis` / `ComputeBlock1Axis` still return
`RESULT_ERR_SOLVER` when **no** profile is found — now only for genuinely
infeasible inputs (after the fallbacks), a much smaller set. No new status codes.

---

## 6. Testing strategy

### 6.1 Gap-state discovery (reproducible)

The ~3% gap states cannot be guessed. A seeded fuzz (a one-off discovery script,
its representative outputs frozen as fixtures) draws random single-axis states
`(p0, v0, a0, pT, vT, aT)` within limits, runs `ComputeProfile1Axis` (SCL) and the
oracle, and collects states where the SCL returns `RESULT_ERR_SOLVER` while the
oracle returns a valid profile. A handful covering the different fallback families
(none / acc0 / vel / acc1_vel where identifiable) are frozen as test inputs.

### 6.2 Unit tests

`tests/unit/test_solve_two_step.py` — load `ComputeProfile1Axis` and assert that
each discovered gap state, which returned `RESULT_ERR_SOLVER` before, now returns
`RESULT_WORKING` and the integrated profile reaches `(pT, vT, aT)` within limits.

### 6.3 Parity scenarios

`v09_xx` single-axis scenarios built from the discovered gap states → match the
oracle cycle-by-cycle to the floating-point floor. Plus 1-2 multi-axis scenarios
where one axis is a two-step state (exercising the fallback through
`ComputeBlock1Axis`, including with a brake from v0.8).

### 6.4 Regression

The fallback runs only when `found = false`, so all 245 existing tests are
unaffected. Confirm they stay green.

---

## 7. Success criteria

- Single-axis states that previously returned `RESULT_ERR_SOLVER` now produce a
  profile matching the Ruckig oracle to the floating-point floor.
- The fallback flows through `ComputeBlock1Axis` (multi-axis + post-brake) too.
- The 245 existing tests stay green; new `v09_xx` parity scenarios pass at the
  floating-point floor.
- README / CHANGELOG / roadmap updated; version bumped to `0.9.0`; merged and
  tagged `v0.9.0`. The remaining documented gap becomes step2 completeness.
