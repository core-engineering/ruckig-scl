# Ruckig SCL v0.10 — step2 Completeness (family selection parity) — Design Spec

**Date** : 2026-06-04
**Owner** : Camille Martin
**Project** : Port of Ruckig (C++ MIT) to Structured Text for Siemens S7-1500
**Status** : Design approved, ready for implementation planning
**Builds on** : v0.9.0 (step1 two-step fallbacks)

---

## 1. Context and Motivation

step2 (`ComputeProfile1AxisTimed`: re-time a single-axis move to an imposed
duration `tf`) returns a **valid** profile (reaches the target, respects the
limits, `sum(t) = tf`) but, for ~1.7% of `(state × tf)` cases — typically a small
displacement stretched to a large `tf` — a **different shape** than Ruckig. The
SCL trajectory is correct (valid, smooth, on-target); it just doesn't match the
oracle's profile. This is the last known parity gap, surfaced by the v0.8/v0.9
final reviews.

A diagnostic spike pinned the cause precisely (measured over 700 `(state × tf)`
pairs, 12 genuine divergences = 1.71%):

1. **Dominant — `SolveTimedVel` root precision (≈10 of 12).** Ruckig's `time_vel`
   has analytic special branches the SCL lacks: an all-zero-boundary case solved
   by an exact **cubic**, and derivative-extrema seeding for the general
   degree-5/6 path. The SCL always uses a dense-scan + single-Newton path that
   lands ~2e-5 off the root — failing `CheckProfile`'s 1e-6 target tolerance — so
   the VEL family returns "not found" and the dispatch falls through to a later
   family (NONE/ACC1) that finds a *different* valid shape. (`CheckProfile`
   accepts the oracle's exact VEL `t[]`, so the fix is purely root accuracy.)
2. **Secondary — missing structural `ReachedLimits` guards (the 2 `VEL→NONE`
   residual + hardening).** Ruckig's `check_with_timing<…, ReachedLimits::X>`
   requires the family's defining plateau to be present (`t[3] ≥ ε` for the
   velocity families, `t[1] ≥ ε` for ACC0, `t[5] ≥ ε` for ACC1, both for
   ACC0_ACC1). `CheckProfile` enforces none of these, so an earlier family can
   greedily accept a structurally-degenerate profile and mask the correct one.

The dispatch ORDER already matches the oracle, and UDUD is already implemented —
neither is the problem.

### Scope decision (settled during brainstorming + spike)

- Fix both causes: (1) `SolveTimedVel` root precision, (2) structural guards in
  the limit-reaching step2 families. Bounded fix (~7 of 8 family FCs touched),
  not a rework — the polynomial/coefficient code is already correct.
- Out of scope: any second-order/velocity-interface step2 (third-order position
  only); the step1 path (complete since v0.9).

---

## 2. Goals and Non-Goals

### Goals

- `ComputeProfile1AxisTimed` matches the Ruckig oracle's profile **shape** (not
  just validity) to the floating-point floor across `(state × tf)` pairs,
  reducing the measured 1.71% divergence to ~0.
- `SolveTimedVel` finds the velocity-plateau root to ≤ 1e-9 (well within
  `CheckProfile`'s 1e-6), matching Ruckig's `time_vel`.
- Each limit-reaching step2 family accepts only structurally-valid profiles
  (its defining plateau present), matching `ReachedLimits`.
- No regression: the 255 existing tests stay green (currently-passing parity
  scenarios already match the oracle, which respects these guards, so stricter
  guards cannot reject them).

### Non-Goals

- Velocity-interface / second-order step2.
- Changing the dispatch order (already correct) or `CheckProfile`'s target/limit
  logic (correct — the fix is root accuracy + structural guards).

---

## 3. Architecture

Two independent, bounded changes within the existing step2 family FCs; no new FC,
no signature changes, `ComputeProfile1AxisTimed` dispatch unchanged.

### 3.1 `SolveTimedVel` root precision

Port the analytic branches Ruckig's `time_vel`
(`position_third_step2.cpp` l.259-487) has and the SCL lacks:

1. **All-zero-boundary cubic branch** (when `|v0|, |a0|, |vf|, |af| < ε`): the
   velocity-plateau profile reduces to an exact **cubic** in the ramp time; solve
   it with the existing `SolveCubic` FC and refine with one Newton step. This is
   the canonical stretched-rest-to-rest case and the highest-value branch.
2. **General degree-5/6 path seeding**: replace (or seed) the brittle 128-segment
   dense scan + single Newton with **derivative-extrema seeding** — Ruckig brackets
   the polynomial's roots using the extrema of its derivative (a lower-degree
   polynomial solved via the existing `SolveQuartic`/`SolveCubic`), giving robust
   initial guesses, then refines. Target: root accuracy ≤ 1e-9 so `CheckProfile`'s
   1e-6 target check accepts the true VEL profile.

Both UDDU (degree-5) and UDUD (degree-6) paths get the precision improvement;
the existing coefficient computation is correct and is reused.

### 3.2 Structural `ReachedLimits` guards

Before each family accepts a candidate (i.e. guarding each `CheckProfile`
call/commit), require the family's defining plateau, matching the oracle's
`ReachedLimits` (`profile.hpp` `check<>`):

| Family FC | Guard (ε = `EPS_TIME`) |
|---|---|
| `SolveTimedVel`, `SolveTimedAcc0Vel`, `SolveTimedAcc1Vel`, `SolveTimedAcc0Acc1Vel` | `t[3] ≥ ε` |
| `SolveTimedAcc0` | `t[1] ≥ ε` |
| `SolveTimedAcc1` | `t[5] ≥ ε` |
| `SolveTimedAcc0Acc1` | `t[1] ≥ ε` **and** `t[5] ≥ ε` |
| `SolveTimedNone` | none |

Implemented as an inline `IF` on the relevant `profile.t[k]` immediately before
the existing `CheckProfile` acceptance in each sub-case (localized to the step2
families; **`CheckProfile` is not modified**, so step1 and all other callers are
untouched). A candidate failing its structural guard is skipped, so the dispatch
correctly continues to the family Ruckig uses.

### 3.3 Why no regression

Currently-passing parity scenarios already match the oracle, which itself respects
these structural guards and the precise VEL root. So for every currently-passing
case the SCL already picks the oracle's (structurally valid) family — the stricter
guards cannot reject it, and the more precise VEL root only changes cases where
the SCL currently diverges. The 255 tests stay green; only the 1.71% diverging
population moves toward the oracle.

---

## 4. Error handling

Unchanged. `ComputeProfile1AxisTimed` still returns `RESULT_ERR_SOLVER` only when
no family finds a profile. The guards never cause a spurious error (a guard-failed
candidate falls through to the next family/sub-case exactly as a CheckProfile
failure does today). No new status codes.

---

## 5. Testing strategy

### 5.1 Diagnostic-derived fixtures

The spike's seeded `(state × tf)` fuzz identified the 12 diverging cases. Freeze a
representative subset (the canonical `p0=0 → 0.5, tf=8` rest-to-rest stretch, plus
≥4 others covering VEL and the VEL→NONE structural cases) as **`v10_*` parity
scenarios** (single-axis, driven via `minimum_duration` to force the stretched
`tf`), each currently diverging, to pass after the fix.

### 5.2 Unit tests

- `SolveTimedVel` precision: the all-zero-boundary case (`p0=0→0.5, tf=8`) yields
  the velocity-plateau profile (`t[3] ≫ 0`) reaching the target to ≤ 1e-9, with
  the same shape as the oracle.
- A structural-guard regression: a state where an earlier family would otherwise
  greedily accept a degenerate profile now correctly defers to the right family.

### 5.3 Broad parity fuzz (the real gate)

Re-run a seeded `(state × tf)` fuzz (a fresh seed) over single-axis states ×
`tf ∈ {1.05, 1.5, 2, 3, 5} × tMin`, comparing `ComputeProfile1AxisTimed` (and the
full path) to the oracle cycle-by-cycle. Target: the genuine divergence rate
drops from 1.71% to ~0 (no shape mismatch > 1e-6), with infeasible-`tf` false
positives excluded.

### 5.4 Regression

All 255 existing tests stay green.

---

## 6. Success criteria

- The measured `(state × tf)` step2 divergence rate is ~0 (down from 1.71%);
  the canonical stretched cases match the oracle's profile shape to the
  floating-point floor.
- `SolveTimedVel` resolves the velocity-plateau root to ≤ 1e-9.
- The structural guards are present in the limit-reaching families; `CheckProfile`
  is unchanged.
- 255 existing tests green; new `v10_*` parity scenarios pass.
- README / CHANGELOG / roadmap updated; version bumped to `0.10.0`; merged and
  tagged `v0.10.0`. With this, both step1 and step2 are parity-complete; the next
  milestone is performance / pre-1.0.
