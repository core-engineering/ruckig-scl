# Ruckig SCL v0.5 — Design Spec (phase synchronization + no-sync)

**Date** : 2026-06-03
**Owner** : Martin C
**Project** : Port of Ruckig (C++ MIT) to Structured Text for Siemens S7-1500
**Status** : Design draft, pending plan

**Scope:** Two additional multi-axis synchronization modes on top of v0.4's
Time sync:
- **`Phase`** — all DoFs follow the *same* normalized time profile (shared jerk
  switch-time pattern), scaled per axis, producing straight-line motion in joint
  space. Possible only when the per-DoF state deltas are **collinear**; when they
  are not, **fall back to Time sync** (exact Ruckig behaviour).
- **`No`** — each DoF runs at its own time-optimal duration, independently (no
  synchronization).

Builds on v0.2.1 step1 (time-optimal), v0.3 step2 (re-time to `tf`), and v0.4
(Block → Synchronize → step2, the Time path).

**Out of scope (deferred to v0.6):** `TimeIfNecessary`,
`DurationDiscretization.Discrete`, `per_dof_synchronization`.

---

## 1. Context and Motivation

v0.4 delivered **Time** synchronization: every axis is re-timed (step2) to a
common `t_sync`. Ruckig offers three more `Synchronization` modes; v0.5 adds the
two highest-value ones:

- **Phase** is the coordinated-motion mode: when the move is collinear (all axes
  share a direction in state space), every axis follows the same jerk-switch
  *timing*, differing only in magnitude. The resulting path is a straight line in
  joint space — the natural mode for linear interpolated motion. It is the
  meatiest remaining synchronization algorithm.
- **No** is the independent mode: each axis takes its own time-optimal duration,
  finishing whenever it is ready. Cheap, and a natural building block.

The oracle (`ruckig` 0.17.3) was probed to fix the exact behaviour:
- `Synchronization.Phase` on a collinear input yields **identical `profile.t[]`
  across all DoFs**, with jerk scaled by the displacement ratio (the
  largest-displacement DoF saturates its limit; the others scale down).
- `Synchronization.Phase` on a **non-collinear** input **silently falls back to
  Time** (same total duration, but per-axis profile shapes differ).

## 2. Behaviour reference (oracle)

For a 2-DoF collinear case `pd = [1, 3]`, `v0 = [0.4, 1.2]`, `vT = [0.2, 0.6]`
(axis 1 = 3 × axis 0), `Synchronization.Phase`:
- `profile[0].t == profile[1].t` (shared switch times),
- `profile[1].j == 3 · profile[0].j` (jerk scaled by the ratio `k = 3`),
- peak `|a|` and `|v|` of axis 1 are `3 ×` axis 0's,
- the larger-displacement axis (axis 1) saturates `jMax`; axis 0 is scaled down.

For a non-collinear case (e.g. `v0 = [1, -1]` against `pd = [1, 3]`), the profile
shapes diverge and the total duration matches the **Time** result.

## 3. Architecture (Approach A — phase as a self-contained, short-circuiting FC)

`RuckigOtg`'s multi-axis recompute dispatches on `input.synchronization`:

```
SELECT synchronization:
  SYNC_TIME  -> v0.4 path (Block -> Synchronize -> step2)        [unchanged]
  SYNC_NO    -> per-axis loop: ComputeProfile1Axis -> profiles[d]
                each axis keeps its own duration;
                trajectory.duration = max(per-axis durations)
  SYNC_PHASE -> "PhaseSynchronize"(...) -> phaseOk
                if phaseOk: all profiles filled (scaled), done
                else:       fall through to the SYNC_TIME path
```

The Time path stays byte-for-byte as in v0.4 (regression guard: 181 tests). The
Phase→Time fallback is a plain `IF NOT phaseOk THEN <Time path>`.

### 3.1 New FC: `PhaseSynchronize` (`: Void`)

Inputs: per-DoF seed/target states, limits, `nDofs`. VAR_IN_OUT: the trajectory
profiles (and a block set, reused to find the limiting axis). VAR_OUTPUT:
`phaseOk : Bool`, `duration : LReal`.

Algorithm:
1. **Reference axis** = the DoF with the largest time-optimal duration (the
   limiting axis; the same notion `Synchronize` already computes). Compute its
   step1 profile — the *phase profile* (its `t[]` switch pattern).
2. **Per-axis ratio** `k_d`: derive from the dominant component of the reference
   state vector (the largest in magnitude among `pd, v0, a0, vT, aT`), e.g.
   `k_d = pd_d / pd_ref` when `pd` dominates. Using the dominant component keeps
   the ratio well-conditioned when some components are ~0.
3. **Collinearity test**: for every DoF `d`, require
   `|state_d[i] - k_d · state_ref[i]| < EPS` on all five components
   (`pd, v0, a0, vT, aT`). Any failure → `phaseOk := false` (Time fallback).
4. **Scale**: `profile[d].t := profile[ref].t` (shared), `profile[d].j[i] :=
   k_d · profile[ref].j[i]`, then `IntegrateProfileStates` + `CheckProfile`
   per axis (limit safety net). A failed check → `phaseOk := false`.
5. All axes pass → `phaseOk := true`, `duration := reference profile duration`.

`CheckProfile` is the same validity gate used since v0.2.1 (reach state +
limits, including interior velocity extrema), so a scaled profile that would
violate a limit (e.g. non-uniform per-axis limits) correctly triggers the Time
fallback rather than emitting an out-of-limits trajectory.

### 3.2 Sync.No

Each axis is solved by the existing single-axis step1 solver
(`ComputeProfile1Axis`) with its own seed/target/limits. Profiles keep their
individual durations; `trajectory.duration = max`. A fast axis that has reached
its target holds there via `StateAtTime`'s past-end extrapolation (jerk = 0,
already in place since v0.2.1). No brake pre-phase (multi-axis, as in v0.4).

## 4. FB integration and backward compatibility

- `typeRuckigInput.synchronization` already exists (`Int`); v0.4 ignored it.
  v0.5 activates it. To preserve v0.4 behaviour for callers who do not set it,
  the **field default changes to `SYNC_TIME`** (matching Ruckig's default, which
  is `Time`). An explicit `synchronization := SYNC_NONE` becomes a deliberate
  choice. Encoding (unchanged in `dbRuckigConst`): `SYNC_NONE=0`, `SYNC_PHASE=1`,
  `SYNC_TIME=2`, `SYNC_PER_DOF=3`.
- **Change detection** gains `synchronization`: switching mode forces a
  recompute.
- The single-DoF path (`nDofs == 1` and no `minimumDuration`) is unchanged: the
  v0.3 step1 stack is the strict-parity guard and ignores `synchronization`.
- Output/status semantics: `SYNC_NO` reports `done` when the slowest axis
  finishes (`currentTime >= trajectory.duration`); Phase/Time report a common
  duration, as in v0.4.

## 5. Testing and parity

**Unit — `PhaseSynchronize`:**
- collinear rest-to-rest and collinear moving-target → `phaseOk=true`, shared
  `t[]`, jerk scaled by ratio, all axes reach target;
- non-collinear (velocity / target-velocity direction differs) → `phaseOk=false`;
- collinear deltas but limits that the scaling would exceed → `phaseOk=false`.

**Unit — FB:**
- `SYNC_NO` 2-DoF: axes reach their targets at their *own* (different) cycles;
- `SYNC_PHASE` collinear 2-DoF: scaled profiles, duration = reference axis;
- `SYNC_PHASE` non-collinear 2-DoF: result equals the `SYNC_TIME` result.

**Parity (vs Ruckig oracle):**
- Thread an optional `synchronization` field through the YAML scenarios and both
  runners: `runner_ref` maps the SCL int → Ruckig enum **by name**
  (`SYNC_NONE→No`, `SYNC_PHASE→Phase`, `SYNC_TIME→Time`); `runner_scl` sets the
  int. **Scenario default = Time**, so the nine v04 scenarios stay Time (zero
  regression). The existing `default_input()` helper sets `SYNC_TIME` explicitly.
- New `v05_*` scenarios: `No` (per-axis independent), `Phase` collinear (exact
  match), `Phase` non-collinear (Time fallback). Expected parity at the
  floating-point floor (~1e-15), consistent with v0.2–v0.4.

## 6. Components summary

| Block | Status | Role |
|-------|--------|------|
| `RuckigOtg` (FB) | modified | dispatch on `synchronization`; Phase short-circuit + Time fallback; No loop |
| `PhaseSynchronize` (FC) | **new** | collinearity test + scale reference profile per axis + per-axis `CheckProfile`; `phaseOk` |
| `ComputeProfile1Axis` (FC) | reused | per-axis step1 for `SYNC_NO` |
| `Synchronize`, `ComputeBlock1Axis`, `ComputeProfile1AxisTimed` | reused | the Time fallback path (v0.4) |
| `typeRuckigInput` (UDT) | modified | `synchronization` default → `SYNC_TIME` |

## 7. Risks / open points

- **Reference-axis / ratio choice** for degenerate collinear cases (e.g. the
  reference's dominant component is itself small). Mitigation: pick the dominant
  component per the largest magnitude across all five state quantities; the
  per-axis `CheckProfile` is the backstop (failure → Time fallback), so a
  mis-scaled profile can never be emitted.
- **Non-uniform per-axis limits** under Phase: the scaled-down axes are within
  limits when limits are uniform; with non-uniform limits the `CheckProfile`
  backstop forces the Time fallback if a limit would be exceeded — matching
  Ruckig, which also abandons phase sync when a scaled profile is infeasible.
