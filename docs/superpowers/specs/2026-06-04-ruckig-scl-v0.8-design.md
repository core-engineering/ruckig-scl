# Ruckig SCL v0.8 — Multi-Axis Brake Pre-Phase — Design Spec

**Date** : 2026-06-04
**Owner** : Camille Martin
**Project** : Port of Ruckig (C++ MIT) to Structured Text for Siemens S7-1500
**Status** : Design approved, ready for implementation planning
**Builds on** : v0.7.0 (velocity control interface)

---

## 1. Context and Motivation

When a trajectory starts from an out-of-limits state — `|v0| > vMax` or
`|a0| > aMax` (first enable while already moving, or a chained retarget that
overshoots) — Ruckig prepends a **brake pre-phase**: a ≤2-segment sub-trajectory
that drives the state back inside the limits before the main profile begins.

ruckig-scl already ports this brake (`ComputeBrakeProfile` =
`get_position_brake_trajectory`) and wires it into the **single-axis** path. That
path is parity-faithful: out-of-limit single-axis cases (`v0 > vMax`, `a0 > aMax`,
negative, short moves) match the Ruckig oracle to the floating-point floor
(~1e-15), verified directly.

The gap: the **multi-axis path** has no brake. A multi-DoF call with an
out-of-limits initial state on any axis is not braked — the multi-axis recompute
assumes each axis starts within limits. v0.8 closes this, completing the brake
story.

### Scope decision (settled during brainstorming)

- **Multi-axis brake only.** Port the brake into the multi-axis unified per-axis
  flow. The single-axis path is unchanged (already correct). Add brake parity
  scenarios (single-axis as a regression guard, multi-axis for the new path).
- **Out of scope:** the step1 `time_*_two_step` fallbacks (~3% of arbitrary
  initial/target states that currently return `RESULT_ERR_SOLVER`) — an
  independent solver-completeness chantier deferred to a later version.
- **Out of scope:** the velocity-interface brake (`get_velocity_brake_trajectory`)
  — the v0.7 velocity solver already absorbs an arbitrary `a0` natively (parity
  verified to 1e-14 on `a0 = 12 > aMax`), so an explicit velocity brake is not
  needed for parity.

---

## 2. Goals and Non-Goals

### Goals

- Apply the existing position brake pre-phase per axis in the multi-axis recompute
  path, so an out-of-limits initial `(v0, a0)` on any DoF is braked before
  synchronization.
- Faithful to Ruckig: the brake folds into the reachable-duration Block (total
  duration = brake + inner), synchronization runs in total-duration space, and
  the inner profile is re-timed to `t_sync − brakeDuration`.
- All existing sync modes (Time / No / per-DoF / TimeIfNecessary / Discrete) keep
  working, with brake applied uniformly.
- Numerical parity with the oracle to the floating-point floor; all 237 existing
  tests stay green (an in-limit axis yields `brakeDuration = 0` → identical to
  v0.7).

### Non-Goals

- Single-axis path changes (already correct).
- Velocity-interface brake (not needed for parity).
- step1 two-step fallbacks (separate version).
- Faithful **Phase + out-of-limits** synchronization — see §5; Phase with an
  out-of-limits axis falls back to Time (which brakes), a documented limitation.

---

## 3. Architecture

### 3.1 Ruckig's brake model (verified against the oracle)

Per DoF, Ruckig (`calculator_target.hpp`):
1. computes `p.brake` from the current state (`get_position_brake_trajectory`),
2. `finalize`s it — integrating the brake and **advancing the boundary**
   `p.p[0], p.v[0], p.a[0]` to the post-brake state,
3. runs step1 from the **post-brake** state to get `blocks[dof]`,
4. **folds the brake into the block** (`block.hpp`:
   `t_min = t_sum + brake.duration`, and the blocked-interval edges likewise) so
   the block is in **total-duration** space,
5. synchronizes on the total-duration blocks → `traj.duration` / `t_sync`,
6. re-times each inner profile to `t_profile = traj.duration − brake.duration`.

`AdvanceTime` / `StateAtTime` already evaluate the brake prefix first.

### 3.2 SCL integration

Only the **multi-axis unified per-axis flow** in `RuckigOtg` changes (the global
`SYNC_PHASE` branch and the single-axis path are untouched). Per axis, inside the
recompute, the new sequence is:

1. **Brake**: call `ComputeBrakeProfile(profile := trajectory.profiles[ax],
   v0 := chainVel[ax], a0 := chainAcc[ax], vMax, aMax, jMax)`. It writes the
   integrated brake prefix into `profiles[ax].brake`, returns `brakeDur[ax]`, and
   exposes the post-brake state as `profiles[ax].brake.p[2] / v[2] / a[2]`.
   The post-brake **position** seed is `chainPos[ax] + brake.p[2]` (the brake
   `p[2]` is a relative advance), velocity `brake.v[2]`, acceleration `brake.a[2]`.
2. **Block from post-brake**: `ComputeBlock1Axis(p0 := postP, v0 := postV,
   a0 := postA, …)` (position interface).
3. **Fold brake into the block** (total-duration space):
   `block.tMin += brakeDur[ax]`; if `aValid`, `aLeft += brakeDur[ax]`,
   `aRight += brakeDur[ax]`; if `bValid`, `bLeft += brakeDur[ax]`,
   `bRight += brakeDur[ax]`.
4. **Synchronize** unchanged → `t_sync` (total-duration space).
5. **Emit** per axis, from the post-brake state:
   - participating / `tinMoving` axes → `ComputeProfile1AxisTimed(p0 := postP,
     v0 := postV, a0 := postA, …, tf := t_sync − brakeDur[ax])`.
   - `No` / `TimeIfNecessary`-rest axes → `ComputeProfile1Axis(p0 := postP, …)`
     (own inner duration).
   - `axDur[ax] = brakeDur[ax] + Σ profiles[ax].t[i]`; `duration = max(axDur)`.
   - `independentMinDurations[ax] = block.tMin` (already includes brake).

`ComputeBrakeProfile`, `Synchronize`, `ComputeBlock1Axis`,
`ComputeProfile1Axis`, `ComputeProfile1AxisTimed`, `AdvanceTime`, `StateAtTime`
are reused **unchanged**. The brake prefix is no longer zero-filled in the
multi-axis emit loop (it is now populated by `ComputeBrakeProfile`).

### 3.3 State seeding note

The single-axis path already seeds the main solve from
`chainPos[0] + profiles[0].brake.p[2]` (relative brake advance). The multi-axis
flow mirrors this per axis. When `brakeDur[ax] = 0`, `ComputeBrakeProfile` clears
`profiles[ax].brake` and returns `0`, with `brake.p[2] = 0`,
`brake.v[2] = chainVel[ax]`, `brake.a[2] = chainAcc[ax]` — so the post-brake seed
equals the chain state and behaviour is identical to v0.7.

---

## 4. Edge cases

- **In-limit axis** → `brakeDur = 0`, brake cleared → byte-identical to v0.7. The
  237 existing tests must stay green.
- **No / TimeIfNecessary-rest axes** are braked too; `brakeDur` enters their
  `tMin` and therefore the `t_sync` floor — faithful to Ruckig (`block.hpp` adds
  brake to every block).
- **Already-at-target and static axis** → brake is zero (in limits) → the
  existing at-target zero-fill guard is unchanged.
- **All axes in limits** → no brake anywhere → v0.7 behaviour exactly.

---

## 5. Phase synchronization + brake (documented limitation)

The global `SYNC_PHASE` branch (`PhaseSynchronize`) does not implement the brake.
If a `Phase`-synchronized call starts from an out-of-limits state, the phase
profile built from the (unbraked) limiting axis will violate limits and fail its
`CheckProfile`, so `PhaseSynchronize` returns `phaseOk = false` and the recompute
falls back to the unified Time flow — which **does** brake. Result:
**Phase + out-of-limits initial state → Time + brake**, which may differ from
Ruckig's Phase-on-post-brake trajectory. This is a documented limitation
(analogous to the v0.7 velocity+Phase limitation). Phase with an in-limits
initial state is unchanged and remains parity-faithful.

---

## 6. Error handling

Unchanged. `ComputeBrakeProfile` cannot fail (it returns a duration ≥ 0).
`ComputeBlock1Axis` / `ComputeProfile1AxisTimed` errors still surface as
`RESULT_ERR_TRAJ`. The brake adds no new status codes.

---

## 7. Testing strategy

No new FC, so validation is parity-driven (plus the existing
`ComputeBrakeProfile` unit tests, unchanged).

### 7.1 Parity scenarios

Single-axis brake (regression guard for the unchanged single-axis path; already
matches to ~1e-15):
- `v08_01` — 1-DoF `v0 > vMax`.
- `v08_02` — 1-DoF `a0 > aMax`.
- `v08_03` — 1-DoF `v0` negative / motion toward the opposite side.

Multi-axis brake (the new path):
- `v08_04` — 2-DoF Time, one axis `v0 > vMax` and that axis is the limiting one.
- `v08_05` — 2-DoF Time, the braked axis is **not** the limiting one (exercises
  the `t_sync − brakeDur` inner re-time offset).
- `v08_06` — 2-DoF, one axis `a0 > aMax`.
- `v08_07` — per-DoF `[No, Time]` with the `No` axis out-of-limits (verifies the
  brake duration enters the `t_sync` floor).
- `v08_08` — (optional) Discrete duration + an out-of-limits axis.

### 7.2 Target

Floating-point floor (~1e-15) on position/velocity/acceleration and duration,
cycle by cycle, as the single-axis brake already achieves. All 237 existing tests
stay green.

---

## 8. Success criteria

- A multi-DoF call with an out-of-limits initial state on one or more axes
  produces a braked, synchronized trajectory matching the Ruckig oracle to the
  floating-point floor, across all reused sync modes.
- In-limit calls are byte-identical to v0.7 (no regression; 237 tests green).
- New `v08_*` parity scenarios pass at the floating-point floor.
- README / CHANGELOG / roadmap updated; version bumped to `0.8.0`; merged and
  tagged `v0.8.0`.
