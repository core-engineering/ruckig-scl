# Ruckig SCL v0.6 — Design Spec (synchronization completeness)

**Date** : 2026-06-03
**Owner** : Martin C
**Project** : Port of Ruckig (C++ MIT) to Structured Text for Siemens S7-1500
**Status** : Design draft, pending plan

**Scope:** Complete the multi-axis synchronization feature set by making the
mode decision **per-axis** and adding the two remaining knobs:
- **`per_dof_synchronization`** — each axis independently `No` / `Time` / `Phase`.
- **`TimeIfNecessary`** — synchronize an axis only if its target is moving
  (`vT ≠ 0` or `aT ≠ 0`); otherwise leave it at its time-optimal duration.
- **`DurationDiscretization.Discrete`** — round `t_sync` up to a multiple of the
  control cycle, then re-time every synchronized axis to that discrete duration.

Builds on v0.4 (Block → Synchronize → step2) and v0.5 (Phase / No). The global
synchronization modes become the special case "all axes share one mode".

**Out of scope (later versions):** the velocity control interface
(`ControlInterface.Velocity`) → v0.7; brake profiles → v0.8.

---

## 1. Context and Motivation

v0.4/v0.5 dispatch on a single **global** `synchronization` mode. Ruckig also
supports a **per-DoF** synchronization array and the `TimeIfNecessary` mode,
both of which make the mode decision per-axis, plus duration discretization for
fixed-cycle controllers. v0.6 generalizes the multi-axis path from a global
mode to a per-axis effective mode, which subsumes the global modes and makes the
three remaining features fall out of one unified flow.

Oracle behaviour (probed, `ruckig` 0.17.3), 2-DoF, target `[1, 5]`, limits
`v2 a5 j10`, `dt = 0.1` (axis0 `t_min ≈ 1.47`, axis1 `t_min ≈ 3.39`):

| Input | trajectory.duration | axis0 dur | axis1 dur |
|-------|--------------------|-----------|-----------|
| `Time` | 3.3944 | 3.3944 | 3.3944 |
| `No` | 3.3944 | 1.4736 | 3.3944 |
| `TimeIfNecessary`, rest target | 3.3944 | 1.4736 | 3.3944 |
| `TimeIfNecessary`, axis0 vT=0.5 | 3.3944 | 3.3944 | 3.3944 |
| `per_dof [No, Time]` | 3.3944 | 1.4736 | 3.3944 |
| `Discrete`, dt=0.1 (Time) | 3.4000 | 3.4000 | 3.4000 |

Confirmed: a rest-target `TimeIfNecessary` axis runs free at its `t_min`; a
moving-target one is time-synced. `per_dof` lets a `No` axis run free while a
`Time` axis synchronizes. `Discrete` rounds `t_sync` **up** to the next `dt`.

## 2. Unified per-axis flow

The multi-axis recompute (replacing v0.5's global No/Phase/Time branches) is:

1. **Resolve the effective mode per axis:** `m[ax] = perDofSynchronization[ax]`
   if `≠ -1`, else the global `synchronization`. (`TimeIfNecessary` stays as a
   mode here; it resolves at emission.)
2. **Block per axis:** `ComputeBlock1Axis` for every axis (`tMin` + blocked
   intervals).
3. **Participation mask:** `participates[ax] = (m[ax] ≠ SYNC_NONE)`. Only `No`
   axes are excluded from `t_sync`; `Time`, `Phase`, `TimeIfNecessary` all
   participate (faithful to Ruckig `synchronize()`, where only
   `Synchronization::None` is excluded).
4. **`t_sync`** = `Synchronize` over participating axes (smallest common
   non-blocked duration `≥ max(tMin of participating axes)`), plus the limiting
   axis (slowest participating). If `durationDiscretization = DISC_DISCRETE`:
   `t_sync := ceil(t_sync / cycleTime) · cycleTime`, then advance by `cycleTime`
   while the result is blocked for any participating axis (bounded loop). A
   `2·eps` slack guards the ceil (as in Ruckig).
5. **Emission per axis** (by `m[ax]`):
   - **No** → step1 profile (`ComputeProfile1Axis`), its own duration.
   - **Time** → `ComputeProfile1AxisTimed(t_sync)`.
   - **TimeIfNecessary** → moving target (`|vT| > eps` or `|aT| > eps`):
     `step2(t_sync)`; rest target: step1 (free).
   - **Phase** → scaled from the limiting axis's profile at `t_sync` (collinear
     phase subgroup); on collinearity/limit failure the axis falls back to
     `step2(t_sync)` (treated as Time).
6. `trajectory.duration = max` over all axes; `independentMinDurations[ax] =
   block.tMin`.

The 1-DoF path (`multiPath = false`) is unchanged (strict-parity guard).

## 3. Component changes

### 3.1 `typeRuckigInput` and `dbRuckigConst`
Add `perDofSynchronization : Array[0..3] of Int` defaulting to all `-1`
("use the global mode"). `durationDiscretization : Int` already exists
(`0 = DISC_CONTINUOUS`, `1 = DISC_DISCRETE`) and becomes active.

`dbRuckigConst` currently has `SYNC_NONE=0`, `SYNC_PHASE=1`, `SYNC_TIME=2`,
`SYNC_PER_DOF=3`. `TimeIfNecessary` needs its own mode constant — add
`SYNC_TIME_IF_NECESSARY := 4` (the existing `SYNC_PER_DOF=3` is not a per-axis
mode value and stays unused as a mode; per-DoF is expressed by the
`perDofSynchronization` array, not by a global mode). Per-axis values in
`perDofSynchronization` therefore use `{SYNC_NONE, SYNC_PHASE, SYNC_TIME,
SYNC_TIME_IF_NECESSARY}` or `-1` for fallback.

### 3.2 `Synchronize` (extended)
Add VAR_INPUT `participates : Array[0..3] of Bool`, `discrete : Bool`,
`cycleTime : LReal`. The `tStart` (max tMin), candidate collection, blocked-check
and limiting-axis selection all skip axes where `participates[ax]` is false. After
selecting the smallest non-blocked candidate, if `discrete`: round up to the next
`cycleTime` multiple and advance by `cycleTime` while blocked for any
participating axis (bounded by a small iteration cap). The v0.4/v0.5 Time path
calls it with an all-true mask and `discrete = false` → identical result
(regression guard).

### 3.3 `PhaseSynchronize` (generalized)
v0.5 computed its own reference (per-axis step1, slowest = reference) and scaled
**all** axes. v0.6 generalizes it to: given the **reference profile** (the
limiting axis at `t_sync`, already re-timed) and a per-axis **`isPhase` mask**,
test collinearity of the phase subgroup against the reference and scale only the
phase axes (shared `t[]`, jerk `× kd = sval[ax]/sval[refAx]`), validating each
with `CheckProfile(tf := t_sync)`. The all-axes-phase, self-computed-reference
case (v0.5 global Phase) remains expressible (reference = slowest, mask = all).

### 3.4 `RuckigOtg`
The multi-axis block becomes the unified flow of §2. Mode resolution is a small
inline step (or a helper `ResolveSyncModes` filling `modeEff[ax]` and
`participates[ax]`). Brake fields are zeroed per axis (no multi-axis brake, as in
v0.4). Change detection already covers `synchronization` (v0.5); it additionally
recomputes on a change to `perDofSynchronization` or `durationDiscretization`.

## 4. Testing

**Unit:**
- `Synchronize`: participation mask (a `No` axis's tMin is ignored for `t_sync`);
  Discrete (rounds up to the dt grid; jumps a blocked interval landed on by the
  rounding).
- `PhaseSynchronize` (generalized): phase subgroup scaled to a supplied reference;
  non-phase axes untouched; collinearity/limit failure flagged.
- Mode resolution: `perDof` overrides global; sentinel `-1` falls back; the
  participation mask excludes `No`.

**Unit (FB):**
- `per_dof` mixes: `[No, Time]`, `[Time, No]`, `[Phase, Time]`, `[Phase, No]` —
  each axis behaves per its mode, all reach target.
- `TimeIfNecessary`: rest target runs free at `t_min`; moving target syncs.
- `Discrete`: `trajectory.duration` is a multiple of `cycleTime`, axes reach
  target.
- **Regression guard:** the global modes (`No` / `Phase` / `Time` for all axes)
  produce the same result as v0.5 through the unified flow.

**Parity (vs Ruckig oracle):**
- Thread `perDofSynchronization` and `durationDiscretization` through the YAML
  scenarios and both runners (`runner_ref` maps the per-axis ints to the Ruckig
  enum and sets `inp.per_dof_synchronization` / `inp.duration_discretization`).
- New `v06_*` scenarios: per_dof `[No, Time]`, per_dof `[Phase, Time]`,
  `TimeIfNecessary` (mixed rest/moving), `Discrete`. Floating-point-floor match.
- The 31 existing v0.4/v0.5 parity scenarios and the 195-test suite stay green.

## 5. Components summary

| Block | Status | Role |
|-------|--------|------|
| `RuckigOtg` (FB) | modified | unified per-axis multi-axis flow (subsumes v0.5 global modes) |
| `Synchronize` (FC) | modified | + participation mask, + Discrete rounding |
| `PhaseSynchronize` (FC) | modified | generalized: scale a phase subgroup to a supplied reference |
| `typeRuckigInput` (UDT) | modified | + `perDofSynchronization`; `durationDiscretization` activated |
| `ComputeBlock1Axis`, `ComputeProfile1Axis`, `ComputeProfile1AxisTimed` | reused | per-axis Block / step1 / step2 |

## 6. Risks / open points

- **Phase in a mixed bag** is the subtle part: the phase reference is the
  limiting axis's profile at `t_sync` (which may itself be a `Time` axis). The
  per-axis `CheckProfile` is the backstop — a phase axis that cannot match the
  reference within limits falls back to `step2(t_sync)`, never emitting a wrong
  trajectory (matches Ruckig abandoning phase sync when infeasible).
- **Discrete + blocked intervals**: rounding `t_sync` up to a dt multiple may
  land inside a blocked interval; the bounded advance-by-dt loop steps out of it.
  A small iteration cap prevents any unbounded loop on a PLC.
- **Regression of the global modes** is the chief integration risk; it is fully
  covered by the existing 195 tests + 31 parity scenarios, which the unified
  flow must reproduce exactly.
