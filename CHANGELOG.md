# Changelog

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/).

## [0.7.0] - 2026-06-04

### Added
- **Velocity control interface** (`controlInterface = IFACE_VELOCITY`,
  third-order). Drives the velocity state `(v0, a0) -> (vf, af)` jerk-limited
  under the acceleration/jerk limits; position is uncontrolled (integrated and
  output, not targeted), `maxVelocity` is ignored (Ruckig parity).
- New FCs: `ComputeVelBlock1Axis` (Step 1 -> Block), `ComputeVelProfileTimed`
  (Step 2 re-time), `CheckVelProfile` (velocity validity), plus the internal
  direction helpers `CollectVelStep1Dir` and `SolveVelTimedDir`.
- Wired through the existing multi-axis flow: single-axis plus Time / No /
  per-DoF / TimeIfNecessary / Discrete synchronization all work for velocity.
- `ValidateInput` velocity-mode rules and `RESULT_ERR_IFACE` (16#8208) for an
  out-of-range `controlInterface`.
- 8 velocity parity scenarios (`v07_01`..`v07_08`).

### Known limitations
- **Velocity + Phase synchronization falls back to Time** and does NOT match
  Ruckig, which genuinely phase-syncs the velocity interface (scales each axis's
  profile by the velocity-delta ratio for straight-line motion in velocity
  space). Faithful velocity-Phase is deferred. All other velocity sync modes
  (Time / No / per-DoF / TimeIfNecessary / Discrete) match the oracle.

### Notes
- Second-order (jerk-unlimited) velocity interface is out of scope; the port is
  third-order throughout.
- No brake pre-phase in velocity mode (the Step-1 solver absorbs an arbitrary
  `a0`).

## [0.6.0] - 2026-06-03

Multi-axis synchronization completeness, on top of v0.5.

### Added
- **`perDofSynchronization`** (`typeRuckigInput`, `Array[0..3] of Int`, `-1` =
  use the global mode) — each axis independently `No` / `Time` /
  `TimeIfNecessary`. (A `Phase` per-DoF entry is treated as `Time`; `Phase`
  stays a global mode.)
- **`TimeIfNecessary`** (`SYNC_TIME_IF_NECESSARY = 4`) — an axis is time-synced
  only if its target is moving (`vT != 0` or `aT != 0`); a rest target runs free
  at its time-optimal duration.
- **`DurationDiscretization.Discrete`** — `t_sync` is rounded up to a multiple of
  `cycleTime` (jumping any blocked interval the rounding lands in), then every
  synchronized axis is re-timed to it.

### Changed
- `RuckigOtg`'s multi-axis recompute keeps the global `Phase` branch and replaces
  the global `No`/`Time` branches with one unified per-axis flow (resolve mode ->
  Block participating axes -> `Synchronize` over a participation mask -> emit per
  axis). The global modes are the special case "all axes share one mode".
- `Synchronize` gains a per-axis participation mask (only `No` axes are excluded
  from `t_sync`) and discrete-duration rounding.

### Parity
- 4 new scenarios (`v06_01..04`): per-DoF `[No, Time]`, `TimeIfNecessary` rest and
  moving, discrete duration. 35 parity scenarios total. 206 tests pass.

### Known limitations
- Per-DoF `Phase` mixes are not supported (a Phase per-DoF entry -> Time); Phase is
  a global mode (matches Ruckig, which abandons phase whenever a Time axis is in
  the mix). `SYNC_PER_DOF` is not a valid global `synchronization` value (it is the
  per-axis array mechanism). The velocity control interface -> v0.7; brake
  profiles -> v0.8.

## [0.5.0] - 2026-06-03

Two additional multi-axis synchronization modes on top of v0.4's Time sync.

### Added
- `Synchronization.Phase` — when the per-DoF state deltas `(pd, v0, a0, vT, aT)`
  are collinear, every axis follows the limiting axis's jerk-switch timing with
  jerk scaled by the displacement ratio (straight-line motion in joint space).
  When not collinear, falls back to Time sync (exact Ruckig behaviour). New FC
  `PhaseSynchronize` (port of `is_input_collinear` + phase build).
- `Synchronization.No` — each axis runs at its own time-optimal duration,
  finishing independently; `trajectory.duration = max`.
- `dbRuckigConst.EPS_PHASE` (collinearity tolerance).

### Changed
- `RuckigOtg` dispatches on `input.synchronization`. The v0.4 Time path is
  unchanged and serves as the Phase fallback.
- `typeRuckigInput.synchronization` default -> `SYNC_TIME` (matches Ruckig's
  default; preserves v0.4 behaviour for callers that do not set it).

### Parity
- 4 new scenarios (`v05_01..03` + `v05_04` no-stationary-axis regression): No
  (independent), Phase collinear (exact), Phase non-collinear (Time fallback),
  and a zero-displacement axis. Floating-point-floor match. 31 parity
  scenarios total. 195 tests pass.

### Known limitations
- `TimeIfNecessary`, `DurationDiscretization.Discrete`, and
  `per_dof_synchronization` are not implemented (v0.6).
- No brake pre-phase in the multi-axis path (v0.7); step1 ~3% gaps (v0.2).

## [0.4.0] - 2026-06-03

**Multi-axis time synchronization.** `RuckigOtg` now drives up to 4 DoFs so that
every axis reaches its target at the same instant `t_sync`. This wires the v0.3
single-axis step2 into Ruckig's full Block → synchronize → re-time pipeline.

### Added
- `ComputeBlock1Axis` (FC) — runs the step1 families collecting *every* valid
  candidate duration, then derives the reachable-duration `Block`: `tMin` plus up
  to two **blocked intervals** (durations that no jerk-limited profile can
  achieve). Port of Ruckig `Block::calculate_block` (`block.hpp`), durations
  only. Includes an already-at-target degenerate guard (`tMin = 0`).
- `Synchronize` (FC) — given each DoF's `Block`, finds `t_sync` = the smallest
  common duration `>= max(tMin)` that is **not blocked** for any axis (skipping
  blocked intervals by jumping to their right edge), plus the limiting axis. Port
  of Ruckig `TargetCalculator::synchronize` (Time mode).
- `typeBlock` / `typeBlockSet` UDTs — per-DoF reachable-duration blocks (the
  wrapper `typeBlockSet` exists because an array-of-UDT cannot be a direct FC
  parameter).
- `typeRuckigInput.minimumDuration` (sentinel `< 0` = none) — impose a floor on
  the trajectory duration.
- `dbRuckigConst.RESULT_ERR_SYNC` (0x8603).

### Changed
- `RuckigOtg` (FB) refactored single-axis → multi-DoF, with two solve paths:
  - **1 DoF and no `minimumDuration`** → the v0.3 step1 stack (brake +
    `ComputeProfile1Axis`), kept identical (strict-parity regression guard).
  - **>1 DoF or `minimumDuration` set** → v0.4: `ComputeBlock1Axis` per axis →
    `Synchronize` → `ComputeProfile1AxisTimed(tf = t_sync)` per axis.
  Chain state, change detection, sampling and `pass_to_input` are generalized to
  N DoFs.
- The brake pre-phase now seeds the post-brake state from `profile.brake`
  (reliable VAR_IN_OUT) instead of the FC's `=>` outputs — the latter are dropped
  by the transpiler when a function's return value is consumed, which had
  silently mis-seeded the 1-DoF over-`vMax` first-enable path. It is now
  C1-continuous across the brake/main junction.

### Parity
- 9 new multi-DoF scenarios (`v04_01`..`v04_09`): 2–4 DoF rest sync, heavily
  stretched fast axis, moving targets, full arbitrary states, **blocked-interval
  synchronization** (`max(tMin)` lands in an axis's blocked interval, so `t_sync`
  advances to the interval's right edge), heterogeneous limits, and
  `minimum_duration` (1- and 2-DoF). All match Ruckig at the floating-point floor
  (~1e-15), duration exact. 27 parity scenarios total. 181 tests pass.

### Known limitations
- **No brake pre-phase in the multi-axis path** — multi-DoF assumes initial
  states within limits; the brake machinery runs only on the 1-DoF path.
- **Time synchronization only** — `Phase` / `None` / per-DoF synchronization
  modes and duration discretization are not implemented.
- step1 (time-optimal) retains its ~3% arbitrary-state gaps from v0.2.

## [0.3.0] - 2026-06-03

Single-axis **step2**: re-time a move to an imposed duration `tf >= t_min`.
This is Ruckig's second solver stage and the foundation for v0.4 multi-axis
time synchronization. FC-only (the `RuckigOtg` FB is unchanged; a
`minimumDuration` input ships with v0.4).

### Added
- `ComputeProfile1AxisTimed` (FC) — step2 entry point. Given `tf`, finds the
  jerk-limited profile reaching the arbitrary final state `(pT, vT, aT)` exactly
  at `tf`. Dispatches the 8 step2 families in Ruckig's order, both directions,
  **UDDU and UDUD** control signs (UDUD is new to the project).
- 8 step2 family FCs: `SolveTimedAcc0Acc1Vel`, `SolveTimedVel` (degree-5/6
  polynomial), `SolveTimedAcc0Vel`, `SolveTimedAcc1Vel`, `SolveTimedAcc0Acc1`
  (free jerk), `SolveTimedAcc0`, `SolveTimedAcc1`, `SolveTimedNone`.
- `PolyEval` / `ShrinkInterval` FCs — polynomial evaluation (Horner) and
  safe-Newton root bracketing (ports of Ruckig `roots.hpp`). `time_vel` finds
  its degree-5/6 real roots by a dense sign-change scan (with local-minimum
  detection for tangent roots) + `ShrinkInterval`.
- `CheckProfile` gains an optional imposed-duration check (`tf`; sentinel
  `tf < 0` disables it, so step1 callers are unaffected).

### Parity / validity
- Velocity-reaching families (`*_vel`) match Ruckig to ~1e-6 (unique shapes).
- No-plateau families (NONE/ACC0/ACC1) use a **validity** criterion: at an
  imposed `tf` several valid trajectories exist, so the solver returns one that
  reaches the state at `tf` within limits (incl. interior velocity extrema,
  guaranteed by `CheckProfile`) — not necessarily Ruckig's exact shape.
- Validity sweep over arbitrary states × reachable `tf`: 720/720 valid, 0
  errors, **0 invalid trajectories**.

### Known limitations
- **Blocked durations**: when `tf` falls in a blocked interval (no jerk-limited
  profile of exactly that duration exists), `ComputeProfile1AxisTimed` returns
  `RESULT_ERR_SOLVER`. Detecting/avoiding blocked intervals is the `Block` /
  synchronization layer, deferred to v0.4 (which picks a non-blocked `t_sync`).
- step1 (time-optimal) retains its ~3% arbitrary-state gaps from v0.2; unrelated
  to step2.
- Multi-axis synchronization, `RuckigOtg.minimumDuration` — v0.4.

## [0.2.1] - 2026-06-02

Correctness and moving-target parity fixes on top of v0.2.0. No interface or
UDT changes.

### Fixed
- **Interior velocity-limit violation (safety).** `CheckProfile` only sampled
  velocity at phase boundaries, so a profile whose velocity peaked *between*
  nodes — when the acceleration crosses zero inside a phase — could pass
  validation and, being shorter, win the minimum-duration selection. The solver
  could therefore emit a trajectory exceeding `vMax`. `CheckProfile` now also
  checks the interior extremum `v_a_zero = v[i] - a[i]^2 / (2*j[i])` for phases
  `i >= 2` (port of Ruckig `profile.hpp` `check<>`, lines 249-254); phases 0-1
  stay exempt, as in Ruckig.
- **Moving-target terminal sample.** Past `duration`, `StateAtTime` now
  extrapolates along the final state with jerk = 0 (`a` stays = `aT`) and
  `AdvanceTime` no longer clamps `currentTime` to `duration`, matching Ruckig's
  `at_time(t > duration)`. Moving-target scenarios now reach floating-point-floor
  parity (~1e-15) on **every** cycle, including the finishing one; the six
  per-scenario `tol_position`/`tol_velocity` overrides are removed.

### Parity
- All 18 parity scenarios pass at the strict default 1e-6 tolerance (measured
  residual ~1e-15), including the moving-target / nonzero-target-acceleration
  ones that previously needed relaxed overrides.

### Known limitations
- **Two-step solver fallbacks not yet ported.** For ~3% of arbitrary
  initial/target state combinations the three main profile families yield no
  feasible profile and the solver returns `RESULT_ERR_SOLVER`. Ruckig recovers
  these via `time_none/acc0/vel/acc1_vel_two_step` (`position_third_step1.cpp`
  lines 287-443); deferred to a dedicated pass. A handful of further cases solve
  feasibly but not yet time-optimally.
- **Brake concatenation is feasible but not time-optimal** (deferred to v0.7).
- The zero-limits special case (`jMax`/`aMax` = 0) is unreachable here:
  `ValidateInput`'s positive-limit checks reject it upstream.

## [0.2.0] - 2026-05-30

Single-axis, arbitrary initial **and** target states (`v0, a0 ≠ 0`,
`vT, aT ≠ 0`), online `pass_to_input` chaining, and a brake pre-phase for an
out-of-limits first enable. Builds on the v0.1 FB/UDT layer.

### Added
- `SolveCubic` / `SolveQuartic` FCs — closed-form real-root solvers (Cardano /
  Ferrari-via-resolvent-cubic), backing the profile-type equations.
- `ComputeProfile1Dof` (FUNCTION `ComputeProfile1Axis`) — the general
  single-DoF time-optimal solver: enumerates Ruckig's profile families
  (`time_all_vel`, `time_acc0_acc1`, `time_all_none_acc0_acc1`) in both jerk
  directions via `SolveDirection`, validates each candidate with
  `CheckProfile`, and selects the minimum-duration feasible profile.
- `IntegrateProfileStates` FC — fills `a/v/p` from `t/j` and `(p0,v0,a0)`.
- `ComputeBrakeProfile` FC — brake sub-profile (port of Ruckig's position
  brake) when the initial state is out of limits; stored in `profile.brake`.
- `StateAtTime` extended to evaluate the brake prefix before the main profile.
- `RuckigOtg` FB: `pass_to_input` chaining (C2-continuous retarget), brake
  pre-phase, and change detection extended to target velocity/acceleration.
- 8 v0.2 parity scenarios (moving target, non-rest start, full-arbitrary,
  negative move) — 18 scenarios total.

### Changed
- `ValidateInput` no longer rejects an out-of-limits **current** state
  (matches Ruckig's `check_current_state_within_limits = false` default); the
  brake pre-phase absorbs it. Finiteness, positive-limit, and nDofs checks
  are retained.
- The general solver replaces the v0.1 rest-to-rest-specific
  `ComputeMinDuration` + `ComputeFinalProfile`, which are **removed** (the
  rest-to-rest behaviour is reproduced as a special case).

### Parity
- Static single-DoF solver vs the official Ruckig solver: agreement at the
  floating-point floor (~1e-15) across the velocity-reaching, short-move
  (ACC0_ACC1), and quartic (NONE/ACC0/ACC1) families, in both directions.
- Cyclic FB parity: rest-to-rest and zero-target-velocity scenarios match to
  strict 1e-6; the v0.1 nominal suite is reproduced exactly.

### Known limitations
- **Moving-target terminal sample**: with a non-zero target velocity, every
  cycle matches Ruckig to ~1e-15 except the single finishing cycle. Ruckig
  samples it at grid time `t = N·dt < duration` and keeps moving along the
  target, whereas `AdvanceTime` clamps `currentTime` to the trajectory
  duration; the bounded (~1e-3) one-sample delta is covered by a documented
  per-scenario position tolerance. Zero-target-velocity motion is unaffected.
- **Brake concatenation is feasible but not time-optimal**: at a first enable
  in motion (`|v0| > vMax`), the FB rides a once-computed brake + main-profile
  concatenation. It brakes correctly and reaches the target, but is longer
  than Ruckig's per-cycle recompute. Full brake machinery is deferred to v0.7.
- Solver gaps not yet ported (rarely the time-optimal selection; the solver
  returns `RESULT_ERR_SOLVER` if ever required): the zero-limits special case
  and the two-step fallbacks (`time_*_two_step`).

[0.7.0]: https://github.com/core-engineering/ruckig-scl/releases/tag/v0.7.0
[0.6.0]: https://github.com/core-engineering/ruckig-scl/releases/tag/v0.6.0
[0.5.0]: https://github.com/core-engineering/ruckig-scl/releases/tag/v0.5.0
[0.4.0]: https://github.com/core-engineering/ruckig-scl/releases/tag/v0.4.0
[0.3.0]: https://github.com/core-engineering/ruckig-scl/releases/tag/v0.3.0
[0.2.1]: https://github.com/core-engineering/ruckig-scl/releases/tag/v0.2.1
[0.2.0]: https://github.com/core-engineering/ruckig-scl/releases/tag/v0.2.0

## [0.1.0] - 2026-05-29

First milestone: single-axis, rest-to-rest, position interface.

### Added
- 5 UDTs (`typeRuckigInput`, `typeRuckigOutput`, `typeProfile`,
  `typeBrakeProfile`, `typeTrajectory`) and the `dbRuckigConst` constants block
  (DA014 status codes, EPS_*, SYNC_*).
- 6 pure FCs: `IsFiniteLreal`, `ValidateInput`, `ComputeMinDuration`,
  `ComputeFinalProfile`, `AdvanceTime`, `StateAtTime`.
- `RuckigOtg` FB with the PLCopen DA011 continuous-enable lifecycle
  (validate → change-detect → recompute → advance → evaluate).
- Test bench (via the `siemens-plc-tools` plc-code transpiler): 43 unit tests.
- Parity bench against the official Ruckig solver (PyPI `ruckig`, `parity`
  extra): 10 nominal scenarios, agreement to ~1e-15.
- GitHub Actions CI (unit + parity).
- Example: single-axis point-to-point.

### Notes
- Written in TIA Portal SCL export format throughout (inter-FC calls via
  `"BlockName"(...)`, shared constants via `"dbRuckigConst".X`).
- Roadmap: v0.2 (arbitrary target states) → v0.9 (first public release, after
  field validation) → v1.0 (post field-testing).

[0.1.0]: https://github.com/core-engineering/ruckig-scl/releases/tag/v0.1.0
