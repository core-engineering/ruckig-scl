# Changelog

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/).

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
