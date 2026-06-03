# ruckig-scl

A Siemens SCL (Structured Text) port of the [Ruckig](https://github.com/pantor/ruckig)
Online Trajectory Generation library, for S7-1500 PLCs. MIT-licensed (same as
upstream Ruckig).

**Status: v0.5.0 — multi-axis Phase and No synchronization modes, building on
v0.4's Time sync. `Synchronization.Phase` produces straight-line joint-space
motion when axes are collinear (falls back to Time when not); `Synchronization.No`
lets each axis finish at its own time-optimal duration. On top of v0.4 multi-axis
Time sync and the v0.2.1 time-optimal (step1) single-axis solver.**

## Features (v0.5)

- **`Synchronization.Phase`** — when per-DoF state deltas `(pd, v0, a0, vT, aT)`
  are collinear, every axis follows the limiting axis's jerk-switch timing with
  jerk scaled by the displacement ratio (straight-line motion in joint space).
  When not collinear, falls back to Time sync (exact Ruckig behaviour). New FC
  `PhaseSynchronize`.
- **`Synchronization.No`** — each axis runs at its own time-optimal duration,
  finishing independently; `trajectory.duration = max`. No inter-axis coupling.
- `typeRuckigInput.synchronization` selects the mode; default `SYNC_TIME`
  preserves v0.4 behaviour for callers that do not set it.

## Features (v0.4)

- **Multi-axis time synchronization** (≤ 4 DoF): per-axis reachable-duration
  `Block` (with blocked intervals) → common `t_sync` → step2 re-time of every
  axis, so all axes arrive together. Matches Ruckig to ~1e-15.
- **`minimumDuration`** input — impose a floor on the trajectory duration
  (routes even a single DoF through step2).
- Single-axis **step2** (re-timing an arbitrary move to an imposed duration
  `tf >= t_min`): 8 profile families × 2 directions (UDDU and UDUD).

## Features (single-axis, v0.2)

- Single-axis jerk-limited (S-curve), **time-optimal** trajectory generation
  for arbitrary `(p0, v0, a0) → (pT, vT, aT)` within symmetric limits
- Full profile-family solver (velocity-reaching, short-move `ACC0_ACC1`, and
  quartic `NONE/ACC0/ACC1`), in both jerk directions, backed by closed-form
  cubic/quartic root solvers — selects the minimum-duration feasible profile
- Online `pass_to_input` chaining: retarget mid-motion with C2 continuity
  (no setpoint jump)
- Brake pre-phase when the initial state is out of limits (first enable in
  motion)
- PLCopen DA011 continuous-enable interface (`enable` / `valid` / `busy` /
  `done` / `error` / `status`) and DA014 status/error codes (`dbRuckigConst`)
- Validated against the official Ruckig solver: the static single-DoF solver
  agrees to the floating-point floor (~1e-15) across all profile families;
  cyclic rest-to-rest / zero-target-velocity parity holds to 1e-6

### Known limitations (v0.5)

- **No brake pre-phase in the multi-axis path.** Multi-DoF assumes the initial
  states are within limits; the brake machinery runs only on the single-axis
  path. Full brake machinery is deferred to v0.7.
- **`TimeIfNecessary`, `DurationDiscretization.Discrete`, and
  `per_dof_synchronization` not implemented.** Deferred to v0.6.
- **step1 two-step fallbacks not yet ported.** For ~3% of arbitrary
  initial/target state combinations the three main profile families yield no
  feasible profile and the solver returns `RESULT_ERR_SOLVER` (Ruckig recovers
  these via its `time_*_two_step` paths). Deferred to a dedicated pass.

Resolved earlier: moving targets match Ruckig to ~1e-15 on **every** cycle
(v0.2.1), and the over-`vMax` first-enable brake seed is now C1-continuous
(v0.4).

## Architecture

One stateful FB orchestrating pure algorithmic FCs, called once per PLC cycle:

| Block | Role |
|-------|------|
| `RuckigOtg` (FB) | Lifecycle: validate → detect change → recompute → advance → evaluate. Dispatches on `input.synchronization` (Time / Phase / No). Multi-DoF: `Block` per axis → `Synchronize` → step2 re-time per axis. Single-DoF keeps the v0.3 step1 path. `pass_to_input` chaining |
| `PhaseSynchronize` (FC) | Phase sync — collinearity test on per-DoF state deltas; builds scaled jerk profile from the limiting axis for each collinear DoF; returns fallback flag when not collinear |
| `ComputeBlock1Axis` (FC) | step1 → reachable-duration `Block` (`tMin` + blocked intervals) |
| `Synchronize` (FC) | Common `t_sync` ≥ max(`tMin`) avoiding every axis's blocked intervals; limiting axis |
| `ComputeProfile1AxisTimed` (FC) | Single-DoF step2: re-time a move to an imposed duration `tf` (8 families × 2 directions) |
| `ValidateInput` (FC) | Finiteness / positive-limit / nDofs checks (DA014 codes) |
| `IsFiniteLreal` (FC) | NaN / Inf guard |
| `ComputeProfile1Dof` (FC, `ComputeProfile1Axis`) | General time-optimal single-DoF solver (enumerate families × directions, select min-duration) |
| `SolveDirection` (FC) | Runs the profile families for one jerk direction |
| `SolveCubic` / `SolveQuartic` (FC) | Closed-form real-root solvers (Cardano / Ferrari) |
| `PolyEval` / `ShrinkInterval` (FC) | Horner evaluation + safe-Newton root bracketing (step2 degree-5/6 roots) |
| `IntegrateProfileStates` (FC) | Fill `a/v/p` from `t/j` + initial state |
| `CheckProfile` (FC) | Validate a candidate profile by integration |
| `ComputeBrakeProfile` (FC) | Brake sub-profile for an out-of-limits initial state |
| `AdvanceTime` / `StateAtTime` (FC) | Integrate time, evaluate p/v/a (brake prefix first) |

Data is carried by UDTs (`typeRuckigInput`, `typeRuckigOutput`, `typeProfile`,
`typeTrajectory`, `typeBrakeProfile`, `typeBlock`, `typeBlockSet`). See the
design specs under [docs/superpowers/specs/](docs/superpowers/specs/)
(`2026-05-28-…-port-design.md` for v0.1, `2026-05-30-…-v0.2-design.md` for v0.2,
`2026-06-02-…-v0.3-design.md` for v0.3, `2026-06-03-…-v0.4-design.md` for v0.4).

## Target platform

- TIA Portal V18 or later
- Siemens S7-1500, firmware V3.0 or later
- Optimized access (`S7_Optimized := "TRUE"`)

## Usage

```scl
VAR
    instOtg   : "RuckigOtg";
    otgInput  : "typeRuckigInput";
    otgOutput : "typeRuckigOutput";
END_VAR

// One-time configuration
otgInput.nDofs := 1;
otgInput.maxVelocity[0] := 2.0;
otgInput.maxAcceleration[0] := 5.0;
otgInput.maxJerk[0] := 10.0;
otgInput.enabled[0] := TRUE;

// Cyclic call (e.g. in a fast-motion OB)
otgInput.targetPosition[0]  := someTargetValue;
otgInput.currentPosition[0] := encoderFeedback;

instOtg(enable    := TRUE,
        input     := otgInput,
        cycleTime := 0.010,
        reset     := FALSE,
        output    => otgOutput);

// otgOutput.newPosition[0] -> position setpoint for the drive
```

See [`examples/single-axis-point-to-point/`](examples/single-axis-point-to-point/).

For multi-axis motion, set `otgInput.nDofs := N` (≤ 4) and fill the per-DoF
arrays (`maxVelocity[d]`, `targetPosition[d]`, …) for each axis; the FB
time-synchronizes them so all axes reach their targets together. Set
`otgInput.minimumDuration` (≥ 0) to impose a minimum trajectory duration.

## Tests

The SCL blocks are tested in Python via the
[`siemens-plc-tools`](https://github.com/core-engineering/siemens-plc-tools)
`plc-code` transpiler (SCL → Python), checked out as a sibling repo.

```bash
uv sync                       # core deps (offline-installable)
uv run pytest tests/unit      # unit tests

uv sync --extra parity        # adds the Ruckig reference (PyPI: ruckig)
uv run pytest tests/parity    # 30 cross-implementation parity scenarios
```

## Roadmap

| Version | Scope |
|---------|-------|
| v0.1 | Single-axis, rest-to-rest, position interface |
| v0.2 | Single-axis with arbitrary initial / target velocity & acceleration |
| v0.3 | Single-axis step2 (re-time to an imposed duration) |
| v0.4 | Multi-axis time synchronization |
| v0.5 | Phase + No synchronization modes *(this release)* |
| v0.6 | per-DoF synchronization, duration discretization, TimeIfNecessary |
| v0.7 | Velocity interface |
| v0.8 | Brake profiles and degenerate cases |
| v0.9 | Performance optimization |
| v1.0 | First public release, after field-validation campaigns |

## License

MIT — see [LICENSE](LICENSE).
