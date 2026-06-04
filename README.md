# ruckig-scl

A Siemens SCL (Structured Text) port of the [Ruckig](https://github.com/pantor/ruckig)
Online Trajectory Generation library, for S7-1500 PLCs. MIT-licensed (same as
upstream Ruckig).

**Status: v0.7.0 — velocity control interface (`controlInterface = IFACE_VELOCITY`,
third-order): drives `(v0, a0) → (vf, af)` jerk-limited with position integrated
but untargeted, wired through all single- and multi-axis synchronization modes
(Time / No / per-DoF / TimeIfNecessary / Discrete). Builds on v0.6's per-DoF
sync completeness.**

## Features (v0.7)

- **Velocity control interface** (`controlInterface = IFACE_VELOCITY`) — select
  third-order velocity targeting instead of the default position interface.
  Drives the velocity state `(v0, a0) → (vf, af)` jerk-limited under the
  acceleration/jerk limits; position is integrated and output but not targeted,
  and `maxVelocity` is ignored (Ruckig parity).
- New FCs: `ComputeVelBlock1Axis` (Step 1 → Block), `ComputeVelProfileTimed`
  (Step 2 re-time to imposed duration), `CheckVelProfile` (velocity-profile
  validity), plus internal helpers `CollectVelStep1Dir` and `SolveVelTimedDir`.
- Wired through the existing multi-axis flow: single-axis and all sync modes
  (Time / No / per-DoF / TimeIfNecessary / Discrete) work for the velocity
  interface. Phase sync falls back to Time for velocity (see Known limitations).

## Features (v0.6)

- **`perDofSynchronization`** (`Array[0..3] of Int`, `-1` = use global mode) —
  each axis independently `No` / `Time` / `TimeIfNecessary`. A `Phase` per-DoF
  entry is treated as `Time`; `Phase` stays a global-only mode.
- **`TimeIfNecessary`** (`SYNC_TIME_IF_NECESSARY = 4`) — an axis is time-synced
  only if its target is moving (`vT != 0` or `aT != 0`); a rest target runs free
  at its time-optimal duration.
- **`DurationDiscretization.Discrete`** — `t_sync` is rounded up to a multiple of
  `cycleTime` (jumping any blocked interval the rounding lands in); every
  synchronized axis is then re-timed to the rounded duration.

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

### Known limitations (v0.7)

- **Per-DoF `Phase` mixes are not supported.** A `Phase` per-DoF entry is
  silently treated as `Time`; `Phase` is a global-only mode (matches Ruckig,
  which abandons phase whenever a Time axis is in the mix).
- **Velocity + Phase synchronization falls back to Time** and does not match
  Ruckig's phase-synced velocity trajectory (scales each axis's profile by the
  velocity-delta ratio for straight-line motion in velocity space). Faithful
  velocity-Phase sync is deferred.
- **No brake pre-phase in the multi-axis path.** Multi-DoF assumes the initial
  states are within limits; the brake machinery runs only on the single-axis
  path. Full brake machinery is deferred to v0.8.
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
| `ComputeVelBlock1Axis` (FC) | Velocity Step 1 → reachable-duration `Block` (tMin + blocked intervals) |
| `ComputeVelProfileTimed` (FC) | Velocity Step 2: re-time a velocity move to an imposed duration `tf` |
| `CheckVelProfile` (FC) | Validate a velocity-mode candidate profile by integration |
| `CollectVelStep1Dir` / `SolveVelTimedDir` (FC) | Internal direction helpers for velocity Step 1 / Step 2 |

Data is carried by UDTs (`typeRuckigInput`, `typeRuckigOutput`, `typeProfile`,
`typeTrajectory`, `typeBrakeProfile`, `typeBlock`, `typeBlockSet`). See the
design specs under [docs/superpowers/specs/](docs/superpowers/specs/)
(`2026-05-28-…-port-design.md` for v0.1, `2026-05-30-…-v0.2-design.md` for v0.2,
`2026-06-02-…-v0.3-design.md` for v0.3, `2026-06-03-…-v0.4-design.md` for v0.4,
`2026-06-03-ruckig-scl-v0.5-design.md` for v0.5,
`2026-06-03-ruckig-scl-v0.6-design.md` for v0.6,
`2026-06-04-ruckig-scl-v0.7-design.md` for v0.7).

## How it was ported

ruckig-scl is a **faithful re-derivation** of Ruckig's algorithms in SCL, not a
mechanical line-by-line translation. The upstream C++ is the reference;
correctness is established **numerically against the real Ruckig solver** rather
than by visual code matching.

### Approach

- **Hybrid: paper-first + C++ reference + parity bench.** Each profile family and
  synchronization mode is re-derived from the kinematics, cross-checked against
  the corresponding Ruckig C++ source, then pinned cycle-by-cycle to the official
  solver. This keeps the SCL idiomatic and readable while every result stays
  anchored to the C++ oracle.
- **MIT throughout**, matching upstream — usable in industrial / OEM contexts
  where the GPLv3 Struckig port cannot be.

### Mapping C++ → SCL

| Ruckig (C++) | ruckig-scl (SCL) |
|---|---|
| `Ruckig<DOF>` class + persistent state | single stateful FB `RuckigOtg` (one implicit cyclic call, no `METHOD`s — idiomatic Siemens) |
| stateless step functions (Step 1 / Step 2, sync, evaluate) | pure FCs (`ComputeBlock1Axis`, `Synchronize`, `ComputeProfile1Axis…`) |
| `double` | `LREAL` throughout (bit-for-bit parity with C++ `double`) |
| `std::array<…, DOF>`, template `DOF` | fixed `ARRAY[0..DOF_MAX-1]`, `DOF_MAX = 4` (SCL requires static array sizing) |
| `enum` (`Synchronization`, `ControlInterface`, …) | typed `INT` constants in `dbRuckigConst` (Siemens ENUMs would need Software Units) |
| `std::optional` / exceptions | DA014 `WORD` status codes + validity flags |
| Cardano / Ferrari / Newton root finders | `SolveCubic` / `SolveQuartic` FCs + safe-Newton `PolyEval` + `ShrinkInterval` |

### How correctness is established

The SCL blocks are **transpiled to Python** by the
[`siemens-plc-tools`](https://github.com/core-engineering/siemens-plc-tools)
`plc-code` engine (an SCL → Python executor), so the *actual* block source runs
in the test harness — there is no separate reimplementation to drift. Two layers:

- **Unit tests** (`tests/unit`) — every FC in isolation: the root solvers, the
  input validator, the per-axis reachable-duration `Block`, the synchronizer, …
- **Numerical parity** (`tests/parity`) — each YAML scenario runs through *both*
  the real Ruckig (PyPI `ruckig`) reference *and* the transpiled SCL, compared
  cycle-by-cycle. The static single-DoF solver matches to the **floating-point
  floor (~1e-15)**; full cyclic trajectories match to **1e-6**.

### Development workflow

Built **one version at a time**, each as a *spec → plan → test-first
implementation → review* cycle, then merged and tagged (`v0.1.0` … `v0.6.0`).
Tests are written before the SCL block and must pass against the Ruckig oracle
before a version ships. The per-version design specs live under
[docs/superpowers/specs/](docs/superpowers/specs/) and the implementation plans
under [docs/superpowers/plans/](docs/superpowers/plans/); see also
[CHANGELOG.md](CHANGELOG.md).

### SCL realities that shaped the code

Porting surfaced several SCL / transpiler constraints that the code deliberately
works around (catalogued in
[docs/PLC_CODE_LIMITATIONS.md](docs/PLC_CODE_LIMITATIONS.md)): no division by a
parenthesized product (denominators are precomputed into a scalar), identifiers
ending in `of` get mis-lexed (so axis indices use `ax` / `refAx`), `END_IF` /
`END_FOR` must sit on their own line, an array-of-UDT is wrapped in a `STRUCT` to
pass as an FC parameter, and so on. These keep the source both transpiler-clean
and valid for TIA Portal.

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
synchronizes them according to `otgInput.synchronization` (Time = arrive
together, Phase = collinear straight-line motion, No = each axis independent).
Set `otgInput.minimumDuration` (≥ 0) to impose a minimum trajectory duration.

## Tests

The SCL blocks are tested in Python via the
[`siemens-plc-tools`](https://github.com/core-engineering/siemens-plc-tools)
`plc-code` transpiler (SCL → Python), checked out as a sibling repo.

```bash
uv sync                       # core deps (offline-installable)
uv run pytest tests/unit      # unit tests

uv sync --extra parity        # adds the Ruckig reference (PyPI: ruckig)
uv run pytest tests/parity    # 45 cross-implementation parity scenarios
```

## Roadmap

| Version | Scope |
|---------|-------|
| v0.1 | Single-axis, rest-to-rest, position interface |
| v0.2 | Single-axis with arbitrary initial / target velocity & acceleration |
| v0.3 | Single-axis step2 (re-time to an imposed duration) |
| v0.4 | Multi-axis time synchronization |
| v0.5 | Multi-axis phase + no synchronization |
| v0.6 | Per-DoF synchronization, TimeIfNecessary, discrete duration |
| v0.7 | Velocity interface *(this release)* |
| v0.8 | Brake profiles and degenerate cases |
| v0.9 | Performance optimization; first public release, after field-validation campaigns |
| v1.0 | Post field-testing |

## License

MIT — see [LICENSE](LICENSE).
