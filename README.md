# ruckig-scl

A Siemens SCL (Structured Text) port of the [Ruckig](https://github.com/pantor/ruckig)
Online Trajectory Generation library, for S7-1500 PLCs. MIT-licensed (same as
upstream Ruckig).

**Status: v0.2.1 — single-axis, arbitrary initial AND target states
(`v0, a0 ≠ 0`, target velocity & acceleration ≠ 0), with online retarget
chaining and a brake pre-phase. v0.2.1 fixes an interior velocity-limit check
and brings moving targets to floating-point-floor parity.**

## Features (v0.2)

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

### Known limitations (v0.2.1)

- **Two-step solver fallbacks not yet ported.** For ~3% of arbitrary
  initial/target state combinations the three main profile families yield no
  feasible profile and the solver returns `RESULT_ERR_SOLVER` (Ruckig recovers
  these via its `time_*_two_step` paths). Deferred to a dedicated pass; a few
  further cases solve feasibly but not yet time-optimally.
- The brake pre-phase brakes correctly and reaches the target, but rides a
  once-computed brake + main concatenation that is feasible, **not**
  time-optimal. Full brake machinery is deferred to v0.7.

Resolved in v0.2.1: moving targets now match Ruckig to ~1e-15 on **every**
cycle (the FB extrapolates past `duration` exactly as Ruckig does), and the
feasibility check no longer misses velocity peaks that occur between phase
nodes.

## Architecture

One stateful FB orchestrating pure algorithmic FCs, called once per PLC cycle:

| Block | Role |
|-------|------|
| `RuckigOtg` (FB) | Lifecycle: validate → detect change → (brake +) recompute → advance → evaluate; `pass_to_input` chaining |
| `ValidateInput` (FC) | Finiteness / positive-limit / nDofs checks (DA014 codes) |
| `IsFiniteLreal` (FC) | NaN / Inf guard |
| `ComputeProfile1Dof` (FC, `ComputeProfile1Axis`) | General time-optimal single-DoF solver (enumerate families × directions, select min-duration) |
| `SolveDirection` (FC) | Runs the three profile families for one jerk direction |
| `SolveCubic` / `SolveQuartic` (FC) | Closed-form real-root solvers (Cardano / Ferrari) |
| `IntegrateProfileStates` (FC) | Fill `a/v/p` from `t/j` + initial state |
| `CheckProfile` (FC) | Validate a candidate profile by integration |
| `ComputeBrakeProfile` (FC) | Brake sub-profile for an out-of-limits initial state |
| `AdvanceTime` / `StateAtTime` (FC) | Integrate time, evaluate p/v/a (brake prefix first) |

Data is carried by UDTs (`typeRuckigInput`, `typeRuckigOutput`, `typeProfile`,
`typeTrajectory`, `typeBrakeProfile`). See the design specs under
[docs/superpowers/specs/](docs/superpowers/specs/)
(`2026-05-28-…-port-design.md` for v0.1, `2026-05-30-…-v0.2-design.md` for v0.2).

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

## Tests

The SCL blocks are tested in Python via the
[`siemens-plc-tools`](https://github.com/core-engineering/siemens-plc-tools)
`plc-code` transpiler (SCL → Python), checked out as a sibling repo.

```bash
uv sync                       # core deps (offline-installable)
uv run pytest tests/unit      # unit tests

uv sync --extra parity        # adds the Ruckig reference (PyPI: ruckig)
uv run pytest tests/parity    # 18 cross-implementation parity scenarios
```

## Roadmap

| Version | Scope |
|---------|-------|
| v0.1 | Single-axis, rest-to-rest, position interface |
| v0.2 | Single-axis with arbitrary initial / target velocity & acceleration *(this release)* |
| v0.3–v0.5 | Multi-axis with time synchronization |
| v0.6 | Velocity interface |
| v0.7 | Brake profiles and degenerate cases |
| v0.8 | Performance optimization |
| v0.9 | First public release, after field-validation campaigns |
| v1.0 | Post field-testing |

## License

MIT — see [LICENSE](LICENSE).
