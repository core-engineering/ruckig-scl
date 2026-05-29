# ruckig-scl

A Siemens SCL (Structured Text) port of the [Ruckig](https://github.com/pantor/ruckig)
Online Trajectory Generation library, for S7-1500 PLCs. MIT-licensed (same as
upstream Ruckig).

**Status: v0.1.0 — single-axis, rest-to-rest, position interface
(target velocity & acceleration = 0).**

## Features (v0.1)

- Single-axis jerk-limited (S-curve) trajectory generation
- Point-to-point motion from rest to rest, with the three sub-cases:
  triangular jerk, a_max-reached, and full trapezoidal (v_max reached)
- PLCopen DA011 continuous-enable interface (`enable` / `valid` / `busy` /
  `done` / `error` / `status`)
- DA014 status/error codes via a shared constants block (`dbRuckigConst`)
- Validated cycle-by-cycle against the official Ruckig solver: agreement to
  ~1e-15 (machine epsilon) on all 10 nominal scenarios

## Architecture

One stateful FB orchestrating pure algorithmic FCs, called once per PLC cycle:

| Block | Role |
|-------|------|
| `RuckigOtg` (FB) | Lifecycle: validate → detect change → recompute → advance → evaluate |
| `ValidateInput` (FC) | Bounds / finiteness / consistency checks (DA014 codes) |
| `IsFiniteLreal` (FC) | NaN / Inf guard |
| `ComputeMinDuration` (FC) | Time-optimal rest-to-rest duration |
| `ComputeFinalProfile` (FC) | Fill the 7-phase profile |
| `AdvanceTime` / `StateAtTime` (FC) | Integrate time, evaluate p/v/a |

Data is carried by UDTs (`typeRuckigInput`, `typeRuckigOutput`, `typeProfile`,
`typeTrajectory`, `typeBrakeProfile`). See
[docs/superpowers/specs/2026-05-28-ruckig-scl-port-design.md](docs/superpowers/specs/2026-05-28-ruckig-scl-port-design.md).

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
uv run pytest tests/unit      # 43 unit tests

uv sync --extra parity        # adds the Ruckig reference (PyPI: ruckig)
uv run pytest tests/parity    # 10 cross-implementation parity scenarios
```

## Roadmap

| Version | Scope |
|---------|-------|
| v0.1 | Single-axis, rest-to-rest, position interface *(this release)* |
| v0.2 | Single-axis with arbitrary target velocity / acceleration |
| v0.3–v0.5 | Multi-axis with time synchronization |
| v0.6 | Velocity interface |
| v0.7 | Brake profiles and degenerate cases |
| v0.8 | Performance optimization |
| v0.9 | First public release, after field-validation campaigns |
| v1.0 | Post field-testing |

## License

MIT — see [LICENSE](LICENSE).
