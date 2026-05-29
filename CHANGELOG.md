# Changelog

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/).

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
