# Performance characterization (operation-count proxy)

Counts of SCL-level operations per `RuckigOtg` update, measured by
hooking the `plc-code` executor (FC-dispatch + `math.sqrt`). These are a
**relative cost proxy** (not microseconds) for hot-path ranking and the
steady-state vs recompute split. Real cycle-time is measured on PLCSIM
Advanced (see `benchmark/README.md`).

## Per-cycle cost

| Scenario | recompute: FC calls | recompute: sqrt | steady: FC calls | steady: sqrt |
|---|---|---|---|---|
| 1-DoF position | 44 | 21 | 9 | 0 |
| 2-DoF position | 240 | 97 | 16 | 0 |
| 4-DoF position | 368 | 150 | 30 | 0 |
| 4-DoF velocity | 55 | 4 | 26 | 0 |

## Hot-path ranking (4-DoF position recompute, FC calls)

- `PolyEval`: 75
- `IntegrateProfileStates`: 56
- `CheckProfile`: 52
- `SolveQuartic`: 39
- `SolveCubic`: 28
- `IsFiniteLreal`: 24
- `SolveDirection`: 8
- `SolveTimedAcc0Acc1Vel`: 8
- `SolveTimedVel`: 8
- `SolveTimedAcc0Vel`: 8
- `SolveTimedAcc1Vel`: 8
- `SolveTimedAcc0Acc1`: 8
- `SolveTimedAcc0`: 8
- `SolveTimedAcc1`: 8
- `SolveTimedNone`: 8
- `ComputeBrakeProfile`: 4
- `ComputeBlock1Axis`: 4
- `ComputeProfile1AxisTimed`: 4
- `StateAtTime`: 4
- `ShrinkInterval`: 3
- `ValidateInput`: 1
- `Synchronize`: 1
- `AdvanceTime`: 1

## Notes
- Steady-state (no retarget) is the common cycle: 0 sqrt, a small fixed
  FC count (per-axis `StateAtTime` + `AdvanceTime` + input validation).
- Recompute (a retarget cycle) is the worst case: per-axis brake + Block +
  `Synchronize` + step2; the root solvers (`PolyEval`/`SolveQuartic`/
  `SolveCubic`/`CheckProfile`) dominate.
- Optimization is deferred to a later, measurement-driven version (this
  report is the 'measure first' step); real µs come from PLCSIM.
