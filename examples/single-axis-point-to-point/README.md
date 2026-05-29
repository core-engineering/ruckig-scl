# Example — single-axis point-to-point

[`example.scl`](example.scl) shows the minimal wiring to move one axis from its
current position to a commanded target with a jerk-limited profile.

## What it does

1. **Configures the limits once** (`maxVelocity`, `maxAcceleration`, `maxJerk`)
   and enables axis 0.
2. **Each cycle**, feeds the current encoder position and the commanded target
   into `typeRuckigInput`.
3. **Calls `RuckigOtg` once per cycle** with `cycleTime` equal to the OB period
   (10 ms here).
4. **Forwards `output.newPosition[0]`** to the drive as the position setpoint,
   and exposes `done` / `error`.

## Integrating it

- Call the block from a cyclic interrupt OB (e.g. a fast-motion OB) whose period
  matches `cycleTime`.
- Replace `"diAxis"` / `"qiAxis"` with your own I/O or DB tags
  (`encoderPosition`, `commandedTarget`, `motionEnable` in; `positionSetpoint`,
  `motionDone`, `motionError` out).
- Keep `enable` TRUE while motion is wanted; setting it FALSE resets the
  generator so the next rising edge recomputes from the current state.

## Scope

v0.1 is **single-axis, rest-to-rest** (the target is reached with zero velocity
and acceleration). Changing `targetPosition` mid-motion triggers a clean
recompute from the current commanded state on the next cycle.
