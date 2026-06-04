# Ruckig SCL v0.7 — Velocity Control Interface — Design Spec

**Date** : 2026-06-04
**Owner** : Camille Martin
**Project** : Port of Ruckig (C++ MIT) to Structured Text for Siemens S7-1500
**Status** : Design approved, ready for implementation planning
**Builds on** : v0.6.0 (per-DoF sync, TimeIfNecessary, Discrete)

---

## 1. Context and Motivation

Through v0.6, ruckig-scl implements the **position control interface** only:
each axis is driven from `(p0, v0, a0)` to a target `(pT, vT, aT)`. Ruckig also
exposes a **velocity control interface** (`ControlInterface.Velocity`), where the
**position is left uncontrolled** and the trajectory instead drives the
*velocity* state from `(v0, a0)` to a target `(vf, af)`, jerk-limited, under the
acceleration and jerk limits.

This is the natural mode for use cases where the goal is a speed, not a position:
conveyor / spindle / pump speed ramps, jog/velocity-following modes, and the
velocity-tracking leg of a higher-level controller. It is part of the Ruckig
Community feature set and is required for full parity.

`controlInterface` already exists in `typeRuckigInput` (default `0` =
`IFACE_POSITION`) and the constants `IFACE_POSITION`/`IFACE_VELOCITY` already
exist in `dbRuckigConst`, but **no block reads them today** — every call runs the
position solver. v0.7 wires the velocity interface in.

### Scope decision (settled during brainstorming)

- **Full integration through the existing multi-axis flow.** The velocity
  interface is wired through the same recompute path, so single-axis **and** all
  synchronization modes the synchronizer already provides
  (Time / No / per-DoF / TimeIfNecessary / Discrete) work for velocity with no
  extra synchronization work. `Phase` falls back to `Time` (exactly as a per-DoF
  `Phase` entry already does).
- **Third-order only.** Ruckig also has a *second-order* velocity interface
  (jerk unlimited, acceleration-controlled). The whole port is third-order /
  jerk-limited; the second-order interface is **out of scope** (consistent with
  the position interface never porting second-order).

---

## 2. Goals and Non-Goals

### Goals

- Functional parity with Ruckig's **third-order velocity interface** for 1–4 DoF.
- Wire `controlInterface = IFACE_VELOCITY` through `RuckigOtg`, reusing the
  existing synchronization, integration and evaluation machinery unchanged.
- New dedicated, independently-testable FCs for the velocity solver (Approach A).
- Numerical parity against the Ruckig oracle (PyPI `ruckig` 0.17.3): static
  single-DoF to the floating-point floor (~1e-15), full cyclic trajectories to
  1e-6.

### Non-Goals

- **Second-order velocity interface** (jerk unlimited) — out of scope.
- **Brake pre-phase in the velocity path** — the velocity Step 1 solver natively
  absorbs an arbitrary (even out-of-`aMax`) `a0`, so no separate brake machinery
  is needed.
- **Position targeting in velocity mode** — `targetPosition` is ignored by design
  (Ruckig parity). Position is still integrated and output, just not constrained.
- **`maxVelocity` as a constraint in velocity mode** — ignored (Ruckig parity);
  the trajectory may exceed it to reach `vf`.

---

## 3. Architecture

### 3.1 Approach A — dedicated velocity FCs

Three new pure FCs, parallel to the position FCs, reusing the existing UDTs
(`typeProfile`, `typeBlock`, `typeTrajectory`):

| FC | Role | Oracle reference |
|---|---|---|
| `ComputeVelBlock1Axis` | Step 1 (velocity): time-optimal extremal profiles → `typeBlock` (`tMin` + blocked intervals) for one axis | `VelocityThirdOrderStep1::get_profile` (`velocity_third_step1.cpp`) |
| `ComputeVelProfileTimed` | Step 2 (velocity): re-time one axis to an imposed duration `tf` | `VelocityThirdOrderStep2::get_profile` (`velocity_third_step2.cpp`) |
| `CheckVelProfile` | Validate a candidate velocity profile (reaches `vf`/`af`, `|a| ≤ aMax`, `t ≥ 0`; **position unconstrained**) | `Profile::check_for_velocity` (`profile.hpp`) |

**Reused unchanged** (no edits): `Synchronize`, `IntegrateProfileStates`,
`AdvanceTime`, `StateAtTime`, `typeBlock`, `typeProfile`, `typeTrajectory`,
`SolveCubic`/`SolveQuartic`/`PolyEval`/`ShrinkInterval` (not needed by the
velocity solver — it is fully closed-form).

### 3.2 RuckigOtg integration

`RuckigOtg` branches on `input.controlInterface` at exactly **three points**, in
both the single-axis and multi-axis paths:

1. **Per-axis Block computation** — call `ComputeVelBlock1Axis` instead of
   `ComputeBlock1Axis` when `controlInterface = IFACE_VELOCITY`.
2. **Per-axis re-time** — call `ComputeVelProfileTimed` instead of
   `ComputeProfile1AxisTimed`.
3. **"Already at target" guard** — use the velocity criterion
   (`ABS(vf - v0) < EPS_VELOCITY AND ABS(af - a0) < EPS_DERIV`) instead of the
   position-based criterion.

Everything else in the recompute flow is unchanged: `modeEff` resolution per
axis, the global `SYNC_PHASE` branch (Phase → Time fallback when the per-DoF or
global mode is Phase), `Synchronize(participates, discrete, cycleTime)`, per-axis
emission (moving target → Step 2 at `tSync`, else Step 1), and
`trajectory.duration = max(axDur)`. `independentMinDurations` is filled exactly as
today (block `tMin` for participating axes, own duration for `No`).

> The branch is implemented as explicit `IF`/`ELSE` at the (few) call sites in
> `RuckigOtg`, not via an interface flag threaded into the position FCs — keeping
> the two solvers cleanly separated (Approach A).

### 3.3 ValidateInput

A new branch on `controlInterface`:

- **`IFACE_VELOCITY`**: for each active DoF require `maxAcceleration > 0` and
  `maxJerk > 0`; **do not** require `maxVelocity > 0` (unused in velocity mode).
  Finiteness checks apply to `currentVelocity`, `currentAcceleration`,
  `targetVelocity`, `targetAcceleration`, and `currentPosition` (the integration
  origin); `targetPosition` is **not** required finite (ignored).
- **`IFACE_POSITION` (0)**: unchanged from v0.6.
- **`controlInterface` outside {0, 1}** → new status `RESULT_ERR_IFACE`.

### 3.4 Constants

`IFACE_POSITION` / `IFACE_VELOCITY` already exist. Add one status code to
`dbRuckigConst` (DA014 parameterization range `16#82xx`):

```scl
RESULT_ERR_IFACE : Word := 16#8208;   // controlInterface not in {IFACE_POSITION, IFACE_VELOCITY}
```

No UDT changes are required.

### 3.5 Change detection

`controlInterface` is added to the `prevInput` change-detection comparison, so
switching position ↔ velocity at runtime triggers a recompute. As with every
recompute, the new trajectory starts from the current `(p, v, a)` at `t = 0`, so
the switch is C2-continuous (no setpoint jump).

---

## 4. Velocity solver algorithm

Per axis, with symmetric limits (`aMin = -maxAcceleration`, `aMax =
+maxAcceleration`), `jMax = maxJerk`. The profile uses segments `t[0..2]` (jerk
up → const-accel plateau → jerk down); `t[3..6] = 0` except the zero-jerk
special case. Everything is closed-form — **no cubic/quartic root finder**.

### 4.1 Step 1 — `ComputeVelBlock1Axis`

`vd = vf - v0`. Two families:

- **`time_acc0`** (reaches the `aMax`/`aMin` plateau, `ReachedLimits::ACC0`):
  ```
  t[0] = (-a0 + aMax)/jMax
  t[1] = (a0² + af²)/(2·aMax·jMax) - aMax/jMax + vd/aMax
  t[2] = (-af + aMax)/jMax
  ```
- **`time_none`** (no plateau, `ReachedLimits::NONE`): with
  `h1 = sqrt((a0² + af²)/2 + jMax·vd)` (only if the radicand ≥ 0), two solutions
  `t[0] = -(a0 ± h1)/jMax`, `t[2] = -(af ± h1)/jMax`.

Dispatch (mirrors the oracle):
- **`jMax = 0`** (zero-jerk special case): `time_all_single_step` — valid only if
  `af = a0`; if `|a0| > eps` then `t[3] = vd/a0`, else requires `|vd| < eps`.
- **`af ≈ 0`**: no blocked interval exists → return after the first valid profile;
  try `time_none` then `time_acc0` in the sign-of-`vd` direction, then the
  opposite direction.
- **`af ≠ 0`**: collect valid profiles from all four variants (`time_none` /
  `time_acc0` × both directions) → assemble the `Block` (`tMin` + up to one
  blocked interval) the same way the position Block is assembled.

A candidate is accepted by `CheckVelProfile`.

### 4.2 Step 2 — `ComputeVelProfileTimed`

Re-time to an imposed `tf` (from `Synchronize`). `vd = vf - v0`, `ad = af - a0`.

- **`time_acc0`** — three candidate solutions tried in order (UD with
  `h1 = sqrt((-ad² + 2·jMax·((a0+af)·tf - 2·vd))/jMax² + tf²)`; UU; UU-2-step),
  each validated by `CheckVelProfile` with timing.
- **`time_none`** — trivial case `a0 = af = vd = 0` (constant phase `t[1] = tf`),
  plus the UD solution `t[0] = 2·(af·tf - vd)/ad` with a recomputed jerk
  `jf = ad²/(2·(af·tf - vd))`, accepted only if `|jf| ≤ jMax + eps`.

Direction is guessed from the sign of `vd` (try the matching direction first).

### 4.3 Validity — `CheckVelProfile`

Integrate `t/j` from the boundary state (reuse `IntegrateProfileStates`), then
check: every `t_i ≥ 0`; `|a_i| ≤ aMax + eps` at the segment boundaries;
`v_final ≈ vf`; `a_final ≈ af`. **Position is not checked** — it is integrated
only to populate the output. This is the velocity analogue of `CheckProfile`
(which additionally pins the position target).

---

## 5. Error handling

Reuses the existing DA014 scheme. New: `RESULT_ERR_IFACE := 16#8208`
(parameterization range) when `controlInterface` is out of range. Velocity-solver
failure to find any feasible profile returns `RESULT_ERR_SOLVER` (`16#8602`), same
as the position path. On `16#82xx` the output is preserved; on `16#86xx` the
trajectory is invalidated. LREAL comparisons use the existing `EPS_*` tolerances.

---

## 6. Testing strategy

The blocks run through the `siemens-plc-tools` `plc-code` transpiler (SCL →
Python), as for every prior version.

### 6.1 Unit tests

- **`test_compute_vel_block_1axis.py`** — both families; zero-jerk special case;
  `af = 0` vs `af ≠ 0`; both signs of `vd`; `a0 ≠ 0`; presence of a blocked
  interval; `tMin` value vs the oracle.
- **`test_compute_vel_profile_timed.py`** — the three `time_acc0` sub-solutions
  and the `time_none` case; re-time to a `tf > tMin`.
- **`test_check_vel_profile.py`** — accepts a valid profile; rejects negative
  segment time, over-`aMax`, and wrong final `vf`/`af`.
- **`test_validate_input.py`** (extended) — velocity mode accepts `vMax = 0`,
  requires `aMax > 0` / `jMax > 0`, ignores `targetPosition` finiteness; invalid
  `controlInterface` → `RESULT_ERR_IFACE`.

### 6.2 Parity scenarios

New `v07_xx` YAML scenarios with `control_interface: velocity`:

- Single-axis: `a0 = 0, vf > 0`; `a0 ≠ 0`; `af ≠ 0`; `vd < 0`.
- Multi-axis: Time (axes reach different `vf` together), No, per-DoF,
  TimeIfNecessary, Discrete.

Runner changes:
- `runner_ref.py` — set `inp.control_interface = ruckig.ControlInterface.Velocity`
  when the scenario requests it; map the field from the scenario.
- `runner_scl.py` — set `input.controlInterface = 1`.
- `comparator.py` — unchanged. Position still matches cycle-by-cycle because both
  implementations integrate the same velocity profile from the same
  `currentPosition`.

### 6.3 Target

Floating-point floor (~1e-15) on the static single-DoF solver; 1e-6 on full
cyclic trajectories. All existing tests (209) must stay green.

---

## 7. Out of scope (explicitly)

- Second-order (jerk-unlimited) velocity interface.
- Velocity-mode brake pre-phase (the Step 1 solver absorbs arbitrary `a0`).
- Position targeting or `maxVelocity` enforcement in velocity mode.
- Acceleration control interface (not part of Ruckig Community).

---

## 8. Success criteria

- `controlInterface = IFACE_VELOCITY` produces trajectories matching the Ruckig
  oracle to the stated tolerances, single-axis and across all reused sync modes.
- The three new FCs are independently unit-tested.
- `ValidateInput` enforces the velocity-mode rules and rejects bad interfaces.
- The full suite stays green; new `v07_xx` parity scenarios pass at the floating
  point floor.
- README, CHANGELOG and roadmap updated; version bumped to `0.7.0`; merged and
  tagged `v0.7.0`.
