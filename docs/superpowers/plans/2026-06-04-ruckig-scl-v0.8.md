# ruckig-scl v0.8 — Multi-Axis Brake Pre-Phase — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Apply Ruckig's position brake pre-phase per axis in the multi-axis recompute path of `RuckigOtg`, so an out-of-limits initial `(v0, a0)` on any DoF is braked before synchronization.

**Architecture:** Inside the multi-axis **unified per-axis flow** only, per axis: `ComputeBrakeProfile` (existing FC) → `ComputeBlock1Axis` from the post-brake state → fold `brakeDuration` into the Block (total-duration space, per Ruckig `block.hpp`) → `Synchronize` unchanged → re-time the inner profile to `t_sync − brakeDuration`. The single-axis path and the global `SYNC_PHASE` branch are untouched. No new FC; the change is contained in `RuckigOtg.s7dcl`. Validated by parity against the Ruckig oracle.

**Tech Stack:** Siemens SCL (`.s7dcl`) via `siemens-plc-tools` `plc-code`; tests in Python with `pytest` and the Ruckig oracle (PyPI `ruckig` 0.17.3); `uv` for all Python invocation. **Run tests as `uv run python -m pytest …`** (the bare `uv run pytest` fails to spawn in this environment).

**Spec:** `docs/superpowers/specs/2026-06-04-ruckig-scl-v0.8-design.md`

**Oracle reference:** `calculator_target.hpp` (brake per dof, lines ~251-280; `t_profile = traj.duration − brake.duration`, lines ~403/467) and `block.hpp` (`t_min = t_sum + brake.duration`, lines 32-33/48).

---

## Key facts (read before starting)

1. **`ComputeBrakeProfile`** (existing, `src/blocks/ComputeBrakeProfile.s7dcl`, returns `LReal` = brake duration): writes the integrated 2-segment brake prefix into `profile.brake`, and exposes the post-brake state via `profile.brake.p[2]` (a **relative** position advance), `profile.brake.v[2]`, `profile.brake.a[2]`. When the state is already in limits it clears `profile.brake` and returns `0` (with `p[2]=0`, `v[2]=v0`, `a[2]=a0`). **Do NOT bind its `=>` VAR_OUTPUTs** (`pBrake/vBrake/aBrake`) — the transpiler drops `=>` outputs when the return value is consumed (quirk #6); read the post-brake state from `profile.brake.[2]` instead, exactly as the single-axis path does (`RuckigOtg.s7dcl` ~line 369-378).
2. **Brake persists** across `ComputeBlock1Axis`, `Synchronize`, and `ComputeProfile1Axis(Timed)` — none of them touch `profile.brake`. So compute the brake once in the Block loop and read it back in the emit loop.
3. **`block.hpp` folds brake into the Block**: `t_min` and the blocked-interval edges are in **total-duration** space. So after `ComputeBlock1Axis` (which returns inner durations), add `brakeDuration[ax]` to `tMin` and to `aLeft/aRight/bLeft/bRight`. Then `Synchronize` works unchanged, and the inner re-time uses `tf = t_sync − brakeDuration[ax]`.
4. **In-limit axis → `brakeDuration = 0`** → identical to v0.7. The 237 existing tests MUST stay green.
5. **SCL/transpiler constraints** (`docs/PLC_CODE_LIMITATIONS.md`): one `:=` per line; `END_IF`/`END_FOR` on their own line, never inline-commented; no division by a parenthesized product; no identifier starting with `IF` or ending in `of`/`Dof`.
6. The change is **only** in the `IF NOT #syncHandled THEN` unified per-axis flow of the multi-axis path (`RuckigOtg.s7dcl`, currently ~lines 452-606). Do not touch the velocity branch, the single-axis path, or the `SYNC_PHASE` branch.

## File structure

| File | New/Modify | Responsibility |
|---|---|---|
| `src/blocks/RuckigOtg.s7dcl` | Modify | per-axis brake in the multi-axis unified flow + version bump |
| `tests/parity/scenarios/v08_*.yaml` | Create | brake parity scenarios (single + multi axis) |
| `CHANGELOG.md`, `README.md`, `pyproject.toml`, `uv.lock` | Modify | release 0.8.0 |

---

## Task 1: Multi-axis brake in `RuckigOtg` + first multi-axis brake scenario

**Files:**
- Modify: `src/blocks/RuckigOtg.s7dcl`
- Create: `tests/parity/scenarios/v08_04_2dof_brake_limiting.yaml`

TDD: add a multi-axis brake parity scenario (fails today — the multi-axis path
ignores the out-of-limits state), implement the brake, watch it pass.

- [ ] **Step 1: Write the failing parity scenario**

Create `tests/parity/scenarios/v08_04_2dof_brake_limiting.yaml`:

```yaml
name: v0.8 2-DoF time-sync with an out-of-limit initial velocity (braked axis limits)
n_dofs: 2
current_velocity: [5.0, 0.0]
target_position: [10.0, 1.0]
max_velocity: [2.0, 2.0]
max_acceleration: [5.0, 5.0]
max_jerk: [10.0, 10.0]
cycle_time: 0.010
```

- [ ] **Step 2: Run it to confirm it fails**

Run: `uv run python -m pytest "tests/parity/test_parity.py::test_parity_scenario[v08_04_2dof_brake_limiting]" -q`
Expected: FAIL — the SCL multi-axis path does not brake the `v0 = 5 > vMax = 2`
axis, so the trajectory diverges from the oracle (or errors).

- [ ] **Step 3: Add the `brakeDur` scratch array**

In `src/blocks/RuckigOtg.s7dcl`, in the `VAR_TEMP` block, after the line
`mainDuration : LReal;` add:

```scl
        brakeDur     : Array[0..3] of LReal;   // per-axis brake duration (multi-axis path)
```

- [ ] **Step 4: Brake + post-brake Block + fold, in the Block loop**

In the unified per-axis flow, replace the **entire** `// 2. Block every axis …`
region (the `FOR #ax := 0 TO #nd - 1 DO … END_FOR;` that calls
`ComputeBlock1Axis`, currently ~lines 467-488) with:

```scl
                        // 2. Brake each axis (out-of-limit v0/a0), Block from the
                        //    post-brake state, then fold the brake duration into the
                        //    Block so Synchronize works in total-duration space
                        //    (Ruckig block.hpp: t_min = t_sum + brake.duration).
                        FOR #ax := 0 TO #nd - 1 DO
                            // ComputeBrakeProfile writes profile.brake (integrated
                            // prefix) and the post-brake state into brake.p[2]
                            // (relative position advance), brake.v[2], brake.a[2].
                            // It clears the brake and returns 0 when already in limits.
                            // Do NOT bind its => outputs (transpiler quirk #6); read
                            // the post-brake state from profile.brake.[2].
                            #brakeDur[#ax] := "ComputeBrakeProfile"(
                                profile := #trajectory.profiles[#ax],
                                v0 := #chainVel[#ax],
                                a0 := #chainAcc[#ax],
                                vMax := #input.maxVelocity[#ax],
                                aMax := #input.maxAcceleration[#ax],
                                jMax := #input.maxJerk[#ax]);
                            #mainP0 := #chainPos[#ax] + #trajectory.profiles[#ax].brake.p[2];
                            #mainV0 := #trajectory.profiles[#ax].brake.v[2];
                            #mainA0 := #trajectory.profiles[#ax].brake.a[2];

                            #blockResult := "ComputeBlock1Axis"(
                                block := #blockSet.items[#ax],
                                profile := #trajectory.profiles[#ax],
                                p0 := #mainP0, v0 := #mainV0, a0 := #mainA0,
                                pT := #input.targetPosition[#ax],
                                vT := #input.targetVelocity[#ax],
                                aT := #input.targetAcceleration[#ax],
                                vMax := #input.maxVelocity[#ax],
                                aMax := #input.maxAcceleration[#ax],
                                jMax := #input.maxJerk[#ax]);
                            IF #blockResult <> "dbRuckigConst".RESULT_WORKING THEN
                                #valid := false;
                                #busy := false;
                                #done := false;
                                #error := true;
                                #status := "dbRuckigConst".RESULT_ERR_TRAJ;
                                #trajectory.isValid := false;
                                RETURN;
                            END_IF;

                            // fold the brake into the Block (total-duration space)
                            #blockSet.items[#ax].tMin := #blockSet.items[#ax].tMin + #brakeDur[#ax];
                            IF #blockSet.items[#ax].aValid THEN
                                #blockSet.items[#ax].aLeft := #blockSet.items[#ax].aLeft + #brakeDur[#ax];
                                #blockSet.items[#ax].aRight := #blockSet.items[#ax].aRight + #brakeDur[#ax];
                            END_IF;
                            IF #blockSet.items[#ax].bValid THEN
                                #blockSet.items[#ax].bLeft := #blockSet.items[#ax].bLeft + #brakeDur[#ax];
                                #blockSet.items[#ax].bRight := #blockSet.items[#ax].bRight + #brakeDur[#ax];
                            END_IF;
                        END_FOR;
```

(The `Synchronize` call — `// 3. …` region — is unchanged.)

- [ ] **Step 5: Post-brake seed + tf offset + axDur, in the emit loop**

In the `// 4. emit per axis` region:

(a) **Delete** the unconditional 13-line brake zero-fill at the top of the
`FOR #ax := 0 TO #nd - 1 DO` loop (currently ~lines 515-527: the block from
`#trajectory.profiles[#ax].brake.t[0] := 0.0;` through
`#trajectory.profiles[#ax].brake.p[2] := 0.0;`). The brake is now populated by
`ComputeBrakeProfile` in step 2 (and cleared there when not needed). The loop now
starts directly with `#tinMoving := …`.

(b) In the **`ELSIF #tinMoving THEN`** branch, replace the `ComputeProfile1AxisTimed`
call so it seeds from the post-brake state and re-times to `t_sync − brakeDur`:

```scl
                            ELSIF #tinMoving THEN
                                #mainP0 := #chainPos[#ax] + #trajectory.profiles[#ax].brake.p[2];
                                #mainV0 := #trajectory.profiles[#ax].brake.v[2];
                                #mainA0 := #trajectory.profiles[#ax].brake.a[2];
                                #profileResult := "ComputeProfile1AxisTimed"(
                                    profile := #trajectory.profiles[#ax],
                                    p0 := #mainP0, v0 := #mainV0, a0 := #mainA0,
                                    pT := #input.targetPosition[#ax], vT := #input.targetVelocity[#ax],
                                    aT := #input.targetAcceleration[#ax],
                                    vMax := #input.maxVelocity[#ax], aMax := #input.maxAcceleration[#ax],
                                    jMax := #input.maxJerk[#ax], tf := #tSync - #brakeDur[#ax]);
                                IF #profileResult <> "dbRuckigConst".RESULT_WORKING AND #profileResult <> "dbRuckigConst".RESULT_FINISHED THEN
                                    #valid := false;
                                    #busy := false;
                                    #done := false;
                                    #error := true;
                                    #status := "dbRuckigConst".RESULT_ERR_TRAJ;
                                    #trajectory.isValid := false;
                                    RETURN;
                                END_IF;
```

(c) In the **`ELSE`** branch (No / TimeIfNecessary-rest), replace the
`ComputeProfile1Axis` call so it seeds from the post-brake state:

```scl
                            ELSE
                                #mainP0 := #chainPos[#ax] + #trajectory.profiles[#ax].brake.p[2];
                                #mainV0 := #trajectory.profiles[#ax].brake.v[2];
                                #mainA0 := #trajectory.profiles[#ax].brake.a[2];
                                #profileResult := "ComputeProfile1Axis"(profile := #trajectory.profiles[#ax],
                                    p0 := #mainP0, v0 := #mainV0, a0 := #mainA0,
                                    pT := #input.targetPosition[#ax], vT := #input.targetVelocity[#ax],
                                    aT := #input.targetAcceleration[#ax],
                                    vMax := #input.maxVelocity[#ax], aMax := #input.maxAcceleration[#ax],
                                    jMax := #input.maxJerk[#ax]);
                                IF #profileResult <> "dbRuckigConst".RESULT_WORKING AND #profileResult <> "dbRuckigConst".RESULT_FINISHED THEN
                                    #valid := false;
                                    #busy := false;
                                    #done := false;
                                    #error := true;
                                    #status := "dbRuckigConst".RESULT_ERR_TRAJ;
                                    #trajectory.isValid := false;
                                    RETURN;
                                END_IF;
                            END_IF;
```

(d) The `#atTarget` branch is **unchanged** (an at-target static axis is in
limits, so `brakeDur[ax] = 0` and `profile.brake` was cleared by
`ComputeBrakeProfile`).

(e) Replace the `axDur` accumulation so it includes the brake duration. Change:

```scl
                            #axDur := 0.0;
                            FOR #i := 0 TO 6 DO
                                #axDur := #axDur + #trajectory.profiles[#ax].t[#i];
                            END_FOR;
```

to:

```scl
                            #axDur := #brakeDur[#ax];
                            FOR #i := 0 TO 6 DO
                                #axDur := #axDur + #trajectory.profiles[#ax].t[#i];
                            END_FOR;
```

(The `independentMinDurations[ax] = blockSet.items[ax].tMin` line stays — `tMin`
already includes the brake after step 4's fold. The `IF #axDur > #mainDuration`
and `#trajectory.duration := #mainDuration` lines are unchanged.)

- [ ] **Step 6: Bump version**

Change the FB header `S7_Version := "0.7.0"` to `S7_Version := "0.8.0"`.

- [ ] **Step 7: Verify the scenario passes and the suite is green**

Run: `uv run python -m pytest "tests/parity/test_parity.py::test_parity_scenario[v08_04_2dof_brake_limiting]" -q`
Expected: PASS.

Run: `uv run python -m pytest -q`
Expected: all green (237 existing + 1 new). The in-limit existing scenarios are
unaffected (brakeDur=0).

If the new scenario fails parity, dump the assertion's posErr/velErr/durErr and
ref-vs-scl durations; the likely culprits are the post-brake position seed
(`chainPos + brake.p[2]`) or the `t_sync − brakeDur` offset. Do NOT relax
tolerances.

- [ ] **Step 8: Commit**

```bash
git add src/blocks/RuckigOtg.s7dcl tests/parity/scenarios/v08_04_2dof_brake_limiting.yaml
git commit -m "v0.8 T1: multi-axis brake pre-phase in RuckigOtg unified flow"
```

---

## Task 2: Brake parity scenarios (single-axis regression + multi-axis coverage)

**Files:**
- Create: `tests/parity/scenarios/v08_01_1dof_brake_vmax.yaml`
- Create: `tests/parity/scenarios/v08_02_1dof_brake_amax.yaml`
- Create: `tests/parity/scenarios/v08_03_1dof_brake_negative.yaml`
- Create: `tests/parity/scenarios/v08_05_2dof_brake_not_limiting.yaml`
- Create: `tests/parity/scenarios/v08_06_2dof_brake_amax.yaml`
- Create: `tests/parity/scenarios/v08_07_2dof_brake_no_axis.yaml`
- Create: `tests/parity/scenarios/v08_08_2dof_brake_discrete.yaml`

All of these should pass immediately after Task 1 (single-axis ones exercise the
unchanged single-axis path; multi-axis ones exercise the new brake path).

- [ ] **Step 1: Create the single-axis regression scenarios**

`tests/parity/scenarios/v08_01_1dof_brake_vmax.yaml`:

```yaml
name: v0.8 1-DoF brake from v0 > vMax
n_dofs: 1
current_velocity: [5.0]
target_position: [10.0]
max_velocity: [2.0]
max_acceleration: [5.0]
max_jerk: [10.0]
cycle_time: 0.010
```

`tests/parity/scenarios/v08_02_1dof_brake_amax.yaml`:

```yaml
name: v0.8 1-DoF brake from a0 > aMax
n_dofs: 1
current_acceleration: [12.0]
target_position: [10.0]
max_velocity: [2.0]
max_acceleration: [5.0]
max_jerk: [10.0]
cycle_time: 0.010
```

`tests/parity/scenarios/v08_03_1dof_brake_negative.yaml`:

```yaml
name: v0.8 1-DoF brake from v0 < -vMax, move toward negative
n_dofs: 1
current_velocity: [-5.0]
target_position: [-3.0]
max_velocity: [2.0]
max_acceleration: [5.0]
max_jerk: [10.0]
cycle_time: 0.010
```

- [ ] **Step 2: Create the multi-axis coverage scenarios**

`tests/parity/scenarios/v08_05_2dof_brake_not_limiting.yaml` (braked axis is NOT
the limiting one — exercises the `t_sync − brakeDur` inner re-time offset):

```yaml
name: v0.8 2-DoF brake where the braked axis is not limiting
n_dofs: 2
current_velocity: [3.0, 0.0]
target_position: [1.0, 12.0]
max_velocity: [2.0, 2.0]
max_acceleration: [5.0, 5.0]
max_jerk: [10.0, 10.0]
cycle_time: 0.010
```

`tests/parity/scenarios/v08_06_2dof_brake_amax.yaml`:

```yaml
name: v0.8 2-DoF brake from a0 > aMax on one axis
n_dofs: 2
current_acceleration: [12.0, 0.0]
target_position: [10.0, 4.0]
max_velocity: [2.0, 2.0]
max_acceleration: [5.0, 5.0]
max_jerk: [10.0, 10.0]
cycle_time: 0.010
```

`tests/parity/scenarios/v08_07_2dof_brake_no_axis.yaml` (per-DoF `[No, Time]` with
the `No` axis out-of-limits — verifies the brake enters the `t_sync` floor):

```yaml
name: v0.8 2-DoF per-DoF [No, Time] with the No axis out-of-limit
per_dof_synchronization: [0, 2]
n_dofs: 2
current_velocity: [5.0, 0.0]
target_position: [10.0, 1.0]
max_velocity: [2.0, 2.0]
max_acceleration: [5.0, 5.0]
max_jerk: [10.0, 10.0]
cycle_time: 0.010
```

`tests/parity/scenarios/v08_08_2dof_brake_discrete.yaml`:

```yaml
name: v0.8 2-DoF brake with discrete duration
duration_discretization: 1
n_dofs: 2
current_velocity: [5.0, 0.0]
target_position: [10.0, 1.0]
max_velocity: [2.0, 2.0]
max_acceleration: [5.0, 5.0]
max_jerk: [10.0, 10.0]
cycle_time: 0.010
```

- [ ] **Step 3: Run all v08 scenarios**

Run: `uv run python -m pytest tests/parity -q -k v08`
Expected: 8 passed (v08_01..v08_08).

If any multi-axis scenario fails, report the diagnostics (posErr/velErr/durErr,
ref-vs-scl durations) — do NOT relax tolerances or delete the scenario.

- [ ] **Step 4: Full suite**

Run: `uv run python -m pytest -q`
Expected: all green (237 + 8 = 245).

- [ ] **Step 5: Commit**

```bash
git add tests/parity/scenarios/v08_01_1dof_brake_vmax.yaml tests/parity/scenarios/v08_02_1dof_brake_amax.yaml tests/parity/scenarios/v08_03_1dof_brake_negative.yaml tests/parity/scenarios/v08_05_2dof_brake_not_limiting.yaml tests/parity/scenarios/v08_06_2dof_brake_amax.yaml tests/parity/scenarios/v08_07_2dof_brake_no_axis.yaml tests/parity/scenarios/v08_08_2dof_brake_discrete.yaml
git commit -m "v0.8 T2: brake parity scenarios (single-axis regression + multi-axis coverage)"
```

---

## Task 3: Release 0.8.0

**Files:**
- Modify: `CHANGELOG.md`, `README.md`, `pyproject.toml`, `uv.lock`

- [ ] **Step 1: CHANGELOG**

Add a top entry to `CHANGELOG.md` (match the existing style):

```markdown
## [0.8.0] - 2026-06-04

### Added
- **Multi-axis brake pre-phase.** The multi-axis recompute path now brakes each
  axis whose initial `(v0, a0)` is out of limits (`|v0| > vMax` or `|a0| > aMax`)
  before synchronization, completing the brake story (the single-axis path
  already braked). Per axis: `ComputeBrakeProfile` -> Block from the post-brake
  state -> brake duration folded into the Block (total-duration space, per Ruckig
  `block.hpp`) -> `Synchronize` unchanged -> inner profile re-timed to
  `t_sync - brakeDuration`. Works across Time / No / per-DoF / TimeIfNecessary /
  Discrete.
- 8 brake parity scenarios (`v08_01`..`v08_08`): single-axis (regression guard)
  and multi-axis (braked-axis-limiting / not-limiting / a0>aMax / No-axis /
  discrete).

### Known limitations
- **Phase + out-of-limits initial state falls back to Time** (which brakes); it
  does not reproduce Ruckig's Phase-on-post-brake trajectory. Phase with an
  in-limits initial state is unchanged.

### Notes
- Velocity-interface brake is not needed (the v0.7 velocity solver absorbs an
  arbitrary `a0`). step1 `time_*_two_step` fallbacks remain deferred.
```

- [ ] **Step 2: README**

- Update the **Status:** line to v0.8.0 (multi-axis brake).
- Add a `## Features (v0.8)` section above `## Features (v0.7)` summarizing the
  multi-axis brake.
- In the **Known limitations** heading bump to v0.8: replace the
  "No brake pre-phase in the multi-axis path" bullet (now resolved) with the
  Phase+out-of-limits limitation; keep the step1 two-step bullet.
- In the **Roadmap** table mark v0.8 as *(this release)*; the next milestone is
  step1 two-step fallbacks / degenerate-state completeness.
- In the **Tests** section update the parity-scenario count: run
  `ls tests/parity/scenarios/*.yaml | wc -l` and use that number.

- [ ] **Step 3: Version bump**

In `pyproject.toml` change `version = "0.7.0"` to `version = "0.8.0"`, then run
`uv lock`.

- [ ] **Step 4: Final full suite**

Run: `uv run python -m pytest -q`
Expected: all green.

- [ ] **Step 5: Commit**

```bash
git add CHANGELOG.md README.md pyproject.toml uv.lock
git commit -m "v0.8 T3: release 0.8.0 (docs + version)"
```

---

## Final integration (handled by the execution skill)

After Task 3, hand off to `superpowers:finishing-a-development-branch`: merge the
feature branch into `master` with `--no-ff`, tag `v0.8.0`, push master + tag.
A final opus review of the brake integration should run before merge — focus on
the post-brake position seed (`chainPos + brake.p[2]`), the block fold, the
`t_sync − brakeDur` offset, the No-axis floor, and that in-limit calls are
byte-identical to v0.7 (no regression).

---

## Self-review (planner)

- **Spec coverage:** multi-axis brake integration (T1), the brake-in-block fold +
  re-time offset (T1 steps 4-5), all sync modes via reuse (T1 + T2 scenarios
  v08_05/07/08), single-axis regression guard (T2 v08_01..03), Phase limitation
  (documented in T3 CHANGELOG/README), release (T3). All spec sections map to a
  task.
- **No placeholders:** every step shows the exact SCL/YAML to write.
- **Consistency:** `brakeDur[ax]` (added T1 step 3) is used in T1 steps 4-5; the
  post-brake seed `chainPos + brake.p[2]` / `brake.v[2]` / `brake.a[2]` is used
  identically in the Block loop and both emit branches; `tf = t_sync − brakeDur`
  matches the spec's `t_profile = t_sync − brake.duration`.
- **Regression safety:** in-limit axis → brakeDur=0, brake cleared by
  `ComputeBrakeProfile` → behaviour identical to v0.7; full-suite gate in T1
  step 7 and T2 step 4.
```
