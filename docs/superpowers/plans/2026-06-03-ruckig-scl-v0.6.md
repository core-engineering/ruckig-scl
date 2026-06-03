# Ruckig SCL v0.6 — Synchronization Completeness — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add per-DoF synchronization, `TimeIfNecessary`, and discrete duration to `RuckigOtg`, by generalizing the non-Phase multi-axis dispatch to a per-axis effective mode.

**Architecture:** `RuckigOtg` keeps the global-`SYNC_PHASE` branch (v0.5 `PhaseSynchronize`, unchanged) and **replaces** the global `No`/`Time` branches with one unified per-axis flow: resolve each axis's mode (from `perDofSynchronization` else global), Block the participating (non-`No`) axes, `Synchronize` over a participation mask (+ optional discrete rounding), then emit per axis (`No`/`TimeIfNecessary`-rest → step1; `Time`/`TimeIfNecessary`-moving → step2(t_sync)). Phase stays global; a Phase per-DoF entry is treated as Time.

**Tech Stack:** Siemens SCL (`.s7dcl`), tested via `siemens-plc-tools` `plc-code` transpiler; parity vs PyPI `ruckig` 0.17.3. Tests: `uv run pytest` (ALWAYS `uv`, never `pip`).

**Branch:** `feature/v0.6-sync-completeness` (spec: `docs/superpowers/specs/2026-06-03-ruckig-scl-v0.6-design.md`). Current suite: **195 tests, 31 parity scenarios**.

**Transpiler quirks:** one `:=` per line; `END_IF`/`END_FOR` own line; no identifier ending in `of`/`Dof` (use `ax`); no `/(a*b)` (precompute scalar denom); array-of-scalar OK as FC param, array-of-UDT must be wrapped in a STRUCT; `=>` VAR_OUTPUT works on a `: Void` FC. No `CEIL`/`FLOOR`/`TRUNC` builtins — use `LREAL_TO_DINT` (truncates toward zero) + `DINT_TO_LREAL` for ceil-to-grid.

---

## File structure

| File | Change | Responsibility |
|------|--------|----------------|
| `src/data-blocks/dbRuckigConst.s7dcl` | Modify | + `SYNC_TIME_IF_NECESSARY := 4` |
| `src/data-types/typeRuckigInput.s7dcl` | Modify | + `perDofSynchronization : Array[0..3] of Int` (default -1) |
| `src/blocks/Synchronize.s7dcl` | Modify | + participation mask, + discrete rounding |
| `src/blocks/RuckigOtg.s7dcl` | Modify | unified per-axis flow (replaces global No/Time branches; keeps Phase) |
| `tests/unit/test_synchronize.py` | Modify | pass new params; + mask + discrete tests |
| `tests/unit/test_ruckig_otg.py` | Modify | per_dof / TimeIfNecessary / Discrete FB tests; helper default |
| `tests/unit/test_validate_input.py` | Modify | helper default |
| `tests/parity/runner_ref.py`, `runner_scl.py`, `test_parity.py` | Modify | thread perDof + durationDiscretization |
| `tests/parity/scenarios/v06_*.yaml` | Create | per_dof / TimeIfNecessary / Discrete scenarios |
| `CHANGELOG.md`, `README.md`, `pyproject.toml` | Modify | v0.6.0 release |

---

## Task 1: Constants, input field, change detection

**Files:** `src/data-blocks/dbRuckigConst.s7dcl`, `src/data-types/typeRuckigInput.s7dcl`, `src/blocks/RuckigOtg.s7dcl`, `tests/unit/test_ruckig_otg.py`, `tests/unit/test_validate_input.py`, `tests/parity/runner_scl.py`

- [ ] **Step 1: Add the TimeIfNecessary constant**

In `src/data-blocks/dbRuckigConst.s7dcl`, after `SYNC_PER_DOF : Int := 3;`:
```
        SYNC_TIME_IF_NECESSARY : Int := 4;
```

- [ ] **Step 2: Add the per-DoF input field**

In `src/data-types/typeRuckigInput.s7dcl`, after the `synchronization` field:
```
        perDofSynchronization  : Array[0..3] of Int := -1, -1, -1, -1;   // -1 = use global
```
(If the transpiler/parser rejects that array-default syntax, fall back to `perDofSynchronization : Array[0..3] of Int;` and rely on tests/callers setting it; the FB treats any value `< 0` as "use global".)

- [ ] **Step 3: Change detection on the new knobs**

In `src/blocks/RuckigOtg.s7dcl`, in the `REGION Change detection` block inside the `IF NOT #recomputeRequired THEN` guard, after the existing `synchronization` check, add (one statement per line):
```
                IF #input.durationDiscretization <> #prevInput.durationDiscretization THEN
                    #recomputeRequired := true;
                END_IF;
                FOR #ax := 0 TO #nd - 1 DO
                    IF #input.perDofSynchronization[#ax] <> #prevInput.perDofSynchronization[#ax] THEN
                        #recomputeRequired := true;
                    END_IF;
                END_FOR;
```
(`#ax` and `#nd` are existing VAR_TEMP.)

- [ ] **Step 4: Test helpers set the new field**

In `tests/unit/test_ruckig_otg.py` `default_input()`, after the `"synchronization": 2,` line add:
```python
        "perDofSynchronization": [-1, -1, -1, -1],
        "durationDiscretization": 0,
```
(Remove any now-duplicate `"durationDiscretization"` key.) Do the same in `tests/unit/test_validate_input.py` base dict and in `tests/parity/runner_scl.py`'s input dict (`"perDofSynchronization": [-1, -1, -1, -1]`, keep `"durationDiscretization": 0`).

- [ ] **Step 5: Run full suite**

Run: `uv run pytest -q`
Expected: `195 passed` (no behaviour change — the FB does not read the new field yet).

- [ ] **Step 6: Commit**
```bash
git add src/data-blocks/dbRuckigConst.s7dcl src/data-types/typeRuckigInput.s7dcl src/blocks/RuckigOtg.s7dcl tests/unit/test_ruckig_otg.py tests/unit/test_validate_input.py tests/parity/runner_scl.py
git commit -m "v0.6 T1: SYNC_TIME_IF_NECESSARY + perDofSynchronization field + change detection"
```

---

## Task 2: `Synchronize` — participation mask + discrete rounding

**Files:** `src/blocks/Synchronize.s7dcl`, `tests/unit/test_synchronize.py`, `src/blocks/RuckigOtg.s7dcl` (existing call site)

- [ ] **Step 1: Write the failing unit tests**

In `tests/unit/test_synchronize.py`, update `_run` to pass the new params and add tests. Replace the `_run` helper with:
```python
def _run(harness, blocks, n_dofs, minimum_duration=-1.0,
         participates=None, discrete=False, cycle_time=0.010):
    padded = list(blocks) + [_blk(0.0)] * (4 - len(blocks))
    if participates is None:
        participates = [True] * 4
    else:
        participates = list(participates) + [False] * (4 - len(participates))
    harness.reset()
    harness.set_inputs(blockSet={"items": padded}, nDofs=n_dofs,
                       minimumDuration=minimum_duration,
                       participates=participates, discrete=discrete, cycleTime=cycle_time)
    harness.execute()
    return (harness.get_output("resolved"),
            harness.get_output("tSync"),
            harness.get_output("limitingAxis"))
```
Add two tests:
```python
def test_participation_mask_excludes_no_axis(harness):
    # DoF0 t_min=3.0 but NOT participating (No); DoF1 t_min=1.0 participating.
    # t_sync must ignore DoF0 -> 1.0, limiting = DoF1.
    ok, tsync, lim = _run(harness, [_blk(3.0), _blk(1.0)], 2,
                          participates=[False, True])
    assert ok is True
    assert tsync == pytest.approx(1.0)
    assert lim == 1


def test_discrete_rounds_up_to_cycle_grid(harness):
    # t_sync = max(tMin) = 2.0; discrete dt=0.3 -> ceil(2.0/0.3)*0.3 = 2.1
    ok, tsync, lim = _run(harness, [_blk(2.0), _blk(1.0)], 2,
                          discrete=True, cycle_time=0.3)
    assert ok is True
    assert tsync == pytest.approx(2.1, abs=1e-9)
```

- [ ] **Step 2: Run to verify failure**

Run: `uv run pytest tests/unit/test_synchronize.py -q`
Expected: FAIL (harness has no `participates`/`discrete`/`cycleTime` inputs yet).

- [ ] **Step 3: Rewrite `Synchronize.s7dcl`**

Replace the whole file with:
```
{
    S7_Author := "Martin C";
    S7_EditorMode := "SCL";
    S7_Family := "Ruckig";
    S7_Optimized := "TRUE";
    S7_Version := "0.6.0"
}
FUNCTION "Synchronize" : Void
    VAR_IN_OUT
        blockSet : _.typeBlockSet;
    END_VAR
    VAR_INPUT
        nDofs           : Int;
        minimumDuration : LReal;   // sentinel < 0 = none
        participates    : Array[0..3] of Bool;   // axes excluded from t_sync (No)
        discrete        : Bool;    // round t_sync up to a cycleTime grid
        cycleTime       : LReal;
    END_VAR
    VAR_OUTPUT
        resolved     : Bool;
        tSync        : LReal;
        limitingAxis : Int;
    END_VAR
    VAR_TEMP
        tStart   : LReal;
        cand     : Array[0..12] of LReal;
        nc       : Int;
        d        : Int;
        i        : Int;
        k        : Int;
        tmp      : LReal;
        c        : LReal;
        blocked  : Bool;
        anyPart  : Bool;
        q        : LReal;
        kFloor   : DInt;
        tFloor   : LReal;
        rEdge    : LReal;
        g        : Int;
        stepDone : Bool;
    END_VAR

    { S7_Language := "SCL" }
    NETWORK
        REGION Block header
            //==========================================================
            // Title:      Synchronize
            // Function:   Time synchronization across the PARTICIPATING DoFs
            //             (participates[d]=false excludes a No axis). Finds
            //             t_sync = smallest common non-blocked duration >=
            //             max(tMin of participating) + the limiting DoF. When
            //             discrete, rounds t_sync up to a cycleTime grid and
            //             jumps any blocked interval the rounding lands in.
            //             Port of Ruckig TargetCalculator::synchronize.
            // Returns:    resolved = TRUE if a t_sync was found (VAR_OUTPUT).
            // Family:     Ruckig
            // Author:     Martin C
            //==========================================================
        END_REGION

        REGION Lower bound over participating DoFs + limiting DoF
            #tStart := 0.0;
            IF #minimumDuration >= 0.0 THEN
                #tStart := #minimumDuration;
            END_IF;
            #anyPart := false;
            FOR #d := 0 TO #nDofs - 1 DO
                IF #participates[#d] THEN
                    #anyPart := true;
                    IF #blockSet.items[#d].tMin > #tStart THEN
                        #tStart := #blockSet.items[#d].tMin;
                    END_IF;
                END_IF;
            END_FOR;
            #limitingAxis := 0;
            #tmp := -1.0;
            FOR #d := 0 TO #nDofs - 1 DO
                IF #participates[#d] AND #blockSet.items[#d].tMin > #tmp THEN
                    #tmp := #blockSet.items[#d].tMin;
                    #limitingAxis := #d;
                END_IF;
            END_FOR;
        END_REGION

        REGION No participating DoF -> trivial resolve
            IF NOT #anyPart THEN
                #tSync := #tStart;
                #resolved := true;
                RETURN;
            END_IF;
        END_REGION

        REGION Collect candidate sync times (>= tStart), participating only
            #nc := 0;
            #cand[#nc] := #tStart;
            #nc := #nc + 1;
            FOR #d := 0 TO #nDofs - 1 DO
                IF #participates[#d] THEN
                    IF #blockSet.items[#d].aValid AND #blockSet.items[#d].aRight > #tStart THEN
                        #cand[#nc] := #blockSet.items[#d].aRight;
                        #nc := #nc + 1;
                    END_IF;
                    IF #blockSet.items[#d].bValid AND #blockSet.items[#d].bRight > #tStart THEN
                        #cand[#nc] := #blockSet.items[#d].bRight;
                        #nc := #nc + 1;
                    END_IF;
                END_IF;
            END_FOR;
        END_REGION

        REGION Sort candidates ascending
            FOR #i := 0 TO #nc - 2 DO
                FOR #k := 0 TO #nc - 2 - #i DO
                    IF #cand[#k] > #cand[#k + 1] THEN
                        #tmp := #cand[#k];
                        #cand[#k] := #cand[#k + 1];
                        #cand[#k + 1] := #tmp;
                    END_IF;
                END_FOR;
            END_FOR;
        END_REGION

        REGION Pick smallest non-blocked candidate (participating only)
            #resolved := false;
            #tSync := #tStart;
            FOR #i := 0 TO #nc - 1 DO
                IF NOT #resolved THEN
                    #c := #cand[#i];
                    #blocked := false;
                    FOR #d := 0 TO #nDofs - 1 DO
                        IF #participates[#d] THEN
                            IF #blockSet.items[#d].aValid AND #blockSet.items[#d].aLeft < #c AND #c < #blockSet.items[#d].aRight THEN
                                #blocked := true;
                            END_IF;
                            IF #blockSet.items[#d].bValid AND #blockSet.items[#d].bLeft < #c AND #c < #blockSet.items[#d].bRight THEN
                                #blocked := true;
                            END_IF;
                        END_IF;
                    END_FOR;
                    IF NOT #blocked THEN
                        #tSync := #c;
                        #resolved := true;
                    END_IF;
                END_IF;
            END_FOR;
        END_REGION

        REGION Discrete: round t_sync up to the cycleTime grid, jump blocked intervals
            IF #discrete AND #resolved AND #cycleTime > 0.0 THEN
                #q := #tSync / #cycleTime;
                #kFloor := LREAL_TO_DINT(#q);
                #tFloor := DINT_TO_LREAL(#kFloor) * #cycleTime;
                IF #tFloor < #tSync - 2.0E-12 THEN
                    #tSync := #tFloor + #cycleTime;
                ELSE
                    #tSync := #tFloor;
                END_IF;
                #stepDone := false;
                FOR #g := 0 TO 8 DO
                    IF NOT #stepDone THEN
                        #blocked := false;
                        #rEdge := #tSync;
                        FOR #d := 0 TO #nDofs - 1 DO
                            IF #participates[#d] THEN
                                IF #blockSet.items[#d].aValid AND #blockSet.items[#d].aLeft < #tSync AND #tSync < #blockSet.items[#d].aRight THEN
                                    #blocked := true;
                                    IF #blockSet.items[#d].aRight > #rEdge THEN
                                        #rEdge := #blockSet.items[#d].aRight;
                                    END_IF;
                                END_IF;
                                IF #blockSet.items[#d].bValid AND #blockSet.items[#d].bLeft < #tSync AND #tSync < #blockSet.items[#d].bRight THEN
                                    #blocked := true;
                                    IF #blockSet.items[#d].bRight > #rEdge THEN
                                        #rEdge := #blockSet.items[#d].bRight;
                                    END_IF;
                                END_IF;
                            END_IF;
                        END_FOR;
                        IF #blocked THEN
                            #q := #rEdge / #cycleTime;
                            #kFloor := LREAL_TO_DINT(#q);
                            #tFloor := DINT_TO_LREAL(#kFloor) * #cycleTime;
                            IF #tFloor < #rEdge - 2.0E-12 THEN
                                #tSync := #tFloor + #cycleTime;
                            ELSE
                                #tSync := #tFloor;
                            END_IF;
                        ELSE
                            #stepDone := true;
                        END_IF;
                    END_IF;
                END_FOR;
            END_IF;
        END_REGION
    END_NETWORK
END_FUNCTION
```

- [ ] **Step 4: Update the existing call site in `RuckigOtg.s7dcl`**

The current `"Synchronize"(...)` call (in the Time path) lacks the new params. Add them so it keeps compiling with current behaviour (all axes participate, no discrete). In `src/blocks/RuckigOtg.s7dcl`, find the call and add a VAR_TEMP `allTrue : Array[0..3] of Bool;` (init it `#allTrue[0..3] := true` in four lines just before the call), then update the call to:
```
                    "Synchronize"(
                        blockSet := #blockSet,
                        nDofs := #nd,
                        minimumDuration := #input.minimumDuration,
                        participates := #allTrue,
                        discrete := false,
                        cycleTime := #cycleTime,
                        resolved => #syncOk,
                        tSync => #tSync,
                        limitingAxis => #limitingAxis);
```
with the four init lines before it:
```
                    #allTrue[0] := true;
                    #allTrue[1] := true;
                    #allTrue[2] := true;
                    #allTrue[3] := true;
```
(This call site is rewritten in Task 3; this step just keeps Task 2 green.)

- [ ] **Step 5: Run tests**

Run: `uv run pytest tests/unit/test_synchronize.py -q` (expect the 5 prior + 2 new pass) then `uv run pytest -q` (expect `197 passed`).

- [ ] **Step 6: Commit**
```bash
git add src/blocks/Synchronize.s7dcl tests/unit/test_synchronize.py src/blocks/RuckigOtg.s7dcl
git commit -m "v0.6 T2: Synchronize participation mask + discrete rounding"
```

---

## Task 3: `RuckigOtg` unified per-axis flow

**Files:** `src/blocks/RuckigOtg.s7dcl`, `tests/unit/test_ruckig_otg.py`

This replaces the v0.5 `SYNC_NONE` and the `IF NOT #syncHandled` (Time) blocks with one unified per-axis flow. The `SYNC_PHASE` branch is kept; on phase failure it now falls through to the unified flow (which, with global `synchronization = SYNC_PHASE`, resolves every axis's effective mode to Time — see Step 3 resolution rule).

- [ ] **Step 1: Write the failing FB tests**

Append to `tests/unit/test_ruckig_otg.py`:
```python
def test_per_dof_no_time_mix(harness):
    """perDofSynchronization [No, Time]: axis0 free at its t_min, axis1 synced."""
    inp = default_input()
    inp["nDofs"] = 2
    inp["synchronization"] = 2  # global Time (overridden per-axis)
    inp["perDofSynchronization"] = [0, 2, -1, -1]  # No, Time
    inp["enabled"] = [True, True, False, False]
    inp["targetPosition"] = [1.0, 5.0, 0.0, 0.0]
    inp["maxVelocity"] = [2.0] * 4
    inp["maxAcceleration"] = [5.0] * 4
    inp["maxJerk"] = [10.0] * 4
    reached0 = None
    for cyc in range(1200):
        harness.set_inputs(enable=True, input=inp, cycleTime=0.010, reset=False)
        harness.execute()
        assert harness.get_output("error") is False
        out = harness.get_output("output")
        if reached0 is None and abs(out.newPosition[0] - 1.0) < 1e-3:
            reached0 = cyc
        if harness.get_output("done"):
            break
    assert reached0 is not None and reached0 < 250   # axis0 free, finishes early
    out = harness.get_output("output")
    assert out.newPosition[0] == pytest.approx(1.0, abs=1e-3)
    assert out.newPosition[1] == pytest.approx(5.0, abs=1e-3)


def test_time_if_necessary_rest_vs_moving(harness):
    """TimeIfNecessary: a rest-target axis runs free; with a moving target it syncs."""
    inp = default_input()
    inp["nDofs"] = 2
    inp["synchronization"] = 4  # SYNC_TIME_IF_NECESSARY (global)
    inp["enabled"] = [True, True, False, False]
    inp["targetPosition"] = [1.0, 5.0, 0.0, 0.0]
    inp["targetVelocity"] = [0.0, 0.0, 0.0, 0.0]  # both rest -> both free
    inp["maxVelocity"] = [2.0] * 4
    inp["maxAcceleration"] = [5.0] * 4
    inp["maxJerk"] = [10.0] * 4
    reached0 = None
    for cyc in range(1200):
        harness.set_inputs(enable=True, input=inp, cycleTime=0.010, reset=False)
        harness.execute()
        assert harness.get_output("error") is False
        out = harness.get_output("output")
        if reached0 is None and abs(out.newPosition[0] - 1.0) < 1e-3:
            reached0 = cyc
        if harness.get_output("done"):
            break
    assert reached0 is not None and reached0 < 250  # rest target -> axis0 free
    assert harness.get_output("output").newPosition[1] == pytest.approx(5.0, abs=1e-3)


def test_discrete_duration_is_cycle_multiple(harness):
    """Discrete: trajectory duration is a multiple of cycleTime."""
    inp = default_input()
    inp["nDofs"] = 2
    inp["synchronization"] = 2
    inp["durationDiscretization"] = 1  # DISC_DISCRETE
    inp["enabled"] = [True, True, False, False]
    inp["targetPosition"] = [1.0, 5.0, 0.0, 0.0]
    inp["maxVelocity"] = [2.0] * 4
    inp["maxAcceleration"] = [5.0] * 4
    inp["maxJerk"] = [10.0] * 4
    dt = 0.05
    dur = 0.0
    for _cyc in range(2000):
        harness.set_inputs(enable=True, input=inp, cycleTime=dt, reset=False)
        harness.execute()
        assert harness.get_output("error") is False
        dur = harness.get_output("output").trajectoryDuration
        if harness.get_output("done"):
            break
    ratio = dur / dt
    assert abs(ratio - round(ratio)) < 1e-6, f"duration {dur} not a multiple of {dt}"
    assert harness.get_output("output").newPosition[1] == pytest.approx(5.0, abs=1e-3)
```

- [ ] **Step 2: Run to verify failure**

Run: `uv run pytest tests/unit/test_ruckig_otg.py -k "per_dof or time_if or discrete" -v`
Expected: FAIL — `perDofSynchronization`/`TimeIfNecessary`/Discrete are not yet honoured (axis0 gets stretched / duration not on grid).

- [ ] **Step 3: Replace the SYNC_NONE + Time blocks with the unified flow**

In `src/blocks/RuckigOtg.s7dcl`:

(a) Add VAR_TEMP:
```
        modeEff   : Array[0..3] of Int;
        participates : Array[0..3] of Bool;
        isDiscrete : Bool;
        tinMoving : Bool;
```

(b) The `SYNC_PHASE` branch stays as-is. **Delete** the `SYNC_NONE` branch (the `IF #input.synchronization = "dbRuckigConst".SYNC_NONE THEN ... END_IF;` block) and the entire `IF NOT #syncHandled THEN <Phase1/2/3> END_IF;` Time block, and replace BOTH with the following single block (placed where the SYNC_NONE branch was, i.e. right after `#syncHandled := false;` and before the `SYNC_PHASE` branch — so order is: resolve+unified runs unless Phase already handled). Concretely, structure the multi-axis body as:

```
                IF #multiPath THEN
                    #syncHandled := false;

                    // ---- Global SYNC_PHASE: v0.5 path, else fall through -------
                    IF #input.synchronization = "dbRuckigConst".SYNC_PHASE THEN
                        "PhaseSynchronize"(trajectory := #trajectory,
                            chainPos := #chainPos, chainVel := #chainVel, chainAcc := #chainAcc,
                            input := #input, nDofs := #nd,
                            phaseOk => #phaseDone, duration => #phaseDur);
                        IF #phaseDone THEN
                            FOR #ax := 0 TO #nd - 1 DO
                                #trajectory.profiles[#ax].brake.t[0] := 0.0;
                                #trajectory.profiles[#ax].brake.t[1] := 0.0;
                                #trajectory.profiles[#ax].brake.j[0] := 0.0;
                                #trajectory.profiles[#ax].brake.j[1] := 0.0;
                                #trajectory.profiles[#ax].brake.a[0] := 0.0;
                                #trajectory.profiles[#ax].brake.a[1] := 0.0;
                                #trajectory.profiles[#ax].brake.a[2] := 0.0;
                                #trajectory.profiles[#ax].brake.v[0] := 0.0;
                                #trajectory.profiles[#ax].brake.v[1] := 0.0;
                                #trajectory.profiles[#ax].brake.v[2] := 0.0;
                                #trajectory.profiles[#ax].brake.p[0] := 0.0;
                                #trajectory.profiles[#ax].brake.p[1] := 0.0;
                                #trajectory.profiles[#ax].brake.p[2] := 0.0;
                            END_FOR;
                            #trajectory.duration := #phaseDur;
                            #syncHandled := true;
                        END_IF;
                    END_IF;

                    // ---- Unified per-axis flow (No / Time / TimeIfNecessary) ---
                    IF NOT #syncHandled THEN
                        // 1. resolve effective mode + participation per axis
                        FOR #ax := 0 TO #nd - 1 DO
                            IF #input.perDofSynchronization[#ax] >= 0 THEN
                                #modeEff[#ax] := #input.perDofSynchronization[#ax];
                            ELSE
                                #modeEff[#ax] := #input.synchronization;
                            END_IF;
                            // Phase per-axis entry (or global Phase fallback) -> Time
                            IF #modeEff[#ax] = "dbRuckigConst".SYNC_PHASE THEN
                                #modeEff[#ax] := "dbRuckigConst".SYNC_TIME;
                            END_IF;
                            #participates[#ax] := #modeEff[#ax] <> "dbRuckigConst".SYNC_NONE;
                        END_FOR;

                        // 2. Block per participating axis (for t_sync)
                        FOR #ax := 0 TO #nd - 1 DO
                            IF #participates[#ax] THEN
                                #blockResult := "ComputeBlock1Axis"(
                                    block := #blockSet.items[#ax],
                                    profile := #trajectory.profiles[#ax],
                                    p0 := #chainPos[#ax], v0 := #chainVel[#ax], a0 := #chainAcc[#ax],
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
                            END_IF;
                        END_FOR;

                        // 3. t_sync over participating axes (+ discrete)
                        #isDiscrete := #input.durationDiscretization = "dbRuckigConst".DISC_DISCRETE;
                        "Synchronize"(
                            blockSet := #blockSet,
                            nDofs := #nd,
                            minimumDuration := #input.minimumDuration,
                            participates := #participates,
                            discrete := #isDiscrete,
                            cycleTime := #cycleTime,
                            resolved => #syncOk,
                            tSync => #tSync,
                            limitingAxis => #limitingAxis);
                        IF NOT #syncOk THEN
                            #valid := false;
                            #busy := false;
                            #done := false;
                            #error := true;
                            #status := "dbRuckigConst".RESULT_ERR_SYNC;
                            #trajectory.isValid := false;
                            RETURN;
                        END_IF;

                        // 4. emit per axis
                        #mainDuration := 0.0;
                        FOR #ax := 0 TO #nd - 1 DO
                            // zero brake (no multi-axis brake)
                            #trajectory.profiles[#ax].brake.t[0] := 0.0;
                            #trajectory.profiles[#ax].brake.t[1] := 0.0;
                            #trajectory.profiles[#ax].brake.j[0] := 0.0;
                            #trajectory.profiles[#ax].brake.j[1] := 0.0;
                            #trajectory.profiles[#ax].brake.a[0] := 0.0;
                            #trajectory.profiles[#ax].brake.a[1] := 0.0;
                            #trajectory.profiles[#ax].brake.a[2] := 0.0;
                            #trajectory.profiles[#ax].brake.v[0] := 0.0;
                            #trajectory.profiles[#ax].brake.v[1] := 0.0;
                            #trajectory.profiles[#ax].brake.v[2] := 0.0;
                            #trajectory.profiles[#ax].brake.p[0] := 0.0;
                            #trajectory.profiles[#ax].brake.p[1] := 0.0;
                            #trajectory.profiles[#ax].brake.p[2] := 0.0;

                            // is this axis time-synced? Time, or TimeIfNecessary w/ moving target
                            #tinMoving := #modeEff[#ax] = "dbRuckigConst".SYNC_TIME;
                            IF #modeEff[#ax] = "dbRuckigConst".SYNC_TIME_IF_NECESSARY THEN
                                IF ABS(#input.targetVelocity[#ax]) > "dbRuckigConst".EPS_VELOCITY
                                    OR ABS(#input.targetAcceleration[#ax]) > "dbRuckigConst".EPS_DERIV THEN
                                    #tinMoving := true;
                                END_IF;
                            END_IF;

                            #atTarget :=
                                ABS(#input.targetPosition[#ax] - #chainPos[#ax]) < "dbRuckigConst".EPS_POSITION
                                AND ABS(#input.targetVelocity[#ax] - #chainVel[#ax]) < "dbRuckigConst".EPS_VELOCITY
                                AND ABS(#input.targetAcceleration[#ax] - #chainAcc[#ax]) < "dbRuckigConst".EPS_DERIV;

                            IF #atTarget THEN
                                FOR #i := 0 TO 6 DO
                                    #trajectory.profiles[#ax].t[#i] := 0.0;
                                    #trajectory.profiles[#ax].j[#i] := 0.0;
                                END_FOR;
                                FOR #i := 0 TO 7 DO
                                    #trajectory.profiles[#ax].a[#i] := #chainAcc[#ax];
                                    #trajectory.profiles[#ax].v[#i] := #chainVel[#ax];
                                    #trajectory.profiles[#ax].p[#i] := #chainPos[#ax];
                                END_FOR;
                            ELSIF #tinMoving THEN
                                #profileResult := "ComputeProfile1AxisTimed"(
                                    profile := #trajectory.profiles[#ax],
                                    p0 := #chainPos[#ax], v0 := #chainVel[#ax], a0 := #chainAcc[#ax],
                                    pT := #input.targetPosition[#ax], vT := #input.targetVelocity[#ax],
                                    aT := #input.targetAcceleration[#ax],
                                    vMax := #input.maxVelocity[#ax], aMax := #input.maxAcceleration[#ax],
                                    jMax := #input.maxJerk[#ax], tf := #tSync);
                                IF #profileResult <> "dbRuckigConst".RESULT_WORKING AND #profileResult <> "dbRuckigConst".RESULT_FINISHED THEN
                                    #valid := false;
                                    #busy := false;
                                    #done := false;
                                    #error := true;
                                    #status := "dbRuckigConst".RESULT_ERR_TRAJ;
                                    #trajectory.isValid := false;
                                    RETURN;
                                END_IF;
                            ELSE
                                #profileResult := "ComputeProfile1Axis"(profile := #trajectory.profiles[#ax],
                                    p0 := #chainPos[#ax], v0 := #chainVel[#ax], a0 := #chainAcc[#ax],
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

                            #axDur := 0.0;
                            FOR #i := 0 TO 6 DO
                                #axDur := #axDur + #trajectory.profiles[#ax].t[#i];
                            END_FOR;
                            #trajectory.independentMinDurations[#ax] := #axDur;
                            IF #axDur > #mainDuration THEN
                                #mainDuration := #axDur;
                            END_IF;
                        END_FOR;
                        #trajectory.duration := #mainDuration;
                        #syncHandled := true;
                    END_IF;
                END_IF;
```

Note: the `#allTrue` VAR_TEMP and init lines added in Task 2 are no longer used here (the call moved into the unified block with `#participates`). Remove the four `#allTrue[...] := true;` lines and the `allTrue` declaration to avoid an unused variable. Keep the unified-flow `participates`.

- [ ] **Step 4: Run the FB tests + regression**

Run: `uv run pytest tests/unit/test_ruckig_otg.py -q`
Expected: all pass — the new per_dof / TimeIfNecessary / Discrete tests AND the existing global No/Time/Phase FB tests (the unified flow reproduces global No = all-No, global Time = all-Time; Phase keeps its branch).

- [ ] **Step 5: Run full suite**

Run: `uv run pytest -q`
Expected: `200 passed` (197 + 3 new). All 31 parity scenarios still pass (no scenario sets the new fields yet).

- [ ] **Step 6: Commit**
```bash
git add src/blocks/RuckigOtg.s7dcl tests/unit/test_ruckig_otg.py
git commit -m "v0.6 T3: RuckigOtg unified per-axis flow (per_dof / TimeIfNecessary / Discrete)"
```

---

## Task 4: Parity — thread perDof + discretization + v06 scenarios

**Files:** `tests/parity/runner_ref.py`, `tests/parity/runner_scl.py`, `tests/parity/test_parity.py`, `tests/parity/scenarios/v06_*.yaml`

- [ ] **Step 1: `runner_ref.py`**

Add a per-axis map and params. After the existing `_SYNC_MAP`, add:
```python
_DISC_MAP = {0: ruckig.DurationDiscretization.Continuous, 1: ruckig.DurationDiscretization.Discrete}
```
Add keyword-only params `per_dof_synchronization: list | None = None,` and `duration_discretization: int = 0,` to `run_reference`. After `inp.synchronization = ...`, add:
```python
    inp.duration_discretization = _DISC_MAP[duration_discretization]
    if per_dof_synchronization is not None:
        inp.per_dof_synchronization = [_SYNC_MAP[m] for m in per_dof_synchronization[:n_dofs]]
```

- [ ] **Step 2: `runner_scl.py`**

Add keyword-only params `per_dof_synchronization: list | None = None,` and `duration_discretization: int = 0,` to `run_scl`. In the input dict set:
```python
        "perDofSynchronization": (list(per_dof_synchronization) + [-1] * 4)[:4] if per_dof_synchronization is not None else [-1, -1, -1, -1],
        "durationDiscretization": duration_discretization,
```

- [ ] **Step 3: `test_parity.py`**

In `_kwargs_from_scenario`, add:
```python
        "per_dof_synchronization": data.get("per_dof_synchronization"),
        "duration_discretization": data.get("duration_discretization", 0),
```

- [ ] **Step 4: Create scenarios**

`tests/parity/scenarios/v06_01_2dof_per_dof_no_time.yaml`:
```yaml
name: v0.6 2-DoF per-DoF [No, Time]
# Axis0 runs free (No) at its own t_min; axis1 time-synced. Must match Ruckig.
n_dofs: 2
per_dof_synchronization: [0, 2]
target_position: [1.0, 5.0]
max_velocity: [2.0, 2.0]
max_acceleration: [5.0, 5.0]
max_jerk: [10.0, 10.0]
cycle_time: 0.010
```

`tests/parity/scenarios/v06_02_2dof_time_if_necessary.yaml`:
```yaml
name: v0.6 2-DoF TimeIfNecessary (rest targets)
# Both targets at rest -> TimeIfNecessary leaves each axis at its own t_min.
n_dofs: 2
synchronization: 4
target_position: [1.0, 5.0]
max_velocity: [2.0, 2.0]
max_acceleration: [5.0, 5.0]
max_jerk: [10.0, 10.0]
cycle_time: 0.010
```

`tests/parity/scenarios/v06_03_2dof_time_if_necessary_moving.yaml`:
```yaml
name: v0.6 2-DoF TimeIfNecessary (one moving target)
# Axis0 has a moving target (vT!=0) -> it is time-synced; axis1 (rest) free.
n_dofs: 2
synchronization: 4
target_position: [1.0, 5.0]
target_velocity: [0.5, 0.0]
max_velocity: [2.0, 2.0]
max_acceleration: [5.0, 5.0]
max_jerk: [10.0, 10.0]
cycle_time: 0.010
```

`tests/parity/scenarios/v06_04_2dof_discrete.yaml`:
```yaml
name: v0.6 2-DoF discrete duration
# Time sync with discrete duration rounding to the cycle grid.
n_dofs: 2
synchronization: 2
duration_discretization: 1
target_position: [1.0, 5.0]
max_velocity: [2.0, 2.0]
max_acceleration: [5.0, 5.0]
max_jerk: [10.0, 10.0]
cycle_time: 0.050
```

- [ ] **Step 5: Run parity + full suite**

Run: `uv run pytest tests/parity -q` (expect 35 passed) then `uv run pytest -q` (expect `204 passed`).
If any v06 scenario mismatches, DEBUG (do not loosen tolerances): confirm the SCL output equals Ruckig; report posErr/velErr/durErr if a genuine mismatch is found.

- [ ] **Step 6: Commit**
```bash
git add tests/parity/runner_ref.py tests/parity/runner_scl.py tests/parity/test_parity.py tests/parity/scenarios/v06_01_2dof_per_dof_no_time.yaml tests/parity/scenarios/v06_02_2dof_time_if_necessary.yaml tests/parity/scenarios/v06_03_2dof_time_if_necessary_moving.yaml tests/parity/scenarios/v06_04_2dof_discrete.yaml
git commit -m "v0.6 T4: parity for per_dof / TimeIfNecessary / discrete"
```

---

## Task 5: Release 0.6.0 (docs + version)

**Files:** `CHANGELOG.md`, `README.md`, `pyproject.toml`, `src/blocks/Synchronize.s7dcl` (S7_Version already 0.6.0), `src/blocks/RuckigOtg.s7dcl`

> Do NOT merge/tag here — that is the finishing step after the final review.

- [ ] **Step 1: Version bump**

`pyproject.toml`: `version = "0.5.0"` → `"0.6.0"`. `src/blocks/RuckigOtg.s7dcl` header `S7_Version` → `"0.6.0"` (Synchronize is already `0.6.0` from Task 2; per-block versioning leaves the rest).

- [ ] **Step 2: CHANGELOG `[0.6.0]`**

In `CHANGELOG.md`, above `## [0.5.0]`:
```markdown
## [0.6.0] - 2026-06-03

Multi-axis synchronization completeness, on top of v0.5.

### Added
- **`perDofSynchronization`** (`typeRuckigInput`, `Array[0..3] of Int`, `-1` =
  use the global mode) — each axis independently `No` / `Time` /
  `TimeIfNecessary`. (A `Phase` per-DoF entry is treated as `Time`; `Phase`
  stays a global mode.)
- **`TimeIfNecessary`** (`SYNC_TIME_IF_NECESSARY = 4`) — an axis is time-synced
  only if its target is moving (`vT ≠ 0` or `aT ≠ 0`); a rest target runs free
  at its time-optimal duration.
- **`DurationDiscretization.Discrete`** — `t_sync` is rounded up to a multiple of
  `cycleTime` (jumping any blocked interval the rounding lands in), then every
  synchronized axis is re-timed to it.

### Changed
- `RuckigOtg`'s multi-axis recompute keeps the global `Phase` branch and replaces
  the global `No`/`Time` branches with one unified per-axis flow (resolve mode →
  Block participating axes → `Synchronize` over a participation mask → emit per
  axis). The global modes are the special case "all axes share one mode".
- `Synchronize` gains a per-axis participation mask (only `No` axes are excluded
  from `t_sync`) and discrete-duration rounding.

### Parity
- 4 new scenarios (`v06_01..04`): per-DoF `[No, Time]`, `TimeIfNecessary` rest and
  moving, discrete duration. 35 parity scenarios total. 204 tests pass.

### Known limitations
- Per-DoF `Phase` mixes are not supported (a Phase per-DoF entry → Time); Phase is
  a global mode (matches Ruckig, which abandons phase whenever a Time axis is in
  the mix). The velocity control interface → v0.7; brake profiles → v0.8.
```
Add the link line near the others:
```markdown
[0.6.0]: https://github.com/core-engineering/ruckig-scl/releases/tag/v0.6.0
```

- [ ] **Step 3: README**

Update: status line → v0.6.0 (per-DoF sync + TimeIfNecessary + discrete duration); add a "Features (v0.6)" group; update parity count to 35; in the roadmap mark v0.6 done and set v0.7 = velocity interface, v0.8 = brake. Read the README first; match its style.

- [ ] **Step 4: Run full suite**

Run: `uv run pytest -q`
Expected: `204 passed`.

- [ ] **Step 5: Commit**
```bash
git add CHANGELOG.md README.md pyproject.toml src/blocks/RuckigOtg.s7dcl
git commit -m "v0.6 T5: release 0.6.0 (docs + version)"
```

---

## Self-review notes

- **Spec coverage:** perDofSynchronization → T1, T3, T4; TimeIfNecessary → T3, T4; Discrete → T2 (Synchronize), T3 (wiring), T4; participation mask → T2; unified flow → T3; Phase-stays-global (PhaseSynchronize unchanged) → T3 keeps the branch; release → T5. All spec sections covered.
- **Type consistency:** `Synchronize` new params (`participates : Array[0..3] of Bool`, `discrete : Bool`, `cycleTime : LReal`) match between its definition (T2) and call site (T3). `modeEff`/`participates`/`isDiscrete`/`tinMoving` VAR_TEMP declared in T3. Mode constants `SYNC_NONE=0`, `SYNC_PHASE=1`, `SYNC_TIME=2`, `SYNC_TIME_IF_NECESSARY=4`, `DISC_DISCRETE=1` consistent across FB, tests, runners.
- **No identifiers ending in of/Dof** (`ax`, `modeEff`, `tinMoving`). One `:=` per line. The Task 2 `allTrue` scaffold is explicitly removed in Task 3 Step 3 to avoid an unused-variable artifact.
- **Critical-fix carryover:** every per-axis `ComputeProfile1Axis` call in the emission loop is guarded by the `atTarget` zero-fill (the v0.5 zero-displacement fix), and `ComputeBlock1Axis` has its own at-target guard.
