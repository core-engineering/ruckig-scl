# Ruckig SCL v0.5 — Phase Synchronization + No-Sync — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add the `Phase` and `No` multi-axis synchronization modes to `RuckigOtg`, on top of v0.4's `Time` sync, matching the Ruckig oracle.

**Architecture:** `RuckigOtg` dispatches on `input.synchronization`. A new self-contained FC `PhaseSynchronize` tries phase sync (collinearity test → scale the limiting axis's profile onto every axis → re-validate); on failure it signals fallback and the FB runs the unchanged v0.4 `Time` path. `No` is a per-axis step1 loop. The v0.4 `Time` code is untouched (regression guard).

**Tech Stack:** Siemens SCL (TIA Portal export, `.s7dcl`), tested in Python via the `siemens-plc-tools` `plc-code` transpiler harness; parity vs PyPI `ruckig` 0.17.3.

**Branch:** `feature/v0.5-phase-sync` (spec at `docs/superpowers/specs/2026-06-03-ruckig-scl-v0.5-design.md`).

**Transpiler quirks to respect (from prior versions):**
1. `END_IF` / `END_FOR` on their own line.
2. No identifier ending in `of`/`Dof` (lexer mis-reads `OF`). → use `refAx`, `scaleAx`, never `refDof`.
3. No division by a parenthesised product `/(a*b)`; precompute scalar denominators.
4. One `:=` per source line (a second on the same line is silently dropped).
5. Array-of-scalar (`Array[0..3] of LReal`) works as an FC parameter; array-of-UDT does **not** (wrap in a STRUCT).
6. `=>` VAR_OUTPUT on a `: Void` FC works (captured). (The return-value-consumed bug was fixed upstream, but `PhaseSynchronize` is `: Void` regardless.)

**Oracle reference (already probed):** `Synchronization.Phase` on a collinear input yields identical `profile.t[]` across DoFs with jerk scaled by the displacement ratio; on a non-collinear input it silently falls back to `Time`. `Synchronization.No` lets each axis keep its own time-optimal duration.

---

## File structure

| File | Create/Modify | Responsibility |
|------|---------------|----------------|
| `src/data-blocks/dbRuckigConst.s7dcl` | Modify | add `EPS_PHASE` |
| `src/data-types/typeRuckigInput.s7dcl` | Modify | `synchronization` default → `SYNC_TIME` |
| `src/blocks/PhaseSynchronize.s7dcl` | **Create** | collinearity test + scale limiting profile per axis + per-axis validate |
| `src/blocks/RuckigOtg.s7dcl` | Modify | dispatch on `synchronization`; No branch; Phase branch + Time fallback; change detection |
| `tests/unit/test_phase_synchronize.py` | **Create** | unit tests for `PhaseSynchronize` |
| `tests/unit/test_ruckig_otg.py` | Modify | No / Phase / fallback FB tests; default helper → `SYNC_TIME` |
| `tests/unit/test_validate_input.py` | Modify | default helper → `SYNC_TIME` |
| `tests/parity/runner_ref.py`, `runner_scl.py`, `test_parity.py` | Modify | thread `synchronization` |
| `tests/parity/scenarios/v05_*.yaml` | **Create** | No / Phase-collinear / Phase-fallback scenarios |
| `CHANGELOG.md`, `README.md`, `pyproject.toml` | Modify | v0.5.0 release |

---

## Task 1: Constants, input default, change detection

**Files:**
- Modify: `src/data-blocks/dbRuckigConst.s7dcl`
- Modify: `src/data-types/typeRuckigInput.s7dcl`
- Modify: `src/blocks/RuckigOtg.s7dcl` (change-detection region)
- Modify: `tests/unit/test_ruckig_otg.py` (`default_input`), `tests/unit/test_validate_input.py`
- Modify: `tests/parity/runner_scl.py` (default sync int)

- [ ] **Step 1: Add `EPS_PHASE` constant**

In `src/data-blocks/dbRuckigConst.s7dcl`, after `EPS_DERIV`:
```
        EPS_PHASE             : LReal := 1.0E-9;
```
(Collinearity / scale-selection tolerance. Looser than Ruckig's machine-eps so LReal rounding on exactly-collinear inputs still passes; parity tests use exactly-collinear or clearly-non-collinear inputs, so the exact value does not affect parity.)

- [ ] **Step 2: Change `synchronization` default to Time**

In `src/data-types/typeRuckigInput.s7dcl`:
```
        synchronization        : Int := 2;   // default SYNC_TIME (matches Ruckig)
```
(Was `0`. v0.4 ignored the field; v0.5 activates it, so the default must reproduce v0.4's Time behaviour for callers who do not set it.)

- [ ] **Step 3: Add `synchronization` to change detection**

In `src/blocks/RuckigOtg.s7dcl`, the `REGION Change detection` block, inside the `IF NOT #recomputeRequired THEN` guard, after the `minimumDuration` check (just before the closing `END_IF;` of that guard):
```
                IF #input.synchronization <> #prevInput.synchronization THEN
                    #recomputeRequired := true;
                END_IF;
```

- [ ] **Step 4: Set `SYNC_TIME` explicitly in test helpers (keep behaviour)**

In `tests/unit/test_ruckig_otg.py` `default_input()`, change `"synchronization": 0,` to `"synchronization": 2,`.
In `tests/unit/test_validate_input.py` base dict, change `"synchronization": 0,` to `"synchronization": 2,`.
In `tests/parity/runner_scl.py`, change the hard-coded `"synchronization": 0,` to use the threaded value (done in Task 5; for now set `"synchronization": 2,`).

- [ ] **Step 5: Run full suite (no behaviour change expected)**

Run: `uv run pytest -q`
Expected: `181 passed` (default now Time, FB still always-Time; no regression).

- [ ] **Step 6: Commit**
```bash
git add src/data-blocks/dbRuckigConst.s7dcl src/data-types/typeRuckigInput.s7dcl src/blocks/RuckigOtg.s7dcl tests/unit/test_ruckig_otg.py tests/unit/test_validate_input.py tests/parity/runner_scl.py
git commit -m "v0.5 T1: EPS_PHASE + synchronization default Time + change detection"
```

---

## Task 2: `PhaseSynchronize` FC

**Files:**
- Create: `src/blocks/PhaseSynchronize.s7dcl`
- Test: `tests/unit/test_phase_synchronize.py`

- [ ] **Step 1: Write the failing tests**

Create `tests/unit/test_phase_synchronize.py`:
```python
"""Unit tests for PhaseSynchronize (multi-DoF phase synchronization)."""
import pytest

ruckig = pytest.importorskip("ruckig")


@pytest.fixture
def harness(make_harness):
    return make_harness("PhaseSynchronize.s7dcl")


def _empty_profile():
    return {
        "t": [0.0] * 7, "j": [0.0] * 7,
        "a": [0.0] * 8, "v": [0.0] * 8, "p": [0.0] * 8,
        "direction": 1,
        "brake": {"t": [0.0, 0.0], "j": [0.0, 0.0],
                  "a": [0.0, 0.0, 0.0], "v": [0.0, 0.0, 0.0], "p": [0.0, 0.0, 0.0]},
        "controlSigns": 0,
    }


def _trajectory():
    return {"profiles": [_empty_profile() for _ in range(4)],
            "duration": 0.0, "independentMinDurations": [0.0] * 4,
            "currentTime": 0.0, "isValid": False}


def _input(n, cp, cv, ca, tp, tv, ta, vM, aM, jM):
    pad = lambda x: list(x) + [0.0] * (4 - len(x))
    return {
        "currentPosition": pad(cp), "currentVelocity": pad(cv), "currentAcceleration": pad(ca),
        "targetPosition": pad(tp), "targetVelocity": pad(tv), "targetAcceleration": pad(ta),
        "maxVelocity": pad(vM), "maxAcceleration": pad(aM), "maxJerk": pad(jM),
        "enabled": [d < n for d in range(4)], "nDofs": n,
        "minimumDuration": -1.0, "synchronization": 1,
        "controlInterface": 0, "durationDiscretization": 0,
    }


def _run(harness, n, cp, cv, ca, tp, tv, ta, vM, aM, jM):
    # chain seed = current state (first solve)
    pad = lambda x: list(x) + [0.0] * (4 - len(x))
    harness.reset()
    harness.set_inputs(
        trajectory=_trajectory(),
        chainPos=pad(cp), chainVel=pad(cv), chainAcc=pad(ca),
        input=_input(n, cp, cv, ca, tp, tv, ta, vM, aM, jM), nDofs=n)
    harness.execute()
    return (harness.get_output("phaseOk"),
            harness.get_output("duration"),
            harness.get_var("trajectory"))


def _profile_t(traj, d):
    pr = traj.profiles[d]
    return [pr.t[i] for i in range(7)]


def test_collinear_rest_to_rest_phase_aligned(harness):
    # axis1 = 3 * axis0 displacement; collinear -> phase OK, shared t[].
    ok, dur, traj = _run(harness, 2, [0, 0], [0, 0], [0, 0],
                         [1.0, 3.0], [0, 0], [0, 0], [3, 3], [5, 5], [10, 10])
    assert ok is True
    assert _profile_t(traj, 0) == pytest.approx(_profile_t(traj, 1), abs=1e-9)
    # jerk scaled by 3
    assert traj.profiles[1].j[0] == pytest.approx(3.0 * traj.profiles[0].j[0], abs=1e-6)


def test_collinear_moving_target(harness):
    # pd, v0, vT all in ratio 1:3 -> collinear.
    ok, dur, traj = _run(harness, 2, [0, 0], [0.4, 1.2], [0, 0],
                         [1.0, 3.0], [0.2, 0.6], [0, 0], [3, 3], [5, 5], [10, 10])
    assert ok is True
    assert _profile_t(traj, 0) == pytest.approx(_profile_t(traj, 1), abs=1e-9)


def test_non_collinear_returns_false(harness):
    # v0 = [1, -1] is not collinear with pd = [1, 3] -> phase impossible.
    ok, dur, traj = _run(harness, 2, [0, 0], [1.0, -1.0], [0, 0],
                         [1.0, 3.0], [0, 0], [0, 0], [3, 3], [5, 5], [10, 10])
    assert ok is False


def test_collinear_reaches_each_target(harness):
    # Phase profiles must each reach their own target at the common duration.
    ok, dur, traj = _run(harness, 2, [0, 0], [0, 0], [0, 0],
                         [1.0, 3.0], [0, 0], [0, 0], [3, 3], [5, 5], [10, 10])
    assert ok is True
    assert traj.profiles[0].p[7] == pytest.approx(1.0, abs=1e-6)
    assert traj.profiles[1].p[7] == pytest.approx(3.0, abs=1e-6)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `uv run pytest tests/unit/test_phase_synchronize.py -q`
Expected: FAIL (harness cannot find `PhaseSynchronize.s7dcl`).

- [ ] **Step 3: Write the implementation**

Create `src/blocks/PhaseSynchronize.s7dcl`:
```
{
    S7_Author := "Martin C";
    S7_EditorMode := "SCL";
    S7_Family := "Ruckig";
    S7_Optimized := "TRUE";
    S7_Version := "0.5.0"
}
FUNCTION "PhaseSynchronize" : Void
    VAR_IN_OUT
        trajectory : _.typeTrajectory;
    END_VAR
    VAR_INPUT
        chainPos : Array[0..3] of LReal;
        chainVel : Array[0..3] of LReal;
        chainAcc : Array[0..3] of LReal;
        input    : _.typeRuckigInput;
        nDofs    : Int;
    END_VAR
    VAR_OUTPUT
        phaseOk  : Bool;
        duration : LReal;
    END_VAR
    VAR_TEMP
        ax      : Int;
        i       : DInt;
        refAx   : Int;
        scaleAx : Int;
        scaleQ  : Int;
        res     : Word;
        dur     : LReal;
        durBest : LReal;
        pd      : Array[0..3] of LReal;
        sval    : Array[0..3] of LReal;
        rPd     : LReal;
        rV0     : LReal;
        rA0     : LReal;
        rVt     : LReal;
        rAt     : LReal;
        denom   : LReal;
        kd      : LReal;
        cs      : LReal;
        ok      : Bool;
        epsP    : LReal;
    END_VAR

    { S7_Language := "SCL" }
    NETWORK
        REGION Block header
            //==========================================================
            // Title:      PhaseSynchronize
            // Function:   v0.5 phase synchronization. Solves step1 per axis,
            //             picks the limiting (slowest) axis as the phase profile,
            //             tests collinearity of (pd, v0, a0, vT, aT) across DoFs,
            //             and if collinear copies the limiting axis's timing onto
            //             every axis with jerk scaled by the displacement ratio,
            //             then re-validates each (CheckProfile). phaseOk = false
            //             on any non-collinearity or limit violation -> caller
            //             falls back to Time sync. Port of Ruckig
            //             TargetCalculator::is_input_collinear + phase build
            //             (calculator_target.hpp l.44-117, 392-456).
            // Family:     Ruckig
            // Author:     Martin C
            //==========================================================
        END_REGION

        REGION Per-axis step1 + limiting (reference) axis
            #phaseOk := false;
            #duration := 0.0;
            #epsP := "dbRuckigConst".EPS_PHASE;
            #refAx := 0;
            #durBest := -1.0;
            FOR #ax := 0 TO #nDofs - 1 DO
                #res := "ComputeProfile1Axis"(profile := #trajectory.profiles[#ax],
                    p0 := #chainPos[#ax], v0 := #chainVel[#ax], a0 := #chainAcc[#ax],
                    pT := #input.targetPosition[#ax], vT := #input.targetVelocity[#ax],
                    aT := #input.targetAcceleration[#ax],
                    vMax := #input.maxVelocity[#ax], aMax := #input.maxAcceleration[#ax],
                    jMax := #input.maxJerk[#ax]);
                IF #res <> "dbRuckigConst".RESULT_WORKING AND #res <> "dbRuckigConst".RESULT_FINISHED THEN
                    RETURN;
                END_IF;
                #pd[#ax] := #input.targetPosition[#ax] - #chainPos[#ax];
                #dur := 0.0;
                FOR #i := 0 TO 6 DO
                    #dur := #dur + #trajectory.profiles[#ax].t[#i];
                END_FOR;
                IF #dur > #durBest THEN
                    #durBest := #dur;
                    #refAx := #ax;
                END_IF;
            END_FOR;
        END_REGION

        REGION Select scale quantity (priority pd > v0 > a0 > vT > aT)
            #scaleAx := -1;
            #scaleQ := -1;
            FOR #ax := 0 TO #nDofs - 1 DO
                IF #scaleAx < 0 THEN
                    IF ABS(#pd[#ax]) > #epsP THEN
                        #scaleAx := #ax;
                        #scaleQ := 0;
                    ELSIF ABS(#chainVel[#ax]) > #epsP THEN
                        #scaleAx := #ax;
                        #scaleQ := 1;
                    ELSIF ABS(#chainAcc[#ax]) > #epsP THEN
                        #scaleAx := #ax;
                        #scaleQ := 2;
                    ELSIF ABS(#input.targetVelocity[#ax]) > #epsP THEN
                        #scaleAx := #ax;
                        #scaleQ := 3;
                    ELSIF ABS(#input.targetAcceleration[#ax]) > #epsP THEN
                        #scaleAx := #ax;
                        #scaleQ := 4;
                    END_IF;
                END_IF;
            END_FOR;
            IF #scaleAx < 0 THEN
                RETURN;
            END_IF;
        END_REGION

        REGION Per-axis scale value
            FOR #ax := 0 TO #nDofs - 1 DO
                IF #scaleQ = 0 THEN
                    #sval[#ax] := #pd[#ax];
                ELSIF #scaleQ = 1 THEN
                    #sval[#ax] := #chainVel[#ax];
                ELSIF #scaleQ = 2 THEN
                    #sval[#ax] := #chainAcc[#ax];
                ELSIF #scaleQ = 3 THEN
                    #sval[#ax] := #input.targetVelocity[#ax];
                ELSE
                    #sval[#ax] := #input.targetAcceleration[#ax];
                END_IF;
            END_FOR;
            IF ABS(#sval[#refAx]) < #epsP THEN
                RETURN;
            END_IF;
        END_REGION

        REGION Collinearity ratios (relative to scale axis)
            #denom := #sval[#scaleAx];
            #rPd := #pd[#scaleAx] / #denom;
            #rV0 := #chainVel[#scaleAx] / #denom;
            #rA0 := #chainAcc[#scaleAx] / #denom;
            #rVt := #input.targetVelocity[#scaleAx] / #denom;
            #rAt := #input.targetAcceleration[#scaleAx] / #denom;
        END_REGION

        REGION Collinearity test (every DoF, all five quantities)
            FOR #ax := 0 TO #nDofs - 1 DO
                #cs := #sval[#ax];
                IF ABS(#pd[#ax] - #rPd * #cs) > #epsP THEN
                    RETURN;
                END_IF;
                IF ABS(#chainVel[#ax] - #rV0 * #cs) > #epsP THEN
                    RETURN;
                END_IF;
                IF ABS(#chainAcc[#ax] - #rA0 * #cs) > #epsP THEN
                    RETURN;
                END_IF;
                IF ABS(#input.targetVelocity[#ax] - #rVt * #cs) > #epsP THEN
                    RETURN;
                END_IF;
                IF ABS(#input.targetAcceleration[#ax] - #rAt * #cs) > #epsP THEN
                    RETURN;
                END_IF;
            END_FOR;
        END_REGION

        REGION Scale limiting profile onto each axis + validate
            FOR #ax := 0 TO #nDofs - 1 DO
                IF #ax <> #refAx THEN
                    #kd := #sval[#ax] / #sval[#refAx];
                    FOR #i := 0 TO 6 DO
                        #trajectory.profiles[#ax].t[#i] := #trajectory.profiles[#refAx].t[#i];
                        #trajectory.profiles[#ax].j[#i] := #kd * #trajectory.profiles[#refAx].j[#i];
                    END_FOR;
                    #trajectory.profiles[#ax].direction := #trajectory.profiles[#refAx].direction;
                    #trajectory.profiles[#ax].controlSigns := #trajectory.profiles[#refAx].controlSigns;
                    "IntegrateProfileStates"(profile := #trajectory.profiles[#ax],
                        p0 := #chainPos[#ax], v0 := #chainVel[#ax], a0 := #chainAcc[#ax]);
                    #ok := "CheckProfile"(profile := #trajectory.profiles[#ax],
                        p0 := #chainPos[#ax], v0 := #chainVel[#ax], a0 := #chainAcc[#ax],
                        pT := #input.targetPosition[#ax], vT := #input.targetVelocity[#ax],
                        aT := #input.targetAcceleration[#ax],
                        vMax := #input.maxVelocity[#ax], aMax := #input.maxAcceleration[#ax],
                        tf := #durBest);
                    IF NOT #ok THEN
                        RETURN;
                    END_IF;
                END_IF;
            END_FOR;
        END_REGION

        REGION Success
            #duration := #durBest;
            #phaseOk := true;
        END_REGION
    END_NETWORK
END_FUNCTION
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/unit/test_phase_synchronize.py -q`
Expected: PASS (4 tests).

- [ ] **Step 5: Commit**
```bash
git add src/blocks/PhaseSynchronize.s7dcl tests/unit/test_phase_synchronize.py
git commit -m "v0.5 T2: PhaseSynchronize FC (collinearity + scale + validate)"
```

---

## Task 3: `RuckigOtg` — `SYNC_NO` branch + Time wrapper

**Files:**
- Modify: `src/blocks/RuckigOtg.s7dcl`
- Test: `tests/unit/test_ruckig_otg.py`

- [ ] **Step 1: Write the failing test**

Append to `tests/unit/test_ruckig_otg.py`:
```python
def test_sync_no_axes_finish_independently(harness):
    """SYNC_NO: a short axis reaches its target at its OWN t_min, not stretched."""
    inp = default_input()
    inp["nDofs"] = 2
    inp["synchronization"] = 0  # SYNC_NONE
    inp["enabled"] = [True, True, False, False]
    inp["targetPosition"] = [1.0, 8.0, 0.0, 0.0]
    inp["maxVelocity"] = [2.0] * 4
    inp["maxAcceleration"] = [5.0] * 4
    inp["maxJerk"] = [10.0] * 4

    reached0 = None
    for cyc in range(2000):
        harness.set_inputs(enable=True, input=inp, cycleTime=0.010, reset=False)
        harness.execute()
        out = harness.get_output("output")
        if reached0 is None and abs(out.newPosition[0] - 1.0) < 1e-3:
            reached0 = cyc
        if harness.get_output("done"):
            break
    # axis0 (1 m) reaches well before the slow axis1 (8 m) finishes.
    assert reached0 is not None and reached0 < 200
    assert harness.get_output("output").newPosition[0] == pytest.approx(1.0, abs=1e-3)
    assert harness.get_output("output").newPosition[1] == pytest.approx(8.0, abs=1e-3)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest tests/unit/test_ruckig_otg.py::test_sync_no_axes_finish_independently -v`
Expected: FAIL — with the default Time path, axis0 is stretched and reaches 1.0 only near the end (`reached0` large).

- [ ] **Step 3: Add VAR_TEMP and wrap the Time block**

In `src/blocks/RuckigOtg.s7dcl` VAR_TEMP, add:
```
        syncHandled : Bool;
        axDur       : LReal;
        phaseDur    : LReal;
        phaseDone   : Bool;
```
Then, in the `IF #multiPath THEN` block, immediately after `IF #multiPath THEN`, insert the SYNC_NO branch and open the handled-guard:
```
                IF #multiPath THEN
                    #syncHandled := false;

                    // ---- SYNC_NO: each axis at its own time-optimal duration --
                    IF #input.synchronization = "dbRuckigConst".SYNC_NONE THEN
                        #mainDuration := 0.0;
                        FOR #ax := 0 TO #nd - 1 DO
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

                    // ---- TIME (default + Phase fallback) -----------------------
                    IF NOT #syncHandled THEN
```
Then **indent nothing** — instead, close the new `IF NOT #syncHandled THEN` before the existing `#trajectory.duration := #tSync;`'s following lines. Concretely: the existing Time body (Phase 1 Block → Phase 2 Synchronize → Phase 3 re-time, ending at `#trajectory.duration := #tSync;`) now lives inside `IF NOT #syncHandled THEN ... END_IF;`. Add the closing `END_IF;` immediately after `#trajectory.duration := #tSync;`:
```
                        #trajectory.duration := #tSync;
                    END_IF;
                END_IF;
```
(The final `END_IF;` is the existing close of `IF #multiPath THEN`.)

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/unit/test_ruckig_otg.py -q`
Expected: PASS (existing FB tests still green — Time default unchanged — plus the new `SYNC_NO` test).

- [ ] **Step 5: Run full suite**

Run: `uv run pytest -q`
Expected: `182 passed` (181 + new No test).

- [ ] **Step 6: Commit**
```bash
git add src/blocks/RuckigOtg.s7dcl tests/unit/test_ruckig_otg.py
git commit -m "v0.5 T3: RuckigOtg SYNC_NO branch + Time fallback wrapper"
```

---

## Task 4: `RuckigOtg` — `SYNC_PHASE` branch + fallback

**Files:**
- Modify: `src/blocks/RuckigOtg.s7dcl`
- Test: `tests/unit/test_ruckig_otg.py`

- [ ] **Step 1: Write the failing tests**

Append to `tests/unit/test_ruckig_otg.py`:
```python
def test_sync_phase_collinear_shared_timing(harness):
    """SYNC_PHASE on a collinear move: both axes share jerk-switch timing and
    reach their (ratio-scaled) targets together."""
    inp = default_input()
    inp["nDofs"] = 2
    inp["synchronization"] = 1  # SYNC_PHASE
    inp["enabled"] = [True, True, False, False]
    inp["targetPosition"] = [1.0, 3.0, 0.0, 0.0]
    inp["maxVelocity"] = [3.0] * 4
    inp["maxAcceleration"] = [5.0] * 4
    inp["maxJerk"] = [10.0] * 4

    p0 = p1 = 0.0
    for _cyc in range(2000):
        harness.set_inputs(enable=True, input=inp, cycleTime=0.010, reset=False)
        harness.execute()
        assert harness.get_output("error") is False
        out = harness.get_output("output")
        p0, p1 = out.newPosition[0], out.newPosition[1]
        if harness.get_output("done"):
            break
    assert harness.get_output("done") is True
    assert p0 == pytest.approx(1.0, abs=1e-3)
    assert p1 == pytest.approx(3.0, abs=1e-3)


def test_sync_phase_non_collinear_falls_back(harness):
    """SYNC_PHASE on a non-collinear move falls back to Time: both still reach
    target, no error (the fallback path is the v0.4 Time machinery)."""
    inp = default_input()
    inp["nDofs"] = 2
    inp["synchronization"] = 1  # SYNC_PHASE -> not collinear -> Time
    inp["enabled"] = [True, True, False, False]
    inp["currentVelocity"] = [1.0, -1.0, 0.0, 0.0]  # not collinear with pd=[1,3]
    inp["targetPosition"] = [1.0, 3.0, 0.0, 0.0]
    inp["maxVelocity"] = [3.0] * 4
    inp["maxAcceleration"] = [5.0] * 4
    inp["maxJerk"] = [10.0] * 4

    p0 = p1 = 0.0
    for _cyc in range(2000):
        harness.set_inputs(enable=True, input=inp, cycleTime=0.010, reset=False)
        harness.execute()
        assert harness.get_output("error") is False
        out = harness.get_output("output")
        p0, p1 = out.newPosition[0], out.newPosition[1]
        if harness.get_output("done"):
            break
    assert harness.get_output("done") is True
    assert p0 == pytest.approx(1.0, abs=1e-3)
    assert p1 == pytest.approx(3.0, abs=1e-3)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `uv run pytest tests/unit/test_ruckig_otg.py::test_sync_phase_collinear_shared_timing -v`
Expected: FAIL — `synchronization = 1` is not yet handled; the FB runs the Time path (axes still reach target, but) — actually with no Phase branch the `IF NOT #syncHandled` Time path runs and the move DOES reach target. To make the test meaningful it asserts behaviour that Time also satisfies (reach target); the **distinguishing** assertion lives in the unit test of Task 2. Re-run after Step 3 to confirm the Phase branch is exercised (see Step 4 note).

> Note: both Phase and its Time fallback reach the target, so these FB tests guard "no regression / no error under SYNC_PHASE". The phase-specific shared-timing behaviour is asserted in `test_phase_synchronize.py` (Task 2). Keep both.

- [ ] **Step 3: Add the SYNC_PHASE branch**

In `src/blocks/RuckigOtg.s7dcl`, inside `IF #multiPath THEN`, between the `SYNC_NO` branch and the `IF NOT #syncHandled THEN` Time wrapper, insert:
```
                    // ---- SYNC_PHASE: try phase sync, else fall through to Time --
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
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/unit/test_ruckig_otg.py -q`
Expected: PASS. To confirm the Phase branch is actually taken (not silently Time), temporarily check `trajectory` in a scratch run: the collinear case must have `profiles[0].t == profiles[1].t` (shared timing). This is already asserted in `test_phase_synchronize.py`.

- [ ] **Step 5: Run full suite**

Run: `uv run pytest -q`
Expected: `184 passed` (182 + 2 phase FB tests).

- [ ] **Step 6: Commit**
```bash
git add src/blocks/RuckigOtg.s7dcl tests/unit/test_ruckig_otg.py
git commit -m "v0.5 T4: RuckigOtg SYNC_PHASE branch + Phase->Time fallback"
```

---

## Task 5: Parity — thread `synchronization` + v05 scenarios

**Files:**
- Modify: `tests/parity/runner_ref.py`, `tests/parity/runner_scl.py`, `tests/parity/test_parity.py`
- Create: `tests/parity/scenarios/v05_01_2dof_no_sync.yaml`, `v05_02_2dof_phase_collinear.yaml`, `v05_03_2dof_phase_fallback.yaml`

- [ ] **Step 1: Thread `synchronization` into `runner_ref`**

In `tests/parity/runner_ref.py`, add the parameter and enum mapping. After the imports, add:
```python
_SYNC_MAP = {0: ruckig.Synchronization.No, 1: ruckig.Synchronization.Phase,
             2: ruckig.Synchronization.Time}
```
Add `synchronization: int = 2,` to the keyword-only params of `run_reference` (next to `minimum_duration`). After `inp.max_jerk = ...`, add:
```python
    inp.synchronization = _SYNC_MAP[synchronization]
```

- [ ] **Step 2: Thread `synchronization` into `runner_scl`**

In `tests/parity/runner_scl.py`, add `synchronization: int = 2,` to the keyword-only params of `run_scl`, and change the dict entry to `"synchronization": synchronization,`.

- [ ] **Step 3: Thread `synchronization` into `test_parity`**

In `tests/parity/test_parity.py` `_kwargs_from_scenario`, add to the returned dict:
```python
        "synchronization": data.get("synchronization", 2),
```

- [ ] **Step 4: Create the v05 scenarios**

`tests/parity/scenarios/v05_01_2dof_no_sync.yaml`:
```yaml
name: v0.5 2-DoF no synchronization
# Synchronization.No: each axis runs at its own time-optimal duration. The short
# axis (1 m) finishes well before the long axis (8 m); neither is stretched.
n_dofs: 2
synchronization: 0
target_position: [1.0, 8.0]
max_velocity: [2.0, 2.0]
max_acceleration: [5.0, 5.0]
max_jerk: [10.0, 10.0]
cycle_time: 0.010
```

`tests/parity/scenarios/v05_02_2dof_phase_collinear.yaml`:
```yaml
name: v0.5 2-DoF phase synchronization (collinear)
# Synchronization.Phase on a collinear move (axis1 = 3 x axis0): all axes share
# the jerk-switch timing, jerk scaled by the displacement ratio -> straight-line
# motion. Exact match to Ruckig.
n_dofs: 2
synchronization: 1
target_position: [1.0, 3.0]
max_velocity: [3.0, 3.0]
max_acceleration: [5.0, 5.0]
max_jerk: [10.0, 10.0]
cycle_time: 0.010
```

`tests/parity/scenarios/v05_03_2dof_phase_fallback.yaml`:
```yaml
name: v0.5 2-DoF phase requested but non-collinear (Time fallback)
# Synchronization.Phase with a non-collinear initial velocity ([1, -1] vs
# pd=[1, 3]): phase sync is impossible, so both Ruckig and the SCL port fall back
# to Time synchronization. The two must match cycle by cycle.
n_dofs: 2
synchronization: 1
current_velocity: [1.0, -1.0]
target_position: [1.0, 3.0]
max_velocity: [3.0, 3.0]
max_acceleration: [5.0, 5.0]
max_jerk: [10.0, 10.0]
cycle_time: 0.010
```

- [ ] **Step 5: Run parity to verify pass**

Run: `uv run pytest tests/parity -q`
Expected: PASS — 30 scenarios (27 prior + 3 v05) at the floating-point floor; the 9 v04 scenarios (no `synchronization` key → default 2 = Time) are unchanged.

- [ ] **Step 6: Run full suite**

Run: `uv run pytest -q`
Expected: `187 passed` (184 + 3 parity).

- [ ] **Step 7: Commit**
```bash
git add tests/parity/runner_ref.py tests/parity/runner_scl.py tests/parity/test_parity.py tests/parity/scenarios/v05_01_2dof_no_sync.yaml tests/parity/scenarios/v05_02_2dof_phase_collinear.yaml tests/parity/scenarios/v05_03_2dof_phase_fallback.yaml
git commit -m "v0.5 T5: multi-DoF parity for No / Phase / Phase-fallback"
```

---

## Task 6: Release 0.5.0

**Files:**
- Modify: `CHANGELOG.md`, `README.md`, `pyproject.toml`

- [ ] **Step 1: Bump version**

In `pyproject.toml`, change `version = "0.4.0"` to `version = "0.5.0"`. (S7_Version of changed blocks: `PhaseSynchronize` is `0.5.0`; `RuckigOtg` bumps to `0.5.0`; `dbRuckigConst`/`typeRuckigInput` carry no S7_Version. Per-block versioning marks last-changed, so other blocks keep their versions.)

In `src/blocks/RuckigOtg.s7dcl` header, change `S7_Version := "0.4.0"` to `S7_Version := "0.5.0"`.

- [ ] **Step 2: CHANGELOG `[0.5.0]` section**

In `CHANGELOG.md`, add above `## [0.4.0]`:
```markdown
## [0.5.0] - 2026-06-03

Two additional multi-axis synchronization modes on top of v0.4's Time sync.

### Added
- `Synchronization.Phase` — when the per-DoF state deltas `(pd, v0, a0, vT, aT)`
  are collinear, every axis follows the limiting axis's jerk-switch timing with
  jerk scaled by the displacement ratio (straight-line motion in joint space).
  When not collinear, falls back to Time sync (exact Ruckig behaviour). New FC
  `PhaseSynchronize` (port of `is_input_collinear` + phase build).
- `Synchronization.No` — each axis runs at its own time-optimal duration,
  finishing independently; `trajectory.duration = max`.
- `dbRuckigConst.EPS_PHASE` (collinearity tolerance).

### Changed
- `RuckigOtg` dispatches on `input.synchronization`. The v0.4 Time path is
  unchanged and serves as the Phase fallback.
- `typeRuckigInput.synchronization` default → `SYNC_TIME` (matches Ruckig's
  default; preserves v0.4 behaviour for callers that do not set it).

### Parity
- 3 new scenarios (`v05_01..03`): No (independent), Phase collinear (exact),
  Phase non-collinear (Time fallback) — floating-point-floor match. 30 parity
  scenarios total. 187 tests pass.

### Known limitations
- `TimeIfNecessary`, `DurationDiscretization.Discrete`, and
  `per_dof_synchronization` are not implemented (v0.6).
- No brake pre-phase in the multi-axis path (v0.7); step1 ~3% gaps (v0.2).
```
And add the link line near the others:
```markdown
[0.5.0]: https://github.com/core-engineering/ruckig-scl/releases/tag/v0.5.0
```

- [ ] **Step 3: README**

In `README.md`: update the status line to v0.5.0 (multi-axis Phase + No sync); add a "Features (v0.5)" bullet group for Phase / No; add `PhaseSynchronize` to the architecture table; update the parity count to 30; in the roadmap, mark v0.5 done and move Phase/per-DoF/Discrete notes (Phase now done, per-DoF + Discrete remain v0.6).

- [ ] **Step 4: Run full suite**

Run: `uv run pytest -q`
Expected: `187 passed`.

- [ ] **Step 5: Commit, merge, tag**
```bash
git add CHANGELOG.md README.md pyproject.toml src/blocks/RuckigOtg.s7dcl
git commit -m "v0.5 T6: release 0.5.0 (docs + version)"
git checkout master
git merge --no-ff feature/v0.5-phase-sync -m "Merge feature/v0.5-phase-sync: v0.5.0 phase + no synchronization"
uv run pytest -q   # expect 187 passed on master
git tag -a v0.5.0 -m "v0.5.0 - phase + no multi-axis synchronization"
git branch -d feature/v0.5-phase-sync
```

---

## Self-review notes

- **Spec coverage:** Phase (collinear + fallback) → Tasks 2, 4, 5; No → Tasks 3, 5; default/back-compat → Task 1; testing/parity → Tasks 2–5; release → Task 6. All spec sections covered.
- **Type consistency:** `PhaseSynchronize` signature (`trajectory` VAR_IN_OUT; `chainPos/Vel/Acc`, `input`, `nDofs` VAR_INPUT; `phaseOk`, `duration` VAR_OUTPUT) is identical in its definition (Task 2) and its call site (Task 4). Constants `SYNC_NONE=0`, `SYNC_PHASE=1`, `SYNC_TIME=2` consistent across FB, tests, runners, scenarios.
- **No identifiers ending in `of`/`Dof`** (`refAx`, `scaleAx`). One `:=` per line throughout (notably the scale-quantity ELSIF chain and the jerk-scaling loop).
