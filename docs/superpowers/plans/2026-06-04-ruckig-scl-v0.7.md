# ruckig-scl v0.7 — Velocity Control Interface — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add Ruckig's third-order **velocity control interface** (`controlInterface = IFACE_VELOCITY`) to ruckig-scl, for 1–4 DoF, wired through the existing synchronization machinery.

**Architecture:** A self-contained velocity solver (Approach A) made of dedicated FCs — `CheckVelProfile`, `CollectVelStep1Dir` + `ComputeVelBlock1Axis` (Step 1 → `typeBlock`), `SolveVelTimedDir` + `ComputeVelProfileTimed` (Step 2 → re-time) — that reuse `typeProfile`/`typeBlock`/`IntegrateProfileStates`/`Synchronize`/`AdvanceTime`/`StateAtTime` unchanged. `RuckigOtg` gets a top-level `IF velocity` recompute branch that mirrors the multi-axis flow (resolve modes → Block per axis → `Synchronize` → re-time per axis with `ComputeVelProfileTimed`), so single-axis and all reused sync modes (Time/No/per-DoF/TimeIfNecessary/Discrete) work; `Phase` falls back to `Time`. Position is uncontrolled (parity), `maxVelocity` ignored, no brake pre-phase.

**Tech Stack:** Siemens SCL (`.s7dcl`) executed/transpiled via `siemens-plc-tools` `plc-code`; tests in Python with `pytest` and the Ruckig oracle (PyPI `ruckig` 0.17.3); `uv` for all Python invocation.

**Spec:** `docs/superpowers/specs/2026-06-04-ruckig-scl-v0.7-design.md`

**Oracle reference:** `velocity_third_step1.cpp`, `velocity_third_step2.cpp`, `profile.hpp` (`check_for_velocity`) under the cached sdist `~/.cache/uv/sdists-v9/pypi/ruckig/0.17.3/.../src/`.

---

## Design notes & refinements (read before starting)

1. **Refinement vs spec wording.** The spec described "3 in-place branch points" in `RuckigOtg`. The existing recompute flow is heavily position-specific (brake, step1/step2 split, position at-target zero-fill), so this plan instead adds a **cohesive top-level `IF controlInterface = IFACE_VELOCITY` branch** that runs a self-contained velocity recompute. Same FC set, same behaviour, lower regression risk on the position path. Two small **internal helper FCs** (`CollectVelStep1Dir`, `SolveVelTimedDir`) are added alongside the three public solver FCs — exactly the pattern the position solver already uses (`SolveDirection`, `SolveTimed*`).
2. **No velocity at-target zero-fill.** An axis already at its velocity target while still coasting (`vf ≠ 0`) must keep advancing in position. `ComputeVelProfileTimed` handles this natively via the oracle's trivial `time_none` case (`a0=af=vd=0 → t[1]=tf`, jerk 0 → constant-velocity coast). So the velocity branch does **not** zero-fill; it always calls `ComputeVelProfileTimed(tf)`. `tf = 0` (all axes already at target) yields an all-zero profile (duration 0) — handled.
3. **Every axis re-timed via Step 2.** For velocity, even a `No` axis is emitted with `ComputeVelProfileTimed(tf = block.tMin)` (Step 2 at `tf=tMin` reproduces the time-optimal profile — same property `typeBlock` relies on). Participating axes use `tf = tSync`. This avoids needing a separate single-axis velocity Step-1 emitter.
4. **Jerk pattern.** Velocity profiles use the standard UDDU jerk pattern with the direction's signed jerk `js`: `j = [js, 0, -js, 0, -js, 0, js]`. The `time_none` Step-2 UD solution uses the *recomputed* jerk `jf` in place of `js`. Only `t[0..2]` (and `t[6]` for the Step-2 UU solutions) are non-zero; the rest are 0.
5. **SCL/transpiler constraints** (see `docs/PLC_CODE_LIMITATIONS.md`): one `:=` per line; `END_IF`/`END_FOR` on their own line; no identifier ending in `of`/`Dof`; no division by a parenthesized product — precompute the denominator into a scalar temp first.

## File structure

| File | New/Modify | Responsibility |
|---|---|---|
| `src/data-blocks/dbRuckigConst.s7dcl` | Modify | add `RESULT_ERR_IFACE := 16#8208` |
| `src/blocks/CheckVelProfile.s7dcl` | Create | velocity profile validity (reaches `vf`/`af`, `|a|≤aMax`, `t≥0`) |
| `src/blocks/CollectVelStep1Dir.s7dcl` | Create | Step-1 helper: collect valid durations for one direction |
| `src/blocks/ComputeVelBlock1Axis.s7dcl` | Create | Step 1 → `typeBlock` (tMin + blocked intervals) |
| `src/blocks/SolveVelTimedDir.s7dcl` | Create | Step-2 helper: try all candidates for one direction |
| `src/blocks/ComputeVelProfileTimed.s7dcl` | Create | Step 2: re-time one axis to `tf` |
| `src/blocks/ValidateInput.s7dcl` | Modify | velocity-mode validation + bad-interface rejection |
| `src/blocks/RuckigOtg.s7dcl` | Modify | velocity recompute branch + `controlInterface` change detection |
| `tests/unit/test_check_vel_profile.py` | Create | unit tests for `CheckVelProfile` |
| `tests/unit/test_compute_vel_block_1axis.py` | Create | unit tests for `ComputeVelBlock1Axis` |
| `tests/unit/test_compute_vel_profile_timed.py` | Create | unit tests for `ComputeVelProfileTimed` |
| `tests/unit/test_validate_input.py` | Modify | velocity-mode validation tests |
| `tests/parity/runner_ref.py` | Modify | set `inp.control_interface` |
| `tests/parity/runner_scl.py` | Modify | set `controlInterface` |
| `tests/parity/test_parity.py` | Modify | map `control_interface` from scenario |
| `tests/parity/scenarios/v07_*.yaml` | Create | velocity parity scenarios |
| `CHANGELOG.md`, `README.md`, `pyproject.toml`, `uv.lock` | Modify | release 0.7.0 |

---

## Task 1: Add `RESULT_ERR_IFACE` constant

**Files:**
- Modify: `src/data-blocks/dbRuckigConst.s7dcl`

- [ ] **Step 1: Add the constant**

In `src/data-blocks/dbRuckigConst.s7dcl`, after the line `RESULT_ERR_SYNC : Word := 16#8603;`, add:

```scl
        RESULT_ERR_IFACE      : Word  := 16#8208;
```

(`16#82xx` = parameterization-error range, consistent with the other `RESULT_ERR_*` input errors.)

- [ ] **Step 2: Sanity-check the suite still imports**

Run: `cd /mnt/c/Users/martinc8/projects/ruckig-scl && uv run pytest tests/unit/test_validate_input.py -q`
Expected: PASS (no behavioural change yet).

- [ ] **Step 3: Commit**

```bash
git add src/data-blocks/dbRuckigConst.s7dcl
git commit -m "v0.7 T1: RESULT_ERR_IFACE constant (16#8208)"
```

---

## Task 2: `CheckVelProfile` FC

**Files:**
- Create: `src/blocks/CheckVelProfile.s7dcl`
- Test: `tests/unit/test_check_vel_profile.py`

Validates a candidate velocity profile: integrate from the boundary state, check
phase times ≥ 0, `|a| ≤ aMax`, and that the **velocity** target (`vf`, `af`) is
reached. Position is integrated (for output) but **not** checked. Optional `tf`
sum check for Step 2 (sentinel `tf < 0` disables it).

- [ ] **Step 1: Write the failing test**

Create `tests/unit/test_check_vel_profile.py`:

```python
"""Unit tests for CheckVelProfile FC."""
import pytest


@pytest.fixture
def harness(make_harness):
    return make_harness("CheckVelProfile.s7dcl")


def _profile(t, j):
    """Build a typeProfile dict from t/j (a/v/p get filled by integration)."""
    t = list(t) + [0.0] * (7 - len(t))
    j = list(j) + [0.0] * (7 - len(j))
    return {
        "t": t, "j": j,
        "a": [0.0] * 8, "v": [0.0] * 8, "p": [0.0] * 8,
        "direction": 0, "controlSigns": 0,
        "brake": {"t": [0.0, 0.0], "j": [0.0, 0.0],
                  "a": [0.0, 0.0, 0.0], "v": [0.0, 0.0, 0.0], "p": [0.0, 0.0, 0.0]},
    }


def _run(harness, profile, p0, v0, a0, vf, af, aMax, tf):
    harness.reset()
    harness.set_inputs(profile=profile, p0=p0, v0=v0, a0=a0,
                       vf=vf, af=af, aMax=aMax, tf=tf)
    harness.execute()
    return harness.get_output("CheckVelProfile")


def test_coast_at_target_is_valid(harness):
    # Already at vf=2, af=0: a constant-velocity coast of duration tf=1.0.
    prof = _profile([0.0, 1.0, 0.0], [0.0, 0.0, 0.0])
    assert _run(harness, prof, p0=0.0, v0=2.0, a0=0.0,
                vf=2.0, af=0.0, aMax=5.0, tf=1.0) is True


def test_negative_phase_time_rejected(harness):
    prof = _profile([-0.1, 1.0, 0.0], [0.0, 0.0, 0.0])
    assert _run(harness, prof, p0=0.0, v0=2.0, a0=0.0,
                vf=2.0, af=0.0, aMax=5.0, tf=-1.0) is False


def test_wrong_velocity_target_rejected(harness):
    # Coast keeps v=2.0 but we ask for vf=5.0 -> not reached.
    prof = _profile([0.0, 1.0, 0.0], [0.0, 0.0, 0.0])
    assert _run(harness, prof, p0=0.0, v0=2.0, a0=0.0,
                vf=5.0, af=0.0, aMax=5.0, tf=-1.0) is False


def test_duration_mismatch_rejected(harness):
    # Valid coast but tf check (1.0) disagrees with sum(t)=2.0.
    prof = _profile([0.0, 2.0, 0.0], [0.0, 0.0, 0.0])
    assert _run(harness, prof, p0=0.0, v0=2.0, a0=0.0,
                vf=2.0, af=0.0, aMax=5.0, tf=1.0) is False
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest tests/unit/test_check_vel_profile.py -q`
Expected: FAIL — block `CheckVelProfile` not found.

- [ ] **Step 3: Create the FC**

Create `src/blocks/CheckVelProfile.s7dcl`:

```scl
{
    S7_Author := "Camille Martin";
    S7_EditorMode := "SCL";
    S7_Family := "Ruckig";
    S7_Optimized := "TRUE";
    S7_Version := "0.7.0"
}
FUNCTION "CheckVelProfile" : Bool
    VAR_IN_OUT
        profile : _.typeProfile;
    END_VAR
    VAR_INPUT
        p0   : LReal;
        v0   : LReal;
        a0   : LReal;
        vf   : LReal;
        af   : LReal;
        aMax : LReal;
        // Step 2 imposed duration. Sentinel tf < 0 disables the duration check
        // (Step 1 / time-optimal callers pass tf := -1.0).
        tf   : LReal;
    END_VAR
    VAR_TEMP
        i    : DInt;
        sumT : LReal;
    END_VAR
    VAR CONSTANT
        EPS_TIME   : LReal := 1.0E-9;
        EPS_LIMIT  : LReal := 1.0E-7;
        EPS_TARGET : LReal := 1.0E-6;
    END_VAR

    { S7_Language := "SCL" }
    NETWORK
        REGION Block header
            //==========================================================
            // Title:      CheckVelProfile
            // Function:   Validate a candidate velocity-interface profile:
            //             phase times >= 0, |a| <= aMax, and reaching the
            //             velocity target (vf, af). Position is integrated for
            //             the output but NOT constrained (Ruckig velocity
            //             interface). Optional imposed-duration check (Step 2).
            // Family:     Ruckig
            // Author:     Camille Martin
            // Ref:        ruckig/include/ruckig/profile.hpp check_for_velocity
            //==========================================================
        END_REGION

        REGION Integrate states from initial condition
            "IntegrateProfileStates"(profile := #profile, p0 := #p0, v0 := #v0, a0 := #a0);
        END_REGION

        REGION Phase times non negative
            FOR #i := 0 TO 6 DO
                IF #profile.t[#i] < -#EPS_TIME THEN
                    #CheckVelProfile := false;
                    RETURN;
                END_IF;
            END_FOR;
        END_REGION

        REGION Acceleration limits (velocity interface: no vMax)
            FOR #i := 0 TO 7 DO
                IF ABS(#profile.a[#i]) > #aMax + #EPS_LIMIT THEN
                    #CheckVelProfile := false;
                    RETURN;
                END_IF;
            END_FOR;
        END_REGION

        REGION Reaches velocity target (position NOT checked)
            IF ABS(#profile.v[7] - #vf) > #EPS_TARGET THEN
                #CheckVelProfile := false;
                RETURN;
            END_IF;
            IF ABS(#profile.a[7] - #af) > #EPS_TARGET THEN
                #CheckVelProfile := false;
                RETURN;
            END_IF;
        END_REGION

        REGION Imposed duration (Step 2)
            IF #tf >= 0.0 THEN
                #sumT := 0.0;
                FOR #i := 0 TO 6 DO
                    #sumT := #sumT + #profile.t[#i];
                END_FOR;
                IF ABS(#sumT - #tf) > #EPS_TIME THEN
                    #CheckVelProfile := false;
                    RETURN;
                END_IF;
            END_IF;
        END_REGION

        #CheckVelProfile := true;
    END_NETWORK
END_FUNCTION
```

- [ ] **Step 4: Run test to verify it passes**

Run: `uv run pytest tests/unit/test_check_vel_profile.py -q`
Expected: PASS (4 tests).

- [ ] **Step 5: Commit**

```bash
git add src/blocks/CheckVelProfile.s7dcl tests/unit/test_check_vel_profile.py
git commit -m "v0.7 T2: CheckVelProfile (velocity profile validity)"
```

---

## Task 3: `CollectVelStep1Dir` + `ComputeVelBlock1Axis` (Step 1 → Block)

**Files:**
- Create: `src/blocks/CollectVelStep1Dir.s7dcl`
- Create: `src/blocks/ComputeVelBlock1Axis.s7dcl`
- Test: `tests/unit/test_compute_vel_block_1axis.py`

`CollectVelStep1Dir` tries the two velocity families (`time_acc0`, `time_none`
×2 solutions) for one signed direction, appending every valid total duration to
`allDur`. `ComputeVelBlock1Axis` runs it in both directions, sorts/dedups, and
assembles the `typeBlock` (no blocked interval when `af ≈ 0`).

- [ ] **Step 1: Write the failing test**

Create `tests/unit/test_compute_vel_block_1axis.py`:

```python
"""Unit tests for ComputeVelBlock1Axis FC (velocity Step 1)."""
import math

import pytest

RESULT_WORKING = 0x7000


@pytest.fixture
def harness(make_harness):
    return make_harness("ComputeVelBlock1Axis.s7dcl")


def _empty_profile():
    return {
        "t": [0.0] * 7, "j": [0.0] * 7,
        "a": [0.0] * 8, "v": [0.0] * 8, "p": [0.0] * 8,
        "direction": 0, "controlSigns": 0,
        "brake": {"t": [0.0, 0.0], "j": [0.0, 0.0],
                  "a": [0.0, 0.0, 0.0], "v": [0.0, 0.0, 0.0], "p": [0.0, 0.0, 0.0]},
    }


def _empty_block():
    return {"tMin": 0.0, "found": False,
            "aValid": False, "aLeft": 0.0, "aRight": 0.0,
            "bValid": False, "bLeft": 0.0, "bRight": 0.0}


def _run(harness, *, v0, a0, vf, af, aMax, jMax):
    harness.reset()
    harness.set_inputs(block=_empty_block(), profile=_empty_profile(),
                       p0=0.0, v0=v0, a0=a0, vf=vf, af=af, aMax=aMax, jMax=jMax)
    harness.execute()
    return (harness.get_output("ComputeVelBlock1Axis"),
            harness.get_output("block"))


def test_zero_to_one_triangular(harness):
    # v0=0,a0=0 -> vf=1,af=0 with aMax high (no plateau): time_none.
    # h1 = sqrt(jMax*vd) = sqrt(4*1) = 2; tMin = 2*h1/jMax = 2*2/4 = 1.0
    res, block = _run(harness, v0=0.0, a0=0.0, vf=1.0, af=0.0, aMax=10.0, jMax=4.0)
    assert res == RESULT_WORKING
    assert block["tMin"] == pytest.approx(1.0, abs=1e-9)
    assert block["aValid"] is False  # af == 0 -> no blocked interval


def test_at_target_zero_duration(harness):
    res, block = _run(harness, v0=2.0, a0=0.0, vf=2.0, af=0.0, aMax=10.0, jMax=4.0)
    assert res == RESULT_WORKING
    assert block["tMin"] == pytest.approx(0.0, abs=1e-9)


def test_negative_vd(harness):
    # symmetric: vf below v0 -> same tMin as the positive case
    res, block = _run(harness, v0=1.0, a0=0.0, vf=0.0, af=0.0, aMax=10.0, jMax=4.0)
    assert res == RESULT_WORKING
    assert block["tMin"] == pytest.approx(1.0, abs=1e-9)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest tests/unit/test_compute_vel_block_1axis.py -q`
Expected: FAIL — block not found.

- [ ] **Step 3a: Create the direction helper `CollectVelStep1Dir`**

Create `src/blocks/CollectVelStep1Dir.s7dcl`:

```scl
{
    S7_Author := "Camille Martin";
    S7_EditorMode := "SCL";
    S7_Family := "Ruckig";
    S7_Optimized := "TRUE";
    S7_Version := "0.7.0"
}
FUNCTION "CollectVelStep1Dir" : Void
    VAR_IN_OUT
        profile  : _.typeProfile;
        allDur   : Array[0..7] of LReal;
        allCount : Int;
    END_VAR
    VAR_INPUT
        p0    : LReal;
        v0    : LReal;
        a0    : LReal;
        vf    : LReal;
        af    : LReal;
        saMax : LReal;   // signed plateau acceleration (+aMax or -aMax)
        sjMax : LReal;   // signed jerk (+jMax or -jMax)
    END_VAR
    VAR_TEMP
        vd    : LReal;
        denom : LReal;
        h1sq  : LReal;
        h1    : LReal;
        t0    : LReal;
        t1    : LReal;
        t2    : LReal;
        aMag  : LReal;
        i     : DInt;
    END_VAR

    { S7_Language := "SCL" }
    NETWORK
        REGION Block header
            //==========================================================
            // Title:      CollectVelStep1Dir
            // Function:   Velocity Step-1 helper. Tries time_acc0 and the two
            //             time_none solutions for ONE signed direction; appends
            //             every CheckVelProfile-valid total duration to allDur.
            // Family:     Ruckig
            // Author:     Camille Martin
            // Ref:        ruckig/src/ruckig/velocity_third_step1.cpp
            //==========================================================
        END_REGION

        REGION Setup
            #vd   := #vf - #v0;
            #aMag := ABS(#saMax);
            // clear unused phases (set once; families only write t[0..2])
            FOR #i := 3 TO 6 DO
                #profile.t[#i] := 0.0;
            END_FOR;
            // UDDU jerk pattern with the signed jerk
            #profile.j[0] := #sjMax;
            #profile.j[1] := 0.0;
            #profile.j[2] := -#sjMax;
            #profile.j[3] := 0.0;
            #profile.j[4] := -#sjMax;
            #profile.j[5] := 0.0;
            #profile.j[6] := #sjMax;
        END_REGION

        REGION time_acc0 (reaches the aMax plateau)
            #denom := 2.0 * #saMax * #sjMax;
            #t0 := (-#a0 + #saMax) / #sjMax;
            #t1 := (#a0 * #a0 + #af * #af) / #denom - #saMax / #sjMax + #vd / #saMax;
            #t2 := (-#af + #saMax) / #sjMax;
            #profile.t[0] := #t0;
            #profile.t[1] := #t1;
            #profile.t[2] := #t2;
            IF "CheckVelProfile"(profile := #profile, p0 := #p0, v0 := #v0, a0 := #a0,
                    vf := #vf, af := #af, aMax := #aMag, tf := -1.0) THEN
                #allDur[#allCount] := #t0 + #t1 + #t2;
                #allCount := #allCount + 1;
            END_IF;
        END_REGION

        REGION time_none (no plateau; two solutions)
            #h1sq := (#a0 * #a0 + #af * #af) / 2.0 + #sjMax * #vd;
            IF #h1sq >= 0.0 THEN
                #h1 := SQRT(#h1sq);

                // Solution 1
                #t0 := -(#a0 + #h1) / #sjMax;
                #t1 := 0.0;
                #t2 := -(#af + #h1) / #sjMax;
                #profile.t[0] := #t0;
                #profile.t[1] := #t1;
                #profile.t[2] := #t2;
                IF "CheckVelProfile"(profile := #profile, p0 := #p0, v0 := #v0, a0 := #a0,
                        vf := #vf, af := #af, aMax := #aMag, tf := -1.0) THEN
                    #allDur[#allCount] := #t0 + #t1 + #t2;
                    #allCount := #allCount + 1;
                END_IF;

                // Solution 2
                #t0 := (-#a0 + #h1) / #sjMax;
                #t1 := 0.0;
                #t2 := (-#af + #h1) / #sjMax;
                #profile.t[0] := #t0;
                #profile.t[1] := #t1;
                #profile.t[2] := #t2;
                IF "CheckVelProfile"(profile := #profile, p0 := #p0, v0 := #v0, a0 := #a0,
                        vf := #vf, af := #af, aMax := #aMag, tf := -1.0) THEN
                    #allDur[#allCount] := #t0 + #t1 + #t2;
                    #allCount := #allCount + 1;
                END_IF;
            END_IF;
        END_REGION
    END_NETWORK
END_FUNCTION
```

- [ ] **Step 3b: Create `ComputeVelBlock1Axis`**

Create `src/blocks/ComputeVelBlock1Axis.s7dcl`:

```scl
{
    S7_Author := "Camille Martin";
    S7_EditorMode := "SCL";
    S7_Family := "Ruckig";
    S7_Optimized := "TRUE";
    S7_Version := "0.7.0"
}
FUNCTION "ComputeVelBlock1Axis" : Word
    VAR_IN_OUT
        block   : _.typeBlock;
        profile : _.typeProfile;
    END_VAR
    VAR_INPUT
        p0   : LReal;
        v0   : LReal;
        a0   : LReal;
        vf   : LReal;
        af   : LReal;
        aMax : LReal;
        jMax : LReal;
    END_VAR
    VAR_TEMP
        allDur   : Array[0..7] of LReal;
        allCount : Int;
        nd       : Int;
        i        : Int;
        k        : Int;
        tmp      : LReal;
    END_VAR

    { S7_Language := "SCL" }
    NETWORK
        REGION Block header
            //==========================================================
            // Title:      ComputeVelBlock1Axis
            // Function:   Velocity Step-1 as a reachable-duration Block. Runs
            //             CollectVelStep1Dir in both directions, sorts/dedups the
            //             valid durations, then derives tMin + blocked intervals
            //             (no interval when af ~ 0, per Ruckig). Foundation for
            //             velocity-interface multi-axis synchronization.
            // Family:     Ruckig
            // Author:     Camille Martin
            // Ref:        ruckig/src/ruckig/velocity_third_step1.cpp
            //==========================================================
        END_REGION

        REGION Already-at-velocity-target degenerate guard
            IF ABS(#vf - #v0) < "dbRuckigConst".EPS_VELOCITY
                AND ABS(#af - #a0) < "dbRuckigConst".EPS_DERIV THEN
                #block.tMin   := 0.0;
                #block.found  := true;
                #block.aValid := false;
                #block.bValid := false;
                #ComputeVelBlock1Axis := "dbRuckigConst".RESULT_WORKING;
                RETURN;
            END_IF;
        END_REGION

        REGION Collect valid durations (both directions)
            #allCount := 0;
            "CollectVelStep1Dir"(profile := #profile, allDur := #allDur, allCount := #allCount,
                p0 := #p0, v0 := #v0, a0 := #a0, vf := #vf, af := #af,
                saMax := #aMax, sjMax := #jMax);
            "CollectVelStep1Dir"(profile := #profile, allDur := #allDur, allCount := #allCount,
                p0 := #p0, v0 := #v0, a0 := #a0, vf := #vf, af := #af,
                saMax := -#aMax, sjMax := -#jMax);
        END_REGION

        REGION Fail if no profile
            IF #allCount = 0 THEN
                #block.found := false;
                #ComputeVelBlock1Axis := "dbRuckigConst".RESULT_ERR_SOLVER;
                RETURN;
            END_IF;
        END_REGION

        REGION Sort ascending (bubble)
            FOR #i := 0 TO #allCount - 2 DO
                FOR #k := 0 TO #allCount - 2 - #i DO
                    IF #allDur[#k] > #allDur[#k + 1] THEN
                        #tmp := #allDur[#k];
                        #allDur[#k] := #allDur[#k + 1];
                        #allDur[#k + 1] := #tmp;
                    END_IF;
                END_FOR;
            END_FOR;
        END_REGION

        REGION Deduplicate -> nd distinct durations
            #nd := 0;
            FOR #i := 0 TO #allCount - 1 DO
                IF #nd = 0 THEN
                    #allDur[0] := #allDur[#i];
                    #nd := 1;
                ELSIF #allDur[#i] - #allDur[#nd - 1] > 1.0E-7 THEN
                    #allDur[#nd] := #allDur[#i];
                    #nd := #nd + 1;
                END_IF;
            END_FOR;
        END_REGION

        REGION Assemble Block (no interval when af ~ 0)
            #block.found  := true;
            #block.tMin   := #allDur[0];
            #block.aValid := false;
            #block.bValid := false;
            IF ABS(#af) >= "dbRuckigConst".EPS_DERIV THEN
                IF #nd = 2 THEN
                    #block.aValid := true;
                    #block.aLeft  := #allDur[0];
                    #block.aRight := #allDur[1];
                ELSIF #nd = 3 THEN
                    #block.aValid := true;
                    #block.aLeft  := #allDur[1];
                    #block.aRight := #allDur[2];
                ELSIF #nd = 5 THEN
                    #block.aValid := true;
                    #block.aLeft  := #allDur[1];
                    #block.aRight := #allDur[2];
                    #block.bValid := true;
                    #block.bLeft  := #allDur[3];
                    #block.bRight := #allDur[4];
                END_IF;
            END_IF;
            #ComputeVelBlock1Axis := "dbRuckigConst".RESULT_WORKING;
        END_REGION
    END_NETWORK
END_FUNCTION
```

- [ ] **Step 4: Run test to verify it passes**

Run: `uv run pytest tests/unit/test_compute_vel_block_1axis.py -q`
Expected: PASS (3 tests). If `tMin` is off, dump the generated Python (the
implementer may need to confirm the `time_none` solution selection); the
expected `tMin = 1.0` for `jMax=4, vd=1` is analytic.

- [ ] **Step 5: Commit**

```bash
git add src/blocks/CollectVelStep1Dir.s7dcl src/blocks/ComputeVelBlock1Axis.s7dcl tests/unit/test_compute_vel_block_1axis.py
git commit -m "v0.7 T3: ComputeVelBlock1Axis + CollectVelStep1Dir (velocity Step 1 -> Block)"
```

---

## Task 4: `SolveVelTimedDir` + `ComputeVelProfileTimed` (Step 2 → re-time)

**Files:**
- Create: `src/blocks/SolveVelTimedDir.s7dcl`
- Create: `src/blocks/ComputeVelProfileTimed.s7dcl`
- Test: `tests/unit/test_compute_vel_profile_timed.py`

`SolveVelTimedDir` tries, for one signed direction and a self-skip on `found`,
the three `time_acc0` candidates (UD, UU, UU-2-step) and the two `time_none`
cases (trivial coast, UD with recomputed jerk), validating each with
`CheckVelProfile(tf)`. `ComputeVelProfileTimed` calls it in both directions
(sign of `vd` first) and integrates the winner.

- [ ] **Step 1: Write the failing test**

Create `tests/unit/test_compute_vel_profile_timed.py`:

```python
"""Unit tests for ComputeVelProfileTimed FC (velocity Step 2)."""
import pytest

RESULT_WORKING = 0x7000


@pytest.fixture
def harness(make_harness):
    return make_harness("ComputeVelProfileTimed.s7dcl")


def _empty_profile():
    return {
        "t": [0.0] * 7, "j": [0.0] * 7,
        "a": [0.0] * 8, "v": [0.0] * 8, "p": [0.0] * 8,
        "direction": 0, "controlSigns": 0,
        "brake": {"t": [0.0, 0.0], "j": [0.0, 0.0],
                  "a": [0.0, 0.0, 0.0], "v": [0.0, 0.0, 0.0], "p": [0.0, 0.0, 0.0]},
    }


def _run(harness, *, v0, a0, vf, af, aMax, jMax, tf):
    prof = _empty_profile()
    harness.reset()
    harness.set_inputs(profile=prof, p0=0.0, v0=v0, a0=a0,
                       vf=vf, af=af, aMax=aMax, jMax=jMax, tf=tf)
    harness.execute()
    return (harness.get_output("ComputeVelProfileTimed"),
            harness.get_output("profile"))


def _sum_t(prof):
    return sum(prof["t"][i] for i in range(7))


def test_retime_reaches_target_at_tf(harness):
    # Time-optimal tMin for vd=1, jMax=4 is 1.0; re-time to tf=2.0.
    res, prof = _run(harness, v0=0.0, a0=0.0, vf=1.0, af=0.0, aMax=10.0, jMax=4.0, tf=2.0)
    assert res == RESULT_WORKING
    assert _sum_t(prof) == pytest.approx(2.0, abs=1e-9)
    assert prof["v"][7] == pytest.approx(1.0, abs=1e-6)
    assert prof["a"][7] == pytest.approx(0.0, abs=1e-6)


def test_coast_when_already_at_target(harness):
    # v0=vf=2, a0=af=0: trivial time_none coast for the whole tf.
    res, prof = _run(harness, v0=2.0, a0=0.0, vf=2.0, af=0.0, aMax=10.0, jMax=4.0, tf=1.5)
    assert res == RESULT_WORKING
    assert _sum_t(prof) == pytest.approx(1.5, abs=1e-9)
    assert prof["v"][7] == pytest.approx(2.0, abs=1e-6)


def test_nonzero_af(harness):
    res, prof = _run(harness, v0=0.0, a0=0.0, vf=1.0, af=1.0, aMax=10.0, jMax=4.0, tf=1.5)
    assert res == RESULT_WORKING
    assert _sum_t(prof) == pytest.approx(1.5, abs=1e-9)
    assert prof["v"][7] == pytest.approx(1.0, abs=1e-6)
    assert prof["a"][7] == pytest.approx(1.0, abs=1e-6)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest tests/unit/test_compute_vel_profile_timed.py -q`
Expected: FAIL — block not found.

- [ ] **Step 3a: Create the direction helper `SolveVelTimedDir`**

Create `src/blocks/SolveVelTimedDir.s7dcl`:

```scl
{
    S7_Author := "Camille Martin";
    S7_EditorMode := "SCL";
    S7_Family := "Ruckig";
    S7_Optimized := "TRUE";
    S7_Version := "0.7.0"
}
FUNCTION "SolveVelTimedDir" : Void
    VAR_IN_OUT
        profile : _.typeProfile;
        found   : Bool;
    END_VAR
    VAR_INPUT
        p0    : LReal;
        v0    : LReal;
        a0    : LReal;
        vf    : LReal;
        af    : LReal;
        saMax : LReal;   // signed plateau acceleration
        sjMax : LReal;   // signed jerk
        tf    : LReal;
    END_VAR
    VAR_TEMP
        vd    : LReal;
        ad    : LReal;
        aMag  : LReal;
        jMag  : LReal;
        h1    : LReal;
        radic : LReal;
        denom : LReal;
        sj2   : LReal;
        jf    : LReal;
        t0    : LReal;
        t1    : LReal;
        t2    : LReal;
        t6    : LReal;
        i     : DInt;
    END_VAR
    VAR CONSTANT
        EPS : LReal := 1.0E-12;
    END_VAR

    { S7_Language := "SCL" }
    NETWORK
        REGION Block header
            //==========================================================
            // Title:      SolveVelTimedDir
            // Function:   Velocity Step-2 helper for ONE signed direction.
            //             Self-skips when found. Tries time_acc0 (UD, UU, UU-2-
            //             step) then time_none (trivial coast, UD recomputed jerk),
            //             validating each with CheckVelProfile(tf). Sets the UDDU
            //             jerk pattern of the winning candidate.
            // Family:     Ruckig
            // Author:     Camille Martin
            // Ref:        ruckig/src/ruckig/velocity_third_step2.cpp
            //==========================================================
        END_REGION

        REGION Skip if a previous direction already solved
            IF #found THEN
                RETURN;
            END_IF;
        END_REGION

        REGION Setup
            #vd   := #vf - #v0;
            #ad   := #af - #a0;
            #aMag := ABS(#saMax);
            #jMag := ABS(#sjMax);
            // default UDDU jerk pattern (signed). time_none UD overrides with jf.
            #profile.j[0] := #sjMax;
            #profile.j[1] := 0.0;
            #profile.j[2] := -#sjMax;
            #profile.j[3] := 0.0;
            #profile.j[4] := -#sjMax;
            #profile.j[5] := 0.0;
            #profile.j[6] := #sjMax;
        END_REGION

        REGION time_acc0 - UD solution
            #sj2 := #sjMax * #sjMax;
            #radic := (-#ad * #ad + 2.0 * #sjMax * ((#a0 + #af) * #tf - 2.0 * #vd)) / #sj2 + #tf * #tf;
            IF #radic >= 0.0 THEN
                #h1 := SQRT(#radic);
                #t0 := #ad / (2.0 * #sjMax) + (#tf - #h1) / 2.0;
                #t1 := #h1;
                #t2 := #tf - (#t0 + #h1);
                #profile.t[0] := #t0;
                #profile.t[1] := #t1;
                #profile.t[2] := #t2;
                #profile.t[3] := 0.0;
                #profile.t[4] := 0.0;
                #profile.t[5] := 0.0;
                #profile.t[6] := 0.0;
                IF "CheckVelProfile"(profile := #profile, p0 := #p0, v0 := #v0, a0 := #a0,
                        vf := #vf, af := #af, aMax := #aMag, tf := #tf) THEN
                    #found := true;
                    RETURN;
                END_IF;
            END_IF;
        END_REGION

        REGION time_acc0 - UU solution
            #denom := -#ad + #sjMax * #tf;
            IF ABS(#denom) > #EPS THEN
                #t0 := -#ad * #ad / (2.0 * #sjMax * #denom) + (#vd - #a0 * #tf) / #denom;
                #t1 := -#ad / #sjMax + #tf;
                #t2 := 0.0;
                #t6 := #tf - (#t0 + #t1);
                #profile.t[0] := #t0;
                #profile.t[1] := #t1;
                #profile.t[2] := 0.0;
                #profile.t[3] := 0.0;
                #profile.t[4] := 0.0;
                #profile.t[5] := 0.0;
                #profile.t[6] := #t6;
                IF "CheckVelProfile"(profile := #profile, p0 := #p0, v0 := #v0, a0 := #a0,
                        vf := #vf, af := #af, aMax := #aMag, tf := #tf) THEN
                    #found := true;
                    RETURN;
                END_IF;
            END_IF;
        END_REGION

        REGION time_acc0 - UU 2-step solution
            #t1 := -#ad / #sjMax + #tf;
            #t6 := #ad / #sjMax;
            #profile.t[0] := 0.0;
            #profile.t[1] := #t1;
            #profile.t[2] := 0.0;
            #profile.t[3] := 0.0;
            #profile.t[4] := 0.0;
            #profile.t[5] := 0.0;
            #profile.t[6] := #t6;
            IF "CheckVelProfile"(profile := #profile, p0 := #p0, v0 := #v0, a0 := #a0,
                    vf := #vf, af := #af, aMax := #aMag, tf := #tf) THEN
                #found := true;
                RETURN;
            END_IF;
        END_REGION

        REGION time_none - trivial coast (a0 = af = vd = 0)
            IF ABS(#a0) < #EPS AND ABS(#af) < #EPS AND ABS(#vd) < #EPS THEN
                #profile.t[0] := 0.0;
                #profile.t[1] := #tf;
                #profile.t[2] := 0.0;
                #profile.t[3] := 0.0;
                #profile.t[4] := 0.0;
                #profile.t[5] := 0.0;
                #profile.t[6] := 0.0;
                #profile.j[0] := 0.0;
                #profile.j[2] := 0.0;
                #profile.j[4] := 0.0;
                #profile.j[6] := 0.0;
                IF "CheckVelProfile"(profile := #profile, p0 := #p0, v0 := #v0, a0 := #a0,
                        vf := #vf, af := #af, aMax := #aMag, tf := #tf) THEN
                    #found := true;
                    RETURN;
                END_IF;
                // restore signed jerk pattern for the next candidate
                #profile.j[0] := #sjMax;
                #profile.j[2] := -#sjMax;
                #profile.j[4] := -#sjMax;
                #profile.j[6] := #sjMax;
            END_IF;
        END_REGION

        REGION time_none - UD solution (recomputed jerk jf)
            #h1 := 2.0 * (#af * #tf - #vd);
            IF ABS(#ad) > #EPS AND ABS(#h1) > #EPS THEN
                #t0 := #h1 / #ad;
                #t1 := #tf - #t0;
                #jf := #ad * #ad / #h1;
                IF ABS(#jf) < #jMag + 1.0E-12 THEN
                    #profile.t[0] := #t0;
                    #profile.t[1] := #t1;
                    #profile.t[2] := 0.0;
                    #profile.t[3] := 0.0;
                    #profile.t[4] := 0.0;
                    #profile.t[5] := 0.0;
                    #profile.t[6] := 0.0;
                    #profile.j[0] := #jf;
                    #profile.j[2] := -#jf;
                    #profile.j[4] := -#jf;
                    #profile.j[6] := #jf;
                    IF "CheckVelProfile"(profile := #profile, p0 := #p0, v0 := #v0, a0 := #a0,
                            vf := #vf, af := #af, aMax := #aMag, tf := #tf) THEN
                        #found := true;
                        RETURN;
                    END_IF;
                END_IF;
            END_IF;
        END_REGION
    END_NETWORK
END_FUNCTION
```

- [ ] **Step 3b: Create `ComputeVelProfileTimed`**

Create `src/blocks/ComputeVelProfileTimed.s7dcl`:

```scl
{
    S7_Author := "Camille Martin";
    S7_EditorMode := "SCL";
    S7_Family := "Ruckig";
    S7_Optimized := "TRUE";
    S7_Version := "0.7.0"
}
FUNCTION "ComputeVelProfileTimed" : Word
    VAR_IN_OUT
        profile : _.typeProfile;
    END_VAR
    VAR_INPUT
        p0   : LReal;
        v0   : LReal;
        a0   : LReal;
        vf   : LReal;
        af   : LReal;
        aMax : LReal;
        jMax : LReal;
        tf   : LReal;
    END_VAR
    VAR_TEMP
        found : Bool;
        vd    : LReal;
    END_VAR

    { S7_Language := "SCL" }
    NETWORK
        REGION Block header
            //==========================================================
            // Title:      ComputeVelProfileTimed
            // Function:   Single-DoF velocity Step-2 solver. Re-times the
            //             velocity move (v0,a0)->(vf,af) to an imposed duration
            //             tf via SolveVelTimedDir in both directions (sign of vd
            //             first), then integrates the winner. tf = block.tMin
            //             reproduces the time-optimal profile.
            // Note:       Identifier ends in "Timed" (no trailing "of"/"Dof":
            //             plc-code lexer mis-tokenises a trailing "of").
            // Family:     Ruckig
            // Author:     Camille Martin
            // Ref:        ruckig/src/ruckig/velocity_third_step2.cpp
            //==========================================================
        END_REGION

        REGION Try both directions (sign of vd first)
            #found := false;
            #vd := #vf - #v0;
            IF #vd > 0.0 THEN
                "SolveVelTimedDir"(profile := #profile, found := #found, p0 := #p0, v0 := #v0, a0 := #a0,
                    vf := #vf, af := #af, saMax := #aMax, sjMax := #jMax, tf := #tf);
                "SolveVelTimedDir"(profile := #profile, found := #found, p0 := #p0, v0 := #v0, a0 := #a0,
                    vf := #vf, af := #af, saMax := -#aMax, sjMax := -#jMax, tf := #tf);
            ELSE
                "SolveVelTimedDir"(profile := #profile, found := #found, p0 := #p0, v0 := #v0, a0 := #a0,
                    vf := #vf, af := #af, saMax := -#aMax, sjMax := -#jMax, tf := #tf);
                "SolveVelTimedDir"(profile := #profile, found := #found, p0 := #p0, v0 := #v0, a0 := #a0,
                    vf := #vf, af := #af, saMax := #aMax, sjMax := #jMax, tf := #tf);
            END_IF;
        END_REGION

        REGION Finalise
            IF #found THEN
                "IntegrateProfileStates"(profile := #profile, p0 := #p0, v0 := #v0, a0 := #a0);
                #ComputeVelProfileTimed := "dbRuckigConst".RESULT_WORKING;
            ELSE
                #ComputeVelProfileTimed := "dbRuckigConst".RESULT_ERR_SOLVER;
            END_IF;
        END_REGION
    END_NETWORK
END_FUNCTION
```

- [ ] **Step 4: Run test to verify it passes**

Run: `uv run pytest tests/unit/test_compute_vel_profile_timed.py -q`
Expected: PASS (3 tests).

- [ ] **Step 5: Commit**

```bash
git add src/blocks/SolveVelTimedDir.s7dcl src/blocks/ComputeVelProfileTimed.s7dcl tests/unit/test_compute_vel_profile_timed.py
git commit -m "v0.7 T4: ComputeVelProfileTimed + SolveVelTimedDir (velocity Step 2 re-time)"
```

---

## Task 5: `ValidateInput` velocity-mode branch

**Files:**
- Modify: `src/blocks/ValidateInput.s7dcl`
- Test: `tests/unit/test_validate_input.py`

In velocity mode: require `aMax > 0` and `jMax > 0`; do **not** require
`vMax > 0`; finiteness on `currentPosition`, current/target velocity &
acceleration, but **not** on `targetPosition`. Reject `controlInterface ∉ {0,1}`.

- [ ] **Step 1: Write the failing tests**

Append to `tests/unit/test_validate_input.py` (add the status constant near the
others, then the tests):

```python
RESULT_ERR_IFACE = 0x8208


def test_velocity_mode_accepts_zero_vmax(harness):
    inp = make_valid_input()
    inp["controlInterface"] = 1
    inp["maxVelocity"] = [0.0, 0.0, 0.0, 0.0]   # ignored in velocity mode
    inp["targetVelocity"] = [1.0, 0.0, 0.0, 0.0]
    assert _call(harness, inp) == RESULT_WORKING


def test_velocity_mode_ignores_nonfinite_target_position(harness):
    inp = make_valid_input()
    inp["controlInterface"] = 1
    inp["targetPosition"] = [math.inf, 0.0, 0.0, 0.0]   # ignored in velocity mode
    assert _call(harness, inp) == RESULT_WORKING


def test_velocity_mode_requires_amax(harness):
    inp = make_valid_input()
    inp["controlInterface"] = 1
    inp["maxAcceleration"] = [0.0, 5.0, 5.0, 5.0]
    assert _call(harness, inp) == RESULT_ERR_AMAX


def test_velocity_mode_requires_jmax(harness):
    inp = make_valid_input()
    inp["controlInterface"] = 1
    inp["maxJerk"] = [0.0, 10.0, 10.0, 10.0]
    assert _call(harness, inp) == RESULT_ERR_JMAX


def test_velocity_mode_rejects_nonfinite_target_velocity(harness):
    inp = make_valid_input()
    inp["controlInterface"] = 1
    inp["targetVelocity"] = [math.nan, 0.0, 0.0, 0.0]
    assert _call(harness, inp) == RESULT_ERR_NON_FINITE


def test_invalid_control_interface_rejected(harness):
    inp = make_valid_input()
    inp["controlInterface"] = 7
    assert _call(harness, inp) == RESULT_ERR_IFACE
```

- [ ] **Step 2: Run to verify they fail**

Run: `uv run pytest tests/unit/test_validate_input.py -q`
Expected: FAIL (velocity branch not implemented; bad interface accepted).

- [ ] **Step 3: Implement the branch**

In `src/blocks/ValidateInput.s7dcl`, replace the `REGION Per-axis validation`
block (the `FOR #i := 0 TO #input.nDofs - 1 DO ... END_FOR;` and its content,
between `REGION Per-axis validation` and its `END_REGION`) with:

```scl
        REGION controlInterface range check
            IF #input.controlInterface <> "dbRuckigConst".IFACE_POSITION
                AND #input.controlInterface <> "dbRuckigConst".IFACE_VELOCITY THEN
                #ValidateInput := "dbRuckigConst".RESULT_ERR_IFACE;
                RETURN;
            END_IF;
        END_REGION

        REGION Per-axis validation
            FOR #i := 0 TO #input.nDofs - 1 DO
                // Finiteness: currentPosition is the integration origin in both
                // interfaces; targetPosition is only used (and only required
                // finite) in the position interface.
                IF NOT "IsFiniteLreal"(x := #input.currentPosition[#i]) THEN
                    #ValidateInput := "dbRuckigConst".RESULT_ERR_NON_FINITE;
                    RETURN;
                END_IF;
                IF NOT "IsFiniteLreal"(x := #input.currentVelocity[#i]) THEN
                    #ValidateInput := "dbRuckigConst".RESULT_ERR_NON_FINITE;
                    RETURN;
                END_IF;
                IF NOT "IsFiniteLreal"(x := #input.currentAcceleration[#i]) THEN
                    #ValidateInput := "dbRuckigConst".RESULT_ERR_NON_FINITE;
                    RETURN;
                END_IF;
                IF #input.controlInterface = "dbRuckigConst".IFACE_POSITION THEN
                    IF NOT "IsFiniteLreal"(x := #input.targetPosition[#i]) THEN
                        #ValidateInput := "dbRuckigConst".RESULT_ERR_NON_FINITE;
                        RETURN;
                    END_IF;
                END_IF;
                IF NOT "IsFiniteLreal"(x := #input.targetVelocity[#i]) THEN
                    #ValidateInput := "dbRuckigConst".RESULT_ERR_NON_FINITE;
                    RETURN;
                END_IF;
                IF NOT "IsFiniteLreal"(x := #input.targetAcceleration[#i]) THEN
                    #ValidateInput := "dbRuckigConst".RESULT_ERR_NON_FINITE;
                    RETURN;
                END_IF;

                // Positive limits: vMax is a constraint only in the position
                // interface; the velocity interface ignores it (Ruckig parity).
                IF #input.controlInterface = "dbRuckigConst".IFACE_POSITION THEN
                    IF #input.maxVelocity[#i] <= 0.0 THEN
                        #ValidateInput := "dbRuckigConst".RESULT_ERR_VMAX;
                        RETURN;
                    END_IF;
                END_IF;
                IF #input.maxAcceleration[#i] <= 0.0 THEN
                    #ValidateInput := "dbRuckigConst".RESULT_ERR_AMAX;
                    RETURN;
                END_IF;
                IF #input.maxJerk[#i] <= 0.0 THEN
                    #ValidateInput := "dbRuckigConst".RESULT_ERR_JMAX;
                    RETURN;
                END_IF;

                // NOTE: the current state (velocity/acceleration) is deliberately
                // NOT required to be within limits (Ruckig validate() default).
            END_FOR;
        END_REGION
```

Also bump the header version to `0.7.0` (the `S7_Version := "0.2.0"` line at the
top) and update `// Author:` is optional. Leave the rest of the file unchanged.

- [ ] **Step 4: Run to verify they pass**

Run: `uv run pytest tests/unit/test_validate_input.py -q`
Expected: PASS (all original + 6 new tests).

- [ ] **Step 5: Commit**

```bash
git add src/blocks/ValidateInput.s7dcl tests/unit/test_validate_input.py
git commit -m "v0.7 T5: ValidateInput velocity-mode rules + bad-interface rejection"
```

---

## Task 6: `RuckigOtg` velocity recompute branch + change detection

**Files:**
- Modify: `src/blocks/RuckigOtg.s7dcl`

Add `controlInterface` to change detection, and wrap the existing recompute body
in `IF position ... ELSE velocity ... END_IF`. The velocity branch mirrors the
multi-axis flow but uses `ComputeVelBlock1Axis` + `ComputeVelProfileTimed` (every
axis re-timed), no brake, no position at-target zero-fill, `Phase → Time`.

- [ ] **Step 1: Add `controlInterface` to change detection**

In `src/blocks/RuckigOtg.s7dcl`, inside `REGION Change detection`, after the
block:

```scl
                IF #input.durationDiscretization <> #prevInput.durationDiscretization THEN
                    #recomputeRequired := true;
                END_IF;
```

add:

```scl
                IF #input.controlInterface <> #prevInput.controlInterface THEN
                    #recomputeRequired := true;
                END_IF;
```

- [ ] **Step 2: Wrap the recompute body with a velocity branch**

In `REGION Recompute trajectory`, the body currently is:

```scl
            IF #recomputeRequired THEN
                //========================================================
                // SINGLE-AXIS PATH (1 DoF, no minimumDuration) -> v0.3 stack
                ...
                IF #multiPath THEN
                    ... (multi-axis path) ...
                END_IF;

                #trajectory.currentTime := 0.0;
                ...
            END_IF;
```

Change the outer structure to branch on the interface. Immediately after
`IF #recomputeRequired THEN`, insert:

```scl
                IF #input.controlInterface = "dbRuckigConst".IFACE_VELOCITY THEN
                    //====================================================
                    // VELOCITY INTERFACE PATH (v0.7) — all DoF counts
                    //====================================================
                    // 1. resolve effective mode + participation per axis
                    //    (Phase -> Time; velocity has no phase sync)
                    FOR #ax := 0 TO #nd - 1 DO
                        IF #input.perDofSynchronization[#ax] >= 0 THEN
                            #modeEff[#ax] := #input.perDofSynchronization[#ax];
                        ELSE
                            #modeEff[#ax] := #input.synchronization;
                        END_IF;
                        IF #modeEff[#ax] = "dbRuckigConst".SYNC_PHASE THEN
                            #modeEff[#ax] := "dbRuckigConst".SYNC_TIME;
                        END_IF;
                        #participates[#ax] := #modeEff[#ax] <> "dbRuckigConst".SYNC_NONE;
                    END_FOR;

                    // 2. Block every axis (No axes' tMin still sets the t_sync floor)
                    FOR #ax := 0 TO #nd - 1 DO
                        #blockResult := "ComputeVelBlock1Axis"(
                            block := #blockSet.items[#ax],
                            profile := #trajectory.profiles[#ax],
                            p0 := #chainPos[#ax], v0 := #chainVel[#ax], a0 := #chainAcc[#ax],
                            vf := #input.targetVelocity[#ax],
                            af := #input.targetAcceleration[#ax],
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

                    // 4. emit per axis (every axis re-timed via Step 2)
                    #mainDuration := 0.0;
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

                        #tinMoving := #modeEff[#ax] = "dbRuckigConst".SYNC_TIME;
                        IF #modeEff[#ax] = "dbRuckigConst".SYNC_TIME_IF_NECESSARY THEN
                            IF ABS(#input.targetVelocity[#ax]) > "dbRuckigConst".EPS_VELOCITY
                                OR ABS(#input.targetAcceleration[#ax]) > "dbRuckigConst".EPS_DERIV THEN
                                #tinMoving := true;
                            END_IF;
                        END_IF;

                        IF #tinMoving THEN
                            #axDur := #tSync;
                        ELSE
                            #axDur := #blockSet.items[#ax].tMin;
                        END_IF;

                        #profileResult := "ComputeVelProfileTimed"(
                            profile := #trajectory.profiles[#ax],
                            p0 := #chainPos[#ax], v0 := #chainVel[#ax], a0 := #chainAcc[#ax],
                            vf := #input.targetVelocity[#ax],
                            af := #input.targetAcceleration[#ax],
                            aMax := #input.maxAcceleration[#ax],
                            jMax := #input.maxJerk[#ax],
                            tf := #axDur);
                        IF #profileResult <> "dbRuckigConst".RESULT_WORKING AND #profileResult <> "dbRuckigConst".RESULT_FINISHED THEN
                            #valid := false;
                            #busy := false;
                            #done := false;
                            #error := true;
                            #status := "dbRuckigConst".RESULT_ERR_TRAJ;
                            #trajectory.isValid := false;
                            RETURN;
                        END_IF;

                        IF #participates[#ax] THEN
                            #trajectory.independentMinDurations[#ax] := #blockSet.items[#ax].tMin;
                        ELSE
                            #trajectory.independentMinDurations[#ax] := #axDur;
                        END_IF;
                        IF #axDur > #mainDuration THEN
                            #mainDuration := #axDur;
                        END_IF;
                    END_FOR;
                    #trajectory.duration := #mainDuration;

                ELSE
```

Then **leave the entire existing position recompute body unchanged** (the
`IF NOT #multiPath THEN ... END_IF;` and `IF #multiPath THEN ... END_IF;`
blocks), and add a single `END_IF;` to close the new `ELSE` **before** the
trailing common lines:

```scl
                END_IF;   // <-- closes IF velocity / ELSE position

                #trajectory.currentTime := 0.0;
                #trajectory.isValid := true;
                #prevInput := #input;
                #firstCall := false;
            END_IF;
```

> The common tail (`currentTime := 0`, `isValid := true`, `prevInput := input`,
> `firstCall := false`) stays shared by both interfaces, exactly as today.

- [ ] **Step 3: Bump the FB header version**

Change the `S7_Version := "0.6.0"` line at the top of `RuckigOtg.s7dcl` to
`S7_Version := "0.7.0"`.

- [ ] **Step 4: Verify the whole unit suite still passes (no regression)**

Run: `uv run pytest tests/unit -q`
Expected: PASS — all existing position unit tests plus the new velocity unit
tests. (Parity for the velocity path is exercised in Task 7.)

- [ ] **Step 5: Commit**

```bash
git add src/blocks/RuckigOtg.s7dcl
git commit -m "v0.7 T6: RuckigOtg velocity recompute branch + controlInterface change detection"
```

---

## Task 7: Parity — runners, scenario mapping, velocity scenarios

**Files:**
- Modify: `tests/parity/runner_ref.py`
- Modify: `tests/parity/runner_scl.py`
- Modify: `tests/parity/test_parity.py`
- Create: `tests/parity/scenarios/v07_01_1dof_vel_rest_to_vel.yaml`
- Create: `tests/parity/scenarios/v07_02_1dof_vel_nonzero_a0.yaml`
- Create: `tests/parity/scenarios/v07_03_1dof_vel_nonzero_af.yaml`
- Create: `tests/parity/scenarios/v07_04_1dof_vel_negative.yaml`
- Create: `tests/parity/scenarios/v07_05_2dof_vel_time.yaml`
- Create: `tests/parity/scenarios/v07_06_2dof_vel_no.yaml`
- Create: `tests/parity/scenarios/v07_07_2dof_vel_per_dof.yaml`
- Create: `tests/parity/scenarios/v07_08_2dof_vel_discrete.yaml`

- [ ] **Step 1: Add `control_interface` to `runner_ref.py`**

Add a module-level map after `_DISC_MAP`:

```python
_IFACE_MAP = {0: ruckig.ControlInterface.Position, 1: ruckig.ControlInterface.Velocity,
              "position": ruckig.ControlInterface.Position,
              "velocity": ruckig.ControlInterface.Velocity}
```

Add `control_interface: int | str = 0,` to the `run_reference(...)` signature
(next to `duration_discretization`), and after the line
`inp.duration_discretization = _DISC_MAP[duration_discretization]` add:

```python
    inp.control_interface = _IFACE_MAP[control_interface]
```

- [ ] **Step 2: Add `control_interface` to `runner_scl.py`**

Add `control_interface: int | str = 0,` to the `run_scl(...)` signature, then
just before building the `inp` dict add:

```python
    iface = 1 if control_interface in (1, "velocity") else 0
```

and change the `inp` dict line `"controlInterface": 0,` to
`"controlInterface": iface,`.

- [ ] **Step 3: Map the field in `test_parity.py`**

In `_kwargs_from_scenario`, add to the returned dict:

```python
        "control_interface": data.get("control_interface", 0),
```

- [ ] **Step 4: Create the velocity scenarios**

`tests/parity/scenarios/v07_01_1dof_vel_rest_to_vel.yaml`:

```yaml
name: v0.7 1-DoF velocity rest -> target velocity
control_interface: velocity
n_dofs: 1
current_velocity: [0.0]
target_velocity: [1.5]
max_velocity: [2.0]
max_acceleration: [5.0]
max_jerk: [10.0]
cycle_time: 0.010
```

`tests/parity/scenarios/v07_02_1dof_vel_nonzero_a0.yaml`:

```yaml
name: v0.7 1-DoF velocity with non-zero initial acceleration
control_interface: velocity
n_dofs: 1
current_velocity: [0.5]
current_acceleration: [2.0]
target_velocity: [2.0]
max_velocity: [3.0]
max_acceleration: [5.0]
max_jerk: [10.0]
cycle_time: 0.010
```

`tests/parity/scenarios/v07_03_1dof_vel_nonzero_af.yaml`:

```yaml
name: v0.7 1-DoF velocity to a non-zero target acceleration
control_interface: velocity
n_dofs: 1
current_velocity: [0.0]
target_velocity: [1.0]
target_acceleration: [1.0]
max_velocity: [3.0]
max_acceleration: [5.0]
max_jerk: [10.0]
cycle_time: 0.010
```

`tests/parity/scenarios/v07_04_1dof_vel_negative.yaml`:

```yaml
name: v0.7 1-DoF velocity decreasing (negative vd)
control_interface: velocity
n_dofs: 1
current_velocity: [2.0]
target_velocity: [0.0]
max_velocity: [3.0]
max_acceleration: [5.0]
max_jerk: [10.0]
cycle_time: 0.010
```

`tests/parity/scenarios/v07_05_2dof_vel_time.yaml`:

```yaml
name: v0.7 2-DoF velocity time-synchronized
control_interface: velocity
synchronization: 2
n_dofs: 2
current_velocity: [0.0, 0.0]
target_velocity: [2.0, 0.5]
max_velocity: [3.0, 3.0]
max_acceleration: [5.0, 5.0]
max_jerk: [10.0, 10.0]
cycle_time: 0.010
```

`tests/parity/scenarios/v07_06_2dof_vel_no.yaml`:

```yaml
name: v0.7 2-DoF velocity, no synchronization
control_interface: velocity
synchronization: 0
n_dofs: 2
current_velocity: [0.0, 0.0]
target_velocity: [2.0, 0.5]
max_velocity: [3.0, 3.0]
max_acceleration: [5.0, 5.0]
max_jerk: [10.0, 10.0]
cycle_time: 0.010
```

`tests/parity/scenarios/v07_07_2dof_vel_per_dof.yaml`:

```yaml
name: v0.7 2-DoF velocity per-DoF [No, Time]
control_interface: velocity
per_dof_synchronization: [0, 2]
n_dofs: 2
current_velocity: [0.0, 0.0]
target_velocity: [2.0, 0.5]
max_velocity: [3.0, 3.0]
max_acceleration: [5.0, 5.0]
max_jerk: [10.0, 10.0]
cycle_time: 0.010
```

`tests/parity/scenarios/v07_08_2dof_vel_discrete.yaml`:

```yaml
name: v0.7 2-DoF velocity time-sync with discrete duration
control_interface: velocity
synchronization: 2
duration_discretization: 1
n_dofs: 2
current_velocity: [0.0, 0.0]
target_velocity: [2.0, 0.5]
max_velocity: [3.0, 3.0]
max_acceleration: [5.0, 5.0]
max_jerk: [10.0, 10.0]
cycle_time: 0.010
```

- [ ] **Step 5: Run the velocity parity scenarios**

Run: `uv run pytest tests/parity -q -k v07`
Expected: PASS (8 scenarios). If a scenario fails, dump `ref`/`scl` durations
and the first failing cycle from the assertion message; compare against the
oracle to localise the Step-1/Step-2 family at fault.

- [ ] **Step 6: Run the full suite (no regression)**

Run: `uv run pytest -q`
Expected: PASS — all unit + all parity (existing + 8 new velocity scenarios).

- [ ] **Step 7: Commit**

```bash
git add tests/parity/runner_ref.py tests/parity/runner_scl.py tests/parity/test_parity.py tests/parity/scenarios/v07_*.yaml
git commit -m "v0.7 T7: parity runners control_interface + 8 velocity scenarios"
```

---

## Task 8: Release 0.7.0

**Files:**
- Modify: `CHANGELOG.md`
- Modify: `README.md`
- Modify: `pyproject.toml`
- Modify: `uv.lock`

- [ ] **Step 1: CHANGELOG**

Add a new top section to `CHANGELOG.md` (match the existing entry style):

```markdown
## [0.7.0] - 2026-06-04

### Added
- **Velocity control interface** (`controlInterface = IFACE_VELOCITY`,
  third-order). Drives the velocity state `(v0, a0) -> (vf, af)` jerk-limited
  under the acceleration/jerk limits; position is uncontrolled (integrated and
  output, not targeted), `maxVelocity` is ignored (Ruckig parity).
- New FCs: `ComputeVelBlock1Axis` (Step 1 -> Block), `ComputeVelProfileTimed`
  (Step 2 re-time), `CheckVelProfile` (velocity validity), plus the internal
  direction helpers `CollectVelStep1Dir` and `SolveVelTimedDir`.
- Wired through the existing multi-axis flow: single-axis plus Time / No /
  per-DoF / TimeIfNecessary / Discrete synchronization all work for velocity;
  `Phase` falls back to `Time`.
- `ValidateInput` velocity-mode rules and `RESULT_ERR_IFACE` (16#8208) for an
  out-of-range `controlInterface`.
- 8 velocity parity scenarios (`v07_01`..`v07_08`).

### Notes
- Second-order (jerk-unlimited) velocity interface is out of scope; the port is
  third-order throughout.
- No brake pre-phase in velocity mode (the Step-1 solver absorbs an arbitrary
  `a0`).
```

- [ ] **Step 2: README**

In `README.md`: add a `## Features (v0.7)` section above `## Features (v0.6)`
describing the velocity interface; update the **Status** line to v0.7.0; in the
**Roadmap** table mark v0.7 as *(this release)* and confirm v0.8 = brake
profiles; add the v0.7 spec link in the design-specs paragraph of the
Architecture section; update the parity-scenario count in the **Tests** section
to the new total.

- [ ] **Step 3: Version bumps**

In `pyproject.toml` change `version = "0.6.0"` to `version = "0.7.0"`, then sync
the lockfile:

Run: `uv lock`
Expected: `uv.lock` updates the project version to 0.7.0.

- [ ] **Step 4: Final full suite**

Run: `uv run pytest -q`
Expected: PASS (all green).

- [ ] **Step 5: Commit**

```bash
git add CHANGELOG.md README.md pyproject.toml uv.lock
git commit -m "v0.7 T8: release 0.7.0 (docs + version)"
```

---

## Final integration (handled by the execution skill)

After Task 8, hand off to `superpowers:finishing-a-development-branch`:
merge the feature branch into `master` with `--no-ff`, tag `v0.7.0`, and push.
A final opus review of the velocity solver (Step-1/Step-2 family completeness,
the at-target/coast handling, the `RuckigOtg` branch) should run before merge,
as in prior versions.

---

## Self-review (planner)

- **Spec coverage:** velocity Step1/Step2/check (T2–T4), RuckigOtg integration +
  change detection (T6), ValidateInput rules + `RESULT_ERR_IFACE` (T1, T5),
  Phase→Time / per-DoF / TimeIfNecessary / Discrete reuse (T6 modeEff + T7
  scenarios), parity runners + scenarios (T7), release (T8). All spec sections
  map to a task.
- **No placeholders:** every code step shows complete SCL/Python.
- **Type/name consistency:** FC names (`CheckVelProfile`, `CollectVelStep1Dir`,
  `ComputeVelBlock1Axis`, `SolveVelTimedDir`, `ComputeVelProfileTimed`) and the
  `vf/af/saMax/sjMax/tf` parameter names are used identically across tasks; the
  constant `RESULT_ERR_IFACE` (T1) is consumed in T5; `controlInterface` change
  detection (T6) matches the `prevInput` pattern.
- **Refinements flagged:** cohesive velocity branch (not 3 in-place IFs), no
  velocity at-target zero-fill (coast via trivial `time_none`), every axis
  re-timed via Step 2 — all documented in "Design notes" above.
```
