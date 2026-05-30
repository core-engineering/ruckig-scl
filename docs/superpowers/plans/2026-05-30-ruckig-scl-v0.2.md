# ruckig-scl v0.2 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Single-axis, time-optimal jerk-limited OTG for arbitrary initial AND target states (`v0,a0 ≠ 0`, `vT,aT ≠ 0`), with online `pass_to_input` chaining and a minimal brake pre-phase, in TIA Portal SCL.

**Architecture:** Approach B (structured re-derivation). A general profile solver (`ComputeProfile1Dof`) replaces v0.1's rest-to-rest `ComputeMinDuration`+`ComputeFinalProfile`. It enumerates Ruckig's profile types (UDDU/UDUD × ACC0_ACC1_VEL…NONE), validates each candidate, and selects the minimum-duration feasible one. Closed-form root solvers (`SolveCubic`/`SolveQuartic`) back the type equations. The `RuckigOtg` FB chains from its last setpoint and prepends a brake when the first-enable state is out of limits.

**Tech Stack:** Siemens SCL (`.s7dcl`, TIA Portal export format) tested via the `siemens-plc-tools` `plc-code` transpiler (SCL→Python) + pytest; numerical parity against the official Ruckig solver (PyPI `ruckig`, `parity` extra).

**Reference (MIT):** Ruckig single-DoF source — consult during implementation:
- Root solving: `https://raw.githubusercontent.com/pantor/ruckig/main/include/ruckig/roots.hpp`
- Profile types: `https://raw.githubusercontent.com/pantor/ruckig/main/include/ruckig/position-first-step1.hpp`
- Brake: `https://raw.githubusercontent.com/pantor/ruckig/main/include/ruckig/brake.hpp`

**Oracle-driven TDD:** Where an exact closed-form equation is intricate (the profile-type FCs), the test asserts against expected values **generated from the installed `ruckig`** for specific inputs; the implementation ports the named Ruckig function and is correct when the oracle test passes. Infrastructure bricks below ship with complete code.

**Conventions (from v0.1):** TIA Portal export format — `{ S7_* }` header block, `FUNCTION "Name" : Type` / `FUNCTION_BLOCK "Name"`, `{ S7_Language := "SCL" } NETWORK … END_NETWORK`, `#`-prefixed locals, MixedCase types (`LReal/Int/Bool/Word/DInt`), lowercase `true/false`, UDT refs as `_.typeName`, DB refs as `"dbRuckigConst".X`, REGION names free-form. Tests use the shared `make_harness` fixture in `tests/conftest.py` (search paths: `src/blocks`, `src/data-types`, `src/data-blocks`). Run a single test with `uv run --no-sync pytest <path> -p no:cacheprovider -q`.

---

## File Structure

**New SCL blocks (`src/blocks/`):**
- `SolveCubic.s7dcl` — real roots of a cubic (Cardano).
- `SolveQuartic.s7dcl` — real roots of a quartic (Ferrari via resolvent cubic).
- `IntegrateProfileStates.s7dcl` — fill `a/v/p` from `t/j` + initial state.
- `CheckProfile.s7dcl` — validate a candidate (limits + reaches target).
- `ComputeProfile1Dof.s7dcl` — the enumerate-and-select solver.
- `ComputeBrakeProfile.s7dcl` — brake sub-profile for out-of-limits initial state.

**Modified:**
- `src/blocks/StateAtTime.s7dcl` — evaluate brake phases before the main profile.
- `src/blocks/RuckigOtg.s7dcl` — pass_to_input chaining, brake, extended change detection.
- `src/blocks/ValidateInput.s7dcl` — (no logic change; confirm non-zero target vel/acc accepted).

**Removed (superseded):**
- `src/blocks/ComputeMinDuration.s7dcl` + `tests/unit/test_compute_min_duration.py`
- `src/blocks/ComputeFinalProfile.s7dcl` + `tests/unit/test_compute_final_profile.py`
  (integration half lives on as `IntegrateProfileStates`).

**New tests (`tests/unit/`):** one per new block. **Parity (`tests/parity/scenarios/`):** +10 v0.2 scenarios.

---

## Task 1: Add inverse-trig builtins to the plc-code transpiler (prerequisite)

The cubic solver's three-real-roots branch needs `ACOS` (and `ATAN` for robustness). The `plc-code` `ExpressionTranslator.BUILTIN_MAP` currently maps `SQRT/SIN/COS/TAN/LN/EXP/ABS` but no inverse trig. Add them in **siemens-plc-tools** (sibling repo), commit on a branch, so `ruckig-scl` can use `ACOS(...)` in SCL.

**Files:**
- Modify: `../siemens-plc-tools/packages/plc-code/src/plc_code/executor/codegen.py` (the `BUILTIN_MAP` default dict)
- Test: `../siemens-plc-tools/packages/plc-code/tests/test_executor/test_codegen.py`

- [ ] **Step 1: Write the failing test**

```python
# append to tests/test_executor/test_codegen.py
from plc_code.executor.codegen import ExpressionTranslator

def test_inverse_trig_builtins():
    t = ExpressionTranslator()
    assert t.translate("ACOS(#x)") == "math.acos(self.x)"
    assert t.translate("ASIN(#x)") == "math.asin(self.x)"
    assert t.translate("ATAN(#x)") == "math.atan(self.x)"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd ../siemens-plc-tools && uv run --no-sync pytest packages/plc-code/tests/test_executor/test_codegen.py::test_inverse_trig_builtins -p no:cacheprovider -q`
Expected: FAIL (`ACOS(` not translated → stays `ACOS(self.x)`).

- [ ] **Step 3: Add the builtins**

In `codegen.py`, in the `BUILTIN_MAP` default-factory dict, add alongside the existing trig entries:

```python
            "ASIN": "math.asin",
            "ACOS": "math.acos",
            "ATAN": "math.atan",
            "ATAN2": "math.atan2",
```

- [ ] **Step 4: Run test to verify it passes**

Run the command from Step 2. Expected: PASS.

- [ ] **Step 5: Run the full plc-code suite (no regressions)**

Run: `cd ../siemens-plc-tools && uv run --no-sync pytest packages/plc-code/tests -p no:cacheprovider -q`
Expected: all pass (was 859 + new test).

- [ ] **Step 6: Commit (in siemens-plc-tools)**

```bash
cd ../siemens-plc-tools
git checkout -b feature/inverse-trig-builtins
git add packages/plc-code/src/plc_code/executor/codegen.py packages/plc-code/tests/test_executor/test_codegen.py
git commit -m "feat(plc-code): map ASIN/ACOS/ATAN/ATAN2 builtins to math.*"
```

> Merge + push this branch to `origin/main` once v0.2 is validated (same flow as the limitation-fix branch).

---

## Task 2: `SolveCubic` FC (real roots of a cubic)

**Files:**
- Create: `src/blocks/SolveCubic.s7dcl`
- Test: `tests/unit/test_solve_cubic.py`

Math oracle = known polynomials. Cardano with degenerate fallbacks; the
three-real-roots branch uses the trigonometric form (needs `ACOS` from Task 1).
Cube root is inlined as `sign(y)·exp(ln(|y|)/3)`. **Transpiler note:** `END_IF`
must be on its own line (single-line `IF…THEN…END_IF;` is not parsed by the test
harness), so all `IF` blocks below are multi-line.

- [ ] **Step 1: Write the failing test**

```python
# tests/unit/test_solve_cubic.py
import pytest


@pytest.fixture
def harness(make_harness):
    return make_harness("SolveCubic.s7dcl")


def _roots(harness, a, b, c, d):
    harness.reset()
    harness.set_inputs(a=a, b=b, c=c, d=d)
    harness.execute()
    n = harness.get_output("count")
    out = harness.get_output("roots")
    return sorted(out[i] for i in range(n))


def test_three_distinct_real_roots(harness):
    # (x-1)(x-2)(x-3) = x^3 - 6x^2 + 11x - 6
    r = _roots(harness, 1.0, -6.0, 11.0, -6.0)
    assert r == pytest.approx([1.0, 2.0, 3.0], abs=1e-6)


def test_one_real_root(harness):
    # x^3 + x - 2 = (x-1)(x^2 + x + 2) -> single real root x=1
    r = _roots(harness, 1.0, 0.0, 1.0, -2.0)
    assert r == pytest.approx([1.0], abs=1e-6)


def test_double_root(harness):
    # (x-1)^2 (x+2) = x^3 - 3x + 2 -> roots {1 (double), -2}
    r = _roots(harness, 1.0, 0.0, -3.0, 2.0)
    assert r == pytest.approx([-2.0, 1.0], abs=1e-6)


def test_degenerate_quadratic(harness):
    # a=0 -> x^2 - 3x + 2 = 0 -> {1, 2}
    r = _roots(harness, 0.0, 1.0, -3.0, 2.0)
    assert r == pytest.approx([1.0, 2.0], abs=1e-6)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run --no-sync pytest tests/unit/test_solve_cubic.py -p no:cacheprovider -q`
Expected: FAIL (file/block not found).

- [ ] **Step 3: Write `src/blocks/SolveCubic.s7dcl`**

```scl
{
    S7_Author := "Martin C";
    S7_EditorMode := "SCL";
    S7_Family := "Ruckig";
    S7_Optimized := "TRUE";
    S7_Version := "0.2.0"
}
FUNCTION "SolveCubic" : Void
    VAR_INPUT
        a : LReal;
        b : LReal;
        c : LReal;
        d : LReal;
    END_VAR
    VAR_OUTPUT
        roots : Array[0..3] of LReal;
        count : Int;
    END_VAR
    VAR_TEMP
        bb    : LReal;
        cc    : LReal;
        dd    : LReal;
        p     : LReal;
        q     : LReal;
        shift : LReal;
        disc  : LReal;
        sq    : LReal;
        u     : LReal;
        v     : LReal;
        m     : LReal;
        theta : LReal;
        qdisc : LReal;
        i     : DInt;
    END_VAR
    VAR CONSTANT
        EPS : LReal := 1.0E-12;
        PI  : LReal := 3.141592653589793;
    END_VAR

    { S7_Language := "SCL" }
    NETWORK
        REGION Degenerate to quadratic or linear
            IF ABS(#a) < #EPS THEN
                IF ABS(#b) < #EPS THEN
                    IF ABS(#c) < #EPS THEN
                        #count := 0;
                        RETURN;
                    END_IF;
                    #roots[0] := -#d / #c;
                    #count := 1;
                    RETURN;
                END_IF;
                #qdisc := #c * #c - 4.0 * #b * #d;
                IF #qdisc < -#EPS THEN
                    #count := 0;
                    RETURN;
                END_IF;
                IF #qdisc < #EPS THEN
                    #roots[0] := -#c / (2.0 * #b);
                    #count := 1;
                    RETURN;
                END_IF;
                #roots[0] := (-#c + SQRT(#qdisc)) / (2.0 * #b);
                #roots[1] := (-#c - SQRT(#qdisc)) / (2.0 * #b);
                #count := 2;
                RETURN;
            END_IF;
        END_REGION

        REGION Depress to t cubed plus p t plus q
            #bb := #b / #a;
            #cc := #c / #a;
            #dd := #d / #a;
            #shift := #bb / 3.0;
            #p := #cc - #bb * #bb / 3.0;
            #q := 2.0 * #bb * #bb * #bb / 27.0 - #bb * #cc / 3.0 + #dd;
        END_REGION

        REGION Solve by discriminant
            #disc := #q * #q / 4.0 + #p * #p * #p / 27.0;
            IF #disc > #EPS THEN
                #sq := SQRT(#disc);
                #u := EXP(LN(ABS(-#q / 2.0 + #sq)) / 3.0);
                IF (-#q / 2.0 + #sq) < 0.0 THEN
                    #u := -#u;
                END_IF;
                #v := EXP(LN(ABS(-#q / 2.0 - #sq)) / 3.0);
                IF (-#q / 2.0 - #sq) < 0.0 THEN
                    #v := -#v;
                END_IF;
                #roots[0] := #u + #v - #shift;
                #count := 1;
            ELSIF #disc > -#EPS THEN
                #u := EXP(LN(ABS(-#q / 2.0)) / 3.0);
                IF (-#q / 2.0) < 0.0 THEN
                    #u := -#u;
                END_IF;
                IF ABS(#u) < #EPS THEN
                    #roots[0] := -#shift;
                    #count := 1;
                ELSE
                    #roots[0] := 2.0 * #u - #shift;
                    #roots[1] := -#u - #shift;
                    #count := 2;
                END_IF;
            ELSE
                #m := 2.0 * SQRT(-#p / 3.0);
                #theta := ACOS(3.0 * #q / (#p * #m)) / 3.0;
                FOR #i := 0 TO 2 DO
                    #roots[#i] := #m * COS(#theta - 2.0 * #PI * #i / 3.0) - #shift;
                END_FOR;
                #count := 3;
            END_IF;
        END_REGION
    END_NETWORK
END_FUNCTION
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run --no-sync pytest tests/unit/test_solve_cubic.py -p no:cacheprovider -q`
Expected: 4 PASS. If the three-real-roots test fails with `ACOS` untranslated, Task 1 was not applied.

- [ ] **Step 5: Commit**

```bash
git add src/blocks/SolveCubic.s7dcl tests/unit/test_solve_cubic.py
git commit -m "feat(solver): add SolveCubic FC (Cardano + degenerate fallbacks)"
```

---
