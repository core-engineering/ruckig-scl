# ruckig-scl v0.2 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Single-axis, time-optimal jerk-limited OTG for arbitrary initial AND target states (`v0,a0 ≠ 0`, `vT,aT ≠ 0`), with online `pass_to_input` chaining and a minimal brake pre-phase, in TIA Portal SCL.

**Architecture:** Approach B (structured re-derivation). A general profile solver (`ComputeProfile1Dof`) replaces v0.1's rest-to-rest `ComputeMinDuration`+`ComputeFinalProfile`. It enumerates Ruckig's profile types (UDDU/UDUD × ACC0_ACC1_VEL…NONE), validates each candidate, and selects the minimum-duration feasible one. Closed-form root solvers (`SolveCubic`/`SolveQuartic`) back the type equations. The `RuckigOtg` FB chains from its last setpoint and prepends a brake when the first-enable state is out of limits.

**Tech Stack:** Siemens SCL (`.s7dcl`, TIA Portal export format) tested via the `siemens-plc-tools` `plc-code` transpiler (SCL→Python) + pytest; numerical parity against the official Ruckig solver (PyPI `ruckig`, `parity` extra).

**Reference (MIT):** Ruckig single-DoF source — consult during implementation:
- Root solving: `include/ruckig/roots.hpp`
- Profile types (single-DoF Step 1 / Step 2, profile validation, time intervals): `include/ruckig/position.hpp`, `include/ruckig/block.hpp`, `include/ruckig/profile.hpp`
- Brake: `include/ruckig/brake.hpp`
- Base: `https://raw.githubusercontent.com/pantor/ruckig/main/` (e.g. `…/include/ruckig/position.hpp`). Fetch with `curl` at implementation time.

**Oracle-driven TDD:** Where an exact closed-form equation is intricate (the profile-type FCs), the test asserts against expected values **generated from the installed `ruckig`** for specific inputs; the implementation ports the named Ruckig function and is correct when the oracle test passes. Infrastructure bricks below ship with complete code.

**Conventions (from v0.1):** TIA Portal export format — `{ S7_* }` header block, `FUNCTION "Name" : Type` / `FUNCTION_BLOCK "Name"`, `{ S7_Language := "SCL" } NETWORK … END_NETWORK`, `#`-prefixed locals, MixedCase types (`LReal/Int/Bool/Word/DInt`), lowercase `true/false`, UDT refs as `_.typeName`, DB refs as `"dbRuckigConst".X`, REGION names free-form. Tests use the shared `make_harness` fixture in `tests/conftest.py` (search paths: `src/blocks`, `src/data-types`, `src/data-blocks`). Run a single test with `uv run --no-sync pytest <path> -p no:cacheprovider -q`.

**plc-code transpiler quirks discovered during v0.2 (work around these):**
1. **`END_IF`/`END_FOR` on their OWN line** — single-line `IF..THEN..END_IF;` mis-parses.
2. **Identifier ending in `of`/`Dof`** — the lexer reads the trailing `of` as the `OF` keyword and corrupts the statement. The solver FUNCTION is named **`ComputeProfile1Axis`** (file stays `ComputeProfile1Dof.s7dcl`; harness keys `get_output("ComputeProfile1Axis")`). Avoid trailing `of` in new identifiers. (Real plc-code bug → candidate upstream fix: lexer must not split `OF` out of a longer identifier.)
3. **Division by a parenthesised product** `/ (a*b)` transpiles wrong — precompute the denominator into a scalar VAR_TEMP (`den := a*b;`) and divide by `#den`. Stage huge numerators in a `#num` temp too. The acc0_acc1 / quartic families have large denominators — budget denom temps up front.
4. **Bool-returning FC with VAR_IN_OUT works**: `#ok := "CheckProfile"(profile := #profile, ...);` → mutates the VAR_IN_OUT profile in place and returns the Bool keyed by the FC name. Reuse this pattern.

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

## Tasks 3–13 — remaining work

Tasks 1–2 above are fully expanded (verifiable infrastructure). Tasks 3–13 are
specified here at file/interface/test-strategy level; each is expanded into
full TDD steps **just-in-time at dispatch**, when the implementer fetches the
relevant Ruckig source and computes oracle values from the installed `ruckig`.
This is deliberate: the profile-type equations are reference-dependent and must
be ported against the live source + oracle rather than transcribed from memory.

Each task follows the same 5-step rhythm (write failing test → verify fail →
implement → verify pass → commit) and the conventions in the header.

### Task 3: `SolveQuartic` FC
- **Files:** `src/blocks/SolveQuartic.s7dcl`, `tests/unit/test_solve_quartic.py`.
- **Approach:** depress to `x⁴+p·x²+q·x+r`; solve the **resolvent cubic** by calling `"SolveCubic"(...)`; form two quadratics; collect real roots into `roots:Array[0..3]` + `count`. Degrade to `SolveCubic` when `|a|<EPS`.
- **Test (oracle = known polynomials):** `(x-1)(x-2)(x-3)(x-4)` → {1,2,3,4}; `(x²+1)(x-1)(x-2)` → {1,2}; a double-root quartic; `a=0` → cubic fallback. `abs=1e-6`.

### Task 4: `IntegrateProfileStates` FC
- **Files:** `src/blocks/IntegrateProfileStates.s7dcl`, `tests/unit/test_integrate_profile_states.py`.
- **Interface:** `VAR_IN_OUT profile : _.typeProfile`, `VAR_INPUT p0,v0,a0 : LReal`. Fills `profile.a[0..7]/v/p` from `profile.t[0..6]/j[0..6]` + initial state, using the cubic integration already in v0.1 `ComputeFinalProfile` (`a[i+1]=a[i]+j[i]·dt`; `v[i+1]=v[i]+a[i]·dt+0.5·j[i]·dt²`; `p[i+1]=p[i]+v[i]·dt+0.5·a[i]·dt²+j[i]·dt³/6`), seeded with `(p0,v0,a0)` instead of `(p0,0,0)`.
- **Test:** a hand-built 7-phase `t/j` from a known rest-to-rest profile reproduces v0.1's `p[7]`; a non-rest `(v0,a0)` case checked against forward Euler-cubic by hand.

### Task 5: `CheckProfile` FC
- **Files:** `src/blocks/CheckProfile.s7dcl`, `tests/unit/test_check_profile.py`.
- **Interface:** `VAR_IN_OUT profile`, `VAR_INPUT p0,v0,a0,pT,vT,aT,vMax,aMax : LReal`, returns `Bool`. Calls `"IntegrateProfileStates"`, then valid ⇔ all `t[i] ≥ −EPS` AND `|profile.v[i]| ≤ vMax+EPS` AND `|profile.a[i]| ≤ aMax+EPS` (i=0..7) AND final `(p,v,a)[7]` ≈ `(pT,vT,aT)` within EPS.
- **Test:** a valid known profile → true; a profile with a `t<0` → false; one exceeding `vMax` → false; one missing the target → false.

### Task 6: `ComputeProfile1Dof` FC — the solver (oracle-driven, sub-tasked)
- **Files:** `src/blocks/ComputeProfile1Dof.s7dcl`, `tests/unit/test_compute_profile_1dof.py`.
- **Interface:** `VAR_IN_OUT profile`, `VAR_INPUT p0,v0,a0,pT,vT,aT,vMax,aMax,jMax : LReal`, returns `Word` (status). Fills `profile.t/j/direction/controlSigns`, calls `IntegrateProfileStates`, returns `WORKING` (or `ERR_SOLVER`).
- **Structure:** for each jerk family (UDDU, UDUD) and each profile type (`ACC0_ACC1_VEL, ACC1_VEL, ACC0_VEL, VEL, ACC0_ACC1, ACC1, ACC0, NONE`), compute candidate phase times analytically (via `SolveCubic`/`SolveQuartic`), build `t/j`, validate with `"CheckProfile"`, keep the min-duration valid candidate.

#### Reference source (CONFIRMED locations — the plan's earlier `.hpp` guess was wrong)
The candidate-time **equations** live in the repo's `src/ruckig/*.cpp` (the `.hpp` files only declare). Fetch at dispatch:
- `https://raw.githubusercontent.com/pantor/ruckig/master/src/ruckig/position_third_step1.cpp` (566 lines — the Step-1 single-DoF solver, the file we port here)
- `https://raw.githubusercontent.com/pantor/ruckig/master/include/ruckig/profile.hpp` (531 lines — `check<ControlSigns, ReachedLimits>` template + `ControlSigns`/`ReachedLimits` enums; we do NOT port this — our `CheckProfile` FC already does the equivalent validation-by-integration)
- `https://raw.githubusercontent.com/pantor/ruckig/master/src/ruckig/brake.cpp` (131 lines — for Task 7)
- These headers are also bundled at `/.venv/lib/python3.12/site-packages/include/ruckig/` in this project.

#### Key porting facts (reverse-engineered from position_third_step1.cpp)
- **Symmetric limits** (our v0.2 assumption): `vMin=-vMax`, `aMin=-aMax`, `jMin=-jMax`. Ruckig keeps them separate; substitute the symmetric values when porting.
- **Precomputed terms** the formulas use (compute once as VAR_TEMP): `pd = pT - p0`; `v0_v0=v0*v0`; `vf_vf=vT*vT`; `a0_a0=a0*a0`, `a0_p3=a0³`, `a0_p4=a0⁴`; `af_af=aT*aT`, `af_p3=aT³`, `af_p4=aT⁴`; `jMax_jMax=jMax*jMax`. (Ruckig names target `vf/af` = our `vT/aT`.)
- **`time_*` methods** each set `profile.t[0..6]` then call `check<UDDU|UDUD, TYPE>`. In our port: set `#profile.t[0..6]`, set the 7 `#profile.j[]` for the family (UDDU = `+j,0,-j,0,-j,0,+j`; UDUD = `+j,0,-j,0,+j,0,-j`, scaled by direction sign), set `#profile.controlSigns`, then call `"CheckProfile"(...)`; if TRUE compute `duration = Σ t`, keep if `< bestDuration`.
- **Direction / sign:** Ruckig's `get_profile` tries the profile with `jMax` and also the mirrored case. Simplest correct port: run the whole enumeration twice — once with the inputs as-is (`+jMax` families) and once on the **mirrored problem** (`p0,v0,a0,pT,vT,aT → -p0,-v0,-a0,-pT,-vT,-aT` with the same vMax/aMax/jMax), and negate the resulting `t`-profile's `j[]`/states back. Equivalstly track `direction` and flip signs. VERIFY against the oracle which sign convention reproduces Ruckig.
- **Selection:** keep the **minimum-duration** candidate that `CheckProfile` accepts. Store best into a scratch `Array[0..6]` (`bestT`) + `bestSigns`/`bestDuration`; at the end write `bestT`→`#profile.t`, set `j[]` from `bestSigns`, call `"IntegrateProfileStates"`, return `WORKING`. If none valid → `ERR_SOLVER`.

#### `time_all_vel` (UDDU velocity-reaching family) — verbatim formulas to port FIRST
From position_third_step1.cpp lines 25–85. Four candidate types, tried in order; first that `check`s is added. Port each as a REGION that fills `t[0..6]` then calls `CheckProfile`:

ACC0_ACC1_VEL:
```
t0 = (-a0 + aMax)/jMax
t1 = (a0_a0/2 - aMax*aMax - jMax*(v0 - vMax))/(aMax*jMax)
t2 = aMax/jMax
t3 = (3*(a0_p4*aMin - af_p4*aMax) + 8*aMax*aMin*(af_p3 - a0_p3 + 3*jMax*(a0*v0 - af*vf)) + 6*a0_a0*aMin*(aMax*aMax - 2*jMax*v0) - 6*af_af*aMax*(aMin*aMin - 2*jMax*vf) - 12*jMax*(aMax*aMin*(aMax*(v0 + vMax) - aMin*(vf + vMax) - 2*jMax*pd) + (aMin - aMax)*jMax*vMax*vMax + jMax*(aMax*vf_vf - aMin*v0_v0)))/(24*aMax*aMin*jMax_jMax*vMax)
t4 = -aMin/jMax
t5 = -(af_af/2 - aMin*aMin - jMax*(vf - vMax))/(aMin*jMax)
t6 = t4 + af/jMax
```
ACC1_VEL (t_acc0 = sqrt(a0_a0/(2*jMax_jMax) + (vMax - v0)/jMax)):
```
t0 = t_acc0 - a0/jMax ; t1 = 0 ; t2 = t_acc0
t3 = -(3*af_p4 - 8*aMin*(af_p3 - a0_p3) - 24*aMin*jMax*(a0*v0 - af*vf) + 6*af_af*(aMin*aMin - 2*jMax*vf) - 12*jMax*(2*aMin*jMax*pd + aMin*aMin*(vf + vMax) + jMax*(vMax*vMax - vf_vf) + aMin*t_acc0*(a0_a0 - 2*jMax*(v0 + vMax))))/(24*aMin*jMax_jMax*vMax)
t4 = -aMin/jMax ; t5 = -(af_af/2 - aMin*aMin + jMax*(vMax - vf))/(aMin*jMax) ; t6 = t4 + af/jMax
```
ACC0_VEL (t_acc1 = sqrt(af_af/(2*jMax_jMax) + (vMax - vf)/jMax)):
```
t0 = (-a0 + aMax)/jMax ; t1 = (a0_a0/2 - aMax*aMax - jMax*(v0 - vMax))/(aMax*jMax) ; t2 = aMax/jMax
t3 = (3*a0_p4 + 8*aMax*(af_p3 - a0_p3) + 24*aMax*jMax*(a0*v0 - af*vf) + 6*a0_a0*(aMax*aMax - 2*jMax*v0) - 12*jMax*(-2*aMax*jMax*pd + aMax*aMax*(v0 + vMax) + jMax*(vMax*vMax - v0_v0) + aMax*t_acc1*(-af_af + 2*(vf + vMax)*jMax)))/(24*aMax*jMax_jMax*vMax)
t4 = t_acc1 ; t5 = 0 ; t6 = t_acc1 + af/jMax
```
VEL (uses both t_acc0, t_acc1):
```
t0 = t_acc0 - a0/jMax ; t1 = 0 ; t2 = t_acc0
t3 = (af_p3 - a0_p3)/(3*jMax_jMax*vMax) + (a0*v0 - af*vf + (af_af*t_acc1 + a0_a0*t_acc0)/2)/(jMax*vMax) - (v0/vMax + 1.0)*t_acc0 - (vf/vMax + 1.0)*t_acc1 + pd/vMax
t4 = t_acc1 ; t5 = 0 ; t6 = t_acc1 + af/jMax
```
The remaining families — `time_acc0_acc1` (no v-plateau; lines 87–129), `time_all_none_acc0_acc1` (the hard one: quartic via `SolveQuartic`; lines 130–286), and the UDUD mirror — are ported in later sub-tasks, fetched verbatim from the same file at dispatch.
- **Sub-tasks at dispatch:** implement one profile type at a time (each its own commit), in Ruckig's order, each with an **oracle test** generated from the installed `ruckig` (`Ruckig(1); Trajectory(1); otg.calculate(inp, traj)` → compare `traj.duration` and `traj.at_time(t)` samples to the SCL profile via `StateAtTime`). Start with `ACC0_ACC1_VEL` (full trapezoid, non-rest) and `NONE`, then fill the rest until all v0.2 parity scenarios pass.
- **Oracle test helper:** reuse `tests/parity/runner_ref.py` patterns to get reference `(duration, p/v/a samples)` for given boundary states.

### Task 7: `ComputeBrakeProfile` FC
- **Files:** `src/blocks/ComputeBrakeProfile.s7dcl`, `tests/unit/test_compute_brake_profile.py`.
- **Interface:** `VAR_IN_OUT profile`, `VAR_INPUT v0,a0,vMax,aMax,jMax : LReal`, `VAR_OUTPUT pBrake,vBrake,aBrake : LReal` (post-brake state), returns `LReal` (brake duration). Port `brake.hpp`: at most 2 segments to bring `|a|≤aMax` then `|v|≤vMax`; fill `profile.brake.t/j` and integrate `brake.a/v/p`.
- **Test (oracle):** out-of-limits `v0>vMax` → post-brake `|v|≤vMax+EPS`, `a→0`, duration matches a hand-computed brake; in-limits input → duration 0, state unchanged.

### Task 8: extend `StateAtTime` for the brake prefix
- **Files:** `src/blocks/StateAtTime.s7dcl` (modify), `tests/unit/test_state_at_time.py` (add cases).
- **Approach:** if `profile.brake.t[0]+t[1] > 0`, evaluate the brake phases first (same cubic eval over `brake.t/j` from `brake.p/v/a[0]`); for `t` beyond the brake span, subtract the brake duration and evaluate the main profile as today.
- **Test:** profile with a 1-segment brake → `t` inside brake returns brake state; `t` after brake returns main-profile state; existing no-brake tests still pass.

### Task 9: confirm `ValidateInput` accepts non-zero target vel/acc
- **Files:** `tests/unit/test_validate_input.py` (add cases). No SCL change expected.
- **Test:** input with `targetVelocity[0]=1.0`, `targetAcceleration[0]=0.5` (finite, within limits) → `RESULT_WORKING`; a non-finite target vel → `RESULT_ERR_NON_FINITE`. If a check wrongly rejects, fix `ValidateInput.s7dcl` minimally.

### Task 10: `RuckigOtg` FB — pass_to_input + brake + extended change detection
- **Files:** `src/blocks/RuckigOtg.s7dcl` (modify), `tests/unit/test_ruckig_otg.py` (add cases).
- **Approach:** per spec §7 — add VAR static `chainPos/chainVel/chainAcc`; seed from `input.current*` on `firstCall`; extend `recompute` to `Δ(targetVel,targetAcc)`; in recompute call `"ComputeBrakeProfile"` when out-of-limits then `"ComputeProfile1Dof"(p0:=chainPos,v0:=chainVel,a0:=chainAcc, pT:=…,vT:=…,aT:=…)`; `duration := brakeDuration + mainDuration`; after evaluate set `chain* := output.new*`.
- **Tests:** (a) moving target (`targetVelocity[0]=0.5`, constant) reached and held at velocity; (b) retarget mid-motion → no setpoint jump between cycles (continuity ≤ `vMax·dt+EPS`); (c) all v0.1 FB tests still pass; (d) first-enable with `currentVelocity[0]>vMax` → brake then converge.

### Task 11: remove superseded v0.1 FCs
- **Files:** delete `src/blocks/ComputeMinDuration.s7dcl`, `src/blocks/ComputeFinalProfile.s7dcl`, `tests/unit/test_compute_min_duration.py`, `tests/unit/test_compute_final_profile.py`.
- **Verify:** `uv run --no-sync pytest tests/unit -p no:cacheprovider -q` — remaining unit suite green (the rest-to-rest behaviour is now covered by `ComputeProfile1Dof` + the parity bench).

### Task 12: +10 v0.2 parity scenarios
- **Files:** `tests/parity/scenarios/v02_*.yaml` (10), no runner change (runners already pass `target_velocity/acceleration`; pass_to_input handled by `otg.update` on the ref side and the FB internally).
- **Scenarios:** non-zero `target_velocity`/`target_acceleration` (approach-and-cruise), negative target velocity, non-rest via a short pre-move, brake-at-first-enable (`current_velocity>vMax`), plus 1–2 retarget-mid-motion (extend `test_parity.py` to optionally drive a time-varying target — a `target_position(t)` ramp — through both runners).
- **Tolerance:** start `1e-9`; if root-solver numerics require, relax per-scenario to `1e-6` and log it (calibrate as in v0.1).

### Task 13: docs, CHANGELOG, version, tag
- **Files:** `README.md` (status → v0.2, features), `CHANGELOG.md` (0.2.0 entry), `examples/` (optional tracking example), bump block `S7_Version` to `0.2.0`.
- **Verify:** full suite green (`uv run --no-sync pytest -p no:cacheprovider -q`); then tag `v0.2.0` after merge (same flow as v0.1).

---

## Self-Review notes

- **Spec coverage:** §4 components → Tasks 2–11; §5 solver → Task 6; §6 roots → Tasks 1–3; §7 FB lifecycle → Task 10; §8 data model → unchanged (no task needed); §9 validation → per-task tests + Task 12; §10 out-of-scope → respected (no multi-axis/velocity-interface/asymmetric tasks); §11 risks → root tests (Tasks 2–3), tie-breaking + tolerance (Tasks 6, 12).
- **Prerequisite:** Task 1 (plc-code `ACOS`) blocks Task 2's three-real-roots branch.
- **Naming consistency:** FC names (`SolveCubic/SolveQuartic/IntegrateProfileStates/CheckProfile/ComputeProfile1Dof/ComputeBrakeProfile`) and the `(roots[0..3], count)` / `(valid Bool)` / `(status Word)` interfaces are used consistently across tasks.
- **Honest deviation:** Tasks 3–13 are interface+test-strategy specs, expanded to full step code just-in-time at dispatch (reference-dependent equations + oracle). Tasks 1–2 are fully expanded.

