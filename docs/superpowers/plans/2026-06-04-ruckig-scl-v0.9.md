# ruckig-scl v0.9 — step1 Two-Step Fallbacks — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Port Ruckig's four closed-form step1 two-step fallback families as a last-resort stage, eliminating the ~3% `RESULT_ERR_SOLVER` failures on arbitrary single-axis states.

**Architecture:** Four new pure FCs — `TwoStepNone`, `TwoStepAcc0`, `TwoStepVel`, `TwoStepAcc1Vel` — each with the **exact `SolveDirection` signature** (so the existing finalize / Block-assembly is reused), each self-skipping when `found`, running its closed-form sub-cases for one signed direction and committing the first `CheckProfile`-valid profile. They are invoked **only when the three main families found nothing**, in the Ruckig dispatch order (interleaved by direction: `none+, none−, acc0+, acc0−, vel+, vel−, acc1vel+, acc1vel−`), inlined into `ComputeProfile1Axis` and `ComputeBlock1Axis`. No dispatcher/helper FC (keeps call depth = the proven `caller → family → CheckProfile → IntegrateProfileStates`).

**Tech Stack:** Siemens SCL (`.s7dcl`) via `siemens-plc-tools` `plc-code`; tests in Python with `pytest` + the Ruckig oracle (PyPI `ruckig` 0.17.3); `uv`. **Run tests as `uv run python -m pytest …`** (bare `uv run pytest` fails to spawn here).

**Spec:** `docs/superpowers/specs/2026-06-04-ruckig-scl-v0.9-design.md`
**Oracle:** `~/.cache/uv/sdists-v9/pypi/ruckig/0.17.3/RHvqwSugDBzM9TJjKLyss/src/src/ruckig/position_third_step1.cpp` — `time_none_two_step` (l.410-444), `time_acc0_two_step` (l.301-371), `time_vel_two_step` (l.372-409), `time_acc1_vel_two_step` (l.287-300), and the `get_profile` fallback dispatch (l.545-560).

---

## Key facts (read before starting)

1. **`SolveDirection.s7dcl`** is the template for every family FC: read it. Each family mirrors its signature (`VAR_IN_OUT profile, bestT[0..6], bestDuration, found, bestSjMax, allDur[0..7], allCount; VAR_INPUT p0,v0,a0,pT,vT,aT, svMax,svMin,saMax,saMin,sjMax`), its precompute region (`pd, v0_v0, vf_vf, a0_a0, a0_p3, a0_p4, af_af, af_p3, af_p4, jMax_jMax`, with `af := aT`, `vf := vT`), its UDDU jerk pattern, and its per-sub-case **commit boilerplate**:
   ```scl
   #ok := "CheckProfile"(profile := #profile, p0 := #p0, v0 := #v0, a0 := #a0,
       pT := #pT, vT := #vT, aT := #aT, vMax := #vMaxMag, aMax := #aMaxMag, tf := -1.0);
   IF #ok THEN
       #dur := 0.0;
       FOR #i := 0 TO 6 DO
           #dur := #dur + #profile.t[#i];
       END_FOR;
       IF #allCount <= 7 THEN
           #allDur[#allCount] := #dur;
           #allCount := #allCount + 1;
       END_IF;
       #bestDuration := #dur;
       #found := true;
       #bestSjMax := #sjMax;
       FOR #i := 0 TO 6 DO
           #bestT[#i] := #profile.t[#i];
       END_FOR;
       RETURN;
   END_IF;
   ```
   (Two-step is last-resort, so the first valid sub-case wins — commit unconditionally and `RETURN`, unlike `SolveDirection`'s min-duration comparison.)
2. **Self-skip**: each family FC begins with `IF #found THEN RETURN; END_IF;` so the interleaved dispatch stops at the first family/direction that succeeds (matches the oracle's `if (profile > start) goto return_block`).
3. **Validity** = the existing `CheckProfile` (tf = −1). The oracle's structural `ReachedLimits` guard is NOT replicated (last-resort feasibility only — see spec §4).
4. **Transpiler** (`docs/PLC_CODE_LIMITATIONS.md`): never divide by a parenthesized product — precompute every compound denominator into a scalar temp (the families have many: `2·a0·jMax`, `2·aMax·jMax`, `jMax·h0`, `3·jMax²·vMax`, `24·aMin·jMax²·vMax`, …). One `:=` per line; `END_IF`/`END_FOR` on their own line (no inline comment); no identifier starting with `IF` or ending in `of`/`Dof`; guard every `SQRT` argument `>= 0`.
5. **UDDU jerk pattern** `j = [sjMax,0,-sjMax,0,-sjMax,0,sjMax]` (set once in each family's precompute, as in `SolveDirection`). All two-step profiles are UDDU.
6. The fallback runs ONLY when `found = false`, so the 245 existing tests are unaffected.

## File structure

| File | New/Modify | Responsibility |
|---|---|---|
| `tests/parity/discover_step1_gaps.py` | Create | seeded fuzz that finds gap states (reproducible); prints fixtures |
| `tests/unit/test_solve_two_step.py` | Create | the discovered gap states now solve (`ComputeProfile1Axis`) |
| `tests/parity/scenarios/v09_*.yaml` | Create | parity scenarios from the discovered gap states |
| `src/blocks/TwoStepNone.s7dcl` | Create | none two-step family (2 sub-cases) |
| `src/blocks/TwoStepAcc0.s7dcl` | Create | acc0 two-step family (4 sub-cases) |
| `src/blocks/TwoStepVel.s7dcl` | Create | vel two-step family (2 sub-cases) |
| `src/blocks/TwoStepAcc1Vel.s7dcl` | Create | acc1_vel two-step family (1 sub-case) |
| `src/blocks/ComputeProfile1Axis.s7dcl` | Modify | invoke the fallback when `found = false` |
| `src/blocks/ComputeBlock1Axis.s7dcl` | Modify | invoke the fallback when `found = false` |
| `CHANGELOG.md`, `README.md`, `pyproject.toml`, `uv.lock` | Modify | release 0.9.0 |

---

## Task 1: Discover gap states + failing fixtures

**Files:**
- Create: `tests/parity/discover_step1_gaps.py`
- Create: `tests/parity/scenarios/v09_01..v09_NN.yaml` (one per discovered state)
- Create: `tests/unit/test_solve_two_step.py` (failing)

- [ ] **Step 1: Write the discovery script**

Create `tests/parity/discover_step1_gaps.py` — a seeded fuzz that builds the
`ComputeProfile1Axis` harness ONCE (reuse via `harness.reset()` per sample — do
NOT recreate it, that re-transpiles and is far too slow), draws random
single-axis states within fixed limits (`vMax=3, aMax=5, jMax=10`), and collects
states where the SCL returns `RESULT_ERR_SOLVER` (`0x8602`) while the Ruckig
oracle returns `Result.Working`. For each gap state, also record the oracle's
reached-limits (from `out.trajectory` / the profile) to label the likely family.

```python
"""Seeded discovery of step1 gap states (SCL errors, oracle solves).

Run:  uv run python -m tests.parity.discover_step1_gaps
Reproducible (fixed seed). Prints YAML-ready gap states; a representative subset
is frozen as v09_* parity scenarios + test_solve_two_step fixtures.
"""
from __future__ import annotations
import random
from pathlib import Path
from plc_code.executor import create_harness
from plc_code.executor.runtime import PLCRuntime
import ruckig

RESULT_ERR_SOLVER = 0x8602
ROOT = Path(__file__).resolve().parent.parent.parent
SP = [ROOT / "src/blocks", ROOT / "src/data-types", ROOT / "src/data-blocks"]
VMAX, AMAX, JMAX = 3.0, 5.0, 10.0


def _profile():
    return {"t": [0.0]*7, "j": [0.0]*7, "a": [0.0]*8, "v": [0.0]*8, "p": [0.0]*8,
            "direction": 0, "controlSigns": 0,
            "brake": {"t": [0.0, 0.0], "j": [0.0, 0.0], "a": [0.0]*3, "v": [0.0]*3, "p": [0.0]*3}}


def main(n=4000, want=10, seed=12345):
    random.seed(seed)
    rt = PLCRuntime(block_search_paths=SP)
    h = create_harness(ROOT / "src/blocks/ComputeProfile1Axis.s7dcl", runtime=rt)
    otg = ruckig.Ruckig(1, 0.01)
    gaps = []
    for _ in range(n):
        s = dict(p0=0.0, pT=round(random.uniform(-3, 3), 3),
                 v0=round(random.uniform(-3, 3), 3), vT=round(random.uniform(-3, 3), 3),
                 a0=round(random.uniform(-5, 5), 3), aT=round(random.uniform(-5, 5), 3))
        h.reset()
        h.set_inputs(profile=_profile(), p0=s["p0"], v0=s["v0"], a0=s["a0"],
                     pT=s["pT"], vT=s["vT"], aT=s["aT"], vMax=VMAX, aMax=AMAX, jMax=JMAX)
        h.execute()
        if h.get_output("ComputeProfile1Axis") == RESULT_ERR_SOLVER:
            inp = ruckig.InputParameter(1); out = ruckig.OutputParameter(1)
            inp.current_position = [s["p0"]]; inp.current_velocity = [s["v0"]]; inp.current_acceleration = [s["a0"]]
            inp.target_position = [s["pT"]]; inp.target_velocity = [s["vT"]]; inp.target_acceleration = [s["aT"]]
            inp.max_velocity = [VMAX]; inp.max_acceleration = [AMAX]; inp.max_jerk = [JMAX]
            try:
                if otg.update(inp, out) == ruckig.Result.Working:
                    gaps.append((s, round(out.trajectory.duration, 4)))
            except Exception:
                pass
        if len(gaps) >= want:
            break
    for s, dur in gaps:
        print(f"# dur={dur}\n"
              f"current_velocity: [{s['v0']}]\ncurrent_acceleration: [{s['a0']}]\n"
              f"target_position: [{s['pT']}]\ntarget_velocity: [{s['vT']}]\n"
              f"target_acceleration: [{s['aT']}]\n---")
    print(f"# tried, found {len(gaps)}")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Run discovery and capture gap states**

Run: `uv run python -m tests.parity.discover_step1_gaps`
Expected: prints ≥6 gap states (each with a non-zero `v0`/`a0` typically). Record
them. (If it finds 0, widen the ranges or increase `n` — but ~3% means a handful
within a few hundred samples.)

- [ ] **Step 3: Freeze representative gap states as failing parity scenarios**

For ~6 captured states, create `tests/parity/scenarios/v09_0k_step1_gap.yaml`
(numbered `v09_01`…), each like (filling in the captured numbers):

```yaml
name: v0.9 step1 two-step gap state k
n_dofs: 1
current_velocity: [<v0>]
current_acceleration: [<a0>]
target_position: [<pT>]
target_velocity: [<vT>]
target_acceleration: [<aT>]
max_velocity: [3.0]
max_acceleration: [5.0]
max_jerk: [10.0]
cycle_time: 0.010
```

- [ ] **Step 4: Write the failing unit test**

Create `tests/unit/test_solve_two_step.py` with the captured states as fixtures:

```python
"""Step1 two-step fallback: states that errored before now solve."""
import pytest

RESULT_WORKING = 0x7000

# (v0, a0, pT, vT, aT) captured by discover_step1_gaps.py (seed 12345)
GAP_STATES = [
    # (<v0>, <a0>, <pT>, <vT>, <aT>),   # <-- fill from discovery, ~6 states
]


@pytest.fixture
def harness(make_harness):
    return make_harness("ComputeProfile1Axis.s7dcl")


def _profile():
    return {"t": [0.0]*7, "j": [0.0]*7, "a": [0.0]*8, "v": [0.0]*8, "p": [0.0]*8,
            "direction": 0, "controlSigns": 0,
            "brake": {"t": [0.0, 0.0], "j": [0.0, 0.0], "a": [0.0]*3, "v": [0.0]*3, "p": [0.0]*3}}


@pytest.mark.parametrize("v0,a0,pT,vT,aT", GAP_STATES)
def test_gap_state_now_solves(harness, v0, a0, pT, vT, aT):
    prof = _profile()
    harness.reset()
    harness.set_inputs(profile=prof, p0=0.0, v0=v0, a0=a0, pT=pT, vT=vT, aT=aT,
                       vMax=3.0, aMax=5.0, jMax=10.0)
    harness.execute()
    assert harness.get_output("ComputeProfile1Axis") == RESULT_WORKING
    out = harness.get_var("profile")
    assert out.v[7] == pytest.approx(vT, abs=1e-6)
    assert out.a[7] == pytest.approx(aT, abs=1e-6)
    assert out.p[7] == pytest.approx(pT, abs=1e-6)
```

- [ ] **Step 5: Confirm they fail**

Run: `uv run python -m pytest tests/unit/test_solve_two_step.py -q` → FAIL
(`RESULT_ERR_SOLVER`). Run `uv run python -m pytest tests/parity -q -k v09` → FAIL.

- [ ] **Step 6: Commit (fixtures + failing tests)**

```bash
git add tests/parity/discover_step1_gaps.py tests/unit/test_solve_two_step.py tests/parity/scenarios/v09_*.yaml
git commit -m "v0.9 T1: discover step1 gap states + failing fixtures/scenarios"
```

> Note: committing failing tests here is intentional — they are the red half of
> the TDD cycle completed in Task 2. If your workflow forbids committing red,
> fold Task 1's files into Task 2's commit instead.

---

## Task 2: The four two-step family FCs + wiring

**Files:**
- Create: `src/blocks/TwoStepNone.s7dcl`, `TwoStepAcc0.s7dcl`, `TwoStepVel.s7dcl`, `TwoStepAcc1Vel.s7dcl`
- Modify: `src/blocks/ComputeProfile1Axis.s7dcl`, `src/blocks/ComputeBlock1Axis.s7dcl`

Each family FC: header (S7_Version "0.9.0"), the `SolveDirection` signature, the
`IF #found THEN RETURN; END_IF;` self-skip, the precompute region (copy from
`SolveDirection`), the UDDU jerk pattern, then the sub-cases — each setting
`profile.t[0..6]` from the oracle formula (with compound denominators
precomputed) and ending with the commit boilerplate from Key Fact #1. Translate
the oracle C++ verbatim; the parity tests are the ground truth.

- [ ] **Step 1: `TwoStepNone.s7dcl` (template — complete code)**

```scl
{
    S7_Author := "Camille Martin";
    S7_EditorMode := "SCL";
    S7_Family := "Ruckig";
    S7_Optimized := "TRUE";
    S7_Version := "0.9.0"
}
FUNCTION "TwoStepNone" : Void
    VAR_IN_OUT
        profile      : _.typeProfile;
        bestT        : Array[0..6] of LReal;
        bestDuration : LReal;
        found        : Bool;
        bestSjMax    : LReal;
        allDur       : Array[0..7] of LReal;
        allCount     : Int;
    END_VAR
    VAR_INPUT
        p0 : LReal; v0 : LReal; a0 : LReal; pT : LReal; vT : LReal; aT : LReal;
        svMax : LReal; svMin : LReal; saMax : LReal; saMin : LReal; sjMax : LReal;
    END_VAR
    VAR_TEMP
        vMaxMag : LReal; aMaxMag : LReal;
        a0_a0 : LReal; af_af : LReal;
        h0 : LReal; radic : LReal; ok : Bool; dur : LReal; i : DInt;
    END_VAR

    { S7_Language := "SCL" }
    NETWORK
        REGION Block header
            //==========================================================
            // Title:      TwoStepNone
            // Function:   step1 last-resort fallback (NONE shape). Port of Ruckig
            //             time_none_two_step (position_third_step1.cpp l.410-444).
            //             Runs ONLY when the 3 main families found nothing; commits
            //             the first CheckProfile-valid sub-case and returns.
            // Family:     Ruckig
            // Author:     Camille Martin
            //==========================================================
        END_REGION

        REGION Skip if already solved
            IF #found THEN
                RETURN;
            END_IF;
        END_REGION

        REGION Precompute + UDDU jerk
            #vMaxMag := ABS(#svMax);
            #aMaxMag := ABS(#saMax);
            #a0_a0 := #a0 * #a0;
            #af_af := #aT * #aT;
            #profile.j[0] := #sjMax;
            #profile.j[1] := 0.0;
            #profile.j[2] := -#sjMax;
            #profile.j[3] := 0.0;
            #profile.j[4] := -#sjMax;
            #profile.j[5] := 0.0;
            #profile.j[6] := #sjMax;
            #profile.direction := 1;
            #profile.controlSigns := 0;
        END_REGION

        REGION Sub-case 1: two-step
            // h0 = sqrt((a0^2+af^2)/2 + jMax*(vf-v0)) * sgn(jMax)
            #radic := (#a0_a0 + #af_af) / 2.0 + #sjMax * (#vT - #v0);
            IF #radic >= 0.0 THEN
                #h0 := SQRT(#radic);
                IF #sjMax < 0.0 THEN
                    #h0 := -#h0;
                END_IF;
                #profile.t[0] := (#h0 - #a0) / #sjMax;
                #profile.t[1] := 0.0;
                #profile.t[2] := (#h0 - #aT) / #sjMax;
                #profile.t[3] := 0.0;
                #profile.t[4] := 0.0;
                #profile.t[5] := 0.0;
                #profile.t[6] := 0.0;
                #ok := "CheckProfile"(profile := #profile, p0 := #p0, v0 := #v0, a0 := #a0,
                    pT := #pT, vT := #vT, aT := #aT, vMax := #vMaxMag, aMax := #aMaxMag, tf := -1.0);
                IF #ok THEN
                    #dur := 0.0;
                    FOR #i := 0 TO 6 DO
                        #dur := #dur + #profile.t[#i];
                    END_FOR;
                    IF #allCount <= 7 THEN
                        #allDur[#allCount] := #dur;
                        #allCount := #allCount + 1;
                    END_IF;
                    #bestDuration := #dur;
                    #found := true;
                    #bestSjMax := #sjMax;
                    FOR #i := 0 TO 6 DO
                        #bestT[#i] := #profile.t[#i];
                    END_FOR;
                    RETURN;
                END_IF;
            END_IF;
        END_REGION

        REGION Sub-case 2: single-step
            #profile.t[0] := (#aT - #a0) / #sjMax;
            #profile.t[1] := 0.0;
            #profile.t[2] := 0.0;
            #profile.t[3] := 0.0;
            #profile.t[4] := 0.0;
            #profile.t[5] := 0.0;
            #profile.t[6] := 0.0;
            #ok := "CheckProfile"(profile := #profile, p0 := #p0, v0 := #v0, a0 := #a0,
                pT := #pT, vT := #vT, aT := #aT, vMax := #vMaxMag, aMax := #aMaxMag, tf := -1.0);
            IF #ok THEN
                #dur := 0.0;
                FOR #i := 0 TO 6 DO
                    #dur := #dur + #profile.t[#i];
                END_FOR;
                IF #allCount <= 7 THEN
                    #allDur[#allCount] := #dur;
                    #allCount := #allCount + 1;
                END_IF;
                #bestDuration := #dur;
                #found := true;
                #bestSjMax := #sjMax;
                FOR #i := 0 TO 6 DO
                    #bestT[#i] := #profile.t[#i];
                END_FOR;
                RETURN;
            END_IF;
        END_REGION
    END_NETWORK
END_FUNCTION
```

- [ ] **Step 2: `TwoStepVel.s7dcl`**

Same skeleton (header/sig/self-skip/precompute incl. `jMax_jMax`/UDDU). Shared
`h1 = sqrt(af²/(2·jMax²) + (vMax−vf)/jMax)` (guard radicand ≥ 0; precompute
`2·jMax²`). Two four-step sub-cases from the oracle (`time_vel_two_step`,
l.372-409); precompute `3·jMax²·vMax` and `jMax·vMax` denominators. Sub-case 1:
```
t0 = -a0/jMax; t1=0; t2=0;
t3 = (af³-a0³)/(3·jMax²·vMax) + (a0·v0 - af·vf + (af²·h1)/2)/(jMax·vMax) - (vf/vMax + 1)·h1 + pd/vMax;
t4 = h1; t5 = 0; t6 = h1 + af/jMax;
```
Sub-case 2:
```
t0=0; t1=0; t2 = a0/jMax;
t3 = (af³-a0³)/(3·jMax²·vMax) + (a0·v0 - af·vf + (af²·h1 + a0³/jMax)/2)/(jMax·vMax) - (v0/vMax + 1)·a0/jMax - (vf/vMax + 1)·h1 + pd/vMax;
t4 = h1; t5 = 0; t6 = h1 + af/jMax;
```
(`svMax` is the signed `vMax`; `vf := vT`, `af := aT`, `pd := pT−p0`.) Each
sub-case ends with the commit boilerplate, then `RETURN`.

- [ ] **Step 3: `TwoStepAcc0.s7dcl`**

Four sub-cases from `time_acc0_two_step` (l.301-371). Precompute denominators
(`2·a0·jMax`, `2·aMax·jMax`, `jMax·h0` with `h0 = 3·(af²−a0²+2·jMax·(v0+vf))`,
etc.). Formulas (signed `aMax := saMax`, `aMin := saMin`):
- Sub-case 1 (two-step): `t0=0; t1=(af²-a0²+2·jMax·(vf-v0))/(2·a0·jMax); t2=(a0-af)/jMax; t3..6=0`.
- Sub-case 2 (three-step, removed pf): `t0=(-a0+aMax)/jMax; t1=(a0²+af²-2·aMax²+2·jMax·(vf-v0))/(2·aMax·jMax); t2=(-af+aMax)/jMax; t3..6=0`.
- Sub-case 3 (three-step, removed aMax): `h0=3·(af²-a0²+2·jMax·(v0+vf)); h1=sqrt(2·(2·h2²+h0·(…)))·sgn(jMax)` per oracle l.336; `t0=(4·af³+2·a0³-6·a0·af²+12·jMax²·pd+12·(af-a0)·jMax·vf + h1)/(2·jMax·h0); t1=-h1/(jMax·h0); t2=(-4·a0³-2·af³+6·a0²·af+12·jMax²·pd-12·(af-a0)·jMax·v0 + h1)/(2·jMax·h0); t3..6=0` (guard the sqrt radicand ≥ 0; `h2 = a0³+2·af³+6·jMax²·pd+6·(af-a0)·jMax·vf-3·a0·af²`).
- Sub-case 4 (three-step, t=(aMax−aMin)/jMax): `t=(aMax-aMin)/jMax; t0=(-a0+aMax)/jMax; t1=(a0²-af²)/(2·aMax·jMax)+(vf-v0+jMax·t²)/aMax-2·t; t2=t; t3..5=0; t6=(af-aMin)/jMax`.
Each ends with the commit boilerplate + `RETURN`. (Quote the oracle l.301-371 to
transcribe sub-case 3's `h1` radicand exactly.)

- [ ] **Step 4: `TwoStepAcc1Vel.s7dcl`**

One sub-case from `time_acc1_vel_two_step` (l.287-300). Needs `af_p3, af_p4, a0_p3`.
```
t0=0; t1=0; t2=a0/jMax;
t3 = -(3·af⁴ - 8·aMin·(af³-a0³) - 24·aMin·jMax·(a0·v0-af·vf) + 6·af²·(aMin²-2·jMax·vf) - 12·jMax·(2·aMin·jMax·pd + aMin²·(vf+vMax) + jMax·(vMax²-vf²) + aMin·a0·(a0²-2·jMax·(v0+vMax))/jMax)) / (24·aMin·jMax²·vMax);
t4 = -aMin/jMax;
t5 = -(af²/2 - aMin² + jMax·(vMax-vf))/(aMin·jMax);
t6 = t4 + af/jMax;
```
(`aMin := saMin`, `vMax := svMax`, `vf := vT`, `af := aT`, `pd := pT−p0`.)
Precompute the denominators (`24·aMin·jMax²·vMax`, `aMin·jMax`, and the inner
`aMin·a0·(…)/jMax` — note the inner `/jMax` is a scalar division, fine; the outer
`(…)/(24·aMin·jMax²·vMax)` needs the precomputed scalar). Commit + `RETURN`.

- [ ] **Step 5: Wire the fallback into `ComputeProfile1Axis`**

In `src/blocks/ComputeProfile1Axis.s7dcl`, between `REGION Solve mirror direction`
and `REGION Finalise winner or fail`, insert:

```scl
        REGION Two-step fallbacks (only if no main-family profile)
            IF NOT #found THEN
                "TwoStepNone"(profile := #profile, bestT := #bestT, bestDuration := #bestDuration,
                    found := #found, bestSjMax := #bestSjMax, allDur := #allDur, allCount := #allCount,
                    p0 := #p0, v0 := #v0, a0 := #a0, pT := #pT, vT := #vT, aT := #aT,
                    svMax := #vMax, svMin := -#vMax, saMax := #aMax, saMin := -#aMax, sjMax := #jMax);
                "TwoStepNone"(profile := #profile, bestT := #bestT, bestDuration := #bestDuration,
                    found := #found, bestSjMax := #bestSjMax, allDur := #allDur, allCount := #allCount,
                    p0 := #p0, v0 := #v0, a0 := #a0, pT := #pT, vT := #vT, aT := #aT,
                    svMax := -#vMax, svMin := #vMax, saMax := -#aMax, saMin := #aMax, sjMax := -#jMax);
                "TwoStepAcc0"( … svMax := #vMax, svMin := -#vMax, saMax := #aMax, saMin := -#aMax, sjMax := #jMax);
                "TwoStepAcc0"( … svMax := -#vMax, svMin := #vMax, saMax := -#aMax, saMin := #aMax, sjMax := -#jMax);
                "TwoStepVel"(  … svMax := #vMax, svMin := -#vMax, saMax := #aMax, saMin := -#aMax, sjMax := #jMax);
                "TwoStepVel"(  … svMax := -#vMax, svMin := #vMax, saMax := -#aMax, saMin := #aMax, sjMax := -#jMax);
                "TwoStepAcc1Vel"( … svMax := #vMax, svMin := -#vMax, saMax := #aMax, saMin := -#aMax, sjMax := #jMax);
                "TwoStepAcc1Vel"( … svMax := -#vMax, svMin := #vMax, saMax := -#aMax, saMin := #aMax, sjMax := -#jMax);
            END_IF;
        END_REGION
```
(Fill each `…` with the same `profile/bestT/bestDuration/found/bestSjMax/allDur/allCount/p0..aT` VAR_IN_OUT+state args as the `TwoStepNone` calls — only the signed `svMax/svMin/saMax/saMin/sjMax` differ by direction. Order = the oracle's interleaved dispatch.) The existing `REGION Finalise` then sets the UDDU jerk from `bestSjMax` and integrates `bestT`, unchanged.

- [ ] **Step 6: Wire the same fallback into `ComputeBlock1Axis`**

In `src/blocks/ComputeBlock1Axis.s7dcl`, between `REGION Run step1 families…`
(the two `SolveDirection` calls) and `REGION Fail if no profile`, insert the
identical `IF NOT #found THEN … END_IF;` block of 8 calls (same args). The
existing sort/dedup/`calculate_block` then derives `tMin` from the single fallback
duration. Note `ComputeBlock1Axis` already has `bestT`/`bestSjMax`/`allDur`/
`allCount` temps (it passes them to `SolveDirection`), so the calls compile.

- [ ] **Step 7: Verify red → green**

Run: `uv run python -m pytest tests/unit/test_solve_two_step.py -q` → all PASS.
Run: `uv run python -m pytest tests/parity -q -k v09` → all PASS (oracle parity at
the floating-point floor).
Run: `uv run python -m pytest -q` → all green (245 existing + new). The 245 are
unaffected (fallback runs only when `found = false`).

If a `v09` scenario still errors, the gap state needs a family not yet correct —
dump which sub-case the oracle uses (its reached-limits) and check that family's
transcription against the oracle line-by-line. If it solves but diverges from the
oracle, check the interleaved call order and the per-sub-case formulas. Do NOT
relax tolerances.

- [ ] **Step 8: Commit**

```bash
git add src/blocks/TwoStepNone.s7dcl src/blocks/TwoStepAcc0.s7dcl src/blocks/TwoStepVel.s7dcl src/blocks/TwoStepAcc1Vel.s7dcl src/blocks/ComputeProfile1Axis.s7dcl src/blocks/ComputeBlock1Axis.s7dcl
git commit -m "v0.9 T2: step1 two-step fallback families (none/acc0/vel/acc1_vel) + wiring"
```

---

## Task 3: Release 0.9.0

**Files:** `CHANGELOG.md`, `README.md`, `pyproject.toml`, `uv.lock`

- [ ] **Step 1: CHANGELOG** — add a top entry:

```markdown
## [0.9.0] - 2026-06-04

### Added
- **step1 two-step fallback families** (`TwoStepNone`, `TwoStepAcc0`,
  `TwoStepVel`, `TwoStepAcc1Vel`). Ported from Ruckig's closed-form
  `time_*_two_step`, run as a last resort (only when the three main step1
  families find no profile), in Ruckig's interleaved-by-direction dispatch
  order. Closes the long-standing ~3% `RESULT_ERR_SOLVER` gap on arbitrary
  single-axis states; benefits the multi-axis Block and the v0.8 post-brake Block
  too. New `v09_*` parity scenarios from a seeded gap-state discovery.

### Known limitations
- step2 short-move-stretched divergence (a valid-but-different shape vs Ruckig
  when a small displacement is re-timed to a much larger tf) remains — next
  version.
```

- [ ] **Step 2: README** — Status → v0.9.0; add `## Features (v0.9)`; in
  Known limitations, REMOVE the "step1 two-step fallbacks not yet ported" bullet
  and KEEP/ADD the step2 short-move bullet; Architecture table: add the four
  `TwoStep*` FC rows; add the v0.9 spec link; Roadmap: mark v0.9 *(this release)*,
  next = step2 completeness; update the parity-scenario count (`ls
  tests/parity/scenarios/*.yaml | wc -l`).

- [ ] **Step 3: Version** — `pyproject.toml` `0.8.0` → `0.9.0`; run `uv lock`.

- [ ] **Step 4: Full suite** — `uv run python -m pytest -q` → all green.

- [ ] **Step 5: Commit**

```bash
git add CHANGELOG.md README.md pyproject.toml uv.lock
git commit -m "v0.9 T3: release 0.9.0 (docs + version)"
```

---

## Final integration (handled by the execution skill)

Hand off to `superpowers:finishing-a-development-branch`: merge `--no-ff`, tag
`v0.9.0`, push. A final opus review should: verify each family's formulas against
the oracle, confirm the interleaved dispatch order matches Ruckig (parity on
states where multiple families are valid), confirm the fallback never perturbs
the in-limit/main-family cases (245 regression), and fuzz additional gap states
(beyond the frozen v09_*) to confirm the ~3% gap is actually closed (report the
residual error rate).

---

## Self-review (planner)

- **Spec coverage:** four families ported (T2 steps 1-4), invoked last-resort in
  both callers in interleaved order (T2 steps 5-6), zero-limits excluded (not
  ported — validation rejects it), discovery + fixtures + parity (T1), regression
  safety (fallback gated on `NOT found`), release (T3). All spec sections map.
- **No placeholders:** `TwoStepNone` is complete; the other three give the exact
  oracle formulas + the shared skeleton/commit boilerplate to transcribe — the
  oracle file is cited line-precise and parity is the gate. (If strict
  full-code-per-step is required, expand T2 steps 2-4 from the cited oracle lines
  before dispatching.)
- **Consistency:** every family uses the `SolveDirection` signature, the same
  commit boilerplate (Key Fact #1), the self-skip, and the UDDU jerk; the wiring
  passes identical args differing only by signed limits; the interleaved order
  matches the oracle `get_profile` dispatch (l.545-560).
- **Refinement flagged:** the spec's single-`SolveTwoStepDir`-called-twice was
  refined to four per-family FCs invoked in interleaved-by-direction order, to
  match Ruckig's dispatch exactly (parity-safe when multiple families are valid
  for one state) and to avoid duplicating each long formula.
```
