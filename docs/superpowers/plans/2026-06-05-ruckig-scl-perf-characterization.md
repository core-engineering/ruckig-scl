# ruckig-scl v0.11 — Performance Characterization & Benchmark Tooling — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans. Steps use `- [ ]` checkboxes.

**Goal:** Characterize the per-cycle cost of `RuckigOtg` with an SCL-operation proxy and produce a committed cost report, plus a TIA-importable benchmark template + PLCSIM measurement methodology. No solver change.

**Architecture:** (1) A Python profiler hooking the executor's FC-dispatch (`PLCRuntime.call_named_block`) + `math.sqrt` to count SCL operations per `update` across representative scenarios → `docs/PERFORMANCE.md`. (2) A TIA-only benchmark FB template + methodology README. Touches only `tests/perf/`, `benchmark/`, docs — the 261 solver tests are untouched.

**Tech Stack:** Python (`uv`), the `plc-code` executor; SCL template (TIA-side). **Tests: `uv run python -m pytest …`** (NOT bare pytest).

**Spec:** `docs/superpowers/specs/2026-06-05-ruckig-scl-perf-characterization-design.md`

## Key facts (validated during planning)
- Hook `PLCRuntime.call_named_block(self, name, ...)` (in `plc_code.executor.runtime`) to count FC calls by `name`. Monkeypatch `math.sqrt` to count SCL `SQRT`. Both are SCL-semantic counts, immune to interpreter overhead. (`sys.settrace` is NOT usable — interpreter overhead dominates.)
- Validated sample: 4-DoF recompute = 150 sqrt; FC ranking PolyEval 75 / IntegrateProfileStates 56 / CheckProfile 52 / SolveQuartic 39 / SolveCubic 28 / IsFiniteLreal 24 / … . Steady-state = 0 sqrt, ~30 FC calls (24 IsFiniteLreal + 4 StateAtTime + 1 ValidateInput + 1 AdvanceTime).
- A scenario's **recompute** cycle = the first `execute()` after `reset()` (firstCall forces it). Its **steady-state** cycle = a second `execute()` with unchanged input.
- `plc-code` can NOT nest FBs (`instOtg : "RuckigOtg";` fails to parse) → the benchmark FB is a TIA-only template, not transpiled here.
- `tests/parity/discover_*` run as `uv run python -m tests.parity.X`, so `tests/` is importable as a namespace package; `tests/perf/` will work the same.

## File structure

| File | New | Responsibility |
|---|---|---|
| `tests/perf/profile_ruckig.py` | Create | FC-call+sqrt profiler; prints + writes `docs/PERFORMANCE.md` |
| `tests/perf/test_profile_sanity.py` | Create | sanity test (steady-state ≪ 4-DoF recompute) |
| `docs/PERFORMANCE.md` | Create (generated) | committed cost report |
| `benchmark/BenchRuckigOtg.s7dcl` | Create | TIA-only benchmark FB template |
| `benchmark/README.md` | Create | PLCSIM measurement methodology |
| `CHANGELOG.md`, `README.md`, `pyproject.toml`, `uv.lock` | Modify | release 0.11.0 |

---

## Task 1: Operation-count profiler + cost report

**Files:** `tests/perf/profile_ruckig.py`, `tests/perf/test_profile_sanity.py`, `docs/PERFORMANCE.md`.

- [ ] **Step 1: Write the profiler**

Create `tests/perf/profile_ruckig.py`:

```python
"""Operation-count proxy profiler for RuckigOtg.

Counts SCL-level operations per update — FC calls (by block name) and SQRT calls
— by hooking the plc-code executor. NOT microseconds: a relative, interpreter-
overhead-immune cost proxy for hot-path ranking and steady-state vs recompute.
Real cycle-time is measured on PLCSIM (see benchmark/README.md).

Run:  uv run python -m tests.perf.profile_ruckig          # prints + writes docs/PERFORMANCE.md
"""
from __future__ import annotations
import math
from pathlib import Path

from plc_code.executor import create_harness
from plc_code.executor.runtime import PLCRuntime

ROOT = Path(__file__).resolve().parent.parent.parent
SP = [ROOT / "src/blocks", ROOT / "src/data-types", ROOT / "src/data-blocks"]
FB = ROOT / "src/blocks/RuckigOtg.s7dcl"


def _input(n_dofs, iface=0, sync=2):
    return {
        "currentPosition": [0.0] * 4, "currentVelocity": [0.0] * 4,
        "currentAcceleration": [0.0] * 4,
        "targetPosition": [1.0, 0.7, 0.5, 0.3], "targetVelocity": [0.0] * 4,
        "targetAcceleration": [0.0] * 4,
        "maxVelocity": [2.0] * 4, "maxAcceleration": [5.0] * 4, "maxJerk": [10.0] * 4,
        "enabled": [d < n_dofs for d in range(4)], "nDofs": n_dofs,
        "minimumDuration": -1.0, "controlInterface": iface, "synchronization": sync,
        "perDofSynchronization": [-1] * 4, "durationDiscretization": 0,
    }


class _Counter:
    def __init__(self):
        self.calls = {}
        self.sqrt = 0
        self._orig_call = None
        self._orig_sqrt = None

    def __enter__(self):
        self.calls = {}
        self.sqrt = 0
        self._orig_call = PLCRuntime.call_named_block
        self._orig_sqrt = math.sqrt
        counter = self

        def hook(self_rt, name, *a, **k):
            counter.calls[name] = counter.calls.get(name, 0) + 1
            return counter._orig_call(self_rt, name, *a, **k)

        def msqrt(x):
            counter.sqrt += 1
            return counter._orig_sqrt(x)

        PLCRuntime.call_named_block = hook
        math.sqrt = msqrt
        return self

    def __exit__(self, *exc):
        PLCRuntime.call_named_block = self._orig_call
        math.sqrt = self._orig_sqrt

    @property
    def total_calls(self):
        return sum(self.calls.values())


def _measure(label, n_dofs, iface=0):
    """Return (recompute_counter, steady_counter) for one scenario."""
    h = create_harness(FB, runtime=PLCRuntime(block_search_paths=list(SP)))
    inp = _input(n_dofs, iface=iface)
    h.set_inputs(enable=True, input=inp, cycleTime=0.01, reset=False)
    h.reset()
    with _Counter() as recompute:
        h.execute()            # firstCall -> recompute
    with _Counter() as steady:
        h.execute()            # no change -> steady-state
    return label, recompute, steady


SCENARIOS = [
    ("1-DoF position", 1, 0),
    ("2-DoF position", 2, 0),
    ("4-DoF position", 4, 0),
    ("4-DoF velocity", 4, 1),
]


def main():
    rows = [_measure(*s) for s in SCENARIOS]

    lines = ["# Performance characterization (operation-count proxy)", "",
             "Counts of SCL-level operations per `RuckigOtg` update, measured by",
             "hooking the `plc-code` executor (FC-dispatch + `math.sqrt`). These are a",
             "**relative cost proxy** (not microseconds) for hot-path ranking and the",
             "steady-state vs recompute split. Real cycle-time is measured on PLCSIM",
             "Advanced (see `benchmark/README.md`).", "",
             "## Per-cycle cost", "",
             "| Scenario | recompute: FC calls | recompute: sqrt | steady: FC calls | steady: sqrt |",
             "|---|---|---|---|---|"]
    for label, rc, st in rows:
        lines.append(f"| {label} | {rc.total_calls} | {rc.sqrt} | {st.total_calls} | {st.sqrt} |")

    # hot-path ranking from the worst case (4-DoF position recompute)
    worst = next(rc for label, rc, st in rows if label == "4-DoF position")
    lines += ["", "## Hot-path ranking (4-DoF position recompute, FC calls)", ""]
    for name, c in sorted(worst.calls.items(), key=lambda x: -x[1]):
        lines.append(f"- `{name}`: {c}")
    lines += ["", "## Notes",
              "- Steady-state (no retarget) is the common cycle: 0 sqrt, a small fixed",
              "  FC count (per-axis `StateAtTime` + `AdvanceTime` + input validation).",
              "- Recompute (a retarget cycle) is the worst case: per-axis brake + Block +",
              "  `Synchronize` + step2; the root solvers (`PolyEval`/`SolveQuartic`/",
              "  `SolveCubic`/`CheckProfile`) dominate.",
              "- Optimization is deferred to a later, measurement-driven version (this",
              "  report is the 'measure first' step); real µs come from PLCSIM.", ""]

    text = "\n".join(lines)
    print(text)
    (ROOT / "docs/PERFORMANCE.md").write_text(text)
    return rows


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Run it and generate the report**

Run: `uv run python -m tests.perf.profile_ruckig`
Expected: prints the tables and writes `docs/PERFORMANCE.md`. Sanity: steady-state FC calls ≪ 4-DoF recompute FC calls; steady-state sqrt = 0; 4-DoF recompute sqrt ≈ 150.

- [ ] **Step 3: Sanity test**

Create `tests/perf/test_profile_sanity.py`:

```python
"""Sanity checks for the perf profiler (ordering, not absolute numbers)."""
from tests.perf.profile_ruckig import _measure


def test_steady_state_is_cheaper_than_recompute():
    _, rc, st = _measure("4-DoF position", 4, 0)
    assert st.sqrt == 0
    assert st.total_calls < rc.total_calls
    assert rc.sqrt > 0


def test_velocity_recompute_runs():
    _, rc, _ = _measure("4-DoF velocity", 4, 1)
    assert rc.total_calls > 0
```

Run: `uv run python -m pytest tests/perf/test_profile_sanity.py -q` → 2 passed. If `tests/perf` isn't importable, add an empty `tests/perf/__init__.py` (match how `tests/parity` is set up).

- [ ] **Step 4: Commit**

```bash
git add tests/perf/ docs/PERFORMANCE.md
git commit -m "v0.11 T1: operation-count proxy profiler + docs/PERFORMANCE.md cost report"
```

---

## Task 2: PLCSIM benchmark template + methodology

**Files:** `benchmark/BenchRuckigOtg.s7dcl`, `benchmark/README.md`.

(Reminder: the FB cannot be transpiled here — plc-code can't nest FBs. Author it carefully against `RuckigOtg`'s real call signature, read from `src/blocks/RuckigOtg.s7dcl` VAR_INPUT/VAR_OUTPUT.)

- [ ] **Step 1: Author the benchmark FB template**

Create `benchmark/BenchRuckigOtg.s7dcl`:

```scl
{
    S7_Author := "Camille Martin";
    S7_EditorMode := "SCL";
    S7_Family := "Ruckig";
    S7_Optimized := "TRUE";
    S7_Version := "0.11.0"
}
FUNCTION_BLOCK "BenchRuckigOtg"
    // PLCSIM Advanced benchmark driver for RuckigOtg. Call from a cyclic OB and
    // time it externally (RUNTIME in the OB, or TIA online trace / OB cycle time);
    // divide the elapsed time by `iterations` to get per-update cost. The checksum
    // output prevents the compiler from eliding the calls.
    // NOTE: this FB nests a RuckigOtg instance; it is a TIA Portal artifact and is
    // NOT exercised by the SCL->Python test bench (plc-code does not nest FBs).
    VAR_INPUT
        iterations : Int := 1000;     // updates per call
        nDofs      : Int := 4;        // 1..4
        mode       : Int := 0;        // 0 = force recompute each update, 1 = steady-state
    END_VAR
    VAR_OUTPUT
        checksum : LReal := 0.0;      // anti-elision accumulator
    END_VAR
    VAR
        instOtg : "RuckigOtg";
        otgIn   : "typeRuckigInput";
        otgOut  : "typeRuckigOutput";
    END_VAR
    VAR_TEMP
        i  : Int;
        ax : Int;
    END_VAR

    { S7_Language := "SCL" }
    NETWORK
        REGION Configure inputs
            #otgIn.nDofs := #nDofs;
            #otgIn.synchronization := 2;
            #otgIn.controlInterface := 0;
            #otgIn.minimumDuration := -1.0;
            #otgIn.durationDiscretization := 0;
            FOR #ax := 0 TO #nDofs - 1 DO
                #otgIn.enabled[#ax] := TRUE;
                #otgIn.maxVelocity[#ax] := 2.0;
                #otgIn.maxAcceleration[#ax] := 5.0;
                #otgIn.maxJerk[#ax] := 10.0;
                #otgIn.targetPosition[#ax] := 1.0;
                #otgIn.perDofSynchronization[#ax] := -1;
            END_FOR;
            #checksum := 0.0;
        END_REGION

        REGION Timed loop (time this externally; divide by iterations)
            FOR #i := 1 TO #iterations DO
                // mode 0: toggle the target each iteration to force a recompute
                // (worst case). mode 1: leave the target fixed (steady-state).
                IF #mode = 0 THEN
                    IF #otgIn.targetPosition[0] > 0.5 THEN
                        #otgIn.targetPosition[0] := 0.0;
                    ELSE
                        #otgIn.targetPosition[0] := 1.0;
                    END_IF;
                END_IF;
                #instOtg(enable := TRUE,
                         input := #otgIn,
                         cycleTime := 0.010,
                         reset := FALSE,
                         output => #otgOut);
                #checksum := #checksum + #otgOut.newPosition[0];
            END_FOR;
        END_REGION
    END_NETWORK
END_FUNCTION_BLOCK
```

Verify it against `RuckigOtg`'s actual VAR_INPUT (`enable, input, cycleTime, reset`) and VAR_OUTPUT (`output`, plus `valid/busy/done/error/status`) — only `output =>` is bound here; that's valid SCL (unbound outputs are allowed).

- [ ] **Step 2: Author the methodology README**

Create `benchmark/README.md` covering:
- **Purpose**: measure real `RuckigOtg` per-update cycle time on PLCSIM Advanced; the gate is **< 2 ms for 4-DoF** (`Update()` steady-state and worst-case recompute).
- **Import**: add `src/blocks`, `src/data-types`, `src/data-blocks`, and `benchmark/BenchRuckigOtg.s7dcl` to a TIA Portal V18+ project (S7-1500, FW V3.0+, optimized access).
- **Run**: instantiate `BenchRuckigOtg` in a cyclic OB; set `iterations` (e.g. 1000), `nDofs` (1/2/4), `mode` (0 recompute / 1 steady-state). Time the call with `RUNTIME(...)` in the OB around the FB call (or read the OB cycle time / use a TIA online trace), and divide by `iterations` for per-update µs.
- **Scenarios to report**: 1/2/4-DoF × {recompute, steady-state}; note the 4-DoF recompute worst case vs the < 2 ms gate.
- **Caveats**: a `mode=0` toggling target forces a recompute every update (worst case, not representative of steady tracking); optimized-access and FW version affect timing; warm up before measuring.
- **Cross-reference**: the relative operation-count proxy is in `docs/PERFORMANCE.md` (run `uv run python -m tests.perf.profile_ruckig`).

- [ ] **Step 3: Commit**

```bash
git add benchmark/
git commit -m "v0.11 T2: PLCSIM benchmark FB template + measurement methodology (TIA-only)"
```

---

## Task 3: Release 0.11.0

**Files:** `CHANGELOG.md`, `README.md`, `pyproject.toml`, `uv.lock`.

- [ ] **Step 1: CHANGELOG**

```markdown
## [0.11.0] - 2026-06-05

### Added
- **Performance characterization tooling.** `tests/perf/profile_ruckig.py` counts
  SCL-level operations per `RuckigOtg` update (FC calls by block + `sqrt`) as a
  relative cost proxy, and writes `docs/PERFORMANCE.md` (per-scenario cost,
  steady-state vs recompute, 4-DoF hot-path ranking). The common steady-state
  cycle is cheap (0 sqrt); the worst-case retarget cycle is dominated by the root
  solvers.
- **PLCSIM benchmark template.** `benchmark/BenchRuckigOtg.s7dcl` (a TIA-importable
  driver) + `benchmark/README.md` (measurement methodology for the < 2 ms / 4-DoF
  gate on PLCSIM Advanced FW V3.0+).

### Notes
- No solver change (the 261 parity/unit tests are unaffected). Real µs measurement
  (PLCSIM) and any resulting optimization are the next, user-driven step.
- `plc-code` cannot nest FBs, so the benchmark FB is a TIA-only artifact (the
  here-runnable loop-driver is the profiler).
```

- [ ] **Step 2: README** — Status note (v0.11.0, perf tooling); add a short **Performance** section pointing to `docs/PERFORMANCE.md` and `benchmark/`; Roadmap: mark v0.11 *(this release)*, next = real PLCSIM measurement + optimization (if needed) + field validation (v1.0 gate). Architecture/usage unchanged.

- [ ] **Step 3: Version** — `pyproject.toml` → `0.11.0`; `uv lock`.

- [ ] **Step 4: Full suite** — `uv run python -m pytest -q` → all green (261 solver tests + the 2 perf sanity tests = 263). Commit:
```bash
git add CHANGELOG.md README.md pyproject.toml uv.lock
git commit -m "v0.11 T3: release 0.11.0 (perf tooling docs + version)"
```

---

## Final integration (execution skill)
Hand off to `superpowers:finishing-a-development-branch`: merge `--no-ff`, tag `v0.11.0`, push. A light final review suffices (no solver change): confirm the profiler runs + report is committed, the benchmark FB matches RuckigOtg's signature, and the 261 solver tests are untouched/green.

## Self-review (planner)
- **Spec coverage:** profiler + report (T1), benchmark template + methodology (T2), release (T3). Methods match the revised spec (FC-call+sqrt proxy; TIA-only FB).
- **No-regression:** only `tests/perf/`, `benchmark/`, docs added → 261 solver tests untouched; full suite gate in T3.
- **No placeholders:** profiler + sanity test + benchmark FB given in full; README content enumerated.
- **Reality-checked:** the profiler hook and FB-nesting limitation were validated by probes during planning (sample data in Key facts).
```
