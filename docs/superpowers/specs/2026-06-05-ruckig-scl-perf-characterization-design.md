# Ruckig SCL v0.11 — Performance Characterization & Benchmark Tooling — Design Spec

**Date** : 2026-06-05
**Owner** : Camille Martin
**Project** : Port of Ruckig (C++ MIT) to Structured Text for Siemens S7-1500
**Status** : Design approved, ready for implementation planning
**Builds on** : v0.10.0 (parity-complete solver: step1 + step2)

---

## 1. Context and Motivation

The solver is now parity-complete (v0.10). The roadmap's next gate is performance:
`Update()` under **2 ms for 4-DoF** on PLCSIM Advanced FW V3.0+, then field
validation. The real cycle-time measurement requires **PLCSIM Advanced**
(Windows / TIA Portal) — which cannot run in this WSL2/Python environment (the
`plc-code` transpiler runs SCL → Python for parity tests, not real PLC timing).

So this version does what *can* be done here — **characterize** the per-cycle cost
with a relative proxy and **prepare** the benchmark artifacts the user runs on
PLCSIM — and explicitly defers both the real µs measurement (user, on PLCSIM) and
any optimization (premature without real numbers). "Measure before optimizing"
applies: this version produces the measurement tooling and a cost model; whether
and what to optimize is a later decision informed by the user's real numbers.

### Scope decision (settled during brainstorming)

- **In:** (1) an operation-count proxy profiler + a committed cost report; (2) a
  TIA-importable benchmark harness + a PLCSIM measurement methodology doc.
- **Out:** solver optimizations (deferred until real PLCSIM numbers exist — avoid
  premature/blind optimization); real µs measurement (user, PLCSIM); field
  validation (user, MLA; the v1.0 gate).

---

## 2. Goals and Non-Goals

### Goals

- A reproducible **operation-count proxy** (Python-executed-lines + `sqrt` calls
  per `RuckigOtg` update) across representative scenarios, giving a relative
  per-cycle cost model and a hot-path ranking — without claiming real µs.
- A committed **`docs/PERFORMANCE.md`** report: steady-state vs worst-case
  recompute, per-scenario proxy cost, 4-DoF worst case, which FCs/families
  dominate.
- A **benchmark harness** (`benchmark/BenchRuckigOtg.s7dcl`) callable from a
  PLCSIM OB, transpile-verifiable here, plus **`benchmark/README.md`** with the
  measurement methodology (import, OB timing, scenarios, reading the <2 ms gate).
- No solver change → the 261 existing tests stay green.

### Non-Goals

- Any change to solver blocks or behaviour (no optimization this version).
- Real cycle-time measurement (needs PLCSIM; user-side).
- Field validation / the v1.0 gate.

---

## 3. Architecture

### 3.1 Operation-count proxy profiler (here)

> **Method validated during planning.** A `sys.settrace` executed-line count is
> NOT usable: the `plc-code` executor is an AST interpreter, so a trace is
> dominated by its own lexer/parser overhead (~3.3M Python lines per update, no
> per-block attribution) — it measures the interpreter, not the SCL algorithm.
> The faithful, overhead-immune proxy is to count **SCL-level operations**: hook
> the executor's FC-dispatch (`PLCRuntime.call_named_block`) to count FC
> invocations by block name, and monkeypatch `math.sqrt` (SCL `SQRT` maps to it,
> `codegen.py`) to count root ops. Confirmed working: a 4-DoF recompute reports
> 150 sqrt and a clear FC-call ranking (PolyEval 75, IntegrateProfileStates 56,
> CheckProfile 52, SolveQuartic 39, SolveCubic 28, …); steady-state reports 0
> sqrt and ~30 FC calls (mostly IsFiniteLreal validation + StateAtTime).

`tests/perf/profile_ruckig.py` — builds the `RuckigOtg` harness once, and for each
representative scenario runs the relevant `update` under instrumentation:

- **FC-call counts by block name**: wrap `PLCRuntime.call_named_block` to tally
  each FC invocation per update → the hot-path ranking (which FCs/families
  dominate) and the total FC-call count.
- **`sqrt` count**: monkeypatch `math.sqrt` for the duration of the update.

These are SCL-semantic counts (independent of interpreter overhead) and serve as
the relative per-cycle cost proxy. (Not µs — that is the user's PLCSIM number.)

Scenarios (each a `RuckigOtg` input + a cycle phase):
- **Steady-state** (a cycle with no recompute — the common case): expected to be
  cheap (`AdvanceTime` + `StateAtTime` × nDofs).
- **Recompute, 1-DoF** (a retarget cycle: brake + step1/two-step).
- **Recompute, 2-DoF and 4-DoF** (worst case: per-axis brake + Block +
  Synchronize + step2).
- **Recompute, velocity interface** (per-axis vel Block + re-time).

To isolate "recompute" cycles deterministically, the profiler drives a first
`update` (firstCall forces a recompute), then changes the target to force a
second recompute, and also samples a no-change cycle for steady-state.

Output: a Markdown table (printed, and written to `docs/PERFORMANCE.md`) with, per
scenario: executed-line count, sqrt count, and the steady-state/recompute ratio,
plus a short hot-path ranking derived from the line attribution (which block files
dominate the trace — `sys.settrace` exposes the filename per line, so executed
lines can be bucketed by source block).

### 3.2 Benchmark harness for PLCSIM (here-authored, user-run, TIA-only)

> **Limitation found during planning.** `plc-code` does NOT support an FB
> instantiating another FB (a multi-instance member like `instOtg : "RuckigOtg";`
> fails to parse — this is why `RuckigOtg` is always the directly-called
> top-level FB in the test bench). So `BenchRuckigOtg.s7dcl`, which must hold a
> `RuckigOtg` instance, **cannot be transpile-verified here** — it is a
> **TIA-Portal-only template** (multi-instance FB calls work normally in TIA).
> The *here-runnable* loop-execution of `RuckigOtg` is already provided by the
> Part 1 profiler (which drives many updates). (Record this as a `plc-code`
> known limitation.)

- **`benchmark/BenchRuckigOtg.s7dcl`** — an FB holding a `RuckigOtg` instance that,
  per call, invokes it a configurable number of times (`VAR_INPUT iterations`)
  over parameterized inputs (a `mode` selecting steady-state vs force-recompute,
  and `nDofs`). Timing is external (the calling OB), so the FB needs no
  PLC-specific timing function. It exposes a checksum output (summed
  `newPosition`) so the compiler cannot elide the calls. Authored as a
  TIA-importable template; not run here (see limitation above).
- **`benchmark/README.md`** — methodology: import the `src/` blocks + the
  benchmark FB into TIA Portal V18+, download to PLCSIM Advanced FW V3.0+, call
  `BenchRuckigOtg` from a cyclic OB, measure elapsed time with `RUNTIME` in the
  calling OB (or TIA online trace / OB cycle-time), divide by `iterations`, across
  the scenarios; how to read the **< 2 ms / 4-DoF** gate; notes on optimized-access
  and FW version.

### 3.3 No solver change

This version touches only `tests/perf/`, `benchmark/`, and docs. No `src/` block
is modified → the 261 tests are unaffected.

---

## 4. Testing strategy

- **Profiler (Part 1):** validated by running it — outputs are deterministic and
  reproducible; a sanity assertion (steady-state executed-line count ≪ a 4-DoF
  recompute count) can live in a small `tests/perf/test_profile_sanity.py`
  (skipped if perf deps unavailable; it just runs the profiler and checks the
  ordering, not absolute numbers).
- **Benchmark FB (Part 2):** NOT transpile-verifiable here (plc-code can't nest
  FBs — see §3.2). It is reviewed for SCL correctness by hand against
  `RuckigOtg`'s call signature; the equivalent *executable* loop-driving of
  `RuckigOtg` is the Part 1 profiler. The real µs measurement is the user's, on
  PLCSIM. The methodology doc is the primary Part 2 deliverable.
- **Regression:** the 261 existing tests stay green (no `src/` change).

---

## 5. Success criteria

- `tests/perf/profile_ruckig.py` runs and produces `docs/PERFORMANCE.md` with the
  per-scenario proxy cost, the steady-state vs recompute ratio, the 4-DoF worst
  case, and a hot-path ranking.
- `benchmark/BenchRuckigOtg.s7dcl` exists as a TIA-importable template (hand-
  reviewed against `RuckigOtg`'s signature; not transpilable here — plc-code can't
  nest FBs); `benchmark/README.md` documents the PLCSIM measurement procedure and
  the <2 ms gate.
- The 261 existing tests stay green.
- README / CHANGELOG / roadmap updated; tagged `v0.11.0` (tooling/characterization
  release; no solver change). The real measurement + any resulting optimization
  are explicitly the next step, user-driven on PLCSIM.
