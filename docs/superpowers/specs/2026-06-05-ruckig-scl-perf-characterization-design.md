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

`tests/perf/profile_ruckig.py` — builds the `RuckigOtg` harness once, and for each
representative scenario runs **a single `update`** under instrumentation:

- **Executed-line count** via `sys.settrace` (a line-event counter active only
  during `harness.execute()`): a solid *relative* proxy for the number of PLC
  instructions executed that cycle. (Not µs; a comparative metric.)
- **`sqrt` count**: monkeypatch `math.sqrt` (and the executor's sqrt path if
  distinct) for the duration of the update to count root operations — the
  solvers are sqrt-heavy, so this localizes hot paths.

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

### 3.2 Benchmark harness for PLCSIM (here-authored, user-run)

- **`benchmark/BenchRuckigOtg.s7dcl`** — an FB that, per call, invokes `RuckigOtg`
  a configurable number of times (`VAR_INPUT iterations`) over a small set of
  parameterized inputs (a `mode` selecting steady-state vs force-recompute, and
  `nDofs`). The timing is done **externally** by the calling OB (so the FB itself
  needs no PLC-specific timing function and remains **transpile-verifiable here**
  via `create_harness`). It exposes a checksum output (e.g. summed
  `newPosition`) so the optimizer/compiler cannot elide the calls.
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
- **Benchmark FB (Part 2):** transpile-verified here — a test builds the
  `BenchRuckigOtg` harness and runs one call, asserting it executes and the
  checksum is finite (proves it compiles and calls `RuckigOtg` correctly). The
  real µs measurement is the user's, on PLCSIM.
- **Regression:** the 261 existing tests stay green (no `src/` change).

---

## 5. Success criteria

- `tests/perf/profile_ruckig.py` runs and produces `docs/PERFORMANCE.md` with the
  per-scenario proxy cost, the steady-state vs recompute ratio, the 4-DoF worst
  case, and a hot-path ranking.
- `benchmark/BenchRuckigOtg.s7dcl` transpiles and runs one call here (checksum
  finite); `benchmark/README.md` documents the PLCSIM measurement procedure and
  the <2 ms gate.
- The 261 existing tests stay green.
- README / CHANGELOG / roadmap updated; tagged `v0.11.0` (tooling/characterization
  release; no solver change). The real measurement + any resulting optimization
  are explicitly the next step, user-driven on PLCSIM.
