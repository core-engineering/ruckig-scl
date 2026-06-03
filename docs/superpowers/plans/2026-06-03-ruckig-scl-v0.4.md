# ruckig-scl v0.4 Implementation Plan — multi-axis time synchronization

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Multi-axis (≤ `DOF_MAX = 4`) **time-synchronized** trajectory generation: all DoFs reach their target at a common `t_sync`. Synchronization mode **Time** only, with the full reachable-duration `Block` so `t_sync` avoids blocked intervals. Builds on v0.2.1 step1 + v0.3 step2.

**Architecture:** `ComputeBlock1Axis` (step1 → `typeBlock` with `t_min`/`p_min`/blocked intervals) per DoF → `Synchronize` (common `t_sync` + limiting DoF) → `ComputeProfile1AxisTimed` (step2) per non-limiting DoF. `RuckigOtg` becomes multi-DoF; `ComputeProfile1Axis` stays as a thin wrapper over `block.p_min` (regression guard).

**Tech Stack:** Siemens SCL (`.s7dcl`, TIA Portal export) tested via `siemens-plc-tools` `plc-code` + pytest; numerical parity against the official Ruckig solver (PyPI `ruckig`, `n_dofs > 1`).

**Reference (MIT):** `include/ruckig/block.hpp` (`Block`, `calculate_block`, `is_blocked`); `include/ruckig/calculator_target.hpp` (`synchronize` l.123-203, mono-DoF fast path l.330-335, step2 invocation l.460-518). Local sdist cache under `~/.cache/uv/sdists-v9/pypi/ruckig/0.17.3/*/src/`.

**Decisions (from spec):** Time-only sync; full Block; `minimumDuration` exposed on the FB in v0.4; `ComputeProfile1Axis` kept behaviour-identical as a wrapper.

---

## Phase 0 — Block foundation

- [ ] **T1. `typeBlock` UDT.** Fields: `tMin : LReal`, `pMin : _.typeProfile`, `aValid : Bool`, `aLeft/aRight : LReal`, `aProfile : _.typeProfile`, `bValid : Bool`, `bLeft/bRight : LReal`, `bProfile : _.typeProfile`. (Flattened optional intervals.) No code depends on it yet.
- [ ] **T2. `ComputeBlock1Axis` (FC) + `calculate_block`.** Run the step1 families (both directions) but **collect every CheckProfile-valid candidate** (duration + profile) into a small array instead of keeping only the min. Port `Block::calculate_block` (block.hpp l.60-132): the 1/2/3/4/5-profile cases → `tMin`, `pMin`, and up to two blocked intervals. Unit tests: compare `tMin` + intervals against a Python port of `calculate_block` on swept single-axis inputs.
- [ ] **T3. `ComputeProfile1Axis` as wrapper.** Re-express it as `ComputeBlock1Axis` + return `block.pMin` / `RESULT_WORKING`. The **entire v0.2/v0.3 suite must stay green unchanged** (regression guard for the step1 refactor). If a full refactor is risky, keep `ComputeProfile1Axis` as-is and have `ComputeBlock1Axis` reuse the same family code — whichever keeps the suite green.

## Phase 1 — Synchronization

- [ ] **T4. `Synchronize` (FC).** Inputs: `typeBlock` per DoF (in/out array or DB), `nDofs`, `minimumDuration` (sentinel < 0 = none). Port `synchronize` (calculator_target.hpp:123-203): build candidate times (`tMin`, `aRight`, `bRight` per DoF, + `minimumDuration`), sort, pick smallest `t_sync ≥ max(tMin)` not blocked for any DoF (`is_blocked`), and the limiting DoF index. Unit tests on hand-built blocks: simple max(tMin); blocked-interval avoidance (a v0.3 blocked case); `minimumDuration` raising `t_sync`; limiting-DoF identity.

## Phase 2 — Multi-DoF FB

- [ ] **T5. `RuckigOtg` multi-DoF.** Replace the single-axis recompute REGION with: for each DoF (0..nDofs-1) brake-if-needed + `ComputeBlock1Axis` → `Synchronize` → for each non-limiting DoF `ComputeProfile1AxisTimed(tf = t_sync)`, limiting DoF uses its `pMin`. `trajectory.duration = t_sync`. Add the optional `minimumDuration` input. Verify `AdvanceTime` / `StateAtTime` already sample per-DoF (extend if single-DoF-only). 1-DoF path must remain identical (special case). Unit tests: 2-DoF FB cycle reaches both targets at the same final time.
- [ ] **T6. Multi-DoF parity runners + scenarios.** Extend `runner_ref` / `runner_scl` to N DoFs. Add 2-4 DoF YAML scenarios with differing per-axis `t_min` (incl. one where `max(tMin)` is blocked for an axis). Assert every DoF reaches its target at the common `t_sync`, within limits; strict parity for unique shapes, validity for stretched no-plateau axes.

## Phase 3 — Release

- [ ] **T7. Docs + release.** CHANGELOG `[0.4.0]` (multi-axis Time sync, Block, Synchronize, FB minimumDuration; Phase/None deferred), README status/features, version `0.4.0` in pyproject + touched block S7_Versions. Full `uv run pytest -q` green before commit. Merge `--no-ff`, annotated tag `v0.4.0`, delete branch. Update memory with verified SHAs.

---

## Notes / guardrails

- **Regression guard is paramount**: the step1→Block refactor must not change single-axis results — the v0.2/v0.3 suite (157 tests) is the safety net. Run it after T2/T3.
- **Validity vs parity** (v0.3 lesson): stretched no-plateau axes may differ from Ruckig's exact shape but must reach the state at `t_sync` within limits.
- **Blocked intervals are the point**: the v0.3 blocked-`tf` cases are v0.4's positive tests — `t_sync` must skip them.
- **TDD per component**, oracle/validity sweep after each; no wrong trajectory (INVALID=0).
- Transpiler quirks unchanged (END_IF/END_FOR own line; no `of`/`Dof` identifier endings; precompute scalar denominators; one `:=` per line; sub-blocks autoloaded by call-name).
- **Block array of profiles**: `calculate_block` needs the profiles at interval edges; store them (`aProfile`/`bProfile`) — `typeProfile` arrays are heavy but `DOF_MAX`-bounded.
