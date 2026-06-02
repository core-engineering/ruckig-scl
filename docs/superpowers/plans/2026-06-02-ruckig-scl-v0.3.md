# ruckig-scl v0.3 Implementation Plan — step2 (re-timing to imposed duration)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Single-axis **step2** — given a target duration `tf ≥ t_min`, compute the jerk-limited profile reaching the arbitrary final state `(pT,vT,aT)` *exactly at* `tf`. Port of Ruckig `PositionThirdOrderStep2`. Foundation for v0.4 multi-axis synchronization. FC-only (no FB change in v0.3).

**Architecture:** A step2 entry FC `ComputeProfile1AxisTimed` mirrors `ComputeProfile1Axis` but takes `tf` and dispatches the eight step2 families (first-valid-wins, both directions, UDDU **and** UDUD control signs). An optional `SolveTimedDirection` FC holds the per-direction family runs. `CheckProfile` gains an optional `tf` duration assertion (guarded so step1 callers are unaffected). Reuses `SolveCubic`/`SolveQuartic`/`IntegrateProfileStates` from v0.2.

**Tech Stack:** Siemens SCL (`.s7dcl`, TIA Portal export) tested via `siemens-plc-tools` `plc-code` (SCL→Python) + pytest; numerical parity against the official Ruckig solver (PyPI `ruckig`, `parity` extra), **step2 forced via `inp.minimum_duration = tf`**.

**Reference (MIT):** Ruckig step2 single-DoF source — consult during implementation:
- `src/ruckig/position_third_step2.cpp` (the 8 families + dispatch). Local sdist cache: `~/.cache/uv/sdists-v9/pypi/ruckig/0.17.3/*/src/src/ruckig/position_third_step2.cpp`. Or GitHub raw `https://raw.githubusercontent.com/pantor/ruckig/main/src/ruckig/position_third_step2.cpp`.
- `include/ruckig/profile.hpp` (`check_with_timing`, UDUD jerk pattern l.213, interior-velocity check), `include/ruckig/block.hpp`, `include/ruckig/calculator_target.hpp` (dispatch context).

**Decisions (from spec):** (1) v0.3 is **FC-only** — `RuckigOtg.minimumDuration` + FB integration ships with v0.4. (2) Duration validation = an **optional `tf` check inside `CheckProfile`**, guarded so a sentinel (e.g. `tf < 0`) disables it for step1 callers.

**Family source line map** (`position_third_step2.cpp`): dispatch 1112-1138; `time_acc0_acc1_vel` 41-75; `time_acc1_vel` 77-163; `time_acc0_vel` 165-257; `time_vel` 259-488; `time_acc0_acc1` 490-542; `time_acc1` 544-624; `time_acc0` 626-687; `time_none` 689-1012; `time_none_smooth` 1014-1110.

---

## Phase 0 — Scaffolding & oracle

- [ ] **T1. Test harness + step2 oracle helper.** Add a `_oracle_timed(state, tf)` helper that runs Ruckig with `inp.minimum_duration = tf` and returns the trajectory; assert it actually exercised step2 (duration ≈ tf). Add a `make_harness("ComputeProfile1AxisTimed.s7dcl")` fixture. No SCL yet — just the bench + a smoke test that the oracle stretches duration to `tf`.
- [ ] **T2. `CheckProfile` duration check (guarded).** Add optional input `tf : LReal` (sentinel `tf < 0.0` ⇒ skip). When `tf ≥ 0`, after the existing limit/target checks, assert `ABS(Σt − tf) ≤ EPS_TIME`. Existing step1 callers pass `tf := -1.0` ⇒ behaviour unchanged. Unit tests: existing CheckProfile tests still green (pass `tf := -1.0`); new test rejects a profile whose `Σt ≠ tf`. Bump `CheckProfile` S7_Version.
- [ ] **T3. UDUD integration sanity.** Before any family: a unit test that `IntegrateProfileStates` + `CheckProfile` correctly handle the UDUD jerk pattern `[+j,0,−j,0,+j,0,−j]` on a hand-built UDUD profile (verify node `a/v/p` match a Python reference). De-risks the new control-signs path.

## Phase 1 — `ComputeProfile1AxisTimed` skeleton + dispatch

- [ ] **T4. Entry FC skeleton.** Create `ComputeProfile1AxisTimed` (FC `: Word`), inputs `p0,v0,a0,pT,vT,aT,vMax,aMax,jMax,tf`, VAR_IN_OUT `profile`. Implement `up_first := (pd > tf*v0)` direction selection and the dispatch *ordering* of the 8 families × 2 directions (calls to `SolveTimedDirection`, initially a stub returning `found=false`). Return `RESULT_WORKING` if a family validated, else `RESULT_ERR_SOLVER`. Identifier ends in `…Timed` (no `of`/`Dof`).
- [ ] **T5. `SolveTimedDirection` skeleton + shared precompute.** FC taking signed limits + `tf`. Precompute the step2 member set: `ad`, `vd`, `vd_vd`, `g1`, powers (`a0_p3/p4`, `af_p3/p4`), `jMax_jMax`, etc. — transcribe definitions exactly from the cpp. Set both jerk patterns as needed per candidate. Empty family bodies for now.

## Phase 2 — Vel-reaching families (closed form first, then quartics)

- [ ] **T6. `time_acc0_acc1_vel` (UDDU + UDUD).** Closed form (cpp 41-75) — simplest, validates the `tf`-coupling (`t[5]=tf−Σ`) and the UDUD path end-to-end. Red parity test (oracle via `minimum_duration`) on a case that reaches both accel limits and the vel plateau at a stretched `tf`; port; green.
- [ ] **T7. `time_vel` (UDDU + UDUD).** Largest vel family (cpp 259-488): quartic + Newton, several sub-cases. Split UDDU vs UDUD into separate red→green steps if needed.
- [ ] **T8. `time_acc0_vel` (UDDU + UDUD).** cpp 165-257, quartic + Newton.
- [ ] **T9. `time_acc1_vel` (UDDU + UDUD).** cpp 77-163, quartic + Newton (note the clamped first Newton step, like step1 ACC1).

## Phase 3 — No-vel-plateau families

- [ ] **T10. `time_acc0_acc1` (UDDU).** cpp 490-542 — closed form with a jerk solve (`check_with_timing` variant passing `jf`).
- [ ] **T11. `time_acc0` (UDDU + UDUD).** cpp 626-687.
- [ ] **T12. `time_acc1` (UDDU + UDUD).** cpp 544-624, quartic.

## Phase 4 — `time_none` (the big one)

- [ ] **T13. `time_none` UDDU, a0=af=0 quartic + general.** cpp 707-768 (a0=af=0) and 790-905 (general a3≠0, four time-layouts T0234/T3456/T2346/T0124). Each sub-case its own red→green.
- [ ] **T14. `time_none` UDUD.** cpp 770-788 (T0246) and the UDUD a3≠0 sub-cases. UDUD jerk pattern.
- [ ] **T15. `time_none` 3-step fallbacks + `time_none_smooth`.** cpp 949-1009 (UZD/UZU/UDU numerical-robustness fallbacks) and `time_none_smooth` 1014-1110.

## Phase 5 — Completeness, docs, release

- [ ] **T16. Full dispatch wiring + completeness sweep.** Enable all families in `ComputeProfile1AxisTimed`'s dispatch in Ruckig's exact order. Sweep arbitrary states × multiple feasible `tf` (e.g. `t_min·{1.0,1.1,1.5,2.0}`, skipping blocked intervals), comparing to the `minimum_duration` oracle: assert 0 spurious failures (profile hits `tf`, target, limits incl. interior-velocity) and parity ~1e-9. Log any residual (e.g. `tf` inside a blocked interval — that needs Block intervals, a v0.4 item) without masking.
- [ ] **T17. Docs + release.** CHANGELOG `[0.3.0]` (step2 added; note multi-axis still pending), README status/features, version bump to `0.3.0` in `pyproject.toml` and all touched block S7_Versions. `uv run pytest -q` fully green before any commit. Merge `--no-ff` to master, annotated tag `v0.3.0`, delete feature branch. Update memory with verified SHAs.

---

## Notes / guardrails

- **TDD per family, oracle is ground truth.** One family (or sub-case) per red→green cycle. Never commit or mark a task complete until `uv run pytest -q` is fully green (lesson from v0.2 T8).
- **first-valid-wins, not min-duration.** Unlike step1, step2 families all target `tf`; the dispatcher returns the first that validates. `SolveTimedDirection` returns on first valid (no running-best).
- **Transpiler quirks (still in force):** `END_IF`/`END_FOR` own line; no `of`/`Dof` identifier endings; precompute scalar denominators (no `/(a*b)`); one `:=` per source line; sub-blocks autoloaded by call-name filename.
- **Feasibility of `tf`.** step2 assumes `tf` is achievable (≥ `t_min`, not in a blocked interval). Block intervals are v0.4; in v0.3 tests choose `tf` outside any blocked region. Document this assumption.
- **No FB change in v0.3.** `RuckigOtg` stays time-optimal; the `minimumDuration` input is a v0.4 opener.
- **Out of scope:** multi-DoF sync, `t_sync`, Block `a`/`b` intervals, the ~3 % step1 completeness gaps, Phase sync, duration discretization.
