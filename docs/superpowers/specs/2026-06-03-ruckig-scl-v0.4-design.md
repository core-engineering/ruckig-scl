# Ruckig SCL v0.4 — Design Spec (multi-axis time synchronization)

**Date** : 2026-06-03
**Owner** : Martin C
**Project** : Port of Ruckig (C++ MIT) to Structured Text for Siemens S7-1500
**Status** : Design draft, pending plan

**Scope:** Multi-axis (up to `DOF_MAX = 4`) **time-synchronized** trajectory
generation: all DoFs reach their target state simultaneously at a common
`t_sync`. Synchronization mode **Time only** (Ruckig default). Includes the
full `Block` (reachable-duration intervals) so `t_sync` is chosen robustly.
Builds on v0.2.1 step1 (time-optimal) + v0.3 step2 (re-timing to `tf`).

---

## 1. Context and Motivation

v0.2/v0.3 delivered the two single-axis solver stages:
- **step1** (`ComputeProfile1Axis`): time-optimal profile, minimum duration.
- **step2** (`ComputeProfile1AxisTimed`): re-time a DoF to an imposed `tf`.

Multi-axis motion needs both: each axis has its own minimum duration, but for a
coordinated move they must finish **together**. Ruckig:
1. runs step1 per DoF to get each axis's reachable-duration `Block`,
2. computes a common `t_sync` (the limiting/slowest axis sets the pace),
3. runs step2 on every other DoF to stretch it to `t_sync`.

Driving use case (unchanged): camera tracking on the MLA arm is genuinely
multi-axis (pan/tilt), and the axes must stay coordinated.

## 2. The Block (reachable durations)

A single-axis move is **not** achievable at every duration ≥ `t_min`: there can
be up to two **blocked intervals** of durations where no jerk-limited profile of
exactly that length exists (v0.3 saw these as "blocked `tf` → ERR_SOLVER").
Ruckig's `Block` (`block.hpp`) captures this: `{t_min, p_min, intervals a, b}`.

`Block::calculate_block` derives `t_min` + blocked intervals from **all** valid
step1 profiles (not just the fastest). So step1 must be extended to **collect
every valid candidate** (currently it keeps only the minimum-duration one).

- `t_sync` must avoid every DoF's blocked intervals; this is why "Block
  complete" is in scope (a naive `t_sync = max(t_min)` could land in a blocked
  interval of some axis → step2 fails).

## 3. Synchronization (Time)

Port of `TargetCalculator::synchronize` (`calculator_target.hpp:123-203`):
- Candidate times = every DoF's `t_min`, plus the right edges of its blocked
  intervals `a.right`, `b.right` (+ optional external `minimum_duration`).
- Sort candidates; take the **smallest** `t_sync ≥ max(t_min)` that is **not
  blocked** for any DoF (`Block::is_blocked(t)`), and ≠ ∞.
- The DoF whose `t_min` set `t_sync` is the **limiting DoF**: it keeps its
  `p_min` (already optimal); all others are re-timed to `t_sync` by step2.

## 4. Architecture / new blocks

- **`typeBlock`** (UDT) — `{ tMin : LReal; pMin : typeProfile; aValid, bValid :
  Bool; aLeft, aRight, bLeft, bRight : LReal; aProfile, bProfile : typeProfile }`.
  (Two optional intervals, flattened — no `std::optional` in SCL.)
- **`ComputeBlock1Axis`** (FC) — step1 that returns a full `typeBlock` instead
  of a single profile: collect all valid candidates (SolveDirection already
  enumerates them; change "keep min" → "append to a small valid-profile array"),
  then port `Block::calculate_block` (the 1/2/3/5-profile interval logic).
  `ComputeProfile1Axis` can become a thin wrapper returning `block.p_min`.
- **`Synchronize`** (FC) — given the per-DoF `typeBlock[]`, `nDofs`, and optional
  `minimumDuration`, compute `t_sync` and the limiting DoF index.
- **`RuckigOtg` (FB) multi-DoF** — replace the single-axis recompute with:
  loop `ComputeBlock1Axis` per DoF → `Synchronize` → for each non-limiting DoF
  `ComputeProfile1AxisTimed(tf = t_sync)`; limiting DoF uses `block.p_min`. Add
  the optional `minimumDuration` input (also forces step2 when `> t_sync`).
  `pass_to_input` chaining and the brake pre-phase extend per-DoF.
- `typeTrajectory` already holds `profiles[0..DOF_MAX-1]`; `AdvanceTime` /
  `StateAtTime` already loop... (verify they sample per-DoF; extend if not).

## 5. Test strategy

- **Oracle:** the `ruckig` package with `n_dofs > 1`; compare the SCL multi-DoF
  FB cycle-by-cycle (extend the parity runners to N DoFs).
- **Per-component unit tests:** `ComputeBlock1Axis` vs a Python port of
  `calculate_block` (t_min + intervals); `Synchronize` on hand-built blocks
  (limiting DoF, blocked-interval avoidance, minimum_duration).
- **Multi-axis parity scenarios:** 2-4 DoF moves where axes have different
  `t_min`; assert every DoF reaches its target at the common `t_sync` and the
  trajectory matches Ruckig (vel-reaching) / is valid (no-plateau), within
  limits. Include a case where the naive `max(t_min)` is blocked for one axis.
- Validity-based criterion for the step2-stretched axes (v0.3 lesson), strict
  parity where shapes are unique.
- Full v0.3 suite stays green (single-axis path = 1 DoF special case).

## 6. Task breakdown (high level; detailed in the plan)

1. `typeBlock` UDT.
2. `ComputeBlock1Axis`: collect valid step1 candidates + `calculate_block`.
3. `Synchronize`: `t_sync` + limiting DoF.
4. `RuckigOtg` multi-DoF orchestration + `minimumDuration` input.
5. Multi-DoF parity runners + scenarios.
6. Docs / release / tag `v0.4.0`.

## 7. Risks and mitigations

- **step1 refactor to emit a Block** is the core change; keep
  `ComputeProfile1Axis` behaviour identical (wrapper over `block.p_min`) so the
  whole v0.2/v0.3 suite stays green — a strong regression guard.
- **`calculate_block` interval logic** (1/2/3/5-profile cases) is fiddly; unit
  test against a Python port directly.
- **Blocked-interval correctness**: the v0.3 blocked-`tf` cases become the
  positive tests here (t_sync must skip them).
- Transpiler quirks unchanged (END_IF own line, no `of`/`Dof`, scalar
  denominators, one `:=` per line, autoload by call-name).

## 8. Out of scope (v0.4)

- `Synchronization::None` / `Phase`, duration discretization.
- Per-DoF different limits beyond the existing arrays (already supported).
- Velocity/second-order interfaces; the step1 ~3% arbitrary-state gaps (v0.2).

## 9. Acceptance criteria

1. Multi-DoF (2-4) time-synchronized moves: every DoF reaches its target at the
   common `t_sync`, within limits; parity with Ruckig (validity for stretched
   no-plateau axes) on a swept scenario set.
2. `t_sync` correctly avoids blocked intervals (the v0.3 blocked cases now
   resolve at the next non-blocked common duration).
3. Single-axis (1 DoF) behaviour unchanged; full prior suite green.
4. CHANGELOG/README/version `0.4.0`; tagged; memory updated with verified SHAs.
