# Ruckig SCL v0.3 — Design Spec (step2: re-timing to an imposed duration)

**Date** : 2026-06-02
**Owner** : Martin C
**Project** : Port of Ruckig (C++ MIT) to Structured Text for Siemens S7-1500
**Status** : Design draft, pending plan

**Scope:** Single-axis **step2** — given a *target duration* `tf`, compute the
jerk-limited profile that reaches the arbitrary final state `(pT, vT, aT)`
**exactly at** `tf` (not the time-optimal duration). This is the second of
Ruckig's two solver stages and the mandatory foundation for multi-axis time
synchronization. **Multi-axis itself (Block intervals + `t_sync` + multi-DoF
FB) is explicitly deferred to v0.4.** Builds on the v0.2.1 FC/FB/UDT layer.

---

## 1. Context and Motivation

v0.2.x delivered **step1**: the *time-optimal* single-axis solver
(`ComputeProfile1Axis`), which finds the **minimum** duration `t_min` and its
profile. Ruckig's full algorithm has a second stage, **step2**, which re-times a
DoF to an **imposed** duration `tf ≥ t_min`.

Why this matters:

- **Multi-axis synchronization needs it.** Ruckig runs step1 per axis to get
  each axis's `t_min`, picks a common `t_sync`, then runs **step2** on every
  non-limiting axis to stretch it to `t_sync`. Without step2 there is no
  synchronized multi-axis motion (the v0.3→v0.5 roadmap goal).
- **It is the missing capability**, not a bug fix. The ~3 % step1 completeness
  gaps observed on arbitrary states are a *separate* track (step1 `Block`/`p_min`);
  in mono-axis Ruckig bypasses step2 entirely (`calculator_target.hpp:330-335`
  returns `blocks[0].p_min` directly when `DOFs==1 && !minimum_duration`). They
  are **out of scope** here.
- **It is independently testable.** Setting Ruckig's `minimum_duration = tf`
  forces the step2 path even for a single DoF, giving a clean numerical oracle.

Real-world driver (unchanged): online camera tracking of a moving MLA target.
Multi-axis (pan/tilt) synchronization is the v0.4 payoff that v0.3 unlocks.

## 2. What step2 computes

Given `(p0, v0, a0)`, target `(pT, vT, aT)`, symmetric limits
`(vMax, aMax, jMax)` and an imposed duration `tf`, produce a 7-phase profile
whose phase times sum to **exactly `tf`** and which reaches the final state
within limits. The degree of freedom that step1 spent on minimizing time is now
spent on hitting `tf`: typically one phase time is solved as
`t[5] = tf − (sum of the other phase times)` (see
`position_third_step2.cpp:51,66`), and the remaining unknowns are fixed by a
closed form or a quartic + Newton refinement.

Unlike step1 (which is min-duration, "keep the shortest valid candidate"),
step2 is **first-found-wins at fixed `tf`**: every family is constructed to land
on `tf`, so the dispatcher returns the first family whose profile validates.

## 3. New vs v0.2

| Aspect | v0.2 (step1) | v0.3 (step2) |
|---|---|---|
| Objective | minimize duration | hit imposed `tf` |
| Selection | global min over candidates | first valid family wins |
| Control signs | **UDDU only** | **UDDU AND UDUD** (new) |
| Direction | positive + mirror (negated limits) | `up_first = (pd > tf·v0)` then both |
| Validation | `CheckProfile` (limits + target) | + **duration == tf** check |

The **UDUD** control-signs pattern is new to the project. Its jerk layout
(`profile.hpp:213`) is `j = [+jf, 0, −jf, 0, +jf, 0, −jf]` (vs UDDU
`[+jf,0,−jf,0,−jf,0,+jf]`). `CheckProfile` / `IntegrateProfileStates` already
integrate from the per-phase `j[]`, so supporting UDUD is a matter of setting
the right jerk pattern per candidate; the duration check is the only new
validation rule.

## 4. step2 dispatch and families

Dispatcher (`position_third_step2.cpp:1112`): pick direction from
`up_first = (pd > tf·v0)`, then try the eight families, each in the chosen
direction and its mirror (first valid wins):

| # | Family | ReachedLimits | Control signs | Method |
|---|---|---|---|---|
| 1 | `time_acc0_acc1_vel` | ACC0_ACC1_VEL | UDDU + UDUD | closed form |
| 2 | `time_vel` | VEL | UDDU + UDUD | quartic + Newton |
| 3 | `time_acc0_vel` | ACC0_VEL | UDDU + UDUD | quartic + Newton |
| 4 | `time_acc1_vel` | ACC1_VEL | UDDU + UDUD | quartic + Newton |
| 5 | `time_acc0_acc1` | ACC0_ACC1 | UDDU | closed form (+ jerk solve) |
| 6 | `time_acc0` | ACC0 / NONE | UDDU + UDUD | closed form |
| 7 | `time_acc1` | ACC1 | UDDU + UDUD | quartic |
| 8 | `time_none` | NONE | UDDU + UDUD | **largest (~324 l., ~18 sub-cases)** + `time_none_smooth` |

`time_none` is the dominant effort: many sub-cases (a0=af=0 quartic; general
a3≠0 with four time-layouts; UDUD T0246; 3-step numerical-robustness
fallbacks). It will be split into its own sub-tasks in the plan.

Reused as-is: `SolveCubic`, `SolveQuartic` (v0.2), `IntegrateProfileStates`.
Extended: `CheckProfile` (add an optional `tf` duration check) or a sibling
`CheckProfileTimed`.

## 5. Architecture / new blocks

- **`ComputeProfile1AxisTimed`** (FC `: Word`) — step2 entry point. Inputs:
  `p0,v0,a0, pT,vT,aT, vMax,aMax,jMax, tf`. Mirrors `ComputeProfile1Axis`'s
  shape but takes `tf` and dispatches the step2 families. Identifier avoids a
  trailing `of`/`Dof` (transpiler lexer quirk) — `…Timed` is safe.
- **`SolveTimedDirection`** (FC, optional) — analog of `SolveDirection`: runs the
  families for one direction with signed limits, returning the first valid
  profile (no min-duration bookkeeping). Keeps `ComputeProfile1AxisTimed` small.
- **Precompute extension** — step2 formulas use extra members: `ad = a0 − af`,
  `vd = vf − v0` (sign per Ruckig), `g1`, `vd_vd`, plus the `tf` couplings. To
  be transcribed exactly from `position_third_step2.cpp`.
- **`CheckProfile`** — add a duration assertion `|Σt − tf| ≤ EPS_TIME` (guarded
  so step1 callers, which pass no `tf`, are unaffected) **or** a `CheckProfileTimed`.
- **`RuckigOtg` FB (optional in v0.3)** — add an optional `minimumDuration`
  input: when `> t_min`, run step2 at `tf = minimumDuration`. Gives an
  end-to-end, user-facing "stretch to a minimum cycle time" feature and an
  integration test. May be deferred to the start of v0.4 if v0.3 stays
  FC-only; the plan decides.

All existing UDTs are unchanged (`typeProfile` already carries `controlSigns`).

## 6. Test strategy

- **Oracle:** Ruckig with `inp.minimum_duration = tf` (forces step2 even for 1
  DoF). For each scenario, pick a feasible `tf` (e.g. `t_min·k`, `k ∈ {1.0, 1.1,
  1.5, 2.0, …}`) avoiding any blocked interval, and assert the SCL profile
  reaches `(pT,vT,aT)`, stays within limits (interior-velocity check included),
  and has `Σt == tf`, to ~1e-9.
- **Per family + per control-sign**: dedicated parity cases driving each of the
  8 families and both UDDU/UDUD, derived by sweeping arbitrary states × `tf`
  and bucketing by the oracle profile's `limits`/`control_signs`.
- **Regression:** the full v0.2.1 suite (108 tests) stays green; step1 and the
  time-optimal FB path are untouched.
- TDD per family: red parity test → port formula → green, one family at a time
  (subagent-driven-development), exactly as v0.2.

## 7. Risks and mitigations

- **Volume** — step2 is ~1140 lines (≈2× step1). Mitigate by splitting per
  family (and `time_none` per sub-case) into independent TDD tasks; the oracle
  catches every transcription error numerically.
- **UDUD newness** — first use of the UDUD jerk pattern; verify the integration
  reproduces `a[i]` at nodes for a known UDUD case before trusting the families.
- **Canonical-frame subtlety** — Ruckig stores profiles in a transformed frame
  and step2 picks direction via `up_first`; mirror handling must be transcribed
  carefully (the same trap that confused the step1 gap diagnosis).
- **Transpiler quirks** (still in force): `END_IF`/`END_FOR` own line; no
  `of`/`Dof` identifier endings; no division by a parenthesized product
  (precompute scalar denominators); one `:=` per source line; sub-blocks
  autoloaded by call-name filename.
- **Duration-check coupling** — extending `CheckProfile` must not regress step1
  callers (guard the `tf` check).

## 8. Out of scope (v0.3)

- Multi-DoF synchronization, `t_sync`, the `Block` blocked-interval computation
  (`a`, `b`) — **v0.4**.
- The ~3 % step1 mono-axis completeness gaps — separate track.
- `Synchronization::Phase`, duration discretization, `TimeIfNecessary`.
- Velocity/second-order interfaces.

## 9. Acceptance criteria

1. `ComputeProfile1AxisTimed` reproduces Ruckig's step2 (forced via
   `minimum_duration`) to ~1e-9 across all 8 families and both UDDU/UDUD control
   signs, in both directions, on a swept arbitrary-state × `tf` parity set.
2. A sweep reports **0 spurious failures**: for every feasible `(state, tf)`
   the SCL solver returns a profile hitting `tf`, the target, and the limits
   (or an honest error only where Ruckig also fails).
3. Full pre-existing suite green; no step1 / time-optimal regression.
4. CHANGELOG/README/version updated; tagged `v0.3.0`; memory updated with
   verified SHAs.
