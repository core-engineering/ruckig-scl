# ruckig-scl v0.10 — step2 Completeness — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans. Steps use checkbox (`- [ ]`) syntax.

**Goal:** Make step2 (`ComputeProfile1AxisTimed`) match the Ruckig oracle's profile **shape** (not just validity) on stretched `(state × tf)` cases, dropping the measured 1.71% divergence to ~0.

**Architecture:** Two bounded fixes inside existing step2 family FCs (no new FC, no signature/dispatch change, `CheckProfile` unchanged): (1) structural `ReachedLimits` guards in the limit-reaching families; (2) `SolveTimedVel` root precision — an all-zero-boundary cubic branch, and (if needed) derivative-extrema seeding for the general degree-5/6 paths. **Measurement-gated**: the cheaper fixes (guards + cubic) are applied and the divergence re-measured before deciding whether the heavier general-path work is needed.

**Tech Stack:** Siemens SCL (`.s7dcl`) via `siemens-plc-tools` `plc-code`; `pytest` + Ruckig oracle (PyPI `ruckig` 0.17.3); `uv`. **Tests: `uv run python -m pytest …`** (NOT bare pytest).

**Spec:** `docs/superpowers/specs/2026-06-04-ruckig-scl-v0.10-design.md`
**Oracle:** `~/.cache/uv/sdists-v9/pypi/ruckig/0.17.3/RHvqwSugDBzM9TJjKLyss/src/src/ruckig/position_third_step2.cpp` `time_vel` (l.259-487); `src/include/ruckig/profile.hpp` `check<>` structural guards (l.~188-204).

## Key facts
1. Dispatch order in `ComputeProfile1AxisTimed.s7dcl` already matches the oracle; UDUD is implemented. The gap is (a) missing structural guards and (b) `SolveTimedVel` root precision (dense-scan + single-Newton lands ~2e-5 off → fails CheckProfile's 1e-6 → VEL masked by a later family).
2. `SolveCubic`, `SolveQuartic`, `PolyEval`, `ShrinkInterval` FCs already exist in `src/blocks`.
3. **Structural guards per family** (ε = `EPS_TIME` = 1e-9): `t[3] ≥ ε` for `SolveTimedVel`, `SolveTimedAcc0Vel`, `SolveTimedAcc1Vel`, `SolveTimedAcc0Acc1Vel`; `t[1] ≥ ε` for `SolveTimedAcc0`; `t[5] ≥ ε` for `SolveTimedAcc1`; `t[1] ≥ ε AND t[5] ≥ ε` for `SolveTimedAcc0Acc1`; none for `SolveTimedNone`.
4. **No regression** (255 tests): currently-passing parity already matches the oracle (which respects these guards + the precise VEL root), so stricter guards can't reject those cases; only the diverging 1.71% moves.
5. Transpiler: one `:=` per line; `END_IF`/`END_FOR` own line (no inline comment); guard every `SQRT` arg ≥ 0; no `IF`-prefix / `of`/`Dof`-suffix identifiers.

## File structure

| File | New/Modify | Responsibility |
|---|---|---|
| `tests/parity/discover_step2_gaps.py` | Create | seeded `(state×tf)` fuzz; measures divergence rate; prints fixtures |
| `tests/parity/scenarios/v10_*.yaml` | Create | step2 divergence scenarios (via `minimum_duration`) |
| `tests/unit/test_solve_timed_vel.py` | Create | VEL precision + structural-guard unit tests |
| `src/blocks/SolveTimedVel.s7dcl` | Modify | cubic branch + general-path precision + t[3] guard |
| `src/blocks/SolveTimedAcc0Vel.s7dcl`, `SolveTimedAcc1Vel.s7dcl`, `SolveTimedAcc0Acc1Vel.s7dcl` | Modify | t[3] guard |
| `src/blocks/SolveTimedAcc0.s7dcl`, `SolveTimedAcc1.s7dcl`, `SolveTimedAcc0Acc1.s7dcl` | Modify | t[1]/t[5] guards |
| `CHANGELOG.md`, `README.md`, `pyproject.toml`, `uv.lock` | Modify | release 0.10.0 |

---

## Task 1: Structural guards + gap discovery + baseline measurement

**Files:** the 7 family FCs (guards); `tests/parity/discover_step2_gaps.py`; `tests/parity/scenarios/v10_*.yaml`; `tests/unit/test_solve_timed_vel.py`.

- [ ] **Step 1: Write the discovery/measurement script**

Create `tests/parity/discover_step2_gaps.py`: build the `ComputeProfile1AxisTimed` harness ONCE (reuse via `reset()`); seeded fuzz over single-axis `(v0,a0,pT,vT,aT)` within `vMax=3/aMax=5/jMax=10`, each at `tf ∈ {1.05,1.5,2,3,5} × tMin` (get `tMin` from the oracle's time-optimal duration). For each, compare the SCL profile boundary states + a cycle sampling to the oracle (use the parity runners or `at_time`). Skip infeasible-`tf` false positives (where the oracle's reported duration ≠ requested `tf`). Print: total pairs, genuine divergences (shape mismatch > 1e-6 with both reaching target at `tf`), and for each the `(v0,a0,pT,vT,aT,tf)` + SCL-vs-oracle family/shape. Mirror `tests/parity/discover_step1_gaps.py` style.

- [ ] **Step 2: Run it to get the BASELINE divergence rate**

Run: `uv run python -m tests.parity.discover_step2_gaps`. Record the baseline rate (expect ~1.7%) and capture the diverging `(state, tf)` cases. Keep this output — it is the gate for later tasks.

- [ ] **Step 3: Freeze ≥5 diverging cases as failing parity scenarios**

Create `tests/parity/scenarios/v10_01_step2_stretch.yaml` … (≥5), incl. the canonical all-zero case. Each single-axis with `minimum_duration` set to the diverging `tf`:
```yaml
name: v0.10 step2 stretched rest-to-rest (canonical VEL)
n_dofs: 1
target_position: [0.5]
max_velocity: [3.0]
max_acceleration: [5.0]
max_jerk: [10.0]
minimum_duration: 8.0
cycle_time: 0.010
```
(For non-zero-boundary cases include `current_velocity`/`current_acceleration`/`target_velocity`/`target_acceleration` from discovery.) Confirm they FAIL parity now: `uv run python -m pytest tests/parity -q -k v10`.

- [ ] **Step 4: Add the structural guards**

In each family FC, wrap the acceptance of every candidate (the `IF "CheckProfile"(...) THEN ... found:=true ... END_IF` site) with the family's structural guard so a candidate lacking its defining plateau is skipped. Read each file to find every CheckProfile call site. Apply:
- `SolveTimedVel.s7dcl`, `SolveTimedAcc0Vel.s7dcl`, `SolveTimedAcc1Vel.s7dcl`, `SolveTimedAcc0Acc1Vel.s7dcl`: only accept if `#profile.t[3] >= 1.0E-9`.
- `SolveTimedAcc0.s7dcl`: only accept if `#profile.t[1] >= 1.0E-9`.
- `SolveTimedAcc1.s7dcl`: only accept if `#profile.t[5] >= 1.0E-9`.
- `SolveTimedAcc0Acc1.s7dcl`: only accept if `#profile.t[1] >= 1.0E-9 AND #profile.t[5] >= 1.0E-9`.

Pattern (per candidate): change
```scl
                IF "CheckProfile"(...) THEN
                    #found := true; ... RETURN/accept ...
                END_IF;
```
to
```scl
                IF #profile.t[3] >= 1.0E-9 THEN
                    IF "CheckProfile"(...) THEN
                        #found := true; ... accept ...
                    END_IF;
                END_IF;
```
(Use the right `t[k]` per family. `CheckProfile` is NOT modified.)

- [ ] **Step 5: Re-measure (gate)**

Run `uv run python -m pytest -q` → expect still 255 green (no regression — guards only reject degenerate profiles that the oracle also rejects). Re-run the discovery script → record the divergence rate AFTER guards. Note which v10 scenarios now pass (the `VEL→NONE` structural ones should) and which remain (the VEL-precision ones).

- [ ] **Step 6: Write the unit test scaffold (failing for the precision cases)**

Create `tests/unit/test_solve_timed_vel.py`: for the canonical all-zero case (`p0=0,v0=0,a0=0,pT=0.5,vT=0,aT=0,tf=8`), assert `ComputeProfile1AxisTimed` returns WORKING and the profile has a velocity plateau (`t[3] > 0`) reaching the target to ≤1e-9 (it currently returns a NONE shape `t=[2,0,4,0,0,0,2]` → this asserts `t[3] > 0` which FAILS now, passes after Task 2). Plus a structural-guard assertion (a state where the guard must defer to the correct family).

- [ ] **Step 7: Commit**

```bash
git add src/blocks/SolveTimed*.s7dcl tests/parity/discover_step2_gaps.py tests/parity/scenarios/v10_*.yaml tests/unit/test_solve_timed_vel.py
git commit -m "v0.10 T1: structural ReachedLimits guards in step2 families + gap discovery + baseline"
```

---

## Task 2: `SolveTimedVel` all-zero-boundary cubic branch

**Files:** `src/blocks/SolveTimedVel.s7dcl`; `tests/unit/test_solve_timed_vel.py`.

Add, at the TOP of `SolveTimedVel` (before the UDDU degree-5 region, after Precompute), the oracle's all-zero-boundary branch (`time_vel` l.264-296). It is an exact cubic.

- [ ] **Step 1: Confirm the canonical unit test fails** (`t[3] > 0` expectation): `uv run python -m pytest tests/unit/test_solve_timed_vel.py -q` → the all-zero precision test FAILS (NONE shape).

- [ ] **Step 2: Add the cubic branch**

Insert a REGION after Precompute:
```scl
        REGION All-zero boundary cubic (UDDU VEL exact)
            // Ruckig time_vel l.264-296: when v0,a0,vf,af all ~0, the velocity-
            // plateau ramp time solves an exact cubic. polynom = [1, -tf/2, 0, pd/(2*jMax)].
            IF ABS(#v0) < 1.0E-12 AND ABS(#a0) < 1.0E-12 AND ABS(#vT) < 1.0E-12 AND ABS(#aT) < 1.0E-12 THEN
                #cubA := 1.0;
                #cubB := -#tf / 2.0;
                #cubC := 0.0;
                #cubD := #pd / (2.0 * #sjMax);
                "SolveCubic"(a := #cubA, b := #cubB, c := #cubC, d := #cubD,
                    roots => #cubRoots, count => #cubCount);
                FOR #cr := 0 TO #cubCount - 1 DO
                    IF NOT #found THEN
                        #t := #cubRoots[#cr];
                        IF #t <= #tf / 4.0 THEN
                            // single Newton step on pd
                            IF #t > 1.0E-12 THEN
                                #orig := -#pd + #sjMax * #t * #t * (#tf - 2.0 * #t);
                                #derivN := 2.0 * #sjMax * #t * (#tf - 3.0 * #t);
                                IF ABS(#derivN) > 1.0E-12 THEN
                                    #t := #t - #orig / #derivN;
                                END_IF;
                            END_IF;
                            #profile.t[0] := #t;
                            #profile.t[1] := 0.0;
                            #profile.t[2] := #t;
                            #profile.t[3] := #tf - 4.0 * #t;
                            #profile.t[4] := #t;
                            #profile.t[5] := 0.0;
                            #profile.t[6] := #t;
                            IF #profile.t[3] >= 1.0E-9 THEN
                                IF "CheckProfile"(profile := #profile, p0 := #p0, v0 := #v0, a0 := #a0,
                                        pT := #pT, vT := #vT, aT := #aT, vMax := #vMaxMag, aMax := #aMaxMag, tf := #tf) THEN
                                    #profile.controlSigns := 0;
                                    #found := true;
                                END_IF;
                            END_IF;
                        END_IF;
                    END_IF;
                END_FOR;
            END_IF;
        END_REGION
```
Add VAR_TEMP: `cubA,cubB,cubC,cubD : LReal; cubRoots : Array[0..2] of LReal; cubCount : Int; cr : Int;` (and reuse existing `t/orig/derivN/vMaxMag/aMaxMag/pd`). Set the jerk pattern (UDDU) before the branch if not already set globally — check the file; if the UDDU jerk is set later per-region, set it here too (`j=[sjMax,0,-sjMax,0,-sjMax,0,sjMax]`). Confirm `SolveCubic`'s exact output signature (roots array size, `count`) by reading `src/blocks/SolveCubic.s7dcl`.

Bump `S7_Version` to `"0.10.0"`.

- [ ] **Step 3: Verify**

`uv run python -m pytest tests/unit/test_solve_timed_vel.py -q` → canonical precision test PASSES (`t[3] > 0`, target ≤1e-9). `uv run python -m pytest tests/parity -q -k v10` → the canonical + other all-zero scenarios pass. `uv run python -m pytest -q` → 255+ green. Re-run discovery → record residual rate.

- [ ] **Step 4: Commit**

```bash
git add src/blocks/SolveTimedVel.s7dcl tests/unit/test_solve_timed_vel.py
git commit -m "v0.10 T2: SolveTimedVel all-zero-boundary cubic branch (exact canonical VEL)"
```

---

## Task 3 (CONDITIONAL on Task 2's residual): `SolveTimedVel` general-path extrema seeding

Only do this task if Task 2's re-measured divergence is still above ~0 (i.e. non-zero-boundary VEL cases remain). If the residual is ~0, SKIP to Task 4 and note it.

**Files:** `src/blocks/SolveTimedVel.s7dcl`.

Replace the brittle dense-scan + near-extremum heuristic (current l.165-181, UDDU) with the oracle's derivative-extrema seeding (`time_vel` l.309-388), and the same for the UDUD degree-6 region (l.390-486). The polynomial coefficients are already correct; only the root-FINDING changes.

- [ ] **Step 1: Port the UDDU extrema seeding**

Per the oracle: compute the polynomial's derivative (degree-4) coefficients (`poly_monic_derivative` — multiply each `poly[i]` by its power, renormalize monic), solve it with the existing `SolveQuartic` (the derivative is degree-4 → `solve_quart_monic`), giving the polynom's extrema `d_extremas`. Walk the extrema in `[tz_min, tz_max]`: refine each extremum by a Newton step on the derivative; at each, if `|poly(tz)|` is within `64·|dderiv(tz)|·tol` accept `tz` as a (tangent) root, else if `poly(tz_current)·poly(tz) < 0` bracket and `ShrinkInterval`; feed each candidate to the existing `check_root` (UDDU Newton, already correct). Use `PolyEval` for evaluations and a small `poly_derivative` helper computed inline. Keep the `t[3] >= 1e-9` guard from Task 1.

- [ ] **Step 2: Port the UDUD extrema seeding** (l.390-486)

Same approach, degree-6: derivative is degree-5, second derivative degree-4 (`solve_quart_monic` on `dderiv`), build the bracketing intervals from the dderiv extrema, `ShrinkInterval` on `deriv` to get the polynom extrema, then bracket/shrink the polynom and feed `check_root` (UDUD double-Newton, already in the file). This mirrors the UDDU structure with one extra derivative level.

- [ ] **Step 3: Verify**

`uv run python -m pytest -q` → 255+ green. Re-run discovery → divergence ~0. The remaining v10 scenarios pass.

- [ ] **Step 4: Commit**

```bash
git add src/blocks/SolveTimedVel.s7dcl
git commit -m "v0.10 T3: SolveTimedVel general-path derivative-extrema seeding (UDDU+UDUD precision)"
```

---

## Task 4: Broad fuzz validation + release 0.10.0

- [ ] **Step 1: Broad parity fuzz (the gate)**

Run `discover_step2_gaps.py` with a FRESH seed and a larger `n` → confirm the genuine divergence rate is ~0 (no shape mismatch > 1e-6, infeasible-`tf` excluded). Record the number. If a residual remains, dump the cases and reconcile against the oracle (do NOT relax tolerances).

- [ ] **Step 2: CHANGELOG**

```markdown
## [0.10.0] - 2026-06-04

### Fixed
- **step2 family-selection parity.** `ComputeProfile1AxisTimed` now matches the
  Ruckig oracle's profile *shape* (not just validity) on stretched `(state×tf)`
  cases — closing the last known parity gap (measured 1.71% → ~0). Two causes
  fixed: (1) `SolveTimedVel` root precision (added the all-zero-boundary cubic
  branch and derivative-extrema seeding so the velocity-plateau root resolves to
  ≤1e-9 instead of ~2e-5, which previously failed `CheckProfile`'s 1e-6 tolerance
  and let a later family mask VEL); (2) structural `ReachedLimits` guards
  (`t[3]/t[1]/t[5] ≥ ε`) in the limit-reaching step2 families so a
  structurally-degenerate profile can no longer be greedily accepted.
- `CheckProfile` and the dispatch order are unchanged.

### Notes
- Both step1 (v0.9) and step2 are now parity-complete. Next: performance / pre-1.0.
```

- [ ] **Step 3: README** — Status → v0.10.0; add a `## Features (v0.10)` / fixed note; remove the step2 short-move-stretched bullet from Known limitations (now resolved); add the v0.10 spec link; Roadmap mark v0.10 *(this release)*, next = performance / pre-1.0; update the parity-scenario count (`ls tests/parity/scenarios/*.yaml | wc -l`).

- [ ] **Step 4: Version** — `pyproject.toml` → `0.10.0`; `uv lock`.

- [ ] **Step 5: Full suite** — `uv run python -m pytest -q` → all green. Commit:
```bash
git add CHANGELOG.md README.md pyproject.toml uv.lock
git commit -m "v0.10 T4: broad parity validation + release 0.10.0"
```

---

## Final integration (execution skill)

Hand off to `superpowers:finishing-a-development-branch`: merge `--no-ff`, tag `v0.10.0`, push. Final opus review: verify the cubic branch + extrema seeding against the oracle, the structural guards per family, NO regression (the guards must not reject any currently-passing case), and an independent broad `(state×tf)` fuzz confirming ~0 divergence.

## Self-review (planner)
- **Spec coverage:** structural guards (T1), VEL cubic (T2), VEL general-path precision (T3, conditional), broad-fuzz gate + release (T4). All spec sections map.
- **Measurement-gated:** T1/T2 re-measure; T3 is conditional on residual — avoids over-building if guards+cubic already close it.
- **No-regression:** asserted at every task (255 green); guards justified in spec §3.3.
- **Placeholders:** T1 guards + T2 cubic are complete; T3 (general-path) is oracle-referenced structure (the heavy degree-5/6 seeding) to port faithfully with existing helpers — the oracle is line-cited and parity is the gate; expand from the oracle at execution if strict full-code is needed.
- **Consistency:** `t[3]/t[1]/t[5]` guard thresholds uniform (1e-9); existing FCs `SolveCubic/SolveQuartic/PolyEval/ShrinkInterval` reused; `check_root` Newtons already in the file are kept.
```
