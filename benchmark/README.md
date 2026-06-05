# PLCSIM Advanced Benchmark — RuckigOtg Cycle Time

## Purpose

Measure the real per-update cycle time of `RuckigOtg` running on PLCSIM Advanced.

**Performance gate: < 2 ms for 4-DoF** (both steady-state and worst-case recompute).

The relative operation-count proxy (FC calls + `sqrt` counts per update) is in
`docs/PERFORMANCE.md`. Run it with:

```
uv run python -m tests.perf.profile_ruckig
```

That proxy is interpreter-overhead-immune but is **not microseconds**. Real µs come
from PLCSIM Advanced using this benchmark template.

---

## Why TIA-Only

`BenchRuckigOtg.s7dcl` nests a `RuckigOtg` FB instance (`instOtg : "RuckigOtg"`).
The SCL-to-Python test bench (`plc-code`) cannot parse nested FB instances, so this
template is exercised only in TIA Portal / PLCSIM Advanced. It lives in `benchmark/`
(not `src/blocks/`) and is therefore invisible to the test runner.

---

## Import

1. Open a TIA Portal V18+ project targeting an **S7-1500**, firmware **V3.0+**,
   **optimized block access ON** (default for S7-1500).
2. Import the following source files (File → Import or drag into the project tree):
   - `src/data-types/` — all UDTs (`typeRuckigInput`, `typeRuckigOutput`, etc.)
   - `src/data-blocks/` — global DBs if any
   - `src/blocks/` — all FCs and the `RuckigOtg` FB
   - `benchmark/BenchRuckigOtg.s7dcl` — the benchmark driver FB
3. Compile. There should be zero errors; warnings about unused outputs are expected
   (`valid`, `busy`, `done`, `error`, `status` are not bound in the benchmark call).

---

## Run

1. Add a **cyclic OB** (e.g. OB30, 10 ms cycle) to the project.
2. In the OB, declare a **multi-instance or single-instance DB** for `BenchRuckigOtg`.
3. Call `BenchRuckigOtg` from the OB. Set parameters:
   - `iterations` — number of `RuckigOtg` updates per OB call (e.g. 1000). A higher
     value gives a more stable average; keep total call time under the OB period.
   - `nDofs` — axes to exercise: 1, 2, or 4.
   - `mode`:
     - `0` = **force recompute** every update (worst case): the target position is
       toggled each iteration, which forces the trajectory solver to recompute.
     - `1` = **steady-state**: the target is unchanged; the solver only advances the
       current state along the existing trajectory (cheap path).

4. **Time the call.** Choose one of:
   - **RUNTIME instruction** (preferred): wrap the FB call in the OB with `RUNTIME`
     before and after; subtract to get elapsed ticks; convert with the CPU's tick
     resolution (typically 1 ns on S7-1500):
     ```scl
     #t0 := RUNTIME(0);
     "BenchDB"(iterations := 1000, nDofs := 4, mode := 0);
     #elapsed := RUNTIME(#t0);    // LReal seconds
     #usPerUpdate := (#elapsed * 1.0e6) / 1000.0;
     ```
   - **OB cycle time monitor**: read the OB execution time in the TIA Portal online
     view (Diagnostics → Cycle time). Subtract any overhead beyond the FB call.
   - **TIA online trace**: add a trace on `#elapsed` or the OB cycle time counter.

5. Divide the elapsed time by `iterations` to obtain the **per-update cost in µs**.

---

## Scenarios to Report

Fill in the measured values from PLCSIM Advanced (FW V3.0+, optimized access):

| Scenario            | Mode          | Measured µs/update | Gate          |
|---------------------|---------------|--------------------|---------------|
| 1-DoF position      | Recompute (0) |                    | —             |
| 1-DoF position      | Steady (1)    |                    | —             |
| 2-DoF position      | Recompute (0) |                    | —             |
| 2-DoF position      | Steady (1)    |                    | —             |
| 4-DoF position      | Recompute (0) |                    | **< 2000 µs** |
| 4-DoF position      | Steady (1)    |                    | **< 2000 µs** |

The 4-DoF recompute (mode 0) is the hard gate: worst-case retarget on 4 axes. If it
exceeds 2 ms on the target hardware, the caller's OB must be lengthened accordingly or
`iterations` reduced so the benchmark call fits within one OB period.

---

## Caveats

- **Mode 0 is not representative of steady tracking.** It forces a full trajectory
  recompute on every call (by toggling the target). In practice, retargets are
  infrequent; the common cycle is steady-state (mode 1), which is significantly
  cheaper (see `docs/PERFORMANCE.md`: 0 sqrt, ~30 FC calls vs. ~150 sqrt + hundreds
  of FC calls for a 4-DoF recompute).

- **Warm up before measuring.** Run the OB for a few seconds before recording times;
  the first few calls may be slower due to cache effects.

- **Optimized access must be ON.** The `RuckigOtg` FB and its UDTs are compiled with
  `S7_Optimized := "TRUE"`. Turning optimized access off on the calling OB or DB will
  cause a compilation error.

- **FW version affects timing.** V3.0+ is the baseline. Earlier FW may have different
  instruction latencies.

- **The `checksum` output prevents call elision.** Read `BenchDB.checksum` online or
  use it in a downstream calculation so the compiler does not optimize away the loop.

- **PLCSIM vs. real hardware.** PLCSIM Advanced provides a realistic software
  simulation but does not perfectly replicate hardware cycle times. Validate on the
  target CPU family (e.g. CPU 1516-3 PN/DP) before committing to a cycle budget.
