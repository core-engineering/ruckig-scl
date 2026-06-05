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
    h.reset()  # fresh instance (firstCall=true) BEFORE set_inputs
    h.set_inputs(enable=True, input=inp, cycleTime=0.01, reset=False)
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
