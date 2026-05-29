"""plc-code-based SCL runner for parity tests.

Runs the ``RuckigOtg`` FB cycle by cycle through the plc-code SCL-to-Python
executor and returns the same :class:`TrajectoryTrace` shape as the Ruckig
reference runner, so the two can be compared directly.
"""
from __future__ import annotations

from pathlib import Path

from plc_code.executor import create_harness
from plc_code.executor.runtime import PLCRuntime

from .runner_ref import CycleSample, TrajectoryTrace

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
FB_PATH = PROJECT_ROOT / "src/blocks/RuckigOtg.s7dcl"
SEARCH_PATHS = [
    PROJECT_ROOT / "src/blocks",
    PROJECT_ROOT / "src/data-types",
    PROJECT_ROOT / "src/data-blocks",
]


def _pad(values: list[float], length: int = 4) -> list[float]:
    """Pad a per-DoF list up to DOF_MAX with zeros."""
    return list(values) + [0.0] * (length - len(values))


def run_scl(
    *,
    n_dofs: int,
    current_pos: list[float],
    current_vel: list[float],
    current_acc: list[float],
    target_pos: list[float],
    target_vel: list[float],
    target_acc: list[float],
    max_vel: list[float],
    max_acc: list[float],
    max_jerk: list[float],
    cycle_time: float,
    max_cycles: int = 2000,
) -> TrajectoryTrace:
    """Run the SCL RuckigOtg FB cycle by cycle via the plc-code harness."""
    runtime = PLCRuntime(block_search_paths=list(SEARCH_PATHS))
    harness = create_harness(FB_PATH, runtime=runtime)

    inp = {
        "currentPosition": _pad(current_pos),
        "currentVelocity": _pad(current_vel),
        "currentAcceleration": _pad(current_acc),
        "targetPosition": _pad(target_pos),
        "targetVelocity": _pad(target_vel),
        "targetAcceleration": _pad(target_acc),
        "maxVelocity": _pad(max_vel),
        "maxAcceleration": _pad(max_acc),
        "maxJerk": _pad(max_jerk),
        "enabled": [d < n_dofs for d in range(4)],
        "nDofs": n_dofs,
        "controlInterface": 0,
        "synchronization": 0,
        "durationDiscretization": 0,
    }

    trace = TrajectoryTrace()
    for cycle in range(max_cycles):
        harness.set_inputs(enable=True, input=inp, cycleTime=cycle_time, reset=False)
        harness.execute()
        out = harness.get_output("output")
        # _AutoStruct: attribute access (out.newPosition), then integer index.
        trace.samples.append(
            CycleSample(
                t=cycle * cycle_time,
                p=[out.newPosition[d] for d in range(n_dofs)],
                v=[out.newVelocity[d] for d in range(n_dofs)],
                a=[out.newAcceleration[d] for d in range(n_dofs)],
            )
        )
        trace.duration = out.trajectoryDuration
        if harness.get_output("done"):
            trace.finished_at_cycle = cycle
            break

    return trace
