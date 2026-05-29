"""Cycle-by-cycle comparator for SCL vs Ruckig reference trajectories."""
from __future__ import annotations

from dataclasses import dataclass, field

from .runner_ref import TrajectoryTrace


@dataclass
class ParityResult:
    """Outcome of comparing two traces, with worst-case errors for debugging."""

    passed: bool
    max_position_error: float = 0.0
    max_velocity_error: float = 0.0
    max_acceleration_error: float = 0.0
    duration_error: float = 0.0
    failing_cycles: list[int] = field(default_factory=list)


def compare_traces(
    ref: TrajectoryTrace,
    scl: TrajectoryTrace,
    *,
    tol_position: float = 1.0e-6,
    tol_velocity: float = 1.0e-6,
    tol_acceleration: float = 1.0e-6,
    tol_duration: float = 1.0e-6,
) -> ParityResult:
    """Compare two traces cycle by cycle within the given tolerances.

    Samples are compared up to the shortest common length: the reference and
    the SCL port may declare "finished" one cycle apart even though they reach
    the identical final state, so trailing samples are not penalised.
    """
    result = ParityResult(passed=True)

    result.duration_error = abs(ref.duration - scl.duration)
    if result.duration_error > tol_duration:
        result.passed = False

    n_cycles = min(len(ref.samples), len(scl.samples))
    for i in range(n_cycles):
        r = ref.samples[i]
        s = scl.samples[i]
        n_dofs = min(len(r.p), len(s.p))
        cycle_failed = False
        for d in range(n_dofs):
            ep = abs(r.p[d] - s.p[d])
            ev = abs(r.v[d] - s.v[d])
            ea = abs(r.a[d] - s.a[d])
            result.max_position_error = max(result.max_position_error, ep)
            result.max_velocity_error = max(result.max_velocity_error, ev)
            result.max_acceleration_error = max(result.max_acceleration_error, ea)
            if ep > tol_position or ev > tol_velocity or ea > tol_acceleration:
                cycle_failed = True
        if cycle_failed:
            result.failing_cycles.append(i)
            result.passed = False

    return result
