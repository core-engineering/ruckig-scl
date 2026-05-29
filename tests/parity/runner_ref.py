"""Ruckig-based reference runner for parity tests.

Wraps the official Ruckig Python bindings (PyPI package ``ruckig``) and returns
a cycle-by-cycle trace of the generated trajectory, so the SCL port can be
compared against it. Requires the optional ``parity`` dependency group
(``uv sync --extra parity``).
"""
from __future__ import annotations

from dataclasses import dataclass, field

import ruckig


@dataclass
class CycleSample:
    """Position / velocity / acceleration at one PLC cycle."""

    t: float
    p: list[float]
    v: list[float]
    a: list[float]


@dataclass
class TrajectoryTrace:
    """A cycle-by-cycle trace plus the total trajectory duration."""

    samples: list[CycleSample] = field(default_factory=list)
    duration: float = 0.0
    finished_at_cycle: int = -1


def run_reference(
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
    """Run Ruckig with the given inputs and return a cycle-by-cycle trace."""
    otg = ruckig.Ruckig(n_dofs, cycle_time)
    inp = ruckig.InputParameter(n_dofs)
    out = ruckig.OutputParameter(n_dofs)

    inp.current_position = current_pos[:n_dofs]
    inp.current_velocity = current_vel[:n_dofs]
    inp.current_acceleration = current_acc[:n_dofs]
    inp.target_position = target_pos[:n_dofs]
    inp.target_velocity = target_vel[:n_dofs]
    inp.target_acceleration = target_acc[:n_dofs]
    inp.max_velocity = max_vel[:n_dofs]
    inp.max_acceleration = max_acc[:n_dofs]
    inp.max_jerk = max_jerk[:n_dofs]

    trace = TrajectoryTrace()
    for cycle in range(max_cycles):
        result = otg.update(inp, out)
        trace.samples.append(
            CycleSample(
                t=cycle * cycle_time,
                p=list(out.new_position),
                v=list(out.new_velocity),
                a=list(out.new_acceleration),
            )
        )
        trace.duration = out.trajectory.duration
        if result == ruckig.Result.Finished:
            trace.finished_at_cycle = cycle
            break
        out.pass_to_input(inp)

    return trace
