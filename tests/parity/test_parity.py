"""Numerical parity tests: SCL RuckigOtg vs the Ruckig reference.

Each YAML scenario under ``scenarios/`` is run through both the Ruckig
reference runner and the SCL runner, then compared cycle by cycle. Skipped
entirely when the optional ``ruckig`` dependency is not installed
(``uv sync --extra parity``).
"""
from __future__ import annotations

from pathlib import Path

import pytest
import yaml

pytest.importorskip("ruckig")

from .comparator import compare_traces  # noqa: E402
from .runner_ref import run_reference  # noqa: E402
from .runner_scl import run_scl  # noqa: E402

SCENARIO_DIR = Path(__file__).parent / "scenarios"
SCENARIOS = sorted(SCENARIO_DIR.glob("*.yaml"))


def _kwargs_from_scenario(data: dict) -> dict:
    n = data["n_dofs"]
    zeros = [0.0] * n
    return {
        "n_dofs": n,
        "current_pos": data.get("current_position", zeros),
        "current_vel": data.get("current_velocity", zeros),
        "current_acc": data.get("current_acceleration", zeros),
        "target_pos": data.get("target_position", zeros),
        "target_vel": data.get("target_velocity", zeros),
        "target_acc": data.get("target_acceleration", zeros),
        "max_vel": data["max_velocity"],
        "max_acc": data["max_acceleration"],
        "max_jerk": data["max_jerk"],
        "cycle_time": data.get("cycle_time", 0.010),
        "minimum_duration": data.get("minimum_duration", -1.0),
        "synchronization": data.get("synchronization", 2),
        "per_dof_synchronization": data.get("per_dof_synchronization"),
        "duration_discretization": data.get("duration_discretization", 0),
    }


@pytest.mark.parametrize("path", SCENARIOS, ids=[p.stem for p in SCENARIOS])
def test_parity_scenario(path: Path) -> None:
    data = yaml.safe_load(path.read_text())
    kwargs = _kwargs_from_scenario(data)

    ref = run_reference(**kwargs)
    scl = run_scl(**kwargs)

    result = compare_traces(
        ref,
        scl,
        tol_position=data.get("tol_position", 1.0e-6),
        tol_velocity=data.get("tol_velocity", 1.0e-6),
        tol_acceleration=data.get("tol_acceleration", 1.0e-6),
        tol_duration=data.get("tol_duration", 1.0e-6),
    )

    assert result.passed, (
        f"{path.stem}: posErr={result.max_position_error:.2e} "
        f"velErr={result.max_velocity_error:.2e} accErr={result.max_acceleration_error:.2e} "
        f"durErr={result.duration_error:.2e} (ref {ref.duration:.4f}s / scl {scl.duration:.4f}s) "
        f"failing_cycles={result.failing_cycles[:5]}"
    )
