"""Unit tests for AdvanceTime FC."""
from pathlib import Path

import pytest

from plc_code.executor import create_harness
from plc_code.executor.runtime import PLCRuntime

PROJECT_ROOT = Path(__file__).parent.parent.parent
BLOCK_PATH = PROJECT_ROOT / "src/blocks/AdvanceTime.s7dcl"
SEARCH_PATHS = [PROJECT_ROOT / "src/data-types"]


@pytest.fixture
def harness():
    rt = PLCRuntime(block_search_paths=SEARCH_PATHS)
    return create_harness(BLOCK_PATH, runtime=rt)


def make_trajectory(current_time=0.0, duration=5.0, is_valid=True) -> dict:
    empty_profile = {
        "t": [0.0] * 7, "j": [0.0] * 7,
        "a": [0.0] * 8, "v": [0.0] * 8, "p": [0.0] * 8,
        "direction": 1,
        "brake": {"t": [0.0, 0.0], "j": [0.0, 0.0],
                  "a": [0.0, 0.0, 0.0], "v": [0.0, 0.0, 0.0], "p": [0.0, 0.0, 0.0]},
        "controlSigns": 0,
    }
    return {
        "profiles": [empty_profile, empty_profile, empty_profile, empty_profile],
        "duration": duration,
        "independentMinDurations": [0.0, 0.0, 0.0, 0.0],
        "currentTime": current_time,
        "isValid": is_valid,
    }


def _call(harness, traj: dict, dt: float):
    harness.reset()
    harness.set_inputs(trajectory=traj, dt=dt)
    harness.execute()
    return harness.get_var("trajectory")


def test_advance_increments_current_time(harness):
    traj = make_trajectory()
    result = _call(harness, traj, 0.010)
    assert result.currentTime == pytest.approx(0.010, abs=1e-12)


def test_advance_does_not_exceed_duration(harness):
    traj = make_trajectory(current_time=4.995, duration=5.0)
    result = _call(harness, traj, 0.010)
    assert result.currentTime == pytest.approx(5.0, abs=1e-9)


def test_invalid_trajectory_unchanged(harness):
    traj = make_trajectory(current_time=1.0, is_valid=False)
    result = _call(harness, traj, 0.010)
    assert result.currentTime == pytest.approx(1.0, abs=1e-12)


def test_exact_advance_to_duration(harness):
    traj = make_trajectory(current_time=4.0, duration=5.0)
    result = _call(harness, traj, 1.0)
    assert result.currentTime == pytest.approx(5.0, abs=1e-12)
