"""Unit tests for ValidateInput FC."""
import math
from pathlib import Path

import pytest

from plc_code.executor import create_harness
from plc_code.executor.runtime import PLCRuntime

PROJECT_ROOT = Path(__file__).parent.parent.parent
BLOCK_PATH = PROJECT_ROOT / "src/blocks/ValidateInput.s7dcl"
SEARCH_PATHS = [
    PROJECT_ROOT / "src/blocks",
    PROJECT_ROOT / "src/data-blocks",
    PROJECT_ROOT / "src/data-types",
]


@pytest.fixture
def harness():
    rt = PLCRuntime(block_search_paths=SEARCH_PATHS)
    return create_harness(BLOCK_PATH, runtime=rt)


def make_valid_input(**overrides) -> dict:
    """Build a valid 1-DOF input, allow specific field overrides by path."""
    base = {
        "currentPosition": [0.0, 0.0, 0.0, 0.0],
        "currentVelocity": [0.0, 0.0, 0.0, 0.0],
        "currentAcceleration": [0.0, 0.0, 0.0, 0.0],
        "targetPosition": [1.0, 0.0, 0.0, 0.0],
        "targetVelocity": [0.0, 0.0, 0.0, 0.0],
        "targetAcceleration": [0.0, 0.0, 0.0, 0.0],
        "maxVelocity": [2.0, 2.0, 2.0, 2.0],
        "maxAcceleration": [5.0, 5.0, 5.0, 5.0],
        "maxJerk": [10.0, 10.0, 10.0, 10.0],
        "enabled": [True, False, False, False],
        "nDofs": 1,
        "minimumDuration": -1.0,
        "controlInterface": 0,
        "synchronization": 2,
        "perDofSynchronization": [-1, -1, -1, -1],
        "durationDiscretization": 0,
    }
    for key, value in overrides.items():
        base[key] = value
    return base


def _call(harness, input_data: dict) -> int:
    harness.reset()
    harness.set_inputs(input=input_data)
    harness.execute()
    return harness.get_output("ValidateInput")


# Status code constants matching dbRuckigConst
RESULT_WORKING = 0x7000
RESULT_ERR_VMAX = 0x8201
RESULT_ERR_AMAX = 0x8202
RESULT_ERR_JMAX = 0x8203
RESULT_ERR_CURR_VEL = 0x8204
RESULT_ERR_CURR_ACC = 0x8205
RESULT_ERR_NDOFS = 0x8206
RESULT_ERR_NON_FINITE = 0x8207


def test_valid_input_returns_working(harness):
    assert _call(harness, make_valid_input()) == RESULT_WORKING


def test_vmax_zero_returns_err(harness):
    inp = make_valid_input()
    inp["maxVelocity"] = [0.0, 2.0, 2.0, 2.0]
    assert _call(harness, inp) == RESULT_ERR_VMAX


def test_amax_negative_returns_err(harness):
    inp = make_valid_input()
    inp["maxAcceleration"] = [-1.0, 5.0, 5.0, 5.0]
    assert _call(harness, inp) == RESULT_ERR_AMAX


def test_jmax_zero_returns_err(harness):
    inp = make_valid_input()
    inp["maxJerk"] = [0.0, 10.0, 10.0, 10.0]
    assert _call(harness, inp) == RESULT_ERR_JMAX


def test_current_vel_over_vmax_accepted_for_brake(harness):
    """v0.2: an out-of-limits CURRENT velocity is admissible (Ruckig parity:
    check_current_state_within_limits=false). The RuckigOtg FB brakes it back
    into limits, so ValidateInput must NOT reject it."""
    inp = make_valid_input()
    inp["currentVelocity"] = [3.0, 0.0, 0.0, 0.0]  # > maxVelocity[0]=2.0
    assert _call(harness, inp) == RESULT_WORKING


def test_current_acc_over_amax_accepted_for_brake(harness):
    """v0.2: an out-of-limits CURRENT acceleration is likewise admissible."""
    inp = make_valid_input()
    inp["currentAcceleration"] = [6.0, 0.0, 0.0, 0.0]  # > maxAcceleration[0]=5.0
    assert _call(harness, inp) == RESULT_WORKING


def test_ndofs_zero_returns_err(harness):
    inp = make_valid_input()
    inp["nDofs"] = 0
    assert _call(harness, inp) == RESULT_ERR_NDOFS


def test_ndofs_too_large_returns_err(harness):
    inp = make_valid_input()
    inp["nDofs"] = 5  # > DOF_MAX = 4
    assert _call(harness, inp) == RESULT_ERR_NDOFS


def test_non_finite_position_returns_err(harness):
    inp = make_valid_input()
    inp["currentPosition"] = [math.nan, 0.0, 0.0, 0.0]
    assert _call(harness, inp) == RESULT_ERR_NON_FINITE


def test_non_finite_target_returns_err(harness):
    inp = make_valid_input()
    inp["targetVelocity"] = [math.inf, 0.0, 0.0, 0.0]
    assert _call(harness, inp) == RESULT_ERR_NON_FINITE


def test_nonzero_target_velocity_and_acceleration_accepted(harness):
    """v0.2: arbitrary target state. A finite, within-limits non-zero target
    velocity and acceleration must validate (moving-target / approach-and-cruise
    use case), not be rejected."""
    inp = make_valid_input()
    inp["targetVelocity"] = [1.0, 0.0, 0.0, 0.0]      # < maxVelocity[0]=2.0
    inp["targetAcceleration"] = [0.5, 0.0, 0.0, 0.0]  # < maxAcceleration[0]=5.0
    assert _call(harness, inp) == RESULT_WORKING


def test_negative_target_velocity_accepted(harness):
    """A negative within-limits target velocity is also valid."""
    inp = make_valid_input()
    inp["targetVelocity"] = [-1.5, 0.0, 0.0, 0.0]
    assert _call(harness, inp) == RESULT_WORKING


def test_non_finite_target_acceleration_returns_err(harness):
    inp = make_valid_input()
    inp["targetAcceleration"] = [math.nan, 0.0, 0.0, 0.0]
    assert _call(harness, inp) == RESULT_ERR_NON_FINITE


RESULT_ERR_IFACE = 0x8208


def test_velocity_mode_accepts_zero_vmax(harness):
    inp = make_valid_input()
    inp["controlInterface"] = 1
    inp["maxVelocity"] = [0.0, 0.0, 0.0, 0.0]   # ignored in velocity mode
    inp["targetVelocity"] = [1.0, 0.0, 0.0, 0.0]
    assert _call(harness, inp) == RESULT_WORKING


def test_velocity_mode_ignores_nonfinite_target_position(harness):
    inp = make_valid_input()
    inp["controlInterface"] = 1
    inp["targetPosition"] = [math.inf, 0.0, 0.0, 0.0]   # ignored in velocity mode
    assert _call(harness, inp) == RESULT_WORKING


def test_velocity_mode_requires_amax(harness):
    inp = make_valid_input()
    inp["controlInterface"] = 1
    inp["maxAcceleration"] = [0.0, 5.0, 5.0, 5.0]
    assert _call(harness, inp) == RESULT_ERR_AMAX


def test_velocity_mode_requires_jmax(harness):
    inp = make_valid_input()
    inp["controlInterface"] = 1
    inp["maxJerk"] = [0.0, 10.0, 10.0, 10.0]
    assert _call(harness, inp) == RESULT_ERR_JMAX


def test_velocity_mode_rejects_nonfinite_target_velocity(harness):
    inp = make_valid_input()
    inp["controlInterface"] = 1
    inp["targetVelocity"] = [math.nan, 0.0, 0.0, 0.0]
    assert _call(harness, inp) == RESULT_ERR_NON_FINITE


def test_invalid_control_interface_rejected(harness):
    inp = make_valid_input()
    inp["controlInterface"] = 7
    assert _call(harness, inp) == RESULT_ERR_IFACE
