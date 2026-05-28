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
        "controlInterface": 0,
        "synchronization": 0,
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


def test_current_vel_exceeds_vmax(harness):
    inp = make_valid_input()
    inp["currentVelocity"] = [3.0, 0.0, 0.0, 0.0]  # > maxVelocity[0]=2.0
    assert _call(harness, inp) == RESULT_ERR_CURR_VEL


def test_current_acc_exceeds_amax(harness):
    inp = make_valid_input()
    inp["currentAcceleration"] = [6.0, 0.0, 0.0, 0.0]  # > maxAcceleration[0]=5.0
    assert _call(harness, inp) == RESULT_ERR_CURR_ACC


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
