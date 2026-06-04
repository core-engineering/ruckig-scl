"""Unit tests for ComputeVelBlock1Axis FC (velocity Step 1)."""
import math

import pytest

RESULT_WORKING = 0x7000


@pytest.fixture
def harness(make_harness):
    return make_harness("ComputeVelBlock1Axis.s7dcl")


def _empty_profile():
    return {
        "t": [0.0] * 7, "j": [0.0] * 7,
        "a": [0.0] * 8, "v": [0.0] * 8, "p": [0.0] * 8,
        "direction": 0, "controlSigns": 0,
        "brake": {"t": [0.0, 0.0], "j": [0.0, 0.0],
                  "a": [0.0, 0.0, 0.0], "v": [0.0, 0.0, 0.0], "p": [0.0, 0.0, 0.0]},
    }


def _empty_block():
    return {"tMin": 0.0, "found": False,
            "aValid": False, "aLeft": 0.0, "aRight": 0.0,
            "bValid": False, "bLeft": 0.0, "bRight": 0.0}


def _run(harness, *, v0, a0, vf, af, aMax, jMax):
    harness.reset()
    harness.set_inputs(block=_empty_block(), profile=_empty_profile(),
                       p0=0.0, v0=v0, a0=a0, vf=vf, af=af, aMax=aMax, jMax=jMax)
    harness.execute()
    return (harness.get_output("ComputeVelBlock1Axis"),
            harness.get_var("block"))


def test_zero_to_one_triangular(harness):
    # v0=0,a0=0 -> vf=1,af=0 with aMax high (no plateau): time_none.
    # h1 = sqrt(jMax*vd) = sqrt(4*1) = 2; tMin = 2*h1/jMax = 2*2/4 = 1.0
    res, block = _run(harness, v0=0.0, a0=0.0, vf=1.0, af=0.0, aMax=10.0, jMax=4.0)
    assert res == RESULT_WORKING
    assert block.tMin == pytest.approx(1.0, abs=1e-9)
    assert block.aValid is False  # af == 0 -> no blocked interval


def test_at_target_zero_duration(harness):
    res, block = _run(harness, v0=2.0, a0=0.0, vf=2.0, af=0.0, aMax=10.0, jMax=4.0)
    assert res == RESULT_WORKING
    assert block.tMin == pytest.approx(0.0, abs=1e-9)


def test_negative_vd(harness):
    # symmetric: vf below v0 -> same tMin as the positive case
    res, block = _run(harness, v0=1.0, a0=0.0, vf=0.0, af=0.0, aMax=10.0, jMax=4.0)
    assert res == RESULT_WORKING
    assert block.tMin == pytest.approx(1.0, abs=1e-9)
