"""Unit tests for ComputeVelProfileTimed FC (velocity Step 2)."""
import pytest

RESULT_WORKING = 0x7000


@pytest.fixture
def harness(make_harness):
    return make_harness("ComputeVelProfileTimed.s7dcl")


def _empty_profile():
    return {
        "t": [0.0] * 7, "j": [0.0] * 7,
        "a": [0.0] * 8, "v": [0.0] * 8, "p": [0.0] * 8,
        "direction": 0, "controlSigns": 0,
        "brake": {"t": [0.0, 0.0], "j": [0.0, 0.0],
                  "a": [0.0, 0.0, 0.0], "v": [0.0, 0.0, 0.0], "p": [0.0, 0.0, 0.0]},
    }


def _run(harness, *, v0, a0, vf, af, aMax, jMax, tf):
    prof = _empty_profile()
    harness.reset()
    harness.set_inputs(profile=prof, p0=0.0, v0=v0, a0=a0,
                       vf=vf, af=af, aMax=aMax, jMax=jMax, tf=tf)
    harness.execute()
    res = harness.get_output("ComputeVelProfileTimed")
    prof_out = harness.get_var("profile")
    return res, prof_out


def _sum_t(prof):
    return sum(prof.t[i] for i in range(7))


def test_retime_reaches_target_at_tf(harness):
    # Time-optimal tMin for vd=1, jMax=4 is 1.0; re-time to tf=2.0.
    res, prof = _run(harness, v0=0.0, a0=0.0, vf=1.0, af=0.0, aMax=10.0, jMax=4.0, tf=2.0)
    assert res == RESULT_WORKING
    assert _sum_t(prof) == pytest.approx(2.0, abs=1e-9)
    assert prof.v[7] == pytest.approx(1.0, abs=1e-6)
    assert prof.a[7] == pytest.approx(0.0, abs=1e-6)


def test_coast_when_already_at_target(harness):
    res, prof = _run(harness, v0=2.0, a0=0.0, vf=2.0, af=0.0, aMax=10.0, jMax=4.0, tf=1.5)
    assert res == RESULT_WORKING
    assert _sum_t(prof) == pytest.approx(1.5, abs=1e-9)
    assert prof.v[7] == pytest.approx(2.0, abs=1e-6)


def test_nonzero_af(harness):
    res, prof = _run(harness, v0=0.0, a0=0.0, vf=1.0, af=1.0, aMax=10.0, jMax=4.0, tf=1.5)
    assert res == RESULT_WORKING
    assert _sum_t(prof) == pytest.approx(1.5, abs=1e-9)
    assert prof.v[7] == pytest.approx(1.0, abs=1e-6)
    assert prof.a[7] == pytest.approx(1.0, abs=1e-6)
