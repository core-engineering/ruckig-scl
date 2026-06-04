"""Unit tests for CheckVelProfile FC."""
import pytest


@pytest.fixture
def harness(make_harness):
    return make_harness("CheckVelProfile.s7dcl")


def _profile(t, j):
    """Build a typeProfile dict from t/j (a/v/p get filled by integration)."""
    t = list(t) + [0.0] * (7 - len(t))
    j = list(j) + [0.0] * (7 - len(j))
    return {
        "t": t, "j": j,
        "a": [0.0] * 8, "v": [0.0] * 8, "p": [0.0] * 8,
        "direction": 0, "controlSigns": 0,
        "brake": {"t": [0.0, 0.0], "j": [0.0, 0.0],
                  "a": [0.0, 0.0, 0.0], "v": [0.0, 0.0, 0.0], "p": [0.0, 0.0, 0.0]},
    }


def _run(harness, profile, p0, v0, a0, vf, af, aMax, tf):
    harness.reset()
    harness.set_inputs(profile=profile, p0=p0, v0=v0, a0=a0,
                       vf=vf, af=af, aMax=aMax, tf=tf)
    harness.execute()
    return harness.get_output("CheckVelProfile")


def test_coast_at_target_is_valid(harness):
    prof = _profile([0.0, 1.0, 0.0], [0.0, 0.0, 0.0])
    assert _run(harness, prof, p0=0.0, v0=2.0, a0=0.0,
                vf=2.0, af=0.0, aMax=5.0, tf=1.0) is True


def test_negative_phase_time_rejected(harness):
    prof = _profile([-0.1, 1.0, 0.0], [0.0, 0.0, 0.0])
    assert _run(harness, prof, p0=0.0, v0=2.0, a0=0.0,
                vf=2.0, af=0.0, aMax=5.0, tf=-1.0) is False


def test_wrong_velocity_target_rejected(harness):
    prof = _profile([0.0, 1.0, 0.0], [0.0, 0.0, 0.0])
    assert _run(harness, prof, p0=0.0, v0=2.0, a0=0.0,
                vf=5.0, af=0.0, aMax=5.0, tf=-1.0) is False


def test_duration_mismatch_rejected(harness):
    prof = _profile([0.0, 2.0, 0.0], [0.0, 0.0, 0.0])
    assert _run(harness, prof, p0=0.0, v0=2.0, a0=0.0,
                vf=2.0, af=0.0, aMax=5.0, tf=1.0) is False
