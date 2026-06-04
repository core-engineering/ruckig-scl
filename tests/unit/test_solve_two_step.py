"""Step1 two-step fallback: states that errored before now solve."""
import pytest

RESULT_WORKING = 0x7000

# Genuine two-step states: solved by the pure ComputeProfile1Axis solver
# (two-step fallback families, no brake needed).
# States with a0=-2.044 and a0=4.665 require a brake pre-phase and are
# NOT pure two-step cases; they are covered by the v09 parity scenarios
# via RuckigOtg (which always calls ComputeBrakeProfile before step1).
GAP_STATES = [
    (-2.864, -1.826, -2.59, -0.917, -0.541),
    (2.79, 2.615, 2.404, -1.29, 1.672),
    (-2.151, -4.718, -1.66, 1.664, -3.165),
    (-2.261, -3.989, -0.798, 1.539, -4.625),
]


@pytest.fixture
def harness(make_harness):
    return make_harness("ComputeProfile1Axis.s7dcl")


def _profile():
    return {"t": [0.0]*7, "j": [0.0]*7, "a": [0.0]*8, "v": [0.0]*8, "p": [0.0]*8,
            "direction": 0, "controlSigns": 0,
            "brake": {"t": [0.0, 0.0], "j": [0.0, 0.0], "a": [0.0]*3, "v": [0.0]*3, "p": [0.0]*3}}


@pytest.mark.parametrize("v0,a0,pT,vT,aT", GAP_STATES)
def test_gap_state_now_solves(harness, v0, a0, pT, vT, aT):
    prof = _profile()
    harness.reset()
    harness.set_inputs(profile=prof, p0=0.0, v0=v0, a0=a0, pT=pT, vT=vT, aT=aT,
                       vMax=3.0, aMax=5.0, jMax=10.0)
    harness.execute()
    assert harness.get_output("ComputeProfile1Axis") == RESULT_WORKING
    out = harness.get_var("profile")
    assert out.v[7] == pytest.approx(vT, abs=1e-6)
    assert out.a[7] == pytest.approx(aT, abs=1e-6)
    assert out.p[7] == pytest.approx(pT, abs=1e-6)
