"""Parity tests for the two/three-step fallbacks (ComputeProfile1Axis ->
SolveTwoStep). Each case is a SHORT move where the three main families find no
feasible profile (pre-fix the solver returned RESULT_ERR_SOLVER); Ruckig solves
them via time_*_two_step. The oracle is computed live, so we only list inputs.
"""
import pytest

ruckig = pytest.importorskip("ruckig")


@pytest.fixture
def harness(make_harness):
    return make_harness("ComputeProfile1Axis.s7dcl")


_FUNC = "ComputeProfile1Axis"
_VMAX, _AMAX, _JMAX = 2.0, 3.0, 10.0


def _empty_profile():
    return {
        "t": [0.0] * 7, "j": [0.0] * 7,
        "a": [0.0] * 8, "v": [0.0] * 8, "p": [0.0] * 8,
        "direction": 1,
        "brake": {"t": [0.0, 0.0], "j": [0.0, 0.0],
                  "a": [0.0, 0.0, 0.0], "v": [0.0, 0.0, 0.0], "p": [0.0, 0.0, 0.0]},
        "controlSigns": 0,
    }


def _oracle(p0, v0, a0, pT, vT, aT):
    otg = ruckig.Ruckig(1); inp = ruckig.InputParameter(1); tr = ruckig.Trajectory(1)
    inp.current_position = [p0]; inp.current_velocity = [v0]; inp.current_acceleration = [a0]
    inp.target_position = [pT]; inp.target_velocity = [vT]; inp.target_acceleration = [aT]
    inp.max_velocity = [_VMAX]; inp.max_acceleration = [_AMAX]; inp.max_jerk = [_JMAX]
    otg.calculate(inp, tr)
    return tr


def _run(harness, p0, v0, a0, pT, vT, aT):
    harness.reset()
    harness.set_inputs(profile=_empty_profile(), p0=p0, v0=v0, a0=a0,
                       pT=pT, vT=vT, aT=aT, vMax=_VMAX, aMax=_AMAX, jMax=_JMAX)
    harness.execute()
    return harness.get_output(_FUNC), harness.get_var("profile")


# (v0, a0, vT, aT, pd) with p0=0, pT=pd. All previously returned ERR_SOLVER.
_CASES = [
    # NONE / ACC0 short moves, rest velocities, nonzero accelerations
    (0.0, 1.0, 0.0, 3.0, 0.3),
    (0.0, 3.0, 0.0, 1.0, -0.3),    # mirror
    (0.0, -1.0, 0.0, -3.0, -0.3),
    (0.0, -3.0, 0.0, -1.0, 0.3),   # mirror
    # net-negative short moves with nonzero start accel
    (-2.0, -3.0, 0.0, -3.0, -2.0),
    (-2.0, -1.0, -0.7, -1.0, -2.0),
    (-2.0, -1.0, 0.0, -1.0, -2.0),
    (-2.0, -3.0, 0.7, 1.0, -2.0),
    (-2.0, -3.0, 0.7, 3.0, -2.0),
    # mirror (net-positive)
    (2.0, 3.0, 0.0, 3.0, 2.0),
    (2.0, 1.0, 0.7, 1.0, 2.0),
    (2.0, 1.0, -2.0, -1.0, -0.3),
]


@pytest.mark.parametrize("v0,a0,vT,aT,pd", _CASES)
def test_two_step_matches_oracle(harness, v0, a0, vT, aT, pd):
    args = (0.0, v0, a0, pd, vT, aT)
    tr = _oracle(*args)
    st, prof = _run(harness, *args)
    assert st == 0x7000, f"expected WORKING, got {hex(st)} for {args}"
    dur = sum(prof.t[i] for i in range(7))
    assert dur == pytest.approx(tr.duration, abs=1e-6), f"dur {dur} vs {tr.duration}"
    assert prof.p[7] == pytest.approx(pd, abs=1e-5), "target position"
    assert prof.v[7] == pytest.approx(vT, abs=1e-5), "target velocity"
    assert prof.a[7] == pytest.approx(aT, abs=1e-5), "target acceleration"
