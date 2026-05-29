"""Unit tests for StateAtTime FC."""
import pytest


@pytest.fixture
def harness(make_harness):
    return make_harness("StateAtTime.s7dcl")


def trapezoidal_profile(p0=0.0, p1=10.0, vmax=2.0, amax=2.0, jmax=10.0) -> dict:
    """Pre-computed 7-phase trapezoidal profile (case A) — matches the case
    used in ComputeFinalProfile tests."""
    tj, ta = 0.2, 0.8
    s_to_vmax = vmax * (2.0 * tj + ta)
    tv = (p1 - p0 - s_to_vmax) / vmax
    t = [tj, ta, tj, tv, tj, ta, tj]
    j = [+jmax, 0.0, -jmax, 0.0, -jmax, 0.0, +jmax]
    a = [0.0]
    v = [0.0]
    p = [p0]
    for i in range(7):
        a.append(a[-1] + j[i] * t[i])
        v.append(v[-1] + a[-2] * t[i] + 0.5 * j[i] * t[i] ** 2)
        p.append(p[-1] + v[-2] * t[i] + 0.5 * a[-2] * t[i] ** 2 + j[i] * t[i] ** 3 / 6.0)
    return {
        "t": t, "j": j, "a": a, "v": v, "p": p,
        "direction": 1,
        "brake": {"t": [0.0, 0.0], "j": [0.0, 0.0],
                  "a": [0.0, 0.0, 0.0], "v": [0.0, 0.0, 0.0], "p": [0.0, 0.0, 0.0]},
        "controlSigns": 0,
    }


def _eval(harness, profile: dict, t: float):
    harness.reset()
    harness.set_inputs(profile=profile, t=t)
    harness.execute()
    return (
        harness.get_output("p"),
        harness.get_output("v"),
        harness.get_output("a"),
    )


def test_state_at_t_zero(harness):
    p, v, a = _eval(harness, trapezoidal_profile(), 0.0)
    assert p == pytest.approx(0.0, abs=1e-9)
    assert v == pytest.approx(0.0, abs=1e-9)
    assert a == pytest.approx(0.0, abs=1e-9)


def test_state_at_t_end(harness):
    p, v, a = _eval(harness, trapezoidal_profile(), 6.2)  # total duration
    assert p == pytest.approx(10.0, abs=1e-6)
    assert v == pytest.approx(0.0, abs=1e-6)
    assert a == pytest.approx(0.0, abs=1e-6)


def test_state_at_end_of_phase_zero(harness):
    """End of jerk-up (t=0.2): a=a_max, v=0.5*j*t² = 0.2, p = j*t³/6 = 0.0133."""
    p, v, a = _eval(harness, trapezoidal_profile(), 0.2)
    assert a == pytest.approx(2.0, abs=1e-6)
    assert v == pytest.approx(0.2, abs=1e-6)
    assert p == pytest.approx(10.0 * 0.008 / 6.0, abs=1e-6)


def test_state_in_middle_of_const_vel(harness):
    """Middle of constant-velocity phase (phase 3). At t = 0.2 + 0.8 + 0.2 + 1.9
    we are halfway through the const-vel phase (which lasts 3.8s)."""
    profile = trapezoidal_profile()
    t_mid_const_vel = 0.2 + 0.8 + 0.2 + 1.9  # halfway into the v=v_max plateau
    p, v, a = _eval(harness, profile, t_mid_const_vel)
    assert v == pytest.approx(2.0, abs=1e-6)
    assert a == pytest.approx(0.0, abs=1e-6)


def test_state_beyond_trajectory_clamps(harness):
    """t > duration → returns final state."""
    profile = trapezoidal_profile()
    p, v, a = _eval(harness, profile, 100.0)
    assert p == pytest.approx(10.0, abs=1e-6)
    assert v == pytest.approx(0.0, abs=1e-6)
    assert a == pytest.approx(0.0, abs=1e-6)
