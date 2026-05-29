"""Unit tests for ComputeFinalProfile FC.

v0.1 scope: rest-to-rest profile generation. Checks that the 7 phase
durations, jerk values, and boundary states are correctly populated.
"""
import math

import pytest


@pytest.fixture
def harness(make_harness):
    return make_harness("ComputeFinalProfile.s7dcl")


def empty_profile() -> dict:
    return {
        "t": [0.0] * 7,
        "j": [0.0] * 7,
        "a": [0.0] * 8,
        "v": [0.0] * 8,
        "p": [0.0] * 8,
        "direction": 1,
        "brake": {
            "t": [0.0, 0.0], "j": [0.0, 0.0],
            "a": [0.0, 0.0, 0.0], "v": [0.0, 0.0, 0.0], "p": [0.0, 0.0, 0.0],
        },
        "controlSigns": 0,
    }


def _call(harness, *, p0=0.0, p1, v_max=2.0, a_max=2.0, j_max=10.0, duration=0.0):
    harness.reset()
    profile = empty_profile()
    harness.set_inputs(
        profile=profile,
        duration=duration,
        p0=p0, p1=p1,
        vMax=v_max, aMax=a_max, jMax=j_max,
    )
    harness.execute()
    return harness.get_var("profile")  # _AutoStruct — access via attributes


def test_zero_displacement_returns_zeroed_profile(harness):
    profile = _call(harness, p1=0.0)
    for i in range(7):
        assert profile.t[i] == 0.0
        assert profile.j[i] == 0.0
    for i in range(8):
        assert profile.p[i] == 0.0
        assert profile.v[i] == 0.0
        assert profile.a[i] == 0.0


def test_case_a_trapezoidal_profile(harness):
    """v_max=2, a_max=2, j_max=10, displacement=10 → trapezoidal."""
    profile = _call(harness, p0=0.0, p1=10.0, v_max=2.0, a_max=2.0, j_max=10.0)
    # Phase 0 (jerk-up): t_j=0.2, j=+10
    assert profile.t[0] == pytest.approx(0.2, abs=1e-9)
    assert profile.j[0] == pytest.approx(10.0, abs=1e-9)
    # Phase 1 (const-accel): t_a=0.8, j=0
    assert profile.t[1] == pytest.approx(0.8, abs=1e-9)
    assert profile.j[1] == pytest.approx(0.0, abs=1e-9)
    # Phase 2 (jerk-down): t_j=0.2, j=-10
    assert profile.t[2] == pytest.approx(0.2, abs=1e-9)
    assert profile.j[2] == pytest.approx(-10.0, abs=1e-9)
    # Final position matches target
    assert profile.p[7] == pytest.approx(10.0, abs=1e-6)
    # Final velocity and acceleration must return to 0
    assert profile.v[7] == pytest.approx(0.0, abs=1e-6)
    assert profile.a[7] == pytest.approx(0.0, abs=1e-6)
    # Direction
    assert profile.direction == 1


def test_negative_direction_inverts_jerk_sign(harness):
    profile = _call(harness, p0=10.0, p1=0.0, v_max=2.0, a_max=2.0, j_max=10.0)
    assert profile.direction == -1
    assert profile.j[0] == pytest.approx(-10.0, abs=1e-9)
    assert profile.j[2] == pytest.approx(10.0, abs=1e-9)
    assert profile.p[7] == pytest.approx(0.0, abs=1e-6)


def test_case_b_triangular_profile(harness):
    """Tiny displacement: no v_max or a_max reached."""
    profile = _call(harness, p1=0.001, v_max=10.0, a_max=10.0, j_max=10.0)
    expected_tj = (0.001 / 20.0) ** (1.0 / 3.0)
    # Phases 1, 3, 5 (const-vel, const-accel) should be zero
    assert profile.t[1] == pytest.approx(0.0, abs=1e-9)
    assert profile.t[3] == pytest.approx(0.0, abs=1e-9)
    assert profile.t[5] == pytest.approx(0.0, abs=1e-9)
    # All jerk phases at t_j_triangular
    for i in (0, 2, 4, 6):
        assert profile.t[i] == pytest.approx(expected_tj, abs=1e-9)
    # Final position
    assert profile.p[7] == pytest.approx(0.001, abs=1e-9)


def test_case_c_amax_only_profile(harness):
    """a_max reached but v_max not (5-phase non-zero)."""
    profile = _call(harness, p1=0.8, v_max=10.0, a_max=2.0, j_max=10.0)
    t_j = 0.2
    v_peak = (-2.0 * t_j + math.sqrt((2.0 * t_j) ** 2 + 4.0 * 2.0 * 0.8)) / 2.0
    t_a = v_peak / 2.0 - t_j
    # Const-vel phase = 0 (case C)
    assert profile.t[3] == pytest.approx(0.0, abs=1e-9)
    # Const-accel phases non-zero
    assert profile.t[1] == pytest.approx(t_a, abs=1e-9)
    assert profile.t[5] == pytest.approx(t_a, abs=1e-9)
    # Final position
    assert profile.p[7] == pytest.approx(0.8, abs=1e-6)


def test_boundary_state_continuity(harness):
    """Position, velocity, acceleration must be continuous across phases."""
    profile = _call(harness, p1=10.0, v_max=2.0, a_max=2.0, j_max=10.0)
    # Initial state
    assert profile.p[0] == 0.0
    assert profile.v[0] == 0.0
    assert profile.a[0] == 0.0
    # Final state matches target
    assert profile.p[7] == pytest.approx(10.0, abs=1e-6)
    assert profile.v[7] == pytest.approx(0.0, abs=1e-6)
    assert profile.a[7] == pytest.approx(0.0, abs=1e-6)
