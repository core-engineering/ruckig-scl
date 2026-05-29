"""Unit tests for ComputeMinDuration FC.

v0.1 scope: rest-to-rest single-axis (current_v = current_a = target_v = target_a = 0).
Three sub-cases:
- Case A (long): displacement reaches v_max, full trapezoidal jerk
- Case B (triangular): neither v_max nor a_max reached
- Case C (medium): a_max reached but not v_max
"""
import math

import pytest


@pytest.fixture
def harness(make_harness):
    return make_harness("ComputeMinDuration.s7dcl")


def _call(harness, *, p0=0.0, p1, v_max=2.0, a_max=5.0, j_max=10.0) -> float:
    """Compute duration starting from rest, ending at rest."""
    harness.reset()
    harness.set_inputs(
        currentPos=p0, currentVel=0.0, currentAcc=0.0,
        targetPos=p1, targetVel=0.0, targetAcc=0.0,
        maxVel=v_max, maxAcc=a_max, maxJerk=j_max,
    )
    harness.execute()
    return harness.get_output("ComputeMinDuration")


def test_zero_displacement_returns_zero(harness):
    assert _call(harness, p1=0.0) == pytest.approx(0.0, abs=1e-9)


def test_long_displacement_case_a_trapezoidal(harness):
    """v_max=2, a_max=2, j_max=10 → t_j=0.2, t_a=0.8
    sToVmax = vMax*(2*tJ + tA) = 2*(0.4 + 0.8) = 2.4
    t_v = (10 - 2.4) / 2 = 3.8
    total = 2*(2*0.2 + 0.8) + 3.8 = 2.4 + 3.8 = 6.2"""
    result = _call(harness, p1=10.0, v_max=2.0, a_max=2.0, j_max=10.0)
    t_j, t_a = 0.2, 0.8
    s_to_vmax = 2.0 * (2.0 * t_j + t_a)
    expected = 2.0 * (2.0 * t_j + t_a) + (10.0 - s_to_vmax) / 2.0
    assert result == pytest.approx(expected, abs=1e-6)


def test_short_displacement_case_b_triangular(harness):
    """Tiny displacement: triangular jerk; t_j = (abs_delta/(2*j_max))^(1/3)
    With p1=0.001, j_max=10: t_j = (0.001/20)^(1/3); duration = 4*t_j"""
    result = _call(harness, p1=0.001, v_max=10.0, a_max=10.0, j_max=10.0)
    expected_tj = (0.001 / 20.0) ** (1.0 / 3.0)
    assert result == pytest.approx(4.0 * expected_tj, abs=1e-6)


def test_medium_displacement_case_c_amax_reached(harness):
    """v_max=10 (loose), a_max=2, j_max=10, target=0.8
    → t_j_triangular = (0.8/20)^(1/3) ≈ 0.342, a_peak = 3.42 > a_max=2 → CASE C
    → V_peak from V² + 0.4*V - 1.6 = 0 → V_peak ≈ 1.0807
    → t_a = V_peak/2 - 0.2 ≈ 0.3403
    → T = 4*0.2 + 2*0.3403 ≈ 1.4807"""
    result = _call(harness, p1=0.8, v_max=10.0, a_max=2.0, j_max=10.0)
    t_j = 2.0 / 10.0  # 0.2
    v_peak = (-2.0 * t_j + math.sqrt((2.0 * t_j) ** 2 + 4.0 * 2.0 * 0.8)) / 2.0
    t_a = v_peak / 2.0 - t_j
    expected = 4.0 * t_j + 2.0 * t_a
    assert result == pytest.approx(expected, abs=1e-6)


def test_negative_displacement_same_as_positive(harness):
    """Duration is direction-independent (function of |delta|)."""
    pos = _call(harness, p1=1.0)
    neg = _call(harness, p1=-1.0)
    assert pos == pytest.approx(neg, abs=1e-9)


def test_negative_origin_positive_target(harness):
    """Same |delta|, different origin → same duration."""
    a = _call(harness, p0=0.0, p1=10.0, v_max=2.0, a_max=2.0, j_max=10.0)
    b = _call(harness, p0=-5.0, p1=5.0, v_max=2.0, a_max=2.0, j_max=10.0)
    assert a == pytest.approx(b, abs=1e-9)
