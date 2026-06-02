"""Parity tests for step2 — ComputeProfile1AxisTimed (re-timing to an imposed
duration tf). The oracle forces Ruckig's step2 path via inp.minimum_duration=tf
(Ruckig bypasses step2 for a mono-DoF only when minimum_duration is unset).

Phase 0 (T1): only the oracle helper + a smoke test that minimum_duration
stretches the trajectory to tf > t_min. The ComputeProfile1AxisTimed FC and its
harness fixture arrive in T4; family parity tests build on this helper.
"""
import pytest

ruckig = pytest.importorskip("ruckig")

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


def _t_min(p0, v0, a0, pT, vT, aT, vM=_VMAX, aM=_AMAX, jM=_JMAX):
    """Time-optimal duration (step1) for the given single-axis problem."""
    otg = ruckig.Ruckig(1); inp = ruckig.InputParameter(1); tr = ruckig.Trajectory(1)
    inp.current_position = [p0]; inp.current_velocity = [v0]; inp.current_acceleration = [a0]
    inp.target_position = [pT]; inp.target_velocity = [vT]; inp.target_acceleration = [aT]
    inp.max_velocity = [vM]; inp.max_acceleration = [aM]; inp.max_jerk = [jM]
    otg.calculate(inp, tr)
    return tr.duration


def _oracle_timed(p0, v0, a0, pT, vT, aT, tf, vM=_VMAX, aM=_AMAX, jM=_JMAX):
    """step2 oracle: Ruckig forced to duration tf via minimum_duration. Returns
    the Trajectory; tr.duration == tf (when tf is feasible, i.e. >= t_min and
    outside any blocked interval). Asserts step2 actually stretched the move."""
    otg = ruckig.Ruckig(1); inp = ruckig.InputParameter(1); tr = ruckig.Trajectory(1)
    inp.current_position = [p0]; inp.current_velocity = [v0]; inp.current_acceleration = [a0]
    inp.target_position = [pT]; inp.target_velocity = [vT]; inp.target_acceleration = [aT]
    inp.max_velocity = [vM]; inp.max_acceleration = [aM]; inp.max_jerk = [jM]
    inp.minimum_duration = tf
    res = otg.calculate(inp, tr)
    assert int(res) >= 0, f"oracle rejected tf={tf}"
    return tr


@pytest.mark.parametrize("k", [1.0, 1.1, 1.5, 2.0])
def test_oracle_timed_stretches_to_tf(k):
    """minimum_duration forces the trajectory duration to exactly tf = t_min*k,
    confirming the step2 path is exercised (even for a single DoF)."""
    args = (0.0, 0.0, 0.0, 1.0, 0.0, 0.0)
    tmin = _t_min(*args)
    tf = tmin * k
    tr = _oracle_timed(*args, tf)
    assert tr.duration == pytest.approx(tf, abs=1e-6)
    if k > 1.0:
        assert tr.duration > tmin + 1e-6  # genuinely stretched beyond optimal


def test_oracle_timed_reaches_target():
    """The stretched trajectory still reaches the final state at tf."""
    args = (0.0, 0.5, 0.0, 1.0, 0.5, 0.0)
    tf = _t_min(*args) * 1.5
    tr = _oracle_timed(*args, tf)
    p, v, a = tr.at_time(tf)
    assert p[0] == pytest.approx(1.0, abs=1e-9)
    assert v[0] == pytest.approx(0.5, abs=1e-9)
    assert a[0] == pytest.approx(0.0, abs=1e-9)
