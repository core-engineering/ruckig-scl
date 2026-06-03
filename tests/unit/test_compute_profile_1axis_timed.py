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
_FUNC = "ComputeProfile1AxisTimed"


@pytest.fixture
def harness(make_harness):
    return make_harness("ComputeProfile1AxisTimed.s7dcl")


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


def _run_timed(harness, p0, v0, a0, pT, vT, aT, tf, vM=_VMAX, aM=_AMAX, jM=_JMAX):
    harness.reset()
    harness.set_inputs(profile=_empty_profile(), p0=p0, v0=v0, a0=a0,
                       pT=pT, vT=vT, aT=aT, vMax=vM, aMax=aM, jMax=jM, tf=tf)
    harness.execute()
    return harness.get_output(_FUNC), harness.get_var("profile")




def _integrate_at(prof, p0, v0, a0, s):
    """Sample the SCL profile (its t[]/j[]) at absolute time s by walking the
    phases — frame-independent, so it sidesteps Ruckig's canonical profile frame."""
    a, v, p, elapsed = a0, v0, p0, 0.0
    for i in range(7):
        dt = prof.t[i]; j = prof.j[i]
        if s <= elapsed + dt + 1e-12:
            d = s - elapsed
            return (p + v * d + 0.5 * a * d * d + j * d ** 3 / 6.0,
                    v + a * d + 0.5 * j * d * d, a + j * d)
        p = p + v * dt + 0.5 * a * dt * dt + j * dt ** 3 / 6.0
        v = v + a * dt + 0.5 * j * dt * dt
        a = a + j * dt
        elapsed += dt
    return (p, v, a)


def _assert_timed_parity(harness, p0, v0, a0, pT, vT, aT, tf, abs_tol=1e-6):
    """step2 parity: SCL profile reaches the state at exactly tf and matches the
    Ruckig (minimum_duration) trajectory at sampled times."""
    st, prof = _run_timed(harness, p0, v0, a0, pT, vT, aT, tf)
    assert st == 0x7000, f"expected WORKING, got {hex(st)}"
    assert sum(prof.t[i] for i in range(7)) == pytest.approx(tf, abs=1e-7)
    tr = _oracle_timed(p0, v0, a0, pT, vT, aT, tf)
    for frac in (0.2, 0.4, 0.6, 0.8, 1.0):
        s = tf * frac
        po, vo, ao = (x[0] for x in tr.at_time(s))
        ps, vs, as_ = _integrate_at(prof, p0, v0, a0, s)
        assert ps == pytest.approx(po, abs=abs_tol), f"p at {s}"
        assert vs == pytest.approx(vo, abs=abs_tol), f"v at {s}"
        assert as_ == pytest.approx(ao, abs=abs_tol), f"a at {s}"


@pytest.mark.parametrize("k", [1.05, 1.1, 1.3])
def test_acc0_acc1_vel_stretched(harness, k):
    """T6: large rest-to-rest move, stretched to tf > t_min -> ACC0_ACC1_VEL
    (reaches vMax and both accel limits; the velocity plateau absorbs the slack)."""
    args = (0.0, 0.0, 0.0, 5.0, 0.0, 0.0)
    _assert_timed_parity(harness, *args, _t_min(*args) * k)


def test_acc0_acc1_vel_negative_direction(harness):
    """T6: net-negative move (up_first=False) -> the reversed dispatch pass.
    Exercises the mirror limits in the family + direction selection."""
    args = (0.0, 0.0, 0.0, -5.0, 0.0, 0.0)
    _assert_timed_parity(harness, *args, _t_min(*args) * 1.1)


# T9: ACC1_VEL — reaches -aMax (decel plateau) and the velocity plateau, not
# +aMax. Structure (1,0,1,1,1,1,1) with t[1]=0. (v0,a0,vT,aT,pd) with p0=0.
_ACC1_VEL_CASES = [
    (1.5, -2.0, 0.0, 0.0, 3.0),
    (1.5, -2.0, 0.0, -1.0, 3.0),
    (1.5, -2.0, 0.0, 1.0, 3.0),
]


@pytest.mark.parametrize("k", [1.0, 1.15])
@pytest.mark.parametrize("v0,a0,vT,aT,pd", _ACC1_VEL_CASES)
def test_acc1_vel(harness, v0, a0, vT, aT, pd, k):
    args = (0.0, v0, a0, pd, vT, aT)
    _assert_timed_parity(harness, *args, _t_min(*args) * k)


# T8: ACC0_VEL — reaches +aMax (accel plateau) and the velocity plateau, not
# -aMax. Structure (1,1,1,1,1,0,1) with t[5]=0. (v0,a0,vT,aT,pd) with p0=0.
_ACC0_VEL_CASES = [
    (-1.5, 2.0, 0.5, 0.0, 1.0),
    (-1.5, 2.0, 0.5, -1.0, 1.0),
    (-1.5, 2.0, 0.5, 1.0, 1.0),
    (-1.5, 0.0, 0.5, 0.0, 1.0),
]


@pytest.mark.parametrize("v0,a0,vT,aT,pd", _ACC0_VEL_CASES)
def test_acc0_vel(harness, v0, a0, vT, aT, pd):
    args = (0.0, v0, a0, pd, vT, aT)
    _assert_timed_parity(harness, *args, _t_min(*args) * 1.15)


# T7: VEL (UDDU) — reaches the velocity plateau, neither accel limit
# (t[1]=t[5]=0). Structure (1,0,1,1,1,0,1), jerk [+,0,-,0,-,0,+]. The general
# case is a degree-5 polynomial solved by sign-scan + ShrinkInterval + Newton.
# (v0,a0,vT,aT,pd,k) with p0=0; tf=t_min*k. UDUD VEL cases come with the UDUD pass.
_VEL_UDDU_CASES = [
    (0.0, -3.0, 0.0, 2.0, 0.3, 1.3),    # was a masked mismatch before time_vel
    (-1.5, 0.0, -1.5, 0.0, -2.0, 1.1),
    (-1.5, 2.0, -1.5, 0.0, -2.0, 1.3),
    (-1.5, -3.0, -1.5, 2.0, -2.0, 1.1),
]


@pytest.mark.parametrize("v0,a0,vT,aT,pd,k", _VEL_UDDU_CASES)
def test_vel_uddu(harness, v0, a0, vT, aT, pd, k):
    args = (0.0, v0, a0, pd, vT, aT)
    _assert_timed_parity(harness, *args, _t_min(*args) * k)


# T7: VEL (UDUD) — velocity plateau, jerk [+,0,-,0,+,0,-]. Degree-6 polynomial,
# double Newton step. These two were the masked mismatches before time_vel.
_VEL_UDUD_CASES = [
    (0.0, -3.0, 0.0, -3.0, 0.3, 1.3),
    (1.0, -3.0, 1.0, -3.0, 1.5, 1.1),
]


@pytest.mark.parametrize("v0,a0,vT,aT,pd,k", _VEL_UDUD_CASES)
def test_vel_udud(harness, v0, a0, vT, aT, pd, k):
    args = (0.0, v0, a0, pd, vT, aT)
    _assert_timed_parity(harness, *args, _t_min(*args) * k)


def test_oracle_timed_reaches_target():
    """The stretched trajectory still reaches the final state at tf."""
    args = (0.0, 0.5, 0.0, 1.0, 0.5, 0.0)
    tf = _t_min(*args) * 1.5
    tr = _oracle_timed(*args, tf)
    p, v, a = tr.at_time(tf)
    assert p[0] == pytest.approx(1.0, abs=1e-9)
    assert v[0] == pytest.approx(0.5, abs=1e-9)
    assert a[0] == pytest.approx(0.0, abs=1e-9)
