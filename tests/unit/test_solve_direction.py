import pytest
ruckig = pytest.importorskip("ruckig")


# Mirror-direction parity for ComputeProfile1Axis (the SolveDirection second pass).
# Ruckig step1 has no separate UDUD profile equations: the negative jerk direction
# is obtained by running the SAME three families with MIRRORED limits
# (svMax=-vMax, svMin=+vMax, saMax=-aMax, saMin=+aMax, sjMax=-jMax). For inputs
# whose time-optimal profile is in the mirror direction (net-negative moves, or
# moving targets with vT<0), the positive pass alone returns RESULT_ERR_SOLVER;
# the mirror pass recovers them. ComputeProfile1Axis runs BOTH passes and keeps
# the global minimum-duration candidate.
#
# This lives in its own module: the plc-code harness degrades once a single test
# module instantiates a 12th make_harness fixture (documented in
# test_compute_profile_1dof.py). Keeping the mirror cases here stays well below
# that cliff in both modules.


@pytest.fixture
def harness(make_harness):
    return make_harness("ComputeProfile1Dof.s7dcl")


_FUNC_NAME = "ComputeProfile1Axis"


def _empty_profile():
    return {
        "t": [0.0] * 7, "j": [0.0] * 7,
        "a": [0.0] * 8, "v": [0.0] * 8, "p": [0.0] * 8,
        "direction": 1,
        "brake": {"t": [0.0, 0.0], "j": [0.0, 0.0],
                  "a": [0.0, 0.0, 0.0], "v": [0.0, 0.0, 0.0], "p": [0.0, 0.0, 0.0]},
        "controlSigns": 0,
    }


def _oracle(p0, v0, a0, pT, vT, aT, vM, aM, jM):
    otg = ruckig.Ruckig(1); inp = ruckig.InputParameter(1); tr = ruckig.Trajectory(1)
    inp.current_position = [p0]; inp.current_velocity = [v0]; inp.current_acceleration = [a0]
    inp.target_position = [pT]; inp.target_velocity = [vT]; inp.target_acceleration = [aT]
    inp.max_velocity = [vM]; inp.max_acceleration = [aM]; inp.max_jerk = [jM]
    otg.calculate(inp, tr)
    return tr


def _run(harness, p0, v0, a0, pT, vT, aT, vM, aM, jM):
    harness.reset()
    harness.set_inputs(profile=_empty_profile(), p0=p0, v0=v0, a0=a0,
                       pT=pT, vT=vT, aT=aT, vMax=vM, aMax=aM, jMax=jM)
    harness.execute()
    status = harness.get_output(_FUNC_NAME)
    prof = harness.get_var("profile")
    dur = sum(prof.t[i] for i in range(7))
    return status, dur, prof


def _check_matches_oracle(harness, args, abs_dur=1e-6, abs_state=1e-5):
    st, dur, prof = _run(harness, *args)
    tr = _oracle(*args)
    assert st == 0x7000, f"expected WORKING, got {hex(st)}"
    assert dur == pytest.approx(tr.duration, abs=abs_dur), f"dur {dur} vs {tr.duration}"
    pT = args[3]
    assert prof.p[7] == pytest.approx(pT, abs=abs_state)


# --- (a) pure negative rest-to-rest large move (pd<0, reaches vMax) ---
def test_mirror_neg_rest_to_rest_long(harness):
    # mirror of test_rest_to_rest_long_acc0acc1vel; oracle duration = 6.2
    args = (0.0, 0.0, 0.0, -10.0, 0.0, 0.0, 2.0, 2.0, 10.0)
    _check_matches_oracle(harness, args)


# --- (b) moving target with vT<0 (mirror velocity-reaching) ---
def test_mirror_moving_target_neg_vT(harness):
    # net-negative move with negative target velocity; oracle duration ~= 5.64166666666667
    args = (0.0, 0.0, 0.0, -10.0, -1.0, 0.0, 2.0, 3.0, 10.0)
    _check_matches_oracle(harness, args)


# --- (c) short negative move (mirror ACC0_ACC1, no vel plateau, t[3]=0) ---
def test_mirror_neg_short_acc0_acc1(harness):
    # mirror of test_acc0_acc1_rest_to_rest_short; oracle duration ~= 1.62828568570857
    args = (0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 10.0, 2.0, 10.0)
    _check_matches_oracle(harness, args)
    _, _, prof = _run(harness, *args)
    assert prof.t[3] == pytest.approx(0.0, abs=1e-12)


# --- (c2) short negative move that selects the mirror NONE quartic sub-case ---
def test_mirror_neg_quartic_none(harness):
    # net-negative move, no vel plateau; oracle duration ~= 1.81712059283214
    args = (0.0, 0.0, 0.0, -1.5, 0.0, 0.0, 10.0, 5.0, 8.0)
    _check_matches_oracle(harness, args)


# --- (d) BOTH directions yield a valid profile, mirror is strictly shorter ---
# Moving target reachable by either jerk direction. SolveDirection accepts a
# valid profile in BOTH passes for this geometry:
#   positive pass duration ~= 0.66693457   (valid, but suboptimal)
#   mirror   pass duration ~= 0.51274474   (valid, global minimum == oracle)
# ComputeProfile1Axis must return the mirror (shorter) duration, proving it keeps
# the global minimum across both passes rather than the first profile found.
_BOTH_VALID = (0.0, -1.0, 1.0, -0.5, -1.0, -1.0, 5.0, 3.0, 10.0)


def _solve_one_direction(make_harness, args, sign):
    """Drive SolveDirection alone in one jerk direction; return (found, duration)."""
    h = make_harness("SolveDirection.s7dcl")
    p0, v0, a0, pT, vT, aT, vM, aM, jM = args
    h.reset()
    h.set_inputs(profile=_empty_profile(),
                 bestT=[0.0] * 7, bestDuration=1.0e30, found=False, bestSjMax=jM,
                 p0=p0, v0=v0, a0=a0, pT=pT, vT=vT, aT=aT,
                 svMax=sign * vM, svMin=-sign * vM,
                 saMax=sign * aM, saMin=-sign * aM, sjMax=sign * jM)
    h.execute()
    return h.get_var("found"), h.get_var("bestDuration")


def test_both_directions_valid_global_min(make_harness):
    # Each direction in isolation finds a valid profile, mirror strictly shorter.
    fp, dp = _solve_one_direction(make_harness, _BOTH_VALID, 1.0)
    fm, dm = _solve_one_direction(make_harness, _BOTH_VALID, -1.0)
    assert fp and fm, f"expected both directions valid, got pos={fp} mir={fm}"
    assert dm < dp - 1e-3, f"expected mirror strictly shorter: pos={dp} mir={dm}"


def test_mirror_both_valid_mirror_shorter(harness):
    # ComputeProfile1Axis runs both passes and must keep the global min (mirror).
    args = _BOTH_VALID
    tr = _oracle(*args)
    st, dur, prof = _run(harness, *args)
    assert st == 0x7000, f"expected WORKING, got {hex(st)}"
    assert dur == pytest.approx(tr.duration, abs=1e-6), f"dur {dur} vs {tr.duration}"
    assert prof.p[7] == pytest.approx(args[3], abs=1e-5)
    # winning profile is the mirror direction: leading jerk is negative
    assert prof.j[0] < 0.0
