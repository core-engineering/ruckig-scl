import pytest
ruckig = pytest.importorskip("ruckig")


@pytest.fixture
def harness(make_harness):
    return make_harness("ComputeProfile1Dof.s7dcl")


# The FUNCTION inside ComputeProfile1Dof.s7dcl is named "ComputeProfile1Axis":
# the plc-code SCL lexer mangles a return-variable whose name ends in "Dof"
# (the trailing "of" is read as the OF keyword). The harness keys outputs on
# the FUNCTION name, so the Word return is read via this name.
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
    # spot-check final position equals target
    pT = args[3]
    assert prof.p[7] == pytest.approx(pT, abs=abs_state)


def test_rest_to_rest_long_acc0acc1vel(harness):
    # 0->10 at rest, reaches vmax (full trapezoid ACC0_ACC1_VEL)
    _check_matches_oracle(harness, (0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 2.0, 2.0, 10.0))


def test_nonrest_start_vel(harness):
    # starts with v0=1.0 (still vel-reaching, long displacement)
    _check_matches_oracle(harness, (0.0, 1.0, 0.0, 10.0, 0.0, 0.0, 2.0, 2.0, 10.0))


def test_moving_target_vel(harness):
    # target velocity vT=1.0 (vel-reaching)
    _check_matches_oracle(harness, (0.0, 0.0, 0.0, 10.0, 1.0, 0.0, 2.0, 3.0, 10.0))


# --- ACC0_ACC1 family: short moves, no velocity plateau (t[3]=0) ---
# High vMax so velocity never saturates; both accel limits are reached.

def test_acc0_acc1_rest_to_rest_short(harness):
    # oracle: ACC0_ACC1, t[3]=0, duration ~= 1.62828568570857
    args = (0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 10.0, 2.0, 10.0)
    _check_matches_oracle(harness, args)
    _, _, prof = _run(harness, *args)
    assert prof.t[3] == pytest.approx(0.0, abs=1e-12)


def test_acc0_acc1_rest_to_rest_shorter(harness):
    # oracle: ACC0_ACC1, t[3]=0, duration ~= 1.21980390271856
    args = (0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 10.0, 2.0, 10.0)
    _check_matches_oracle(harness, args)
    _, _, prof = _run(harness, *args)
    assert prof.t[3] == pytest.approx(0.0, abs=1e-12)


def test_acc0_acc1_nonzero_v0_a0(harness):
    # oracle: ACC0_ACC1, t[3]=0, duration ~= 1.77273663157048
    args = (0.0, 0.2, 1.0, 1.5, 0.0, 0.0, 10.0, 2.0, 10.0)
    _check_matches_oracle(harness, args)
    _, _, prof = _run(harness, *args)
    assert prof.t[3] == pytest.approx(0.0, abs=1e-12)


def test_acc0_acc1_nonzero_aT(harness):
    # oracle: ACC0_ACC1, t[3]=0, duration ~= 1.68572943613517
    args = (0.0, 0.0, 0.0, 1.0, 0.0, 0.5, 10.0, 2.0, 10.0)
    _check_matches_oracle(harness, args)
    _, _, prof = _run(harness, *args)
    assert prof.t[3] == pytest.approx(0.0, abs=1e-12)


def test_acc0_acc1_nonzero_a0_and_aT(harness):
    # nonzero initial AND target acceleration together -> ACC0_ACC1 (t[3]=0);
    # exercises the full asymmetric a0/aT interaction in num1/h2/h3/t[6]
    # oracle: ACC0_ACC1, t[3]=0, duration ~= 1.7856714977213048
    args = (0.0, 0.3, 1.0, 1.5, 0.0, 0.5, 10.0, 2.0, 10.0)
    _check_matches_oracle(harness, args)
    _, _, prof = _run(harness, *args)
    assert prof.t[3] == pytest.approx(0.0, abs=1e-12)


# --- time_all_none_acc0_acc1 family: quartic, no vel plateau, <=1 accel limit ---
# Ruckig's most numerically delicate family ("this one is in particular prone to
# numerical issues"). Each sub-case is a monic quartic whose real roots (refined
# by Newton) become candidate phase-3 times. The NONE sub-case (no velocity
# plateau, neither acceleration limit reached: only the three jerk phases
# t[0],t[2],t[6] are non-zero) is the one the oracle selects as time-optimal.
# All cases below were verified oracle-valid AND reproduced by the SCL solver at
# the floating-point floor (duration error < 1e-6, t[1]=t[3]=t[5]=0).


def test_none_rest_to_rest(harness):
    # NONE sub-case, rest-to-rest. oracle duration ~= 1.169607095285147
    args = (0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 10.0, 3.0, 10.0)
    _check_matches_oracle(harness, args)
    _, _, prof = _run(harness, *args)
    assert prof.t[1] == pytest.approx(0.0, abs=1e-12)
    assert prof.t[3] == pytest.approx(0.0, abs=1e-12)
    assert prof.t[5] == pytest.approx(0.0, abs=1e-12)


def test_none_asymmetric_a(harness):
    # NONE with asymmetric nonzero v0/a0/vT/aT. oracle duration ~= 1.141664228710
    args = (0.0, -0.22179, 1.47189, 0.51667, 0.30549, 1.12091, 8.0, 4.0, 10.0)
    _check_matches_oracle(harness, args)
    _, _, prof = _run(harness, *args)
    assert prof.t[1] == pytest.approx(0.0, abs=1e-12)
    assert prof.t[3] == pytest.approx(0.0, abs=1e-12)
    assert prof.t[5] == pytest.approx(0.0, abs=1e-12)


def test_none_asymmetric_b(harness):
    # NONE, opposite-sign target acceleration (aT<0), high jerk.
    # oracle duration ~= 0.562531013815
    args = (0.0, 1.26092, 1.9389, 0.80212, 0.93598, -2.37124, 8.0, 4.0, 15.0)
    _check_matches_oracle(harness, args)
    _, _, prof = _run(harness, *args)
    assert prof.t[1] == pytest.approx(0.0, abs=1e-12)
    assert prof.t[3] == pytest.approx(0.0, abs=1e-12)
    assert prof.t[5] == pytest.approx(0.0, abs=1e-12)


# NOTE: a fourth NONE case (e.g. v0=-0.86103, a0=-2.56798, pT=-0.46958,
# vT=-0.44794, aT=0.48106, vMax=8, aMax=5, jMax=10 -> oracle 1.828037995792)
# was verified solvable in isolation but is omitted: the plc-code test harness
# degrades once a single test module instantiates a 12th make_harness fixture
# (8/9/10/11 fixtures pass cleanly; the 12th makes earlier executions return
# stale/NaN state). Three NONE cases (11 total in this module) stay safely
# below that harness cliff while covering rest-to-rest plus two asymmetric
# nonzero-a0/aT geometries.

# NOTE: the ACC0 and ACC1 sub-cases of time_all_none_acc0_acc1 (a single
# acceleration plateau, t[1]>0 XOR t[5]>0, no vel plateau) could NOT be
# provoked as the *time-optimal* solution from the official ruckig oracle.
# In sweeps of thousands of oracle-valid, no-vel-plateau configs that the SCL
# solver reproduces at the FP floor, the accepted optimum is always the NONE
# sub-case, the VEL family, or the full ACC0_ACC1 family; none reached a single
# acceleration limit as the optimum. The ACC0/ACC1 regions are still ported
# faithfully and evaluated on every call -- their quartics are solved, roots
# Newton-refined and run through CheckProfile, which together with min-duration
# bookkeeping correctly leaves them unselected whenever a shorter feasible
# profile exists.
