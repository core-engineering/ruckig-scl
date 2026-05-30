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
