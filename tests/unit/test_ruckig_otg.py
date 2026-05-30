"""Unit tests for the RuckigOtg FB — PLCopen DA011 continuous-enable lifecycle."""
import pytest


def default_input() -> dict:
    """A valid single-DOF rest-to-rest request (target 1.0 on axis 0)."""
    return {
        "currentPosition": [0.0] * 4,
        "currentVelocity": [0.0] * 4,
        "currentAcceleration": [0.0] * 4,
        "targetPosition": [1.0, 0.0, 0.0, 0.0],
        "targetVelocity": [0.0] * 4,
        "targetAcceleration": [0.0] * 4,
        "maxVelocity": [2.0] * 4,
        "maxAcceleration": [5.0] * 4,
        "maxJerk": [10.0] * 4,
        "enabled": [True, False, False, False],
        "nDofs": 1,
        "controlInterface": 0,
        "synchronization": 0,
        "durationDiscretization": 0,
    }


@pytest.fixture
def harness(make_harness):
    return make_harness("RuckigOtg.s7dcl")


def test_disabled_fb_returns_invalid_output(harness):
    """enable=False -> all status flags False, status FINISHED."""
    harness.set_inputs(enable=False, input=default_input(), cycleTime=0.010, reset=False)
    harness.execute()
    assert harness.get_output("valid") is False
    assert harness.get_output("busy") is False
    assert harness.get_output("done") is False
    assert harness.get_output("error") is False
    assert harness.get_output("status") == 0x0000


def test_invalid_input_returns_error(harness):
    """maxVelocity=0 -> error True, status RESULT_ERR_VMAX (0x8201)."""
    inp = default_input()
    inp["maxVelocity"][0] = 0.0
    harness.set_inputs(enable=True, input=inp, cycleTime=0.010, reset=False)
    harness.execute()
    assert harness.get_output("error") is True
    assert harness.get_output("status") == 0x8201
    assert harness.get_output("valid") is False


def test_first_call_with_valid_input_no_error(harness):
    """First enabled call with valid input clears error and reports WORKING."""
    harness.set_inputs(enable=True, input=default_input(), cycleTime=0.010, reset=False)
    harness.execute()
    assert harness.get_output("error") is False
    assert harness.get_output("status") == 0x7000


def test_full_trajectory_reaches_target(harness):
    """Run cycles at dt=10ms until done; final position must match the target."""
    inp = default_input()
    inp["targetPosition"][0] = 10.0
    inp["maxVelocity"][0] = 2.0
    inp["maxAcceleration"][0] = 2.0
    inp["maxJerk"][0] = 10.0

    final_pos = 0.0
    for _cycle in range(700):  # duration ~6.2 s -> ~620 cycles
        harness.set_inputs(enable=True, input=inp, cycleTime=0.010, reset=False)
        harness.execute()
        out = harness.get_output("output")
        final_pos = out.newPosition[0]
        if harness.get_output("done"):
            break

    assert final_pos == pytest.approx(10.0, abs=1e-3)
    assert harness.get_output("done") is True
    assert harness.get_output("status") == 0x0000


def test_zero_displacement_finishes_immediately(harness):
    """target == current -> done immediately, no error, position holds at p0."""
    inp = default_input()
    inp["targetPosition"][0] = 0.0  # current position is also 0.0
    harness.set_inputs(enable=True, input=inp, cycleTime=0.010, reset=False)
    harness.execute()
    assert harness.get_output("error") is False
    assert harness.get_output("done") is True
    assert harness.get_output("status") == 0x0000
    out = harness.get_output("output")
    assert out.newPosition[0] == pytest.approx(0.0, abs=1e-12)


def test_output_continuity_no_discontinuity(harness):
    """Position change per cycle must not exceed v_max * dt (smooth motion)."""
    inp = default_input()
    inp["targetPosition"][0] = 1.0

    prev_pos = 0.0
    for _cycle in range(100):
        harness.set_inputs(enable=True, input=inp, cycleTime=0.010, reset=False)
        harness.execute()
        out = harness.get_output("output")
        new_pos = out.newPosition[0]
        assert abs(new_pos - prev_pos) <= 2.0 * 0.010 + 1e-9
        prev_pos = new_pos


# ---------------------------------------------------------------------------
# v0.2 arbitrary-state behaviour: moving target, retarget chaining, brake.
# ---------------------------------------------------------------------------


def test_moving_target_reaches_target_velocity(harness):
    """Constant non-zero target velocity -> at done, v == targetVel and p == targetPos."""
    inp = default_input()
    inp["targetPosition"][0] = 1.0
    inp["targetVelocity"][0] = 0.5  # cruise at +0.5 at the target
    inp["maxVelocity"][0] = 2.0
    inp["maxAcceleration"][0] = 5.0
    inp["maxJerk"][0] = 10.0

    final_pos = 0.0
    final_vel = 0.0
    for _cycle in range(800):
        harness.set_inputs(enable=True, input=inp, cycleTime=0.010, reset=False)
        harness.execute()
        out = harness.get_output("output")
        final_pos = out.newPosition[0]
        final_vel = out.newVelocity[0]
        if harness.get_output("done"):
            break

    assert harness.get_output("done") is True
    assert harness.get_output("error") is False
    assert final_vel == pytest.approx(0.5, abs=1e-3)
    assert final_pos == pytest.approx(1.0, abs=1e-3)


def test_retarget_mid_motion_no_setpoint_jump(harness):
    """Change target 1.0 -> 2.0 mid-motion: no commanded jump, eventually reaches 2.0."""
    dt = 0.010
    vmax = 2.0
    inp = default_input()
    inp["targetPosition"][0] = 1.0
    inp["maxVelocity"][0] = vmax
    inp["maxAcceleration"][0] = 5.0
    inp["maxJerk"][0] = 10.0

    # Run ~20 cycles toward target 1.0.
    prev_pos = 0.0
    for _cycle in range(20):
        harness.set_inputs(enable=True, input=inp, cycleTime=dt, reset=False)
        harness.execute()
        prev_pos = harness.get_output("output").newPosition[0]

    # Retarget to 2.0 — assert NO setpoint jump on the recompute cycle.
    inp["targetPosition"][0] = 2.0
    harness.set_inputs(enable=True, input=inp, cycleTime=dt, reset=False)
    harness.execute()
    retarget_pos = harness.get_output("output").newPosition[0]
    jump = abs(retarget_pos - prev_pos)
    assert jump <= vmax * dt + 1e-6, f"setpoint jumped {jump} across retarget"

    # Continue to completion -> reaches 2.0.
    final_pos = retarget_pos
    for _cycle in range(800):
        harness.set_inputs(enable=True, input=inp, cycleTime=dt, reset=False)
        harness.execute()
        final_pos = harness.get_output("output").newPosition[0]
        if harness.get_output("done"):
            break

    assert harness.get_output("done") is True
    assert harness.get_output("error") is False
    assert final_pos == pytest.approx(2.0, abs=1e-3)


def test_first_enable_in_motion_over_vmax_brakes_then_converges(harness):
    """First enable with currentVelocity > vMax: brake, no error, converges to target."""
    dt = 0.010
    vmax = 2.0
    inp = default_input()
    inp["currentVelocity"][0] = 5.0  # well over vMax=2.0
    inp["targetPosition"][0] = 1.0
    inp["maxVelocity"][0] = vmax
    inp["maxAcceleration"][0] = 5.0
    inp["maxJerk"][0] = 10.0

    final_pos = 0.0
    final_vel = 0.0
    for _cycle in range(1500):
        harness.set_inputs(enable=True, input=inp, cycleTime=dt, reset=False)
        harness.execute()
        out = harness.get_output("output")
        final_pos = out.newPosition[0]
        final_vel = out.newVelocity[0]
        assert harness.get_output("error") is False
        if harness.get_output("done"):
            break

    assert harness.get_output("done") is True
    assert final_pos == pytest.approx(1.0, abs=1e-3)
    assert abs(final_vel) <= vmax + 1e-3


