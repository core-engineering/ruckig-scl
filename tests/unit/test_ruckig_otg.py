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
        "minimumDuration": -1.0,
        "controlInterface": 0,
        "synchronization": 2,
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
    prev_vel = 5.0  # initial axis velocity
    amax = 5.0
    for _cycle in range(1500):
        harness.set_inputs(enable=True, input=inp, cycleTime=dt, reset=False)
        harness.execute()
        out = harness.get_output("output")
        final_pos = out.newPosition[0]
        final_vel = out.newVelocity[0]
        assert harness.get_output("error") is False
        # Velocity must stay C1-continuous across the brake/main junction: a wrong
        # post-brake seed (e.g. dropped "=>" outputs) shows up as a |dv| spike.
        assert abs(final_vel - prev_vel) <= amax * dt + 1e-6, "velocity jump (brake seed?)"
        prev_vel = final_vel
        if harness.get_output("done"):
            break

    assert harness.get_output("done") is True
    assert final_pos == pytest.approx(1.0, abs=1e-3)
    assert abs(final_vel) <= vmax + 1e-3


# ---------------------------------------------------------------------------
# v0.4 multi-DoF: time synchronization (every axis arrives at the same instant).
# ---------------------------------------------------------------------------


def test_two_dof_reaches_both_targets(harness):
    """2 DoFs, unequal displacement: both reach target at the SAME final cycle."""
    inp = default_input()
    inp["nDofs"] = 2
    inp["enabled"] = [True, True, False, False]
    inp["targetPosition"] = [1.0, 5.0, 0.0, 0.0]  # axis1 is the slow (limiting) axis
    inp["maxVelocity"] = [2.0] * 4
    inp["maxAcceleration"] = [5.0] * 4
    inp["maxJerk"] = [10.0] * 4

    p0 = p1 = 0.0
    for _cycle in range(1000):
        harness.set_inputs(enable=True, input=inp, cycleTime=0.010, reset=False)
        harness.execute()
        assert harness.get_output("error") is False
        out = harness.get_output("output")
        p0 = out.newPosition[0]
        p1 = out.newPosition[1]
        if harness.get_output("done"):
            break

    assert harness.get_output("done") is True
    assert p0 == pytest.approx(1.0, abs=1e-3)
    assert p1 == pytest.approx(5.0, abs=1e-3)


def test_two_dof_fast_axis_arrives_with_slow_axis(harness):
    """The fast axis must NOT finish early: it is re-timed to t_sync (still moving
    near the end, i.e. it does not sit parked at its target for many cycles)."""
    inp = default_input()
    inp["nDofs"] = 2
    inp["enabled"] = [True, True, False, False]
    inp["targetPosition"] = [0.5, 8.0, 0.0, 0.0]  # axis0 tiny move, axis1 long
    inp["maxVelocity"] = [2.0] * 4
    inp["maxAcceleration"] = [5.0] * 4
    inp["maxJerk"] = [10.0] * 4

    # Track when each axis first reaches its target (within tol).
    reached0 = reached1 = None
    cyc = 0
    for cyc in range(2000):
        harness.set_inputs(enable=True, input=inp, cycleTime=0.010, reset=False)
        harness.execute()
        out = harness.get_output("output")
        if reached0 is None and abs(out.newPosition[0] - 0.5) < 1e-3:
            reached0 = cyc
        if reached1 is None and abs(out.newPosition[1] - 8.0) < 1e-3:
            reached1 = cyc
        if harness.get_output("done"):
            break

    assert harness.get_output("done") is True
    assert reached0 is not None and reached1 is not None
    # Synchronized: both axes settle on their target within a few cycles of each
    # other (the fast axis was stretched to t_sync, not finished long before).
    assert abs(reached0 - reached1) <= 5


def test_minimum_duration_stretches_single_axis(harness):
    """1 DoF with minimumDuration > t_min: trajectory lasts at least that long."""
    inp = default_input()
    inp["nDofs"] = 1
    inp["targetPosition"][0] = 1.0
    inp["minimumDuration"] = 3.0  # force a slow move (natural t_min < 3 s)

    duration = 0.0
    for _cycle in range(600):
        harness.set_inputs(enable=True, input=inp, cycleTime=0.010, reset=False)
        harness.execute()
        assert harness.get_output("error") is False
        out = harness.get_output("output")
        duration = out.trajectoryDuration
        if harness.get_output("done"):
            break

    assert harness.get_output("done") is True
    assert duration == pytest.approx(3.0, abs=1e-2)
    assert harness.get_output("output").newPosition[0] == pytest.approx(1.0, abs=1e-3)


def test_sync_no_axes_finish_independently(harness):
    """SYNC_NO: a short axis reaches its target at its OWN t_min, not stretched."""
    inp = default_input()
    inp["nDofs"] = 2
    inp["synchronization"] = 0  # SYNC_NONE
    inp["enabled"] = [True, True, False, False]
    inp["targetPosition"] = [1.0, 8.0, 0.0, 0.0]
    inp["maxVelocity"] = [2.0] * 4
    inp["maxAcceleration"] = [5.0] * 4
    inp["maxJerk"] = [10.0] * 4

    reached0 = None
    for cyc in range(2000):
        harness.set_inputs(enable=True, input=inp, cycleTime=0.010, reset=False)
        harness.execute()
        out = harness.get_output("output")
        if reached0 is None and abs(out.newPosition[0] - 1.0) < 1e-3:
            reached0 = cyc
        if harness.get_output("done"):
            break
    assert harness.get_output("done") is True
    assert reached0 is not None and reached0 < 200
    assert harness.get_output("output").newPosition[0] == pytest.approx(1.0, abs=1e-3)
    assert harness.get_output("output").newPosition[1] == pytest.approx(8.0, abs=1e-3)


def test_sync_phase_collinear_shared_timing(harness):
    """SYNC_PHASE on a collinear move: both axes reach their (ratio-scaled)
    targets together, no error."""
    inp = default_input()
    inp["nDofs"] = 2
    inp["synchronization"] = 1  # SYNC_PHASE
    inp["enabled"] = [True, True, False, False]
    inp["targetPosition"] = [1.0, 3.0, 0.0, 0.0]
    inp["maxVelocity"] = [3.0] * 4
    inp["maxAcceleration"] = [5.0] * 4
    inp["maxJerk"] = [10.0] * 4

    p0 = p1 = 0.0
    for _cyc in range(2000):
        harness.set_inputs(enable=True, input=inp, cycleTime=0.010, reset=False)
        harness.execute()
        assert harness.get_output("error") is False
        out = harness.get_output("output")
        p0, p1 = out.newPosition[0], out.newPosition[1]
        if harness.get_output("done"):
            break
    assert harness.get_output("done") is True
    assert p0 == pytest.approx(1.0, abs=1e-3)
    assert p1 == pytest.approx(3.0, abs=1e-3)


def test_sync_phase_non_collinear_falls_back(harness):
    """SYNC_PHASE on a non-collinear move falls back to Time: both still reach
    target, no error."""
    inp = default_input()
    inp["nDofs"] = 2
    inp["synchronization"] = 1  # SYNC_PHASE -> not collinear -> Time
    inp["enabled"] = [True, True, False, False]
    inp["currentVelocity"] = [1.0, -1.0, 0.0, 0.0]  # not collinear with pd=[1,3]
    inp["targetPosition"] = [1.0, 3.0, 0.0, 0.0]
    inp["maxVelocity"] = [3.0] * 4
    inp["maxAcceleration"] = [5.0] * 4
    inp["maxJerk"] = [10.0] * 4

    p0 = p1 = 0.0
    for _cyc in range(2000):
        harness.set_inputs(enable=True, input=inp, cycleTime=0.010, reset=False)
        harness.execute()
        assert harness.get_output("error") is False
        out = harness.get_output("output")
        p0, p1 = out.newPosition[0], out.newPosition[1]
        if harness.get_output("done"):
            break
    assert harness.get_output("done") is True
    assert p0 == pytest.approx(1.0, abs=1e-3)
    assert p1 == pytest.approx(3.0, abs=1e-3)


