"""Unit tests for ComputeBlock1Axis (step1 -> reachable-duration Block)."""
import pytest

ruckig = pytest.importorskip("ruckig")

_VMAX, _AMAX, _JMAX = 2.0, 3.0, 10.0


@pytest.fixture
def harness(make_harness):
    return make_harness("ComputeBlock1Axis.s7dcl")


def _empty_profile():
    return {
        "t": [0.0] * 7, "j": [0.0] * 7,
        "a": [0.0] * 8, "v": [0.0] * 8, "p": [0.0] * 8,
        "direction": 1,
        "brake": {"t": [0.0, 0.0], "j": [0.0, 0.0],
                  "a": [0.0, 0.0, 0.0], "v": [0.0, 0.0, 0.0], "p": [0.0, 0.0, 0.0]},
        "controlSigns": 0,
    }


def _empty_block():
    return {"tMin": 0.0, "found": False,
            "aValid": False, "aLeft": 0.0, "aRight": 0.0,
            "bValid": False, "bLeft": 0.0, "bRight": 0.0}


def _t_min(p0, v0, a0, pT, vT, aT, vM=_VMAX, aM=_AMAX, jM=_JMAX):
    otg = ruckig.Ruckig(1); inp = ruckig.InputParameter(1); tr = ruckig.Trajectory(1)
    inp.current_position = [p0]; inp.current_velocity = [v0]; inp.current_acceleration = [a0]
    inp.target_position = [pT]; inp.target_velocity = [vT]; inp.target_acceleration = [aT]
    inp.max_velocity = [vM]; inp.max_acceleration = [aM]; inp.max_jerk = [jM]
    otg.calculate(inp, tr)
    return tr.duration


def _run(harness, p0, v0, a0, pT, vT, aT, vM=_VMAX, aM=_AMAX, jM=_JMAX):
    harness.reset()
    harness.set_inputs(block=_empty_block(), profile=_empty_profile(),
                       p0=p0, v0=v0, a0=a0, pT=pT, vT=vT, aT=aT,
                       vMax=vM, aMax=aM, jMax=jM)
    harness.execute()
    return harness.get_output("ComputeBlock1Axis"), harness.get_var("block")


# tMin must equal Ruckig's time-optimal duration.
_TMIN_CASES = [
    (0.0, 0.0, 0.0, 1.0, 0.0, 0.0),
    (0.0, 0.0, 0.0, 5.0, 0.0, 0.0),
    (0.0, 0.5, 0.0, 1.0, 0.5, 0.0),
    (0.0, -1.5, 2.0, -2.0, 0.0, 0.0),
    (0.0, 1.0, 0.0, 0.3, 1.0, 0.0),
]


@pytest.mark.parametrize("p0,v0,a0,pT,vT,aT", _TMIN_CASES)
def test_block_tmin_matches_oracle(harness, p0, v0, a0, pT, vT, aT):
    st, blk = _run(harness, p0, v0, a0, pT, vT, aT)
    assert st == 0x7000
    assert blk.found is True
    assert blk.tMin == pytest.approx(_t_min(p0, v0, a0, pT, vT, aT), abs=1e-6)


def test_block_has_interval_for_braking_case(harness):
    """The v0.3 'braking' case (v0 high vs small pd) has a blocked interval:
    a duration just above t_min is NOT reachable (Ruckig jumps to a much longer
    duration). The Block must flag it, and the blocked tf must lie inside the
    interval."""
    p0, v0, a0, pT, vT, aT = 0.0, 1.0, 0.0, 0.3, 1.0, 0.0
    tmin = _t_min(p0, v0, a0, pT, vT, aT)
    st, blk = _run(harness, p0, v0, a0, pT, vT, aT)
    assert st == 0x7000
    assert blk.tMin == pytest.approx(tmin, abs=1e-6)
    assert blk.aValid is True, "braking case should expose a blocked interval"
    tf_blocked = tmin * 1.1  # v0.3 showed this tf is not reachable
    assert blk.aLeft <= tf_blocked <= blk.aRight, "blocked tf inside the interval"


def test_block_simple_move_no_interval(harness):
    """A plain rest-to-rest move is reachable at every duration >= t_min: no
    blocked interval."""
    st, blk = _run(harness, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0)
    assert st == 0x7000
    assert blk.aValid is False and blk.bValid is False
