"""Unit tests for PhaseSynchronize (multi-DoF phase synchronization)."""
import pytest

ruckig = pytest.importorskip("ruckig")


@pytest.fixture
def harness(make_harness):
    return make_harness("PhaseSynchronize.s7dcl")


def _empty_profile():
    return {
        "t": [0.0] * 7, "j": [0.0] * 7,
        "a": [0.0] * 8, "v": [0.0] * 8, "p": [0.0] * 8,
        "direction": 1,
        "brake": {"t": [0.0, 0.0], "j": [0.0, 0.0],
                  "a": [0.0, 0.0, 0.0], "v": [0.0, 0.0, 0.0], "p": [0.0, 0.0, 0.0]},
        "controlSigns": 0,
    }


def _trajectory():
    return {"profiles": [_empty_profile() for _ in range(4)],
            "duration": 0.0, "independentMinDurations": [0.0] * 4,
            "currentTime": 0.0, "isValid": False}


def _input(n, cp, cv, ca, tp, tv, ta, vM, aM, jM):
    pad = lambda x: list(x) + [0.0] * (4 - len(x))
    return {
        "currentPosition": pad(cp), "currentVelocity": pad(cv), "currentAcceleration": pad(ca),
        "targetPosition": pad(tp), "targetVelocity": pad(tv), "targetAcceleration": pad(ta),
        "maxVelocity": pad(vM), "maxAcceleration": pad(aM), "maxJerk": pad(jM),
        "enabled": [d < n for d in range(4)], "nDofs": n,
        "minimumDuration": -1.0, "synchronization": 1,
        "controlInterface": 0, "durationDiscretization": 0,
    }


def _run(harness, n, cp, cv, ca, tp, tv, ta, vM, aM, jM):
    pad = lambda x: list(x) + [0.0] * (4 - len(x))
    harness.reset()
    harness.set_inputs(
        trajectory=_trajectory(),
        chainPos=pad(cp), chainVel=pad(cv), chainAcc=pad(ca),
        input=_input(n, cp, cv, ca, tp, tv, ta, vM, aM, jM), nDofs=n)
    harness.execute()
    return (harness.get_output("phaseOk"),
            harness.get_output("duration"),
            harness.get_var("trajectory"))


def _profile_t(traj, d):
    pr = traj.profiles[d]
    return [pr.t[i] for i in range(7)]


def test_collinear_rest_to_rest_phase_aligned(harness):
    ok, dur, traj = _run(harness, 2, [0, 0], [0, 0], [0, 0],
                         [1.0, 3.0], [0, 0], [0, 0], [3, 3], [5, 5], [10, 10])
    assert ok is True
    assert _profile_t(traj, 0) == pytest.approx(_profile_t(traj, 1), abs=1e-9)
    assert traj.profiles[1].j[0] == pytest.approx(3.0 * traj.profiles[0].j[0], abs=1e-6)


def test_collinear_moving_target(harness):
    ok, dur, traj = _run(harness, 2, [0, 0], [0.4, 1.2], [0, 0],
                         [1.0, 3.0], [0.2, 0.6], [0, 0], [3, 3], [5, 5], [10, 10])
    assert ok is True
    assert _profile_t(traj, 0) == pytest.approx(_profile_t(traj, 1), abs=1e-9)


def test_non_collinear_returns_false(harness):
    ok, dur, traj = _run(harness, 2, [0, 0], [1.0, -1.0], [0, 0],
                         [1.0, 3.0], [0, 0], [0, 0], [3, 3], [5, 5], [10, 10])
    assert ok is False


def test_collinear_reaches_each_target(harness):
    ok, dur, traj = _run(harness, 2, [0, 0], [0, 0], [0, 0],
                         [1.0, 3.0], [0, 0], [0, 0], [3, 3], [5, 5], [10, 10])
    assert ok is True
    assert traj.profiles[0].p[7] == pytest.approx(1.0, abs=1e-6)
    assert traj.profiles[1].p[7] == pytest.approx(3.0, abs=1e-6)


def test_collinear_limit_exceeded_returns_false(harness):
    # Collinear rest-to-rest (v0=a0=vT=aT=0): displacements [1.0, 5.0] are
    # collinear by construction (ratio 5x, only one ratio to check).
    # axis0 has tight aM/jM (0.02 / 0.1) so its independent profile is
    # acceleration-limited and SLOWER than axis1 → axis0 is chosen as reference.
    # kd = pd[1]/pd[0] = 5; scaled axis1 peak velocity = 5 × v0_ref ≈ 5 × 0.14 = 0.70,
    # which exceeds axis1's vMax=0.5 → CheckProfile fails → phaseOk must be False.
    # Ruckig also falls back: under Synchronization.Phase the two axis profiles
    # have *different* time arrays (confirmed with the oracle).
    ok, dur, traj = _run(harness, 2, [0, 0], [0, 0], [0, 0],
                         [1.0, 5.0], [0, 0], [0, 0], [1.0, 0.5], [0.02, 5.0], [0.1, 5.0])
    assert ok is False
