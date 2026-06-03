import pytest


@pytest.fixture
def harness(make_harness):
    return make_harness("IntegrateProfileStates.s7dcl")


def _empty_profile(t, j):
    return {
        "t": list(t), "j": list(j),
        "a": [0.0] * 8, "v": [0.0] * 8, "p": [0.0] * 8,
        "direction": 1,
        "brake": {"t": [0.0, 0.0], "j": [0.0, 0.0],
                  "a": [0.0, 0.0, 0.0], "v": [0.0, 0.0, 0.0], "p": [0.0, 0.0, 0.0]},
        "controlSigns": 0,
    }


def _expected(t, j, p0, v0, a0):
    """Reference cubic integration (the definition)."""
    a = [a0]; v = [v0]; p = [p0]
    for i in range(7):
        dt = t[i]
        a.append(a[i] + j[i] * dt)
        v.append(v[i] + a[i] * dt + 0.5 * j[i] * dt * dt)
        p.append(p[i] + v[i] * dt + 0.5 * a[i] * dt * dt + j[i] * dt * dt * dt / 6.0)
    return a, v, p


def _run(harness, t, j, p0, v0, a0):
    harness.reset()
    harness.set_inputs(profile=_empty_profile(t, j), p0=p0, v0=v0, a0=a0)
    harness.execute()
    prof = harness.get_var("profile")
    a = [prof.a[i] for i in range(8)]
    v = [prof.v[i] for i in range(8)]
    p = [prof.p[i] for i in range(8)]
    return a, v, p


def test_rest_to_rest_matches_definition(harness):
    # symmetric trapezoidal-ish 7-phase, rest to rest
    t = [0.2, 0.8, 0.2, 3.8, 0.2, 0.8, 0.2]
    j = [10.0, 0.0, -10.0, 0.0, -10.0, 0.0, 10.0]
    a, v, p = _run(harness, t, j, 0.0, 0.0, 0.0)
    ea, ev, ep = _expected(t, j, 0.0, 0.0, 0.0)
    assert a == pytest.approx(ea, abs=1e-9)
    assert v == pytest.approx(ev, abs=1e-9)
    assert p == pytest.approx(ep, abs=1e-9)


def test_nonzero_initial_state(harness):
    # arbitrary initial state (v0,a0 != 0)
    t = [0.1, 0.3, 0.1, 0.5, 0.1, 0.3, 0.1]
    j = [5.0, 0.0, -5.0, 0.0, -5.0, 0.0, 5.0]
    a, v, p = _run(harness, t, j, 2.0, 0.7, -0.3)
    ea, ev, ep = _expected(t, j, 2.0, 0.7, -0.3)
    assert a == pytest.approx(ea, abs=1e-9)
    assert v == pytest.approx(ev, abs=1e-9)
    assert p == pytest.approx(ep, abs=1e-9)


def test_udud_pattern_matches_definition(harness):
    """v0.3 step2 introduces the UDUD jerk pattern [+j,0,-j,0,+j,0,-j]
    (vs UDDU [+j,0,-j,0,-j,0,+j]). IntegrateProfileStates is pattern-agnostic
    (it integrates from j[]), so it must reproduce UDUD node states exactly."""
    t = [0.1, 0.05, 0.2, 0.1, 0.2, 0.05, 0.1]
    j = [10.0, 0.0, -10.0, 0.0, 10.0, 0.0, -10.0]  # UDUD: phase4 is +j, not -j
    a, v, p = _run(harness, t, j, 0.0, 0.0, 0.0)
    ea, ev, ep = _expected(t, j, 0.0, 0.0, 0.0)
    assert a == pytest.approx(ea, abs=1e-9)
    assert v == pytest.approx(ev, abs=1e-9)
    assert p == pytest.approx(ep, abs=1e-9)


def test_zero_durations_hold_state(harness):
    # all t = 0 -> every sample equals the initial state
    t = [0.0] * 7
    j = [0.0] * 7
    a, v, p = _run(harness, t, j, 1.5, 0.4, -0.2)
    assert a == pytest.approx([-0.2] * 8, abs=1e-12)
    assert v == pytest.approx([0.4] * 8, abs=1e-12)
    assert p == pytest.approx([1.5] * 8, abs=1e-12)
