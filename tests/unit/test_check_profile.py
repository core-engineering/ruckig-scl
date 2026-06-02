import pytest


@pytest.fixture
def harness(make_harness):
    return make_harness("CheckProfile.s7dcl")


def _profile(t, j):
    return {
        "t": list(t), "j": list(j),
        "a": [0.0] * 8, "v": [0.0] * 8, "p": [0.0] * 8,
        "direction": 1,
        "brake": {"t": [0.0, 0.0], "j": [0.0, 0.0],
                  "a": [0.0, 0.0, 0.0], "v": [0.0, 0.0, 0.0], "p": [0.0, 0.0, 0.0]},
        "controlSigns": 0,
    }


def _ref(t, j, p0=0.0, v0=0.0, a0=0.0):
    """Integrate to get final state and peak |v|,|a| (the definition)."""
    a = [a0]; v = [v0]; p = [p0]
    for i in range(7):
        dt = t[i]
        a.append(a[i] + j[i] * dt)
        v.append(v[i] + a[i] * dt + 0.5 * j[i] * dt * dt)
        p.append(p[i] + v[i] * dt + 0.5 * a[i] * dt * dt + j[i] * dt * dt * dt / 6.0)
    return p[7], v[7], a[7], max(abs(x) for x in v), max(abs(x) for x in a)


def _check(harness, prof, p0, v0, a0, pT, vT, aT, vMax, aMax):
    harness.reset()
    harness.set_inputs(profile=prof, p0=p0, v0=v0, a0=a0,
                       pT=pT, vT=vT, aT=aT, vMax=vMax, aMax=aMax)
    harness.execute()
    return harness.get_output("CheckProfile")


# A clean rest-to-rest trapezoidal profile (vmax=2, amax=2, jmax=10)
T = [0.2, 0.8, 0.2, 3.8, 0.2, 0.8, 0.2]
J = [10.0, 0.0, -10.0, 0.0, -10.0, 0.0, 10.0]


def test_valid_profile_true(harness):
    p7, v7, a7, mv, ma = _ref(T, J)
    assert _check(harness, _profile(T, J), 0.0, 0.0, 0.0,
                  p7, v7, a7, mv + 0.1, ma + 0.1) is True


def test_negative_time_false(harness):
    t = list(T); t[3] = -0.1  # illegal negative phase time
    p7, v7, a7, mv, ma = _ref(T, J)  # target from the legal profile
    assert _check(harness, _profile(t, J), 0.0, 0.0, 0.0,
                  p7, v7, a7, mv + 0.1, ma + 0.1) is False


def test_exceeds_vmax_false(harness):
    p7, v7, a7, mv, ma = _ref(T, J)
    # vMax set well below the actual peak velocity
    assert _check(harness, _profile(T, J), 0.0, 0.0, 0.0,
                  p7, v7, a7, mv - 0.5, ma + 0.1) is False


def test_exceeds_amax_false(harness):
    p7, v7, a7, mv, ma = _ref(T, J)
    assert _check(harness, _profile(T, J), 0.0, 0.0, 0.0,
                  p7, v7, a7, mv + 0.1, ma - 0.5) is False


def test_wrong_target_false(harness):
    p7, v7, a7, mv, ma = _ref(T, J)
    assert _check(harness, _profile(T, J), 0.0, 0.0, 0.0,
                  p7 + 1.0, v7, a7, mv + 0.1, ma + 0.1) is False


# ---------------------------------------------------------------------------
# Internal velocity extremum (Ruckig profile.hpp check<>, lines 249-254).
# When acceleration crosses zero INSIDE a phase (i>=2), velocity has an interior
# extremum v_a_zero = v[i] - a[i]^2/(2*j[i]) that the node samples miss. The
# solver must reject a profile whose interior velocity peak exceeds vMax, even
# though every node sample stays within limits — otherwise it can emit (and pick
# as "shorter") a trajectory that overshoots vMax between nodes. v0.2 bug fix.
# ---------------------------------------------------------------------------

# rest start; phase 0 ramps a up to +10, phase 2 ramps it down through zero to
# -10 -> velocity peaks at 1.0 mid-phase-2 while every node stays at v=0.5.
T_PEAK = [0.1, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0]
J_PEAK = [100.0, 0.0, -100.0, 0.0, 0.0, 0.0, 0.0]


def _internal_peak(t, j, p0=0.0, v0=0.0, a0=0.0):
    """Max |v| including interior a-crossing extrema (the true definition)."""
    a = [a0]; v = [v0]
    peak = abs(v0)
    for i in range(7):
        dt = t[i]
        a1 = a[i] + j[i] * dt
        if a[i] * a1 < 0.0 and j[i] != 0.0:  # a crosses zero inside the segment
            v_az = v[i] - (a[i] * a[i]) / (2.0 * j[i])
            peak = max(peak, abs(v_az))
        v.append(v[i] + a[i] * dt + 0.5 * j[i] * dt * dt)
        a.append(a1)
        peak = max(peak, abs(v[i + 1]))
    return peak


def test_internal_velocity_peak_false(harness):
    """Interior velocity peak (1.0) exceeds vMax (0.8) although all node samples
    sit at v=0.5 < 0.8. Must be rejected (was wrongly accepted pre-fix)."""
    p7, v7, a7, mv_nodes, ma = _ref(T_PEAK, J_PEAK)
    peak = _internal_peak(T_PEAK, J_PEAK)
    assert peak == pytest.approx(1.0, abs=1e-9)
    assert mv_nodes == pytest.approx(0.5, abs=1e-9)
    # vMax between the node max (0.5) and the interior peak (1.0)
    assert _check(harness, _profile(T_PEAK, J_PEAK), 0.0, 0.0, 0.0,
                  p7, v7, a7, 0.8, ma + 0.1) is False


def test_internal_velocity_peak_within_limits_true(harness):
    """Same profile, vMax above the interior peak -> accepted."""
    p7, v7, a7, mv_nodes, ma = _ref(T_PEAK, J_PEAK)
    assert _check(harness, _profile(T_PEAK, J_PEAK), 0.0, 0.0, 0.0,
                  p7, v7, a7, 1.1, ma + 0.1) is True
