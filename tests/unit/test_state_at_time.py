"""Unit tests for StateAtTime FC."""
import pytest


@pytest.fixture
def harness(make_harness):
    return make_harness("StateAtTime.s7dcl")


def trapezoidal_profile(p0=0.0, p1=10.0, vmax=2.0, amax=2.0, jmax=10.0) -> dict:
    """Pre-computed 7-phase trapezoidal profile (case A) — the v0.1 rest-to-rest
    trapezoid the solver reproduces."""
    tj, ta = 0.2, 0.8
    s_to_vmax = vmax * (2.0 * tj + ta)
    tv = (p1 - p0 - s_to_vmax) / vmax
    t = [tj, ta, tj, tv, tj, ta, tj]
    j = [+jmax, 0.0, -jmax, 0.0, -jmax, 0.0, +jmax]
    a = [0.0]
    v = [0.0]
    p = [p0]
    for i in range(7):
        a.append(a[-1] + j[i] * t[i])
        v.append(v[-1] + a[-2] * t[i] + 0.5 * j[i] * t[i] ** 2)
        p.append(p[-1] + v[-2] * t[i] + 0.5 * a[-2] * t[i] ** 2 + j[i] * t[i] ** 3 / 6.0)
    return {
        "t": t, "j": j, "a": a, "v": v, "p": p,
        "direction": 1,
        "brake": {"t": [0.0, 0.0], "j": [0.0, 0.0],
                  "a": [0.0, 0.0, 0.0], "v": [0.0, 0.0, 0.0], "p": [0.0, 0.0, 0.0]},
        "controlSigns": 0,
    }


def _eval(harness, profile: dict, t: float):
    harness.reset()
    harness.set_inputs(profile=profile, t=t)
    harness.execute()
    return (
        harness.get_output("p"),
        harness.get_output("v"),
        harness.get_output("a"),
    )


def test_state_at_t_zero(harness):
    p, v, a = _eval(harness, trapezoidal_profile(), 0.0)
    assert p == pytest.approx(0.0, abs=1e-9)
    assert v == pytest.approx(0.0, abs=1e-9)
    assert a == pytest.approx(0.0, abs=1e-9)


def test_state_at_t_end(harness):
    p, v, a = _eval(harness, trapezoidal_profile(), 6.2)  # total duration
    assert p == pytest.approx(10.0, abs=1e-6)
    assert v == pytest.approx(0.0, abs=1e-6)
    assert a == pytest.approx(0.0, abs=1e-6)


def test_state_at_end_of_phase_zero(harness):
    """End of jerk-up (t=0.2): a=a_max, v=0.5*j*t² = 0.2, p = j*t³/6 = 0.0133."""
    p, v, a = _eval(harness, trapezoidal_profile(), 0.2)
    assert a == pytest.approx(2.0, abs=1e-6)
    assert v == pytest.approx(0.2, abs=1e-6)
    assert p == pytest.approx(10.0 * 0.008 / 6.0, abs=1e-6)


def test_state_in_middle_of_const_vel(harness):
    """Middle of constant-velocity phase (phase 3). At t = 0.2 + 0.8 + 0.2 + 1.9
    we are halfway through the const-vel phase (which lasts 3.8s)."""
    profile = trapezoidal_profile()
    t_mid_const_vel = 0.2 + 0.8 + 0.2 + 1.9  # halfway into the v=v_max plateau
    p, v, a = _eval(harness, profile, t_mid_const_vel)
    assert v == pytest.approx(2.0, abs=1e-6)
    assert a == pytest.approx(0.0, abs=1e-6)


def test_state_beyond_trajectory_clamps(harness):
    """t > duration → returns final state."""
    profile = trapezoidal_profile()
    p, v, a = _eval(harness, profile, 100.0)
    assert p == pytest.approx(10.0, abs=1e-6)
    assert v == pytest.approx(0.0, abs=1e-6)
    assert a == pytest.approx(0.0, abs=1e-6)


# ---------------------------------------------------------------------------
# Task 8: brake-prefix awareness
# ---------------------------------------------------------------------------

def _integrate_seg(p0, v0, a0, j, dt):
    """Cubic segment integration, identical idiom to StateAtTime."""
    a1 = a0 + j * dt
    v1 = v0 + a0 * dt + 0.5 * j * dt ** 2
    p1 = p0 + v0 * dt + 0.5 * a0 * dt ** 2 + j * dt ** 3 / 6.0
    return p1, v1, a1


def _profile_with_brake(v0=3.0, a0=0.0, bt0=0.3, bj0=-10.0, bt1=0.2, bj1=0.0):
    """Build a profile with a 2-segment brake prefix.

    The brake is seeded relative (brake.p[0]=0, brake.v[0]=v0, brake.a[0]=a0)
    and integrated forward through its two segments to fill brake.a/v/p[1..2].
    The MAIN profile start state is then seeded from the brake END state for
    continuity (a[0]/v[0] = brake end; p[0] = brake.p[2] so positions are
    continuous across the boundary — see StateAtTime continuity assumption).
    The main profile here is a simple 1-segment ramp-down to make the post-brake
    state easy to hand-check; remaining phases are zero-length holds.
    """
    # --- brake integration (relative position seed) ---
    ba = [a0, 0.0, 0.0]
    bv = [v0, 0.0, 0.0]
    bp = [0.0, 0.0, 0.0]
    bj = [bj0, bj1]
    bt = [bt0, bt1]
    bp[1], bv[1], ba[1] = _integrate_seg(bp[0], bv[0], ba[0], bj[0], bt[0])
    bp[2], bv[2], ba[2] = _integrate_seg(bp[1], bv[1], ba[1], bj[1], bt[1])

    # --- main profile: phase 0 = const-jerk for 0.5s, rest = zero holds ---
    mt = [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    mj = [-5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ma = [ba[2]]
    mv = [bv[2]]
    mp = [bp[2]]  # position continuity: main start = brake end
    for i in range(7):
        p1, v1, a1 = _integrate_seg(mp[-1], mv[-1], ma[-1], mj[i], mt[i])
        ma.append(a1)
        mv.append(v1)
        mp.append(p1)

    return {
        "t": mt, "j": mj, "a": ma, "v": mv, "p": mp,
        "direction": 1,
        "brake": {"t": bt, "j": bj, "a": ba, "v": bv, "p": bp},
        "controlSigns": 0,
    }, (bt, bj, ba, bv, bp), (mt, mj, ma, mv, mp)


def test_brake_state_inside_segment_zero(harness):
    profile, (bt, bj, ba, bv, bp), _ = _profile_with_brake()
    t = 0.1  # inside brake segment 0 (lasts bt[0]=0.3)
    ep, ev, ea = _integrate_seg(bp[0], bv[0], ba[0], bj[0], t)
    p, v, a = _eval(harness, profile, t)
    assert a == pytest.approx(ea, abs=1e-9)
    assert v == pytest.approx(ev, abs=1e-9)
    assert p == pytest.approx(ep, abs=1e-9)


def test_brake_state_inside_segment_one(harness):
    profile, (bt, bj, ba, bv, bp), _ = _profile_with_brake()
    t = bt[0] + 0.1  # 0.1s into brake segment 1
    ep, ev, ea = _integrate_seg(bp[1], bv[1], ba[1], bj[1], 0.1)
    p, v, a = _eval(harness, profile, t)
    assert a == pytest.approx(ea, abs=1e-9)
    assert v == pytest.approx(ev, abs=1e-9)
    assert p == pytest.approx(ep, abs=1e-9)


def test_brake_boundary_continuity(harness):
    """t == brakeDur exactly must equal the MAIN profile start state."""
    profile, (bt, bj, ba, bv, bp), (mt, mj, ma, mv, mp) = _profile_with_brake()
    brake_dur = bt[0] + bt[1]
    p, v, a = _eval(harness, profile, brake_dur)
    # main start state
    assert a == pytest.approx(ma[0], abs=1e-9)
    assert v == pytest.approx(mv[0], abs=1e-9)
    assert p == pytest.approx(mp[0], abs=1e-9)
    # and brake end state == main start (consistent seeding)
    assert ba[2] == pytest.approx(ma[0], abs=1e-12)
    assert bv[2] == pytest.approx(mv[0], abs=1e-12)
    assert bp[2] == pytest.approx(mp[0], abs=1e-12)


def test_state_just_after_brake_uses_main(harness):
    """t slightly past brakeDur → main profile phase-0 integrated with small tMain."""
    profile, (bt, bj, ba, bv, bp), (mt, mj, ma, mv, mp) = _profile_with_brake()
    brake_dur = bt[0] + bt[1]
    t_main = 0.05
    ep, ev, ea = _integrate_seg(mp[0], mv[0], ma[0], mj[0], t_main)
    p, v, a = _eval(harness, profile, brake_dur + t_main)
    assert a == pytest.approx(ea, abs=1e-9)
    assert v == pytest.approx(ev, abs=1e-9)
    assert p == pytest.approx(ep, abs=1e-9)


def test_state_beyond_brake_plus_main_clamps(harness):
    """t past brake+main duration → holds final main state (p/v/a[7])."""
    profile, (bt, bj, ba, bv, bp), (mt, mj, ma, mv, mp) = _profile_with_brake()
    p, v, a = _eval(harness, profile, 1000.0)
    assert p == pytest.approx(mp[7], abs=1e-9)
    assert v == pytest.approx(mv[7], abs=1e-9)
    assert a == pytest.approx(ma[7], abs=1e-9)


def test_state_t_zero_with_brake_returns_brake_start(harness):
    profile, (bt, bj, ba, bv, bp), _ = _profile_with_brake()
    p, v, a = _eval(harness, profile, 0.0)
    assert p == pytest.approx(bp[0], abs=1e-9)
    assert v == pytest.approx(bv[0], abs=1e-9)
    assert a == pytest.approx(ba[0], abs=1e-9)


def test_state_t_negative_with_brake_returns_brake_start(harness):
    profile, (bt, bj, ba, bv, bp), _ = _profile_with_brake()
    p, v, a = _eval(harness, profile, -5.0)
    assert p == pytest.approx(bp[0], abs=1e-9)
    assert v == pytest.approx(bv[0], abs=1e-9)
    assert a == pytest.approx(ba[0], abs=1e-9)


def test_no_brake_regression(harness):
    """No-brake profile must behave EXACTLY as before (mirror of phase-0 case)."""
    p, v, a = _eval(harness, trapezoidal_profile(), 0.2)
    assert a == pytest.approx(2.0, abs=1e-6)
    assert v == pytest.approx(0.2, abs=1e-6)
    assert p == pytest.approx(10.0 * 0.008 / 6.0, abs=1e-6)
