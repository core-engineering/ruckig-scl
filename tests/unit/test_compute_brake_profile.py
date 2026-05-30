"""Unit tests for ComputeBrakeProfile FC (v0.2 Task 7).

Ports Ruckig's third-order POSITION brake (brake.cpp:
get_position_brake_trajectory + acceleration_brake + velocity_brake).
When the initial state (v0, a0) is OUT of limits (|a0|>aMax or |v0|>vMax),
Ruckig prepends a <=2 segment brake trajectory that drives the state back
inside the limits before the main profile runs. Limits are symmetric in
v0.2: vMin = -vMax, aMin = -aMax.

ORACLE: the official `ruckig` PyPI package DOES expose the brake. After
`otg.calculate(inp, tr)`, `tr.profiles[0][0].brake` is a BrakeProfile with
attributes t (len 2), j (len 2), a/v/p (per-segment START state) and
duration. We harvest those directly and assert the SCL FC reproduces:
  * brake.t[0..1], brake.j[0..1]  to ~1e-9
  * the per-segment START states brake.a/v[k] (oracle) to ~1e-7
  * the POST-brake state (state after BOTH segments integrated) is within
    limits (|aBrake|<=aMax+EPS, |vBrake|<=vMax+EPS).

SEEDING CHOICE (documented in the FC header too): the brake is RELATIVE in
position. We seed brake.p[0] := 0.0, brake.v[0] := v0, brake.a[0] := a0,
then integrate BOTH segments via the same cubic recurrence as
IntegrateProfileStates. A zero-duration segment leaves the state unchanged,
so we can always integrate two segments and read index 2. Outputs:
  pBrake = brake.p[2]  (position ADVANCE during braking, relative; 0 if no brake)
  vBrake = brake.v[2]  (absolute post-brake velocity; == v0 if no brake)
  aBrake = brake.a[2]  (absolute post-brake acceleration; == a0 if no brake)
The oracle confirms brake.p[0] == 0.0 (relative), validating this choice.
"""
import pytest

ruckig = pytest.importorskip("ruckig")

EPS = 1e-9


@pytest.fixture
def harness(make_harness):
    return make_harness("ComputeBrakeProfile.s7dcl")


def _empty_profile():
    return {
        "t": [0.0] * 7, "j": [0.0] * 7,
        "a": [0.0] * 8, "v": [0.0] * 8, "p": [0.0] * 8,
        "direction": 1,
        "brake": {"t": [0.0, 0.0], "j": [0.0, 0.0],
                  "a": [0.0, 0.0, 0.0], "v": [0.0, 0.0, 0.0], "p": [0.0, 0.0, 0.0]},
        "controlSigns": 0,
    }


def _oracle_brake(v0, a0, vMax, aMax, jMax):
    """Return Ruckig's internal BrakeProfile for the given out-of-limits state.

    We drive the OTG with an arbitrary in-limits target; Ruckig prepends the
    brake based purely on (current state, limits), independent of the target.
    """
    otg = ruckig.Ruckig(1)
    inp = ruckig.InputParameter(1)
    tr = ruckig.Trajectory(1)
    inp.current_position = [0.0]
    inp.current_velocity = [v0]
    inp.current_acceleration = [a0]
    inp.target_position = [0.0]
    inp.target_velocity = [0.0]
    inp.target_acceleration = [0.0]
    inp.max_velocity = [vMax]
    inp.max_acceleration = [aMax]
    inp.max_jerk = [jMax]
    otg.calculate(inp, tr)
    return tr.profiles[0][0].brake


def _integrate_two_segments(v0, a0, t, j):
    """Same cubic recurrence as IntegrateProfileStates, over 2 brake segments.

    Seeds p0=0 (relative), v=v0, a=a0; returns (a2, v2, p2) post-brake state.
    """
    a, v, p = a0, v0, 0.0
    for k in range(2):
        dt, jk = t[k], j[k]
        p = p + v * dt + 0.5 * a * dt * dt + jk * dt * dt * dt / 6.0
        v = v + a * dt + 0.5 * jk * dt * dt
        a = a + jk * dt
    return a, v, p


def _run(harness, v0, a0, vMax, aMax, jMax):
    harness.reset()
    harness.set_inputs(profile=_empty_profile(),
                       v0=v0, a0=a0, vMax=vMax, aMax=aMax, jMax=jMax)
    harness.execute()
    dur = harness.get_output("ComputeBrakeProfile")
    prof = harness.get_var("profile")
    out = {
        "dur": dur,
        "pBrake": harness.get_output("pBrake"),
        "vBrake": harness.get_output("vBrake"),
        "aBrake": harness.get_output("aBrake"),
        "brake": prof.brake,
    }
    return out


# ---------------------------------------------------------------------------
# In-limits input -> no brake, duration 0, state unchanged
# ---------------------------------------------------------------------------

def test_in_limits_no_brake(harness):
    v0, a0, vMax, aMax, jMax = 0.5, 0.5, 1.0, 2.0, 10.0
    o = _run(harness, v0, a0, vMax, aMax, jMax)
    assert o["dur"] == pytest.approx(0.0, abs=EPS)
    b = o["brake"]
    assert b.t[0] == pytest.approx(0.0, abs=EPS)
    assert b.t[1] == pytest.approx(0.0, abs=EPS)
    # post-brake state == input state (p advance 0)
    assert o["aBrake"] == pytest.approx(a0, abs=1e-9)
    assert o["vBrake"] == pytest.approx(v0, abs=1e-9)
    assert o["pBrake"] == pytest.approx(0.0, abs=1e-9)


# ---------------------------------------------------------------------------
# Out-of-limits acceleration: a0 > aMax  -> acceleration_brake
# ---------------------------------------------------------------------------

def test_a0_above_amax(harness):
    v0, a0, vMax, aMax, jMax = 0.0, 5.0, 1.0, 2.0, 10.0
    o = _run(harness, v0, a0, vMax, aMax, jMax)
    b = o["brake"]
    ob = _oracle_brake(v0, a0, vMax, aMax, jMax)
    assert o["dur"] > 0.0
    assert o["dur"] == pytest.approx(ob.duration, abs=1e-9)
    assert b.t[0] == pytest.approx(ob.t[0], abs=1e-9)
    assert b.t[1] == pytest.approx(ob.t[1], abs=1e-9)
    assert b.j[0] == pytest.approx(ob.j[0], abs=1e-9)
    assert b.j[1] == pytest.approx(ob.j[1], abs=1e-9)
    # per-segment START states vs oracle
    for k in range(len(ob.a)):
        assert b.a[k] == pytest.approx(ob.a[k], abs=1e-7)
        assert b.v[k] == pytest.approx(ob.v[k], abs=1e-7)
    # post-brake state matches independent python integration
    ea, ev, ep = _integrate_two_segments(v0, a0, list(b.t), list(b.j))
    assert o["aBrake"] == pytest.approx(ea, abs=1e-9)
    assert o["vBrake"] == pytest.approx(ev, abs=1e-9)
    assert o["pBrake"] == pytest.approx(ep, abs=1e-9)
    # within limits after brake
    assert abs(o["aBrake"]) <= aMax + 1e-7
    assert abs(o["vBrake"]) <= vMax + 1e-7


def test_a0_above_amax_two_segments(harness):
    # a0>aMax AND the constant-accel hold segment is non-zero (t[1]>0)
    v0, a0, vMax, aMax, jMax = 3.0, 6.0, 1.0, 2.0, 10.0
    o = _run(harness, v0, a0, vMax, aMax, jMax)
    b = o["brake"]
    ob = _oracle_brake(v0, a0, vMax, aMax, jMax)
    assert b.t[0] == pytest.approx(ob.t[0], abs=1e-9)
    assert b.t[1] == pytest.approx(ob.t[1], abs=1e-9)
    assert b.t[1] > 0.0
    assert b.j[0] == pytest.approx(ob.j[0], abs=1e-9)
    ea, ev, ep = _integrate_two_segments(v0, a0, list(b.t), list(b.j))
    assert o["aBrake"] == pytest.approx(ea, abs=1e-9)
    assert o["vBrake"] == pytest.approx(ev, abs=1e-9)
    assert o["pBrake"] == pytest.approx(ep, abs=1e-9)
    assert abs(o["aBrake"]) <= aMax + 1e-7
    assert abs(o["vBrake"]) <= vMax + 1e-7


# ---------------------------------------------------------------------------
# Out-of-limits acceleration: a0 < aMin  -> acceleration_brake (mirrored args)
# ---------------------------------------------------------------------------

def test_a0_below_amin(harness):
    v0, a0, vMax, aMax, jMax = 0.0, -5.0, 1.0, 2.0, 10.0
    o = _run(harness, v0, a0, vMax, aMax, jMax)
    b = o["brake"]
    ob = _oracle_brake(v0, a0, vMax, aMax, jMax)
    assert o["dur"] > 0.0
    assert o["dur"] == pytest.approx(ob.duration, abs=1e-9)
    assert b.t[0] == pytest.approx(ob.t[0], abs=1e-9)
    assert b.t[1] == pytest.approx(ob.t[1], abs=1e-9)
    assert b.j[0] == pytest.approx(ob.j[0], abs=1e-9)  # +jMax (mirrored)
    for k in range(len(ob.a)):
        assert b.a[k] == pytest.approx(ob.a[k], abs=1e-7)
        assert b.v[k] == pytest.approx(ob.v[k], abs=1e-7)
    ea, ev, ep = _integrate_two_segments(v0, a0, list(b.t), list(b.j))
    assert o["aBrake"] == pytest.approx(ea, abs=1e-9)
    assert o["vBrake"] == pytest.approx(ev, abs=1e-9)
    assert o["pBrake"] == pytest.approx(ep, abs=1e-9)
    assert abs(o["aBrake"]) <= aMax + 1e-7
    assert abs(o["vBrake"]) <= vMax + 1e-7


# ---------------------------------------------------------------------------
# Out-of-limits velocity: v0 > vMax  -> velocity_brake
# ---------------------------------------------------------------------------

def test_v0_above_vmax(harness):
    v0, a0, vMax, aMax, jMax = 5.0, 0.0, 1.0, 2.0, 10.0
    o = _run(harness, v0, a0, vMax, aMax, jMax)
    b = o["brake"]
    ob = _oracle_brake(v0, a0, vMax, aMax, jMax)
    assert o["dur"] > 0.0
    assert o["dur"] == pytest.approx(ob.duration, abs=1e-9)
    assert b.t[0] == pytest.approx(ob.t[0], abs=1e-9)
    assert b.t[1] == pytest.approx(ob.t[1], abs=1e-9)
    assert b.j[0] == pytest.approx(ob.j[0], abs=1e-9)  # -jMax
    for k in range(len(ob.a)):
        assert b.a[k] == pytest.approx(ob.a[k], abs=1e-7)
        assert b.v[k] == pytest.approx(ob.v[k], abs=1e-7)
    ea, ev, ep = _integrate_two_segments(v0, a0, list(b.t), list(b.j))
    assert o["aBrake"] == pytest.approx(ea, abs=1e-9)
    assert o["vBrake"] == pytest.approx(ev, abs=1e-9)
    assert o["pBrake"] == pytest.approx(ep, abs=1e-9)
    assert abs(o["aBrake"]) <= aMax + 1e-7
    assert o["vBrake"] <= vMax + 1e-7  # braked down toward vMax


# ---------------------------------------------------------------------------
# Out-of-limits velocity: v0 < vMin  -> velocity_brake (mirrored args)
# ---------------------------------------------------------------------------

def test_v0_below_vmin(harness):
    v0, a0, vMax, aMax, jMax = -5.0, 0.0, 1.0, 2.0, 10.0
    o = _run(harness, v0, a0, vMax, aMax, jMax)
    b = o["brake"]
    ob = _oracle_brake(v0, a0, vMax, aMax, jMax)
    assert o["dur"] > 0.0
    assert o["dur"] == pytest.approx(ob.duration, abs=1e-9)
    assert b.t[0] == pytest.approx(ob.t[0], abs=1e-9)
    assert b.t[1] == pytest.approx(ob.t[1], abs=1e-9)
    assert b.j[0] == pytest.approx(ob.j[0], abs=1e-9)  # +jMax (mirrored)
    for k in range(len(ob.a)):
        assert b.a[k] == pytest.approx(ob.a[k], abs=1e-7)
        assert b.v[k] == pytest.approx(ob.v[k], abs=1e-7)
    ea, ev, ep = _integrate_two_segments(v0, a0, list(b.t), list(b.j))
    assert o["aBrake"] == pytest.approx(ea, abs=1e-9)
    assert o["vBrake"] == pytest.approx(ev, abs=1e-9)
    assert o["pBrake"] == pytest.approx(ep, abs=1e-9)
    assert abs(o["aBrake"]) <= aMax + 1e-7
    assert o["vBrake"] >= -vMax - 1e-7  # braked up toward vMin


# ---------------------------------------------------------------------------
# Zero-limits guard: jMax==0 (or aMax==0) -> no brake, duration 0
# ---------------------------------------------------------------------------

def test_zero_jmax_no_brake(harness):
    # jMax == 0 -> guard returns immediately, no brake
    o = _run(harness, v0=5.0, a0=5.0, vMax=1.0, aMax=2.0, jMax=0.0)
    assert o["dur"] == pytest.approx(0.0, abs=EPS)
    b = o["brake"]
    assert b.t[0] == pytest.approx(0.0, abs=EPS)
    assert b.t[1] == pytest.approx(0.0, abs=EPS)
    assert o["aBrake"] == pytest.approx(5.0, abs=1e-9)
    assert o["vBrake"] == pytest.approx(5.0, abs=1e-9)
