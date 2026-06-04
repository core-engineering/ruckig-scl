"""Unit test scaffold for SolveTimedVel structural guard (v0.10 T1).

Tests the canonical all-zero case (p0=0, v0=0, a0=0, pT=0.5, vT=0, aT=0, tf=8)
via ComputeProfile1AxisTimed.  The oracle (Ruckig minimum_duration=8) produces a
VEL profile with a genuine velocity plateau (t[3] > 0).  After the T1 guard
blocks the degenerate NONE candidate, the SCL solver SHOULD also return a VEL
profile with t[3] > 0 and reach the target.

Current status (T1 only): SolveTimedVel finds a root candidate but CheckProfile
rejects it due to root-precision error (sum(t) ≈ 7.92 ≠ 8.0); SolveTimedNone
still wins with t[3]=0.  The ``t[3] > 1e-6`` assertion therefore FAILS now and
will pass after Task 2 (SolveTimedVel root-precision fix).
"""
from __future__ import annotations

import pytest

ruckig = pytest.importorskip("ruckig")

VMAX, AMAX, JMAX = 3.0, 5.0, 10.0
RESULT_WORKING = 0x7000
_FUNC = "ComputeProfile1AxisTimed"


@pytest.fixture
def harness(make_harness):
    return make_harness("ComputeProfile1AxisTimed.s7dcl")


def _empty_profile():
    return {
        "t": [0.0] * 7, "j": [0.0] * 7,
        "a": [0.0] * 8, "v": [0.0] * 8, "p": [0.0] * 8,
        "direction": 1,
        "controlSigns": 0,
        "brake": {"t": [0.0, 0.0], "j": [0.0, 0.0],
                  "a": [0.0, 0.0, 0.0], "v": [0.0, 0.0, 0.0], "p": [0.0, 0.0, 0.0]},
    }


def _run_timed(harness, p0, v0, a0, pT, vT, aT, tf):
    harness.reset()
    harness.set_inputs(
        profile=_empty_profile(),
        p0=p0, v0=v0, a0=a0,
        pT=pT, vT=vT, aT=aT,
        vMax=VMAX, aMax=AMAX, jMax=JMAX,
        tf=tf,
    )
    harness.execute()
    return harness.get_output(_FUNC), harness.get_var("profile")


def test_canonical_zero_vel_plateau(harness):
    """Canonical all-zero stretched rest-to-rest: tf=8 >> tmin≈1.17.

    The oracle (Ruckig minimum_duration=8) returns a VEL profile with a
    genuine velocity plateau t[3] > 0.  After T1 guards, the SCL solver must:
      - return WORKING (0x7000)
      - produce a profile whose t[3] > 1e-6 (genuine VEL plateau)
      - reach the target: out.p[7] ≈ 0.5, out.v[7] ≈ 0, out.a[7] ≈ 0

    NOTE: This assertion on t[3] > 1e-6 FAILS with T1 only (root imprecision
    keeps SolveTimedVel from being accepted; SolveTimedNone wins with t[3]=0).
    It will PASS after Task 2 (SolveTimedVel root-precision fix).
    """
    p0, v0, a0 = 0.0, 0.0, 0.0
    pT, vT, aT = 0.5, 0.0, 0.0
    tf = 8.0

    ret, prof = _run_timed(harness, p0, v0, a0, pT, vT, aT, tf)

    assert ret == RESULT_WORKING, f"expected WORKING (0x7000), got {hex(ret)}"

    # Target reached
    assert prof.p[7] == pytest.approx(pT, abs=1e-5), "profile must reach target position"
    assert prof.v[7] == pytest.approx(vT, abs=1e-5), "profile must reach target velocity"
    assert prof.a[7] == pytest.approx(aT, abs=1e-5), "profile must reach target acceleration"

    # Velocity plateau guard — FAILS until Task 2
    assert prof.t[3] > 1e-6, (
        f"expected VEL plateau (t[3] > 1e-6) but got t[3]={prof.t[3]:.6f}. "
        "SCL is returning a NONE-shape profile (t[3]=0) instead of the correct "
        "VEL profile. This will be fixed in Task 2 (SolveTimedVel root precision)."
    )
