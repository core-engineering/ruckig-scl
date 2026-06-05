"""Sanity checks for the perf profiler (ordering, not absolute numbers)."""
from tests.perf.profile_ruckig import _measure


def test_steady_state_is_cheaper_than_recompute():
    _, rc, st = _measure("4-DoF position", 4, 0)
    assert st.sqrt == 0
    assert st.total_calls < rc.total_calls
    assert rc.sqrt > 0


def test_velocity_recompute_runs():
    _, rc, _ = _measure("4-DoF velocity", 4, 1)
    assert rc.total_calls > 0
