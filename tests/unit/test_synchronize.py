"""Unit tests for Synchronize (multi-DoF time synchronization)."""
import pytest


@pytest.fixture
def harness(make_harness):
    return make_harness("Synchronize.s7dcl")


def _blk(tMin, aValid=False, aLeft=0.0, aRight=0.0,
         bValid=False, bLeft=0.0, bRight=0.0, found=True):
    return {"tMin": tMin, "found": found,
            "aValid": aValid, "aLeft": aLeft, "aRight": aRight,
            "bValid": bValid, "bLeft": bLeft, "bRight": bRight}


def _run(harness, blocks, n_dofs, minimum_duration=-1.0,
         participates=None, discrete=False, cycle_time=0.010):
    padded = list(blocks) + [_blk(0.0)] * (4 - len(blocks))
    if participates is None:
        participates = [True] * 4
    else:
        participates = list(participates) + [False] * (4 - len(participates))
    harness.reset()
    harness.set_inputs(blockSet={"items": padded}, nDofs=n_dofs,
                       minimumDuration=minimum_duration,
                       participates=participates, discrete=discrete, cycleTime=cycle_time)
    harness.execute()
    return (harness.get_output("resolved"),
            harness.get_output("tSync"),
            harness.get_output("limitingAxis"))


def test_max_tmin_no_interval(harness):
    ok, tsync, lim = _run(harness, [_blk(1.0), _blk(2.0)], 2)
    assert ok is True
    assert tsync == pytest.approx(2.0)
    assert lim == 1


def test_single_dof(harness):
    ok, tsync, lim = _run(harness, [_blk(1.3)], 1)
    assert ok is True
    assert tsync == pytest.approx(1.3)
    assert lim == 0


def test_skips_blocked_interval(harness):
    # DoF0: t_min=1.0 with a blocked interval (1.5, 3.0); DoF1: t_min=2.0.
    # max(t_min)=2.0 is blocked for DoF0 -> advance to the interval's right edge 3.0.
    ok, tsync, lim = _run(harness, [_blk(1.0, aValid=True, aLeft=1.5, aRight=3.0),
                                    _blk(2.0)], 2)
    assert ok is True
    assert tsync == pytest.approx(3.0)


def test_minimum_duration_raises_tsync(harness):
    ok, tsync, lim = _run(harness, [_blk(1.0), _blk(2.0)], 2, minimum_duration=5.0)
    assert ok is True
    assert tsync == pytest.approx(5.0)


def test_minimum_duration_in_blocked_interval(harness):
    # minimum_duration=2.5 lands inside DoF0's blocked (1.5,3.0) -> advance to 3.0.
    ok, tsync, lim = _run(harness, [_blk(1.0, aValid=True, aLeft=1.5, aRight=3.0),
                                    _blk(0.8)], 2, minimum_duration=2.5)
    assert ok is True
    assert tsync == pytest.approx(3.0)


def test_participation_mask_excludes_no_axis(harness):
    # DoF0 t_min=3.0 but NOT participating (No); DoF1 t_min=1.0 participating.
    # t_sync must ignore DoF0 -> 1.0, limiting = DoF1.
    ok, tsync, lim = _run(harness, [_blk(3.0), _blk(1.0)], 2,
                          participates=[False, True])
    assert ok is True
    assert tsync == pytest.approx(1.0)
    assert lim == 1


def test_discrete_rounds_up_to_cycle_grid(harness):
    # t_sync = max(tMin) = 2.0; discrete dt=0.3 -> ceil(2.0/0.3)*0.3 = 2.1
    ok, tsync, lim = _run(harness, [_blk(2.0), _blk(1.0)], 2,
                          discrete=True, cycle_time=0.3)
    assert ok is True
    assert tsync == pytest.approx(2.1, abs=1e-9)
