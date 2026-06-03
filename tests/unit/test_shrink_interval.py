"""Unit tests for ShrinkInterval (safe-Newton single root in a bracket)."""
import math

import pytest


@pytest.fixture
def harness(make_harness):
    return make_harness("ShrinkInterval.s7dcl")


def _root(harness, coeffs, l, h):
    n = len(coeffs)
    p = list(coeffs) + [0.0] * (7 - n)
    harness.reset()
    harness.set_inputs(p=p, n=n, l=l, h=h)
    harness.execute()
    return harness.get_output("ShrinkInterval")


def test_sqrt2(harness):
    # x^2 - 2 on [1,2] -> sqrt(2)
    assert _root(harness, [1.0, 0.0, -2.0], 1.0, 2.0) == pytest.approx(math.sqrt(2), abs=1e-12)


def test_linear(harness):
    # 2x - 3 on [0,5] -> 1.5
    assert _root(harness, [2.0, -3.0], 0.0, 5.0) == pytest.approx(1.5, abs=1e-12)


def test_cubic_root(harness):
    # (x-0.3)(x-1.2)(x+2) expanded, isolate root 0.3 on [0,0.8]
    # x^3 + 0.5x^2 - 2.64x + 0.72
    c = [1.0, 0.5, -2.64, 0.72]
    assert _root(harness, c, 0.0, 0.8) == pytest.approx(0.3, abs=1e-10)


def test_quintic_root(harness):
    # x^5 - 3 on [1,2] -> 3^(1/5)
    assert _root(harness, [1.0, 0.0, 0.0, 0.0, 0.0, -3.0], 1.0, 2.0) == pytest.approx(3.0 ** 0.2, abs=1e-10)


def test_endpoint_root(harness):
    # root exactly at l
    assert _root(harness, [1.0, -1.0], 1.0, 3.0) == pytest.approx(1.0, abs=1e-12)
