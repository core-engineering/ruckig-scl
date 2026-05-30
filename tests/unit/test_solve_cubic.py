import pytest


@pytest.fixture
def harness(make_harness):
    return make_harness("SolveCubic.s7dcl")


def _roots(harness, a, b, c, d):
    harness.reset()
    harness.set_inputs(a=a, b=b, c=c, d=d)
    harness.execute()
    n = harness.get_output("count")
    out = harness.get_output("roots")
    return sorted(out[i] for i in range(n))


def test_three_distinct_real_roots(harness):
    # (x-1)(x-2)(x-3) = x^3 - 6x^2 + 11x - 6
    r = _roots(harness, 1.0, -6.0, 11.0, -6.0)
    assert r == pytest.approx([1.0, 2.0, 3.0], abs=1e-6)


def test_one_real_root(harness):
    # x^3 + x - 2 = (x-1)(x^2 + x + 2) -> single real root x=1
    r = _roots(harness, 1.0, 0.0, 1.0, -2.0)
    assert r == pytest.approx([1.0], abs=1e-6)


def test_double_root(harness):
    # (x-1)^2 (x+2) = x^3 - 3x + 2 -> roots {1 (double), -2}
    r = _roots(harness, 1.0, 0.0, -3.0, 2.0)
    assert r == pytest.approx([-2.0, 1.0], abs=1e-6)


def test_degenerate_quadratic(harness):
    # a=0 -> x^2 - 3x + 2 = 0 -> {1, 2}
    r = _roots(harness, 0.0, 1.0, -3.0, 2.0)
    assert r == pytest.approx([1.0, 2.0], abs=1e-6)


def test_irrational_single_root(harness):
    # x^3 - 2 = 0 -> single real root 2^(1/3)
    r = _roots(harness, 1.0, 0.0, 0.0, -2.0)
    assert r == pytest.approx([2.0 ** (1.0 / 3.0)], abs=1e-6)
