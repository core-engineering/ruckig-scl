import pytest


@pytest.fixture
def harness(make_harness):
    return make_harness("SolveQuartic.s7dcl")


def _roots(harness, a, b, c, d, e):
    harness.reset()
    harness.set_inputs(a=a, b=b, c=c, d=d, e=e)
    harness.execute()
    n = harness.get_output("count")
    out = harness.get_output("roots")
    return sorted(out[i] for i in range(n))


def test_four_distinct_real_roots(harness):
    # (x-1)(x-2)(x-3)(x-4) = x^4 -10x^3 +35x^2 -50x +24  (biquadratic in depressed coord)
    r = _roots(harness, 1.0, -10.0, 35.0, -50.0, 24.0)
    assert r == pytest.approx([1.0, 2.0, 3.0, 4.0], abs=1e-6)


def test_two_real_two_complex(harness):
    # (x^2+1)(x-1)(x-2) = x^4 -3x^3 +3x^2 -3x +2  (general Ferrari branch)
    r = _roots(harness, 1.0, -3.0, 3.0, -3.0, 2.0)
    assert r == pytest.approx([1.0, 2.0], abs=1e-6)


def test_double_root(harness):
    # (x-1)^2 (x-3)(x+2) = expand
    # (x-1)^2 = x^2-2x+1 ; (x-3)(x+2)=x^2 -x -6 ; product:
    # x^4 -3x^3 -3x^2 +11x -6  -> roots {1(double),3,-2}
    r = _roots(harness, 1.0, -3.0, -3.0, 11.0, -6.0)
    assert r == pytest.approx([-2.0, 1.0, 1.0, 3.0], abs=1e-6)


def test_degenerate_cubic(harness):
    # a=0 -> x^3 -6x^2 +11x -6 -> {1,2,3}
    r = _roots(harness, 0.0, 1.0, -6.0, 11.0, -6.0)
    assert r == pytest.approx([1.0, 2.0, 3.0], abs=1e-6)


def test_no_real_roots(harness):
    # (x^2+1)(x^2+4) = x^4 +5x^2 +4 -> no real roots
    r = _roots(harness, 1.0, 0.0, 5.0, 0.0, 4.0)
    assert r == []
