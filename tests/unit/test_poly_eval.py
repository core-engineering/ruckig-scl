"""Unit tests for PolyEval (Horner polynomial evaluation)."""
import pytest


@pytest.fixture
def harness(make_harness):
    return make_harness("PolyEval.s7dcl")


def _eval(harness, coeffs, x):
    """coeffs[0] = highest-degree term; padded to 7 entries."""
    n = len(coeffs)
    p = list(coeffs) + [0.0] * (7 - n)
    harness.reset()
    harness.set_inputs(p=p, n=n, x=x)
    harness.execute()
    return harness.get_output("PolyEval")


def _ref(coeffs, x):
    """Python reference: coeffs[0]=highest degree."""
    n = len(coeffs)
    return sum(coeffs[i] * x ** (n - 1 - i) for i in range(n))


def test_constant(harness):
    assert _eval(harness, [3.5], 2.0) == pytest.approx(3.5)


def test_linear(harness):
    # 2x + 1 at x=3 -> 7
    assert _eval(harness, [2.0, 1.0], 3.0) == pytest.approx(7.0)


def test_quintic(harness):
    # x^5 - 2x^4 + 0x^3 + 3x^2 - x + 5
    c = [1.0, -2.0, 0.0, 3.0, -1.0, 5.0]
    for x in (-1.5, 0.0, 0.7, 2.0):
        assert _eval(harness, c, x) == pytest.approx(_ref(c, x), abs=1e-9)


def test_sextic(harness):
    c = [1.0, 0.5, -3.0, 2.0, 0.0, -1.0, 4.0]
    for x in (-2.0, 0.3, 1.0, 1.8):
        assert _eval(harness, c, x) == pytest.approx(_ref(c, x), abs=1e-9)


def test_x_zero_returns_constant(harness):
    c = [1.0, -2.0, 3.0, 7.0]
    assert _eval(harness, c, 0.0) == pytest.approx(7.0)
