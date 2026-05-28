"""Unit tests for IsFiniteLreal FC."""
import math
from pathlib import Path

import pytest

from plc_code.executor import create_harness

BLOCK_PATH = Path(__file__).parent.parent.parent / "src/blocks/IsFiniteLreal.s7dcl"


@pytest.fixture
def harness():
    return create_harness(BLOCK_PATH)


def _call(harness, x: float) -> bool:
    harness.reset()
    harness.set_inputs(x=x)
    harness.execute()
    return harness.get_output("IsFiniteLreal")


def test_finite_value_returns_true(harness):
    assert _call(harness, 1.5) is True


def test_zero_returns_true(harness):
    assert _call(harness, 0.0) is True


def test_large_finite_returns_true(harness):
    assert _call(harness, 1.0e100) is True


def test_positive_infinity_returns_false(harness):
    assert _call(harness, math.inf) is False


def test_negative_infinity_returns_false(harness):
    assert _call(harness, -math.inf) is False


def test_nan_returns_false(harness):
    assert _call(harness, math.nan) is False
