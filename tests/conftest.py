"""Shared fixtures for ruckig-scl unit tests.

Exposes a single set of block search paths so that, at execution time:

- inter-FC calls such as ``"IsFiniteLreal"(...)`` resolve from ``src/blocks``,
- ``_.typeXxx`` UDT references resolve from ``src/data-types``,
- the shared constant block ``"dbRuckigConst"`` auto-loads from ``src/data-blocks``.

Use the ``make_harness`` fixture (a factory) to build a harness for a block:

    @pytest.fixture
    def harness(make_harness):
        return make_harness("ComputeMinDuration.s7dcl")
"""
from pathlib import Path

import pytest

from plc_code.executor import create_harness
from plc_code.executor.runtime import PLCRuntime

PROJECT_ROOT = Path(__file__).parent.parent
BLOCKS_DIR = PROJECT_ROOT / "src/blocks"
SEARCH_PATHS = [
    BLOCKS_DIR,
    PROJECT_ROOT / "src/data-types",
    PROJECT_ROOT / "src/data-blocks",
]


@pytest.fixture
def make_harness():
    """Return a factory that builds a harness for a block file name."""

    def _make(block_name: str):
        runtime = PLCRuntime(block_search_paths=list(SEARCH_PATHS))
        return create_harness(BLOCKS_DIR / block_name, runtime=runtime)

    return _make
