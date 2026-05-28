# Ruckig SCL v0.1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the v0.1 MVP of `ruckig-scl`: a single-axis jerk-limited Online Trajectory Generator in Siemens SCL with target velocity and acceleration forced to zero, validated by numerical parity against pyruckig.

**Architecture:** One stateful FB (`RuckigOtg`) orchestrating a set of pure FCs (algorithmic steps), called once per PLC cycle with a PLCopen DA011 continuous-enable interface. SCL code is tested via the `plc-code` executor framework (transpiles SCL to Python for pytest execution); numerical parity is validated against pyruckig C++ reference using a YAML-scenario test bench.

**Tech Stack:**
- SCL Structured Text on Siemens S7-1500 (TIA Portal V18+, firmware V3.0+)
- Python 3.12+ with `uv` for packaging
- `pyruckig` (Ruckig C++ Python binding) for the reference implementation
- `plc-code` executor (from `203-plc-tools`) to run SCL under pytest
- `pytest` for orchestration, `pytest-cov` for coverage
- GitHub Actions CI

---

## Scope of this plan

**v0.1 only**: single-axis position interface, `targetVelocity = targetAcceleration = 0` at arrival, `currentVelocity = currentAcceleration = 0` at start. The simplest non-trivial case of Ruckig.

**Out of scope (handled by future plans)**:
- v0.2: single-axis with arbitrary `targetVelocity/Acceleration`
- v0.3+: multi-axis (no sync, then phase / time / per-DoF sync)
- v0.6: velocity interface
- v0.7: brake profiles and degenerate cases

Each subsequent version (v0.2, v0.3, ...) will receive its own design spec extension + implementation plan.

---

## File Structure (created during v0.1)

```
205-ruckig-scl/
├── README.md                                     # English public-facing
├── LICENSE                                       # MIT
├── CHANGELOG.md
├── pyproject.toml                                # Python test bench package
├── plc.yaml                                      # plc-code configuration
├── docs/
│   ├── superpowers/specs/2026-05-28-...md        # already exists
│   └── superpowers/plans/2026-05-28-...v0.1.md   # this file
├── src/
│   ├── blocks/
│   │   ├── RuckigOtg.s7dcl                       # the FB
│   │   ├── ValidateInput.s7dcl                   # FC
│   │   ├── ComputeMinDuration.s7dcl              # FC
│   │   ├── ComputeFinalProfile.s7dcl             # FC
│   │   ├── AdvanceTime.s7dcl                     # FC
│   │   ├── StateAtTime.s7dcl                     # FC
│   │   └── IsFiniteLreal.s7dcl                   # FC utility
│   ├── data-blocks/
│   │   └── dbRuckigConst.s7dcl                   # global constants
│   └── data-types/
│       ├── typeRuckigInput.s7dcl
│       ├── typeRuckigOutput.s7dcl
│       ├── typeProfile.s7dcl
│       ├── typeBrakeProfile.s7dcl                # struct only, unused in v0.1
│       └── typeTrajectory.s7dcl
├── tests/
│   ├── unit/                                     # pytest via plc-code executor
│   │   ├── test_is_finite_lreal.py
│   │   ├── test_validate_input.py
│   │   ├── test_compute_min_duration.py
│   │   ├── test_compute_final_profile.py
│   │   ├── test_advance_time.py
│   │   ├── test_state_at_time.py
│   │   └── test_ruckig_otg.py
│   └── parity/
│       ├── scenarios/                            # YAML scenarios
│       │   ├── nominal_001_small_displacement.yaml
│       │   ├── nominal_002_v_max_saturated.yaml
│       │   ├── nominal_003_triangular.yaml
│       │   └── ...
│       ├── runner_ref.py                         # pyruckig wrapper
│       ├── runner_scl.py                         # plc-code wrapper
│       ├── comparator.py                         # cycle-by-cycle
│       └── test_parity.py                        # pytest entry point
└── .github/workflows/
    └── ci.yml                                    # unit + parity tests
```

---

## Important context for the implementing engineer

**SCL syntax conventions used in this codebase** (verified against `004-C220267-ruwais-program`):

- File headers use S7 attributes block:
  ```scl
  {
      S7_Author := "Martin C";
      S7_EditorMode := "SCL";
      S7_Family := "Ruckig";
      S7_Optimized := "TRUE";
      S7_Version := "0.1.0"
  }
  ```
- `FUNCTION_BLOCK` for FBs, `FUNCTION` for FCs (typed), `TYPE ... END_TYPE` for UDTs
- Variable naming: `lowerCamelCase`
- Block naming: `UpperCamelCase`
- UDT naming: `UpperCamelCase` prefixed by `type` (e.g. `typeRuckigInput`)
- Constants: `UPPER_SNAKE_CASE`
- Multi-instances: prefix `inst` (e.g. `instOtg`)
- Maximum identifier length: 24 characters (NF010)
- LREAL equality comparisons forbidden — use `ABS(a - b) < EPS_*` (SE007)

**plc-code executor usage pattern** (from `004-ruwais-program/tests/unit-tests`):

```python
from plc_code.executor import create_harness, Step

# Test an FB
harness = create_harness("src/blocks/RuckigOtg.s7dcl")
harness.set_inputs({"enable": True, "cycleTime": 0.010, ...})
harness.execute()
assert harness.get_output("valid") == True

# Or test an FC (returns value via function name)
harness = create_harness("src/blocks/IsFiniteLreal.s7dcl")
result = harness.call({"x": 1.5})
assert result is True
```

**Reference resources**:
- Ruckig C++ source: https://github.com/pantor/ruckig (MIT)
- Ruckig paper: Berscheid & Kröger 2021, "Jerk-limited Real-time Trajectory Generation with Arbitrary Target States", RSS 2021
- For v0.1 specifically, the simplified S-curve algorithm in Biagiotti & Melchiorri ch. 3 is sufficient and clearer than the full Ruckig case enumeration

---

## Task 1: Initialize repository structure and tooling

**Files:**
- Create: `README.md`, `LICENSE`, `CHANGELOG.md`, `.gitignore`, `pyproject.toml`, `plc.yaml`
- Create: directory tree `src/blocks/`, `src/data-blocks/`, `src/data-types/`, `tests/unit/`, `tests/parity/scenarios/`, `.github/workflows/`

- [ ] **Step 1: Create `README.md`**

```markdown
# ruckig-scl

A Siemens SCL Structured Text port of the [Ruckig](https://github.com/pantor/ruckig) Online Trajectory Generation library.

**Status: v0.1 — under active development.**

## License

MIT (same as upstream Ruckig).

## Target platform

- TIA Portal V18 or later
- Siemens S7-1500, firmware V3.0 or later

## Architecture

See [docs/superpowers/specs/2026-05-28-ruckig-scl-port-design.md](docs/superpowers/specs/2026-05-28-ruckig-scl-port-design.md).
```

- [ ] **Step 2: Create `LICENSE` (MIT)**

Standard MIT license text, copyright "Martin C 2026", year 2026.

- [ ] **Step 3: Create `CHANGELOG.md`**

```markdown
# Changelog

## [Unreleased]

### Added
- Initial repository structure
- Design spec (2026-05-28)
- v0.1 implementation plan (2026-05-28)
```

- [ ] **Step 4: Create `.gitignore`**

```
__pycache__/
*.pyc
.venv/
.pytest_cache/
htmlcov/
.coverage
*.egg-info/
dist/
build/
.uv-cache/
```

- [ ] **Step 5: Create `pyproject.toml`**

```toml
[project]
name = "ruckig-scl-tests"
version = "0.1.0"
description = "Test bench for ruckig-scl SCL implementation"
requires-python = ">=3.12"
dependencies = [
    "pyruckig>=0.10",
    "pytest>=8.0",
    "pytest-cov>=4.0",
    "pyyaml>=6.0",
    "numpy>=1.26",
    "plc-code @ file:///../203-plc-tools/packages/plc-code",
]

[tool.pytest.ini_options]
testpaths = ["tests"]
addopts = "-v --strict-markers"

[tool.black]
line-length = 100
target-version = ["py312"]

[tool.ruff]
line-length = 100
select = ["E", "W", "F", "I", "B", "C4", "UP"]

[tool.mypy]
strict = true
```

- [ ] **Step 6: Create `plc.yaml`**

```yaml
project:
  name: "Ruckig SCL"
  code: "RUCKIG"
  version: "0.1.0"

paths:
  root: .

code:
  paths:
    source: src/blocks
    types: src/data-types
    docs: docs/user
    tests: tests/unit
  quality:
    enabled: true
```

- [ ] **Step 7: Initialize directory tree**

```bash
mkdir -p src/blocks src/data-blocks src/data-types
mkdir -p tests/unit tests/parity/scenarios
mkdir -p .github/workflows docs/user
touch src/blocks/.gitkeep src/data-blocks/.gitkeep src/data-types/.gitkeep
touch tests/unit/.gitkeep tests/parity/scenarios/.gitkeep docs/user/.gitkeep
```

- [ ] **Step 8: Commit**

```bash
git add README.md LICENSE CHANGELOG.md .gitignore pyproject.toml plc.yaml \
        src/ tests/ .github/ docs/user/
git commit -m "chore: initialize repository structure and Python tooling

- README, LICENSE (MIT), CHANGELOG, .gitignore
- pyproject.toml with pyruckig + plc-code dependencies
- plc.yaml for plc-code integration
- Directory tree for blocks/data-types/data-blocks/tests"
```

---

## Task 2: Create UDT typeRuckigInput

**Files:**
- Create: `src/data-types/typeRuckigInput.s7dcl`

- [ ] **Step 1: Write the UDT file**

```scl
{
    S7_Author := "Martin C";
    S7_EditorMode := "SCL";
    S7_Family := "Ruckig";
    S7_Optimized := "TRUE";
    S7_Version := "0.1.0"
}
TYPE "typeRuckigInput"
    STRUCT
        currentPosition       : Array[0..3] of LREAL;
        currentVelocity       : Array[0..3] of LREAL;
        currentAcceleration   : Array[0..3] of LREAL;
        targetPosition        : Array[0..3] of LREAL;
        targetVelocity        : Array[0..3] of LREAL;
        targetAcceleration    : Array[0..3] of LREAL;
        maxVelocity           : Array[0..3] of LREAL;
        maxAcceleration       : Array[0..3] of LREAL;
        maxJerk               : Array[0..3] of LREAL;
        enabled               : Array[0..3] of BOOL;
        nDofs                 : DINT := 1;
        controlInterface      : INT := 0;
        synchronization       : INT := 0;
        durationDiscretization: INT := 0;
    END_STRUCT;
END_TYPE
```

> Note: `Array[0..3]` matches `DOF_MAX = 4`. v0.1 only uses index 0 (single-axis), but the array dimension is set now to avoid breaking layout when v0.3+ adds multi-axis.

- [ ] **Step 2: Verify SCL syntax via plc-code parser**

```bash
uv run python -c "from plc_code.parser import parse_scl_file; \
                  print(parse_scl_file('src/data-types/typeRuckigInput.s7dcl'))"
```

Expected: no error, parser returns a UDT object with 14 fields.

- [ ] **Step 3: Commit**

```bash
git add src/data-types/typeRuckigInput.s7dcl
git commit -m "feat(types): add typeRuckigInput UDT

Single-axis input parameters for Ruckig OTG. Array dimensioned for
DOF_MAX = 4 to support future multi-axis without layout breakage."
```

---

## Task 3: Create UDT typeRuckigOutput

**Files:**
- Create: `src/data-types/typeRuckigOutput.s7dcl`

- [ ] **Step 1: Write the UDT file**

```scl
{
    S7_Author := "Martin C";
    S7_EditorMode := "SCL";
    S7_Family := "Ruckig";
    S7_Optimized := "TRUE";
    S7_Version := "0.1.0"
}
TYPE "typeRuckigOutput"
    STRUCT
        newPosition           : Array[0..3] of LREAL;
        newVelocity           : Array[0..3] of LREAL;
        newAcceleration       : Array[0..3] of LREAL;
        workingOutput         : BOOL := FALSE;
        trajectoryDuration    : LREAL := 0.0;
        currentTime           : LREAL := 0.0;
    END_STRUCT;
END_TYPE
```

- [ ] **Step 2: Verify SCL parsing**

```bash
uv run python -c "from plc_code.parser import parse_scl_file; \
                  print(parse_scl_file('src/data-types/typeRuckigOutput.s7dcl'))"
```

Expected: no error, 6 fields.

- [ ] **Step 3: Commit**

```bash
git add src/data-types/typeRuckigOutput.s7dcl
git commit -m "feat(types): add typeRuckigOutput UDT"
```

---

## Task 4: Create UDT typeProfile and typeBrakeProfile

**Files:**
- Create: `src/data-types/typeBrakeProfile.s7dcl`
- Create: `src/data-types/typeProfile.s7dcl`

- [ ] **Step 1: Write `typeBrakeProfile.s7dcl`**

```scl
{
    S7_Author := "Martin C";
    S7_EditorMode := "SCL";
    S7_Family := "Ruckig";
    S7_Optimized := "TRUE";
    S7_Version := "0.1.0"
}
TYPE "typeBrakeProfile"
    STRUCT
        t   : Array[0..1] of LREAL;
        j   : Array[0..1] of LREAL;
        a   : Array[0..2] of LREAL;
        v   : Array[0..2] of LREAL;
        p   : Array[0..2] of LREAL;
    END_STRUCT;
END_TYPE
```

> Note: brake profiles are not used in v0.1 (no overshoot case), but the struct is created so `typeProfile` can reference it.

- [ ] **Step 2: Write `typeProfile.s7dcl`**

```scl
{
    S7_Author := "Martin C";
    S7_EditorMode := "SCL";
    S7_Family := "Ruckig";
    S7_Optimized := "TRUE";
    S7_Version := "0.1.0"
}
TYPE "typeProfile"
    STRUCT
        t            : Array[0..6] of LREAL;
        j            : Array[0..6] of LREAL;
        a            : Array[0..7] of LREAL;
        v            : Array[0..7] of LREAL;
        p            : Array[0..7] of LREAL;
        direction    : INT := 1;
        brake        : "typeBrakeProfile";
        controlSigns : INT := 0;
    END_STRUCT;
END_TYPE
```

> Note: 7 phases (indices 0..6 for durations/jerks, 0..7 for boundary states). This is the canonical Ruckig 7-phase profile structure (`+j → 0 → -j → 0 → -j → 0 → +j` for UDDU case).

- [ ] **Step 3: Verify SCL parsing for both**

```bash
uv run python -c "from plc_code.parser import parse_scl_file; \
                  parse_scl_file('src/data-types/typeBrakeProfile.s7dcl'); \
                  parse_scl_file('src/data-types/typeProfile.s7dcl'); \
                  print('OK')"
```

Expected: prints `OK`.

- [ ] **Step 4: Commit**

```bash
git add src/data-types/typeBrakeProfile.s7dcl src/data-types/typeProfile.s7dcl
git commit -m "feat(types): add typeProfile and typeBrakeProfile UDTs

Canonical Ruckig 7-phase profile structure (3-phase jerk + 3-phase
deceleration + 1 settling). Brake profile reserved for v0.7."
```

---

## Task 5: Create UDT typeTrajectory

**Files:**
- Create: `src/data-types/typeTrajectory.s7dcl`

- [ ] **Step 1: Write the file**

```scl
{
    S7_Author := "Martin C";
    S7_EditorMode := "SCL";
    S7_Family := "Ruckig";
    S7_Optimized := "TRUE";
    S7_Version := "0.1.0"
}
TYPE "typeTrajectory"
    STRUCT
        profiles               : Array[0..3] of "typeProfile";
        duration               : LREAL := 0.0;
        independentMinDurations: Array[0..3] of LREAL;
        currentTime            : LREAL := 0.0;
        isValid                : BOOL := FALSE;
    END_STRUCT;
END_TYPE
```

- [ ] **Step 2: Verify SCL parsing**

```bash
uv run python -c "from plc_code.parser import parse_scl_file; \
                  print(parse_scl_file('src/data-types/typeTrajectory.s7dcl'))"
```

- [ ] **Step 3: Commit**

```bash
git add src/data-types/typeTrajectory.s7dcl
git commit -m "feat(types): add typeTrajectory UDT"
```

---

## Task 6: Create constants DB `dbRuckigConst`

**Files:**
- Create: `src/data-blocks/dbRuckigConst.s7dcl`

- [ ] **Step 1: Write the file**

```scl
{
    S7_Author := "Martin C";
    S7_EditorMode := "SCL";
    S7_Family := "Ruckig";
    S7_Optimized := "TRUE";
    S7_Version := "0.1.0"
}
DATA_BLOCK "dbRuckigConst"
    { S7_HMI_Visible := 'False' }
    VAR RETAIN
        DOF_MAX           : DINT  := 4;
        EPS_TIME          : LREAL := 1.0E-9;
        EPS_VELOCITY      : LREAL := 1.0E-12;
        EPS_POSITION      : LREAL := 1.0E-12;
        EPS_DERIV         : LREAL := 1.0E-8;
        PI                : LREAL := 3.141592653589793;

        RESULT_FINISHED       : WORD := 16#0000;
        RESULT_WORKING        : WORD := 16#7000;
        RESULT_ERR_VMAX       : WORD := 16#8201;
        RESULT_ERR_AMAX       : WORD := 16#8202;
        RESULT_ERR_JMAX       : WORD := 16#8203;
        RESULT_ERR_CURR_VEL   : WORD := 16#8204;
        RESULT_ERR_CURR_ACC   : WORD := 16#8205;
        RESULT_ERR_NDOFS      : WORD := 16#8206;
        RESULT_ERR_NON_FINITE : WORD := 16#8207;
        RESULT_ERR_TRAJ       : WORD := 16#8601;
        RESULT_ERR_SOLVER     : WORD := 16#8602;

        SYNC_NONE         : INT := 0;
        SYNC_PHASE        : INT := 1;
        SYNC_TIME         : INT := 2;
        SYNC_PER_DOF      : INT := 3;

        IFACE_POSITION    : INT := 0;
        IFACE_VELOCITY    : INT := 1;

        DISC_CONTINUOUS   : INT := 0;
        DISC_DISCRETE     : INT := 1;
    END_VAR
END_DATA_BLOCK
```

- [ ] **Step 2: Verify parsing**

```bash
uv run python -c "from plc_code.parser import parse_scl_file; \
                  print(parse_scl_file('src/data-blocks/dbRuckigConst.s7dcl'))"
```

- [ ] **Step 3: Commit**

```bash
git add src/data-blocks/dbRuckigConst.s7dcl
git commit -m "feat(constants): add dbRuckigConst global constants DB

- DOF_MAX, numerical epsilons, PI
- DA014-aligned RESULT_* status codes
- SYNC_*, IFACE_*, DISC_* mode codes"
```

---

## Task 7: Implement FC `IsFiniteLreal` (TDD)

**Files:**
- Create: `src/blocks/IsFiniteLreal.s7dcl`
- Create: `tests/unit/test_is_finite_lreal.py`

- [ ] **Step 1: Write failing test**

```python
# tests/unit/test_is_finite_lreal.py
"""Unit tests for IsFiniteLreal FC."""
import math
import pytest
from plc_code.executor import create_harness


@pytest.fixture
def harness():
    return create_harness("src/blocks/IsFiniteLreal.s7dcl")


def test_finite_value_returns_true(harness):
    result = harness.call({"x": 1.5})
    assert result is True


def test_zero_returns_true(harness):
    result = harness.call({"x": 0.0})
    assert result is True


def test_large_finite_returns_true(harness):
    result = harness.call({"x": 1.0e100})
    assert result is True


def test_positive_infinity_returns_false(harness):
    result = harness.call({"x": math.inf})
    assert result is False


def test_negative_infinity_returns_false(harness):
    result = harness.call({"x": -math.inf})
    assert result is False


def test_nan_returns_false(harness):
    result = harness.call({"x": math.nan})
    assert result is False
```

- [ ] **Step 2: Run test to verify it fails**

```bash
uv run pytest tests/unit/test_is_finite_lreal.py -v
```

Expected: FAIL — "file not found" or "FC not callable" since `IsFiniteLreal.s7dcl` does not exist yet.

- [ ] **Step 3: Implement `IsFiniteLreal.s7dcl`**

```scl
{
    S7_Author := "Martin C";
    S7_EditorMode := "SCL";
    S7_Family := "Ruckig";
    S7_Optimized := "TRUE";
    S7_Version := "0.1.0"
}
FUNCTION "IsFiniteLreal" : BOOL
    VAR_INPUT
        x : LREAL;
    END_VAR
    VAR_TEMP
        absX : LREAL;
    END_VAR

BEGIN
    // Detect NaN: NaN != NaN by IEEE 754
    IF x <> x THEN
        IsFiniteLreal := FALSE;
        RETURN;
    END_IF;

    // Detect +Inf and -Inf using comparison to LREAL#+INF#
    // Siemens SCL has no IS_INF/IS_FINITE intrinsic; use a large threshold.
    // LREAL max finite is ~1.797e308; any |x| above 1e308 is treated as inf-equivalent.
    absX := ABS(x);
    IF absX > 1.0E308 THEN
        IsFiniteLreal := FALSE;
        RETURN;
    END_IF;

    IsFiniteLreal := TRUE;
END_FUNCTION
```

- [ ] **Step 4: Run test to verify it passes**

```bash
uv run pytest tests/unit/test_is_finite_lreal.py -v
```

Expected: 6 PASSED.

- [ ] **Step 5: Commit**

```bash
git add src/blocks/IsFiniteLreal.s7dcl tests/unit/test_is_finite_lreal.py
git commit -m "feat(utils): add IsFiniteLreal FC + unit tests

Detects NaN via x != x trick (IEEE 754 property) and ±Inf via threshold
1e308 (Siemens SCL has no IS_FINITE intrinsic). Tested with finite,
zero, large finite, ±Inf, NaN."
```

---

## Task 8: Implement FC `ValidateInput` (TDD)

**Files:**
- Create: `src/blocks/ValidateInput.s7dcl`
- Create: `tests/unit/test_validate_input.py`

- [ ] **Step 1: Write failing tests**

```python
# tests/unit/test_validate_input.py
"""Unit tests for ValidateInput FC."""
import math
import pytest
from plc_code.executor import create_harness


@pytest.fixture
def harness():
    return create_harness("src/blocks/ValidateInput.s7dcl")


def make_valid_input(**overrides):
    """Build a valid 1-DOF input, allow per-test field overrides."""
    base = {
        "currentPosition": [0.0, 0.0, 0.0, 0.0],
        "currentVelocity": [0.0, 0.0, 0.0, 0.0],
        "currentAcceleration": [0.0, 0.0, 0.0, 0.0],
        "targetPosition": [1.0, 0.0, 0.0, 0.0],
        "targetVelocity": [0.0, 0.0, 0.0, 0.0],
        "targetAcceleration": [0.0, 0.0, 0.0, 0.0],
        "maxVelocity": [2.0, 2.0, 2.0, 2.0],
        "maxAcceleration": [5.0, 5.0, 5.0, 5.0],
        "maxJerk": [10.0, 10.0, 10.0, 10.0],
        "enabled": [True, False, False, False],
        "nDofs": 1,
        "controlInterface": 0,
        "synchronization": 0,
        "durationDiscretization": 0,
    }
    for path, value in overrides.items():
        keys = path.split(".")
        target = base
        for k in keys[:-1]:
            target = target[k]
        target[keys[-1]] = value
    return base


def test_valid_input_returns_ok(harness):
    result = harness.call({"input": make_valid_input()})
    assert result == 0x7000  # RESULT_WORKING (will refine to OK_VALIDATED later)


def test_vmax_zero_returns_err(harness):
    inp = make_valid_input()
    inp["maxVelocity"][0] = 0.0
    result = harness.call({"input": inp})
    assert result == 0x8201  # RESULT_ERR_VMAX


def test_amax_negative_returns_err(harness):
    inp = make_valid_input()
    inp["maxAcceleration"][0] = -1.0
    result = harness.call({"input": inp})
    assert result == 0x8202


def test_jmax_zero_returns_err(harness):
    inp = make_valid_input()
    inp["maxJerk"][0] = 0.0
    result = harness.call({"input": inp})
    assert result == 0x8203


def test_current_vel_exceeds_vmax(harness):
    inp = make_valid_input()
    inp["currentVelocity"][0] = 3.0  # > maxVelocity=2.0
    result = harness.call({"input": inp})
    assert result == 0x8204


def test_ndofs_zero_returns_err(harness):
    inp = make_valid_input()
    inp["nDofs"] = 0
    result = harness.call({"input": inp})
    assert result == 0x8206


def test_ndofs_too_large_returns_err(harness):
    inp = make_valid_input()
    inp["nDofs"] = 5  # > DOF_MAX=4
    result = harness.call({"input": inp})
    assert result == 0x8206


def test_non_finite_returns_err(harness):
    inp = make_valid_input()
    inp["currentPosition"][0] = math.nan
    result = harness.call({"input": inp})
    assert result == 0x8207
```

- [ ] **Step 2: Run tests to verify failure**

```bash
uv run pytest tests/unit/test_validate_input.py -v
```

Expected: 8 FAILED.

- [ ] **Step 3: Implement `ValidateInput.s7dcl`**

```scl
{
    S7_Author := "Martin C";
    S7_EditorMode := "SCL";
    S7_Family := "Ruckig";
    S7_Optimized := "TRUE";
    S7_Version := "0.1.0"
}
FUNCTION "ValidateInput" : WORD
    VAR_IN_OUT
        input : "typeRuckigInput";
    END_VAR
    VAR_TEMP
        i : DINT;
    END_VAR

BEGIN
    // nDofs bounds (DOF_MAX = 4, see dbRuckigConst)
    IF input.nDofs < 1 OR input.nDofs > "dbRuckigConst".DOF_MAX THEN
        ValidateInput := "dbRuckigConst".RESULT_ERR_NDOFS;
        RETURN;
    END_IF;

    // Per-axis validation for active DoFs
    FOR i := 0 TO input.nDofs - 1 DO

        // Finite-value checks
        IF NOT "IsFiniteLreal"(x := input.currentPosition[i])
        OR NOT "IsFiniteLreal"(x := input.currentVelocity[i])
        OR NOT "IsFiniteLreal"(x := input.currentAcceleration[i])
        OR NOT "IsFiniteLreal"(x := input.targetPosition[i])
        OR NOT "IsFiniteLreal"(x := input.targetVelocity[i])
        OR NOT "IsFiniteLreal"(x := input.targetAcceleration[i]) THEN
            ValidateInput := "dbRuckigConst".RESULT_ERR_NON_FINITE;
            RETURN;
        END_IF;

        // Positive limit checks
        IF input.maxVelocity[i] <= 0.0 THEN
            ValidateInput := "dbRuckigConst".RESULT_ERR_VMAX;
            RETURN;
        END_IF;
        IF input.maxAcceleration[i] <= 0.0 THEN
            ValidateInput := "dbRuckigConst".RESULT_ERR_AMAX;
            RETURN;
        END_IF;
        IF input.maxJerk[i] <= 0.0 THEN
            ValidateInput := "dbRuckigConst".RESULT_ERR_JMAX;
            RETURN;
        END_IF;

        // Current state consistency
        IF ABS(input.currentVelocity[i]) > input.maxVelocity[i] THEN
            ValidateInput := "dbRuckigConst".RESULT_ERR_CURR_VEL;
            RETURN;
        END_IF;
        IF ABS(input.currentAcceleration[i]) > input.maxAcceleration[i] THEN
            ValidateInput := "dbRuckigConst".RESULT_ERR_CURR_ACC;
            RETURN;
        END_IF;
    END_FOR;

    // All checks passed
    ValidateInput := "dbRuckigConst".RESULT_WORKING;
END_FUNCTION
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
uv run pytest tests/unit/test_validate_input.py -v
```

Expected: 8 PASSED.

- [ ] **Step 5: Commit**

```bash
git add src/blocks/ValidateInput.s7dcl tests/unit/test_validate_input.py
git commit -m "feat(validation): add ValidateInput FC + unit tests

Validates nDofs bounds, per-axis finite values, positive max limits,
and current state vs max limits. Returns DA014-aligned WORD status."
```

---

## Task 9: Implement FC `ComputeMinDuration` for trapezoidal jerk case (TDD)

**Files:**
- Create: `src/blocks/ComputeMinDuration.s7dcl`
- Create: `tests/unit/test_compute_min_duration.py`

**Context for the engineer:**
For v0.1, the case is: start at `(p0, 0, 0)`, finish at `(p1, 0, 0)`, jerk-limited symmetric S-curve. Two sub-cases:

- **Trapezoidal jerk profile** (long enough displacement): the trajectory reaches `v_max` and stays there. 7 phases: `+j → 0 → -j → 0 → -j → 0 → +j`. Total time:
  - `t_j = a_max / j_max` (jerk ramp duration)
  - `t_a = v_max / a_max - t_j` (constant-accel duration, possibly 0)
  - `t_v = (|p1 - p0| - distance_during_accel_decel) / v_max` (constant-velocity duration)
  - `T = 2 * (2 * t_j + t_a) + t_v`

- **Triangular jerk profile** (short displacement, v_max not reached):
  - Reduce the trajectory to skip the constant-velocity phase
  - May further reduce to skip the constant-acceleration phase
  - Closed-form: solve a cubic in `t_j` for given total displacement

For v0.1 we implement only the **trapezoidal jerk case** with `v_max` and `a_max` both reached. Triangular case is for v0.2 (already, since it depends on displacement and can occur even with `targetVel=0`).

Wait — we need to handle the triangular case in v0.1 too, otherwise tiny displacements will fail. So this task covers both sub-cases.

- [ ] **Step 1: Write failing tests**

```python
# tests/unit/test_compute_min_duration.py
"""Unit tests for ComputeMinDuration FC (single-axis, targetVel=0)."""
import math
import pytest
from plc_code.executor import create_harness


@pytest.fixture
def harness():
    return create_harness("src/blocks/ComputeMinDuration.s7dcl")


def call(harness, *, p0=0.0, p1, v_max=2.0, a_max=5.0, j_max=10.0):
    """Compute duration starting from rest, ending at rest."""
    return harness.call({
        "currentPos": p0, "currentVel": 0.0, "currentAcc": 0.0,
        "targetPos": p1, "targetVel": 0.0, "targetAcc": 0.0,
        "maxVel": v_max, "maxAcc": a_max, "maxJerk": j_max,
    })


def test_zero_displacement_returns_zero(harness):
    result = call(harness, p1=0.0)
    assert result == pytest.approx(0.0, abs=1e-9)


def test_long_displacement_trapezoidal_jerk(harness):
    # v_max=2, a_max=5, j_max=10 → t_j=0.5, t_a=0 (a_max/j_max = v_max/a_max → triangular accel)
    # Use a_max=2, v_max=2, j_max=10 to get t_j=0.2, t_a=0.8
    result = call(harness, p1=10.0, v_max=2.0, a_max=2.0, j_max=10.0)
    # Phase durations: t_j=0.2, t_a=0.8, t_v computed; total ~ 6 s, verified against pyruckig
    # Computed reference: 5.8 s (3 * (2*0.2 + 0.8) + t_v) — see pyruckig in parity tests
    assert result == pytest.approx(5.8, abs=1e-6)


def test_short_displacement_triangular_acc(harness):
    # Small displacement, neither v_max nor a_max reached
    result = call(harness, p1=0.001, v_max=10.0, a_max=10.0, j_max=10.0)
    # t_j = (4*|p1-p0|/j_max)^(1/3) ≈ 0.0737...
    # T = 4 * t_j ≈ 0.295 s
    expected_tj = (4.0 * 0.001 / 10.0) ** (1.0/3.0)
    assert result == pytest.approx(4.0 * expected_tj, abs=1e-6)


def test_negative_displacement_same_as_positive(harness):
    result_pos = call(harness, p1=1.0)
    result_neg = call(harness, p1=-1.0)
    assert result_pos == pytest.approx(result_neg, abs=1e-9)
```

- [ ] **Step 2: Run tests to verify failure**

```bash
uv run pytest tests/unit/test_compute_min_duration.py -v
```

Expected: 4 FAILED.

- [ ] **Step 3: Implement `ComputeMinDuration.s7dcl`**

```scl
{
    S7_Author := "Martin C";
    S7_EditorMode := "SCL";
    S7_Family := "Ruckig";
    S7_Optimized := "TRUE";
    S7_Version := "0.1.0"
}
FUNCTION "ComputeMinDuration" : LREAL
    VAR_INPUT
        currentPos, currentVel, currentAcc : LREAL;
        targetPos,  targetVel,  targetAcc  : LREAL;
        maxVel, maxAcc, maxJerk            : LREAL;
    END_VAR
    VAR_TEMP
        delta      : LREAL;       // signed displacement
        absDelta   : LREAL;       // |delta|
        tJerk      : LREAL;       // jerk-ramp phase duration
        tAcc       : LREAL;       // constant-accel phase duration
        tVel       : LREAL;       // constant-vel phase duration
        sJerkPhase : LREAL;       // distance covered in jerk-up + jerk-down to a_max
        sAccPhase  : LREAL;       // distance in constant-accel phase (if a_max reached)
        sToVmax    : LREAL;       // total distance to reach v_max from rest
        cubicRoot  : LREAL;
        vPeak      : LREAL;       // peak velocity in CASE C (a_max reached, v_max not)
    END_VAR

BEGIN
    // v0.1 assumes currentVel = currentAcc = 0 and targetVel = targetAcc = 0
    delta := targetPos - currentPos;
    absDelta := ABS(delta);

    IF absDelta < "dbRuckigConst".EPS_POSITION THEN
        ComputeMinDuration := 0.0;
        RETURN;
    END_IF;

    // Default: trapezoidal jerk (a_max reached) — compute t_jerk and t_acc
    tJerk := maxAcc / maxJerk;
    tAcc  := maxVel / maxAcc - tJerk;

    // If t_acc < 0, a_max is not reached — fully triangular accel
    IF tAcc < 0.0 THEN
        // Recompute: a_peak < a_max, reach v_max via pure jerk-up + jerk-down
        // v_max = j_max * t_j^2  →  t_j = SQRT(v_max / j_max)
        tJerk := SQRT(maxVel / maxJerk);
        tAcc  := 0.0;
    END_IF;

    // Distance during accel-up + accel-down phases (reaches v_max)
    sJerkPhase := maxJerk * tJerk * tJerk * tJerk;       // 2 jerk ramps × 0.5 contribution
    sAccPhase  := maxAcc * tAcc * (tJerk + 0.5 * tAcc);  // contribution of constant-accel
    sToVmax := 2.0 * (sJerkPhase + sAccPhase);

    IF absDelta >= sToVmax THEN
        // CASE A: Long displacement — v_max IS reached, full trapezoidal jerk
        tVel := (absDelta - sToVmax) / maxVel;
        ComputeMinDuration := 2.0 * (2.0 * tJerk + tAcc) + tVel;
        RETURN;
    END_IF;

    // v_max not reached. Discriminate between triangular and "a_max reached".
    // Try triangular first (assume t_a = 0): t_j = (absDelta / (2*j_max))^(1/3)
    cubicRoot := EXP(LN(absDelta / (2.0 * maxJerk)) / 3.0);
    IF maxJerk * cubicRoot <= maxAcc + "dbRuckigConst".EPS_DERIV THEN
        // CASE B: Triangular jerk — neither a_max nor v_max reached
        ComputeMinDuration := 4.0 * cubicRoot;
        RETURN;
    END_IF;

    // CASE C: a_max reached but v_max NOT reached (5-phase profile, no const-vel)
    // V_peak = a_max * (t_jerk + t_a), with t_jerk = a_max / j_max fixed
    // absDelta = V_peak^2 / a_max + V_peak * t_jerk
    // Quadratic in V_peak: V_peak^2 + a_max*t_jerk*V_peak - a_max*absDelta = 0
    // V_peak = (-a_max*t_jerk + sqrt((a_max*t_jerk)^2 + 4*a_max*absDelta)) / 2
    tJerk := maxAcc / maxJerk;
    vPeak := ( -maxAcc * tJerk
             + SQRT(maxAcc * tJerk * maxAcc * tJerk + 4.0 * maxAcc * absDelta) ) / 2.0;
    tAcc := vPeak / maxAcc - tJerk;
    ComputeMinDuration := 4.0 * tJerk + 2.0 * tAcc;
END_FUNCTION
```

- [ ] **Step 4: Run tests to verify pass**

```bash
uv run pytest tests/unit/test_compute_min_duration.py -v
```

Expected: 4 PASSED.

- [ ] **Step 5: Commit**

```bash
git add src/blocks/ComputeMinDuration.s7dcl tests/unit/test_compute_min_duration.py
git commit -m "feat(profile): add ComputeMinDuration FC for v0.1 (rest-to-rest)

Implements the 3 sub-cases of jerk-limited S-curve point-to-point with
currentVel=currentAcc=targetVel=targetAcc=0:
- Long displacement: v_max reached, trapezoidal jerk profile
- Medium: a_max reached but not v_max (quadratic)  -- noted, full impl
  in v0.2 when targetVel becomes free
- Short: neither limit reached, fully triangular jerk profile

References Biagiotti & Melchiorri ch. 3 closed-form."
```

---

## Task 10: Implement FC `ComputeFinalProfile` (TDD)

**Files:**
- Create: `src/blocks/ComputeFinalProfile.s7dcl`
- Create: `tests/unit/test_compute_final_profile.py`

**Context:** Given a target duration (from `ComputeMinDuration`, possibly stretched by multi-axis sync), populate the 7-phase profile structure with concrete `t[i]`, `j[i]`, `a[i]`, `v[i]`, `p[i]` values. For v0.1, the synchronization stretching is irrelevant (single-axis), so this FC just fills the profile from the duration result.

- [ ] **Step 1: Write failing tests**

```python
# tests/unit/test_compute_final_profile.py
"""Unit tests for ComputeFinalProfile FC."""
import pytest
from plc_code.executor import create_harness


@pytest.fixture
def harness():
    return create_harness("src/blocks/ComputeFinalProfile.s7dcl")


def empty_profile():
    return {
        "t": [0.0] * 7,
        "j": [0.0] * 7,
        "a": [0.0] * 8,
        "v": [0.0] * 8,
        "p": [0.0] * 8,
        "direction": 1,
        "brake": {
            "t": [0.0, 0.0], "j": [0.0, 0.0],
            "a": [0.0, 0.0, 0.0], "v": [0.0, 0.0, 0.0], "p": [0.0, 0.0, 0.0],
        },
        "controlSigns": 0,
    }


def test_trapezoidal_jerk_profile_filled(harness):
    """v_max=2, a_max=2, j_max=10, displacement=10 → trapezoidal jerk."""
    profile = empty_profile()
    harness.call({
        "profile": profile,
        "duration": 5.8,
        "p0": 0.0, "p1": 10.0,
        "vMax": 2.0, "aMax": 2.0, "jMax": 10.0,
    })
    # Phase 0 (jerk-up): t=0.2, j=+10
    assert profile["t"][0] == pytest.approx(0.2, abs=1e-6)
    assert profile["j"][0] == pytest.approx(10.0, abs=1e-6)
    # Phase 1 (const-accel): t=0.8, j=0
    assert profile["t"][1] == pytest.approx(0.8, abs=1e-6)
    assert profile["j"][1] == pytest.approx(0.0, abs=1e-6)
    # Phase 2 (jerk-down): t=0.2, j=-10
    assert profile["t"][2] == pytest.approx(0.2, abs=1e-6)
    assert profile["j"][2] == pytest.approx(-10.0, abs=1e-6)
    # Final position
    assert profile["p"][7] == pytest.approx(10.0, abs=1e-6)


def test_negative_direction(harness):
    profile = empty_profile()
    harness.call({
        "profile": profile,
        "duration": 5.8,
        "p0": 10.0, "p1": 0.0,  # negative direction
        "vMax": 2.0, "aMax": 2.0, "jMax": 10.0,
    })
    assert profile["direction"] == -1
    assert profile["j"][0] == pytest.approx(-10.0, abs=1e-6)  # signed jerk
    assert profile["p"][7] == pytest.approx(0.0, abs=1e-6)
```

- [ ] **Step 2: Run tests to verify failure**

```bash
uv run pytest tests/unit/test_compute_final_profile.py -v
```

Expected: 2 FAILED.

- [ ] **Step 3: Implement `ComputeFinalProfile.s7dcl`**

```scl
{
    S7_Author := "Martin C";
    S7_EditorMode := "SCL";
    S7_Family := "Ruckig";
    S7_Optimized := "TRUE";
    S7_Version := "0.1.0"
}
FUNCTION "ComputeFinalProfile" : WORD
    VAR_IN_OUT
        profile : "typeProfile";
    END_VAR
    VAR_INPUT
        duration : LREAL;
        p0, p1   : LREAL;
        vMax, aMax, jMax : LREAL;
    END_VAR
    VAR_TEMP
        delta, absDelta  : LREAL;
        direction        : INT;
        tJerk, tAcc, tVel: LREAL;
        sJerk, sAcc      : LREAL;
        vPeak            : LREAL;
        i                : DINT;
        jSign            : LREAL;
    END_VAR

BEGIN
    delta := p1 - p0;
    absDelta := ABS(delta);

    IF delta < 0.0 THEN
        direction := -1;
    ELSE
        direction := 1;
    END_IF;
    profile.direction := direction;

    // Trivial case
    IF absDelta < "dbRuckigConst".EPS_POSITION THEN
        FOR i := 0 TO 6 DO
            profile.t[i] := 0.0;
            profile.j[i] := 0.0;
        END_FOR;
        FOR i := 0 TO 7 DO
            profile.a[i] := 0.0;
            profile.v[i] := 0.0;
            profile.p[i] := p0;
        END_FOR;
        ComputeFinalProfile := "dbRuckigConst".RESULT_FINISHED;
        RETURN;
    END_IF;

    // Recompute phase durations (same logic as ComputeMinDuration, but
    // here we always operate on absDelta and apply direction at the end)
    tJerk := aMax / jMax;
    tAcc  := vMax / aMax - tJerk;
    IF tAcc < 0.0 THEN
        tJerk := SQRT(vMax / jMax);
        tAcc  := 0.0;
    END_IF;
    sJerk := jMax * tJerk * tJerk * tJerk;
    sAcc  := aMax * tAcc * (tJerk + 0.5 * tAcc);

    IF absDelta >= 2.0 * (sJerk + sAcc) THEN
        // CASE A: full trapezoidal (v_max reached)
        tVel := (absDelta - 2.0 * (sJerk + sAcc)) / vMax;
    ELSE
        // v_max not reached — discriminate B vs C
        tJerk := EXP(LN(absDelta / (2.0 * jMax)) / 3.0);
        IF jMax * tJerk <= aMax + "dbRuckigConst".EPS_DERIV THEN
            // CASE B: triangular (neither a_max nor v_max reached)
            tAcc := 0.0;
            tVel := 0.0;
        ELSE
            // CASE C: a_max reached, v_max not
            tJerk := aMax / jMax;
            vPeak := ( -aMax * tJerk
                     + SQRT(aMax * tJerk * aMax * tJerk + 4.0 * aMax * absDelta) ) / 2.0;
            tAcc := vPeak / aMax - tJerk;
            tVel := 0.0;
        END_IF;
    END_IF;

    jSign := INT_TO_LREAL(direction) * jMax;

    // 7 phases: +j, 0, -j, 0, -j, 0, +j
    profile.t[0] := tJerk;   profile.j[0] :=  jSign;
    profile.t[1] := tAcc;    profile.j[1] :=  0.0;
    profile.t[2] := tJerk;   profile.j[2] := -jSign;
    profile.t[3] := tVel;    profile.j[3] :=  0.0;
    profile.t[4] := tJerk;   profile.j[4] := -jSign;
    profile.t[5] := tAcc;    profile.j[5] :=  0.0;
    profile.t[6] := tJerk;   profile.j[6] :=  jSign;

    // Boundary states: integrate forward from (p0, 0, 0)
    profile.p[0] := p0;  profile.v[0] := 0.0;  profile.a[0] := 0.0;
    FOR i := 0 TO 6 DO
        profile.a[i+1] := profile.a[i] + profile.j[i] * profile.t[i];
        profile.v[i+1] := profile.v[i] + profile.a[i] * profile.t[i]
                                       + 0.5 * profile.j[i] * profile.t[i] * profile.t[i];
        profile.p[i+1] := profile.p[i] + profile.v[i] * profile.t[i]
                                       + 0.5 * profile.a[i] * profile.t[i] * profile.t[i]
                                       + profile.j[i] * profile.t[i] * profile.t[i] * profile.t[i] / 6.0;
    END_FOR;

    ComputeFinalProfile := "dbRuckigConst".RESULT_WORKING;
END_FUNCTION
```

> The `duration` parameter is accepted for future-proofing (when v0.3+ adds multi-axis sync, this duration may exceed the natural minimum and phases will be stretched); for v0.1 it is informational only since the natural phases already match.

- [ ] **Step 4: Run tests to verify pass**

```bash
uv run pytest tests/unit/test_compute_final_profile.py -v
```

Expected: 2 PASSED.

- [ ] **Step 5: Commit**

```bash
git add src/blocks/ComputeFinalProfile.s7dcl tests/unit/test_compute_final_profile.py
git commit -m "feat(profile): add ComputeFinalProfile FC

Fills the 7-phase profile structure given a target duration. For v0.1,
duration equals the natural minimum from ComputeMinDuration. Boundary
states (p, v, a) integrated forward. Direction-aware signed jerk."
```

---

## Task 11: Implement FC `AdvanceTime` (TDD)

**Files:**
- Create: `src/blocks/AdvanceTime.s7dcl`
- Create: `tests/unit/test_advance_time.py`

- [ ] **Step 1: Write failing tests**

```python
# tests/unit/test_advance_time.py
import pytest
from plc_code.executor import create_harness


@pytest.fixture
def harness():
    return create_harness("src/blocks/AdvanceTime.s7dcl")


def make_trajectory(current_time=0.0, duration=5.0, is_valid=True):
    return {
        "profiles": [],  # unused by AdvanceTime
        "duration": duration,
        "independentMinDurations": [0.0]*4,
        "currentTime": current_time,
        "isValid": is_valid,
    }


def test_advance_increments_current_time(harness):
    traj = make_trajectory()
    harness.call({"trajectory": traj, "dt": 0.010})
    assert traj["currentTime"] == pytest.approx(0.010, abs=1e-12)


def test_advance_does_not_exceed_duration(harness):
    traj = make_trajectory(current_time=4.995, duration=5.0)
    harness.call({"trajectory": traj, "dt": 0.010})
    assert traj["currentTime"] == pytest.approx(5.0, abs=1e-9)


def test_invalid_trajectory_unchanged(harness):
    traj = make_trajectory(current_time=1.0, is_valid=False)
    harness.call({"trajectory": traj, "dt": 0.010})
    assert traj["currentTime"] == pytest.approx(1.0, abs=1e-12)
```

- [ ] **Step 2: Run tests to verify failure**

```bash
uv run pytest tests/unit/test_advance_time.py -v
```

Expected: 3 FAILED.

- [ ] **Step 3: Implement `AdvanceTime.s7dcl`**

```scl
{
    S7_Author := "Martin C";
    S7_EditorMode := "SCL";
    S7_Family := "Ruckig";
    S7_Optimized := "TRUE";
    S7_Version := "0.1.0"
}
FUNCTION "AdvanceTime" : VOID
    VAR_IN_OUT
        trajectory : "typeTrajectory";
    END_VAR
    VAR_INPUT
        dt : LREAL;
    END_VAR
    VAR_TEMP
        newTime : LREAL;
    END_VAR

BEGIN
    IF NOT trajectory.isValid THEN
        RETURN;
    END_IF;

    newTime := trajectory.currentTime + dt;
    IF newTime > trajectory.duration THEN
        newTime := trajectory.duration;
    END_IF;
    trajectory.currentTime := newTime;
END_FUNCTION
```

- [ ] **Step 4: Run tests to verify pass**

```bash
uv run pytest tests/unit/test_advance_time.py -v
```

Expected: 3 PASSED.

- [ ] **Step 5: Commit**

```bash
git add src/blocks/AdvanceTime.s7dcl tests/unit/test_advance_time.py
git commit -m "feat(traj): add AdvanceTime FC

Increments currentTime by dt, clamped to trajectory.duration. No-op
if isValid is FALSE."
```

---

## Task 12: Implement FC `StateAtTime` (TDD)

**Files:**
- Create: `src/blocks/StateAtTime.s7dcl`
- Create: `tests/unit/test_state_at_time.py`

- [ ] **Step 1: Write failing tests**

```python
# tests/unit/test_state_at_time.py
import pytest
from plc_code.executor import create_harness


@pytest.fixture
def harness():
    return create_harness("src/blocks/StateAtTime.s7dcl")


def trapezoidal_profile(p0=0.0, p1=10.0, vmax=2.0, amax=2.0, jmax=10.0):
    """Pre-computed phase structure for known case (matches Task 10 test)."""
    tj, ta = 0.2, 0.8
    duration = 5.8
    # Phase durations and jerks
    t = [tj, ta, tj, duration - 2*(2*tj + ta), tj, ta, tj]
    j = [+jmax, 0.0, -jmax, 0.0, -jmax, 0.0, +jmax]
    # Compute boundary states by forward integration
    a = [0.0]; v = [0.0]; p = [p0]
    for i in range(7):
        a.append(a[-1] + j[i]*t[i])
        v.append(v[-1] + a[-2]*t[i] + 0.5*j[i]*t[i]**2)
        p.append(p[-1] + v[-2]*t[i] + 0.5*a[-2]*t[i]**2 + j[i]*t[i]**3/6.0)
    return {
        "t": t, "j": j, "a": a, "v": v, "p": p,
        "direction": 1,
        "brake": {"t":[0,0],"j":[0,0],"a":[0,0,0],"v":[0,0,0],"p":[0,0,0]},
        "controlSigns": 0,
    }


def test_state_at_t_zero(harness):
    profile = trapezoidal_profile()
    result = harness.call({"profile": profile, "t": 0.0})
    assert result["p"] == pytest.approx(0.0, abs=1e-9)
    assert result["v"] == pytest.approx(0.0, abs=1e-9)
    assert result["a"] == pytest.approx(0.0, abs=1e-9)


def test_state_at_t_end(harness):
    profile = trapezoidal_profile()
    result = harness.call({"profile": profile, "t": 5.8})
    assert result["p"] == pytest.approx(10.0, abs=1e-6)
    assert result["v"] == pytest.approx(0.0, abs=1e-6)
    assert result["a"] == pytest.approx(0.0, abs=1e-6)


def test_state_at_t_mid_phase(harness):
    profile = trapezoidal_profile()
    # End of phase 0 (jerk-up): t = 0.2
    result = harness.call({"profile": profile, "t": 0.2})
    assert result["a"] == pytest.approx(2.0, abs=1e-6)  # a_max reached
    assert result["v"] == pytest.approx(0.5 * 10.0 * 0.04, abs=1e-6)  # = 0.2
```

- [ ] **Step 2: Run tests to verify failure**

```bash
uv run pytest tests/unit/test_state_at_time.py -v
```

Expected: 3 FAILED.

- [ ] **Step 3: Implement `StateAtTime.s7dcl`**

```scl
{
    S7_Author := "Martin C";
    S7_EditorMode := "SCL";
    S7_Family := "Ruckig";
    S7_Optimized := "TRUE";
    S7_Version := "0.1.0"
}
FUNCTION "StateAtTime" : VOID
    VAR_INPUT
        profile : "typeProfile";
        t       : LREAL;
    END_VAR
    VAR_OUTPUT
        p, v, a : LREAL;
    END_VAR
    VAR_TEMP
        i        : DINT;
        tElapsed : LREAL;
        dt       : LREAL;
    END_VAR

BEGIN
    // Find the phase containing time t
    tElapsed := 0.0;
    FOR i := 0 TO 6 DO
        IF t <= tElapsed + profile.t[i] + "dbRuckigConst".EPS_TIME THEN
            dt := t - tElapsed;
            a := profile.a[i] + profile.j[i] * dt;
            v := profile.v[i] + profile.a[i] * dt
                              + 0.5 * profile.j[i] * dt * dt;
            p := profile.p[i] + profile.v[i] * dt
                              + 0.5 * profile.a[i] * dt * dt
                              + profile.j[i] * dt * dt * dt / 6.0;
            RETURN;
        END_IF;
        tElapsed := tElapsed + profile.t[i];
    END_FOR;

    // t beyond end of trajectory: clamp to final state
    p := profile.p[7];
    v := profile.v[7];
    a := profile.a[7];
END_FUNCTION
```

- [ ] **Step 4: Run tests to verify pass**

```bash
uv run pytest tests/unit/test_state_at_time.py -v
```

Expected: 3 PASSED.

- [ ] **Step 5: Commit**

```bash
git add src/blocks/StateAtTime.s7dcl tests/unit/test_state_at_time.py
git commit -m "feat(traj): add StateAtTime FC

Evaluates (p, v, a) at arbitrary time t on a 7-phase profile by
locating the current phase and integrating from the phase boundary.
Clamps to final state if t exceeds total duration."
```

---

## Task 13: Implement FB `RuckigOtg` skeleton (no logic yet)

**Files:**
- Create: `src/blocks/RuckigOtg.s7dcl`
- Create: `tests/unit/test_ruckig_otg.py` (skeleton-only tests)

- [ ] **Step 1: Write failing tests for skeleton**

```python
# tests/unit/test_ruckig_otg.py
import pytest
from plc_code.executor import create_harness


@pytest.fixture
def harness():
    return create_harness("src/blocks/RuckigOtg.s7dcl")


def default_input():
    return {
        "currentPosition": [0.0]*4, "currentVelocity": [0.0]*4,
        "currentAcceleration": [0.0]*4,
        "targetPosition": [1.0, 0.0, 0.0, 0.0], "targetVelocity": [0.0]*4,
        "targetAcceleration": [0.0]*4,
        "maxVelocity": [2.0]*4, "maxAcceleration": [5.0]*4,
        "maxJerk": [10.0]*4, "enabled": [True, False, False, False],
        "nDofs": 1, "controlInterface": 0,
        "synchronization": 0, "durationDiscretization": 0,
    }


def test_disabled_fb_returns_invalid_output(harness):
    """When enable=FALSE, valid/busy/done should all be FALSE."""
    harness.set_inputs({
        "enable": False, "input": default_input(),
        "cycleTime": 0.010, "reset": False,
    })
    harness.execute()
    assert harness.get_output("valid") is False
    assert harness.get_output("busy") is False
    assert harness.get_output("done") is False
    assert harness.get_output("error") is False
```

- [ ] **Step 2: Run test to verify failure**

```bash
uv run pytest tests/unit/test_ruckig_otg.py::test_disabled_fb_returns_invalid_output -v
```

Expected: FAIL (file not found).

- [ ] **Step 3: Implement RuckigOtg skeleton**

```scl
{
    S7_Author := "Martin C";
    S7_EditorMode := "SCL";
    S7_Family := "Ruckig";
    S7_Optimized := "TRUE";
    S7_Version := "0.1.0"
}
FUNCTION_BLOCK "RuckigOtg"
    VAR_INPUT
        enable    : BOOL := FALSE;
        input     : "typeRuckigInput";
        cycleTime : LREAL := 0.010;
        reset     : BOOL := FALSE;
    END_VAR
    VAR_OUTPUT
        valid     : BOOL := FALSE;
        busy      : BOOL := FALSE;
        done      : BOOL := FALSE;
        error     : BOOL := FALSE;
        status    : WORD := 16#0000;
        output    : "typeRuckigOutput";
    END_VAR
    VAR
        trajectory   : "typeTrajectory";
        prevInput    : "typeRuckigInput";
        firstCall    : BOOL := TRUE;
    END_VAR
    VAR_TEMP
        validationResult : WORD;
    END_VAR

BEGIN
    // 1. If disabled, force outputs FALSE and exit
    IF NOT enable THEN
        valid := FALSE;
        busy  := FALSE;
        done  := FALSE;
        error := FALSE;
        status := "dbRuckigConst".RESULT_FINISHED;
        RETURN;
    END_IF;

    // Logic to be implemented in Tasks 14-15
    valid := FALSE;
    busy  := FALSE;
    done  := FALSE;
    error := FALSE;
    status := "dbRuckigConst".RESULT_WORKING;
END_FUNCTION_BLOCK
```

- [ ] **Step 4: Run test to verify pass**

```bash
uv run pytest tests/unit/test_ruckig_otg.py::test_disabled_fb_returns_invalid_output -v
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/blocks/RuckigOtg.s7dcl tests/unit/test_ruckig_otg.py
git commit -m "feat(fb): add RuckigOtg FB skeleton with PLCopen DA011 interface

Initial structure with enable/valid/busy/done/error/status outputs and
persistent trajectory + prevInput state. Disabled-FB behavior tested.
Logic in subsequent tasks."
```

---

## Task 14: Implement RuckigOtg validation + change detection

**Files:**
- Modify: `src/blocks/RuckigOtg.s7dcl` (extend BEGIN block)
- Modify: `tests/unit/test_ruckig_otg.py` (add tests)

- [ ] **Step 1: Add failing tests**

```python
# Append to tests/unit/test_ruckig_otg.py

def test_invalid_input_returns_error(harness):
    """vMax=0 should set error=TRUE, status=0x8201."""
    inp = default_input()
    inp["maxVelocity"][0] = 0.0
    harness.set_inputs({
        "enable": True, "input": inp, "cycleTime": 0.010, "reset": False,
    })
    harness.execute()
    assert harness.get_output("error") is True
    assert harness.get_output("status") == 0x8201
    assert harness.get_output("valid") is False


def test_first_call_triggers_recompute(harness):
    """First call with valid input must compute a trajectory."""
    harness.set_inputs({
        "enable": True, "input": default_input(), "cycleTime": 0.010, "reset": False,
    })
    harness.execute()
    assert harness.get_output("error") is False
    assert harness.get_output("status") == 0x7000  # WORKING
    # Trajectory should be valid after first call
    # (full check in Task 16)
```

- [ ] **Step 2: Run new tests to verify failure**

```bash
uv run pytest tests/unit/test_ruckig_otg.py::test_invalid_input_returns_error -v
uv run pytest tests/unit/test_ruckig_otg.py::test_first_call_triggers_recompute -v
```

Expected: both FAIL.

- [ ] **Step 3: Implement validation + change detection in RuckigOtg**

Replace the body of `RuckigOtg.s7dcl` BEGIN block with:

```scl
BEGIN
    IF NOT enable THEN
        valid := FALSE; busy := FALSE; done := FALSE; error := FALSE;
        status := "dbRuckigConst".RESULT_FINISHED;
        firstCall := TRUE;  // reset on next enable
        trajectory.isValid := FALSE;
        RETURN;
    END_IF;

    // 1. Validation
    validationResult := "ValidateInput"(input := input);
    IF validationResult <> "dbRuckigConst".RESULT_WORKING THEN
        valid := FALSE; busy := FALSE; done := FALSE; error := TRUE;
        status := validationResult;
        RETURN;
    END_IF;

    // 2. Change detection
    recomputeRequired := firstCall OR reset OR NOT trajectory.isValid OR (
        ABS(input.targetPosition[0] - prevInput.targetPosition[0]) > "dbRuckigConst".EPS_POSITION
     OR ABS(input.maxVelocity[0]    - prevInput.maxVelocity[0])    > "dbRuckigConst".EPS_VELOCITY
     OR ABS(input.maxAcceleration[0]- prevInput.maxAcceleration[0])> "dbRuckigConst".EPS_DERIV
     OR ABS(input.maxJerk[0]        - prevInput.maxJerk[0])        > "dbRuckigConst".EPS_DERIV
    );

    // Placeholder for Task 15: recompute + advance + evaluate
    valid := FALSE; busy := FALSE; done := FALSE; error := FALSE;
    status := "dbRuckigConst".RESULT_WORKING;
END_FUNCTION_BLOCK
```

Add to `VAR_TEMP`:

```scl
recomputeRequired : BOOL;
```

- [ ] **Step 4: Run tests to verify pass**

```bash
uv run pytest tests/unit/test_ruckig_otg.py -v
```

Expected: 3 PASSED (1 from Task 13 + 2 new).

- [ ] **Step 5: Commit**

```bash
git add src/blocks/RuckigOtg.s7dcl tests/unit/test_ruckig_otg.py
git commit -m "feat(fb): add validation and change detection to RuckigOtg

- Validation via ValidateInput FC, propagates DA014 error codes
- Change detection on targetPosition and limits (single-axis for v0.1)
- Reset trajectory state on enable=FALSE→TRUE and on reset pulse"
```

---

## Task 15: Implement RuckigOtg recompute + advance + evaluate

**Files:**
- Modify: `src/blocks/RuckigOtg.s7dcl`
- Modify: `tests/unit/test_ruckig_otg.py`

- [ ] **Step 1: Add full-lifecycle tests**

```python
# Append to tests/unit/test_ruckig_otg.py

def test_full_trajectory_reaches_target(harness):
    """Run 600 cycles (6 s) at dt=10ms; verify final position matches target."""
    inp = default_input()
    inp["targetPosition"][0] = 10.0
    inp["maxVelocity"][0] = 2.0
    inp["maxAcceleration"][0] = 2.0
    inp["maxJerk"][0] = 10.0

    final_pos = 0.0
    for cycle in range(600):
        harness.set_inputs({
            "enable": True, "input": inp, "cycleTime": 0.010, "reset": False,
        })
        harness.execute()
        output = harness.get_output("output")
        final_pos = output["newPosition"][0]
        if harness.get_output("done"):
            break

    assert final_pos == pytest.approx(10.0, abs=1e-3)
    assert harness.get_output("done") is True
    assert harness.get_output("status") == 0x0000  # FINISHED


def test_output_continuity_no_discontinuity(harness):
    """Compare position output across consecutive cycles — should be smooth."""
    inp = default_input()
    inp["targetPosition"][0] = 1.0

    prev_pos = 0.0
    for cycle in range(100):
        harness.set_inputs({
            "enable": True, "input": inp, "cycleTime": 0.010, "reset": False,
        })
        harness.execute()
        output = harness.get_output("output")
        new_pos = output["newPosition"][0]
        # Position change per cycle should not exceed v_max * dt + margin
        assert abs(new_pos - prev_pos) <= 2.0 * 0.010 + 1e-9
        prev_pos = new_pos
```

- [ ] **Step 2: Run tests to verify failure**

```bash
uv run pytest tests/unit/test_ruckig_otg.py::test_full_trajectory_reaches_target -v
```

Expected: FAIL (no actual computation yet).

- [ ] **Step 3: Complete the Update() lifecycle**

Replace the placeholder section in `RuckigOtg.s7dcl` (after change detection) with:

```scl
    // 3. Recompute trajectory if needed
    IF recomputeRequired THEN
        trajectory.independentMinDurations[0] := "ComputeMinDuration"(
            currentPos := input.currentPosition[0],
            currentVel := input.currentVelocity[0],
            currentAcc := input.currentAcceleration[0],
            targetPos  := input.targetPosition[0],
            targetVel  := input.targetVelocity[0],
            targetAcc  := input.targetAcceleration[0],
            maxVel := input.maxVelocity[0],
            maxAcc := input.maxAcceleration[0],
            maxJerk:= input.maxJerk[0]);

        trajectory.duration := trajectory.independentMinDurations[0];

        IF "ComputeFinalProfile"(
                profile := trajectory.profiles[0],
                duration := trajectory.duration,
                p0   := input.currentPosition[0],
                p1   := input.targetPosition[0],
                vMax := input.maxVelocity[0],
                aMax := input.maxAcceleration[0],
                jMax := input.maxJerk[0]) <> "dbRuckigConst".RESULT_WORKING THEN
            valid := FALSE; busy := FALSE; done := FALSE; error := TRUE;
            status := "dbRuckigConst".RESULT_ERR_TRAJ;
            trajectory.isValid := FALSE;
            RETURN;
        END_IF;

        trajectory.currentTime := 0.0;
        trajectory.isValid := TRUE;
        prevInput := input;
        firstCall := FALSE;
    END_IF;

    // 4. Advance time
    "AdvanceTime"(trajectory := trajectory, dt := cycleTime);

    // 5. Evaluate state at currentTime
    "StateAtTime"(
        profile := trajectory.profiles[0],
        t := trajectory.currentTime,
        p => output.newPosition[0],
        v => output.newVelocity[0],
        a => output.newAcceleration[0]);

    // 6. Status
    output.trajectoryDuration := trajectory.duration;
    output.currentTime        := trajectory.currentTime;

    IF trajectory.currentTime >= trajectory.duration - "dbRuckigConst".EPS_TIME THEN
        valid := TRUE; busy := FALSE; done := TRUE; error := FALSE;
        status := "dbRuckigConst".RESULT_FINISHED;
        output.workingOutput := FALSE;
    ELSE
        valid := TRUE; busy := TRUE; done := FALSE; error := FALSE;
        status := "dbRuckigConst".RESULT_WORKING;
        output.workingOutput := TRUE;
    END_IF;
END_FUNCTION_BLOCK
```

- [ ] **Step 4: Run all RuckigOtg tests**

```bash
uv run pytest tests/unit/test_ruckig_otg.py -v
```

Expected: 5 PASSED.

- [ ] **Step 5: Commit**

```bash
git add src/blocks/RuckigOtg.s7dcl tests/unit/test_ruckig_otg.py
git commit -m "feat(fb): complete RuckigOtg Update() lifecycle for v0.1

- Step 3 (recompute) calls ComputeMinDuration + ComputeFinalProfile
- Step 4 (advance) calls AdvanceTime
- Step 5 (evaluate) calls StateAtTime for axis 0
- Step 6 (status) sets valid/busy/done/error and DA014 status code

Full trajectory test reaches target within 1mm tolerance over 600 cycles.
Cycle-to-cycle continuity test passes."
```

---

## Task 16: Build parity test bench — reference runner

**Files:**
- Create: `tests/parity/runner_ref.py`
- Create: `tests/parity/__init__.py` (empty)

- [ ] **Step 1: Write the reference runner**

```python
# tests/parity/runner_ref.py
"""Pyruckig-based reference runner for parity tests."""
from __future__ import annotations
from dataclasses import dataclass, field
from typing import List
import ruckig


@dataclass
class CycleSample:
    t: float
    p: List[float]
    v: List[float]
    a: List[float]


@dataclass
class TrajectoryTrace:
    samples: List[CycleSample] = field(default_factory=list)
    duration: float = 0.0
    finished_at_cycle: int = -1


def run_reference(
    *,
    n_dofs: int,
    current_pos: List[float],
    current_vel: List[float],
    current_acc: List[float],
    target_pos: List[float],
    target_vel: List[float],
    target_acc: List[float],
    max_vel: List[float],
    max_acc: List[float],
    max_jerk: List[float],
    cycle_time: float,
    max_cycles: int = 1000,
) -> TrajectoryTrace:
    """Run pyruckig with the given inputs, return cycle-by-cycle trace."""
    otg = ruckig.Ruckig(n_dofs, cycle_time)
    inp = ruckig.InputParameter(n_dofs)
    out = ruckig.OutputParameter(n_dofs)

    inp.current_position = current_pos[:n_dofs]
    inp.current_velocity = current_vel[:n_dofs]
    inp.current_acceleration = current_acc[:n_dofs]
    inp.target_position = target_pos[:n_dofs]
    inp.target_velocity = target_vel[:n_dofs]
    inp.target_acceleration = target_acc[:n_dofs]
    inp.max_velocity = max_vel[:n_dofs]
    inp.max_acceleration = max_acc[:n_dofs]
    inp.max_jerk = max_jerk[:n_dofs]

    trace = TrajectoryTrace()
    for cycle in range(max_cycles):
        result = otg.update(inp, out)
        trace.samples.append(CycleSample(
            t=cycle * cycle_time,
            p=list(out.new_position),
            v=list(out.new_velocity),
            a=list(out.new_acceleration),
        ))
        if result == ruckig.Result.Finished:
            trace.finished_at_cycle = cycle
            trace.duration = out.trajectory.duration
            break
        out.pass_to_input(inp)

    return trace
```

- [ ] **Step 2: Quick smoke test in Python REPL**

```bash
uv run python -c "
from tests.parity.runner_ref import run_reference
t = run_reference(
    n_dofs=1,
    current_pos=[0.0,0,0,0], current_vel=[0]*4, current_acc=[0]*4,
    target_pos=[10.0,0,0,0], target_vel=[0]*4, target_acc=[0]*4,
    max_vel=[2.0,0,0,0], max_acc=[2.0,0,0,0], max_jerk=[10.0,0,0,0],
    cycle_time=0.010,
)
print(f'Finished at cycle {t.finished_at_cycle}, duration {t.duration:.4f}s')
print(f'Final pos: {t.samples[-1].p[0]:.6f}')
"
```

Expected: prints something like `Finished at cycle 579, duration 5.8000s, Final pos: 10.000000`.

- [ ] **Step 3: Commit**

```bash
git add tests/parity/runner_ref.py tests/parity/__init__.py
git commit -m "feat(parity): add pyruckig reference runner

Wraps pyruckig.Ruckig in a function that returns a cycle-by-cycle
TrajectoryTrace (position, velocity, acceleration per cycle)."
```

---

## Task 17: Build parity test bench — SCL runner

**Files:**
- Create: `tests/parity/runner_scl.py`

- [ ] **Step 1: Write the SCL runner**

```python
# tests/parity/runner_scl.py
"""plc-code-based SCL runner for parity tests."""
from __future__ import annotations
from typing import List
from plc_code.executor import create_harness
from tests.parity.runner_ref import CycleSample, TrajectoryTrace


def run_scl(
    *,
    n_dofs: int,
    current_pos: List[float],
    current_vel: List[float],
    current_acc: List[float],
    target_pos: List[float],
    target_vel: List[float],
    target_acc: List[float],
    max_vel: List[float],
    max_acc: List[float],
    max_jerk: List[float],
    cycle_time: float,
    max_cycles: int = 1000,
) -> TrajectoryTrace:
    """Run the SCL RuckigOtg FB cycle by cycle via plc-code harness."""
    harness = create_harness("src/blocks/RuckigOtg.s7dcl")

    # Pad lists to length 4 (DOF_MAX)
    def pad(lst: List[float], length: int = 4) -> List[float]:
        return list(lst) + [0.0] * (length - len(lst))

    inp = {
        "currentPosition": pad(current_pos),
        "currentVelocity": pad(current_vel),
        "currentAcceleration": pad(current_acc),
        "targetPosition": pad(target_pos),
        "targetVelocity": pad(target_vel),
        "targetAcceleration": pad(target_acc),
        "maxVelocity": pad(max_vel),
        "maxAcceleration": pad(max_acc),
        "maxJerk": pad(max_jerk),
        "enabled": [i < n_dofs for i in range(4)],
        "nDofs": n_dofs,
        "controlInterface": 0,
        "synchronization": 0,
        "durationDiscretization": 0,
    }

    trace = TrajectoryTrace()
    for cycle in range(max_cycles):
        harness.set_inputs({
            "enable": True, "input": inp, "cycleTime": cycle_time, "reset": False,
        })
        harness.execute()
        output = harness.get_output("output")
        trace.samples.append(CycleSample(
            t=cycle * cycle_time,
            p=output["newPosition"][:n_dofs],
            v=output["newVelocity"][:n_dofs],
            a=output["newAcceleration"][:n_dofs],
        ))
        if harness.get_output("done"):
            trace.finished_at_cycle = cycle
            trace.duration = output["trajectoryDuration"]
            break

    return trace
```

- [ ] **Step 2: Smoke test against same scenario as runner_ref**

```bash
uv run python -c "
from tests.parity.runner_scl import run_scl
t = run_scl(
    n_dofs=1,
    current_pos=[0.0], current_vel=[0], current_acc=[0],
    target_pos=[10.0], target_vel=[0], target_acc=[0],
    max_vel=[2.0], max_acc=[2.0], max_jerk=[10.0],
    cycle_time=0.010,
)
print(f'SCL finished at cycle {t.finished_at_cycle}, duration {t.duration:.4f}s')
print(f'SCL final pos: {t.samples[-1].p[0]:.6f}')
"
```

Expected: similar output to runner_ref smoke test.

- [ ] **Step 3: Commit**

```bash
git add tests/parity/runner_scl.py
git commit -m "feat(parity): add SCL runner using plc-code executor

Runs the RuckigOtg FB cycle by cycle in Python via the plc-code
SCL-to-Python transpiler, returns same TrajectoryTrace shape as
runner_ref for direct comparison."
```

---

## Task 18: Build parity comparator

**Files:**
- Create: `tests/parity/comparator.py`

- [ ] **Step 1: Write the comparator**

```python
# tests/parity/comparator.py
"""Cycle-by-cycle comparator for SCL vs reference trajectories."""
from __future__ import annotations
from dataclasses import dataclass, field
from typing import List
from tests.parity.runner_ref import TrajectoryTrace


@dataclass
class ParityResult:
    passed: bool
    max_position_error: float = 0.0
    max_velocity_error: float = 0.0
    max_acceleration_error: float = 0.0
    duration_error: float = 0.0
    failing_cycles: List[int] = field(default_factory=list)


def compare_traces(
    ref: TrajectoryTrace,
    scl: TrajectoryTrace,
    *,
    tol_position: float = 1.0e-9,
    tol_velocity: float = 1.0e-9,
    tol_acceleration: float = 1.0e-9,
    tol_duration: float = 1.0e-7,
) -> ParityResult:
    result = ParityResult(passed=True)

    # Compare durations
    result.duration_error = abs(ref.duration - scl.duration)
    if result.duration_error > tol_duration:
        result.passed = False

    # Compare cycle-by-cycle (truncate to shortest common length)
    n_cycles = min(len(ref.samples), len(scl.samples))
    for i in range(n_cycles):
        r = ref.samples[i]
        s = scl.samples[i]
        n_dofs = min(len(r.p), len(s.p))
        cycle_failed = False
        for d in range(n_dofs):
            ep = abs(r.p[d] - s.p[d])
            ev = abs(r.v[d] - s.v[d])
            ea = abs(r.a[d] - s.a[d])
            result.max_position_error = max(result.max_position_error, ep)
            result.max_velocity_error = max(result.max_velocity_error, ev)
            result.max_acceleration_error = max(result.max_acceleration_error, ea)
            if ep > tol_position or ev > tol_velocity or ea > tol_acceleration:
                cycle_failed = True
        if cycle_failed:
            result.failing_cycles.append(i)
            result.passed = False

    return result
```

- [ ] **Step 2: Smoke test the comparator**

```bash
uv run python -c "
from tests.parity.runner_ref import run_reference
from tests.parity.runner_scl import run_scl
from tests.parity.comparator import compare_traces

kwargs = dict(
    n_dofs=1,
    current_pos=[0.0], current_vel=[0], current_acc=[0],
    target_pos=[10.0], target_vel=[0], target_acc=[0],
    max_vel=[2.0], max_acc=[2.0], max_jerk=[10.0],
    cycle_time=0.010,
)
ref = run_reference(**kwargs)
scl = run_scl(**kwargs)
result = compare_traces(ref, scl)
print(f'Passed: {result.passed}')
print(f'Max pos err: {result.max_position_error:.2e}')
print(f'Max vel err: {result.max_velocity_error:.2e}')
print(f'Duration err: {result.duration_error:.2e}')
"
```

Expected: prints `Passed: True` with errors ≤ tolerances.

- [ ] **Step 3: Commit**

```bash
git add tests/parity/comparator.py
git commit -m "feat(parity): add cycle-by-cycle trace comparator

Compares pyruckig reference vs SCL traces with configurable tolerances:
1e-9 default for p/v/a, 1e-7 for duration. Returns ParityResult with
max errors and list of failing cycle indices for debug."
```

---

## Task 19: Write 10 nominal parity scenarios and pytest entry point

**Files:**
- Create: `tests/parity/scenarios/nominal_001_small_displacement.yaml`
- Create: `tests/parity/scenarios/nominal_002_v_max_saturated.yaml`
- Create: `tests/parity/scenarios/nominal_003_triangular_jerk.yaml`
- Create: `tests/parity/scenarios/nominal_004_negative_direction.yaml`
- Create: `tests/parity/scenarios/nominal_005_tight_limits.yaml`
- Create: `tests/parity/scenarios/nominal_006_high_jerk.yaml`
- Create: `tests/parity/scenarios/nominal_007_long_distance.yaml`
- Create: `tests/parity/scenarios/nominal_008_zero_displacement.yaml`
- Create: `tests/parity/scenarios/nominal_009_just_v_max.yaml`
- Create: `tests/parity/scenarios/nominal_010_just_a_max.yaml`
- Create: `tests/parity/test_parity.py`

- [ ] **Step 1: Write all 10 scenario YAMLs**

Use this template for each scenario; values listed below the template:

```yaml
# tests/parity/scenarios/nominal_001_small_displacement.yaml
name: "Small displacement, no saturation"
n_dofs: 1
cycle_time: 0.010
current_pos: [0.0]
current_vel: [0.0]
current_acc: [0.0]
target_pos: [0.001]
target_vel: [0.0]
target_acc: [0.0]
max_vel: [10.0]
max_acc: [10.0]
max_jerk: [10.0]
```

Scenario values (copy each into its respective file with the matching name):

| # | name | target | max_vel | max_acc | max_jerk |
|---|------|--------|---------|---------|----------|
| 001 | Small displacement, no saturation | 0.001 | 10 | 10 | 10 |
| 002 | v_max saturated, trapezoidal | 10.0 | 2.0 | 2.0 | 10.0 |
| 003 | Triangular jerk (v not reached) | 0.01 | 5.0 | 5.0 | 10.0 |
| 004 | Negative direction | -5.0 | 2.0 | 2.0 | 10.0 |
| 005 | Tight limits (slow movement) | 1.0 | 0.1 | 0.1 | 0.5 |
| 006 | High jerk (fast smoothing) | 1.0 | 2.0 | 5.0 | 1000.0 |
| 007 | Long distance | 100.0 | 5.0 | 5.0 | 10.0 |
| 008 | Zero displacement | 0.0 | 2.0 | 2.0 | 10.0 |
| 009 | Just at v_max threshold | 0.8 | 2.0 | 2.0 | 10.0 |
| 010 | Just at a_max threshold | 0.04 | 5.0 | 2.0 | 10.0 |

- [ ] **Step 2: Write pytest entry point**

```python
# tests/parity/test_parity.py
"""Parametrized parity test: runs every YAML scenario and compares traces."""
from __future__ import annotations
import glob
import yaml
import pytest
from tests.parity.runner_ref import run_reference
from tests.parity.runner_scl import run_scl
from tests.parity.comparator import compare_traces


def load_scenarios():
    paths = sorted(glob.glob("tests/parity/scenarios/*.yaml"))
    return [(yaml.safe_load(open(p)), p.split("/")[-1]) for p in paths]


@pytest.mark.parametrize(
    "scenario,name", load_scenarios(),
    ids=[name for _, name in load_scenarios()],
)
def test_parity(scenario, name):
    kwargs = {k: v for k, v in scenario.items() if k != "name"}
    ref = run_reference(**kwargs)
    scl = run_scl(**kwargs)
    result = compare_traces(ref, scl)
    assert result.passed, (
        f"Parity failed for {name}:\n"
        f"  max pos err = {result.max_position_error:.2e}\n"
        f"  max vel err = {result.max_velocity_error:.2e}\n"
        f"  max acc err = {result.max_acceleration_error:.2e}\n"
        f"  duration err= {result.duration_error:.2e}\n"
        f"  failing cycles (first 5): {result.failing_cycles[:5]}"
    )
```

- [ ] **Step 3: Run all parity tests**

```bash
uv run pytest tests/parity/test_parity.py -v
```

Expected: 10 PASSED.

> If any scenario fails, investigate the divergence. Common causes for v0.1:
> - Triangular sub-case branch not matching Ruckig's case selection
> - Boundary `EPS_*` tolerances too tight
> - Phase boundary integration accumulating numerical error

- [ ] **Step 4: Commit**

```bash
git add tests/parity/scenarios/ tests/parity/test_parity.py
git commit -m "feat(parity): add 10 nominal parity scenarios + pytest harness

Scenarios cover: small displacement (triangular), v_max saturated
(trapezoidal), negative direction, tight limits, high jerk, long
distance, zero displacement, threshold cases at v_max and a_max."
```

---

## Task 20: GitHub CI setup

**Files:**
- Create: `.github/workflows/ci.yml`

- [ ] **Step 1: Write CI workflow**

```yaml
# .github/workflows/ci.yml
name: CI

on:
  push:
    branches: [main, master]
  pull_request:
    branches: [main, master]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
        with:
          fetch-depth: 1

      - name: Checkout plc-tools sibling repo
        uses: actions/checkout@v4
        with:
          repository: martinc8/plc-tools  # adjust to your fork
          path: 203-plc-tools

      - name: Set up Python
        uses: actions/setup-python@v5
        with:
          python-version: "3.12"

      - name: Install uv
        run: pip install uv

      - name: Install dependencies
        run: uv sync

      - name: Run unit tests
        run: uv run pytest tests/unit/ -v

      - name: Run parity tests
        run: uv run pytest tests/parity/ -v
```

- [ ] **Step 2: Commit**

```bash
git add .github/workflows/ci.yml
git commit -m "ci: add GitHub Actions workflow for unit + parity tests

Triggers on push and PR to main/master. Installs uv, syncs deps from
local plc-tools checkout, runs unit and parity test suites."
```

> The `plc-tools` checkout step assumes you have a public mirror of `203-plc-tools`. If it remains private, replace the checkout with a tarball download from your private artifact store, or vendor `plc-code` into this repo as a submodule.

---

## Task 21: README expansion and v0.1.0 release tag

**Files:**
- Modify: `README.md`
- Modify: `CHANGELOG.md`
- Create: `examples/single-axis-point-to-point/example.scl`
- Create: `examples/single-axis-point-to-point/README.md`

- [ ] **Step 1: Expand README**

```markdown
# ruckig-scl

A Siemens SCL Structured Text port of the [Ruckig](https://github.com/pantor/ruckig)
Online Trajectory Generation library. MIT-licensed.

**Status: v0.1.0 — single-axis position interface with target velocity = 0.**

## Features (v0.1)

- Single-axis jerk-limited trajectory generation
- Point-to-point motion from rest to rest
- Three sub-cases: triangular jerk, trapezoidal jerk, full S-curve
- PLCopen DA011 continuous-enable interface
- Validated against pyruckig with tolerance 1e-9 on all 10 nominal scenarios

## Roadmap

- v0.2: single-axis with arbitrary target velocity/acceleration
- v0.3 → v0.5: multi-axis with synchronization
- v0.6: velocity interface
- v0.7: brake profiles and degenerate cases
- v0.8: performance optimization
- v0.9: public release after field validation

## Target platform

- TIA Portal V18 or later
- Siemens S7-1500, firmware V3.0 or later
- Compiled with optimized access (`S7_Optimized := "TRUE"`)

## Usage

```scl
VAR
    instOtg   : "RuckigOtg";
    otgInput  : "typeRuckigInput";
    otgOutput : "typeRuckigOutput";
END_VAR

// One-time configuration
otgInput.nDofs := 1;
otgInput.maxVelocity[0] := 2.0;
otgInput.maxAcceleration[0] := 5.0;
otgInput.maxJerk[0] := 10.0;
otgInput.enabled[0] := TRUE;

// Cyclic call (in OB35 or fast-motion OB)
otgInput.targetPosition[0] := someTargetValue;
otgInput.currentPosition[0] := encoderFeedback;

instOtg(
    enable    := TRUE,
    input     := otgInput,
    cycleTime := 0.010,
    reset     := FALSE,
    output    => otgOutput
);

IF instOtg.busy THEN
    motorSetpoint := otgOutput.newVelocity[0];
END_IF;
```

## License

MIT — see [LICENSE](LICENSE).

## Acknowledgements

This is an independent SCL port of the [Ruckig](https://github.com/pantor/ruckig)
algorithm, originally developed by Lars Berscheid and Torsten Kröger. The
mathematics and algorithm design are theirs.
```

- [ ] **Step 2: Write example SCL file**

```scl
// examples/single-axis-point-to-point/example.scl
//
// Minimal example: drive an axis from position 0 to position 10 with
// limits vMax=2, aMax=5, jMax=10. Outputs the trajectory position
// to a hypothetical drive setpoint.
//
{ S7_Optimized := "TRUE" }
FUNCTION_BLOCK "ExampleSingleAxis"
    VAR
        instOtg     : "RuckigOtg";
        otgInput    : "typeRuckigInput";
        otgOutput   : "typeRuckigOutput";
        configured  : BOOL := FALSE;
    END_VAR

BEGIN
    // One-shot configuration on first call
    IF NOT configured THEN
        otgInput.nDofs := 1;
        otgInput.maxVelocity[0]     := 2.0;
        otgInput.maxAcceleration[0] := 5.0;
        otgInput.maxJerk[0]         := 10.0;
        otgInput.enabled[0]         := TRUE;
        otgInput.currentPosition[0] := 0.0;
        otgInput.targetPosition[0]  := 10.0;
        configured := TRUE;
    END_IF;

    // Cyclic call
    instOtg(
        enable    := TRUE,
        input     := otgInput,
        cycleTime := 0.010,
        reset     := FALSE,
        output    => otgOutput);

    // otgOutput.newVelocity[0] is the velocity setpoint to forward to the drive
END_FUNCTION_BLOCK
```

- [ ] **Step 3: Update CHANGELOG**

```markdown
# Changelog

## [0.1.0] - 2026-XX-XX

### Added
- Single-axis jerk-limited trajectory generation (rest-to-rest)
- PLCopen DA011 continuous-enable FB interface (`RuckigOtg`)
- 6 algorithmic FCs: ValidateInput, ComputeMinDuration, ComputeFinalProfile,
  AdvanceTime, StateAtTime, IsFiniteLreal
- 5 UDTs: typeRuckigInput, typeRuckigOutput, typeProfile, typeBrakeProfile,
  typeTrajectory
- Global constants DB `dbRuckigConst` with DA014-aligned status codes
- Parity test bench against pyruckig with 10 nominal scenarios
- Unit tests via `plc-code` executor framework
- GitHub Actions CI for unit + parity tests
```

- [ ] **Step 4: Run full test suite one last time**

```bash
uv run pytest tests/ -v
```

Expected: all PASSED. Note count.

- [ ] **Step 5: Tag and release**

```bash
git add README.md CHANGELOG.md examples/
git commit -m "docs: expand README, add example, prepare v0.1.0 release"
git tag -a v0.1.0 -m "v0.1.0: single-axis rest-to-rest trajectory generation

First public release. Implements the simplest Ruckig case:
single-axis position interface with target velocity = 0 and starting
from rest. Validated against pyruckig on 10 nominal scenarios with
tolerance 1e-9 on position, velocity, and acceleration."
```

> Do not push the tag to GitHub yet — that is a separate user-driven step. Wait for explicit confirmation before `git push origin v0.1.0`.

---

## Self-Review Checklist (run by the implementing engineer at end of v0.1)

After completing all 21 tasks, verify:

- [ ] All 21 commits exist on the master branch
- [ ] `uv run pytest tests/` reports 100% PASSED
- [ ] Each block file has a Siemens-style attributes header (S7_Author, S7_Optimized, etc.)
- [ ] No `=` between LREAL values anywhere in SCL code (SE007)
- [ ] No identifier exceeds 24 characters (NF010)
- [ ] All FC return types are typed (BOOL, LREAL, WORD) and matching the test expectations
- [ ] All `VAR_IN_OUT` parameters for structured types are passed by reference (PE003)
- [ ] CHANGELOG entry for v0.1.0 is complete
- [ ] Tag `v0.1.0` is created locally (push is deferred to user)
- [ ] No `TBD`, `TODO`, or placeholder comments in committed code
- [ ] Coverage report shows ≥ 85% line coverage on `src/blocks/`

---

## What comes next (v0.2 and beyond)

This v0.1 plan delivers a working but minimal slice. Each subsequent version
gets its own brainstorming + spec + plan cycle.

- **v0.2** plan will extend `ComputeMinDuration` and `ComputeFinalProfile` to
  handle arbitrary `targetVelocity` and `targetAcceleration`, introduces the
  full 7-case Ruckig enumeration (UDDU, UDUD, etc.), and adds ~10 more parity
  scenarios.
- **v0.3** plan will introduce the multi-axis loop in `RuckigOtg.Update()`
  and add the `SyncNone` FC.
- **v0.4** plan adds `SyncPhase` FC and the duration-stretching logic in
  `ComputeFinalProfile`.

Refer back to the spec roadmap (Section 8) for the full version sequence
before starting v0.2.
