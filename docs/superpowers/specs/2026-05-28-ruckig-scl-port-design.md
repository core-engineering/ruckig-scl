# Ruckig SCL — Design Spec

**Date** : 2026-05-28
**Owner** : Martin C
**Project** : Port of Ruckig (C++ MIT) to Structured Text for Siemens S7-1500
**Status** : Design approved, ready for implementation planning

---

## 1. Context and Motivation

Ruckig is an open-source library (MIT license) implementing **Online Trajectory Generation (OTG)** — time-optimal, jerk-limited, multi-axis trajectories that react instantaneously to changes in target state. It is the de-facto reference in modern robotics for trajectory generation under realistic dynamic constraints (Franka Emika robots, ROS ecosystem, several industrial robotics OEMs).

Today, no production-grade open-source OTG library exists in Siemens SCL. The only known port (Struckig) is GPLv3-licensed, which makes it incompatible with most industrial OEM use cases.

This project ports Ruckig **Community feature set** directly from the C++ MIT source to Siemens SCL S7-1500. The result is a reusable, MIT-licensed library suitable for industrial integration, including for use in projects such as Ruwais MLA tracking.

### Driving needs

- **Tracking of mobile targets** (Marine Loading Arm following a vessel, with Kalman-smoothed setpoint at 100 Hz)
- **Multi-axis coordinated motion** with phase synchronization across 4-DOF systems
- **Arbitrary initial conditions** (`x₀, ẋ₀, ẍ₀ ≠ 0`) for setpoint reconfiguration mid-motion without discontinuity
- **Target velocity / acceleration ≠ 0** to follow moving targets without lag

---

## 2. Goals and Non-Goals

### Goals

- **Functional parity** with Ruckig Community: single-axis + multi-axis, phase/time/per-DoF synchronization, target velocity and acceleration ≠ 0, arbitrary initial conditions, position and velocity interfaces, brake profiles for degenerate cases
- **Production-grade quality**: comprehensive numerical parity test bench against the C++ reference, unit tests on every component, performance benchmarked on PLCSIM Advanced
- **Open-source MIT** library, published on GitHub from day one
- **Idiomatic Siemens SCL** code following Siemens Programming Styleguide V2.1.0 (NF005–NF010, AL001, DA014, SE007, PE003)
- **Reusable across projects** — usable as-is in MLA tracking and any other 1- to 4-DOF motion application

### Non-Goals

- Ruckig Pro features (intermediate waypoints, per-DoF target states, position bounds) — out of scope for this version
- TwinCAT 3 or CodeSys support — Siemens S7-1500 only
- Hardware integration (FK/IK, drive-specific scaling) — those belong in user code consuming this library
- Web-based examples, online playground, or hosted services — code library only

---

## 3. Design Decisions Settled During Brainstorming

| Decision | Choice | Rationale |
|---|---|---|
| Ambition level | Production-ready, reusable library | Long-term value vs single-use POC |
| Scope | Full Ruckig Community parity | Replaceable use of Ruckig in PLC context |
| License | MIT | Industrial usability, alignment with upstream Ruckig |
| Distribution | Open-source GitHub from v0.1 | Community contribution, motivation, visibility |
| Target platform | TIA Portal V18+ / S7-1500 firmware V3.0+ | Modern OOP partial (METHOD, PROPERTY), LREAL performance |
| Porting approach | Hybrid: paper-first + Ruckig C++ as reference + parity test bench | Best trade-off code cleanliness, validation rigor, manageable risk |
| Numerical precision | LREAL throughout | Parity with Ruckig C++ `double`, acceptable performance on FW V3.0+ |
| DOF_MAX | 4 (compile-time constant) | Matches MLA use case; static array sizing required by SCL |
| Enums | Not used — replaced by typed constants in FB or constants DB | Siemens ENUMs require Software Units; Martin's coding style uses constants |
| Object orientation | Multi-instance composition (no inheritance) | Idiomatic Siemens; FB members declared as static instances of other FBs |
| Naming conventions | UpperCamelCase for blocks/UDTs, lowerCamelCase for variables, `type` prefix for UDTs, `inst` prefix for multi-instances, UPPER_SNAKE for constants | Siemens Programming Styleguide NF005-NF010 |

---

## 4. Architecture

### 4.1 Overall layering

Four layers, each with a single clear responsibility:

```
┌───────────────────────────────────────────────────────────────┐
│  FB RuckigOtg  (public API, single point of entry)            │
│                                                                │
│  METHOD Update(InOut input, output) : RESULT                  │
│    1. Validate input                                          │
│    2. Compute trajectory if input changed                     │
│    3. Advance time by cycleTime                               │
│    4. Output current state (p, v, a)                          │
└───────────────────────────────────────────────────────────────┘
                              │
                              ▼  composition by static instances
        ┌─────────────────────┼─────────────────────┐
        ▼                     ▼                     ▼
┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐
│ FB ProfileCalc   │  │ FB MultiAxisSync │  │ FB TrajEvaluator │
│                  │  │                  │  │                  │
│ - MinDuration    │  │ - SyncPhase      │  │ - AdvanceTime    │
│ - FinalProfile   │  │ - SyncTime       │  │ - StateAtTime    │
│ - BrakeHandling  │  │ - SyncNone       │  │                  │
└──────────────────┘  └──────────────────┘  └──────────────────┘
```

### 4.2 Components

**FB `RuckigOtg`** — Public API (user-facing)

```scl
VAR_INPUT
    input        : typeRuckigInput;
    cycleTime    : LREAL;
END_VAR
VAR_OUTPUT
    output       : typeRuckigOutput;
    result       : WORD;             // status code per DA014
END_VAR
VAR (stat, multi-instance composition)
    instValidator: InputValidator;
    instProfile  : ProfileCalc;
    instSync     : MultiAxisSync;
    instEval     : TrajEvaluator;
    trajectory   : typeTrajectory;   // persistent state
    prevInput    : typeRuckigInput;  // for change detection
END_VAR
METHOD Update()  : WORD              // public, cyclic entry point
METHOD Reset()   : VOID              // explicit reset for error recovery
METHOD ValidateInput() : WORD        // exposed for early validation
```

**FB `ProfileCalc`** — Single-axis profile computation

```scl
METHOD ComputeMinDuration(
    currentPos, currentVel, currentAcc,
    targetPos,  targetVel,  targetAcc,
    maxVel, maxAcc, maxJerk : LREAL) : LREAL    // Ruckig Step 1
METHOD ComputeFinalProfile(InOut profile : typeProfile;
                           duration : LREAL) : WORD   // Ruckig Step 2
METHOD ComputeBrake(InOut profile : typeProfile) : WORD  // degenerate cases
```

**FB `MultiAxisSync`** — Multi-axis time synchronization

```scl
METHOD SyncPhase(InOut trajectory : typeTrajectory) : WORD
METHOD SyncTime (InOut trajectory : typeTrajectory) : WORD
METHOD SyncNone (InOut trajectory : typeTrajectory) : WORD
METHOD SyncPerDoF(InOut trajectory : typeTrajectory) : WORD
```

**FB `TrajEvaluator`** — Time advancement and state extraction

```scl
METHOD AdvanceTime(InOut trajectory : typeTrajectory; dt : LREAL) : VOID
METHOD StateAtTime(profile : typeProfile; t : LREAL;
                   OUT p, v, a : LREAL) : VOID
```

**FB `InputValidator`** — Input parameter validation

```scl
METHOD Validate(input : typeRuckigInput) : WORD   // returns RESULT_OK or RESULT_ERR_*
```

### 4.3 Data types (UDTs)

```
typeRuckigInput
    currentPosition       : ARRAY[0..DOF_MAX-1] OF LREAL
    currentVelocity       : ARRAY[0..DOF_MAX-1] OF LREAL
    currentAcceleration   : ARRAY[0..DOF_MAX-1] OF LREAL
    targetPosition        : ARRAY[0..DOF_MAX-1] OF LREAL
    targetVelocity        : ARRAY[0..DOF_MAX-1] OF LREAL
    targetAcceleration    : ARRAY[0..DOF_MAX-1] OF LREAL
    maxVelocity           : ARRAY[0..DOF_MAX-1] OF LREAL
    maxAcceleration       : ARRAY[0..DOF_MAX-1] OF LREAL
    maxJerk               : ARRAY[0..DOF_MAX-1] OF LREAL
    enabled               : ARRAY[0..DOF_MAX-1] OF BOOL
    nDofs                 : DINT
    controlInterface      : INT       // see constants IFACE_*
    synchronization       : INT       // see constants SYNC_*
    durationDiscretization: INT       // see constants DISC_*

typeRuckigOutput
    newPosition           : ARRAY[0..DOF_MAX-1] OF LREAL
    newVelocity           : ARRAY[0..DOF_MAX-1] OF LREAL
    newAcceleration       : ARRAY[0..DOF_MAX-1] OF LREAL
    workingOutput         : BOOL
    trajectoryDuration    : LREAL
    currentTime           : LREAL

typeProfile
    t                     : ARRAY[0..6] OF LREAL    // phase durations
    j                     : ARRAY[0..6] OF LREAL    // jerk per phase
    a                     : ARRAY[0..7] OF LREAL    // boundary accelerations
    v                     : ARRAY[0..7] OF LREAL    // boundary velocities
    p                     : ARRAY[0..7] OF LREAL    // boundary positions
    direction             : INT
    brake                 : typeBrakeProfile
    controlSigns          : INT       // case ID (UDDU, UDUD, etc.)

typeBrakeProfile
    t                     : ARRAY[0..1] OF LREAL
    j                     : ARRAY[0..1] OF LREAL
    a, v, p               : ARRAY[0..2] OF LREAL

typeTrajectory
    profiles              : ARRAY[0..DOF_MAX-1] OF typeProfile
    duration              : LREAL
    independentMinDurations : ARRAY[0..DOF_MAX-1] OF LREAL
    currentTime           : LREAL
    isValid               : BOOL
```

### 4.4 Constants

Declared either in a global constants DB (`dbRuckigConst`) or within the `RuckigOtg` FB's `CONSTANT` section.

```scl
CONSTANT
    DOF_MAX           : DINT  := 4;
    EPS_TIME          : LREAL := 1.0E-9;
    EPS_VELOCITY      : LREAL := 1.0E-12;
    EPS_POSITION      : LREAL := 1.0E-12;
    EPS_DERIV         : LREAL := 1.0E-8;
    PI                : LREAL := 3.141592653589793;

    // Status codes (DA014 convention)
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

    // Synchronization modes
    SYNC_NONE         : INT := 0;
    SYNC_PHASE        : INT := 1;
    SYNC_TIME         : INT := 2;
    SYNC_PER_DOF      : INT := 3;

    // Control interface
    IFACE_POSITION    : INT := 0;
    IFACE_VELOCITY    : INT := 1;

    // Duration discretization
    DISC_CONTINUOUS   : INT := 0;
    DISC_DISCRETE     : INT := 1;
END_CONSTANT
```

---

## 5. Update() Lifecycle

Six ordered steps per cycle:

1. **Validate input** — `instValidator.Validate()`. On failure, output unchanged, `result := RESULT_ERR_*`, return early.
2. **Change detection** — compare current input fields against `prevInput` with tolerances `EPS_*`. If any change, set `recomputeRequired := TRUE`.
3. **Recompute trajectory (if required)** — for each active DoF call `instProfile.ComputeMinDuration()`, then `instSync.Sync*()` for synchronization, then `instProfile.ComputeFinalProfile()` per DoF. Reset `trajectory.currentTime := 0.0`, mark `isValid := TRUE`, save inputs to `prevInput`.
4. **Advance time** — `trajectory.currentTime += cycleTime`. Detect end-of-trajectory when `currentTime ≥ trajectory.duration - EPS_TIME`.
5. **Evaluate current state** — for each DoF call `instEval.StateAtTime()` to produce `output.newPosition[i], newVelocity[i], newAcceleration[i]`.
6. **Update status** — set `result := RESULT_WORKING` or `RESULT_FINISHED`. Populate `output.trajectoryDuration` and `output.currentTime` for ETA queries downstream.

**Continuity guarantee**: at each recomputation, the new trajectory starts from `(currentPos, currentVel, currentAcc)` at `t=0`, so output at `t=dt` of the new profile is the lisse continuation of the previous cycle's output. No discontinuity despite full replan on every change.

---

## 6. Error Handling

Status codes follow DA014 ranges:

| Range | Category | Examples |
|---|---|---|
| 16#0000 | Success | Trajectory completed |
| 16#7xxx | In progress | `RESULT_WORKING` |
| 16#82xx | Parameterization error | `vMax ≤ 0`, `|currentVel| > vMax`, `nDofs > DOF_MAX` |
| 16#86xx | Internal execution error | Solver divergence, non-finite LREAL detected, profile case impossible |

**On 16#82xx error**: `output` is not modified (preserves previous valid state), `workingOutput := FALSE`. User corrects input and re-calls.

**On 16#86xx error**: `trajectory.isValid := FALSE`, `output` frozen. Recovery requires explicit `Reset()` or a fresh coherent input set.

**LREAL comparison policy** (SE007): all equality and termination comparisons use explicit tolerances. No `=` between LREAL values anywhere.

---

## 7. Validation Strategy

Three test levels, executed in order during development.

### 7.1 SCL unit tests (within library)

Per-component tests using the `plc-code` executor framework from `203-plc-tools`. Coverage: `ProfileCalc`, `MultiAxisSync`, `TrajEvaluator`, `InputValidator`. Target: ~100 unit tests, ≥95% branch coverage.

### 7.2 Parity test bench (external Python)

External Python bench in `tests/parity/`, compares SCL output to Ruckig C++ reference cycle-by-cycle.

```
ScenarioGenerator  →  ReferenceRunner (pyruckig C++ binding)  ─┐
                  →  SclRunner       (PLCSIM Advanced + OPC UA)─┤
                                                                 ▼
                                                       ParityComparator
                                                       ├─ tolerances 1e-9 pos/vel/acc
                                                       ├─ 1e-7 on trajectory duration
                                                       └─ PASS / FAIL / WARN report
```

Stack: Python 3.12+, `uv`, `pyruckig`, `asyncua`, `pytest`, `matplotlib` (debug plots).

Three test classes:
- **Nominal**: ~30 deterministic scenarios (point-to-point, target vel ≠ 0)
- **Multi-axis**: ~20 scenarios across all sync modes
- **Degenerate**: ~20 scenarios (already at target, unreachable target, near-singular jerk)

CI on GitHub Actions: nominal scenarios on every push, full suite + fuzzing (1000 random scenarios with seed) on nightly schedule.

### 7.3 Integration tests (post-v0.9)

After v0.9 release, the library is consumed by the `plc-tools` integration framework (`plc sim test`) via YAML scenarios analogous to EFAT. Validates behavior in a real project context (e.g. Ruwais MLA).

---

## 8. Roadmap

```
v0.1  Single-axis position, targetVel=0           4 weeks
      ├─ FB RuckigOtg, InputValidator, ProfileCalc
      ├─ UDTs typeRuckigInput/Output/Profile/Trajectory
      ├─ Parity bench: 10 nominal scenarios
      ├─ README + 1 simple SCL example
      └─ GitHub release v0.1.0

v0.2  Single-axis with targetVel/Acc ≠ 0          2 weeks
      ├─ ProfileCalc extended for arbitrary target state
      └─ +10 parity scenarios

v0.3  Multi-axis without sync (SYNC_NONE)         2 weeks
      ├─ Multi-DoF loop in Update()
      └─ Multi-axis input validation

v0.4  Multi-axis phase sync (SYNC_PHASE)          3 weeks
      ├─ MultiAxisSync.SyncPhase()
      ├─ +15 multi-axis parity scenarios
      └─ Cartesian 4-DOF example

v0.5  Multi-axis time sync + per-DoF              2 weeks
      └─ MultiAxisSync.SyncTime() + SyncPerDoF()

v0.6  Velocity interface                          2 weeks
      └─ controlInterface = IFACE_VELOCITY

v0.7  Brake profiles + edge cases                 3 weeks
      ├─ ComputeBrake() for overshoot detection
      ├─ Exhaustive degenerate cases
      └─ +20 degenerate parity scenarios

v0.8  Performance optimization + stabilization    2 weeks
      └─ PLCSIM bench, target <2 ms per Update() at 4-DOF

v0.9  Public documentation + pre-1.0 release      2 weeks
      ├─ Complete English user docs
      ├─ Advanced examples (Cartesian tracking, multi-axis sync)
      ├─ README, CONTRIBUTING, LICENSE (MIT)
      ├─ First public GitHub release v0.9.0
      └─ Community announcement (LinkedIn, /r/PLC)

═══ Gate v0.9 → v1.0: field test campaigns ═══

v1.0  Production-ready (date TBD)
      ├─ Integration with plc-tools test framework
      ├─ Validation on real MLA project (Cartesian tracking)
      ├─ Field test feedback consolidated in docs
      ├─ Long-run stability confirmed
      └─ Tag v1.0.0
```

**Total v0.1 → v0.9: ~22 weeks cumulative free time** (no fixed schedule, paced as personal project).

---

## 9. Repository Structure

```
205-ruckig-scl/                       # project root (local)
├── README.md                         # English, GitHub-facing
├── LICENSE                           # MIT
├── CONTRIBUTING.md                   # contribution guidelines
├── CHANGELOG.md                      # per-release notes
├── docs/
│   ├── superpowers/
│   │   ├── specs/
│   │   │   └── 2026-05-28-ruckig-scl-port-design.md  # this file
│   │   └── plans/
│   │       └── (created by writing-plans skill)
│   └── user/                         # public docs (mkdocs)
│       ├── index.md
│       ├── getting-started.md
│       ├── api-reference.md
│       ├── concepts.md
│       └── examples/
├── src/                              # SCL source (one .s7dcl per block)
│   ├── RuckigOtg.s7dcl
│   ├── ProfileCalc.s7dcl
│   ├── MultiAxisSync.s7dcl
│   ├── TrajEvaluator.s7dcl
│   ├── InputValidator.s7dcl
│   ├── dbRuckigConst.s7dcl
│   └── data-types/
│       ├── typeRuckigInput.s7dcl
│       ├── typeRuckigOutput.s7dcl
│       ├── typeProfile.s7dcl
│       ├── typeBrakeProfile.s7dcl
│       └── typeTrajectory.s7dcl
├── tests/
│   ├── unit/                         # plc-code executor tests (Python)
│   └── parity/
│       ├── scenarios/                # YAML scenario definitions
│       ├── runner_ref.py             # pyruckig C++ wrapper
│       ├── runner_scl.py             # PLCSIM + OPC UA wrapper
│       ├── comparator.py             # cycle-by-cycle parity check
│       └── test_*.py                 # pytest entry points
├── examples/
│   ├── single-axis-point-to-point/
│   ├── multi-axis-phase-sync/
│   └── cartesian-tracking/
├── benchmark/                        # PLCSIM performance benchmark
├── .github/
│   └── workflows/
│       ├── unit-tests.yml
│       ├── parity-tests.yml
│       └── nightly-fuzz.yml
├── pyproject.toml                    # Python tooling for tests
├── mkdocs.yml                        # docs site
└── plc.yaml                          # plc-tools configuration
```

---

## 10. Open Questions and Risks

| Item | Type | Notes |
|---|---|---|
| LREAL performance on FW V3.0+ for cyclic Ruckig computation | Risk | To benchmark in v0.1 — target <2 ms per Update() at 4-DOF. If exceeded, optimization may require REAL fallback for inner solver. |
| Profile case enumeration completeness | Risk | Ruckig handles ~7 profile cases (UDDU, UDUD, etc.) with empirical fallbacks. Coverage requires careful audit of upstream code, not just paper-following. |
| Multi-Software-Unit organization in TIA Portal | Open | Decision deferred to v0.1: single-unit project initially; may split per-FB if compilation/scoping becomes problematic. |
| Naming length for nested constants (e.g. `RESULT_ERR_CURR_VEL_EXCEEDS_MAX`) | Open | NF010 caps at 24 chars; abbreviations needed. Final names will be reviewed at first commit. |
| `pyruckig` Python binding API stability | Low risk | Used as-is from upstream; if API breaks, pin to v0.x compatible version. |
| OPC UA performance between PLCSIM Advanced and Python | Low risk | If too slow for fuzzing volume, fall back to PLCSIM batch mode or direct API. |

---

## 11. Out of Scope (Explicitly)

- **Ruckig Pro features**: intermediate waypoints, per-DoF target boundary conditions, position bounds. May be added post-v1.0 if there is demand and licensing remains MIT-compatible.
- **Hardware integration helpers**: FK/IK, drive scaling, axis calibration, encoder filtering — these belong in user code consuming the library.
- **TwinCAT, CodeSys, or other PLC platforms**: scope is strictly S7-1500 TIA V18+.
- **Real-time guarantees beyond the cycle-time benchmark**: this is a software library, not a real-time motion controller in the IEC 61800 sense.
- **Safety functions or PLCopen Safety integration**: out of scope for the library; if needed in a project, must be implemented separately upstream.

---

## 12. Success Criteria

The project succeeds when, at v0.9:

- All Ruckig Community features are functional and validated against the C++ reference with `<1e-9` tolerance on position/velocity/acceleration outputs
- Performance on PLCSIM Advanced FW V3.0+ shows `Update()` execution under 2 ms for 4-DOF in steady state
- Parity test bench passes on 100% of nominal + multi-axis + degenerate scenarios
- Documentation enables a third-party Siemens engineer to integrate the library in <1 day for a typical 1-4 DOF tracking use case
- The repository is publicly available on GitHub under MIT license with full commit history
- At least one production-grade external project (likely Ruwais MLA tracking) has begun integration as part of the v0.9 → v1.0 gate
