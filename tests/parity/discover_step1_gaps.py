"""Seeded discovery of step1 gap states (SCL errors, oracle solves).

Run:  uv run python -m tests.parity.discover_step1_gaps
Reproducible (fixed seed). Prints YAML-ready gap states; a representative subset
is frozen as v09_* parity scenarios + test_solve_two_step fixtures.
"""
from __future__ import annotations
import random
from pathlib import Path
from plc_code.executor import create_harness
from plc_code.executor.runtime import PLCRuntime
import ruckig

RESULT_ERR_SOLVER = 0x8602
ROOT = Path(__file__).resolve().parent.parent.parent
SP = [ROOT / "src/blocks", ROOT / "src/data-types", ROOT / "src/data-blocks"]
VMAX, AMAX, JMAX = 3.0, 5.0, 10.0


def _profile():
    return {"t": [0.0]*7, "j": [0.0]*7, "a": [0.0]*8, "v": [0.0]*8, "p": [0.0]*8,
            "direction": 0, "controlSigns": 0,
            "brake": {"t": [0.0, 0.0], "j": [0.0, 0.0], "a": [0.0]*3, "v": [0.0]*3, "p": [0.0]*3}}


def main(n=4000, want=10, seed=12345):
    random.seed(seed)
    rt = PLCRuntime(block_search_paths=SP)
    h = create_harness(ROOT / "src/blocks/ComputeProfile1Axis.s7dcl", runtime=rt)
    otg = ruckig.Ruckig(1, 0.01)
    gaps = []
    for _ in range(n):
        s = dict(p0=0.0, pT=round(random.uniform(-3, 3), 3),
                 v0=round(random.uniform(-3, 3), 3), vT=round(random.uniform(-3, 3), 3),
                 a0=round(random.uniform(-5, 5), 3), aT=round(random.uniform(-5, 5), 3))
        h.reset()
        h.set_inputs(profile=_profile(), p0=s["p0"], v0=s["v0"], a0=s["a0"],
                     pT=s["pT"], vT=s["vT"], aT=s["aT"], vMax=VMAX, aMax=AMAX, jMax=JMAX)
        h.execute()
        if h.get_output("ComputeProfile1Axis") == RESULT_ERR_SOLVER:
            inp = ruckig.InputParameter(1); out = ruckig.OutputParameter(1)
            inp.current_position = [s["p0"]]; inp.current_velocity = [s["v0"]]; inp.current_acceleration = [s["a0"]]
            inp.target_position = [s["pT"]]; inp.target_velocity = [s["vT"]]; inp.target_acceleration = [s["aT"]]
            inp.max_velocity = [VMAX]; inp.max_acceleration = [AMAX]; inp.max_jerk = [JMAX]
            try:
                if otg.update(inp, out) == ruckig.Result.Working:
                    gaps.append((s, round(out.trajectory.duration, 4)))
            except Exception:
                pass
        if len(gaps) >= want:
            break
    for s, dur in gaps:
        print(f"GAP {s['v0']},{s['a0']},{s['pT']},{s['vT']},{s['aT']}  dur={dur}")
    print(f"found {len(gaps)}")


if __name__ == "__main__":
    main()
