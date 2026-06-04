"""Seeded discovery of step2 gap states (SCL shape differs from Ruckig oracle).

Run:  uv run python -m tests.parity.discover_step2_gaps
Reproducible (fixed seed). For each fuzz sample with a feasible tf, compares
the SCL ComputeProfile1AxisTimed profile to Ruckig's oracle (minimum_duration=tf).
Counts GENUINE divergences: both solvers reach the target at tf but the profile
shapes (sampled state trajectory) differ > 1e-6. Infeasible-tf false positives
(where the oracle's reported duration != tf within 1e-6) are excluded.

Reports: total pairs, genuine divergence count + rate, diverging cases with
SCL-vs-oracle shape hints (t[] extrema).
"""
from __future__ import annotations

import random
from pathlib import Path

from plc_code.executor import create_harness
from plc_code.executor.runtime import PLCRuntime
import ruckig

ROOT = Path(__file__).resolve().parent.parent.parent
SP = [ROOT / "src/blocks", ROOT / "src/data-types", ROOT / "src/data-blocks"]
VMAX, AMAX, JMAX = 3.0, 5.0, 10.0

RESULT_WORKING = 0x7000

TF_FACTORS = [1.05, 1.5, 2.0, 3.0, 5.0]


def _empty_profile():
    return {
        "t": [0.0] * 7, "j": [0.0] * 7,
        "a": [0.0] * 8, "v": [0.0] * 8, "p": [0.0] * 8,
        "direction": 1,
        "controlSigns": 0,
        "brake": {"t": [0.0, 0.0], "j": [0.0, 0.0],
                  "a": [0.0] * 3, "v": [0.0] * 3, "p": [0.0] * 3},
    }


def _integrate_at(t_arr, j_arr, p0, v0, a0, s):
    """Sample a SCL profile (t[]/j[]) at absolute time s."""
    a, v, p, elapsed = a0, v0, p0, 0.0
    for i in range(7):
        dt = t_arr[i]; j = j_arr[i]
        if s <= elapsed + dt + 1e-12:
            d = s - elapsed
            return (p + v * d + 0.5 * a * d * d + j * d ** 3 / 6.0,
                    v + a * d + 0.5 * j * d * d,
                    a + j * d)
        p = p + v * dt + 0.5 * a * dt * dt + j * dt ** 3 / 6.0
        v = v + a * dt + 0.5 * j * dt * dt
        a = a + j * dt
        elapsed += dt
    return (p, v, a)


def _get_tmin(p0, v0, a0, pT, vT, aT):
    """Time-optimal duration (step1) via Ruckig.calculate."""
    otg = ruckig.Ruckig(1)
    inp = ruckig.InputParameter(1)
    tr = ruckig.Trajectory(1)
    inp.current_position = [p0]; inp.current_velocity = [v0]; inp.current_acceleration = [a0]
    inp.target_position = [pT]; inp.target_velocity = [vT]; inp.target_acceleration = [aT]
    inp.max_velocity = [VMAX]; inp.max_acceleration = [AMAX]; inp.max_jerk = [JMAX]
    try:
        res = otg.calculate(inp, tr)
    except Exception:
        return None
    if int(res) < 0:
        return None
    return tr.duration


def _oracle_at(p0, v0, a0, pT, vT, aT, tf):
    """Oracle trajectory forced to duration tf. Returns (tr, duration) or None if infeasible."""
    otg = ruckig.Ruckig(1)
    inp = ruckig.InputParameter(1)
    tr = ruckig.Trajectory(1)
    inp.current_position = [p0]; inp.current_velocity = [v0]; inp.current_acceleration = [a0]
    inp.target_position = [pT]; inp.target_velocity = [vT]; inp.target_acceleration = [aT]
    inp.max_velocity = [VMAX]; inp.max_acceleration = [AMAX]; inp.max_jerk = [JMAX]
    inp.minimum_duration = tf
    try:
        res = otg.calculate(inp, tr)
    except Exception:
        return None
    if int(res) < 0:
        return None
    return tr


def _max_abs_v(t_arr, j_arr, v0, a0):
    """Quick peak |v| over the SCL profile phases."""
    a, v = a0, v0
    peak = abs(v0)
    for i in range(7):
        dt = t_arr[i]; j = j_arr[i]
        v_end = v + a * dt + 0.5 * j * dt * dt
        peak = max(peak, abs(v_end))
        a = a + j * dt
        v = v_end
    return peak


def main(n=50, seed=42345):
    random.seed(seed)
    rt = PLCRuntime(block_search_paths=SP)
    h = create_harness(ROOT / "src/blocks/ComputeProfile1AxisTimed.s7dcl", runtime=rt)

    total_pairs = 0
    genuine_divergences = []

    for _ in range(n):
        v0 = round(random.uniform(-VMAX, VMAX), 3)
        a0 = round(random.uniform(-AMAX, AMAX), 3)
        pT = round(random.uniform(-3.0, 3.0), 3)
        vT = round(random.uniform(-VMAX, VMAX), 3)
        aT = round(random.uniform(-AMAX, AMAX), 3)
        p0 = 0.0

        tmin = _get_tmin(p0, v0, a0, pT, vT, aT)
        if tmin is None or tmin < 1e-9:
            continue

        for factor in TF_FACTORS:
            tf = tmin * factor
            total_pairs += 1

            # Exclude infeasible-tf (oracle lands on different duration)
            oracle_tr = _oracle_at(p0, v0, a0, pT, vT, aT, tf)
            if oracle_tr is None:
                continue
            if abs(oracle_tr.duration - tf) > 1e-6:
                # blocked interval — not a genuine step2 test case
                continue

            # Run SCL ComputeProfile1AxisTimed
            h.reset()
            h.set_inputs(profile=_empty_profile(),
                         p0=p0, v0=v0, a0=a0,
                         pT=pT, vT=vT, aT=aT,
                         vMax=VMAX, aMax=AMAX, jMax=JMAX,
                         tf=tf)
            h.execute()
            ret_code = h.get_output("ComputeProfile1AxisTimed")
            if ret_code != RESULT_WORKING:
                # SCL solver failed — not a shape divergence, record as solver gap
                continue

            prof = h.get_var("profile")
            t_arr = [prof.t[i] for i in range(7)]
            j_arr = [prof.j[i] for i in range(7)]

            # Compare SCL vs oracle at 5 sample points
            diverge = False
            max_err = 0.0
            for frac in (0.2, 0.4, 0.6, 0.8, 1.0):
                s = tf * frac
                try:
                    oracle_p, oracle_v, oracle_a = (x[0] for x in oracle_tr.at_time(s))
                except Exception:
                    break
                scl_p, scl_v, scl_a = _integrate_at(t_arr, j_arr, p0, v0, a0, s)
                err = max(abs(scl_p - oracle_p), abs(scl_v - oracle_v), abs(scl_a - oracle_a))
                max_err = max(max_err, err)
                if err > 1e-6:
                    diverge = True
                    break

            if diverge:
                scl_peak_v = _max_abs_v(t_arr, j_arr, v0, a0)
                try:
                    oracle_peak_v = max(abs(oracle_tr.at_time(s)[1][0])
                                       for s in [tf * f for f in (0.0, 0.25, 0.5, 0.75, 1.0)])
                except Exception:
                    oracle_peak_v = float("nan")
                genuine_divergences.append({
                    "v0": v0, "a0": a0, "pT": pT, "vT": vT, "aT": aT,
                    "tf": round(tf, 6),
                    "factor": factor,
                    "max_err": max_err,
                    "scl_t3": t_arr[3],
                    "scl_t1": t_arr[1],
                    "scl_t5": t_arr[5],
                    "scl_peak_v": round(scl_peak_v, 5),
                    "oracle_peak_v": round(oracle_peak_v, 5) if oracle_peak_v == oracle_peak_v else None,
                })

    print(f"total pairs sampled : {total_pairs}")
    print(f"genuine divergences : {len(genuine_divergences)}")
    rate = 100.0 * len(genuine_divergences) / total_pairs if total_pairs else 0.0
    print(f"divergence rate     : {rate:.2f}%")
    print()
    for d in genuine_divergences[:30]:
        print(
            f"DIVERGE  v0={d['v0']:+.3f} a0={d['a0']:+.3f} "
            f"pT={d['pT']:+.3f} vT={d['vT']:+.3f} aT={d['aT']:+.3f}  "
            f"tf={d['tf']:.4f}  "
            f"scl_t[1,3,5]=[{d['scl_t1']:.4f},{d['scl_t3']:.4f},{d['scl_t5']:.4f}]  "
            f"scl_peak|v|={d['scl_peak_v']:.3f}  "
            f"oracle_peak|v|={d['oracle_peak_v']}  "
            f"max_err={d['max_err']:.2e}"
        )
    return genuine_divergences


if __name__ == "__main__":
    main()
