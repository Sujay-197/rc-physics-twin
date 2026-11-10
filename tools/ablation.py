"""Ablation of the campus autopilot: the full autopilot, then the full
autopilot minus one component at a time, on the same trips.

The autopilot, as the Unity twin drives it and Python scores it
(Docs/policy-study.md, "A corner speed plan"), is pure pursuit with the
Unity-car gains (policies/controllers_unity_stop.json) and the planned stop,
wrapped in the corner speed plan, wrapped in the kerb recovery:

    recover:corner:3.4,1.2,1.2,0.5:controller:pure_pursuit:policies/controllers_unity_stop.json

Each variant takes one component out:

  no_corner_plan      the corner speed plan (controllers.CornerSpeed) removed
  no_launch_ramp      the plan without its pull-away ramp from the stop (v_launch)
  no_ramps            the plan without its exit and launch ramps (a_accel, v_launch)
  no_kerb_recovery    the kerb recovery (controllers.KerbRecovery) removed
  linear_stop         the planned stop (brake at a_stop to stop_margin short of the
                      end) replaced by the simplest stop on the observation: speed in
                      proportion to the distance left over the last 10 m
                      (stop_plan "linear")
  no_cornering_limit  pure pursuit's own reactive cornering limit (a_lat 12, read off
                      the observed centreline 2 m ahead) removed
  study_gains         the Unity-car gains replaced by the policy study's
                      (policies/controllers.json: k_speed 4, a_lat 5)

and two baselines:

  pp_unity_gains      pure pursuit with the Unity-car gains alone (planned stop only)
  plain_pp            pure pursuit with the study's gains (planned stop only)

The stop task cannot be driven at all without some stop, so both baselines keep
the planned stop. Protocol as the published fresh-trip table: stop task,
car "unity", conditions train / held_out / perturbed, 100 trips each from seed
50000, cruise speeds 1 to 4 m/s. Every variant drives the same trips and cars.

Per variant, speed and condition: success with a Wilson 95 % interval, how the
failures ended, kerb contact (share of all trips that touched a wall), mean
|offset| over the trips that finished, median trip time over the trips that
finished, the median trip-time ratio to the full autopilot over the trips both
finished, and a paired two-sided sign test against the full autopilot (trips
one finished and the other did not).

    python tools/ablation.py                         # the whole grid, ~half the cores
    python tools/ablation.py --speeds 4 --trips 20   # a quick look

Writes results/reference/ablation.json (summaries and a compact per-trip
record) and prints Markdown tables.
"""

from __future__ import annotations

import argparse
import json
import os
import pathlib
import statistics
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim_py"))

import evaluate_policies as ev  # noqa: E402

UNITY = "policies/controllers_unity_stop.json"
STUDY = "policies/controllers.json"
PLAN = "3.4,1.2,1.2,0.5"         # the by-design corner plan: a_lat, a_brake, a_accel, v_launch


def _gains(path: str, **override) -> str:
    g = dict(json.loads((ROOT / path).read_text(encoding="utf-8"))["pure_pursuit"]["gains"])
    g.update(override)
    return json.dumps(g, sort_keys=True, separators=(",", ":"))


def _pp(gains: str | None = None) -> str:
    return f"controller:pure_pursuit:{gains or UNITY}"


def variants() -> dict:
    """name -> (label, policy spec), the full autopilot first."""
    pp = _pp()
    plan = json.loads((ROOT / UNITY).read_text(encoding="utf-8"))["corner_speed"]["gains"]
    assert PLAN == ",".join(f"{plan[k]:g}" for k in ("a_lat", "a_brake", "a_accel", "v_launch")), plan
    return {
        "full": ("full autopilot", f"recover:corner:{PLAN}:{pp}"),
        "no_corner_plan": ("- corner speed plan", f"recover:{pp}"),
        "no_launch_ramp": ("- launch ramp (plan keeps exit ramp)", f"recover:corner:3.4,1.2,1.2:{pp}"),
        "no_ramps": ("- exit and launch ramps", f"recover:corner:3.4,1.2:{pp}"),
        "no_kerb_recovery": ("- kerb recovery", f"corner:{PLAN}:{pp}"),
        "linear_stop": ("- planned stop (linear slow-down)",
                        f"recover:corner:{PLAN}:{_pp(_gains(UNITY, stop_plan='linear'))}"),
        "no_cornering_limit": ("- reactive cornering limit",
                               f"recover:corner:{PLAN}:{_pp(_gains(UNITY, a_lat=None))}"),
        "study_gains": ("- Unity-car gains (study gains)", f"recover:corner:{PLAN}:controller:pure_pursuit"),
        "pp_unity_gains": ("pure pursuit, Unity-car gains only", pp),
        "plain_pp": ("plain pure pursuit (study gains)", "controller:pure_pursuit"),
    }


def _ok(e) -> bool:
    return e["status"] == "reached_end"


def compare(eps, ref) -> dict:
    """Paired with the full autopilot (same seed: same trip, same car)."""
    base = {e["seed"]: e for e in ref}
    wins = sum(1 for e in eps if _ok(e) and not _ok(base[e["seed"]]))
    losses = sum(1 for e in eps if not _ok(e) and _ok(base[e["seed"]]))
    ratios = [e["ticks"] / base[e["seed"]]["ticks"] for e in eps if _ok(e) and _ok(base[e["seed"]])]
    return {"wins": wins, "losses": losses, "p": ev.sign_test(wins, losses),
            "time_ratio_median": statistics.median(ratios) if ratios else None, "both_finished": len(ratios)}


def condense(eps, tick: float) -> dict:
    s = ev.summarise(eps)
    ok = [e for e in eps if _ok(e)]
    times = [e["ticks"] * tick for e in ok]
    out = {"trips": s["trips"], "success": s["success"], "wall_contact": s["wall_contact"],
           "wall_contact_finished": ev.wilson(sum(e["wall"] for e in ok), len(ok)),
           "outcomes": {k: sum(e["status"] == k for e in eps)
                        for k in ("reached_end", "left_road", "overshot", "timeout")},
           "offset_mean": s["offset_mean"], "steer_roughness": s["steer_roughness"],
           "energy_wh_per_m": s["energy_wh_per_m"],
           "trip_time_median_s": statistics.median(times) if times else None,
           "back_offs": sum(e["back_offs"] for e in eps)}
    assert sum(out["outcomes"].values()) == len(eps), {e["status"] for e in eps}
    return out


def compact(e) -> list:
    return [e["seed"], e["status"], int(e["wall"]), e["ticks"], round(e["offset_mean"], 5), e["back_offs"]]


def run(speeds, trips: int, seed: int, workers: int, lib=None) -> dict:
    tick = ev.campus().envmod.TICK
    vs = variants()
    specs = [spec for _, spec in vs.values()]
    out = {"protocol": {"task": "stop", "car": "unity", "trips": trips, "seed": seed,
                        "conditions": list(ev.CONDITIONS), "speeds": list(speeds),
                        "perturbation": ev.PERTURBATION, "tick_s": tick},
           "variants": {k: {"label": lab, "spec": spec} for k, (lab, spec) in vs.items()},
           "per_trip_fields": ["seed", "status", "wall", "ticks", "offset_mean", "back_offs"],
           "results": {}}
    t0 = time.perf_counter()
    for v in speeds:
        rep = ev.evaluate(specs, trips, ev.CONDITIONS, lib, workers, seed=seed, target_speed=v,
                          task="stop", car="unity")
        pol = rep["policies"]
        full = vs["full"][1]
        res = {}
        for name, (_, spec) in vs.items():
            res[name] = {}
            for cond in ev.CONDITIONS:
                eps = pol[spec][cond]["episodes"]
                d = condense(eps, tick)
                d["vs_full"] = compare(eps, pol[full][cond]["episodes"])
                d["episodes"] = [compact(e) for e in eps]
                res[name][cond] = d
            # Pooled over the three conditions (paired trip by trip within each).
            w = sum(res[name][c]["vs_full"]["wins"] for c in ev.CONDITIONS)
            lo = sum(res[name][c]["vs_full"]["losses"] for c in ev.CONDITIONS)
            res[name]["pooled_vs_full"] = {"wins": w, "losses": lo, "p": ev.sign_test(w, lo)}
        out["results"][f"{v:g}"] = res
        print(f"{v:g} m/s done ({time.perf_counter() - t0:.0f} s)", file=sys.stderr)
    # Pooled over every speed and condition.
    for name in vs:
        w = sum(out["results"][f"{v:g}"][name]["pooled_vs_full"]["wins"] for v in speeds)
        lo = sum(out["results"][f"{v:g}"][name]["pooled_vs_full"]["losses"] for v in speeds)
        out["variants"][name]["pooled_vs_full_all_speeds"] = {"wins": w, "losses": lo, "p": ev.sign_test(w, lo)}
    out["seconds"] = round(time.perf_counter() - t0, 1)
    return out


# ------------------------------------------------------------------ tables

def _pct(t) -> str:
    return f"{100 * t[0]:.0f} ({100 * t[1]:.0f}-{100 * t[2]:.0f})"


def _p(p: float) -> str:
    return "< 0.001" if p < 0.001 else f"{p:.3f}"


def _vs(d) -> str:
    c = d["vs_full"]
    return "-" if c["wins"] == c["losses"] == 0 else f"+{c['wins']} / -{c['losses']}, p {_p(c['p'])}"


def speed_table(report: dict, speed: str) -> str:
    res = report["results"][speed]
    rows = [f"Stop task, Unity car, {speed} m/s, {report['protocol']['trips']} trips per condition from seed "
            f"{report['protocol']['seed']}. Success in % with Wilson 95 % interval. Kerb contact: share of "
            "held-out trips touching a wall. Offset and trip time: held-out trips that finished. Time ratio: "
            "median over held-out trips both finished. Against the full autopilot: trips won / lost, two-sided "
            "sign test.", "",
            "| Variant | Train | Held-out | Perturbed | Kerb contact | Mean offset | Trip time | Time ratio "
            "| vs full (held-out) | vs full (perturbed) |",
            "|---|---|---|---|---|---|---|---|---|---|"]
    for name, meta in report["variants"].items():
        r = res[name]
        h = r["held_out"]
        off = h["offset_mean"][0]
        tt = h["trip_time_median_s"]
        tr = h["vs_full"]["time_ratio_median"]
        rows.append(f"| {meta['label']} | {_pct(r['train']['success'])} | {_pct(h['success'])} "
                    f"| {_pct(r['perturbed']['success'])} | {100 * h['wall_contact'][0]:.0f} % "
                    f"| {'n/a' if off != off else f'{100 * off:.1f} cm'} | {'n/a' if tt is None else f'{tt:.1f} s'} "
                    f"| {'n/a' if tr is None else f'{tr:.2f}'} | {_vs(h)} | {_vs(r['perturbed'])} |")
    return "\n".join(rows)


def summary_table(report: dict) -> str:
    speeds = list(report["results"])
    rows = ["Success in % (train / held-out / perturbed) at each cruise speed; last column: every trip at every "
            "speed and condition against the full autopilot, two-sided sign test.", "",
            "| Variant | " + " | ".join(f"{s} m/s" for s in speeds) + " | vs full, pooled |",
            "|---|" + "---|" * len(speeds) + "---|"]
    for name, meta in report["variants"].items():
        cells = []
        for s in speeds:
            r = report["results"][s][name]
            cells.append(" / ".join(f"{100 * r[c]['success'][0]:.0f}" for c in ev.CONDITIONS))
        pv = meta["pooled_vs_full_all_speeds"]
        vs = "-" if pv["wins"] == pv["losses"] == 0 else f"+{pv['wins']} / -{pv['losses']}, p {_p(pv['p'])}"
        rows.append(f"| {meta['label']} | " + " | ".join(cells) + f" | {vs} |")
    return "\n".join(rows)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--speeds", type=float, nargs="+", default=[1.0, 2.0, 3.0, 4.0], help="cruise speeds [m/s]")
    ap.add_argument("--trips", type=int, default=100)
    ap.add_argument("--seed", type=int, default=50_000, help="first trip's seed (default: the fresh trips)")
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) // 2),
                    help="worker processes (default: half the cores)")
    ap.add_argument("--out", type=pathlib.Path, default=ROOT / "results" / "reference" / "ablation.json")
    ap.add_argument("--lib")
    args = ap.parse_args(argv)
    report = run(args.speeds, args.trips, args.seed, args.workers, os.path.abspath(args.lib) if args.lib else None)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, separators=(",", ":")) + "\n", encoding="utf-8")
    print(summary_table(report) + "\n")
    for s in report["results"]:
        print(speed_table(report, s) + "\n")
    print(f"wrote {args.out} ({report['seconds']} s)", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
