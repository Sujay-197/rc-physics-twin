#!/usr/bin/env python3
"""Evaluate everything the policy study trained, and tabulate it.

Policies: the three tuned controllers (policies/controllers.json), and every
model under policies/study/ (<method>-s<seed>: .json for CEM, .zip and
friends for SB3). Each is driven over the same trips and cars by
sim_py/evaluate_policies.py in all three conditions at the study's speed.

Per method (seeds pooled): outcomes with Wilson intervals over all its trips,
the spread of success across seeds (min to max), driving quality over the
trips that finished, and a paired comparison with the tuned pure pursuit:
every policy meets the same trip and car, so for each condition we count the
trips a method finished that pure pursuit did not, and the reverse.

Every trip is also classed by its route: "turn at start" when its first
turn of more than 45 degrees comes within 3 m of the start. The task starts
each episode at the target speed, so on those routes (about a quarter of
the campus's stop pairs, whose first junction is 1.2 to 1.6 m from the
stop) the car meets a 51 to 90 degree turn at 4 m/s almost at once; success
is reported on the other trips too.

Writes results/policy_study.json (every episode) and results/policy_study.md.

    .venv/Scripts/python tools/study_report.py --trips 100
"""

from __future__ import annotations

import argparse
import json
import math
import os
import pathlib
import re
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim_py"))
import evaluate_policies as ev  # noqa: E402

STUDY = ROOT / "policies" / "study"
BASELINE = "controller:pure_pursuit"
ORDER = ["controller:pure_pursuit", "controller:stanley", "controller:lqr", "cem", "ppo", "sac", "rppo",
         "ppo-res", "sac-res", "td3-res"]
LABEL = {"controller:pure_pursuit": "pure pursuit (tuned)", "controller:stanley": "Stanley (tuned)",
         "controller:lqr": "LQR (tuned)", "cem": "CEM, linear", "ppo": "PPO + history",
         "sac": "SAC + history", "rppo": "RecurrentPPO (LSTM)", "ppo-res": "residual PPO on pure pursuit",
         "sac-res": "residual SAC on pure pursuit", "td3-res": "residual TD3 on pure pursuit"}


def discover():
    """{method: [spec per seed]} for everything trained so far."""
    methods: dict[str, list[str]] = {k: [k] for k in ORDER if k.startswith("controller:")}
    for f in sorted(STUDY.glob("*-s*.json")):
        m = re.fullmatch(r"(.+)-s(\d+)", f.stem)
        if not m:
            continue
        method = m.group(1)
        if method == "cem":
            spec = f"cem:{f}"
        elif f.with_suffix(".zip").exists():
            spec = f"sb3:{f.with_suffix('')}"
        else:
            continue
        methods.setdefault(method, []).append(spec)
    return {k: methods[k] for k in sorted(methods, key=lambda k: ORDER.index(k) if k in ORDER else 99)}


_TURN = {}


def turn_at_start(trip, within: float = 3.0, angle: float = 45.0) -> bool:
    """The route's first turn of more than `angle` degrees comes within
    `within` m of the start."""
    trip = tuple(trip)
    if trip not in _TURN:
        net = ev.campus().CampusNetwork.load()
        pts = net.route_points(net.stops[trip[0]]["node"], net.stops[trip[1]]["node"])
        s, prev, hit = 0.0, None, False
        for a, b in zip(pts[:-1], pts[1:]):
            L = math.hypot(b[0] - a[0], b[1] - a[1])
            if L < 1e-9:
                continue
            h = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
            if prev is not None and abs((h - prev + 180.0) % 360.0 - 180.0) > angle:
                hit = s < within
                break
            prev, s = h, s + L
        _TURN[trip] = hit
    return _TURN[trip]


def aggregate(report, methods, conditions):
    pol = report["policies"]
    out = {}
    for method, specs in methods.items():
        out[method] = {}
        for cond in conditions:
            eps = [e for s in specs for e in pol[s][cond]["episodes"]]
            seed_success = [pol[s][cond]["summary"]["success"][0] for s in specs]
            summ = ev.summarise(eps)
            # Paired with pure pursuit: same seeds, so the same trip and car.
            base = {e["seed"]: e["status"] == "reached_end" for e in pol[BASELINE][cond]["episodes"]}
            better = sum(1 for e in eps if e["status"] == "reached_end" and not base[e["seed"]])
            worse = sum(1 for e in eps if e["status"] != "reached_end" and base[e["seed"]])
            rest = [e for e in eps if not turn_at_start(e["trip"])]
            tight = [e for e in eps if turn_at_start(e["trip"])]
            out[method][cond] = {"seeds": len(specs), "summary": summ, "seed_success": seed_success,
                                 "success_turn_at_start": ev.wilson(sum(e["status"] == "reached_end" for e in tight), len(tight)),
                                 "success_other_trips": ev.wilson(sum(e["status"] == "reached_end" for e in rest), len(rest)),
                                 "vs_pure_pursuit": {"finished_where_it_failed": better,
                                                     "failed_where_it_finished": worse,
                                                     "trips": len(eps)}}
    return out


def markdown(agg, report, methods) -> str:
    proto = report["protocol"]
    lines = [f"Target speed {proto['target_speed']:g} m/s; {proto['trips']} trips per condition per seed "
             f"(seeds {proto['seed']} to {proto['seed'] + proto['trips'] - 1}); perturbation "
             f"{json.dumps(proto['perturbation'])}.", ""]
    for cond in proto["conditions"]:
        lines += [f"### {cond}", "",
                  "| method | seeds | success (95 % CI) | seed range | wall contact | left road "
                  "| vs pure pursuit (+/-) | offset mean [cm] | offset p95 [cm] | speed error [%] "
                  "| steer roughness | energy [mWh/m] |",
                  "|---|---|---|---|---|---|---|---|---|---|---|---|"]
        for method in methods:
            d = agg[method][cond]
            s = d["summary"]
            pct = lambda t: f"{100 * t[0]:.1f} ({100 * t[1]:.0f}-{100 * t[2]:.0f})"
            rng = f"{100 * min(d['seed_success']):.0f}-{100 * max(d['seed_success']):.0f}" \
                if d["seeds"] > 1 else "-"
            vs = d["vs_pure_pursuit"]
            q = lambda k, f: "n/a" if s[k][0] != s[k][0] else f(s[k][0])
            lines.append(
                f"| {LABEL.get(method, method)} | {d['seeds']} | {pct(s['success'])} | {rng} "
                f"| {100 * s['wall_contact'][0]:.1f} | {100 * s['left_road'][0]:.1f} "
                f"| +{vs['finished_where_it_failed']} / -{vs['failed_where_it_finished']} "
                f"| {q('offset_mean', lambda x: f'{100 * x:.1f}')} | {q('offset_p95', lambda x: f'{100 * x:.1f}')} "
                f"| {q('speed_error', lambda x: f'{100 * x:.1f}')} | {q('steer_roughness', lambda x: f'{x:.4f}')} "
                f"| {q('energy_wh_per_m', lambda x: f'{1000 * x:.3f}')} |")
        lines.append("")
    lines += ["### Success by route: turn at the start or not", "",
              "| method | condition | turn at start | other trips |", "|---|---|---|---|"]
    for method in methods:
        for cond in proto["conditions"]:
            d = agg[method][cond]
            f = lambda t: f"{100 * t[0]:.1f} ({100 * t[1]:.0f}-{100 * t[2]:.0f})"
            lines.append(f"| {LABEL.get(method, method)} | {cond} | {f(d['success_turn_at_start'])} "
                         f"| {f(d['success_other_trips'])} |")
    lines.append("")
    return "\n".join(lines)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--trips", type=int, default=100)
    ap.add_argument("--target-speed", type=float, default=4.0)
    ap.add_argument("--workers", type=int)
    ap.add_argument("--lib")
    args = ap.parse_args(argv)
    lib = os.path.abspath(args.lib) if args.lib else str(ROOT / "build" / "bin" / "rc_twin.dll")
    methods = discover()
    specs = [s for v in methods.values() for s in v]
    print(f"evaluating {len(specs)} policies in {len(methods)} methods", flush=True)
    t0 = time.perf_counter()
    report = ev.evaluate(specs, trips=args.trips, lib=lib, workers=args.workers, target_speed=args.target_speed)
    report["seconds"] = round(time.perf_counter() - t0, 1)
    agg = aggregate(report, methods, report["protocol"]["conditions"])
    curves = {}
    for method, ss in methods.items():
        for s in ss:
            if s.startswith("sb3:"):
                meta = json.loads(pathlib.Path(s[4:]).with_suffix(".json").read_text(encoding="utf-8"))
                curves[s] = {"curve": meta["curve"], "train_seconds": meta["train_seconds"],
                             "timesteps": meta["timesteps"]}
            elif s.startswith("cem:"):
                meta = json.loads(pathlib.Path(s[4:]).read_text(encoding="utf-8"))
                curves[s] = {"train_seconds": meta["train_seconds"]}
    out = {"methods": methods, "aggregate": agg, "training": curves, "evaluation": report}
    res = ROOT / "results"
    res.mkdir(exist_ok=True)
    (res / "policy_study.json").write_text(json.dumps(out) + "\n", encoding="utf-8")
    md = markdown(agg, report, methods)
    (res / "policy_study.md").write_text(md + "\n", encoding="utf-8")
    print(md)
    print(f"evaluated in {report['seconds']} s; wrote results/policy_study.json and .md", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
