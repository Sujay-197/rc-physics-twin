#!/usr/bin/env python3
"""Pool the policy study's learned runs over seeds and compare them with the
classical baseline, trip by trip (Docs/rl-seeds.md).

Input: reports of sim_py/evaluate_policies.py --out (every episode), one per
comparison. Policies are grouped into methods by their model name with the
seed removed (".../sac-s2" -> "sac"); wrappers stay in the method's name
("recover:" -> "recover:ppo-res-stop-unity-r3").

Per method and condition:
  - pooled success over all its seeds' trips, with a Wilson 95 % interval;
  - each seed's success (the spread);
  - against the baseline, trip by trip (the same seed is the same trip and
    car): trips the method finished that the baseline did not (+) and the
    reverse (-), pooled over seeds and per seed, each with a two-sided exact
    sign test on those discordant trips. Pooled over seeds the same trip
    counts once per seed, so the pooled test treats seeds' trips as
    independent, as the study did;
  - driving quality over the trips each finished, and the median trip-time
    ratio to the baseline over trips both finished.

    python tools/rl_seeds_report.py --baseline controller:pure_pursuit results/rl_seeds/follow_20000.json \
        --out results/reference/rl_seeds/follow_20000
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import re
import statistics
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim_py"))
import evaluate_policies as ev  # noqa: E402


def sign_test(plus: int, minus: int) -> float:
    """Two-sided exact binomial test of plus vs minus discordant trips."""
    n = plus + minus
    if n == 0:
        return 1.0
    k = min(plus, minus)
    tail = sum(math.comb(n, i) for i in range(k + 1)) / 2 ** n
    return min(1.0, 2 * tail)


def method_of(spec: str) -> tuple[str, int | None]:
    """(method, seed) for a policy spec."""
    m = re.fullmatch(r"(.*?)sb3:(.*[\\/])?([^\\/]+)-s(\d+)", spec)
    if not m:
        return spec, None
    return f"{m.group(1)}{m.group(3)}", int(m.group(4))


def ok(e) -> bool:
    return e["status"] == "reached_end"


def compare(eps, base_by_seed):
    plus = sum(1 for e in eps if ok(e) and not ok(base_by_seed[e["seed"]]))
    minus = sum(1 for e in eps if not ok(e) and ok(base_by_seed[e["seed"]]))
    return plus, minus


def time_ratio(eps, base_by_seed):
    r = [e["ticks"] / base_by_seed[e["seed"]]["ticks"] for e in eps if ok(e) and ok(base_by_seed[e["seed"]])]
    return statistics.median(r) if r else float("nan")


def trip_level(per_seed_eps, base_by_seed, boot: int = 10000) -> dict:
    """The method against the baseline with the trip as the unit, so a trip
    counts once however many seeds drove it: per trip, the share of seeds that
    finished it minus whether the baseline did. A sign test on the trips where
    that differs from 0; a paired sign-flip permutation test of the mean of
    those per-trip differences (two-sided, 20000 flips, fixed seed); and a
    bootstrap 95 % interval (trips resampled, fixed seed) on the difference in
    success rate."""
    import random
    trips = sorted(base_by_seed)
    share = {t: statistics.mean(ok(next(e for e in eps if e["seed"] == t)) for eps in per_seed_eps) for t in trips}
    d = [share[t] - ok(base_by_seed[t]) for t in trips]
    plus, minus = sum(x > 0 for x in d), sum(x < 0 for x in d)
    rng = random.Random(0)
    means = sorted(statistics.mean(rng.choice(d) for _ in d) for _ in range(boot))
    obs = abs(sum(d))
    nz = [x for x in d if x]
    flips = 20000
    hits = sum(abs(sum(x if rng.random() < 0.5 else -x for x in nz)) >= obs - 1e-12 for _ in range(flips))
    return {"plus": plus, "minus": minus, "p": sign_test(plus, minus), "diff": statistics.mean(d),
            "p_perm": (hits + 1) / (flips + 1) if nz else 1.0,
            "ci": [means[int(0.025 * boot)], means[int(0.975 * boot) - 1]]}


def analyse(report: dict, baseline: str) -> dict:
    pol = report["policies"]
    conds = report["protocol"]["conditions"]
    methods: dict[str, dict[int | None, str]] = {}
    for spec in pol:
        name, seed = method_of(spec)
        methods.setdefault(name, {})[seed] = spec
    base = {c: {e["seed"]: e for e in pol[baseline][c]["episodes"]} for c in conds}
    out = {"protocol": report["protocol"], "baseline": baseline, "methods": {}}
    for name, seeds in methods.items():
        order = sorted(seeds, key=lambda s: -1 if s is None else s)
        m = {"seeds": [s for s in order if s is not None], "specs": [seeds[s] for s in order], "conditions": {}}
        for c in conds:
            per_seed, all_eps, seed_eps = [], [], []
            for s in order:
                eps = pol[seeds[s]][c]["episodes"]
                all_eps += eps
                seed_eps.append(eps)
                p, q = compare(eps, base[c])
                per_seed.append({"seed": s, "success": sum(map(ok, eps)), "trips": len(eps),
                                 "plus": p, "minus": q, "p": sign_test(p, q)})
            p, q = compare(all_eps, base[c])
            summ = ev.summarise(all_eps)
            k = sum(map(ok, all_eps))
            m["conditions"][c] = {
                "success": k, "trips": len(all_eps), "wilson": ev.wilson(k, len(all_eps)),
                "per_seed": per_seed, "plus": p, "minus": q, "p": sign_test(p, q),
                "trip_level": trip_level(seed_eps, base[c]),
                "seeds_better": sum(x["plus"] > x["minus"] for x in per_seed),
                "seeds_worse": sum(x["plus"] < x["minus"] for x in per_seed),
                "left_road": sum(e["status"] == "left_road" for e in all_eps),
                "timed_out": sum(e["status"] not in ("reached_end", "left_road", "overshot") for e in all_eps),
                "overshot": sum(e["status"] == "overshot" for e in all_eps),
                "wall_contact": sum(e["wall"] for e in all_eps),
                "offset_mean": summ["offset_mean"][0], "offset_p95": summ["offset_p95"][0],
                "steer_roughness": summ["steer_roughness"][0], "energy_wh_per_m": summ["energy_wh_per_m"][0],
                "speed_error": summ["speed_error"][0], "time_ratio": time_ratio(all_eps, base[c])}
        out["methods"][name] = m
    return out


def outcomes(report: dict) -> dict:
    """Compact per-trip record: {spec: {condition: [[seed, status, ticks, wall], ...]}}."""
    return {spec: {c: [[e["seed"], e["status"], e["ticks"], int(e["wall"])] for e in d["episodes"]]
                   for c, d in conds.items()} for spec, conds in report["policies"].items()}


def fmt_p(p: float) -> str:
    return "p < 0.001" if p < 0.001 else f"p = {p:.3f}" if p < 0.1 else f"p = {p:.2f}"


def markdown(a: dict) -> str:
    proto = a["protocol"]
    conds = proto["conditions"]
    lines = [f"Task {proto.get('task', 'follow')}, car {proto.get('car', 'bare')}, {proto['target_speed']:g} m/s, {proto['trips']} trips per "
             f"condition per seed (seeds {proto['seed']} to {proto['seed'] + proto['trips'] - 1}). Baseline "
             f"`{a['baseline']}`.", "",
             "| method | seeds | " + " | ".join(f"{c}: pooled (95 % CI)" for c in conds) + " | "
             + " | ".join(f"{c}: per seed" for c in conds) + " |",
             "|---|---|" + "---|" * (2 * len(conds))]
    for name, m in a["methods"].items():
        cells, seeds = [], []
        for c in conds:
            d = m["conditions"][c]
            w = d["wilson"]
            cells.append(f"{100 * w[0]:.1f} ({100 * w[1]:.0f}-{100 * w[2]:.0f})")
            seeds.append(", ".join(str(s["success"]) for s in d["per_seed"]))
        n = len(m["seeds"]) or "-"
        lines.append(f"| {name} | {n} | " + " | ".join(cells) + " | " + " | ".join(seeds) + " |")
    lines += ["", "Against the baseline, trip by trip: + finished where the baseline failed, − the reverse; "
              "two-sided exact sign test.", "",
              "| method | " + " | ".join(f"{c}: pooled" for c in conds) + " | "
              + " | ".join(f"{c}: per seed" for c in conds) + " |",
              "|---|" + "---|" * (2 * len(conds))]
    for name, m in a["methods"].items():
        if name == a["baseline"]:
            continue
        pooled = [f"+{m['conditions'][c]['plus']} / −{m['conditions'][c]['minus']}, "
                  f"{fmt_p(m['conditions'][c]['p'])}" for c in conds]
        per = ["; ".join(f"+{s['plus']}/−{s['minus']}" + (f" ({fmt_p(s['p'])})" if s["p"] < 0.1 else "")
                         for s in m["conditions"][c]["per_seed"]) for c in conds]
        lines.append(f"| {name} | " + " | ".join(pooled) + " | " + " | ".join(per) + " |")
    lines += ["", "The same with the trip as the unit (a trip counts once, however many seeds drove it): the "
              "difference in success rate with a bootstrap 95 % interval over trips; trips where the share of "
              "seeds that finished beats the baseline (+) or loses to it (−), with a sign test; a paired "
              "sign-flip permutation test of the per-trip differences; and seeds with more + than − trips, "
              "and fewer.", "",
              "| method | " + " | ".join(f"{c}: difference [pp] (95 % CI), permutation p" for c in conds) + " | "
              + " | ".join(f"{c}: trips +/−" for c in conds) + " | "
              + " | ".join(f"{c}: seeds better/worse" for c in conds) + " |",
              "|---|" + "---|" * (3 * len(conds))]
    for name, m in a["methods"].items():
        if name == a["baseline"]:
            continue
        t = [m["conditions"][c]["trip_level"] for c in conds]
        lines.append(f"| {name} | " + " | ".join(f"{100 * x['diff']:+.1f} ({100 * x['ci'][0]:+.1f} to "
                                                  f"{100 * x['ci'][1]:+.1f}), {fmt_p(x['p_perm'])}" for x in t) + " | "
                     + " | ".join(f"+{x['plus']} / −{x['minus']}, {fmt_p(x['p'])}" for x in t) + " | "
                     + " | ".join(f"{m['conditions'][c]['seeds_better']} / {m['conditions'][c]['seeds_worse']}"
                                  f" of {len(m['conditions'][c]['per_seed'])}" for c in conds) + " |")
    c = "held_out" if "held_out" in conds else conds[0]
    lines += ["", f"Driving quality, {c} cars, over the trips each finished (pooled over seeds); trip time is the "
              "median ratio to the baseline over trips both finished.", "",
              "| method | mean offset [cm] | p95 offset [cm] | speed error [%] | steer roughness | energy [mWh/m] "
              "| wall contact [% trips] | left road | timed out | overshot | trip time |",
              "|---|---|---|---|---|---|---|---|---|---|---|"]
    for name, m in a["methods"].items():
        d = m["conditions"][c]
        lines.append(f"| {name} | {100 * d['offset_mean']:.1f} | {100 * d['offset_p95']:.1f} "
                     f"| {100 * d['speed_error']:.1f} | {d['steer_roughness']:.4f} | {1000 * d['energy_wh_per_m']:.2f} "
                     f"| {100 * d['wall_contact'] / d['trips']:.0f} | {d['left_road']} | {d['timed_out']} "
                     f"| {d['overshot']} | {d['time_ratio']:.2f} |")
    return "\n".join(lines) + "\n"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("report", type=pathlib.Path, help="evaluate_policies.py --out JSON")
    ap.add_argument("--baseline", required=True, help="the baseline's policy spec, as in the report")
    ap.add_argument("--out", type=pathlib.Path, help="writes <out>.md, <out>.json and <out>.outcomes.json")
    args = ap.parse_args(argv)
    report = json.loads(args.report.read_text(encoding="utf-8"))
    # model paths relative to this checkout, so the saved reports name no local folder
    prefix = str(ROOT) + "\\" if "\\" in str(ROOT) else str(ROOT) + "/"
    report["policies"] = {k.replace(prefix, "").replace("\\", "/"): v for k, v in report["policies"].items()}
    a = analyse(report, args.baseline)
    md = markdown(a)
    sys.stdout.buffer.write(md.encode("utf-8"))
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.with_suffix(".md").write_text(md, encoding="utf-8")
        args.out.with_suffix(".json").write_text(json.dumps(a, indent=1) + "\n", encoding="utf-8")
        pathlib.Path(str(args.out) + ".outcomes.json").write_text(json.dumps(outcomes(report)) + "\n",
                                                                  encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
