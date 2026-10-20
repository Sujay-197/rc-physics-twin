#!/usr/bin/env python3
"""Tune the classical controllers (sim_py/controllers.py) by grid search.

Each controller's gains, including its cornering limit, are searched on the
training randomisation (RANGES_TRAIN) with seeds of their own (TUNE_SEED
onward, disjoint from the evaluation's), ranked by success, then fewest wall
contacts, then mean return, and the winners are frozen to
policies/controllers.json. The held-out cars and the perturbation are not
used: they are the test.

    python tools/tune_controllers.py --target-speed 4 --trips 60
"""

from __future__ import annotations

import argparse
import itertools
import json
import os
import pathlib
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parent.parent
TUNE_SEED = 30_000

# Imported by name, so the evaluation's worker processes can unpickle its jobs.
sys.path.insert(0, str(ROOT / "sim_py"))
import evaluate_policies as ev  # noqa: E402

GRIDS = {
    "pure_pursuit": {"lookahead": [0.6, 0.8, 1.0, 1.2, 1.5, 1.8, 2.0], "k_speed": [2.0, 4.0, 6.0],
                     "a_lat": [None, 3.0, 5.0, 8.0, 12.0]},
    # The first grids put Stanley's k and LQR's q_e and r on their edges, so
    # these reach past them both ways.
    "stanley": {"k": [1.0, 2.0, 4.0, 8.0, 12.0, 16.0, 24.0], "k_soft": [0.25, 1.0, 2.0],
                "k_damp": [0.0, 0.03, 0.06, 0.1], "a_lat": [None, 5.0, 8.0, 12.0]},
    "lqr": {"q": [[qe, 1.0, qp, 0.1] for qe in (1.0, 3.0, 10.0, 30.0, 100.0) for qp in (0.3, 1.0, 3.0, 10.0)],
            "r": [1.0, 3.0, 10.0, 30.0], "a_lat": [None, 5.0, 8.0, 12.0]},
}


def configs(kind: str):
    grid = GRIDS[kind]
    keys = list(grid)
    for values in itertools.product(*(grid[k] for k in keys)):
        yield {k: v for k, v in zip(keys, values)}


def rank_key(summary: dict):
    return (summary["success"][0], -summary["wall_contact"][0], summary["return"][0])


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--target-speed", type=float, default=4.0)
    ap.add_argument("--trips", type=int, default=60)
    ap.add_argument("--kinds", nargs="+", default=list(GRIDS))
    ap.add_argument("--out", type=pathlib.Path, default=ROOT / "policies" / "controllers.json")
    ap.add_argument("--lib")
    args = ap.parse_args(argv)
    lib = os.path.abspath(args.lib) if args.lib else None

    out = json.loads(args.out.read_text(encoding="utf-8")) if args.out.exists() else {}
    for kind in args.kinds:
        t0 = time.perf_counter()
        cands = list(configs(kind))
        specs = [f"controller:{kind}:{json.dumps(c, separators=(',', ':'))}" for c in cands]
        rep = ev.evaluate(specs, trips=args.trips, conditions=("train",), lib=lib, seed=TUNE_SEED,
                          target_speed=args.target_speed)
        scored = sorted(((rank_key(rep["policies"][sp]["train"]["summary"]), sp, c)
                         for sp, c in zip(specs, cands)), key=lambda t: t[0], reverse=True)
        (best_key, _, best), (second_key, _, _) = scored[0], scored[1]
        s = rep["policies"][scored[0][1]]["train"]["summary"]
        out[kind] = {"gains": best, "target_speed": args.target_speed,
                     "tuning": {"trips": args.trips, "seed": TUNE_SEED, "candidates": len(cands),
                                "condition": "train", "success": s["success"][0],
                                "wall_contact": s["wall_contact"][0], "return": s["return"][0],
                                "runner_up": {"success": second_key[0], "wall_contact": -second_key[1],
                                              "return": second_key[2]}}}
        print(f"{kind}: {len(cands)} candidates in {time.perf_counter() - t0:.0f} s; best {best} "
              f"success {s['success'][0]:.2f}, walls {s['wall_contact'][0]:.2f}, return {s['return'][0]:.0f}",
              flush=True)
        for k, sp, c in scored[:5]:
            print(f"    {k[0]:.2f} {-k[1]:.2f} {k[2]:8.1f}  {c}")
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(out, indent=1) + "\n", encoding="utf-8")
    print(f"wrote {args.out}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
