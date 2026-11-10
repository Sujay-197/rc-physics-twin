#!/usr/bin/env python3
"""Evaluate the RL seeds (Docs/rl-seeds.md), resumably.

Each comparison runs sim_py/evaluate_policies.py of a given checkout, with
that checkout's Core: the follow task on e552c96 (the policy study's commit),
the Unity car on 47175b9 (the second round), and both again on this checkout
(today's Core) as a check. The study-era evaluators have no --seed option, so
the first trip's seed is set here, by calling their evaluate() with it; the
trips are otherwise the evaluator's own (seed 20000 is its default).

A comparison whose report exists is skipped. Reports go to
results/rl_seeds/eval/<name>.json (every episode); tools/rl_seeds_report.py
summarises them.

    python tools/rl_seeds_eval.py [--only NAME ...] [--dry-run]
"""

from __future__ import annotations

import argparse
import functools
import importlib.util
import json
import os
import pathlib
import subprocess
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parent.parent
OUT = ROOT / "results" / "rl_seeds" / "eval"
TREES = {"e552c96": ROOT.parent / f"{ROOT.name}-e552c96", "47175b9": ROOT.parent / f"{ROOT.name}-47175b9",
         "today": ROOT}
SEEDS = range(5)
UNITY_GAINS = "policies/controllers_unity_stop.json"


def sb3(folder: str, name: str) -> list[str]:
    return [f"sb3:{ROOT / folder / f'{name}-s{s}'}" for s in SEEDS]


FOLLOW = ["controller:pure_pursuit", *sb3("policies/study", "sac"), *sb3("policies/study", "sac-res"),
          *sb3("policies/study", "ppo"), *sb3("policies/study", "ppo-res")]
UNITY_BASE = f"recover:controller:pure_pursuit:{UNITY_GAINS}"
UNITY = [UNITY_BASE, f"controller:pure_pursuit:{UNITY_GAINS}",
         *[f"recover:{s}" for s in sb3("policies/twin-car/seeds", "ppo-res-stop-unity-r3")],
         *sb3("policies/twin-car/seeds", "ppo-res-stop-unity-r3"),
         *sb3("policies/twin-car/seeds", "ppo-stop-unity-r3"),
         *[f"recover:{s}" for s in sb3("policies/twin-car/seeds", "ppo-stop-unity-r3")]]
CORNER = f"recover:corner:3.4,1.2,1.2,0.5:controller:pure_pursuit:{UNITY_GAINS}"   # today's shipped autopilot

# name: (tree, first seed, extra evaluate_policies.py arguments, specs), in the order they run
RUNS = {
    "follow_e552c96_20000": ("e552c96", 20000, [], FOLLOW),
    "unity_47175b9_20000": ("47175b9", 20000, ["--task", "stop", "--car", "unity"], UNITY),
    "follow_today_20000": ("today", 20000, [], FOLLOW),
    "unity_today_20000": ("today", 20000, ["--task", "stop", "--car", "unity"], [*UNITY, CORNER]),
    "follow_e552c96_50000": ("e552c96", 50000, [], FOLLOW),
    "unity_47175b9_50000": ("47175b9", 50000, ["--task", "stop", "--car", "unity"], UNITY),
}


def run_in_tree(tree: pathlib.Path, seed: int, argv: list[str]) -> int:
    """evaluate_policies.main(argv) of that checkout, with its first trip at seed."""
    sim = tree / "sim_py"
    sys.path.insert(0, str(sim))
    os.chdir(tree)
    spec = importlib.util.spec_from_file_location("evaluate_policies", sim / "evaluate_policies.py")
    ev = importlib.util.module_from_spec(spec)
    sys.modules["evaluate_policies"] = ev          # so spawned workers find _job by this name
    spec.loader.exec_module(ev)
    ev.evaluate = functools.partial(ev.evaluate, seed=seed)
    return ev.main(argv)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--only", nargs="+", choices=list(RUNS))
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--child", nargs=3, metavar=("TREE", "SEED", "ARGV_JSON"), help=argparse.SUPPRESS)
    args = ap.parse_args(argv)
    if args.child:
        return run_in_tree(pathlib.Path(args.child[0]), int(args.child[1]), json.loads(args.child[2]))
    OUT.mkdir(parents=True, exist_ok=True)
    for name, (tree_id, seed, extra, specs) in RUNS.items():
        if args.only and name not in args.only:
            continue
        out = OUT / f"{name}.json"
        if out.exists():
            print(f"skip {name}: done", flush=True)
            continue
        tree = TREES[tree_id]
        if tree_id != "today":
            head = subprocess.run(["git", "-C", str(tree), "rev-parse", "HEAD"], capture_output=True,
                                  text=True).stdout.strip()
            if not head.startswith(tree_id):
                raise SystemExit(f"{tree} is at {head or '?'}, not {tree_id}")
        ev_argv = [*specs, "--trips", "100", "--target-speed", "4", *extra, "--out", str(out)]
        cmd = [sys.executable, "-u", str(pathlib.Path(__file__).resolve()), "--child", str(tree), str(seed),
               json.dumps(ev_argv)]
        print(f"{time.strftime('%Y-%m-%d %H:%M:%S')} run  {name} (tree {tree_id}, seed {seed})", flush=True)
        if args.dry_run:
            print("   ", " ".join(cmd))
            continue
        with open(OUT.parent / "commands.txt", "a", encoding="utf-8") as f:
            f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} eval {name}: {' '.join(cmd)}\n")
        t0 = time.perf_counter()
        with open(OUT / f"{name}.log", "w", encoding="utf-8") as log:
            rc = subprocess.run(cmd, cwd=tree, stdout=log, stderr=subprocess.STDOUT).returncode
        print(f"{time.strftime('%Y-%m-%d %H:%M:%S')} {'done' if rc == 0 else f'FAILED ({rc})'} {name} "
              f"in {time.perf_counter() - t0:.0f} s", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
