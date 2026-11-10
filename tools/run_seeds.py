#!/usr/bin/env python3
"""More training seeds for the policy study's learned-vs-classical comparison
(Docs/rl-seeds.md), resumably.

Only seeds are added: every run uses its method's original command, with the
same budget, reward, car, validation selection and hyperparameters
(tools/run_study.py for the follow task; Docs/policy-study.md, "A second
round", for the Unity car), and the code and Core its runs trained on. The
Core has changed since (ADR 0010's ESC reverse frees kerb-pinned cars, ADR
0011's low-speed blend), so each group trains in a checkout of its commit:

  follow  e552c96, the policy study's commit (before ADR 0010)
  unity   47175b9, reward v3 (the second round; before ADR 0011)

each with its own build/bin/rc_twin.dll (git worktree add --detach <tree>
<commit>, then the README's cmake build). Both reproduce their recorded
baselines episode for episode. The models are written into this checkout.

Two groups:

  follow  bare car, follow task (the study's first table): SAC + history and
          residual SAC from 1 seed to 3, then 5; PPO + history and residual
          PPO from 3 to 5. Seed 0 (and PPO's 1, 2) are the study's own models,
          in policies/study/, re-evaluated as they are.
  unity   the Unity car, stop task, reward v3, residual on the Unity-car gains
          (the second round on the Unity car, the setup the twin keeps:
          --val-edge stays off). Its models were not kept, so all five seeds
          train again, under policies/twin-car/seeds/, beside the round's
          recorded configs (which are left as they are).

The queue runs the priority jobs first. A run whose model JSON exists is
skipped, so the queue can be stopped and restarted. Each run logs to
results/rl_seeds/logs/<name>.log, and the queue writes the command of every
run it starts to results/rl_seeds/commands.txt.

    python tools/run_seeds.py [--dry-run] [--only follow|unity] [--priority N]
"""

from __future__ import annotations

import argparse
import pathlib
import subprocess
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parent.parent
LOGS = ROOT / "results" / "rl_seeds" / "logs"
SPEED = "4"
OFF_POLICY = ["--batch", "4096", "--utd", "0.0625", "--learning-starts", "20000"]   # as tools/run_study.py
UNITY = ["--task", "stop", "--car", "unity"]

TREES = {"follow": "e552c96", "unity": "47175b9"}

# name prefix, output folder, train_rl.py arguments
METHODS = {
    "sac": ("policies/study", "sac-s{seed}", ["--algo", "sac", "--timesteps", "2500000", *OFF_POLICY]),
    "sac-res": ("policies/study", "sac-res-s{seed}",
                ["--algo", "sac", "--residual", "--timesteps", "2000000", *OFF_POLICY]),
    "ppo": ("policies/study", "ppo-s{seed}", ["--algo", "ppo", "--timesteps", "30000000"]),
    "ppo-res": ("policies/study", "ppo-res-s{seed}", ["--algo", "ppo", "--residual", "--timesteps", "20000000"]),
    "unity-res": ("policies/twin-car/seeds", "ppo-res-stop-unity-r3-s{seed}",
                  ["--algo", "ppo", "--residual", "--residual-gains", "policies/controllers_unity_stop.json",
                   "--timesteps", "20000000", *UNITY]),
    "unity-ppo": ("policies/twin-car/seeds", "ppo-stop-unity-r3-s{seed}",
                  ["--algo", "ppo", "--timesteps", "45000000", *UNITY]),
}

# (priority, method, seed), in the order they run
QUEUE = [
    # 1. the single-seed methods to 3 seeds
    (1, "sac", 1), (1, "sac-res", 1), (1, "sac", 2), (1, "sac-res", 2),
    # 2. the Unity car's second round to 5 seeds (all retrained: the models were not kept)
    *[(2, "unity-res", s) for s in range(5)], *[(2, "unity-ppo", s) for s in range(5)],
    # 3. if time allows: the follow task's methods to 5 seeds
    (3, "sac", 3), (3, "sac-res", 3), (3, "sac", 4), (3, "sac-res", 4),
    (3, "ppo", 3), (3, "ppo-res", 3), (3, "ppo", 4), (3, "ppo-res", 4),
]
GROUP = {"follow": ("sac", "sac-res", "ppo", "ppo-res"), "unity": ("unity-res", "unity-ppo")}


def group_of(method: str) -> str:
    return next(g for g, ms in GROUP.items() if method in ms)


def tree_for(group: str, trees: dict) -> pathlib.Path:
    """The checkout of the group's commit, checked: that commit, with its DLL."""
    import subprocess as sp
    tree = pathlib.Path(trees.get(group) or ROOT.parent / f"{ROOT.name}-{TREES[group]}")
    head = sp.run(["git", "-C", str(tree), "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
    if not head.startswith(TREES[group]):
        raise SystemExit(f"{tree} is at {head or '?'}, not {TREES[group]} ({group})")
    if not (tree / "build" / "bin" / "rc_twin.dll").exists():
        raise SystemExit(f"{tree} has no build/bin/rc_twin.dll: build it first")
    return tree


def command(method: str, seed: int, tree: pathlib.Path) -> tuple[list, pathlib.Path, str]:
    folder, pattern, args = METHODS[method]
    name = pattern.format(seed=seed)
    out = ROOT / folder / name
    cmd = [sys.executable, "-I", "-u", str(tree / "sim_py" / "train_rl.py"), *args, "--seed", str(seed),
           "--target-speed", SPEED, "--out", str(out)]
    return cmd, out.with_suffix(".json"), name


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--only", choices=list(GROUP))
    ap.add_argument("--priority", type=int, default=3, help="run jobs up to this priority")
    ap.add_argument("--follow-tree", help=f"checkout of {TREES['follow']} (default ../<this>-{TREES['follow']})")
    ap.add_argument("--unity-tree", help=f"checkout of {TREES['unity']} (default ../<this>-{TREES['unity']})")
    args = ap.parse_args(argv)
    trees = {"follow": args.follow_tree, "unity": args.unity_tree}
    LOGS.mkdir(parents=True, exist_ok=True)
    for prio, method, seed in QUEUE:
        if prio > args.priority or (args.only and method not in GROUP[args.only]):
            continue
        tree = tree_for(group_of(method), trees)
        cmd, marker, name = command(method, seed, tree)
        if marker.exists():
            print(f"skip {name}: done", flush=True)
            continue
        line = " ".join(cmd)
        print(f"{time.strftime('%Y-%m-%d %H:%M:%S')} run  {name}", flush=True)
        if args.dry_run:
            print("   ", line)
            continue
        with open(LOGS.parent / "commands.txt", "a", encoding="utf-8") as f:
            f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} {name}: {line}\n")
        marker.parent.mkdir(parents=True, exist_ok=True)
        t0 = time.perf_counter()
        with open(LOGS / f"{name}.log", "w", encoding="utf-8") as log:
            rc = subprocess.run(cmd, cwd=tree, stdout=log, stderr=subprocess.STDOUT).returncode
        print(f"{time.strftime('%Y-%m-%d %H:%M:%S')} {'done' if rc == 0 else f'FAILED ({rc})'} {name} "
              f"in {time.perf_counter() - t0:.0f} s", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
