#!/usr/bin/env python3
"""Run the policy study's training queue (Docs/policy-study.md), resumably.

Every method trains with three seeds. The queue runs seed 0 of every method
first, so a complete first comparison exists early, then seeds 1 and 2. A run
whose model already exists is skipped, so the queue can be stopped and
restarted. Each run logs to results/study_logs/<name>.log.

Budgets are set per method from measured throughput on the development
machine (16 threads, RTX 5060 Ti), aiming at comparable wall-clock per run:
  ppo        30M steps   ~16-30k steps/s, 14 processes x 8 envs, GPU updates
  ppo-res    20M
  sac         2.5M       ~0.9k steps/s: batch 4096, 1 gradient step per 16 env steps
  sac-res     2M
  td3-res     2M
  rppo        8M         ~2.4k steps/s, LSTM on the CPU
  cem         40 iterations (standard library; sim_py/train_cem.py)

    .venv/Scripts/python tools/run_study.py [--dry-run] [--only ppo-res]
"""

from __future__ import annotations

import argparse
import pathlib
import subprocess
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parent.parent
PY = str(ROOT / ".venv" / "Scripts" / "python.exe")
STUDY = ROOT / "policies" / "study"
LOGS = ROOT / "results" / "study_logs"
SPEED = "4"
SEEDS = (0, 1, 2)
OFF_POLICY = ["--batch", "4096", "--utd", "0.0625", "--learning-starts", "20000"]

METHODS = {
    "ppo": ["--algo", "ppo", "--timesteps", "30000000"],
    "ppo-res": ["--algo", "ppo", "--residual", "--timesteps", "20000000"],
    "sac": ["--algo", "sac", "--timesteps", "2500000", *OFF_POLICY],
    "sac-res": ["--algo", "sac", "--residual", "--timesteps", "2000000", *OFF_POLICY],
    "td3-res": ["--algo", "td3", "--residual", "--timesteps", "2000000", *OFF_POLICY],
    "rppo": ["--algo", "rppo", "--timesteps", "8000000", "--device", "cpu"],
    "cem": None,
}


def command(method: str, seed: int) -> tuple[list, pathlib.Path]:
    out = STUDY / f"{method}-s{seed}"
    if method == "cem":
        return ([PY, "-I", "-u", str(ROOT / "sim_py" / "train_cem.py"), "--iterations", "40", "--population", "24",
                 "--elite", "6", "--episodes", "4", "--eval-episodes", "0", "--seed", str(seed),
                 "--target-speed", SPEED, "--out", str(out.with_suffix(".json"))], out.with_suffix(".json"))
    return ([PY, "-I", "-u", str(ROOT / "sim_py" / "train_rl.py"), *METHODS[method], "--seed", str(seed),
             "--target-speed", SPEED, "--out", str(out)], out.with_suffix(".json"))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--only", nargs="+", choices=list(METHODS))
    args = ap.parse_args(argv)
    methods = args.only or list(METHODS)
    queue = [(m, s) for s in SEEDS for m in methods]
    LOGS.mkdir(parents=True, exist_ok=True)
    for method, seed in queue:
        cmd, done_marker = command(method, seed)
        name = f"{method}-s{seed}"
        if done_marker.exists():
            print(f"skip {name}: done", flush=True)
            continue
        print(f"{time.strftime('%H:%M:%S')} run  {name}", flush=True)
        if args.dry_run:
            print("   ", " ".join(cmd))
            continue
        t0 = time.perf_counter()
        with open(LOGS / f"{name}.log", "w", encoding="utf-8") as log:
            rc = subprocess.run(cmd, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT).returncode
        print(f"{time.strftime('%H:%M:%S')} {'done' if rc == 0 else f'FAILED ({rc})'} {name} "
              f"in {time.perf_counter() - t0:.0f} s", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
