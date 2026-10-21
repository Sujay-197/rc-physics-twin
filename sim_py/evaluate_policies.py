"""Evaluate driving policies on the campus, all on the same footing.

Every policy drives the same trips with the same cars, in three conditions:

  train      RANGES_TRAIN, the randomisation the learned policies trained on
  held_out   RANGES_HELD_OUT, cars outside it (less grip, heavier, slower
             servo, weaker motor, softer tyres, higher battery resistance)
  perturbed  RANGES_TRAIN, then 4 m into the trip the car changes at once:
             grip x0.6, +1 kg, servo bandwidth x0.6

Trips are drawn by demand weight as CampusEnv draws them. The campus has 90
stop pairs, so a learned policy has seen every route in training: what it has
not seen is the car (held_out) or a change mid-trip (perturbed). The seeds
here (EVAL_SEED onward) are not the tuning seeds of tools/tune_controllers.py.

Per trip: outcome (reached the end, left the road, timed out), any wall
contact, mean |speed error| / target, mean and 95th-percentile |offset|,
steering roughness (mean |change in steer command| per tick), energy per
metre, and the environment's return. Aggregates carry 95 % intervals (Wilson
for rates, normal for means).

Policy specs (strings, so worker processes can build them):
  controller:<kind>            tuned gains from policies/controllers.json
  controller:<kind>:default    the untuned defaults
  controller:<kind>:{json}     explicit gains, as tools/tune_controllers.py uses
  controller:<kind>:<file.json>  tuned gains from another tuning, e.g.
                               policies/controllers_unity_stop.json
  recover:<spec>               <spec> with the kerb recovery (controllers.KerbRecovery)
  cem:<path.json>              sim_py/train_cem.py
  sb3:<prefix>                 sim_py/train_rl.py output (needs the .venv)

    python sim_py/evaluate_policies.py controller:pure_pursuit controller:lqr --trips 100
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import multiprocessing as mp
import os
import pathlib
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

EVAL_SEED = 20_000
PERTURBATION = {"at_s": 4.0, "friction": 0.6, "payload_kg": 1.0, "servo_bandwidth": 0.6}
CONDITIONS = ("train", "held_out", "perturbed")


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


_campus = None


def campus():
    global _campus
    if _campus is None:
        _campus = _load("campus", HERE / "campus.py")
    return _campus


def ranges_for(condition: str):
    envmod = campus().envmod
    return envmod.RANGES_HELD_OUT if condition == "held_out" else envmod.RANGES_TRAIN


def options_for(condition: str):
    return {"perturb": dict(PERTURBATION)} if condition == "perturbed" else None


# ------------------------------------------------------------------ policies

def build_policy(spec: str, env, any_car: bool = False):
    """A policy from its spec, for this env (its nominal car, target speed).
    any_car lets a learned policy drive a car it did not train on."""
    envmod = campus().envmod
    kind, _, rest = spec.partition(":")
    if kind == "recover":
        ctl = _load("controllers", HERE / "controllers.py")
        return ctl.KerbRecovery(build_policy(rest, env, any_car), envmod, env.target_speed)
    if kind == "controller":
        ctl = _load("controllers", HERE / "controllers.py")
        name, _, variant = rest.partition(":")
        gains = {}
        if variant.startswith("{"):
            gains = json.loads(variant)          # explicit gains (tuning)
        elif variant.endswith(".json"):
            tuned = json.loads((ROOT / variant).read_text(encoding="utf-8"))
            gains = tuned[name]["gains"]
        elif variant != "default":
            tuned = json.loads((ROOT / "policies" / "controllers.json").read_text(encoding="utf-8"))
            gains = tuned[name]["gains"]
        nominal = env.nominal_params()
        return ctl.build(name, envmod, nominal, 0.5 * env.network.width, env.target_speed, task=env.task, **gains)
    if kind == "cem":
        cem = _load("train_cem", HERE / "train_cem.py")
        return cem.LinearPolicy.from_json(json.loads(pathlib.Path(rest).read_text(encoding="utf-8")))
    if kind == "sb3":
        rl = _load("train_rl", HERE / "train_rl.py")
        return rl.load_policy(pathlib.Path(rest), env, any_car=any_car)
    raise ValueError(f"unknown policy spec {spec!r}")


# ------------------------------------------------------------------- episode

def run_episode(env, policy, seed: int, options=None) -> dict:
    obs, info = env.reset(seed=seed, options=options)
    if hasattr(policy, "reset"):
        policy.reset()
    v_t = env.target_speed
    ticks, ret, v_err, off, offs, rough = 0, 0.0, 0.0, 0.0, [], 0.0
    last_steer, s0 = None, env.s
    done = False
    while not done:
        a = policy(obs)
        obs, r, te, tr, info = env.step(a)
        ret += r
        ticks += 1
        v_err += abs(env.state.vx - v_t) / v_t
        o = abs(info["offset"])
        off += o
        offs.append(o)
        steer = max(-1.0, min(1.0, float(a[0])))
        if last_steer is not None:
            rough += abs(steer - last_steer)
        last_steer = steer
        done = te or tr
    offs.sort()
    dist = max(1e-6, env.s - s0)
    return {"seed": seed, "trip": list(info.get("trip") or ()), "status": info["status"],
            "wall": info["wall_contacts"] > 0, "return": ret, "ticks": ticks,
            "speed_error": v_err / ticks, "offset_mean": off / ticks,
            "offset_p95": offs[min(len(offs) - 1, int(0.95 * len(offs)))],
            "steer_roughness": rough / max(1, ticks - 1), "energy_wh_per_m": info["energy_wh"] / dist,
            "back_offs": getattr(policy, "back_offs", 0)}


def _job(args):
    spec, condition, seeds, lib, target_speed, task, car, any_car = args
    c = campus()
    extra = {}
    inner = spec[len("recover:"):] if spec.startswith("recover:") else spec
    if inner.startswith("sb3:"):
        # A learned policy sees the observation it trained on (history 0 for a
        # recurrent one); the harness checks it trained at this speed.
        cfg = json.loads(pathlib.Path(inner[4:]).with_suffix(".json").read_text(encoding="utf-8"))["config"]
        extra["history"] = cfg["history"]
    env = c.CampusEnv(seed=seeds[0], ranges=ranges_for(condition), lib_path=lib, target_speed=target_speed,
                      task=task, car=car, **extra)
    policy = build_policy(spec, env, any_car)
    out = [run_episode(env, policy, s, options_for(condition)) for s in seeds]
    env.close()
    return spec, condition, out


# --------------------------------------------------------------- statistics

def wilson(k: int, n: int, z: float = 1.96):
    if n == 0:
        return (0.0, 0.0, 0.0)
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (p, max(0.0, c - h), min(1.0, c + h))


def mean_ci(xs, z: float = 1.96):
    n = len(xs)
    if n == 0:
        return (float("nan"),) * 3
    m = sum(xs) / n
    if n == 1:
        return (m, m, m)
    sd = math.sqrt(sum((x - m) ** 2 for x in xs) / (n - 1))
    h = z * sd / math.sqrt(n)
    return (m, m - h, m + h)


def summarise(episodes: list) -> dict:
    n = len(episodes)
    ok = [e for e in episodes if e["status"] == "reached_end"]
    out = {"trips": n,
           "success": wilson(len(ok), n),
           "left_road": wilson(sum(e["status"] == "left_road" for e in episodes), n),
           "overshot": wilson(sum(e["status"] == "overshot" for e in episodes), n),
           "wall_contact": wilson(sum(e["wall"] for e in episodes), n),
           "return": mean_ci([e["return"] for e in episodes])}
    # Driving quality over the trips that finished, so a crash does not
    # flatter a policy by ending its worst stretch early.
    for k in ("speed_error", "offset_mean", "offset_p95", "steer_roughness", "energy_wh_per_m"):
        out[k] = mean_ci([e[k] for e in ok])
    return out


# --------------------------------------------------------------------- main

def evaluate(specs, trips: int = 100, conditions=CONDITIONS, lib=None, workers=None, chunk: int = 10,
             seed: int = EVAL_SEED, target_speed: float = 2.0, task: str = "follow", car: str = "bare",
             any_car: bool = False) -> dict:
    seeds = list(range(seed, seed + trips))
    jobs = [(spec, cond, seeds[i:i + chunk], lib, target_speed, task, car, any_car) for spec in specs for cond in conditions
            for i in range(0, trips, chunk)]
    workers = workers or max(1, (os.cpu_count() or 4) - 2)
    results = {spec: {cond: [] for cond in conditions} for spec in specs}
    if workers == 1:
        done = map(_job, jobs)
    else:
        pool = mp.get_context("spawn").Pool(workers)
        done = pool.imap_unordered(_job, jobs)
    for spec, cond, eps in done:
        results[spec][cond].extend(eps)
    if workers != 1:
        pool.close()
        pool.join()
    for spec in specs:
        for cond in conditions:
            results[spec][cond].sort(key=lambda e: e["seed"])
    return {"protocol": {"trips": trips, "seed": seed, "conditions": list(conditions),
                         "target_speed": target_speed, "task": task, "car": car,
                         "perturbation": PERTURBATION},
            "policies": {spec: {cond: {"summary": summarise(eps), "episodes": eps}
                                for cond, eps in conds.items()} for spec, conds in results.items()}}


def table(report: dict) -> str:
    """A Markdown table: one row per policy and condition."""
    def pct(t):
        return f"{100 * t[0]:.0f} % ({100 * t[1]:.0f}-{100 * t[2]:.0f})"

    def num(t, f="{:.3f}"):
        return "n/a" if t[0] != t[0] else f.format(t[0]) + " +/- " + f.format(t[2] - t[0])

    rows = ["| policy | condition | success | wall contact | return | speed error | offset mean [m] "
            "| offset p95 [m] | steer roughness | energy [Wh/m] |",
            "|---|---|---|---|---|---|---|---|---|---|"]
    for spec, conds in report["policies"].items():
        for cond, d in conds.items():
            s = d["summary"]
            rows.append(f"| {spec} | {cond} | {pct(s['success'])} | {pct(s['wall_contact'])} "
                        f"| {num(s['return'], '{:.0f}')} | {num(s['speed_error'])} | {num(s['offset_mean'])} "
                        f"| {num(s['offset_p95'])} | {num(s['steer_roughness'], '{:.4f}')} "
                        f"| {num(s['energy_wh_per_m'], '{:.5f}')} |")
    return "\n".join(rows)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("specs", nargs="+")
    ap.add_argument("--trips", type=int, default=100)
    ap.add_argument("--conditions", nargs="+", default=list(CONDITIONS), choices=CONDITIONS)
    ap.add_argument("--workers", type=int)
    ap.add_argument("--target-speed", type=float, default=2.0, help="[m/s]")
    ap.add_argument("--task", choices=("follow", "stop"), default="follow", help="rc_twin_env.py's tasks")
    ap.add_argument("--car", choices=("bare", "unity", "both"), default="bare",
                    help="rc_twin_env.py's cars: bare (the policy study's), unity (the twin's), both")
    ap.add_argument("--any-car", action="store_true",
                    help="let learned policies drive a car they did not train on (to measure the gap)")
    ap.add_argument("--out", type=pathlib.Path, help="JSON report (episodes included)")
    ap.add_argument("--lib")
    args = ap.parse_args(argv)
    lib = os.path.abspath(args.lib) if args.lib else None
    t0 = time.perf_counter()
    report = evaluate(args.specs, args.trips, args.conditions, lib, args.workers,
                      target_speed=args.target_speed, task=args.task, car=args.car, any_car=args.any_car)
    report["seconds"] = round(time.perf_counter() - t0, 1)
    print(table(report))
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(report, indent=1) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
