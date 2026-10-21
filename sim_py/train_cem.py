"""Train a driving policy on the campus with the cross-entropy method,
standard library only.

The policy is linear in the current observation frame (FRAME_FIELDS: speed,
slip, offset, heading error, the centreline ahead, ...), squashed by tanh:

    action = tanh(W . frame + b)      W: 2 x FRAME_SIZE, b: 2

Each iteration samples a population of weight vectors from a diagonal
Gaussian, scores each by its mean return over the same few campus trips and
randomised cars (common random numbers, so candidates are compared on equal
terms), and refits the Gaussian to the best ones. No numpy, gymnasium or
torch: it runs wherever rc_twin.dll does. It is a dependency-free baseline
learner, not a replacement for PPO or SAC (make_gymnasium_env and SB3, once
installed): a linear policy cannot use the observation history, so it cannot
adapt to the car the way a history-conditioned network can.

    python sim_py/train_cem.py --iterations 40 --out policies/campus_cem.json
    python sim_py/train_cem.py --evaluate policies/campus_cem.json

Training uses RANGES_TRAIN; the evaluation reports held-out randomisation
(RANGES_HELD_OUT) next to the pure-pursuit baseline, on trips the training
never drew (different seeds).
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import pathlib
import random
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
_spec = importlib.util.spec_from_file_location("campus", HERE / "campus.py")
campus = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(campus)
envmod = campus.envmod

FORMAT = "rc-physics-twin linear policy 0.1.0"
N_IN = envmod.FRAME_SIZE


class LinearPolicy:
    """action = tanh(W . frame + b), on the observation's current frame."""

    def __init__(self, theta: list[float]):
        if len(theta) != 2 * (N_IN + 1):
            raise ValueError(f"expected {2 * (N_IN + 1)} weights, got {len(theta)}")
        self.theta = list(theta)

    @staticmethod
    def size() -> int:
        return 2 * (N_IN + 1)

    def __call__(self, obs) -> tuple[float, float]:
        out = []
        for a in range(2):
            w = self.theta[a * (N_IN + 1): (a + 1) * (N_IN + 1)]
            z = w[N_IN]
            for i in range(N_IN):
                z += w[i] * obs[i]
            out.append(math.tanh(z))
        return out[0], out[1]

    def to_json(self, meta: dict) -> dict:
        return {"format": FORMAT, "frame_fields": list(envmod.FRAME_FIELDS),
                "steer": self.theta[: N_IN + 1], "throttle": self.theta[N_IN + 1:], **meta}

    @classmethod
    def from_json(cls, d: dict) -> "LinearPolicy":
        if d.get("format") != FORMAT:
            raise ValueError(f"not a {FORMAT!r} file")
        if d["frame_fields"] != list(envmod.FRAME_FIELDS):
            raise ValueError("the policy was trained on a different observation frame")
        return cls(list(d["steer"]) + list(d["throttle"]))


def episode(env, policy, seed: int) -> tuple[float, dict]:
    obs, info = env.reset(seed=seed)
    ret, done = 0.0, False
    while not done:
        obs, r, te, tr, info = env.step(policy(obs))
        ret += r
        done = te or tr
    return ret, info


def score(env, policy, seeds) -> float:
    return sum(episode(env, policy, s)[0] for s in seeds) / len(seeds)


def train(env, iterations: int, population: int, elite: int, episodes: int, seed: int,
          sigma0: float = 0.5, sigma_min: float = 0.02, log=print):
    """The cross-entropy method. Returns (the final mean as a policy, history).

    The final mean, not the best single candidate seen: each iteration scores
    on different trips, so the best-ever score mostly picks the iteration
    that drew the easiest trips."""
    rng = random.Random(seed)
    n = LinearPolicy.size()
    mu, sigma = [0.0] * n, [sigma0] * n
    history = []
    for it in range(iterations):
        # The same trips and cars for every candidate this iteration.
        seeds = [rng.randrange(1 << 30) for _ in range(episodes)]
        cands = [[m + s * rng.gauss(0.0, 1.0) for m, s in zip(mu, sigma)] for _ in range(population)]
        cands.append(list(mu))   # the current mean competes too
        scored = sorted(((score(env, LinearPolicy(c), seeds), k, c) for k, c in enumerate(cands)),
                        key=lambda t: (-t[0], t[1]))
        top = [c for _, _, c in scored[:elite]]
        mu = [sum(c[i] for c in top) / elite for i in range(n)]
        sigma = [max(sigma_min, math.sqrt(sum((c[i] - mu[i]) ** 2 for c in top) / elite)) for i in range(n)]
        history.append({"iteration": it, "best": scored[0][0], "elite_mean": sum(s for s, _, _ in scored[:elite]) / elite,
                        "sigma_mean": sum(sigma) / n})
        log(f"iter {it:3d}  best {scored[0][0]:8.2f}  elite mean {history[-1]['elite_mean']:8.2f}  "
            f"sigma {history[-1]['sigma_mean']:.3f}")
    return LinearPolicy(mu), history


def evaluate(policy, episodes: int, seed: int, ranges, lib=None) -> dict:
    """Success, wall strikes, and how it drove: speed and offset over every
    tick (the task is the target speed on the centreline, so both matter)."""
    env = campus.CampusEnv(seed=seed, ranges=ranges, lib_path=lib)
    total, success, hits, ticks, v_sum, v_max, off_sum = 0.0, 0, 0, 0, 0.0, 0.0, 0.0
    for k in range(episodes):
        obs, info = env.reset(seed=seed + k)
        done = False
        while not done:
            obs, r, te, tr, info = env.step(policy(obs))
            total += r
            ticks += 1
            v_sum += env.state.vx
            v_max = max(v_max, env.state.vx)
            off_sum += abs(info["offset"])
            done = te or tr
        success += info["status"] == "reached_end"
        hits += info["wall_contacts"] > 0
    env.close()
    return {"episodes": episodes, "mean_return": total / episodes, "success_rate": success / episodes,
            "episodes_touching_a_wall": hits, "mean_speed_mps": v_sum / ticks, "max_speed_mps": v_max,
            "target_speed_mps": env.target_speed, "mean_abs_offset_m": off_sum / ticks}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--iterations", type=int, default=40)
    ap.add_argument("--population", type=int, default=24)
    ap.add_argument("--elite", type=int, default=6)
    ap.add_argument("--episodes", type=int, default=4, help="trips per candidate per iteration")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--max-time", type=float, default=45.0, help="[s] per training episode")
    ap.add_argument("--eval-episodes", type=int, default=30)
    ap.add_argument("--out", type=pathlib.Path, default=ROOT / "policies" / "campus_cem.json")
    ap.add_argument("--evaluate", type=pathlib.Path, help="evaluate a saved policy instead of training")
    ap.add_argument("--target-speed", type=float, default=2.0, help="[m/s]")
    ap.add_argument("--lib")
    args = ap.parse_args(argv)

    if args.evaluate:
        policy = LinearPolicy.from_json(json.loads(args.evaluate.read_text(encoding="utf-8")))
    else:
        env = campus.CampusEnv(seed=args.seed, ranges=envmod.RANGES_TRAIN, max_time=args.max_time,
                               lib_path=args.lib, target_speed=args.target_speed)
        t0 = time.perf_counter()
        policy, history = train(env, args.iterations, args.population, args.elite, args.episodes, args.seed)
        env.close()
        seconds = time.perf_counter() - t0

    # Evaluation on trips the training never drew: seeds far from its own.
    # --eval-episodes 0 skips it (sim_py/evaluate_policies.py does the study's).
    eval_seed = 10_000 + args.seed
    report = None if args.eval_episodes == 0 else {
        "trained_policy": {
            "train_ranges": evaluate(policy, args.eval_episodes, eval_seed, envmod.RANGES_TRAIN, args.lib),
            "held_out_ranges": evaluate(policy, args.eval_episodes, eval_seed, envmod.RANGES_HELD_OUT, args.lib),
        },
        "pure_pursuit_baseline": {
            "train_ranges": evaluate(envmod.pure_pursuit_policy, args.eval_episodes, eval_seed,
                                     envmod.RANGES_TRAIN, args.lib),
            "held_out_ranges": evaluate(envmod.pure_pursuit_policy, args.eval_episodes, eval_seed,
                                        envmod.RANGES_HELD_OUT, args.lib),
        },
    }
    if report is not None:
        print(json.dumps(report, indent=1))
    if args.evaluate and report is not None:
        d = json.loads(args.evaluate.read_text(encoding="utf-8"))
        d["evaluation"] = report
        args.evaluate.write_text(json.dumps(d, indent=1) + "\n", encoding="utf-8")
    elif not args.evaluate:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        meta = {"trainer": "cross-entropy method, sim_py/train_cem.py",
                "settings": {k: getattr(args, k) for k in
                             ("iterations", "population", "elite", "episodes", "seed", "max_time",
                              "target_speed")},
                "environment": f"CampusEnv, walls on, RANGES_TRAIN, target speed {args.target_speed:g} m/s",
                "car": "SlipX provisional defaults (not a measured car)",
                "train_seconds": round(seconds, 1), "history": history, "evaluation": report}
        args.out.write_text(json.dumps(policy.to_json(meta), indent=1) + "\n", encoding="utf-8")
        print(f"wrote {args.out}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
