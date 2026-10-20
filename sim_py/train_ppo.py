"""Train a PPO policy with observation history on the campus.

Needs numpy, gymnasium, stable-baselines3 and torch (the .venv in the repo
root; see sim_py/README.md). The policy is an MLP over the full observation:
the current frame plus the last `history` frames with the actions taken after
them (RMA-style, rc_twin_env.py), so it can infer the episode's car from how
it has been responding. Observations and rewards are normalised with
VecNormalize; its statistics are saved with the model.

Throughput: the environments step on the CPU, one process each (--envs, by
default all but two logical CPUs, SubprocVecEnv); the network trains on
--device (cuda when a CUDA build of torch sees a GPU, else cpu). Stepping the
simulation dominates, so the processes are where the speed comes from; the
GPU earns its keep with the larger network and batches set by --net and
--batch.

    .venv/Scripts/python sim_py/train_ppo.py --timesteps 2000000
    .venv/Scripts/python sim_py/train_ppo.py --evaluate policies/campus_ppo

Training: CampusEnv, walls on, RANGES_TRAIN, trips drawn by demand. The
evaluation is train_cem.py's: 30 trips the training never drew, under
training and held-out randomisation, next to the pure-pursuit baseline.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import pathlib
import sys
import time

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
# batched_vec_env is imported by name so worker processes can unpickle its
# worker function; this line runs again in each spawned worker.
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


campus = _load("campus", HERE / "campus.py")
envmod = campus.envmod
cem = _load("train_cem", HERE / "train_cem.py")


def _make_campus_env(seed: int, max_time: float, lib, ranges):
    """One monitored campus env. Top level and self-loading, so a spawned
    worker process can build it from a pickle of this function's arguments."""
    from stable_baselines3.common.monitor import Monitor
    c = _load("campus", HERE / "campus.py")
    env = c.envmod.make_gymnasium_env(env_cls=c.CampusEnv, seed=seed, max_time=max_time,
                                      lib_path=lib, ranges=ranges or c.envmod.RANGES_TRAIN)
    env.reset(seed=seed)
    return Monitor(env)


class _Factory:
    def __init__(self, *args):
        self.args = args

    def __call__(self):
        return _make_campus_env(*self.args)


def make_vec_env(n_envs: int, seed: int, max_time: float, lib, ranges=None, processes: bool = True,
                 per_worker: int = 1):
    """n_envs campus envs, each with its own trip and car stream: per_worker
    of them in each worker process (BatchedSubprocVecEnv), or all in this
    process."""
    from stable_baselines3.common.vec_env import DummyVecEnv
    fns = [_Factory(seed + 1000 * k, max_time, lib, ranges) for k in range(n_envs)]
    if processes and n_envs > 1:
        from batched_vec_env import BatchedSubprocVecEnv
        return BatchedSubprocVecEnv(fns, per_worker=per_worker, start_method="spawn")
    return DummyVecEnv(fns)


class PPOPolicy:
    """A trained model as a plain policy(obs) -> (steer, throttle), with the
    saved observation normalisation, deterministic."""

    def __init__(self, model, obs_rms, clip_obs: float, epsilon: float):
        self.model, self.obs_rms, self.clip, self.eps = model, obs_rms, clip_obs, epsilon

    def __call__(self, obs):
        o = np.asarray(obs, dtype=np.float64)
        o = np.clip((o - self.obs_rms.mean) / np.sqrt(self.obs_rms.var + self.eps), -self.clip, self.clip)
        a, _ = self.model.predict(o.astype(np.float32), deterministic=True)
        a = np.clip(a, -1.0, 1.0)
        return float(a[0]), float(a[1])

    @classmethod
    def load(cls, prefix: pathlib.Path):
        import pickle
        from stable_baselines3 import PPO
        model = PPO.load(str(prefix.with_suffix(".zip")), device="cpu")   # acting is cheap
        with open(prefix.with_suffix(".vecnormalize.pkl"), "rb") as f:
            vn = pickle.load(f)
        return cls(model, vn.obs_rms, vn.clip_obs, vn.epsilon)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--timesteps", type=int, default=2_000_000)
    ap.add_argument("--workers", type=int, default=max(1, (__import__("os").cpu_count() or 4) - 2),
                    help="environment processes (default: all but two logical CPUs)")
    ap.add_argument("--per-worker", type=int, default=4, help="environments stepped by each process")
    ap.add_argument("--envs", type=int, help="total environments (default workers x per-worker)")
    ap.add_argument("--device", default="auto", help="cuda, cpu, or auto (cuda when available)")
    ap.add_argument("--net", type=int, nargs="+", default=[256, 256], help="hidden layer sizes")
    ap.add_argument("--n-steps", type=int, default=1024, help="steps per env per rollout")
    ap.add_argument("--batch", type=int, default=4096, help="minibatch size")
    ap.add_argument("--in-process", action="store_true", help="step all envs in this process")
    ap.add_argument("--gamma", type=float, default=0.995,
                    help="discount per 10 ms tick: 0.995 looks about 2 s ahead, a corner's worth")
    ap.add_argument("--log-std", type=float, default=-1.5,
                    help="initial log std of the actions: 100 Hz control wants little noise")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--max-time", type=float, default=45.0, help="[s] per training episode")
    ap.add_argument("--eval-episodes", type=int, default=30)
    ap.add_argument("--out", type=pathlib.Path, default=ROOT / "policies" / "campus_ppo")
    ap.add_argument("--evaluate", type=pathlib.Path, help="evaluate a saved model prefix instead of training")
    ap.add_argument("--lib")
    args = ap.parse_args(argv)
    if args.envs is None:
        args.envs = args.workers * args.per_worker

    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import VecNormalize
    device = args.device if args.device != "auto" else ("cuda" if torch.cuda.is_available() else "cpu")
    torch.set_num_threads(2)   # the CPUs are the environments' (one process each)
    print(f"device {device}" + (f" ({torch.cuda.get_device_name(0)})" if device == "cuda" else "")
          + f", {args.envs} environments in {min(args.workers, args.envs)} processes", flush=True)

    if args.evaluate:
        policy = PPOPolicy.load(args.evaluate)
        seconds, settings = None, None
    else:
        venv = VecNormalize(make_vec_env(args.envs, args.seed, args.max_time, args.lib,
                                         processes=not args.in_process,
                                         per_worker=max(1, -(-args.envs // args.workers))),
                            norm_obs=True, norm_reward=True, clip_obs=10.0, gamma=args.gamma)
        settings = dict(n_steps=args.n_steps, batch_size=args.batch, n_epochs=10, gamma=args.gamma,
                        gae_lambda=0.95, learning_rate=3e-4, clip_range=0.2, ent_coef=0.0,
                        policy_kwargs=dict(net_arch=list(args.net), log_std_init=args.log_std))
        model = PPO("MlpPolicy", venv, seed=args.seed, device=device, verbose=0, **settings)
        t0 = time.perf_counter()
        chunk = max(1, args.timesteps // 20)
        done = 0
        while done < args.timesteps:
            model.learn(total_timesteps=chunk, reset_num_timesteps=False)
            done += chunk
            eps = list(model.ep_info_buffer)
            mean_r = sum(e["r"] for e in eps) / max(1, len(eps))
            print(f"{done:9d} steps  {time.perf_counter() - t0:7.1f} s  "
                  f"mean episode return {mean_r:9.1f} ({len(eps)} episodes)", flush=True)
        seconds = time.perf_counter() - t0
        args.out.parent.mkdir(parents=True, exist_ok=True)
        model.save(str(args.out.with_suffix(".zip")))
        venv.save(str(args.out.with_suffix(".vecnormalize.pkl")))
        policy = PPOPolicy(model, venv.obs_rms, venv.clip_obs, venv.epsilon)
        venv.close()

    eval_seed = 10_000 + args.seed
    report = {
        "trained_policy": {
            "train_ranges": cem.evaluate(policy, args.eval_episodes, eval_seed, envmod.RANGES_TRAIN, args.lib),
            "held_out_ranges": cem.evaluate(policy, args.eval_episodes, eval_seed, envmod.RANGES_HELD_OUT, args.lib),
        },
        "pure_pursuit_baseline": {
            "train_ranges": cem.evaluate(envmod.pure_pursuit_policy, args.eval_episodes, eval_seed,
                                         envmod.RANGES_TRAIN, args.lib),
            "held_out_ranges": cem.evaluate(envmod.pure_pursuit_policy, args.eval_episodes, eval_seed,
                                            envmod.RANGES_HELD_OUT, args.lib),
        },
    }
    print(json.dumps(report, indent=1))
    if args.evaluate:
        meta_path = args.evaluate.with_suffix(".json")
        d = json.loads(meta_path.read_text(encoding="utf-8"))
        d["evaluation"] = report
        meta_path.write_text(json.dumps(d, indent=1) + "\n", encoding="utf-8")
    else:
        import gymnasium, stable_baselines3
        meta = {"trainer": "PPO (stable-baselines3), sim_py/train_ppo.py",
                "observation": f"current frame + {4} history frames with actions, VecNormalize",
                "settings": {**{k: v for k, v in settings.items() if k != "policy_kwargs"},
                             "net_arch": list(args.net), "log_std_init": args.log_std,
                    "timesteps": args.timesteps, "envs": args.envs,
                    "device": device,
                             "seed": args.seed, "max_time": args.max_time},
                "versions": {"stable_baselines3": stable_baselines3.__version__, "torch": torch.__version__,
                             "gymnasium": gymnasium.__version__, "numpy": np.__version__},
                "environment": "CampusEnv, walls on, RANGES_TRAIN, target speed 2 m/s",
                "car": "SlipX provisional defaults (not a measured car)",
                "train_seconds": round(seconds, 1), "evaluation": report}
        args.out.with_suffix(".json").write_text(json.dumps(meta, indent=1) + "\n", encoding="utf-8")
        print(f"wrote {args.out}.zip, .vecnormalize.pkl, .json", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
