# sim_py

Python on top of `rc_twin.dll`, standard library only.

## `rc_twin_env.py`: the RL environment (spec section 8)

A Gymnasium-style environment: follow a road (`roads/road.schema.json`) at a target speed.

```python
import importlib.util
spec = importlib.util.spec_from_file_location("rc_twin_env", "sim_py/rc_twin_env.py")
env_mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(env_mod)

env = env_mod.RcTwinEnv("roads/samples/hill_breaker_bend.json", target_speed=2.0)
obs, info = env.reset(seed=0)
obs, reward, terminated, truncated, info = env.step((0.1, 0.5))   # steer, throttle in [-1, 1]
```

- **Action:** `[steer, throttle]` in [-1, 1], the same mapping as the keyboard car: steer to `steer_max` (positive left), throttle to `accel_max`, brake as negative throttle to `decel_max`. 100 Hz over 1 ms Core steps.
- **Observation:** the current frame (`FRAME_FIELDS`, 23 values) followed by the previous `history` frames, each with the action taken after it, newest first. The frame has speed, lateral velocity, yaw rate, the tyres' built-up slip angles, wheel speeds, state of charge, achieved steer, grade along the heading, offset and heading error from the centreline, the centreline 0.5, 1 and 2 m ahead in the car's frame, and the target speed.
- **Domain randomisation** per episode from a seeded generator: friction, payload, servo bandwidth, motor torque, tyre stiffness, battery resistance. `RANGES_TRAIN` for training, `RANGES_HELD_OUT` (disjoint) for evaluation, `RANGES_NOMINAL` for the provisional car unchanged. The ranges are training choices, not measurements.
- **Reward** per tick: progress along the road, normalised to 1 at the target speed and capped there, minus offset, steering-change, energy and wall terms and a constant time cost (`REWARD`). Reaching the end adds a bonus; `max_time` truncates. Leaving the road ends the episode with a penalty that also charges every remaining tick at the time cost plus the full offset cost, so ending an episode is never cheaper than staying on the road.
- **Why it is shaped this way.** Each piece closes a loophole a trained policy actually found:
  - Without the cap, finishing sooner always paid, and PPO drove at 3.7 m/s against a 2 m/s target.
  - Without the time cost, every speed up to the target scored alike, and CEM crept along at 1.25 m/s.
  - Without charging the remaining time, driving off the road was cheaper than being stuck, and CEM learnt to do it on purpose.
  
  Together they make the return highest at the target speed, on the road. Tests hold all three.
- **Perturbation tests:** `reset(options={"perturb": {"at_s": 10, "friction": 0.5}})` changes the car mid-episode, keeping its state.
- **`evaluate(env, policy, episodes, seed)`**: mean return, success rate, mean offset, energy. **`pure_pursuit_policy`** is a baseline that drives from the observation alone; it finishes the sample roads under nominal, training and held-out randomisation.
- **`make_gymnasium_env(...)`** wraps it as a `gymnasium.Env` with `Box` spaces, if `gymnasium` and `numpy` are installed. Training with SB3 (PPO/SAC) needs those plus `stable-baselines3`; none are installed or required here.

Tests: `python -I sim_py/test_rc_twin_env.py` (also in ctest as `sim_py.env`).

## `campus.py`: the campus fleet sim's roads

`CampusEnv` is `RcTwinEnv` on the campus fleet sim's road network at 1/10 (`campus/README.md`): each reset draws a stop-to-stop trip by the stops' demand weights and drives its shortest route, so the policy trains on the roads the fleet runs.

```python
spec = importlib.util.spec_from_file_location("campus", "sim_py/campus.py")
campus = importlib.util.module_from_spec(spec); spec.loader.exec_module(campus)
env = campus.CampusEnv(seed=0, ranges=campus.envmod.RANGES_TRAIN)
obs, info = env.reset(seed=0)                        # info["trip"], e.g. ("S3", "AB4")
obs, info = env.reset(options={"trip": ("AB1", "S6")})
```

The campus's kerbs and buildings are walls (ADR 0009): a strike bounces the car and costs `REWARD["wall"]` times the speed it lost; `info["wall_contacts"]` and `info["wall_impulse"]` count them. Any `RcTwinEnv` takes walls the same way (`walls=[{"points": [[x, y], ...], "closed": bool}]`).

`RcTwinEnv.reset(options={"road": road_dict})` swaps the road for any env, which is how `CampusEnv` changes route per episode. `max_time` defaults to 120 s here (the longest trip is 69 m). Tests: `python -I sim_py/test_campus.py` (ctest `sim_py.campus`).

## `train_cem.py`: training on the campus without extra packages

A dependency-free learner, so a policy can be trained on the campus where only `rc_twin.dll` and Python are available: the cross-entropy method over a linear policy, `action = tanh(W . frame + b)` on the current observation frame. Each iteration scores a population of weight vectors on the same few campus trips and randomised cars, and refits a Gaussian to the best.

```bash
python sim_py/train_cem.py --iterations 40 --out policies/campus_cem.json
```

```bash
python sim_py/train_cem.py --evaluate policies/campus_cem.json
```

The report compares the policy with the pure-pursuit baseline under training and held-out randomisation, on trips the training never drew. It is a baseline learner, not a replacement for PPO or SAC: a linear policy cannot use the observation history, so it cannot adapt to the car the way a history-conditioned network trained with `make_gymnasium_env` and SB3 can. Tests: `python -I sim_py/test_train_cem.py` (ctest `sim_py.train_cem`).

## `train_ppo.py`: PPO with observation history

PPO (stable-baselines3) over the full observation, the current frame plus four history frames with the actions taken after them, so the policy can infer the episode's car from how it has been responding (RMA-style). Observations and rewards are normalised (VecNormalize, saved next to the model).

Needs the packages in `requirements-rl.txt`, kept out of the rest of the project in the repo's `.venv`:

```bash
python -m venv .venv
```

```bash
.venv/Scripts/python -m pip install -r sim_py/requirements-rl.txt
```

```bash
.venv/Scripts/python sim_py/train_ppo.py --timesteps 30000000 --per-worker 8 --n-steps 512 --batch 8192
```

```bash
.venv/Scripts/python sim_py/train_ppo.py --evaluate policies/campus_ppo
```

**Using the machine.** The simulation steps on the CPU: `--workers` processes (default all but two logical CPUs), each stepping `--per-worker` environments per message (`batched_vec_env.py`). On Windows a pipe round trip costs more than a simulation step, so batching is what scales: on a 16-thread CPU, one environment per process ran about 6,000 steps/s, four per process about 18,000, eight about 25,000 to 30,000. The network trains on the GPU when torch has CUDA (`--device auto`; for an NVIDIA card install the CUDA build noted in `requirements-rl.txt`). With this small MLP the GPU is lightly loaded and the CPU sets the pace; stable-baselines3 says as much.

Tests: `.venv/Scripts/python -I sim_py/test_train_ppo.py` (ctest `sim_py.train_ppo`, which uses `.venv` when it exists and skips otherwise): the campus env passes SB3's checker, batched workers step exactly as one process does, and a tiny run saves, loads and acts.
