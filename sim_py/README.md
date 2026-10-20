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

## Controllers, learned policies and the policy study

The full comparison is written up in `Docs/policy-study.md`.

- **`controllers.py`: classical controllers**, using the observation and the nominal car only:
  - pure pursuit;
  - Stanley;
  - a speed-scheduled LQR on the dynamic-bicycle error model.

  All three share a speed loop with an optional cornering limit. Stanley and LQR take their heading error from the chord just ahead: the fleet sim's routes turn 90 degrees at junction nodes, tighter than the car can turn inside the road, so they need that anticipation.
- **`tools/tune_controllers.py`** grid-searches each controller's gains on training-randomised cars, with seeds of its own, and freezes them in `policies/controllers.json`.
- **`train_rl.py`** trains PPO, SAC, TD3 or RecurrentPPO, each from scratch or as a `--residual` correction on a tuned controller. Environments step in worker processes, several per process (`batched_vec_env.py`). On Windows a pipe round trip costs more than a simulation step, so batching is what scales: on 16 threads, one environment per process ran about 6k steps/s, eight per process 16k to 30k. Networks train on the GPU when torch has CUDA.
- **`evaluate_policies.py`** drives every policy over the same trips and cars in three conditions: training cars, held-out cars, and a mid-trip change. It reports outcomes with Wilson intervals, plus driving quality (speed error, offset mean and 95th percentile, steering roughness, energy per metre). It uses all cores.
- **`tools/run_study.py`** runs the training queue, three seeds per method, resumably.

The RL packages are in `requirements-rl.txt`, installed into the repo's `.venv`:

```bash
python -m venv .venv
```

```bash
.venv/Scripts/python -m pip install -r sim_py/requirements-rl.txt
```

```bash
.venv/Scripts/python sim_py/train_rl.py --algo ppo --residual --seed 0 --timesteps 20000000
```

```bash
.venv/Scripts/python sim_py/evaluate_policies.py controller:pure_pursuit sb3:policies/study/ppo-res-s0 --target-speed 4
```

### Watching them drive

`tools/record_drives.py` drives each policy over a set of trips and records every drive at 25 Hz: position, heading, speed, steer, throttle and offset, plus where the car hit a wall and where the mid-trip change struck. With `--pairs` the trips are every stop-to-stop route, 90 of them, each with a car of its own; without it, they are the evaluation's own demand-weighted draws. Every policy drives every trip with the same car. By default it records the tuned controllers and every model in `policies/study/`; other models can be named as specs, with labels.

```bash
.venv/Scripts/python tools/record_drives.py --pairs --out results/drives.json
```

```bash
.venv/Scripts/python tools/record_drives.py controller:pure_pursuit "sb3:policies/study/ppo-s0=PPO s0" --out results/drives.json
```

Unity plays the recording on the campus: **RcTwin > Build Replay Scene**, then Play, or the Windows player from **RcTwin > Build Replay Player**. The player is `build/replay/RcTwinReplay.exe`, and it reads `drives.json` from its own folder, or from `-drives <path>`.

Pick a route with the **From** and **To** stop buttons; **Reverse** drives it the other way. Each policy is a coloured car, and every one drives the same trip with the same car. The panel shows each car's outcome, time and mean offset. A grid shows every trip's outcome per policy:
- green: finished;
- amber: finished after touching a kerb;
- red: left the road;
- purple: timed out, usually pinned against a kerb.

Controls:

| Key | Action |
|---|---|
| Left / Right | change trip |
| V | reverse the route |
| Up / Down | change condition |
| C | cycle the camera: pack, chase, route, campus |
| 1 to 9 | follow a policy |
| Space | play or pause |
| `[` and `]` | playback speed |
| F5 | reload the file |

The player also takes `-from <stop> -to <stop>`. Add `-screenshot <png>`, with `-condition`, `-trip`, `-camera`, `-follow` and `-at` as needed, and it saves one frame and quits.

The cars replay the Core states the harness scored; nothing is re-simulated in Unity.

Tests:
- `python -I sim_py/test_controllers.py` (ctest `sim_py.controllers`): geometry, LQR stability at every scheduled speed, tuned controllers driving at 4 m/s, the cornering limit, the statistics, and a deterministic harness.
- `.venv/Scripts/python -I sim_py/test_train_rl.py` (ctest `sim_py.train_rl`; it uses `.venv` when it exists and skips otherwise): the SB3 env checker, a residual that starts from its controller, batched workers matching one process, and every algorithm training, saving and loading.
