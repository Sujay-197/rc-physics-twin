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
- **Reward** per tick: progress along the road normalised to about 1 at speed, minus offset, steering-change and energy terms (`REWARD`); leaving the road ends the episode with a penalty, reaching the end with a bonus; `max_time` truncates.
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
