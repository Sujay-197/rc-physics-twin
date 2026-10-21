"""Tests for sim_py/rc_twin_env.py. Standard library only.

    python -I sim_py/test_rc_twin_env.py
RC_TWIN_LIB overrides the DLL path (default build/bin/rc_twin.dll).
"""

from __future__ import annotations

import importlib.util
import math
import os
import pathlib
import unittest

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
_spec = importlib.util.spec_from_file_location("rc_twin_env", HERE / "rc_twin_env.py")
envmod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(envmod)

LIB = os.environ.get("RC_TWIN_LIB")
HILL = ROOT / "roads" / "samples" / "hill_breaker_bend.json"
FLAT = ROOT / "roads" / "samples" / "straight_flat.json"


def make(road=HILL, **kw):
    return envmod.RcTwinEnv(road, lib_path=LIB, **kw)


def rollout(env, policy, seed, steps=None):
    obs, info = env.reset(seed=seed)
    trace = [obs]
    rewards = []
    done, n = False, 0
    cap = steps if steps is not None else 20000  # 200 s: a broken termination fails, not hangs
    while not done and n < cap:
        obs, r, te, tr, info = env.step(policy(obs))
        trace.append(obs)
        rewards.append(r)
        done = te or tr
        n += 1
    return trace, rewards, info


class Env(unittest.TestCase):
    def test_observation_layout_is_fixed(self):
        env = make(history=3)
        self.assertEqual(envmod.FRAME_SIZE, len(envmod.FRAME_FIELDS))
        self.assertEqual(env.observation_size, envmod.FRAME_SIZE + 3 * (envmod.FRAME_SIZE + 2))
        trace, _, _ = rollout(env, envmod.pure_pursuit_policy, seed=1, steps=50)
        self.assertTrue(all(len(o) == env.observation_size for o in trace))

    def test_the_same_seed_is_the_same_episode(self):
        env = make()
        a = rollout(env, envmod.pure_pursuit_policy, seed=7, steps=300)
        b = rollout(env, envmod.pure_pursuit_policy, seed=7, steps=300)
        self.assertEqual(a, b)
        env.reset(seed=8)
        other = dict(env.sample)
        env.reset(seed=7)
        self.assertNotEqual(env.sample, other)

    def test_randomisation_stays_in_its_ranges_and_reaches_the_car(self):
        env = make()
        defaults = make(ranges=envmod.RANGES_NOMINAL)
        defaults.reset(seed=0)
        for seed in range(20):
            env.reset(seed=seed)
            for k, (lo, hi) in envmod.RANGES_TRAIN.items():
                self.assertTrue(lo <= env.sample[k] <= hi, k)
            p, d = env.params, defaults.params
            self.assertAlmostEqual(p.tyre_front.mu_y0, d.tyre_front.mu_y0 * env.sample["friction"])
            self.assertAlmostEqual(p.mass, d.mass + env.sample["payload_kg"])
            self.assertAlmostEqual(p.pack_internal_resistance,
                                   d.pack_internal_resistance * env.sample["battery_resistance"])
        # The edge of the training ranges lies inside them, at their hard end
        # (less grip, more load, slower servo, weaker motor, softer tyres,
        # more battery resistance), apart from the held-out ones.
        for k, (lo, hi) in envmod.RANGES_TRAIN_EDGE.items():
            tlo, thi = envmod.RANGES_TRAIN[k]
            hlo, hhi = envmod.RANGES_HELD_OUT[k]
            self.assertTrue(tlo <= lo < hi <= thi, k)
            self.assertTrue(hi <= hlo or lo >= hhi, k)
            self.assertTrue(lo == tlo if hhi <= tlo else hi == thi, f"{k}: the hard end")
        # Held-out ranges do not overlap training ones (payload may touch).
        for k, (lo, hi) in envmod.RANGES_HELD_OUT.items():
            tlo, thi = envmod.RANGES_TRAIN[k]
            self.assertTrue(hi <= tlo or lo >= thi, k)

    def test_the_baseline_drives_from_the_observation_alone(self):
        for ranges in (envmod.RANGES_NOMINAL, envmod.RANGES_TRAIN, envmod.RANGES_HELD_OUT):
            env = make(ranges=ranges)
            res = envmod.evaluate(env, envmod.pure_pursuit_policy, episodes=3, seed=0)
            self.assertEqual(res["success_rate"], 1.0)
            self.assertGreater(res["mean_return"], 0.0)
            self.assertLess(res["mean_abs_offset_m"], 0.05)

    def test_leaving_the_road_ends_the_episode_with_its_penalty(self):
        env = make(road=FLAT, ranges=envmod.RANGES_NOMINAL)
        trace, rewards, info = rollout(env, lambda o: (1.0, 0.5), seed=0)
        self.assertLess(len(trace), 20000, "the episode ended")
        self.assertEqual(info["status"], "left_road")
        r = envmod.REWARD
        left = (env.max_time - info["time"]) / envmod.TICK
        self.assertAlmostEqual(rewards[-1], r["off_road"] - (r["time"] + r["offset"]) * left, places=9)

    def test_time_runs_out_as_truncation(self):
        env = make(road=FLAT, ranges=envmod.RANGES_NOMINAL, max_time=0.5)
        obs, info = env.reset(seed=0)
        steps = 0
        while True:
            obs, r, te, tr, info = env.step(envmod.pure_pursuit_policy(obs))
            steps += 1
            if te or tr:
                break
        self.assertTrue(tr and not te)
        self.assertEqual(info["status"], "timeout")
        self.assertEqual(steps, 50)

    def test_history_holds_the_last_frames_and_actions_newest_first(self):
        env = make(history=2, ranges=envmod.RANGES_NOMINAL)
        obs0, _ = env.reset(seed=0)
        f = envmod.FRAME_SIZE
        obs1, *_ = env.step((0.3, 0.2))
        obs2, *_ = env.step((-0.1, 0.4))
        self.assertEqual(obs2[f:f + f], obs1[:f])            # previous frame
        self.assertEqual(obs2[2 * f:2 * f + 2], (-0.1, 0.4))  # with the action after it
        self.assertEqual(obs2[2 * f + 2:3 * f + 2], obs0[:f])
        self.assertEqual(obs2[3 * f + 2:3 * f + 4], (0.3, 0.2))

    def test_actions_are_clipped(self):
        a = make(ranges=envmod.RANGES_NOMINAL)
        b = make(ranges=envmod.RANGES_NOMINAL)
        a.reset(seed=0)
        b.reset(seed=0)
        self.assertEqual(a.step((5.0, -9.0)), b.step((1.0, -1.0)))

    def test_a_perturbation_changes_the_car_mid_episode(self):
        env = make(road=FLAT, ranges=envmod.RANGES_NOMINAL)
        obs, _ = env.reset(seed=0, options={"perturb": {"at_s": 8.0, "friction": 0.5}})
        mu0 = env.params.tyre_front.mu_y0
        while env.s < 8.0:
            obs, *_ = env.step(envmod.pure_pursuit_policy(obs))
        x_before = env.state.x
        obs, r, te, tr, info = env.step(envmod.pure_pursuit_policy(obs))
        self.assertAlmostEqual(env.params.tyre_front.mu_y0, 0.5 * mu0)
        self.assertGreater(env.state.x, x_before)  # kept going: same state, new car
        self.assertEqual(info["status"], "running")

    @unittest.skipUnless(importlib.util.find_spec("gymnasium") and importlib.util.find_spec("numpy"),
                         "gymnasium and numpy not installed")
    def test_gymnasium_wrapper(self):
        env = envmod.make_gymnasium_env(HILL, lib_path=LIB)
        obs, _ = env.reset(seed=0)
        self.assertEqual(obs.shape, env.observation_space.shape)


class StopTask(unittest.TestCase):
    """task="stop": from rest at the start to a stop at the end."""

    @staticmethod
    def planner(env):
        # The baseline's steering, with a speed plan that stops at the end:
        # the remaining distance comes from the frame alone.
        i_rem = env.frame_fields.index("remaining_ratio")

        def policy(obs):
            steer, _ = envmod.pure_pursuit_policy(obs)
            rem = obs[i_rem] * envmod.REMAINING_SCALE_M if obs[i_rem] < 1.0 else 1e9
            goal = min(1.0, (2.0 * 1.5 * max(0.0, rem - 0.05)) ** 0.5 / env.target_speed)
            v = obs[0]
            thr = max(-1.0, min(1.0, 3.0 * (goal - v)))
            if thr < 0.0 and v <= 0.01:
                thr = 0.0
            return steer, thr
        return policy

    def test_it_starts_at_rest_and_reports_the_distance_left(self):
        env = make(FLAT, task="stop", target_speed=2.0, history=2)
        obs, _ = env.reset(seed=0)
        self.assertEqual(env.frame_fields[-1], "remaining_ratio")
        self.assertEqual(len(obs), env.frame_size + 2 * (env.frame_size + 2))
        self.assertEqual(env.frame_size, envmod.FRAME_SIZE + 1)
        self.assertAlmostEqual(obs[0], 0.0, places=9, msg="at rest")
        self.assertEqual(obs[env.frame_size - 1], 1.0, "far from the end: the ratio saturates")
        env.close()

    def test_arriving_means_stopping_at_the_end(self):
        env = make(FLAT, task="stop", target_speed=2.0)
        trace, rewards, info = rollout(env, self.planner(env), seed=1)
        self.assertEqual(info["status"], "reached_end")
        self.assertLessEqual(env._length - env.s, envmod.STOP_TOLERANCE_M)
        self.assertLessEqual(abs(env.state.vx), envmod.STOP_SPEED)
        self.assertEqual(rewards[-1] > 5.0, True, "the finish bonus")
        env.close()

    def test_running_through_the_end_is_not_arriving(self):
        env = make(FLAT, task="stop", target_speed=2.0)
        trace, rewards, info = rollout(env, envmod.pure_pursuit_policy, seed=1)   # never brakes for the end
        self.assertEqual(info["status"], "overshot")
        env.close()

    def test_the_follow_task_is_unchanged(self):
        env = make(FLAT, target_speed=2.0)
        self.assertEqual(env.task, "follow")
        self.assertEqual(env.frame_size, envmod.FRAME_SIZE)
        obs, _ = env.reset(seed=0)
        self.assertAlmostEqual(obs[0], 1.0, places=6, msg="starts at the target speed")
        with self.assertRaises(ValueError):
            make(FLAT, task="race")
        env.close()


class Car(unittest.TestCase):
    """car=: the bare Core car, the Unity twin's car (TwinCar.cs), or both."""

    def test_the_bare_car_is_the_default_and_unchanged(self):
        env = make(ranges=envmod.RANGES_NOMINAL)
        self.assertEqual(env.car, "bare")
        _, info = env.reset(seed=0)
        self.assertEqual(info["car"], "bare")
        p = env.params
        self.assertEqual((p.wheel_inertia, p.ackermann, p.wheel_rate_front, p.damping_front), (0.0, 0.0, 0.0, 0.0))
        env.close()

    def test_the_unity_car_is_twincars_defaults(self):
        env = make(ranges=envmod.RANGES_NOMINAL, car="unity")
        _, info = env.reset(seed=0)
        self.assertEqual(info["car"], "unity")
        p = env.params
        m = make(ranges=envmod.RANGES_NOMINAL).nominal_params().mass
        # TwinCar.DemoWheelRate(m, 3 Hz) and DemoDamping(m, k, 0.3), per wheel.
        k = m * (2.0 * math.pi * 3.0) ** 2 / 4.0
        self.assertAlmostEqual(p.wheel_inertia, 1e-5)
        self.assertEqual(p.ackermann, 1.0)
        self.assertAlmostEqual(p.wheel_rate_front, k)
        self.assertAlmostEqual(p.wheel_rate_rear, k)
        self.assertAlmostEqual(p.damping_front, 2.0 * 0.3 * math.sqrt(4.0 * k * m) / 4.0)
        self.assertAlmostEqual(p.suspension_travel, 0.015)
        env.close()

    def test_the_springs_are_set_for_the_unladen_car(self):
        # As in Unity: a payload loads the springs, it does not stiffen them.
        env, nominal = make(car="unity"), make(car="unity", ranges=envmod.RANGES_NOMINAL)
        nominal.reset(seed=0)
        env.reset(seed=3)
        self.assertGreater(env.sample["payload_kg"], 0.1)
        self.assertAlmostEqual(env.params.wheel_rate_front, nominal.params.wheel_rate_front)
        env.close()
        nominal.close()

    def test_both_draws_either_car_by_the_seed(self):
        env = make(car="both")
        cars = [env.reset(seed=s)[1]["car"] for s in range(20)]
        self.assertEqual(set(cars), {"bare", "unity"})
        self.assertEqual(cars, [env.reset(seed=s)[1]["car"] for s in range(20)], "the seed fixes the car")
        for s in range(20):
            _, info = env.reset(seed=s)
            self.assertEqual(env.params.ackermann, 1.0 if info["car"] == "unity" else 0.0)
        # The car is drawn after the randomisation, which stays as it was.
        self.assertEqual(env.reset(seed=7)[1]["randomisation"], make().reset(seed=7)[1]["randomisation"])
        env.close()

    def test_an_unknown_car_is_refused(self):
        with self.assertRaises(ValueError):
            make(car="kart")

    def test_the_unity_car_drives_the_stop_task(self):
        env = make(FLAT, task="stop", target_speed=2.0, car="unity")
        trace, rewards, info = rollout(env, StopTask.planner(env), seed=1)
        self.assertEqual(info["status"], "reached_end")
        env.close()


class StopReward(unittest.TestCase):
    """The stop task pays for a tidy line and smooth steering."""

    def test_offset_and_steer_change_cost_linearly_on_the_stop_task(self):
        self.assertEqual(envmod.tidiness("stop", -0.1, 0.05), (0.1, 0.05))
        self.assertEqual(envmod.tidiness("stop", 0.0, -0.3), (0.0, 0.3))
        # "follow" keeps its squares, so its results stay reproducible.
        self.assertEqual(envmod.tidiness("follow", -0.1, 0.5), ((-0.1) ** 2, 0.5 ** 2))
        self.assertEqual(envmod.reward_weights("follow"), envmod.REWARD)
        stop = envmod.reward_weights("stop")
        self.assertGreater(stop["offset"], envmod.REWARD["offset"])
        self.assertGreater(stop["steering"], envmod.REWARD["steering"])

    def test_dithering_steering_costs_on_the_stop_task(self):
        def run(dither):
            env = make(FLAT, task="stop", target_speed=2.0, ranges=envmod.RANGES_NOMINAL)
            plan = StopTask.planner(env)
            obs, info = env.reset(seed=1)
            done, ret, k = False, 0.0, 0
            while not done:
                steer, thr = plan(obs)
                obs, r, te, tr, info = env.step((steer + (dither if k % 2 else -dither), thr))
                ret += r
                k += 1
                done = te or tr
            env.close()
            return ret, info["status"], k
        clean, dithered = run(0.0), run(0.03)
        self.assertEqual((clean[1], dithered[1]), ("reached_end", "reached_end"))
        # Each tick's change of 0.06 is charged at the steering weight.
        cost = envmod.reward_weights("stop")["steering"] * 0.06 * dithered[2]
        self.assertGreater(clean[0] - dithered[0], 0.8 * cost)

    def test_the_drivable_line_rounds_each_corner_inside_the_road(self):
        # A 90-degree corner with 3 m legs: the line leaves the first leg R
        # before the corner, rejoins the second R after it, and cuts the
        # corner by R (sqrt 2 - 1).
        R = envmod.LINE_RADIUS_M
        line = envmod.drivable_line([(0.0, 0.0), (3.0, 0.0), (3.0, 3.0)], R)
        self.assertEqual(line[0], (0.0, 0.0))
        self.assertEqual(line[-1], (3.0, 3.0))
        self.assertTrue(any(abs(x - (3.0 - R)) < 1e-9 and abs(y) < 1e-9 for x, y in line), "leaves at 3 - R")
        apex = min(math.hypot(x - 3.0, y) for x, y in line)
        self.assertAlmostEqual(apex, R * (math.sqrt(2.0) - 1.0), places=2)
        steps = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(line, line[1:])]
        self.assertLess(max(s for s in steps if s < 2.0), 0.06, "the arc is dense")
        # Short legs take a smaller radius: each corner may use half of each leg.
        tight = envmod.drivable_line([(0.0, 0.0), (0.4, 0.0), (0.4, 0.4)], R)
        self.assertGreaterEqual(min(x for x, y in tight[1:]), 0.2 - 1e-9)
        # A straight road is its own line.
        self.assertEqual(envmod.drivable_line([(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)], R),
                         [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)])

    def test_the_stop_task_charges_offset_from_the_drivable_line(self):
        env = make(HILL, task="stop", ranges=envmod.RANGES_NOMINAL)
        env.reset(seed=0)
        pts = [(p["x"], p["y"]) for p in env.road["centreline"]]
        line = envmod.drivable_line(pts)
        for x, y in line[:: max(1, len(line) // 20)]:
            self.assertAlmostEqual(env._line_offset(x, y), 0.0, places=6)
        x, y = line[len(line) // 2]
        self.assertAlmostEqual(env._line_offset(x + 0.1, y), 0.1, delta=0.1001)
        env.close()

    def test_kerb_contact_costs_every_tick_it_lasts(self):
        # Against a wall across the road, pushing: every tick in contact is charged.
        wall = [{"points": [[3.0, -2.0], [3.0, 2.0]], "closed": False}]
        env = make(FLAT, task="stop", target_speed=2.0, ranges=envmod.RANGES_NOMINAL, walls=wall)
        env.reset(seed=0)
        touching = []
        for _ in range(400):
            obs, r, te, tr, info = env.step((0.0, 1.0))
            touching.append(env.last_contact)
            if te or tr:
                break
        self.assertGreater(sum(touching[-100:]), 90, "pinned against the wall")
        self.assertGreater(envmod.reward_weights("stop")["contact"], 0.0)
        self.assertNotIn("contact", envmod.REWARD, "follow is unchanged")
        env.close()

    def test_leaving_the_road_charges_the_stop_tasks_weights(self):
        env = make(road=FLAT, ranges=envmod.RANGES_NOMINAL, task="stop")
        trace, rewards, info = rollout(env, lambda o: (1.0, 0.5), seed=0)
        self.assertEqual(info["status"], "left_road")
        r = envmod.reward_weights("stop")
        left = (env.max_time - info["time"]) / envmod.TICK
        self.assertAlmostEqual(rewards[-1], r["off_road"] - (r["time"] + r["offset"]) * left, places=9)
        env.close()

    def test_driving_off_the_road_never_beats_staying_stopped(self):
        out = {}
        for name, policy in (("stopped", lambda o: (0.0, 0.0)), ("off", lambda o: (1.0, 1.0))):
            env = make(HILL, task="stop", target_speed=1.5, ranges=envmod.RANGES_NOMINAL, max_time=20.0)
            trace, rewards, info = rollout(env, policy, seed=0)
            out[name] = (sum(rewards), info["status"])
            env.close()
        self.assertEqual(out["off"][1], "left_road")
        self.assertEqual(out["stopped"][1], "timeout")
        self.assertGreater(out["stopped"][0], out["off"][0], out)


class History(unittest.TestCase):
    def test_the_observation_keeps_exactly_history_frames(self):
        for h in (0, 1, 4):
            env = envmod.RcTwinEnv(HILL, history=h, ranges=envmod.RANGES_NOMINAL, lib_path=LIB)
            obs, _ = env.reset(seed=0)
            for _ in range(6):
                self.assertEqual(len(obs), env.observation_size, h)
                obs, *_ = env.step((0.0, 0.3))
            self.assertEqual(len(obs), envmod.FRAME_SIZE + h * (envmod.FRAME_SIZE + envmod.ACTION_SIZE))
            env.close()


class RewardCap(unittest.TestCase):
    def test_speeding_earns_no_more_progress_than_the_target(self):
        # A 1 m/s target and full throttle: the car runs well past the
        # target, but no tick pays more than the target speed's progress.
        env = envmod.RcTwinEnv(HILL, target_speed=1.0, ranges=envmod.RANGES_NOMINAL, lib_path=LIB)
        env.reset(seed=0, options={"start_speed": 1.0})
        best = -1e9
        for _ in range(100):
            obs, r, te, tr, info = env.step((0.0, 1.0))
            best = max(best, r)
            if te or tr:
                break
        self.assertGreater(env.state.vx, 1.5, "it is speeding")
        self.assertLessEqual(best, 1.0 + 1e-12, "progress is capped at the target speed")

    def test_the_return_is_highest_at_the_target_speed(self):
        # Pure pursuit steering on the sample road, holding half the target,
        # the target, or full throttle: holding the target scores best.
        def holding(ratio):
            def policy(obs):
                steer, _ = envmod.pure_pursuit_policy(obs)
                v = obs[envmod.FRAME_FIELDS.index("speed_ratio")]
                return steer, (1.0 if ratio is None else max(-1.0, min(1.0, 3.0 * (ratio - v))))
            return policy
        returns = {}
        for name, ratio in (("half", 0.5), ("target", 1.0), ("full", None)):
            env = envmod.RcTwinEnv(HILL, target_speed=1.5, ranges=envmod.RANGES_NOMINAL, lib_path=LIB,
                                   max_time=120.0)
            obs, info = env.reset(seed=0, options={"start_speed": 1.5})
            done, ret = False, 0.0
            while not done:
                obs, r, te, tr, info = env.step(holding(ratio)(obs))
                ret += r
                done = te or tr
            returns[name] = ret
            env.close()
        self.assertGreater(returns["target"], returns["half"], returns)
        self.assertGreater(returns["target"], returns["full"], returns)

    def test_driving_off_the_road_never_beats_staying_stopped(self):
        # Stopped on the road until the timeout against steering off it at
        # once: with a time cost, the early exit must still cost more.
        # Brake only while rolling forward: held at a standstill, the brake
        # drives the car backwards.
        speed = envmod.FRAME_FIELDS.index("speed_ratio")
        stop = lambda o: (0.0, -1.0 if o[speed] > 0.01 else 0.0)
        out = {}
        for name, policy in (("stopped", stop), ("off", lambda o: (1.0, 1.0))):
            env = envmod.RcTwinEnv(HILL, target_speed=1.5, ranges=envmod.RANGES_NOMINAL, lib_path=LIB,
                                   max_time=20.0)
            obs, info = env.reset(seed=0, options={"start_speed": 1.5})
            done, ret = False, 0.0
            while not done:
                obs, rew, te, tr, info = env.step(policy(obs))
                ret += rew
                done = te or tr
            out[name] = (ret, info["status"])
            env.close()
        self.assertEqual(out["off"][1], "left_road")
        self.assertEqual(out["stopped"][1], "timeout")
        self.assertGreater(out["stopped"][0], out["off"][0], out)


if __name__ == "__main__":
    unittest.main()
