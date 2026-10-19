"""Tests for sim_py/rc_twin_env.py. Standard library only.

    python -I sim_py/test_rc_twin_env.py
RC_TWIN_LIB overrides the DLL path (default build/bin/rc_twin.dll).
"""

from __future__ import annotations

import importlib.util
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
        self.assertEqual(rewards[-1], envmod.REWARD["off_road"])

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


if __name__ == "__main__":
    unittest.main()
