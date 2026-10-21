"""Tests for sim_py/train_cem.py (the dependency-free campus trainer).

    python -I sim_py/test_train_cem.py
RC_TWIN_LIB overrides the DLL path (default build/bin/rc_twin.dll).
"""

from __future__ import annotations

import importlib.util
import json
import os
import pathlib
import unittest

HERE = pathlib.Path(__file__).resolve().parent
_spec = importlib.util.spec_from_file_location("train_cem", HERE / "train_cem.py")
tc = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(tc)
LIB = os.environ.get("RC_TWIN_LIB")
quiet = lambda *a, **k: None


def env():
    return tc.campus.CampusEnv(seed=0, ranges=tc.envmod.RANGES_TRAIN, max_time=20.0, lib_path=LIB)


class Policy(unittest.TestCase):
    def test_round_trip_and_frame_check(self):
        theta = [0.01 * k for k in range(tc.LinearPolicy.size())]
        p = tc.LinearPolicy(theta)
        d = json.loads(json.dumps(p.to_json({"note": "x"})))
        q = tc.LinearPolicy.from_json(d)
        obs = [0.1 * (k % 7) for k in range(200)]
        self.assertEqual(p(obs), q(obs))
        d["frame_fields"] = d["frame_fields"][:-1] + ["something_else"]
        with self.assertRaises(ValueError):
            tc.LinearPolicy.from_json(d)
        with self.assertRaises(ValueError):
            tc.LinearPolicy(theta[:-1])

    def test_actions_are_bounded_and_linear_inside(self):
        n = tc.N_IN
        theta = [0.0] * (2 * (n + 1))
        theta[n] = 0.3            # steer bias
        theta[n + 1 + 0] = 100.0  # throttle on speed_ratio, saturating
        steer, thr = tc.LinearPolicy(theta)([1.0] + [0.0] * (n - 1))
        self.assertAlmostEqual(steer, __import__("math").tanh(0.3))
        self.assertTrue(-1.0 <= thr <= 1.0 and thr > 0.999)


class Training(unittest.TestCase):
    def test_a_short_run_beats_doing_nothing_and_repeats(self):
        e = env()
        seeds = [11, 12, 13]
        zero = tc.score(e, tc.LinearPolicy([0.0] * tc.LinearPolicy.size()), seeds)
        p1, h1 = tc.train(e, iterations=4, population=10, elite=3, episodes=2, seed=3, log=quiet)
        p2, h2 = tc.train(e, iterations=4, population=10, elite=3, episodes=2, seed=3, log=quiet)
        self.assertEqual(p1.theta, p2.theta, "seeded training is reproducible")
        self.assertEqual(h1, h2)
        self.assertGreater(tc.score(e, p1, seeds), zero, "it learned something")
        self.assertEqual(len(h1), 4)


if __name__ == "__main__":
    unittest.main()
