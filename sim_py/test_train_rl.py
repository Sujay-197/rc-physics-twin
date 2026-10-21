"""Tests for sim_py/train_rl.py and sim_py/batched_vec_env.py. Skipped unless
numpy, gymnasium, stable-baselines3, sb3-contrib and torch are installed (the
repo's .venv):

    .venv/Scripts/python -I sim_py/test_train_rl.py
RC_TWIN_LIB overrides the DLL path (default build/bin/rc_twin.dll).
"""

from __future__ import annotations

import importlib.util
import os
import pathlib
import sys
import tempfile
import unittest

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
HAVE = all(importlib.util.find_spec(m) for m in ("numpy", "gymnasium", "stable_baselines3", "sb3_contrib", "torch"))
LIB = os.environ.get("RC_TWIN_LIB") or str(ROOT / "build" / "bin" / "rc_twin.dll")
TINY = ["--timesteps", "1024", "--chunks", "2", "--val-trips", "2", "--workers", "1", "--per-worker", "2", "--n-steps", "256", "--batch", "256",
        "--net", "32", "32", "--learning-starts", "256", "--buffer", "10000", "--device", "cpu",
        "--max-time", "5", "--lib", LIB]


@unittest.skipUnless(HAVE, "the RL packages are not installed")
class TrainRL(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import train_rl
        cls.rl = train_rl
        cls.cfg = {"target_speed": 4.0, "history": 4, "max_time": 5.0, "residual": None, "residual_scale": 0.5,
                   "lib": LIB}

    def test_the_campus_env_passes_the_sb3_checker(self):
        from stable_baselines3.common.env_checker import check_env
        for residual in (None, "pure_pursuit"):
            env = self.rl.make_env({**self.cfg, "residual": residual}, seed=0)
            check_env(env.env, warn=False)
            env.close()

    def test_the_residual_starts_from_the_controller(self):
        # A zero correction drives exactly as the tuned controller does, and
        # the observation carries the controller's action at its end.
        import numpy as np
        env = self.rl.make_env({**self.cfg, "residual": "pure_pursuit"}, seed=3)
        inner = env.env.env.inner
        base = self.rl.build_base(inner, "pure_pursuit")
        obs, _ = env.reset(seed=3)
        for _ in range(50):
            raw = obs[:-2]
            self.assertEqual(tuple(obs[-2:]), base(raw))
            obs, r, te, tr, info = env.step(np.zeros(2))
            if te or tr:
                break
        self.assertEqual(self.rl.residual_action((0.9, -0.2), (1.0, -1.0), 0.5), (1.0, -0.7))
        env.close()

    def test_batched_workers_match_one_process(self):
        import numpy as np
        batched = self.rl.make_vec({**self.cfg, "max_time": 3.0}, 3, workers=2, seed=5)
        local = self.rl.make_vec({**self.cfg, "max_time": 3.0}, 3, workers=1, seed=5)
        try:
            self.assertEqual(batched.sizes, [2, 1])
            with self.assertRaisesRegex(RuntimeError, "environment worker failed"):
                batched.get_attr("no_such_attribute")
            np.testing.assert_array_equal(batched.reset(), local.reset())
            rng = np.random.default_rng(0)
            saw_done = False
            for _ in range(400):
                act = rng.uniform(-0.3, 1.0, size=(3, 2))
                b, l = batched.step(act), local.step(act)
                for x, y in zip(b[:3], l[:3]):
                    np.testing.assert_array_equal(x, y)
                saw_done |= bool(b[2].any())
            self.assertTrue(saw_done)
        finally:
            batched.close()
            local.close()

    def test_every_algorithm_trains_saves_and_loads(self):
        campus = self.rl._campus()
        with tempfile.TemporaryDirectory() as tmp:
            for algo, extra in (("ppo", []), ("ppo", ["--residual"]), ("sac", []), ("td3", ["--residual"]),
                                ("rppo", [])):
                out = pathlib.Path(tmp) / f"{algo}{len(extra)}"
                self.assertEqual(self.rl.main(["--algo", algo, *extra, "--out", str(out), *TINY]), 0)
                history = 0 if algo == "rppo" else 4
                env = campus.CampusEnv(seed=0, target_speed=4.0, history=history, lib_path=LIB)
                p = self.rl.load_policy(out, env)
                obs, _ = env.reset(seed=1)
                a1 = [p(obs) for _ in range(3)]
                p.reset()
                a2 = [p(obs) for _ in range(3)]
                self.assertEqual(a1, a2, f"{algo}: deterministic, and reset clears any memory")
                self.assertTrue(all(-1.0 <= x <= 1.0 for a in a1 for x in a))
                with self.assertRaisesRegex(ValueError, "trained at"):
                    self.rl.load_policy(out, campus.CampusEnv(seed=0, target_speed=2.0, history=history,
                                                              lib_path=LIB))
                env.close()

    def test_the_car_is_recorded_and_checked_on_load(self):
        import json
        campus = self.rl._campus()
        with tempfile.TemporaryDirectory() as tmp:
            out = pathlib.Path(tmp) / "res-unity"
            self.assertEqual(self.rl.main(["--algo", "ppo", "--residual", "--task", "stop", "--car", "unity",
                                           "--out", str(out), *TINY]), 0)
            meta = json.loads(out.with_suffix(".json").read_text(encoding="utf-8"))
            self.assertEqual(meta["config"]["car"], "unity")
            self.assertEqual(meta["reward"], campus.envmod.reward_weights("stop"), "the weights it trained under")

            def env(car):
                return campus.CampusEnv(seed=0, target_speed=4.0, history=4, lib_path=LIB, task="stop", car=car)
            self.rl.load_policy(out, env("unity"))
            with self.assertRaisesRegex(ValueError, "trained on the 'unity' car"):
                self.rl.load_policy(out, env("bare"))
            self.rl.load_policy(out, env("bare"), any_car=True)     # to measure the gap
            # A "both" policy drives either car; a model from before the option trained on the bare car.
            for trained, car, fits in (("both", "bare", True), ("both", "unity", True), ("unity", "both", False),
                                       ("bare", "unity", False)):
                self.assertEqual(self.rl.car_fits(trained, car), fits, (trained, car))
            del meta["config"]["car"]
            out.with_suffix(".json").write_text(json.dumps(meta), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "trained on the 'bare' car"):
                self.rl.load_policy(out, env("unity"))


    def test_a_residual_builds_on_the_tuning_it_names(self):
        import json
        campus = self.rl._campus()
        unity = "policies/controllers_unity_stop.json"
        sys.path.insert(0, str(ROOT / "tools"))
        import export_policy
        with tempfile.TemporaryDirectory() as tmp:
            out = pathlib.Path(tmp) / "res-gains"
            self.assertEqual(self.rl.main(["--algo", "ppo", "--residual", "--residual-gains", unity, "--task", "stop",
                                           "--car", "unity", "--out", str(out), *TINY]), 0)
            meta = json.loads(out.with_suffix(".json").read_text(encoding="utf-8"))
            self.assertEqual(meta["config"]["residual_gains"], unity)
            env = campus.CampusEnv(seed=0, target_speed=4.0, history=4, lib_path=LIB, task="stop", car="unity")
            gains = self.rl.base_gains("pure_pursuit", unity)["gains"]
            self.assertNotEqual(gains, self.rl.base_gains("pure_pursuit")["gains"])
            p = self.rl.load_policy(out, env)
            ctl = self.rl._controllers()
            want = ctl.build("pure_pursuit", campus.envmod, env.nominal_params(), 0.5 * env.network.width, 4.0,
                             task="stop", **gains)
            obs, _ = env.reset(seed=2)
            self.assertEqual(p.base(obs), want(obs))
            self.assertEqual(export_policy.export(out, LIB, check_ticks=3)["residual"]["gains"], gains)
            env.close()


if __name__ == "__main__":
    unittest.main()
