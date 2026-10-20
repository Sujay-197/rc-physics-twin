"""Tests for sim_py/train_ppo.py. Skipped unless numpy, gymnasium,
stable-baselines3 and torch are installed (the repo's .venv):

    .venv/Scripts/python -I sim_py/test_train_ppo.py
RC_TWIN_LIB overrides the DLL path (default build/bin/rc_twin.dll).
"""

from __future__ import annotations

import importlib.util
import os
import pathlib
import tempfile
import unittest

HERE = pathlib.Path(__file__).resolve().parent
HAVE = all(importlib.util.find_spec(m) for m in ("numpy", "gymnasium", "stable_baselines3", "torch"))
LIB = os.environ.get("RC_TWIN_LIB")


@unittest.skipUnless(HAVE, "numpy, gymnasium, stable-baselines3 and torch not installed")
class PPO(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location("train_ppo", HERE / "train_ppo.py")
        cls.tp = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.tp)

    def test_the_campus_env_passes_the_sb3_checker(self):
        from stable_baselines3.common.env_checker import check_env
        env = self.tp.envmod.make_gymnasium_env(env_cls=self.tp.campus.CampusEnv, seed=0, lib_path=LIB)
        check_env(env, warn=False)
        obs, info = env.reset(seed=0)
        self.assertEqual(obs.shape, (env.inner.observation_size,))
        self.assertIn("trip", info)
        env.close()

    def test_batched_workers_match_one_process(self):
        # 3 envs over 2 workers (2 + 1) step exactly as the same 3 envs in
        # this process: same observations, rewards, dones, auto-resets.
        import numpy as np
        batched = self.tp.make_vec_env(3, seed=5, max_time=3.0, lib=LIB, processes=True, per_worker=2)
        local = self.tp.make_vec_env(3, seed=5, max_time=3.0, lib=LIB, processes=False)
        try:
            self.assertEqual(batched.sizes, [2, 1])
            self.assertEqual(batched.num_envs, 3)
            self.assertEqual(batched.get_attr("spec"), local.get_attr("spec"))
            with self.assertRaisesRegex(RuntimeError, "environment worker failed"):
                batched.get_attr("no_such_attribute")   # reported, and the worker lives on
            ob, ol = batched.reset(), local.reset()
            np.testing.assert_array_equal(ob, ol)
            rng = np.random.default_rng(0)
            saw_done = False
            for _ in range(400):   # 3 s episodes at 100 Hz: every env finishes at least once
                act = rng.uniform(-0.3, 1.0, size=(3, 2))
                b, l = batched.step(act), local.step(act)
                for x, y in zip(b[:3], l[:3]):
                    np.testing.assert_array_equal(x, y)
                for ib, il in zip(b[3], l[3]):
                    self.assertEqual("terminal_observation" in ib, "terminal_observation" in il)
                saw_done |= bool(b[2].any())
            self.assertTrue(saw_done)
            self.assertEqual(batched.env_is_wrapped(__import__("stable_baselines3").common.monitor.Monitor),
                             [True, True, True])
            self.assertEqual(len(batched.get_attr("spec", indices=[2])), 1)
        finally:
            batched.close()
            local.close()

    def test_train_save_load_act(self):
        # A tiny run through the script itself: the saved model and its
        # normalisation load back into a policy that acts the same.
        import numpy as np
        with tempfile.TemporaryDirectory() as tmp:
            out = pathlib.Path(tmp) / "m"
            argv = ["--timesteps", "2048", "--envs", "2", "--eval-episodes", "1", "--max-time", "5",
                    "--n-steps", "512", "--batch", "256", "--net", "32", "32", "--device", "cpu",
                    "--out", str(out)] + (["--lib", LIB] if LIB else [])
            self.assertEqual(self.tp.main(argv), 0)
            for ext in (".zip", ".vecnormalize.pkl", ".json"):
                self.assertTrue(out.with_suffix(ext).exists(), ext)
            p = self.tp.PPOPolicy.load(out)
            q = self.tp.PPOPolicy.load(out)
            obs = np.linspace(-1.0, 1.0, p.obs_rms.mean.shape[0])
            a = p(obs)
            self.assertEqual(a, q(obs))
            self.assertTrue(all(-1.0 <= x <= 1.0 for x in a))


if __name__ == "__main__":
    unittest.main()
