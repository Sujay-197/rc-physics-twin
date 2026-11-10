"""Tests for tools/ablation.py. Standard library only.

    python -I tools/test_ablation.py
RC_TWIN_LIB overrides the DLL path (default build/bin/rc_twin.dll).
"""

from __future__ import annotations

import importlib.util
import json
import os
import pathlib
import unittest

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
_spec = importlib.util.spec_from_file_location("ablation", HERE / "ablation.py")
ab = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(ab)

LIB = os.environ.get("RC_TWIN_LIB") or str(ROOT / "build" / "bin" / "rc_twin.dll")


class Ablation(unittest.TestCase):
    def test_the_full_variant_is_the_shipped_autopilot_and_each_other_differs(self):
        v = ab.variants()
        self.assertEqual(v["full"][1], "recover:corner:3.4,1.2,1.2,0.5:controller:pure_pursuit:"
                                       "policies/controllers_unity_stop.json")
        self.assertEqual(len({spec for _, spec in v.values()}), len(v))
        # The explicit-gains variants change only the one gain they name.
        unity = json.loads((ROOT / ab.UNITY).read_text(encoding="utf-8"))["pure_pursuit"]["gains"]
        spec = v["linear_stop"][1]
        g = json.loads(spec[spec.index("{"):])
        self.assertEqual(g, {**unity, "stop_plan": "linear"})
        spec = v["no_cornering_limit"][1]
        g = json.loads(spec[spec.index("{"):])
        self.assertEqual(g, {**unity, "a_lat": None})

    def test_a_small_grid_runs_and_pairs_every_variant_with_the_full_autopilot(self):
        rep = ab.run([4.0], trips=2, seed=50_000, workers=1, lib=LIB)
        res = rep["results"]["4"]
        self.assertEqual(set(res), set(ab.variants()))
        for name, conds in res.items():
            for cond in ("train", "held_out", "perturbed"):
                d = conds[cond]
                self.assertEqual([e[0] for e in d["episodes"]], [50_000, 50_001], name)
                self.assertEqual(sum(d["outcomes"].values()), 2)
        full = res["full"]["train"]["vs_full"]
        self.assertEqual((full["wins"], full["losses"], full["p"], full["time_ratio_median"]), (0, 0, 1.0, 1.0))
        self.assertIn("full autopilot", ab.speed_table(rep, "4"))
        self.assertIn("| full autopilot | 100 / 100 / 100 | - |", ab.summary_table(rep))


if __name__ == "__main__":
    unittest.main()
