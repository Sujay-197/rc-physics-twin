"""Tests for tools/record_drives.py, the Unity replay's recorder. Standard
library only (the tuned controllers need nothing else).

    python -I tools/test_record_drives.py
RC_TWIN_LIB overrides the DLL path (default build/bin/rc_twin.dll).
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys
import unittest

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
import record_drives as rd  # noqa: E402

LIB = os.environ.get("RC_TWIN_LIB") or str(ROOT / "build" / "bin" / "rc_twin.dll")


class RecordDrives(unittest.TestCase):
    def test_pairs_cover_every_route_once(self):
        net = rd.ev.campus().CampusNetwork.load()
        pairs = rd.stop_pairs()
        self.assertEqual(len(pairs), len(set(pairs)))
        expect = sum(1 for a in net.stops for b in net.stops if net.stops[a]["node"] != net.stops[b]["node"])
        self.assertEqual(len(pairs), expect)
        self.assertIn(("S1", "AB4"), pairs)
        self.assertIn(("AB4", "S1"), pairs)
        trips = rd.trip_list(True, 0)
        self.assertEqual([s for s, _ in trips], list(range(rd.PAIR_SEED, rd.PAIR_SEED + len(pairs))))
        self.assertEqual(rd.trip_list(False, 3), [(rd.ev.EVAL_SEED + k, None) for k in range(3)])

    def test_still_tail_keeps_its_first_and_last_sample(self):
        moving = [[0.1 * k, 0.0, 0.0, 4, 0, 0, 0] for k in range(5)]
        still = [[0.4 + 0.001 * (k % 2), 0.0, 0.002, 0, 0, 1, 0] for k in range(50)]
        # moving[4] is already within 5 mm and 0.01 rad of the final pose:
        # it opens the still run, the final sample closes it.
        self.assertEqual(rd.trim_still(moving + still), moving + [still[-1]])
        self.assertEqual(rd.trim_still(moving), moving, "a moving car keeps every sample")
        self.assertEqual(rd.trim_still(moving[:1]), moving[:1])

    def test_timed_out_drives_stop_where_the_replay_does(self):
        drives = [{"status": "reached_end", "time": 6.0}, {"status": "left_road", "time": 8.5},
                  {"status": "timeout", "time": 120.0}]
        self.assertEqual(rd.shown_until(drives), 8.5 + rd.AFTER_S)
        self.assertEqual(rd.shown_until(drives[2:]), rd.ALL_TIMED_OUT_S)
        frames = [[k] for k in range(3001)]                  # 120 s at 25 Hz
        cut = rd.cut_after(frames, 11.5, 25)
        self.assertEqual(cut[-2], [int(11.5 * 25)], "every sample up to 11.5 s, still at i / 25")
        self.assertEqual(cut[-1], [3000], "and where the drive ended")
        self.assertEqual(rd.cut_after(frames[:100], 11.5, 25), frames[:100])

    def test_every_policy_meets_the_same_trip_and_car(self):
        trips = [(rd.PAIR_SEED, ("S1", "AB4")), (rd.PAIR_SEED + 1, ("S3", "S2"))]
        pol = [("controller:pure_pursuit", "pure pursuit"), ("controller:stanley", "Stanley")]
        data = rd.record(pol, trips, ["train", "perturbed"], LIB, workers=2, target_speed=4.0)
        data = json.loads(json.dumps(data))                     # what the file holds
        self.assertEqual(data["format"], rd.FORMAT)
        self.assertEqual(data["trip_set"], "pairs")
        self.assertEqual(data["sample_hz"], 25)
        for cond in ("train", "perturbed"):
            table = data["trips"][cond]
            self.assertEqual([(t["from"], t["to"]) for t in table], [("S1", "AB4"), ("S3", "S2")])
            for k, t in enumerate(table):
                self.assertGreater(len(t["route"]), 1)
                self.assertGreater(t["until"], max(data["runs"][s][cond][k]["time"] for s, _ in pol
                                                   if data["runs"][s][cond][k]["status"] != "timeout"))
                for spec, _ in pol:
                    d = data["runs"][spec][cond][k]
                    self.assertIn(d["status"], ("reached_end", "left_road", "timeout"))
                    if spec == "controller:pure_pursuit" and cond == "train":
                        self.assertEqual(d["status"], "reached_end", (cond, k))
                    x0, y0 = d["frames"][0][:2]
                    self.assertLess(abs(x0 - t["route"][0][0]) + abs(y0 - t["route"][0][1]), 0.5)
                    gaps = [(b[0] - a[0], math.hypot(b[1] - a[1], b[2] - a[2])) for a, b in zip(d["hits"], d["hits"][1:])]
                    self.assertTrue(all(dt >= rd.HIT_GAP_S - 0.011 or dm >= rd.HIT_GAP_M - 0.002 for dt, dm in gaps))
                    self.assertLessEqual(len(d["hits"]), d["wall_contacts"])
                    if d["status"] == "reached_end":
                        self.assertGreaterEqual(len(d["frames"]), int(d["time"] * 25))
                    if cond == "perturbed":
                        self.assertIsNotNone(d["perturbed"], "both trips run past 4 m")
                    else:
                        self.assertIsNone(d["perturbed"])
        # Each pair has a car of its own.
        self.assertNotEqual(data["trips"]["train"][0]["car"], data["trips"]["train"][1]["car"])


if __name__ == "__main__":
    unittest.main()
