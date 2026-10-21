"""The Unity autopilot's parity fixture is what Python computes today
(tools/make_autopilot_fixture.py). Standard library only.

    python -I tools/test_autopilot_fixture.py
RC_TWIN_LIB overrides the DLL path (default build/bin/rc_twin.dll).
"""

from __future__ import annotations

import json
import pathlib
import sys
import unittest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import make_autopilot_fixture as fx  # noqa: E402


class AutopilotFixture(unittest.TestCase):
    def test_the_committed_fixture_is_current(self):
        committed = json.loads(fx.FIXTURE.read_text(encoding="utf-8"))
        fresh = fx.make()
        self.assertEqual(committed.keys(), fresh.keys())
        for key in committed:
            self.assertEqual(committed[key], fresh[key],
                             f"{key} changed: rerun tools/make_autopilot_fixture.py and the Unity Edit Mode tests")

    def test_the_synthetic_policies_are_current(self):
        doc = json.loads(fx.FIXTURE.read_text(encoding="utf-8"))
        for name, fresh in fx.make_policies(doc).items():
            committed = json.loads((fx.FIXTURES / name).read_text(encoding="utf-8"))
            self.assertEqual(committed, fresh, f"{name}: rerun tools/make_autopilot_fixture.py")
            self.assertEqual(len(committed["checks"]), 40)

    def test_the_kerb_recovery_trace_is_current(self):
        committed = json.loads(fx.RECOVERY.read_text(encoding="utf-8"))
        self.assertEqual(committed, fx.make_recovery(), "rerun tools/make_autopilot_fixture.py")

    def test_the_kerb_recovery_trace_backs_off_and_hands_back(self):
        doc = json.loads(fx.RECOVERY.read_text(encoding="utf-8"))
        ticks, tick = doc["ticks"], doc["tick"]
        backing = [t["action"][1] < 0.0 for t in ticks]
        first = backing.index(True)
        self.assertAlmostEqual(first, 30 + 0.5 / tick, delta=1, msg="backs off once pinned for 0.5 s")
        self.assertEqual(ticks[first]["action"], [-1.0, -0.6], "reversing on the other lock")
        back = backing.index(False, first)
        self.assertAlmostEqual((back - first) * 0.4 * tick, 0.3, delta=0.01, msg="for 0.3 m")
        self.assertFalse(any(backing[back:340]), "drives on; light throttle at rest is not a pin")
        last = backing.index(True, back)
        self.assertEqual(ticks[last]["action"], [1.0, -0.6], "the other lock from a right turn")
        self.assertAlmostEqual(len(ticks) - last, 1.5 / tick, delta=1, msg="no reverse: gives up after 1.5 s")
        self.assertEqual(ticks[-1]["back_offs"], 2)

    def test_the_corner_speed_trace_is_current(self):
        committed = json.loads(fx.CORNER.read_text(encoding="utf-8"))
        self.assertEqual(committed, fx.make_corner(), "rerun tools/make_autopilot_fixture.py")

    def test_the_corner_speed_trace_slows_for_corners_and_leaves_straights_alone(self):
        doc = fx.make_corner()
        prof = doc["profile"]
        self.assertLess(min(prof), 2.5, "a junction on the route")
        self.assertGreater(sum(c >= doc["target_speed"] for c in prof), len(prof) // 8, "straights at full speed")
        rise = max(b * b - a * a for a, b in zip(prof, prof[1:]))
        self.assertLessEqual(rise, 2.0 * doc["a_accel"] * doc["step"] + 1e-9, "pulling away no faster than a_accel")
        braked = [t for t in doc["ticks"] if t["action"][1] < t["inner"][1]]
        kept = [t for t in doc["ticks"] if t["action"] == t["inner"]]
        self.assertGreater(len(braked), 20)
        self.assertGreater(len(kept), 100)
        self.assertTrue(all(t["action"][1] <= t["inner"][1] for t in doc["ticks"]), "never raises the throttle")

    def test_it_covers_a_real_drive(self):
        doc = json.loads(fx.FIXTURE.read_text(encoding="utf-8"))
        follow, stop = doc["episodes"]
        self.assertEqual((follow["task"], stop["task"]), ("follow", "stop"))
        self.assertEqual(len(follow["ticks"]), fx.TICKS)
        steers = [t["action"][0] for t in follow["ticks"]]
        self.assertGreater(max(abs(s) for s in steers), 0.05, "the drive turns")
        self.assertEqual(len(follow["ticks"][0]["obs"]), 23 + 4 * 25)
        self.assertEqual(len(stop["ticks"][0]["obs"]), 24 + 4 * 26)
        self.assertEqual(stop["status"], "reached_end", "the stop episode stops at the end")
        self.assertLess(min(t["action"][1] for t in stop["ticks"]), -0.05, "and brakes for it")


if __name__ == "__main__":
    unittest.main()
