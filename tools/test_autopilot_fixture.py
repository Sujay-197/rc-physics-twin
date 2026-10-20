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

    def test_it_covers_a_real_drive(self):
        doc = json.loads(fx.FIXTURE.read_text(encoding="utf-8"))
        self.assertEqual(len(doc["ticks"]), fx.TICKS)
        steers = [t["action"][0] for t in doc["ticks"]]
        self.assertGreater(max(abs(s) for s in steers), 0.05, "the drive turns")
        self.assertEqual(len(doc["ticks"][0]["obs"]), 23 + 4 * 25)


if __name__ == "__main__":
    unittest.main()
