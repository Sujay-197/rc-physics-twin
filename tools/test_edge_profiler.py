"""Tests for tools/edge_profiler.py. Standard library only.

    python -I tools/test_edge_profiler.py
RC_TWIN_LIB overrides the DLL path (default build/bin/rc_twin.dll).
"""

from __future__ import annotations

import argparse
import copy
import importlib.util
import json
import os
import pathlib
import re
import unittest

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
_spec = importlib.util.spec_from_file_location("edge_profiler", HERE / "edge_profiler.py")
ep = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(ep)
dr = ep.dr

LIB = dr.load_lib(pathlib.Path(os.environ.get("RC_TWIN_LIB", ROOT / "build" / "bin" / "rc_twin.dll")))
SCHEMA = json.loads((ROOT / "profiles" / "edge_profile.schema.json").read_text(encoding="utf-8"))


def args(**kw) -> argparse.Namespace:
    base = dict(speeds=[1.0, 2.0, 3.0], payloads=[0.0, 1.0], soc=[0.8, 0.3],
                vehicle_class="test_class", timeout=120.0, wheel_inertia=0.0, ackermann=0.0,
                wheel_rate=0.0, damping=0.0, travel=0.015, breaker_speed=1.0, comfort_decel=2.0)
    base.update(kw)
    return argparse.Namespace(**base)


def road(rid: str, z_end: float = 0.0, breaker: float | None = None, width: float = 1.0) -> dict:
    r = {"schema_version": "0.1.0", "id": rid, "width": width,
         "centreline": [{"x": 0.0, "y": 0.0, "z": 0.0}, {"x": 12.0, "y": 0.0, "z": z_end}]}
    if breaker:
        r["speed_breakers"] = [{"s": 6.0, "height": breaker, "length": 0.3, "profile": "round_hump"}]
    return r


def entry(cache: dict, rid: str, speed: float, payload: float = 0.0, soc: float = 0.8) -> dict:
    (e,) = [e for e in cache["entries"] if e["edge_id"] == rid and e["speed_target_mps"] == speed
            and e["payload_kg"] == payload and e["soc_bucket"] == soc]
    return e


def check_against_schema(cache: dict) -> None:
    """The schema's required keys and digest patterns, without jsonschema."""
    top = SCHEMA["properties"]
    assert set(SCHEMA["required"]) <= set(cache), "top-level keys"
    item = top["entries"]["items"]
    hexd = re.compile(item["properties"]["road_digest"]["pattern"])
    for e in cache["entries"]:
        assert set(e) == set(item["required"]), f"entry keys {sorted(set(e) ^ set(item['required']))}"
        for d in ("road_digest", "params_digest", "controller_digest"):
            assert hexd.match(e[d]), d
        assert e["status"] in item["properties"]["status"]["enum"]


class Profiler(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.roads = [road("flat"), road("climb", z_end=0.6), road("bumpy", breaker=0.015)]
        cls.cache, cls.counts = ep.profile(LIB, cls.roads, args(), None)

    def test_one_entry_per_key_sorted_and_schema_shaped(self):
        self.assertEqual(len(self.cache["entries"]), 3 * 3 * 2 * 2)
        keys = [ep.key(e) for e in self.cache["entries"]]
        self.assertEqual(keys, sorted(set(keys)))
        self.assertEqual(self.counts["computed"], 36)
        check_against_schema(self.cache)
        self.assertEqual(self.cache["vehicle_classes"]["test_class"]["provenance"], "provisional")

    def test_costs_follow_the_physics(self):
        c = self.cache
        # Faster is quicker and, against drag, costs more energy.
        self.assertLess(entry(c, "flat", 3.0)["time_s"], entry(c, "flat", 1.0)["time_s"])
        self.assertGreater(entry(c, "flat", 3.0)["energy_wh"], entry(c, "flat", 1.0)["energy_wh"])
        # A climb and a payload cost energy.
        self.assertGreater(entry(c, "climb", 2.0)["energy_wh"], 2 * entry(c, "flat", 2.0)["energy_wh"])
        self.assertGreater(entry(c, "climb", 2.0, payload=1.0)["energy_wh"],
                           entry(c, "climb", 2.0)["energy_wh"])

    def test_the_breaker_penalty_is_time_lost_to_slowing(self):
        c = self.cache
        self.assertEqual(entry(c, "flat", 2.0)["breaker_penalty_s"], 0.0)
        # At or below the breaker speed the driver never slows.
        self.assertAlmostEqual(entry(c, "bumpy", 1.0)["breaker_penalty_s"], 0.0, places=6)
        p2 = entry(c, "bumpy", 2.0)["breaker_penalty_s"]
        p3 = entry(c, "bumpy", 3.0)["breaker_penalty_s"]
        self.assertGreater(p2, 0.2)
        self.assertGreater(p3, p2)  # more speed to shed and regain

    def test_an_unchanged_rerun_reuses_everything(self):
        again, counts = ep.profile(LIB, self.roads, args(), copy.deepcopy(self.cache))
        self.assertEqual(counts, {"reused": 36, "computed": 0, "dropped": 0})
        self.assertEqual(again, self.cache)

    def test_a_changed_road_recomputes_only_that_edge(self):
        roads = copy.deepcopy(self.roads)
        roads[2]["speed_breakers"][0]["height"] = 0.02
        _, counts = ep.profile(LIB, roads, args(), copy.deepcopy(self.cache))
        self.assertEqual((counts["computed"], counts["reused"]), (12, 24))

    def test_a_road_no_longer_profiled_is_dropped(self):
        cache, counts = ep.profile(LIB, self.roads[:1], args(), copy.deepcopy(self.cache))
        self.assertEqual((counts["reused"], counts["dropped"]), (12, 24))
        self.assertEqual({e["edge_id"] for e in cache["entries"]}, {"flat"})

    def test_a_changed_car_or_driver_recomputes_everything(self):
        for change in ({"wheel_inertia": 1e-5}, {"breaker_speed": 0.8}):
            _, counts = ep.profile(LIB, self.roads, args(**change), copy.deepcopy(self.cache))
            self.assertEqual((counts["computed"], counts["reused"]), (36, 0), change)

    def test_a_road_the_car_cannot_drive_is_marked_not_costed(self):
        cache, _ = ep.profile(LIB, [road("narrow", width=0.2)], args(speeds=[2.0], payloads=[0.0], soc=[0.8]), None)
        (e,) = cache["entries"]
        self.assertEqual(e["status"], "left_road")
        self.assertIsNone(e["time_s"])
        self.assertIsNone(e["energy_wh"])

    def test_the_command_line_writes_and_then_reuses_a_cache(self):
        import tempfile
        with tempfile.TemporaryDirectory() as tmp:
            out = pathlib.Path(tmp) / "cache.json"
            samples = sorted(str(p) for p in (ROOT / "roads" / "samples").glob("*.json"))
            argv = samples + ["--out", str(out), "--speeds", "2", "--soc", "0.7", "--breaker-speed", "1"]
            self.assertEqual(ep.main(argv), 0)
            first = out.read_text(encoding="utf-8")
            self.assertEqual(ep.main(argv), 0)
            self.assertEqual(out.read_text(encoding="utf-8"), first)
            check_against_schema(json.loads(first))


if __name__ == "__main__":
    unittest.main()
