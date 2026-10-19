"""Tests for tools/drive_road.py, the headless harness. Standard library only.

Run from the repository root (ctest runs it as tools.drive_road):
    python -I tools/test_drive_road.py
RC_TWIN_LIB overrides the DLL path (default build/bin/rc_twin.dll).
"""

from __future__ import annotations

import argparse
import copy
import ctypes
import importlib.util
import json
import os
import pathlib
import unittest

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
_spec = importlib.util.spec_from_file_location("drive_road", HERE / "drive_road.py")
dr = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(dr)

LIB = dr.load_lib(pathlib.Path(os.environ.get("RC_TWIN_LIB", ROOT / "build" / "bin" / "rc_twin.dll")))
SAMPLES = ROOT / "roads" / "samples"


def args(**kw) -> argparse.Namespace:
    base = dict(speed=2.0, start_speed=None, timeout=120.0, wheel_inertia=0.0, ackermann=0.0,
                wheel_rate=0.0, damping=0.0, travel=0.015)
    base.update(kw)
    return argparse.Namespace(**base)


def sample(name: str) -> dict:
    return json.loads((SAMPLES / name).read_text(encoding="utf-8"))


def straight(z_end: float = 0.0, width: float = 1.0, breakers=None) -> dict:
    road = {"schema_version": "0.1.0", "id": "test", "width": width,
            "centreline": [{"x": 0.0, "y": 0.0, "z": 0.0}, {"x": 20.0, "y": 0.0, "z": z_end}]}
    if breakers:
        road["speed_breakers"] = breakers
    return road


class Harness(unittest.TestCase):
    def test_the_samples_are_valid_and_driven_to_the_end(self):
        for name in ("straight_flat.json", "hill_breaker_bend.json"):
            road = sample(name)
            dr.check_road(road)
            code, rep = dr.drive(LIB, road, args(wheel_inertia=1e-5, ackermann=1.0,
                                                 wheel_rate=311.0, damping=9.9))
            self.assertEqual((code, rep["status"]), (0, "reached_end"), name)
            self.assertLess(rep["peak_offset_m"], road["width"] / 2, name)

    def test_a_straight_road_runs_at_speed_on_the_line(self):
        code, rep = dr.drive(LIB, sample("straight_flat.json"), args())
        self.assertEqual(code, 0)
        self.assertAlmostEqual(rep["mean_speed_mps"], 2.0, delta=0.1)
        self.assertLess(rep["peak_offset_m"], 1e-6)

    def test_level_energy_is_resistance_times_distance_over_efficiency(self):
        # Drag plus rolling resistance at the run's speed, pushed over the
        # distance, through the drivetrain efficiency. Provisional SlipX values.
        code, rep = dr.drive(LIB, sample("straight_flat.json"), args())
        p = dr.Params()
        LIB.rc_twin_params_default(p)
        v = rep["mean_speed_mps"]
        force = p.drag_coeff * v * v + p.roll_resist * p.mass * 9.80665
        expected_wh = force * rep["distance_m"] / p.drive_efficiency / 3600.0
        self.assertAlmostEqual(rep["energy_wh"], expected_wh, delta=0.1 * expected_wh)

    def test_climbing_costs_energy_and_descending_returns_some(self):
        e = {}
        for name, z in (("up", 1.0), ("flat", 0.0), ("down", -1.0)):  # 5 % grades
            code, rep = dr.drive(LIB, straight(z), args())
            self.assertEqual(code, 0, name)
            e[name] = rep["energy_wh"]
        self.assertGreater(e["up"], 2 * e["flat"])
        self.assertLess(e["down"], e["flat"])

    def test_a_speed_breaker_pitches_a_sprung_body(self):
        sprung = args(wheel_rate=311.0, damping=9.9)
        _, smooth = dr.drive(LIB, straight(), sprung)
        _, bumped = dr.drive(LIB, straight(breakers=[
            {"s": 8.0, "height": 0.015, "length": 0.3, "profile": "round_hump"}]), sprung)
        self.assertGreater(bumped["peak_pitch_rad"], smooth["peak_pitch_rad"] + 0.003)

    def test_too_narrow_a_road_is_left_at_once(self):
        code, rep = dr.drive(LIB, straight(width=0.2), args())  # narrower than the track
        self.assertEqual((code, rep["status"]), (2, "left_road"))

    def test_the_same_run_twice_is_the_same_report(self):
        road = sample("hill_breaker_bend.json")
        a = dr.drive(LIB, road, args(wheel_inertia=1e-5, wheel_rate=311.0, damping=9.9))
        b = dr.drive(LIB, copy.deepcopy(road), args(wheel_inertia=1e-5, wheel_rate=311.0, damping=9.9))
        self.assertEqual(a, b)

    def test_bad_roads_are_refused_with_a_reason(self):
        bad = straight()
        del bad["width"]
        with self.assertRaisesRegex(dr.RoadError, "width"):
            dr.check_road(bad)
        bad = straight()
        bad["speed_breakers"] = [{"s": 1.0, "height": 0.01, "length": 0.2, "profile": "table"}]
        with self.assertRaisesRegex(dr.RoadError, "profile"):
            dr.check_road(bad)
        bad = straight()
        bad["centreline"][1]["bank"] = 0.9
        with self.assertRaisesRegex(dr.RoadError, "bank"):
            dr.check_road(bad)
        self.assertEqual(dr.main([str(SAMPLES / "missing.json")]), 1)


class Walls(unittest.TestCase):
    def test_add_walls_open_and_closed(self):
        lib = LIB
        p = dr.Params()
        lib.rc_twin_params_default(ctypes.byref(p))
        car = lib.rc_twin_create(2, 0, ctypes.byref(p), None)
        try:
            tri = [[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]]
            dr.add_walls(lib, car, [{"points": tri, "closed": False}])
            self.assertEqual(lib.rc_twin_wall_count(car), 2)
            dr.add_walls(lib, car, [{"points": tri, "closed": True}])
            self.assertEqual(lib.rc_twin_wall_count(car), 5)
            with self.assertRaises(ValueError):
                dr.add_walls(lib, car, [{"name": "one point", "points": [[0.0, 0.0]]}])
            self.assertEqual(dr.wall_report(lib, car).contacts, 0)
        finally:
            lib.rc_twin_destroy(car)


if __name__ == "__main__":
    unittest.main()
