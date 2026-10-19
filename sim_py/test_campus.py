"""Tests for the campus twin: tools/campus_import.py and sim_py/campus.py.

    python -I sim_py/test_campus.py
RC_TWIN_LIB overrides the DLL path (default build/bin/rc_twin.dll).
"""

from __future__ import annotations

import collections
import importlib.util
import itertools
import json
import math
import os
import pathlib
import random
import unittest

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


campus = _load("campus", HERE / "campus.py")
importer = _load("campus_import", ROOT / "tools" / "campus_import.py")
dr = campus.envmod.dr
LIB = os.environ.get("RC_TWIN_LIB")
NET = campus.CampusNetwork.load()
EXPORT = json.loads((ROOT / "campus" / "campus_export.json").read_text(encoding="utf-8"))


def length_of(road: dict) -> float:
    pts = [(p["x"], p["y"]) for p in road["centreline"]]
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


class Import(unittest.TestCase):
    def test_the_committed_data_is_what_the_importer_makes(self):
        network, roads = importer.convert(EXPORT)
        committed = json.loads((ROOT / "campus" / "campus_network.json").read_text(encoding="utf-8"))
        self.assertEqual(json.loads(json.dumps(network)), committed)
        for eid, road in roads.items():
            path = ROOT / "roads" / "campus" / f"edge_{eid:02d}.json"
            self.assertEqual(json.loads(json.dumps(road)), json.loads(path.read_text(encoding="utf-8")))

    def test_scale_and_frame(self):
        # The fleet sim's tile pitch is 27.85 Unity metres: 2.785 m here, and
        # Unity +z is ISO +x, Unity +x is ISO -y.
        self.assertEqual(importer.iso([1.0, 2.0, 3.0]), (0.30000000000000004, -0.1, 0.2))
        self.assertEqual(len(NET.nodes), len(EXPORT["nodes"]))
        self.assertEqual(len(NET.edges), len(EXPORT["edges"]))
        for e in EXPORT["edges"]:
            self.assertAlmostEqual(NET.edges[e["id"]]["length"], 0.1 * e["length"], delta=0.05 * 0.1 * e["length"])
        # The measured asphalt: 14.09 Unity metres across, so 1.409 m.
        self.assertAlmostEqual(NET.width, 1.409, places=3)
        self.assertAlmostEqual(NET.data["road"]["kerb_height"], 0.06, places=3)
        # Each sidewalk: (18.79 - 14.09) / 2 Unity metres, beside the asphalt.
        self.assertAlmostEqual(NET.data["road"]["sidewalk_width"], 0.235, places=3)

    def test_every_edge_file_is_a_valid_drivable_road(self):
        lib = dr.load_lib(pathlib.Path(LIB) if LIB else ROOT / "build" / "bin" / "rc_twin.dll")
        args = __import__("argparse").Namespace(speed=2.0, start_speed=None, timeout=60.0,
                                                 wheel_inertia=0.0, ackermann=0.0, wheel_rate=0.0,
                                                 damping=0.0, travel=0.015)
        for path in sorted((ROOT / "roads" / "campus").glob("edge_*.json")):
            road = json.loads(path.read_text(encoding="utf-8"))
            dr.check_road(road)
            code, rep = dr.drive(lib, road, args)
            self.assertEqual(rep["status"], "reached_end", path.name)


class Routing(unittest.TestCase):
    def test_routes_are_shortest_paths(self):
        # Against Floyd-Warshall over the whole graph.
        ids = sorted(NET.nodes)
        d = {(a, b): (0.0 if a == b else math.inf) for a in ids for b in ids}
        for e in NET.edges.values():
            d[e["a"], e["b"]] = min(d[e["a"], e["b"]], e["length"])
            d[e["b"], e["a"]] = d[e["a"], e["b"]]
        for k, i, j in itertools.product(ids, ids, ids):
            if d[i, k] + d[k, j] < d[i, j]:
                d[i, j] = d[i, k] + d[k, j]
        for a, b in itertools.permutations(sorted(NET.stops), 2):
            na, nb = NET.stops[a]["node"], NET.stops[b]["node"]
            if na == nb:
                continue
            path = NET.shortest_path(na, nb)
            self.assertAlmostEqual(sum(NET.edges[e]["length"] for e, _ in path), d[na, nb], places=9)

    def test_a_route_runs_from_the_origin_node_to_the_destination_node(self):
        for a, b in (("S3", "AB4"), ("AB1", "S6"), ("S4", "S3")):
            road = NET.route_road(a, b)
            start, end = road["centreline"][0], road["centreline"][-1]
            na, nb = NET.nodes[NET.stops[a]["node"]], NET.nodes[NET.stops[b]["node"]]
            self.assertAlmostEqual(math.hypot(start["x"] - na["x"], start["y"] - na["y"]), 0.0, places=6)
            self.assertAlmostEqual(math.hypot(end["x"] - nb["x"], end["y"] - nb["y"]), 0.0, places=6)
            dr.check_road(road)

    def test_trips_follow_the_demand_weights(self):
        rng = random.Random(1)
        origins = collections.Counter(NET.sample_trip(rng)[0] for _ in range(20000))
        total_w = sum(s["weight"] for s in NET.stops.values())
        for name, s in NET.stops.items():
            self.assertAlmostEqual(origins[name] / 20000, s["weight"] / total_w, delta=0.015, msg=name)
        for _ in range(500):
            o, dst = NET.sample_trip(rng)
            self.assertNotEqual(NET.stops[o]["node"], NET.stops[dst]["node"])


class Env(unittest.TestCase):
    def test_the_baseline_drives_campus_trips_including_the_longest(self):
        env = campus.CampusEnv(seed=0, ranges=campus.envmod.RANGES_TRAIN, lib_path=LIB)
        pairs = [(a, b) for a, b in itertools.permutations(sorted(NET.stops), 2)
                 if NET.stops[a]["node"] != NET.stops[b]["node"]]
        longest = max(pairs, key=lambda p: length_of(NET.route_road(*p)))
        for k, trip in enumerate([longest, ("S3", "AB4"), ("AB4", "S3")]):
            obs, info = env.reset(seed=k, options={"trip": trip})
            done = False
            while not done:
                obs, r, te, tr, info = env.step(campus.envmod.pure_pursuit_policy(obs))
                done = te or tr
            self.assertEqual(info["status"], "reached_end", trip)
            self.assertEqual(info["trip"], trip)

    def test_episodes_draw_different_trips_reproducibly(self):
        env = campus.CampusEnv(seed=0, ranges=campus.envmod.RANGES_NOMINAL, lib_path=LIB)
        trips = [env.reset(seed=k)[1]["trip"] for k in range(8)]
        again = [env.reset(seed=k)[1]["trip"] for k in range(8)]
        self.assertEqual(trips, again)
        self.assertGreater(len(set(trips)), 3)


if __name__ == "__main__":
    unittest.main()
