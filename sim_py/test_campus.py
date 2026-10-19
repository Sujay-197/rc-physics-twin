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


def dist_to_polyline(q, poly) -> float:
    best = math.inf
    for a, b in zip(poly, poly[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        t = max(0.0, min(1.0, ((q[0] - a[0]) * dx + (q[1] - a[1]) * dy) / (dx * dx + dy * dy)))
        best = min(best, math.hypot(a[0] + t * dx - q[0], a[1] + t * dy - q[1]))
    return best


class Walls(unittest.TestCase):
    def test_chains_cover_every_edge_once(self):
        ids = [i for chain, _ in importer.chains(NET.data["nodes"], NET.data["edges"]) for i in chain]
        self.assertEqual(sorted(ids), sorted(NET.edges))

    def test_chains_follow_a_loop_with_no_junction(self):
        # A square of four edges, every node with two: one chain of all four.
        sq = [(0, 0), (1, 0), (1, 1), (0, 1)]
        nodes = [{"id": i} for i in range(4)]
        edges = [{"id": i, "a": i, "b": (i + 1) % 4,
                  "polyline": [[*sq[i], 0.0], [*sq[(i + 1) % 4], 0.0]]} for i in range(4)]
        out = importer.chains(nodes, edges)
        self.assertEqual(len(out), 1)
        self.assertEqual(sorted(out[0][0]), [0, 1, 2, 3])

    def test_every_road_is_kerbed_both_sides_away_from_junctions(self):
        # Sample each edge every 5 cm; away from junctions (nodes with other
        # than two edges) there must be a kerb wall at the asphalt edge on
        # both sides, so there is no gap a car could leave the road through.
        half = 0.5 * NET.width
        degree = collections.Counter()
        for e in NET.edges.values():
            degree[e["a"]] += 1
            degree[e["b"]] += 1
        junctions = [NET.nodes[n] for n, d in degree.items() if d != 2]
        segs = [(a, b) for w in NET.walls if w["name"].startswith("kerb")
                for a, b in zip(w["points"], w["points"][1:])]
        for e in NET.edges.values():
            poly = e["polyline"]
            for a, b in zip(poly, poly[1:]):
                L = math.dist(a[:2], b[:2])
                ux, uy = (b[0] - a[0]) / L, (b[1] - a[1]) / L
                for k in range(1, int(L / 0.05)):
                    c = (a[0] + ux * 0.05 * k, a[1] + uy * 0.05 * k)
                    if min(math.hypot(c[0] - j["x"], c[1] - j["y"]) for j in junctions) < 2.0 * NET.width:
                        continue
                    if min(math.dist(c, p[:2]) for p in poly) < half:
                        # Near a vertex: on the inside of a bend the kerb turns
                        # at the mitre, half width x tan(half the bend) early.
                        continue
                    for side in (-1, 1):
                        q = (c[0] - side * uy * half, c[1] + side * ux * half)
                        near = min(dist_to_polyline(q, [s0, s1]) for s0, s1 in segs)
                        self.assertLess(near, 0.01, f"edge {e['id']} at {c}, side {side}")

    def test_no_wall_reaches_into_any_road(self):
        # Every wall, sampled densely, keeps at least the asphalt's half width
        # from every centreline: the kerbs bound the roads, never cut them.
        half = 0.5 * NET.width
        for w in NET.walls:
            pts = w["points"] + ([w["points"][0]] if w["closed"] else [])
            for a, b in zip(pts, pts[1:]):
                for k in range(21):
                    q = (a[0] + (b[0] - a[0]) * k / 20, a[1] + (b[1] - a[1]) * k / 20)
                    near = min(dist_to_polyline(q, e["polyline"]) for e in NET.edges.values())
                    self.assertGreater(near, half - 1e-4, w["name"])   # 0.1 mm: mitres at graph kinks

    def test_kerbs_run_along_the_roads_at_the_asphalt_edge(self):
        kerbs = [w for w in NET.walls if w["name"].startswith("kerb")]
        self.assertEqual(len(kerbs), len(NET.data["sidewalks"]))
        total = 0.0
        for w in kerbs:
            for q in w["points"]:
                near = min(dist_to_polyline(q, e["polyline"]) for e in NET.edges.values())
                # On the asphalt edge. A mitred point on the outside of a bend
                # is half width / cos(half the bend) from the corner: under
                # sqrt(2) times it for the campus's bends, all under 90 deg.
                self.assertGreater(near, 0.5 * NET.width - 1e-4, w["name"])
                self.assertLess(near, 0.5 * NET.width * math.sqrt(2.0), w["name"])
            total += sum(math.dist(a, b) for a, b in zip(w["points"], w["points"][1:]))
        # Both sides of 132.7 m of road, less the junction mouths.
        self.assertGreater(total, 2 * 132.7 * 0.85)
        self.assertLess(total, 2 * 132.7 * 1.05)
        self.assertEqual(sum(w["name"].startswith("building") for w in NET.walls), len(NET.data["buildings"]))


class Profiles(unittest.TestCase):
    def test_the_committed_edge_profiles_are_current(self):
        # profiles/campus_edges.json is the fleet sim's input for the campus.
        # Rerunning the profiler with the same sweep must reuse every entry:
        # a change to the car, the controller or the roads that moves a cost
        # shows up here until the cache is regenerated (campus/README.md).
        import shutil, subprocess, sys, tempfile
        with tempfile.TemporaryDirectory() as tmp:
            out = pathlib.Path(tmp) / "cache.json"
            shutil.copy(ROOT / "profiles" / "campus_edges.json", out)
            lib = LIB or str(ROOT / "build" / "bin" / "rc_twin.dll")
            r = subprocess.run([sys.executable, "-I", str(ROOT / "tools" / "edge_profiler.py"),
                                *sorted(str(x) for x in (ROOT / "roads" / "campus").glob("edge_*.json")),
                                "--out", str(out), "--speeds", "1", "2", "3", "--payloads", "0", "1",
                                "--soc", "0.8", "0.4", "--lib", lib], capture_output=True, text=True)
            self.assertEqual(r.returncode, 0, r.stderr)
            self.assertIn("444 entries (0 computed, 444 reused, 0 dropped)", r.stderr + r.stdout)
            cache = json.loads(out.read_text(encoding="utf-8"))
            self.assertTrue(all(e["status"] == "reached_end" for e in cache["entries"]))


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

    def test_the_baseline_never_touches_a_kerb(self):
        env = campus.CampusEnv(seed=0, ranges=campus.envmod.RANGES_TRAIN, lib_path=LIB)
        for k, trip in enumerate([("S3", "AB4"), ("AB1", "S6"), ("S4", "S3")]):
            obs, info = env.reset(seed=k, options={"trip": trip})
            done = False
            while not done:
                obs, r, te, tr, info = env.step(campus.envmod.pure_pursuit_policy(obs))
                done = te or tr
            self.assertEqual(info["status"], "reached_end", trip)
            self.assertEqual(info["wall_contacts"], 0, trip)

    def test_the_footprint_is_widened_by_the_drawn_tyre(self):
        env = campus.CampusEnv(seed=0, ranges=campus.envmod.RANGES_NOMINAL, lib_path=LIB)
        env.reset(seed=0)
        got, base = dr.Contact(), dr.Contact()
        env.lib.rc_twin_get_contact(env._car, __import__("ctypes").byref(got))
        env.lib.rc_twin_contact_default(__import__("ctypes").byref(env.params), __import__("ctypes").byref(base))
        self.assertAlmostEqual(got.half_width, base.half_width + campus.envmod.FOOTPRINT_MARGIN, places=12)
        self.assertGreater(env.lib.rc_twin_wall_count(env._car), 100)

    def test_a_kerb_strike_bounces_and_costs_instead_of_ending(self):
        # Hard left at full throttle from the start of a trip: without walls
        # the car leaves the road and the episode ends; with the campus's
        # walls it hits the kerb, keeps going, and pays the wall term.
        out = {}
        for walls in (False, True):
            env = campus.CampusEnv(seed=0, ranges=campus.envmod.RANGES_NOMINAL, lib_path=LIB, walls=walls)
            obs, info = env.reset(seed=0, options={"trip": ("S3", "AB4")})
            worst, first_hit, k = 0.0, None, 0
            for k in range(300):
                obs, r, te, tr, info = env.step((1.0, 1.0))
                if info["wall_contacts"] and first_hit is None:
                    first_hit = (k, r)
                worst = min(worst, r)
                if te or tr:
                    break
            out[walls] = (info, first_hit, k)
        no_walls, _, k_end = out[False]
        self.assertEqual(no_walls["status"], "left_road")
        self.assertEqual(no_walls["wall_contacts"], 0)
        info, first_hit, k = out[True]
        self.assertIsNotNone(first_hit, "it hit a kerb")
        self.assertLess(first_hit[0], k_end + 1, "where the wall-less car left the road")
        self.assertLess(first_hit[1], 0.0, "and the strike cost reward")
        self.assertGreater(info["wall_impulse"], 0.0)

    def test_episodes_draw_different_trips_reproducibly(self):
        env = campus.CampusEnv(seed=0, ranges=campus.envmod.RANGES_NOMINAL, lib_path=LIB)
        trips = [env.reset(seed=k)[1]["trip"] for k in range(8)]
        again = [env.reset(seed=k)[1]["trip"] for k in range(8)]
        self.assertEqual(trips, again)
        self.assertGreater(len(set(trips)), 3)


if __name__ == "__main__":
    unittest.main()
