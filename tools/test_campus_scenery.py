"""Tests for campus/scenery.json (tools/campus_scenery.py): the real campus
drawn around the fleet sim's roads never lands on them. Standard library
only: it checks the committed file, not the fit.

    python -I tools/test_campus_scenery.py
"""

from __future__ import annotations

import json
import math
import pathlib
import unittest

ROOT = pathlib.Path(__file__).resolve().parent.parent
SCENERY = json.loads((ROOT / "campus" / "scenery.json").read_text(encoding="utf-8"))
NET = json.loads((ROOT / "campus" / "campus_network.json").read_text(encoding="utf-8"))
ROADS = [[(q[0], q[1]) for q in e["polyline"]] for e in NET["edges"]]
SEGS = [(a, b) for r in ROADS for a, b in zip(r[:-1], r[1:])]
KERB = 0.5 * NET["road"]["width"] + NET["road"]["sidewalk_width"]


def pt_seg(p, a, b):
    ax, ay = b[0] - a[0], b[1] - a[1]
    t = max(0.0, min(1.0, ((p[0] - a[0]) * ax + (p[1] - a[1]) * ay) / max(ax * ax + ay * ay, 1e-12)))
    return math.hypot(a[0] + t * ax - p[0], a[1] + t * ay - p[1])


def cross(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def seg_seg(p1, p2, q1, q2):
    if cross(p1, p2, q1) * cross(p1, p2, q2) < 0 and cross(q1, q2, p1) * cross(q1, q2, p2) < 0:
        return 0.0
    return min(pt_seg(p1, q1, q2), pt_seg(p2, q1, q2), pt_seg(q1, p1, p2), pt_seg(q2, p1, p2))


def inside(p, poly):
    c = False
    for i in range(len(poly)):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % len(poly)]
        if (y1 > p[1]) != (y2 > p[1]) and p[0] < x1 + (p[1] - y1) * (x2 - x1) / (y2 - y1):
            c = not c
    return c


def area(poly):
    return 0.5 * sum(poly[i][0] * poly[(i + 1) % len(poly)][1] - poly[(i + 1) % len(poly)][0] * poly[i][1]
                     for i in range(len(poly)))


class CampusScenery(unittest.TestCase):
    def test_it_says_where_it_comes_from(self):
        self.assertIn("OpenStreetMap contributors", SCENERY["attribution"])
        self.assertIn("ODbL", SCENERY["attribution"])
        self.assertTrue((ROOT / SCENERY["osm_extract"]).exists())
        self.assertAlmostEqual(SCENERY["clearance_m"], KERB + 0.15, places=3)

    def test_buildings_stay_off_the_roads_and_kerbs(self):
        self.assertGreater(len(SCENERY["buildings"]), 50)
        for b in SCENERY["buildings"]:
            fp = [tuple(p) for p in b["footprint"]]
            self.assertGreater(area(fp), 0.0, f"{b['name'] or b['osm_id']}: counter-clockwise")
            self.assertGreater(b["height"], 0.0)
            ring = fp + fp[:1]
            for a, c in SEGS:
                self.assertFalse(inside(a, fp), f"{b['name'] or b['osm_id']}: a road runs through it")
                d = min(seg_seg(p, q, a, c) for p, q in zip(ring[:-1], ring[1:]))
                self.assertGreaterEqual(d, SCENERY["clearance_m"] - 1e-3, f"{b['name'] or b['osm_id']}")

    def test_buildings_do_not_overlap(self):
        fps = [[tuple(p) for p in b["footprint"]] for b in SCENERY["buildings"]]
        for i, p in enumerate(fps):
            for q in fps[i + 1:]:
                self.assertFalse(any(inside(v, q) for v in p) or any(inside(v, p) for v in q))

    def test_the_named_blocks_stand_where_the_fleet_sim_puts_them(self):
        stands = {s: b["name"] for b in SCENERY["buildings"] for s in b["stands_for"]}
        self.assertEqual(stands.get("AB1"), "Academic Block 1 - School of Engineering")
        self.assertEqual(stands.get("AB2"), "Academic Block 2")

    def test_side_roads_keep_off_the_fleet_sim_roads(self):
        self.assertTrue(SCENERY["side_roads"])
        for r in SCENERY["side_roads"]:
            for p in r["points"]:
                d = min(pt_seg(p, a, b) for a, b in SEGS)
                self.assertGreater(d, KERB + 0.5 * r["width"], r["kind"])

    def test_the_fit_is_recorded(self):
        f = SCENERY["fit"]
        self.assertGreater(f["road_points_matched"], 80)
        self.assertTrue(1.5 < f["real_per_fleet_sim_m"] < 2.0, "the fleet sim is drawn at a little over half size")


if __name__ == "__main__":
    unittest.main()
