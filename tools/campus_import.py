#!/usr/bin/env python3
"""Convert the campus fleet sim's exported geometry into the twin's data.

Input: campus/campus_export.json, written by a temporary editor script run in
Sujay-197/campus_fleet_man_unity (see campus/README.md): the fleet sim's road
graph, its bus stops, every road tile with its measured surfaces, and the
building footprints, in Unity metres and Unity's frame.

Output, at 1/10 scale in the ISO 8855 frame (x forward = Unity z, y left =
-Unity x, z up = Unity y):
  campus/campus_network.json   nodes, edges (centreline polylines), stops,
                               buildings, and the measured road dimensions
  roads/campus/edge_NN.json    one roads/road.schema.json file per edge, so
                               the edge profiler can cost the real campus

1/10 because the campus is modelled at full scale (its city pack is sized for
real cars) and the twin is a 1/10-scale car: a 3.5 m lane becomes 0.35 m.
The drivable width is the asphalt, measured from the road tiles' meshes; the
sidewalks are raised kerbs (6 cm at this scale) that the car cannot climb.

Usage:
    python tools/campus_import.py
"""

from __future__ import annotations

import json
import math
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
SCALE = 0.1
SOURCE = {
    "repository": "Sujay-197/campus_fleet_man_unity",
    "commit": "b7452205da2d2a7fad37761adc260761a975b2fc",
    "scene": "Assets/Scenes/SampleScene.unity",
    "exporter": "temporary editor script, see campus/README.md",
}


def iso(p: list[float]) -> tuple[float, float, float]:
    """Unity world point (metres) to ISO at 1/10."""
    return (p[2] * SCALE, -p[0] * SCALE, p[1] * SCALE)


def measured_dimensions(export: dict) -> dict:
    """Asphalt half-width, the sidewalks' outer half-width (their surface
    spans both kerbs, asphalt between) and kerb height from the straight
    tiles, in Unity metres (the tile frame's x is across the road)."""
    widths, walks, kerbs = [], [], []
    for t in export["road_tiles"]:
        if t["prefab"] != "straightRoad":
            continue
        for part in t["parts"]:
            half = 0.5 * (part["hi"][0] - part["lo"][0])
            if part["material"] == "intersection road":
                widths.append(half)
            elif part["material"] == "side walk":
                walks.append(half)
                kerbs.append(part["hi"][1] - part["lo"][1])
    if not widths:
        raise ValueError("no straight road tiles in the export")
    for name, vals in (("asphalt", widths), ("sidewalk", walks), ("kerb", kerbs)):
        if max(vals) - min(vals) > 1e-3:
            raise ValueError(f"straight tiles disagree on {name} size: {sorted(set(vals))}")
    return {"asphalt_half_width": widths[0], "sidewalk_outer_half_width": walks[0],
            "kerb_height": kerbs[0]}


def clean(points: list[tuple]) -> list[tuple]:
    """Drop consecutive points that coincide in plan (the road API refuses them)."""
    out = []
    for p in points:
        if out and math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) < 1e-6:
            continue
        out.append(p)
    return out


def convert(export: dict) -> tuple[dict, dict[int, dict]]:
    dims = measured_dimensions(export)
    width = 2.0 * dims["asphalt_half_width"] * SCALE
    nodes = [{"id": n["id"], "x": iso(n["p"])[0], "y": iso(n["p"])[1], "z": iso(n["p"])[2]}
             for n in export["nodes"]]
    edges, roads = [], {}
    for e in export["edges"]:
        poly = clean([iso(p) for p in e["polyline"]])
        if len(poly) < 2:
            raise ValueError(f"edge {e['id']} has a degenerate polyline")
        length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(poly, poly[1:]))
        edges.append({"id": e["id"], "a": e["a"], "b": e["b"], "length": length,
                      "polyline": [list(p) for p in poly]})
        roads[e["id"]] = {
            "schema_version": "0.1.0",
            "id": f"campus-edge-{e['id']:02d}",
            "description": f"Campus fleet sim edge {e['id']} (node {e['a']} to node {e['b']}), "
                           f"1/10 scale, from {SOURCE['repository']}@{SOURCE['commit'][:7]}.",
            "width": width,
            "centreline": [{"x": p[0], "y": p[1], "z": p[2]} for p in poly],
        }
    stops = []
    for s in sorted(export["stops"], key=lambda s: s["stop_id"]):
        x, y, _ = iso(s["p"])
        stops.append({"name": s["name"], "stop_id": s["stop_id"], "node": s["node"],
                      "weight": s["weight"], "x": x, "y": y})
    buildings = []
    for b in export["buildings"]:
        lo, hi = b["lo"], b["hi"]
        # Footprint centre in the building's frame, then to the world. Unity yaw
        # is clockwise from above about +y; ISO yaw is counter-clockwise.
        cx, cz = 0.5 * (lo[0] + hi[0]), 0.5 * (lo[2] + hi[2])
        yaw_u = math.radians(b["yaw_deg"])
        wx = b["p"][0] + cx * math.cos(yaw_u) + cz * math.sin(yaw_u)
        wz = b["p"][2] - cx * math.sin(yaw_u) + cz * math.cos(yaw_u)
        x, y, _ = iso([wx, 0.0, wz])
        buildings.append({"name": b["name"], "prefab": b["prefab"], "x": x, "y": y,
                          "yaw": -yaw_u,
                          "half_length": 0.5 * (hi[2] - lo[2]) * SCALE,
                          "half_width": 0.5 * (hi[0] - lo[0]) * SCALE,
                          "height": (hi[1] - lo[1]) * SCALE})
    network = {
        "format": "rc-physics-twin campus network 0.1.0",
        "source": SOURCE,
        "scale": SCALE,
        "frame": "ISO 8855: x = Unity z, y = -Unity x, z = Unity y, all x 0.1",
        "road": {"width": width,
                 "sidewalk_width": (dims["sidewalk_outer_half_width"] - dims["asphalt_half_width"]) * SCALE,
                 "kerb_height": dims["kerb_height"] * SCALE,
                 "note": "drivable width is the asphalt; a sidewalk of sidewalk_width runs along each side, "
                         "kerb_height up: a kerb the car cannot climb"},
        "nodes": nodes,
        "edges": edges,
        "stops": stops,
        "buildings": buildings,
    }
    return network, roads


def main() -> int:
    export = json.loads((ROOT / "campus" / "campus_export.json").read_text(encoding="utf-8"))
    network, roads = convert(export)
    (ROOT / "campus" / "campus_network.json").write_text(json.dumps(network, indent=1) + "\n",
                                                          encoding="utf-8")
    out = ROOT / "roads" / "campus"
    out.mkdir(parents=True, exist_ok=True)
    for old in out.glob("edge_*.json"):
        old.unlink()
    for eid, road in roads.items():
        (out / f"edge_{eid:02d}.json").write_text(json.dumps(road, indent=1) + "\n", encoding="utf-8")
    total = sum(e["length"] for e in network["edges"])
    print(f"campus_import: {len(network['nodes'])} nodes, {len(network['edges'])} edges "
          f"({total:.1f} m at 1/10), {len(network['stops'])} stops, "
          f"{len(network['buildings'])} buildings; road width {network['road']['width']:.3f} m",
          file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
