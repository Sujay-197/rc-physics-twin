#!/usr/bin/env python3
"""Convert the campus fleet sim's exported geometry into the twin's data.

Input: campus/campus_export.json, written by a temporary editor script run in
Sujay-197/campus_fleet_man_unity (see campus/README.md): the fleet sim's road
graph, its bus stops, every road tile with its measured surfaces, and the
building footprints, in Unity metres and Unity's frame.

Output, at 1/10 scale in the ISO 8855 frame (x forward = Unity z, y left =
-Unity x, z up = Unity y):
  campus/campus_network.json   nodes, edges (centreline polylines), stops,
                               buildings, the measured road dimensions, the
                               sidewalk runs, and the walls (kerb lines and
                               building footprints, ADR 0009) that the Unity
                               scene and the RL environment share
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


STEP = 0.05   # resampling of the centrelines for kerb trimming [m]


def _resample(pts: list) -> tuple[list, list]:
    """Points every STEP metres along each segment (x, y, z), and which were
    corners of the input."""
    out, corner = [list(pts[0])], [True]
    for a, b in zip(pts, pts[1:]):
        n = max(1, math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / STEP))
        for k in range(1, n + 1):
            t = k / n
            out.append([a[i] + (b[i] - a[i]) * t for i in range(3)])
            corner.append(k == n)
    return out, corner


def right_normals(pts: list) -> list:
    """Unit normals to the right of the line in plan, mitred at corners, so
    an offset line keeps its distance round them. The Unity scene offsets
    sidewalk runs with the same rule."""
    seg = []
    for a, b in zip(pts, pts[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        L = math.hypot(dx, dy)
        seg.append((dy / L, -dx / L))
    out = [seg[0]]
    for s0, s1 in zip(seg, seg[1:]):
        mx, my = s0[0] + s1[0], s0[1] + s1[1]
        L = math.hypot(mx, my)
        c = (mx * s1[0] + my * s1[1]) / L if L > 0 else 0.0
        out.append((mx / L / c, my / L / c) if c > 0.2 else s1)
    out.append(seg[-1])
    return out


def chains(nodes: list, edges: list) -> list:
    """The roads between junctions: edges joined through every node with two
    edges, so a kerb runs on round a joint instead of stopping at it. Each
    chain is (edge ids, polyline), in a deterministic order."""
    degree = {n["id"]: 0 for n in nodes}
    at = {n["id"]: [] for n in nodes}
    for e in edges:
        degree[e["a"]] += 1
        degree[e["b"]] += 1
        at[e["a"]].append(e)
        at[e["b"]].append(e)
    used, out = set(), []

    def walk(start_node, first):
        ids, poly, node, e = [], [], start_node, first
        while e is not None and e["id"] not in used:
            used.add(e["id"])
            ids.append(e["id"])
            seg = e["polyline"] if e["a"] == node else e["polyline"][::-1]
            poly += seg if not poly else seg[1:]
            node = e["b"] if e["a"] == node else e["a"]
            nxt = [x for x in at[node] if x["id"] not in used]
            e = nxt[0] if degree[node] == 2 and nxt else None
        return ids, poly

    for n in sorted(degree):
        if degree[n] == 2:
            continue
        for e in sorted(at[n], key=lambda x: x["id"]):
            if e["id"] not in used:
                out.append(walk(n, e))
    for e in sorted(edges, key=lambda x: x["id"]):   # loops with no junction
        if e["id"] not in used:
            out.append(walk(e["a"], e))
    return out


def _near_other_road(q, own: set, edges: list, r: float) -> bool:
    """Whether q is within r of a centreline outside its own chain, by more
    than a millimetre of rounding."""
    for e in edges:
        if e["id"] in own:
            continue
        for a, b in zip(e["polyline"], e["polyline"][1:]):
            ax, ay, bx, by = a[0], a[1], b[0], b[1]
            dx, dy = bx - ax, by - ay
            t = max(0.0, min(1.0, ((q[0] - ax) * dx + (q[1] - ay) * dy) / max(dx * dx + dy * dy, 1e-24)))
            if math.hypot(ax + t * dx - q[0], ay + t * dy - q[1]) < r - 1e-3:
                return True
    return False


def sidewalk_runs(nodes: list, edges: list, width: float, sidewalk: float) -> list:
    """Each road's sidewalks either side as centreline runs: left out where
    its kerb would run onto another road's asphalt (the junctions),
    thinned to each run's ends and the corners within it. side -1 is left of
    the run's direction, +1 right; the sidewalk spans width/2 to width/2 +
    sidewalk from the run, and its kerb line (at width/2) is a wall."""
    inner = 0.5 * width
    out = []
    for ids, poly in chains(nodes, edges):
        pts, corner = _resample(poly)
        n = right_normals(pts)
        own = set(ids)
        for side, name in ((-1, "left"), (1, "right")):
            # The kerb line must be clear of every other road's asphalt. (The
            # sidewalk's middle, further out, then is too; testing the kerb
            # rather than the middle is what keeps a kerb from ending inside
            # the other road at a sharp junction.)
            keep = [not _near_other_road((p[0] + nx * side * inner, p[1] + ny * side * inner), own, edges, inner)
                    for p, (nx, ny) in zip(pts, n)]
            runs, run = [], None
            for i, p in enumerate(pts):
                if not keep[i]:
                    if run and len(run) >= 2:
                        runs.append(run)
                    run = None
                    continue
                last = i + 1 == len(pts) or not keep[i + 1]
                if run is None:
                    run = [p]
                elif corner[i] or last:
                    run.append(p)
            if run and len(run) >= 2:
                runs.append(run)
            for k, r in enumerate(runs):
                label = "edges " + "-".join(str(i) for i in ids) + f", {name}"
                out.append({"name": label + (f", part {k + 1}" if len(runs) > 1 else ""),
                            "side": side, "points": r})
    return out


def kerb_walls(runs: list, width: float) -> list:
    inner = 0.5 * width
    walls = []
    for r in runs:
        rn = right_normals(r["points"])
        line = [[p[0] + nx * r["side"] * inner, p[1] + ny * r["side"] * inner]
                for p, (nx, ny) in zip(r["points"], rn)]
        walls.append({"name": "kerb, " + r["name"], "closed": False, "points": line})
    return walls


def building_walls(buildings: list) -> list:
    out = []
    for b in buildings:
        c, s = math.cos(b["yaw"]), math.sin(b["yaw"])
        hl, hw = b["half_length"], b["half_width"]
        pts = [[b["x"] + c * hl * a - s * hw * w, b["y"] + s * hl * a + c * hw * w]
               for a, w in ((1, 1), (1, -1), (-1, -1), (-1, 1))]
        out.append({"name": f"building {b['name']}", "closed": True, "points": pts})
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
    sidewalk = (dims["sidewalk_outer_half_width"] - dims["asphalt_half_width"]) * SCALE
    network["sidewalks"] = sidewalk_runs(nodes, edges, width, sidewalk)
    network["walls"] = kerb_walls(network["sidewalks"], width) + building_walls(buildings)
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
          f"{len(network['buildings'])} buildings, {len(network['walls'])} walls; "
          f"road width {network['road']['width']:.3f} m",
          file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
