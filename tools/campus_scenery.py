#!/usr/bin/env python3
"""Real-campus scenery for the twin: Amrita Vishwa Vidyapeetham, Coimbatore,
fitted around the fleet sim's road network (campus/README.md).

The fleet sim's roads are a simplified version of the real campus: about
0.57 of real size, its roads mostly within 16 m of the real ones, and a few
that do not exist (across the playground). They stay exactly as they are,
since the fleet sim, the RL environment and the policy study all use them.
The real campus is fitted around them instead:

  1. similarity: real (OpenStreetMap, metres east/north of LAT0, LON0) to
     the fleet sim frame, from the three academic blocks both maps name,
     refined on the roads (iterative closest point);
  2. thin-plate-spline warp: points along each fleet sim road are matched to
     the real road they stand for (near and parallel; no match where the
     fleet sim has a road the campus has not), and the warp carries the real
     roads onto them, and every building, lawn and path with them;
  3. culling: whatever lands on a road or its kerbs moves clear (buildings,
     up to MAX_PUSH) or goes (side roads that duplicate a fleet sim road,
     buildings that cannot move clear).

Outputs, all drawn only (no colliders, nothing the car or the RL env feels):
  campus/scenery.json   buildings (footprint, height, kind), lawns and other
                        land use, side roads and paths, in the twin's frame
                        (ISO, 1/10). From OpenStreetMap: ODbL, attribution
                        in the file and in THIRD_PARTY.md.
  with --images (local only, campus/reference/, not in git: map screenshots
  are third-party imagery):
  campus/reference/vegetation.json   trees and a ground-cover grid (grass,
                        bare earth, tree cover) read from the screenshots
                        after pinning each to the real map on its road
                        overlay;
  and in every case the ground texture the Unity builder lays on the grass:
  unity/RcTwin/Assets/RcTwin/Generated/Campus/Local/Ground.png (+ .json for
  its extent), kept out of git with the rest of Local/.

    python tools/campus_scenery.py fetch            # OpenStreetMap extract -> campus/osm/
    python tools/campus_scenery.py build [--images campus/reference/images.json]

Needs numpy and Pillow (a plain Python with them; not the RL .venv).
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import random
import sys
import time
import urllib.parse
import urllib.request

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent.parent
NETWORK = ROOT / "campus" / "campus_network.json"
OSM = ROOT / "campus" / "osm" / "amrita.osm.json"
SCENERY = ROOT / "campus" / "scenery.json"
LOCAL = ROOT / "unity" / "RcTwin" / "Assets" / "RcTwin" / "Generated" / "Campus" / "Local"

FORMAT = "rc-physics-twin campus scenery 0.1.0"
LAT0, LON0 = 10.9024056, 76.9029569           # the campus view the references were taken from
BBOX = (10.8935, 76.8920, 10.9115, 76.9140)   # south, west, north, east
EARTH_R = 6378137.0
TWIN = 0.1                                     # twin metres per fleet sim metre
OVERPASS = ("https://overpass-api.de/api/interpreter",
            "https://maps.mail.ru/osm/tools/overpass/api/interpreter",
            "https://overpass.kumi.systems/api/interpreter")
ATTRIBUTION = "Map data (c) OpenStreetMap contributors, ODbL 1.0, https://www.openstreetmap.org/copyright"

# Fleet sim buildings and the real ones they stand for (OpenStreetMap names).
ANCHORS = {"AB1": "Academic Block 1 - School of Engineering", "AB2": "Academic Block 2",
           "AB3": "Academic Block 3 - International school of Law"}
ROAD_TAGS = ("primary", "secondary", "tertiary", "unclassified", "residential", "service", "living_street")
PATH_TAGS = ("footway", "path", "pedestrian", "cycleway", "track", "steps")
# Real widths, drawn at 1/10 and at the fleet sim's compression (about 1/1.75).
SIDE_ROAD_WIDTH = 0.42                         # twin m
PATH_WIDTH = 0.12
MARGIN = 0.15                                  # twin m kept clear beyond the kerb
MAX_PUSH = 2.5                                 # twin m a building may move to clear a road
LEVEL_M = 3.2


# ------------------------------------------------------------------ data

def enu(lat, lon):
    return (math.radians(lon - LON0) * EARTH_R * math.cos(math.radians(LAT0)),
            math.radians(lat - LAT0) * EARTH_R)


def fetch(out: pathlib.Path = OSM) -> None:
    s, w, n, e = BBOX
    box = f"({s},{w},{n},{e})"
    q = f'[out:json][timeout:90];(way{box};relation["type"="multipolygon"]{box};);out body;>;out skel qt;'
    data = urllib.parse.urlencode({"data": q}).encode()
    last = None
    for url in OVERPASS:
        try:
            req = urllib.request.Request(url, data=data, headers={"User-Agent": "rc-physics-twin campus_scenery"})
            body = urllib.request.urlopen(req, timeout=180).read()
            doc = json.loads(body)
            out.parent.mkdir(parents=True, exist_ok=True)
            doc["fetched"] = {"from": url, "bbox": BBOX, "at": time.strftime("%Y-%m-%d"), "attribution": ATTRIBUTION}
            out.write_text(json.dumps(doc, ensure_ascii=False, separators=(",", ":")) + "\n", encoding="utf-8")
            print(f"wrote {out} from {url}: {len(doc['elements'])} elements")
            return
        except Exception as exc:          # a busy mirror: try the next
            last = exc
            print(f"{url}: {exc}", file=sys.stderr)
    raise SystemExit(f"no Overpass mirror answered: {last}")


def load_osm(path: pathlib.Path = OSM) -> list[dict]:
    """Ways (and multipolygon outer rings) as {"tags", "pts" (n x 2, metres
    east/north), "closed"}."""
    doc = json.loads(path.read_text(encoding="utf-8"))
    nodes = {e["id"]: enu(e["lat"], e["lon"]) for e in doc["elements"] if e["type"] == "node" and "lat" in e}
    ways = {e["id"]: e for e in doc["elements"] if e["type"] == "way"}
    out = []
    for w in ways.values():
        p = [nodes[n] for n in w.get("nodes", []) if n in nodes]
        if len(p) >= 2:
            out.append({"id": w["id"], "tags": w.get("tags", {}), "pts": np.array(p),
                        "closed": w["nodes"][0] == w["nodes"][-1]})
    for r in (e for e in doc["elements"] if e["type"] == "relation"):
        outer = [ways[m["ref"]] for m in r.get("members", []) if m["type"] == "way" and m.get("role") == "outer"
                 and m["ref"] in ways]
        for ring in _rings([[n for n in w["nodes"] if n in nodes] for w in outer]):
            out.append({"id": r["id"], "tags": r.get("tags", {}), "pts": np.array([nodes[n] for n in ring]),
                        "closed": True})
    return out


def _rings(parts):
    """Join way node lists end to end into closed rings."""
    parts = [p for p in parts if len(p) >= 2]
    rings = []
    while parts:
        ring = parts.pop(0)
        changed = True
        while ring[0] != ring[-1] and changed:
            changed = False
            for i, p in enumerate(parts):
                if p[0] == ring[-1]:
                    ring += p[1:]
                elif p[-1] == ring[-1]:
                    ring += p[::-1][1:]
                elif p[-1] == ring[0]:
                    ring = p[:-1] + ring
                elif p[0] == ring[0]:
                    ring = p[::-1][:-1] + ring
                else:
                    continue
                parts.pop(i)
                changed = True
                break
        if ring[0] == ring[-1] and len(ring) >= 4:
            rings.append(ring)
    return rings


def network():
    net = json.loads(NETWORK.read_text(encoding="utf-8"))
    roads = [np.array([[q[0], q[1]] for q in e["polyline"]]) / TWIN for e in net["edges"]]   # fleet sim metres
    return net, roads


# ------------------------------------------------------------- geometry

def umeyama(src, dst):
    """Similarity dst ~ s R src + t (rotation only), least squares."""
    ms, md = src.mean(0), dst.mean(0)
    s0, d0 = src - ms, dst - md
    U, S, Vt = np.linalg.svd(d0.T @ s0 / len(src))
    D = np.eye(2)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        D[1, 1] = -1
    Rm = U @ D @ Vt
    s = np.trace(np.diag(S) @ D) / (s0 ** 2).sum(1).mean()
    return s, Rm, md - s * Rm @ ms


def segments(polys):
    a = np.array([p for poly in polys for p in poly[:-1]])
    b = np.array([p for poly in polys for p in poly[1:]])
    return a, b


def nearest_on(a, b, q, chunk=2048):
    """Nearest point on segments a->b for each q: (points, distances, unit directions, segment index)."""
    ab = b - a
    L2 = np.maximum((ab * ab).sum(1), 1e-12)
    out_p, out_d, out_i = [], [], []
    for k in range(0, len(q), chunk):
        qq = q[k:k + chunk]
        t = np.clip(((qq[:, None] - a[None]) * ab[None]).sum(-1) / L2[None], 0, 1)
        proj = a[None] + t[..., None] * ab[None]
        dist = np.linalg.norm(proj - qq[:, None], axis=-1)
        i = dist.argmin(1)
        r = np.arange(len(qq))
        out_p.append(proj[r, i]); out_d.append(dist[r, i]); out_i.append(i)
    i = np.concatenate(out_i)
    u = ab[i] / np.sqrt(L2[i])[:, None]
    return np.concatenate(out_p), np.concatenate(out_d), u, i


def sample(poly, step):
    pts, dirs = [], []
    for a, b in zip(poly[:-1], poly[1:]):
        L = float(np.linalg.norm(b - a))
        n = max(1, int(L / step))
        for k in range(n):
            pts.append(a + (b - a) * k / n)
            dirs.append((b - a) / max(L, 1e-9))
    return np.array(pts), np.array(dirs)


class Fit:
    """Real (east, north metres) -> twin frame (ISO, 1/10): similarity, then
    a thin-plate spline anchored on road correspondences."""

    def __init__(self, ways, roads_fs, log=print):
        named = {w["tags"].get("name"): w for w in ways}
        net = json.loads(NETWORK.read_text(encoding="utf-8"))
        fs_b = {b["name"]: np.array([b["x"], b["y"]]) / TWIN for b in net["buildings"]}
        src = np.array([fs_b[k] for k in ANCHORS])
        dst = np.array([named[v]["pts"].mean(0) for v in ANCHORS.values()])
        s, Rm, t = umeyama(src, dst)                                # fleet sim -> real
        real_roads = [w["pts"] for w in ways if w["tags"].get("highway") in ROAD_TAGS]
        ra, rb = segments(real_roads)
        fs = np.vstack([sample(r, 5.0)[0] for r in roads_fs])
        for _ in range(60):
            near, dist, _, _ = nearest_on(ra, rb, (s * (Rm @ fs.T)).T + t)
            keep = dist < max(8.0, np.percentile(dist, 70))
            s, Rm, t = umeyama(fs[keep], near[keep])
        self.scale, self.R, self.t = s, Rm, t
        log(f"similarity: real = {s:.3f} x fleet sim, turned {math.degrees(math.atan2(Rm[1, 0], Rm[0, 0])):.1f} deg")

        # Thin-plate spline in the fleet sim frame: pre-warp (similarity) -> fleet sim road.
        ra_fs, rb_fs = self.to_fs(ra), self.to_fs(rb)
        centre = np.vstack(roads_fs).mean(0)
        extent = np.abs(np.vstack(roads_fs) - centre).max()
        ring = np.array([centre + 2.2 * extent * np.array([math.cos(a), math.sin(a)])
                         for a in np.linspace(0, 2 * math.pi, 20, endpoint=False)])
        self.P, self.W = ring, None
        for round_ in range(3):
            wa, wb = self._tps(ra_fs), self._tps(rb_fs)
            src, dst, total = [], [], 0
            for r in roads_fs:
                q, dirs = sample(r, 6.0)
                total += len(q)
                near, dist, u, i = nearest_on(wa, wb, q)
                ok = (dist < 30.0 / self.scale * (0.6 if round_ else 1.0)) & \
                     (np.abs((u * dirs).sum(1)) > math.cos(math.radians(30)))
                # the matched point before warping, at the same place along its segment
                tt = np.clip(((q - wa[i]) * (wb[i] - wa[i])).sum(1) /
                             np.maximum(((wb[i] - wa[i]) ** 2).sum(1), 1e-12), 0, 1)
                pre = ra_fs[i] + tt[:, None] * (rb_fs[i] - ra_fs[i])
                src += list(pre[ok]); dst += list(q[ok])
            src, dst = np.vstack([np.array(src), ring]), np.vstack([np.array(dst), ring])
            self._fit_tps(src, dst, lam=40.0)
            res = np.linalg.norm(self._tps(src[:-len(ring)]) - dst[:-len(ring)], axis=1)
            log(f"warp round {round_ + 1}: {len(src) - len(ring)} of {total} road points matched, "
                f"residual median {np.median(res):.1f} m, 95th {np.percentile(res, 95):.1f} m (fleet sim metres)")
        self.matched = len(src) - len(ring)

    def to_fs(self, p):
        return ((self.R.T @ (np.asarray(p) - self.t).T) / self.scale).T

    @staticmethod
    def _U(r2):
        with np.errstate(divide="ignore", invalid="ignore"):
            return np.nan_to_num(r2 * np.log(r2))

    def _fit_tps(self, P, Q, lam):
        n = len(P)
        A = np.zeros((n + 3, n + 3))
        A[:n, :n] = self._U(((P[:, None] - P[None]) ** 2).sum(-1)) + lam * np.eye(n)
        Pm = np.hstack([np.ones((n, 1)), P])
        A[:n, n:], A[n:, :n] = Pm, Pm.T
        b = np.zeros((n + 3, 2))
        b[:n] = Q
        self.P, self.W = P, np.linalg.solve(A, b)

    def _tps(self, X, chunk=4096):
        X = np.asarray(X, dtype=float)
        if self.W is None:
            return X.copy()
        out = []
        for k in range(0, len(X), chunk):
            x = X[k:k + chunk]
            K = self._U(((x[:, None] - self.P[None]) ** 2).sum(-1))
            out.append(K @ self.W[:len(self.P)] + np.hstack([np.ones((len(x), 1)), x]) @ self.W[len(self.P):])
        return np.vstack(out) if out else X.copy()

    def __call__(self, real_pts):
        """Real east/north metres -> twin frame (ISO x, y, 1/10)."""
        return self._tps(self.to_fs(real_pts)) * TWIN


# ---------------------------------------------------------- road corridor

class Corridor:
    """The fleet sim roads with their kerbs, in the twin frame."""

    def __init__(self, net):
        self.half = 0.5 * float(net["road"]["width"]) + float(net["road"]["sidewalk_width"])
        polys = [np.array([[q[0], q[1]] for q in e["polyline"]]) for e in net["edges"]]
        self.a, self.b = segments(polys)
        self.box_names = [b["name"] for b in net["buildings"]]
        self.boxes = [np.array([[b["x"] + sx * b["half_length"], b["y"] + sy * b["half_width"]]
                                for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1))]) for b in net["buildings"]]
        self.lo = np.vstack(polys).min(0)
        self.hi = np.vstack(polys).max(0)

    def distance(self, q):
        _, d, u, _ = nearest_on(self.a, self.b, np.atleast_2d(q))
        return d, u

    def nearest(self, q):
        p, d, u, _ = nearest_on(self.a, self.b, np.atleast_2d(q))
        return p, d


def point_in_poly(pt, poly):
    x, y = pt
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            inside = not inside
    return inside


def seg_dist(p1, p2, q1, q2):
    """Distance between segments p1p2 and q1q2 (0 when they cross)."""
    def orient(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    if (orient(p1, p2, q1) * orient(p1, p2, q2) < 0) and (orient(q1, q2, p1) * orient(q1, q2, p2) < 0):
        return 0.0

    def pd(p, a, b):
        ab = b - a
        t = np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-12), 0, 1)
        return float(np.linalg.norm(a + t * ab - p))
    return min(pd(p1, q1, q2), pd(p2, q1, q2), pd(q1, p1, p2), pd(q2, p1, p2))


def poly_clearance(poly, cor: Corridor):
    """Smallest distance from the polygon to a road centreline (0 if a road
    runs through it), and the road point it is nearest."""
    ring = np.vstack([poly, poly[:1]])
    best, at, on = math.inf, None, None
    # candidate road segments near the polygon
    lo, hi = poly.min(0) - 6, poly.max(0) + 6
    sel = np.where(((np.minimum(cor.a, cor.b) <= hi) & (np.maximum(cor.a, cor.b) >= lo)).all(1))[0]
    for j in sel:
        if point_in_poly(cor.a[j], poly):
            return 0.0, cor.a[j], cor.a[j]
        for p1, p2 in zip(ring[:-1], ring[1:]):
            d = seg_dist(p1, p2, cor.a[j], cor.b[j])
            if d < best:
                best = d
                pr, _ = nearest_on(cor.a[j:j + 1], cor.b[j:j + 1], np.array([(p1 + p2) / 2]))[:2]
                at, on = pr[0], (p1 + p2) / 2
    return best, at, on


def polys_overlap(p, q):
    if any(point_in_poly(v, q) for v in p) or any(point_in_poly(v, p) for v in q):
        return True
    rp, rq = np.vstack([p, p[:1]]), np.vstack([q, q[:1]])
    return any(seg_dist(a, b, c, d) == 0.0 for a, b in zip(rp[:-1], rp[1:]) for c, d in zip(rq[:-1], rq[1:]))


def area(poly):
    x, y = poly[:, 0], poly[:, 1]
    return 0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


# ----------------------------------------------------------------- scenery

def building_kind(tags):
    name = tags.get("name", "")
    if tags.get("building") in ("university", "school", "college") or "Academic" in name or "School" in name:
        return "academic"
    if tags.get("building") in ("residential", "apartments", "dormitory", "house") or "Bhavanam" in name or \
            tags.get("building") == "hostel":
        return "hostel"
    return "other"


def building_height(tags, kind):
    if tags.get("building:levels", "").isdigit():
        return int(tags["building:levels"]) * LEVEL_M
    if tags.get("height", "").replace(".", "", 1).isdigit():
        return float(tags["height"])
    return {"academic": 4 * LEVEL_M, "hostel": 4 * LEVEL_M}.get(kind, 2 * LEVEL_M)


def _clear_of_roads(poly, cor: Corridor, clear: float):
    """Shift (or None) that moves the polygon clear of every road, found by
    stepping away from the nearest road point."""
    shift = np.zeros(2)
    for _ in range(40):
        d, at, on = poly_clearance(poly + shift, cor)
        if d >= clear:
            return shift if np.linalg.norm(shift) <= MAX_PUSH else None
        c = (poly + shift).mean(0)
        v = c - at if d == 0.0 else on - at
        n = np.linalg.norm(v)
        v = v / n if n > 1e-9 else np.array([1.0, 0.0])
        shift = shift + v * (clear - d + 0.02)
    return None


SHRINK = (1.0, 0.85, 0.7, 0.55)


def place_buildings(ways, fit: Fit, cor: Corridor, log=print):
    """Each real building where the fit puts it; one on a road moves clear
    (up to MAX_PUSH), shrinking about its centre (down to SHRINK[-1]) when
    moving alone cannot clear it or would land it on another building."""
    out, moved, shrunk, dropped = [], 0, 0, []
    clear = cor.half + MARGIN
    # big buildings first: they are the landmarks, and the small ones fit round them
    cands = []
    for w in (w for w in ways if "building" in w["tags"] and w["closed"]):
        poly = fit(w["pts"][:-1])
        if abs(area(poly)) >= 0.02:
            cands.append((w, poly if area(poly) > 0 else poly[::-1]))
    cands.sort(key=lambda c: (-abs(area(c[1])), c[0]["id"]))
    for w, poly0 in cands:
        name = w["tags"].get("name", "") or str(w["id"])
        c0 = poly0.mean(0)
        placed = None
        for f in SHRINK:
            poly = c0 + (poly0 - c0) * f
            shift = _clear_of_roads(poly, cor, clear)
            if shift is None:
                continue
            poly = poly + shift
            if any(polys_overlap(poly, b["footprint"]) for b in out):
                continue
            placed = (poly, f, shift)
            break
        if placed is None:
            dropped.append(name)
            continue
        poly, f, shift = placed
        moved += np.linalg.norm(shift) > 1e-9
        shrunk += f < 1.0
        kind = building_kind(w["tags"])
        out.append({"name": w["tags"].get("name", ""), "osm_id": w["id"], "kind": kind,
                    "stands_for": [n for n, box in zip(cor.box_names, cor.boxes) if polys_overlap(poly, box)],
                    "scale": f, "height": round(building_height(w["tags"], kind) * TWIN, 3), "footprint": poly})
    out.sort(key=lambda b: b["osm_id"])
    log(f"buildings: {len(out)} placed ({moved} moved clear of a road, {shrunk} shrunk to fit), "
        f"{len(dropped)} left out: {', '.join(dropped)}")
    return out


LAND = {("landuse", "grass"): "grass", ("leisure", "garden"): "garden", ("leisure", "pitch"): "pitch",
        ("leisure", "playground"): "pitch", ("leisure", "sports_centre"): "pitch", ("natural", "wood"): "wood",
        ("landuse", "forest"): "wood", ("natural", "water"): "water", ("leisure", "swimming_pool"): "water",
        ("landuse", "farmland"): "farmland", ("landuse", "orchard"): "orchard", ("natural", "scrub"): "scrub",
        ("landuse", "meadow"): "grass", ("leisure", "park"): "garden"}
LAND_ORDER = ["farmland", "orchard", "wood", "scrub", "grass", "garden", "pitch", "water"]


def land_areas(ways, fit: Fit):
    out = []
    for w in ways:
        if not w["closed"]:
            continue
        kind = next((v for (k, val), v in LAND.items() if w["tags"].get(k) == val), None)
        if kind and len(w["pts"]) >= 4:
            out.append({"kind": kind, "name": w["tags"].get("name", ""), "polygon": fit(w["pts"][:-1])})
    out.sort(key=lambda a: LAND_ORDER.index(a["kind"]))
    return out


def side_roads(ways, fit: Fit, cor: Corridor, log=print):
    """Real roads and paths the fleet sim does not drive, clipped clear of its
    roads; a real road running alongside a fleet sim road (its duplicate)
    goes."""
    out = []
    for w in ways:
        hw = w["tags"].get("highway")
        if hw not in ROAD_TAGS and hw not in PATH_TAGS:
            continue
        kind = "road" if hw in ROAD_TAGS else "path"
        width = SIDE_ROAD_WIDTH if kind == "road" else PATH_WIDTH
        dense = []
        for a, b in zip(w["pts"][:-1], w["pts"][1:]):
            n = max(1, int(np.linalg.norm(b - a) / 2.0))
            dense += [a + (b - a) * k / n for k in range(n)]
        dense.append(w["pts"][-1])
        p = fit(np.array(dense))
        d, u = cor.distance(p)
        tang = np.gradient(p, axis=0)
        tang /= np.maximum(np.linalg.norm(tang, axis=1, keepdims=True), 1e-9)
        duplicate = (kind == "road") & (d < 3.0) & (np.abs((tang * u).sum(1)) > 0.9)
        keep = (d > cor.half + 0.5 * width + MARGIN) & ~duplicate
        runs, run = [], []
        for q, k in zip(p, keep):
            if k:
                run.append(q)
            elif run:
                runs.append(run)
                run = []
        if run:
            runs.append(run)
        for r in runs:
            pl = np.array(r)
            if len(pl) >= 2 and float(np.linalg.norm(np.diff(pl, axis=0), axis=1).sum()) >= 0.6:
                out.append({"kind": kind, "width": width, "points": pl})
    log(f"side roads and paths: {sum(r['kind'] == 'road' for r in out)} road and "
        f"{sum(r['kind'] == 'path' for r in out)} path pieces")
    return out


# ---------------------------------------------------------------- imagery

GRASS, SOIL, TREES, OTHER = 1, 2, 3, 4


def classify(rgb):
    """Ground cover of satellite pixels (uint8 n x 3): 0 unknown (map
    overlay, labels), GRASS, SOIL, TREES, OTHER (roofs, paving)."""
    r, g, b = (rgb[:, k].astype(int) for k in range(3))
    v = np.maximum(np.maximum(r, g), b)
    sat = v - np.minimum(np.minimum(r, g), b)
    cls = np.full(len(rgb), OTHER, dtype=np.uint8)
    green = (g >= r + 6) & (g >= b - 4)
    cls[green & (v < 105)] = TREES
    cls[green & (v >= 105)] = GRASS
    cls[(r >= g + 4) & (r >= b + 12) & (v >= 70)] = SOIL
    overlay = (sat <= 10) & (v >= 110) & (v <= 175)
    label = (v > 215) & (sat < 40) | (v < 25)
    cls[overlay | label] = 0
    return cls


def georeference(img, ties, ways, log=print):
    """Pixel -> real (east, north) for a screenshot: a similarity (image y
    runs down, hence the mirror) from tie points on named map features,
    refined by fitting the screenshot's grey road overlay to the real roads."""
    named = {w["tags"].get("name"): w["pts"].mean(0) for w in ways if w["tags"].get("name")}
    src = np.array([[t[0], -t[1]] for t in ties])
    dst = np.array([named[t[2]] for t in ties])
    s, Rm, t = umeyama(src, dst)
    a = np.asarray(img.convert("RGB"))
    H, W = a.shape[:2]
    ys, xs = np.mgrid[0:H:2, 0:W:2]
    px = a[ys, xs].reshape(-1, 3).astype(int)
    v = px.max(1)
    sat = v - px.min(1)
    road = (sat <= 10) & (v >= 115) & (v <= 165)
    pts = np.stack([xs.ravel()[road], -ys.ravel()[road]], 1).astype(float)
    if len(pts) > 6000:
        pts = pts[np.random.default_rng(0).choice(len(pts), 6000, replace=False)]
    ra, rb = segments([w["pts"] for w in ways if w["tags"].get("highway") in ROAD_TAGS + PATH_TAGS])
    for _ in range(40):
        near, dist, _, _ = nearest_on(ra, rb, (s * (Rm @ pts.T)).T + t)
        keep = dist < max(4.0, np.percentile(dist, 60))
        s, Rm, t = umeyama(pts[keep], near[keep])
    _, dist, _, _ = nearest_on(ra, rb, (s * (Rm @ pts.T)).T + t)
    log(f"    {s:.3f} m/px, road overlay to real roads: median {np.median(dist):.1f} m")
    return lambda p: (s * (Rm @ np.stack([p[:, 0], -p[:, 1]], 1).T)).T + t, s


def vegetation(images_json: pathlib.Path, ways, fit: Fit, cor: Corridor, buildings, grid, log=print):
    """Ground cover per grid cell and trees, from the screenshots. Finer
    screenshots outvote coarser ones."""
    from PIL import Image
    cfg = json.loads(images_json.read_text(encoding="utf-8"))
    (x0, y0), cell, (nx, ny) = grid
    votes = np.zeros((ny, nx, 5))
    for item in cfg["images"]:
        path = images_json.parent / item["file"]
        img = Image.open(path)
        log(f"  {item['file']}:")
        to_real, mpp = georeference(img, item["ties"], ways, log)
        a = np.asarray(img.convert("RGB"))
        H, W = a.shape[:2]
        step = 2
        ys, xs = np.mgrid[0:H:step, 0:W:step]
        cls = classify(a[ys, xs].reshape(-1, 3))
        ok = cls > 0
        twin = fit(to_real(np.stack([xs.ravel()[ok], ys.ravel()[ok]], 1).astype(float)))
        ix = ((twin[:, 0] - x0) / cell).astype(int)
        iy = ((twin[:, 1] - y0) / cell).astype(int)
        inside = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
        np.add.at(votes, (iy[inside], ix[inside], cls[ok][inside]), 1.0 / mpp)
    cover = votes[..., 1:].argmax(-1).astype(np.uint8) + 1
    cover[votes[..., 1:].sum(-1) == 0] = 0
    tree_frac = votes[..., TREES] / np.maximum(votes[..., 1:].sum(-1), 1e-9)

    rng = random.Random(7)
    trees = []
    clear = cor.half + MARGIN
    for iy in range(ny):
        for ix in range(nx):
            f = tree_frac[iy, ix]
            if f < 0.25:
                continue
            n = int(f * cell * cell / 0.18 + rng.random())      # one crown per ~0.18 m2 of cover
            for _ in range(n):
                x = x0 + (ix + rng.random()) * cell
                y = y0 + (iy + rng.random()) * cell
                r = rng.uniform(0.14, 0.32)
                trees.append((x, y, r))
    if trees:
        t = np.array(trees)
        d, _ = cor.distance(t[:, :2])
        keep = d > clear + t[:, 2]
        t = t[keep]
        for b in buildings:
            fp = b["footprint"]
            lo, hi = fp.min(0) - 0.3, fp.max(0) + 0.3
            near = np.where((t[:, 0] > lo[0]) & (t[:, 0] < hi[0]) & (t[:, 1] > lo[1]) & (t[:, 1] < hi[1]))[0]
            drop = [i for i in near if point_in_poly(t[i, :2], fp)]
            t = np.delete(t, drop, axis=0)
        covered = {n for b in buildings for n in b["stands_for"]}
        for name, box in zip(cor.box_names, cor.boxes):
            if name not in covered:
                drop = [i for i in range(len(t)) if point_in_poly(t[i, :2], box)]
                t = np.delete(t, drop, axis=0)
        trees = [[round(x, 3), round(y, 3), round(r, 3), round(r * rng.uniform(2.2, 3.4), 3)] for x, y, r in t]
    log(f"vegetation: {len(trees)} trees; cover "
        + ", ".join(f"{n} {100 * (cover == k).mean():.0f} %" for k, n in ((GRASS, "grass"), (SOIL, "bare earth"),
                                                                           (TREES, "trees"), (OTHER, "built"))))
    return cover, trees


# ----------------------------------------------------------- ground image

COLOURS = {"base": (104, 126, 80), GRASS: (112, 138, 82), SOIL: (150, 124, 94), TREES: (78, 100, 64),
           OTHER: (128, 124, 112), "farmland": (140, 132, 92), "orchard": (92, 116, 70), "wood": (74, 96, 60),
           "scrub": (118, 116, 84), "grass": (118, 146, 86), "garden": (110, 142, 84), "pitch": (164, 132, 100),
           "water": (70, 140, 170), "road": (88, 88, 92), "path": (176, 160, 132)}


def ground_image(grid, cover, land, roads, px_per_m, path_png: pathlib.Path):
    from PIL import Image, ImageDraw, ImageFilter
    (x0, y0), cell, (nx, ny) = grid
    ext_x, ext_y = nx * cell, ny * cell
    # Seen from above with ISO x up the page: image right is -ISO y, image
    # down is -ISO x. The Unity quad maps it the same way.
    W, H = int(ext_y * px_per_m), int(ext_x * px_per_m)
    xmax, ymax = x0 + ext_x, y0 + ext_y

    def to_px(p):
        return [((ymax - q[1]) * px_per_m, (xmax - q[0]) * px_per_m) for q in p]
    if cover is not None:
        small = Image.new("RGB", (ny, nx))
        pix = small.load()
        for iy in range(ny):
            for ix in range(nx):
                c = COLOURS.get(int(cover[iy, ix]), COLOURS["base"])
                pix[ny - 1 - iy, nx - 1 - ix] = c
        im = small.resize((W, H), Image.BILINEAR).filter(ImageFilter.GaussianBlur(px_per_m * cell * 0.35))
    else:
        im = Image.new("RGB", (W, H), COLOURS["base"])
    dr = ImageDraw.Draw(im)
    for a in land:
        dr.polygon(to_px(a["polygon"]), fill=COLOURS[a["kind"]])
    for r in roads:
        dr.line(to_px(r["points"]), fill=COLOURS[r["kind"]], width=max(1, int(r["width"] * px_per_m)), joint="curve")
    # a little grain so large areas do not look flat
    rng = np.random.default_rng(3)
    arr = np.asarray(im).astype(int) + rng.integers(-6, 7, size=(H, W, 1))
    im = Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8))
    path_png.parent.mkdir(parents=True, exist_ok=True)
    im.save(path_png)
    meta = {"x_min": x0, "x_max": xmax, "y_min": y0, "y_max": ymax, "pixels": [W, H],
            "mapping": "u = (y_max - y) / (y_max - y_min), v = (x_max - x) / (x_max - x_min), image top-left origin"}
    path_png.with_suffix(".json").write_text(json.dumps(meta, indent=1) + "\n", encoding="utf-8")
    return meta


# -------------------------------------------------------------------- main

def build(images: pathlib.Path | None, px_per_m: float, log=print) -> dict:
    ways = load_osm()
    net, roads_fs = network()
    fit = Fit(ways, roads_fs, log)
    cor = Corridor(net)
    buildings = place_buildings(ways, fit, cor, log)
    land = land_areas(ways, fit)
    roads = side_roads(ways, fit, cor, log)
    margin = 10.0
    lo, hi = cor.lo - margin, cor.hi + margin
    cell = 0.25
    grid = ((float(lo[0]), float(lo[1])), cell, (int((hi[0] - lo[0]) / cell) + 1, int((hi[1] - lo[1]) / cell) + 1))
    cover, trees = (None, [])
    if images:
        cover, trees = vegetation(images, ways, fit, cor, buildings, grid, log)
        veg = {"format": "rc-physics-twin campus vegetation 0.1.0", "source": str(images.name),
               "note": "from map screenshots: local only, not for redistribution",
               "trees": trees, "trees_fields": ["x", "y", "crown_radius", "height"]}
        (images.parent / "vegetation.json").write_text(json.dumps(veg, separators=(",", ":")) + "\n", encoding="utf-8")
        log(f"wrote {images.parent / 'vegetation.json'}")
    meta = ground_image(grid, cover, land, roads, px_per_m, LOCAL / "Ground.png")
    log(f"wrote {LOCAL / 'Ground.png'} ({meta['pixels'][0]} x {meta['pixels'][1]})")
    r3 = lambda a: [[round(float(x), 3), round(float(y), 3)] for x, y in a]
    doc = {"format": FORMAT, "attribution": ATTRIBUTION, "frame": "twin ISO, 1/10 (campus/campus_network.json)",
           "osm_extract": str(OSM.relative_to(ROOT)).replace("\\", "/"),
           "fit": {"real_per_fleet_sim_m": round(fit.scale, 4),
                   "rotation_deg": round(math.degrees(math.atan2(fit.R[1, 0], fit.R[0, 0])), 2),
                   "lat0": LAT0, "lon0": LON0, "road_points_matched": int(fit.matched)},
           "clearance_m": round(cor.half + MARGIN, 4),
           "buildings": [{**{k: v for k, v in b.items() if k != "footprint"}, "footprint": r3(b["footprint"])}
                         for b in buildings],
           "land": [{"kind": a["kind"], "name": a["name"], "polygon": r3(a["polygon"])} for a in land],
           "side_roads": [{"kind": r["kind"], "width": r["width"], "points": r3(r["points"])} for r in roads]}
    SCENERY.write_text(json.dumps(doc, ensure_ascii=False, separators=(",", ":")) + "\n", encoding="utf-8")
    log(f"wrote {SCENERY}")
    return doc


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("fetch", help="download the OpenStreetMap extract")
    b = sub.add_parser("build", help="fit, place and write the scenery")
    b.add_argument("--images", type=pathlib.Path, help="images.json: screenshots with tie points (local)")
    b.add_argument("--px-per-m", type=float, default=24.0, help="ground texture resolution [px per twin m]")
    args = ap.parse_args(argv)
    if args.cmd == "fetch":
        fetch()
    else:
        build(args.images, args.px_per_m)
    return 0


if __name__ == "__main__":
    sys.exit(main())
