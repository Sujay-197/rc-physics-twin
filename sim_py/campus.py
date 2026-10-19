"""The campus fleet sim's road network, for the twin (campus/README.md).

    net = CampusNetwork.load()                     # campus/campus_network.json
    road = net.route_road("S3", "AB4")             # a roads/road.schema.json dict
    env = CampusEnv(seed=0)                        # RL on stop-to-stop routes

Routes are Dijkstra shortest paths by edge length over the fleet sim's own
graph, as its GraphRouter finds them, from one stop's node to another's.
CampusEnv draws each episode's origin and destination by the stops' demand
weights, as the fleet sim's DemandAgent weights them, and drives the route on
the RcTwinEnv task (sim_py/rc_twin_env.py): same actions, observations,
randomisation and reward.
"""

from __future__ import annotations

import heapq
import importlib.util
import json
import math
import pathlib
import random

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
_spec = importlib.util.spec_from_file_location("rc_twin_env", HERE / "rc_twin_env.py")
envmod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(envmod)

NETWORK = ROOT / "campus" / "campus_network.json"


class CampusNetwork:
    def __init__(self, data: dict):
        self.data = data
        self.width = float(data["road"]["width"])
        self.nodes = {n["id"]: n for n in data["nodes"]}
        self.edges = {e["id"]: e for e in data["edges"]}
        self.stops = {s["name"]: s for s in data["stops"]}
        self.adj: dict[int, list[tuple[int, int, float]]] = {n: [] for n in self.nodes}
        for e in data["edges"]:
            self.adj[e["a"]].append((e["b"], e["id"], e["length"]))
            self.adj[e["b"]].append((e["a"], e["id"], e["length"]))

    @classmethod
    def load(cls, path=NETWORK) -> "CampusNetwork":
        return cls(json.loads(pathlib.Path(path).read_text(encoding="utf-8")))

    def shortest_path(self, a: int, b: int) -> list[tuple[int, bool]]:
        """Edges from node a to node b as (edge id, reversed?) in travel order.
        Ties break on node then edge id, so the route is deterministic."""
        dist = {a: 0.0}
        prev: dict[int, tuple[int, int]] = {}
        heap = [(0.0, a)]
        while heap:
            d, n = heapq.heappop(heap)
            if n == b:
                break
            if d > dist.get(n, math.inf):
                continue
            for m, eid, length in sorted(self.adj[n]):
                nd = d + length
                if nd < dist.get(m, math.inf):
                    dist[m] = nd
                    prev[m] = (n, eid)
                    heapq.heappush(heap, (nd, m))
        if b not in dist:
            raise ValueError(f"no route from node {a} to node {b}")
        path, n = [], b
        while n != a:
            p, eid = prev[n]
            path.append((eid, self.edges[eid]["a"] != p))  # reversed if travelling b -> a
            n = p
        return path[::-1]

    def route_points(self, a: int, b: int) -> list[list[float]]:
        points: list[list[float]] = []
        for eid, rev in self.shortest_path(a, b):
            poly = self.edges[eid]["polyline"]
            for p in (poly[::-1] if rev else poly):
                if points and math.hypot(p[0] - points[-1][0], p[1] - points[-1][1]) < 1e-6:
                    continue
                points.append(p)
        return points

    def route_road(self, origin: str, destination: str) -> dict:
        """The route from one stop to another as a road file dict."""
        a, b = self.stops[origin]["node"], self.stops[destination]["node"]
        if a == b:
            raise ValueError(f"{origin} and {destination} share node {a}")
        pts = self.route_points(a, b)
        return {
            "schema_version": "0.1.0",
            "id": f"campus-route-{origin}-{destination}",
            "description": f"Campus route {origin} to {destination}, nodes {a} to {b}, 1/10 scale.",
            "width": self.width,
            "centreline": [{"x": p[0], "y": p[1], "z": p[2]} for p in pts],
        }

    def sample_trip(self, rng: random.Random) -> tuple[str, str]:
        """Origin and destination by demand weight; never the same stop or node."""
        names = sorted(self.stops)
        weights = [self.stops[n]["weight"] for n in names]
        origin = rng.choices(names, weights)[0]
        rest = [n for n in names if self.stops[n]["node"] != self.stops[origin]["node"]]
        dest = rng.choices(rest, [self.stops[n]["weight"] for n in rest])[0]
        return origin, dest


class CampusEnv(envmod.RcTwinEnv):
    """RcTwinEnv on campus routes: each reset draws a trip and drives it.
    reset(options={"trip": ("S3", "AB4")}) fixes the trip instead."""

    def __init__(self, network: CampusNetwork | None = None, seed: int | None = None, **kwargs):
        self.network = network or CampusNetwork.load()
        self._trip_rng = random.Random(seed)
        first = self.network.sample_trip(random.Random(seed))
        kwargs.setdefault("max_time", 120.0)
        super().__init__(self.network.route_road(*first), **kwargs)

    def reset(self, seed: int | None = None, options: dict | None = None):
        options = dict(options or {})
        if seed is not None:
            self._trip_rng.seed(seed)
        trip = options.pop("trip", None) or self.network.sample_trip(self._trip_rng)
        self.trip = tuple(trip)
        options["road"] = self.network.route_road(*self.trip)
        obs, info = super().reset(seed=seed, options=options)
        info["trip"] = self.trip
        return obs, info

    def _info(self, status: str) -> dict:
        info = super()._info(status)
        info["trip"] = getattr(self, "trip", None)
        return info
