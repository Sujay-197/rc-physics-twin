#!/usr/bin/env python3
"""Edge profiler: physics-grounded costs per road edge for the fleet sim.

For each road edge and each (target speed, payload, battery charge) the car
is driven along the edge headlessly (tools/drive_road.py, the reference
driver below) and the run's traversal time, energy, peak accelerations and
speed-breaker penalty are cached as edge costs (spec section 7, ADR 0004).
The fleet sim reads the cache and never runs the physics.

The speed-breaker penalty is the time the same run loses to the edge's
breakers: the run against the same edge with its breakers removed.

The cache (profiles/edge_profile.schema.json) is keyed by
(edge_id, vehicle_class, payload_kg, soc_bucket, speed_target_mps). Each
entry carries the digests it was computed from: the road file's content, the
car's full parameter set, and the profiler and controller version. On a
rerun, entries whose digests still match are kept and only the rest are
computed; entries for roads, cars or controllers that changed are dropped.

Usage:
    python tools/edge_profiler.py roads/samples/*.json --out profiles/cache.json \\
        --speeds 1 2 3 --payloads 0 0.5 1 --soc 0.9 0.6 0.3 --breaker-speed 1.0

A charge bucket of exactly 1.0 is legal but overstates energy wherever the
car brakes: a full pack cannot take regenerated charge (the Core clamps
state of charge at 1), so the profiler warns about it.

Parameters are SlipX's provisional defaults unless set; nothing here is a
measured car, and the vehicle class is recorded with its provenance.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import importlib.util
import json
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
_spec = importlib.util.spec_from_file_location("drive_road", HERE / "drive_road.py")
dr = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(dr)

SCHEMA_VERSION = "0.1.0"
PROFILER_VERSION = "0.1.0"


def road_digest(road: dict) -> str:
    """Content digest of a road, independent of formatting and key order."""
    canon = json.dumps(road, sort_keys=True, separators=(",", ":"), ensure_ascii=True)
    return hashlib.sha256(canon.encode()).hexdigest()


def params_digest(p: "dr.Params") -> str:
    """Digest of the car's complete parameter set, as the Core receives it."""
    return hashlib.sha256(bytes(p)).hexdigest()


def controller_digest(args: argparse.Namespace) -> str:
    spec = {"profiler": PROFILER_VERSION, "controller": dr.CONTROLLER,
            "breaker_speed": args.breaker_speed, "comfort_decel": args.comfort_decel,
            "timeout": args.timeout}
    return hashlib.sha256(json.dumps(spec, sort_keys=True).encode()).hexdigest()


def run(lib, road: dict, base: argparse.Namespace, speed: float, payload: float,
        soc: float) -> dict:
    a = copy.copy(base)
    a.speed, a.start_speed, a.payload, a.soc = speed, None, payload, soc
    _, rep = dr.drive(lib, road, a)
    return rep


def profile_entry(lib, road: dict, base: argparse.Namespace, speed: float,
                  payload: float, soc: float) -> dict:
    rep = run(lib, road, base, speed, payload, soc)
    ok = rep["status"] == "reached_end"
    penalty = 0.0
    if ok and road.get("speed_breakers"):
        bare = {k: v for k, v in road.items() if k != "speed_breakers"}
        rep_bare = run(lib, bare, base, speed, payload, soc)
        if rep_bare["status"] == "reached_end":
            penalty = round(rep["time_s"] - rep_bare["time_s"], 6)
        else:
            penalty = None
    return {
        "status": rep["status"],
        "time_s": rep["time_s"] if ok else None,
        "energy_wh": rep["energy_wh"] if ok else None,
        "mean_speed_mps": rep["mean_speed_mps"] if ok else None,
        "peak_ax_mps2": rep["peak_ax_mps2"],
        "peak_ay_mps2": rep["peak_ay_mps2"],
        "breaker_penalty_s": penalty if ok else None,
    }


def key(e: dict) -> tuple:
    return (e["edge_id"], e["vehicle_class"], e["payload_kg"], e["soc_bucket"],
            e["speed_target_mps"])


def profile(lib, roads: list[dict], args: argparse.Namespace, cache: dict | None) -> tuple[dict, dict]:
    """Returns (cache, counts). Reuses entries whose digests match."""
    p = dr.car_params(lib, argparse.Namespace(**{**vars(args), "payload": 0.0}))
    pdig = params_digest(p)
    cdig = controller_digest(args)
    old = {}
    if cache is not None and cache.get("schema_version") == SCHEMA_VERSION:
        for e in cache.get("entries", []):
            old[key(e)] = e
    entries, counts = [], {"reused": 0, "computed": 0, "dropped": 0}
    wanted = set()
    for road in roads:
        rdig = road_digest(road)
        for speed in args.speeds:
            for payload in args.payloads:
                for soc in args.soc:
                    k = (road["id"], args.vehicle_class, payload, soc, speed)
                    wanted.add(k)
                    prev = old.get(k)
                    if prev and prev["road_digest"] == rdig and prev["params_digest"] == pdig \
                            and prev["controller_digest"] == cdig:
                        entries.append(prev)
                        counts["reused"] += 1
                        continue
                    e = {"edge_id": road["id"], "vehicle_class": args.vehicle_class,
                         "payload_kg": payload, "soc_bucket": soc, "speed_target_mps": speed,
                         "road_digest": rdig, "params_digest": pdig, "controller_digest": cdig}
                    e.update(profile_entry(lib, road, args, speed, payload, soc))
                    entries.append(e)
                    counts["computed"] += 1
    counts["dropped"] = sum(1 for k, e in old.items() if k not in wanted or e not in entries)
    entries.sort(key=key)
    out = {
        "schema_version": SCHEMA_VERSION,
        "profiler_version": PROFILER_VERSION,
        "controller": dr.CONTROLLER | {"breaker_speed_mps": args.breaker_speed,
                                       "comfort_decel_mps2": args.comfort_decel},
        "vehicle_classes": {
            args.vehicle_class: {
                "params_digest": pdig,
                "provenance": "provisional",
                "note": "SlipX provisional defaults plus the options below; not a measured car",
                "options": {"wheel_inertia": args.wheel_inertia, "ackermann": args.ackermann,
                            "wheel_rate": args.wheel_rate, "damping": args.damping,
                            "suspension_travel": args.travel,
                            # ADR 0011: only when on, so earlier caches keep their options
                            **({"v_kinematic": args.v_kinematic}
                               if getattr(args, "v_kinematic", 0.0) else {})},
            }
        },
        "entries": entries,
    }
    return out, counts


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("roads", type=pathlib.Path, nargs="+", help="road JSON files")
    ap.add_argument("--out", type=pathlib.Path, required=True, help="cache JSON, updated in place")
    ap.add_argument("--speeds", type=float, nargs="+", default=[2.0], help="target speeds [m/s]")
    ap.add_argument("--payloads", type=float, nargs="+", default=[0.0], help="payloads [kg]")
    ap.add_argument("--soc", type=float, nargs="+", default=[0.8], help="charge buckets, 0..1")
    ap.add_argument("--vehicle-class", default="reference_1_10_provisional")
    dr.add_car_options(ap)
    args = ap.parse_args(argv)
    if any(s >= 1.0 for s in args.soc):
        print("edge_profiler: warning: a charge bucket of 1.0 cannot take regenerated charge; "
              "energy is overstated wherever the car brakes", file=sys.stderr)
    try:
        roads = []
        for path in args.roads:
            road = json.loads(path.read_text(encoding="utf-8"))
            dr.check_road(road)
            roads.append(road)
        ids = [r["id"] for r in roads]
        if len(set(ids)) != len(ids):
            raise dr.RoadError("two road files share an id")
        cache = json.loads(args.out.read_text(encoding="utf-8")) if args.out.exists() else None
        lib = dr.load_lib(args.lib)
        out, counts = profile(lib, roads, args, cache)
    except (dr.RoadError, OSError, json.JSONDecodeError) as e:
        print(f"edge_profiler: {e}", file=sys.stderr)
        return 1
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(out, indent=2) + "\n", encoding="utf-8")
    print(f"edge_profiler: {len(out['entries'])} entries "
          f"({counts['computed']} computed, {counts['reused']} reused, {counts['dropped']} dropped) "
          f"-> {args.out}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
