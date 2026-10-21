#!/usr/bin/env python3
"""Record policies driving the campus, for the Unity replay (RcTwin > Build
Replay Scene, PolicyReplay.cs).

Two sets of trips:
  default   the evaluation's own: trip k is seed EVAL_SEED + k, its stops
            drawn by demand weight, as sim_py/evaluate_policies.py drives them
  --pairs   every origin and destination stop on different nodes, one car
            per pair (seed PAIR_SEED + k), so any route can be picked

In each condition every policy drives every trip with the same car, and each
drive is sampled at 25 Hz: position, heading, speed, achieved steer,
throttle command and offset, plus where the car touched a wall (a new mark
only HIT_GAP_S or HIT_GAP_M from the last, so a car held against a kerb
leaves one) and where the mid-trip change struck. Each trip's stops, car and route are stored once,
beside the drives, with the time the replay shows it until: a few seconds
after the last drive that finished or left the road. A drive that timed out
(usually a car pinned against a kerb until 120 s) keeps its samples up to
then and its final one; a car sitting still at the end keeps only the first
and last of its still samples.

    .venv/Scripts/python tools/record_drives.py --pairs --out results/drives.json

Policies default to the tuned controllers plus every model under
policies/study/; name others as evaluate_policies.py specs, with an optional
label: "sb3:path/to/ppo-s0=PPO (early)".
"""

from __future__ import annotations

import argparse
import ctypes
import json
import math
import multiprocessing as mp
import os
import pathlib
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim_py"))
sys.path.insert(0, str(ROOT / "tools"))
import evaluate_policies as ev  # noqa: E402

FORMAT = "rc-physics-twin drives 0.2.0"
EVERY = 4            # ticks per sample: 100 Hz control, 25 Hz recording
PAIR_SEED = 50_000   # cars for --pairs; tuning, evaluation and validation use 30k, 20k and 40k
STILL_M, STILL_RAD = 0.005, 0.01
HIT_GAP_S, HIT_GAP_M = 0.25, 0.05
AFTER_S = 3.0        # replay runs on this long after the last drive that ended on its own
ALL_TIMED_OUT_S = 30.0


def default_specs() -> list[tuple[str, str]]:
    import study_report
    out = []
    for method, specs in study_report.discover().items():
        for spec in specs:
            seed = "" if spec.startswith("controller:") else " s" + spec.rsplit("-s", 1)[-1].removesuffix(".json")
            out.append((spec, study_report.LABEL.get(method, method) + seed))
    return out


def parse_spec(text: str) -> tuple[str, str]:
    spec, _, label = text.partition("=")
    return spec, label or spec


def stop_pairs() -> list[tuple[str, str]]:
    """Every ordered pair of stops on different nodes, in name order."""
    net = ev.campus().CampusNetwork.load()
    names = sorted(net.stops, key=lambda n: (n.rstrip("0123456789"), int(n[len(n.rstrip("0123456789")):] or 0)))
    return [(a, b) for a in names for b in names if net.stops[a]["node"] != net.stops[b]["node"]]


def trip_list(pairs: bool, trips: int) -> list[tuple[int, tuple | None]]:
    if pairs:
        return [(PAIR_SEED + k, p) for k, p in enumerate(stop_pairs())]
    return [(ev.EVAL_SEED + k, None) for k in range(trips)]


def trim_still(frames: list) -> list:
    """Drop samples where the car sits still at the end, keeping the first
    and last of them: the replay holds the pose between the two."""
    def same(a, b):
        return abs(a[0] - b[0]) <= STILL_M and abs(a[1] - b[1]) <= STILL_M and abs(a[2] - b[2]) <= STILL_RAD
    end = len(frames) - 1
    first = end
    while first > 0 and same(frames[first - 1], frames[end]):
        first -= 1
    return frames[: first + 1] + ([frames[end]] if first < end else [])


def shown_until(drives: list) -> float:
    """How long the replay shows a trip: AFTER_S past the last drive that
    finished or left the road, or ALL_TIMED_OUT_S when every drive timed out."""
    ends = [d["time"] for d in drives if d["status"] != "timeout"]
    return round(max(ends) + AFTER_S, 2) if ends else ALL_TIMED_OUT_S


def cut_after(frames: list, until: float, hz: int) -> list:
    """Samples up to time `until` (sample i is at i / hz), then the last."""
    n = int(until * hz) + 1
    return frames if len(frames) <= n + 1 else frames[:n] + [frames[-1]]


def drive(env, policy, seed: int, options) -> dict:
    """One trip, sampled every EVERY ticks."""
    obs, info = env.reset(seed=seed, options=options)
    if hasattr(policy, "reset"):
        policy.reset()
    p = env.params
    frames, hits, perturbed_at = [], [], None
    ret, ticks, contacts, off = 0.0, 0, 0, 0.0

    def sample(st, thr):
        frames.append([round(st.x, 3), round(st.y, 3), round(st.yaw, 3), round(st.vx, 2),
                       round(st.steer / p.steer_max, 2), round(thr, 2), round(env.d, 3)])

    sample(env.state, 0.0)
    done = False
    while not done:
        a = policy(obs)
        obs, r, te, tr, info = env.step(a)
        ret += r
        ticks += 1
        off += abs(info["offset"])
        st = env.state
        if info["wall_contacts"] > contacts:
            contacts = info["wall_contacts"]
            if not hits or env.t - hits[-1][0] >= HIT_GAP_S or                     math.hypot(st.x - hits[-1][1], st.y - hits[-1][2]) >= HIT_GAP_M:
                hits.append([round(env.t, 2), round(st.x, 3), round(st.y, 3)])
        if perturbed_at is None and getattr(env, "_perturbed", False):
            perturbed_at = [round(env.t, 2), round(st.x, 3), round(st.y, 3)]
        done = te or tr
        if done or ticks % EVERY == 0:
            sample(st, max(-1.0, min(1.0, float(a[1]))))
    return {"status": info["status"], "time": round(env.t, 2), "return": round(ret, 1),
            "offset_mean": round(off / max(1, ticks), 4), "wall_contacts": info["wall_contacts"],
            "energy_wh": round(info["energy_wh"], 4), "s": round(env.s, 2),
            "hits": hits, "perturbed": perturbed_at, "frames": trim_still(frames)}


def _job(args):
    spec, condition, items, lib, target_speed = args
    c = ev.campus()
    extra = {}
    if spec.startswith("sb3:"):
        cfg = json.loads(pathlib.Path(spec[4:]).with_suffix(".json").read_text(encoding="utf-8"))["config"]
        extra["history"] = cfg["history"]
    env = c.CampusEnv(seed=items[0][1], ranges=ev.ranges_for(condition), lib_path=lib,
                      target_speed=target_speed, **extra)
    policy = ev.build_policy(spec, env)
    out = []
    for k, seed, pair in items:
        options = dict(ev.options_for(condition) or {})
        if pair:
            options["trip"] = pair
        d = drive(env, policy, seed, options)
        trip = {"from": env.trip[0], "to": env.trip[1], "seed": seed,
                "car": {key: round(v, 3) for key, v in env.sample.items()},
                "route": [[round(q["x"], 3), round(q["y"], 3)] for q in env.road["centreline"]]}
        out.append((k, trip, d))
    env.close()
    return spec, condition, out


def campus_car(lib: str) -> dict:
    c = ev.campus()
    env = c.CampusEnv(seed=0, lib_path=lib)
    env.reset(seed=0)
    contact = c.envmod.dr.Contact()
    env.lib.rc_twin_get_contact(env._car, ctypes.byref(contact))
    p = env.params
    car = {"half_length": contact.half_length, "half_width": contact.half_width,
           "centre_offset": contact.centre_offset, "steer_max": p.steer_max, "lf": p.lf, "lr": p.lr,
           "track_front": p.track_front, "track_rear": p.track_rear, "h_cog": p.h_cog,
           "wheel_radius": p.wheel_radius}
    width = env.network.width
    env.close()
    return {"road_width": width, "car": car}


def record(policies, trips: list, conditions, lib: str, workers: int, target_speed: float) -> dict:
    items = [(k, seed, pair) for k, (seed, pair) in enumerate(trips)]
    # About three jobs per worker, each one policy in one condition on a run of trips.
    pieces = max(1, -(-3 * workers // max(1, len(policies) * len(conditions))))
    chunk = max(1, -(-len(items) // pieces))
    jobs = [(spec, cond, items[i:i + chunk], lib, target_speed)
            for spec, _ in policies for cond in conditions for i in range(0, len(items), chunk)]
    table = {cond: [None] * len(items) for cond in conditions}
    runs = {spec: {cond: [None] * len(items) for cond in conditions} for spec, _ in policies}
    with mp.get_context("spawn").Pool(workers) as pool:
        for spec, cond, out in pool.imap_unordered(_job, jobs):
            for k, trip, d in out:
                if table[cond][k] is None:
                    table[cond][k] = trip
                elif table[cond][k] != trip:
                    # Every policy must meet the same trip and the same car.
                    raise RuntimeError(f"{spec} drove a different trip or car on {cond} trip {k}")
                runs[spec][cond][k] = d
    hz = 100 // EVERY
    for cond in conditions:
        for k, trip in enumerate(table[cond]):
            drives = [runs[spec][cond][k] for spec, _ in policies]
            trip["until"] = shown_until(drives)
            for d in drives:
                if d["status"] == "timeout":
                    d["frames"] = cut_after(d["frames"], trip["until"], hz)
                    d["hits"] = [h for h in d["hits"] if h[0] <= trip["until"]]
    return {"format": FORMAT, "created": time.strftime("%Y-%m-%d %H:%M"),
            "target_speed": target_speed, "sample_hz": 100 // EVERY,
            "trip_set": "pairs" if trips and trips[0][1] else "evaluation", "after_s": AFTER_S,
            "conditions": list(conditions), "perturbation": ev.PERTURBATION,
            "policies": [{"spec": s, "label": lbl} for s, lbl in policies],
            "campus": campus_car(lib), "trips": table, "runs": runs}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("policies", nargs="*", help="evaluate_policies.py specs, optionally spec=label")
    ap.add_argument("--pairs", action="store_true", help="every stop-to-stop route instead of evaluation draws")
    ap.add_argument("--trips", type=int, default=20, help="evaluation draws (without --pairs)")
    ap.add_argument("--conditions", nargs="+", default=list(ev.CONDITIONS), choices=ev.CONDITIONS)
    ap.add_argument("--target-speed", type=float, default=4.0)
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) // 4))
    ap.add_argument("--out", type=pathlib.Path, default=ROOT / "results" / "drives.json")
    ap.add_argument("--lib")
    args = ap.parse_args(argv)
    lib = os.path.abspath(args.lib) if args.lib else str(ROOT / "build" / "bin" / "rc_twin.dll")
    policies = [parse_spec(p) for p in args.policies] or default_specs()
    t0 = time.perf_counter()
    data = record(policies, trip_list(args.pairs, args.trips), args.conditions, lib, args.workers,
                  args.target_speed)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(data, separators=(",", ":")) + "\n", encoding="utf-8")
    for spec, label in policies:
        line = []
        for cond in args.conditions:
            rs = data["runs"][spec][cond]
            line.append(f"{cond} {sum(r['status'] == 'reached_end' for r in rs)}/{len(rs)}")
        print(f"{label:40s} " + "  ".join(line))
    print(f"recorded {len(policies)} policies in {time.perf_counter() - t0:.0f} s; wrote {args.out} "
          f"({args.out.stat().st_size / 1e6:.1f} MB)", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
