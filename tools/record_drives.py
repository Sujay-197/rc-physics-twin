#!/usr/bin/env python3
"""Record policies driving the campus, for the replay viewer.

Every policy drives the same trips with the same cars as
sim_py/evaluate_policies.py does (trip k is evaluation seed EVAL_SEED + k, in
each condition), and each drive is sampled at 25 Hz: position, heading,
speed, achieved steer, throttle command, offset, plus where the car touched a
wall and where the perturbation struck. The campus geometry goes in the same
file, so tools/replay_viewer.html can draw everything from it alone.

    .venv/Scripts/python tools/record_drives.py --trips 20 --out results/drives.json --html results/replay.html

Policies default to the tuned controllers plus every model under
policies/study/; name others as evaluate_policies.py specs, with an optional
label: "sb3:path/to/ppo-s0=PPO (early)". --html writes a standalone viewer
with the recording embedded (open it in any browser, no server needed).
"""

from __future__ import annotations

import argparse
import ctypes
import json
import multiprocessing as mp
import os
import pathlib
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim_py"))
sys.path.insert(0, str(ROOT / "tools"))
import evaluate_policies as ev  # noqa: E402

EVERY = 4          # ticks per sample: 100 Hz control, 25 Hz recording
VIEWER = ROOT / "tools" / "replay_viewer.html"
PLACEHOLDER = "/*DRIVES*/null"


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
            hits.append([round(env.t, 2), round(st.x, 3), round(st.y, 3)])
            contacts = info["wall_contacts"]
        if perturbed_at is None and getattr(env, "_perturbed", False):
            perturbed_at = [round(env.t, 2), round(st.x, 3), round(st.y, 3)]
        done = te or tr
        if done or ticks % EVERY == 0:
            sample(st, max(-1.0, min(1.0, float(a[1]))))
    return {"status": info["status"], "time": round(env.t, 2), "return": round(ret, 1),
            "offset_mean": round(off / max(1, ticks), 4), "wall_contacts": info["wall_contacts"],
            "energy_wh": round(info["energy_wh"], 4), "s": round(env.s, 2),
            "hits": hits, "perturbed": perturbed_at, "frames": frames}


def _job(args):
    spec, condition, seeds, lib, target_speed = args
    c = ev.campus()
    extra = {}
    if spec.startswith("sb3:"):
        cfg = json.loads(pathlib.Path(spec[4:]).with_suffix(".json").read_text(encoding="utf-8"))["config"]
        extra["history"] = cfg["history"]
    env = c.CampusEnv(seed=seeds[0], ranges=ev.ranges_for(condition), lib_path=lib,
                      target_speed=target_speed, **extra)
    policy = ev.build_policy(spec, env)
    out = []
    for k, s in enumerate(seeds):
        d = drive(env, policy, s, ev.options_for(condition))
        d["trip_seed"] = s
        d["trip"] = list(env.trip)
        d["car"] = {k2: round(v, 3) for k2, v in env.sample.items()}
        d["route"] = [[round(q["x"], 3), round(q["y"], 3)] for q in env.road["centreline"]]
        out.append(d)
    env.close()
    return spec, condition, out


def campus_geometry(lib: str) -> dict:
    c = ev.campus()
    net = c.CampusNetwork.load()
    env = c.CampusEnv(seed=0, lib_path=lib)
    env.reset(seed=0)
    contact = c.envmod.dr.Contact()
    env.lib.rc_twin_get_contact(env._car, ctypes.byref(contact))
    p = env.params
    car = {"half_length": contact.half_length, "half_width": contact.half_width,
           "centre_offset": contact.centre_offset, "steer_max": p.steer_max, "lf": p.lf, "lr": p.lr,
           "track_front": p.track_front, "track_rear": p.track_rear, "h_cog": p.h_cog,
           "wheel_radius": p.wheel_radius}
    env.close()
    r2 = lambda pts: [[round(p[0], 3), round(p[1], 3)] for p in pts]
    d = net.data
    return {"road_width": net.width, "sidewalk_width": d["road"]["sidewalk_width"],
            "edges": [r2(e["polyline"]) for e in d["edges"]],
            "sidewalks": [r2(s["points"]) for s in d["sidewalks"]],
            "walls": [{"name": w["name"], "closed": w["closed"], "points": r2(w["points"])} for w in d["walls"]],
            "buildings": [{k: b[k] for k in ("name", "x", "y", "yaw", "half_length", "half_width")}
                          for b in d["buildings"]],
            "stops": [{k: s[k] for k in ("name", "x", "y", "weight")} for s in d["stops"]],
            "car": car}


def record(policies, trips: int, conditions, lib: str, workers: int, target_speed: float) -> dict:
    seeds = list(range(ev.EVAL_SEED, ev.EVAL_SEED + trips))
    chunk = max(1, (trips + 1) // 2)
    jobs = [(spec, cond, seeds[i:i + chunk], lib, target_speed)
            for spec, _ in policies for cond in conditions for i in range(0, trips, chunk)]
    runs = {spec: {cond: [] for cond in conditions} for spec, _ in policies}
    with mp.get_context("spawn").Pool(workers) as pool:
        for spec, cond, out in pool.imap_unordered(_job, jobs):
            runs[spec][cond] += out
    for spec in runs:
        for cond in conditions:
            runs[spec][cond].sort(key=lambda d: d["trip_seed"])
    return {"format": "rc-physics-twin drives 0.1.0", "created": time.strftime("%Y-%m-%d %H:%M"),
            "target_speed": target_speed, "sample_hz": 100 // EVERY, "eval_seed": ev.EVAL_SEED,
            "conditions": list(conditions), "perturbation": ev.PERTURBATION,
            "policies": [{"spec": s, "label": lbl} for s, lbl in policies],
            "campus": campus_geometry(lib), "runs": runs}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("policies", nargs="*", help="evaluate_policies.py specs, optionally spec=label")
    ap.add_argument("--trips", type=int, default=20)
    ap.add_argument("--conditions", nargs="+", default=list(ev.CONDITIONS), choices=ev.CONDITIONS)
    ap.add_argument("--target-speed", type=float, default=4.0)
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) // 4))
    ap.add_argument("--out", type=pathlib.Path, default=ROOT / "results" / "drives.json")
    ap.add_argument("--html", type=pathlib.Path, help="also write a standalone viewer with the drives embedded")
    ap.add_argument("--lib")
    args = ap.parse_args(argv)
    lib = os.path.abspath(args.lib) if args.lib else str(ROOT / "build" / "bin" / "rc_twin.dll")
    policies = [parse_spec(p) for p in args.policies] or default_specs()
    t0 = time.perf_counter()
    data = record(policies, args.trips, args.conditions, lib, args.workers, args.target_speed)
    text = json.dumps(data, separators=(",", ":"))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(text + "\n", encoding="utf-8")
    if args.html:
        page = VIEWER.read_text(encoding="utf-8")
        assert PLACEHOLDER in page, "viewer template lost its data placeholder"
        args.html.parent.mkdir(parents=True, exist_ok=True)
        args.html.write_text(page.replace(PLACEHOLDER, text.replace("</", "<\\/"), 1), encoding="utf-8")
    for spec, label in policies:
        line = []
        for cond in args.conditions:
            rs = data["runs"][spec][cond]
            ok = sum(r["status"] == "reached_end" for r in rs)
            line.append(f"{cond} {ok}/{len(rs)}")
        print(f"{label:40s} " + "  ".join(line))
    print(f"recorded {len(policies)} policies in {time.perf_counter() - t0:.0f} s; wrote {args.out}"
          + (f" and {args.html}" if args.html else ""), file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
