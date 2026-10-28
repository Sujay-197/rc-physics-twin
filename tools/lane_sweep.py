"""Drive every campus stop pair in a lane offset to the left of the route's
centreline, with the shipped autopilot (pure pursuit with the Unity-car gains,
the corner speed plan and kerb recovery) on the Unity car with nominal
parameters, and count the trips that touch a wall (ADR 0012). Also prints how
far the car strays from the lane (the largest 95th-percentile offset of a trip),
which the traffic rules allow for where routes bend.

    python tools/lane_sweep.py 0,0.2,0.25 [--speed 2.0] [--trips S3,AB2,...]

The lane is mitred at the polyline's corners as `Lanes.Offset` (Unity) mitres
it. As the Unity autopilot does, a trip counts as running past its end only
within 1 m of the end: a route that doubles back starts beyond its end's line.
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sim_py"))
import evaluate_policies as ev  # noqa: E402

SPEC = "recover:corner:tuned:controller:pure_pursuit:policies/controllers_unity_stop.json"


def offset(pts, d: float):
    """The polyline (x, y, z) offset d to the left of its direction, mitred."""
    n = len(pts)
    out = []
    for i in range(n):
        hs = []
        if i > 0:
            hs.append(math.atan2(pts[i][1] - pts[i - 1][1], pts[i][0] - pts[i - 1][0]))
        if i < n - 1:
            hs.append(math.atan2(pts[i + 1][1] - pts[i][1], pts[i + 1][0] - pts[i][0]))
        nx = sum(-math.sin(h) for h in hs)
        ny = sum(math.cos(h) for h in hs)
        length = math.hypot(nx, ny)
        nx, ny = nx / length, ny / length
        k = d / max(nx * -math.sin(hs[0]) + ny * math.cos(hs[0]), 0.5)
        out.append([pts[i][0] + nx * k, pts[i][1] + ny * k, pts[i][2]])
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("offsets", help="comma-separated lane offsets [m]")
    ap.add_argument("--speed", type=float, default=2.0, help="target speed [m/s]")
    ap.add_argument("--trips", default=None, help="comma-separated stops to sweep among (default: all)")
    ap.add_argument("--lib", default=str(ROOT / "build" / "bin" / "rc_twin.dll"))
    args = ap.parse_args()

    c = ev.campus()
    net = c.CampusNetwork.load()
    env = c.CampusEnv(seed=0, ranges=None, lib_path=args.lib, target_speed=args.speed, task="stop", car="unity")
    past_end = env._past_end
    env._past_end = lambda st: past_end(st) if env.s > env._length - 1.0 else -1.0
    names = args.trips.split(",") if args.trips else sorted(net.stops)
    for d in (float(x) for x in args.offsets.split(",")):
        bad, n, p95 = [], 0, 0.0
        for a in names:
            for b in names:
                if a == b or net.stops[a]["node"] == net.stops[b]["node"]:
                    continue
                road = net.route_road(a, b)
                pts = [[p["x"], p["y"], p["z"]] for p in road["centreline"]]
                road["centreline"] = [{"x": p[0], "y": p[1], "z": p[2]} for p in offset(pts, d)]
                policy = ev.build_policy(SPEC, env)
                # CampusEnv.reset would replace the road with the trip's centreline.
                reset = env.reset
                env.reset = lambda seed=None, options=None: c.envmod.RcTwinEnv.reset(env, seed=seed, options={"road": road})
                try:
                    r = ev.run_episode(env, policy, seed=1)
                finally:
                    env.reset = reset
                n += 1
                p95 = max(p95, r["offset_p95"])
                if r["status"] != "reached_end" or r["wall"]:
                    bad.append((a, b, r["status"], r["wall"]))
        touched = sum(1 for x in bad if x[3])
        print(f"offset {d:+.3f} m: {n} trips, {n - sum(1 for x in bad if x[2] != 'reached_end')} reached, "
              f"{touched} touched a wall; the largest 95th-percentile offset from the lane {p95:.3f} m")
        for x in bad:
            print("   ", x)
        sys.stdout.flush()
    return 0


if __name__ == "__main__":
    sys.exit(main())
