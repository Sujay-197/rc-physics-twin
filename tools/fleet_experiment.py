"""Fleet-level evaluation of the campus shuttle game (Docs/fleet-results.md).

The real campus player (Unity, the twin's car on the rc_twin core, the shuttle
game's dispatcher, battery and traffic rules) is run headless and fast-forwarded,
with a fixed request seed, for a fixed simulated time, and writes what it measured
as JSON (ShuttleFleet: -seed, -request-interval, -duration, -warmup, -fast,
-metrics). Fast-forward runs a fixed number of 10 ms control ticks per frame, each
of ten 1 ms core steps as at normal speed, and plans turns within a tick, so a
seed gives the same file at any speed (checked by `check`).

    python tools/fleet_experiment.py build        # the player, from the committed scene (Unity closed)
    python tools/fleet_experiment.py check        # same seed at 1, 20 and 50 ticks a frame: identical?
    python tools/fleet_experiment.py run          # 1-6 buses x seeds x request intervals
    python tools/fleet_experiment.py analyse      # results/reference/fleet.json, the figure, the table

`run` skips runs whose file exists. Raw runs go to build/fleet/runs/ (not in git).
The analysis needs numpy-free stdlib only; the figure needs matplotlib.
"""
from __future__ import annotations

import argparse
import json
import math
import re
import os
import statistics
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
UNITY = Path(r"C:\Program Files\Unity\Hub\Editor\6000.5.3f1\Editor\Unity.exe")
PLAYER = ROOT / "build" / "fleet" / "player" / "RcTwinCampus.exe"
RUNS = ROOT / "build" / "fleet" / "runs"
OUT = ROOT / "results" / "reference"

# The protocol (Docs/fleet-results.md).
BUSES = [1, 2, 3, 4, 5, 6]
INTERVALS = [20.0, 10.0]          # one request every ... s: the game's default, and twice the demand
SEEDS = list(range(1, 11))
WARMUP = 600.0                     # [s] the buses start full and the stops empty
DURATION = 4500.0                  # [s] the window is (WARMUP, DURATION]: 65 min
TAIL = 300.0                       # [s] requests raised in the last TAIL s are not scored (give-up 180 s + a ride)
FAST = 20                          # control ticks per frame


def run_path(interval: float, buses: int, seed: int) -> Path:
    return RUNS / f"i{interval:g}" / f"b{buses}_s{seed}.json"


def player_cmd(player: Path, buses: int, seed: int, interval: float, out: Path, duration=DURATION,
               warmup=WARMUP, fast=FAST) -> list[str]:
    return [str(player), "-batchmode", "-nographics", "-buses", str(buses), "-seed", str(seed),
            "-request-interval", f"{interval:g}", "-duration", f"{duration:g}", "-warmup", f"{warmup:g}",
            "-fast", str(fast), "-metrics", str(out), "-logFile", str(out.with_suffix(".log"))]


def run_one(player: Path, buses: int, seed: int, interval: float, out: Path, **kw) -> tuple[Path, float, int]:
    out.parent.mkdir(parents=True, exist_ok=True)
    t0 = time.perf_counter()
    p = subprocess.run(player_cmd(player, buses, seed, interval, out, **kw),
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=7200)
    return out, time.perf_counter() - t0, p.returncode


# ------------------------------------------------------------------ commands

def cmd_build(a) -> None:
    log = ROOT / "build" / "fleet" / "player-build.log"
    subprocess.run([str(UNITY), "-batchmode", "-quit", "-projectPath", str(ROOT / "unity" / "RcTwin"),
                    "-executeMethod", "RcTwin.EditorTools.CampusSceneBuilder.BuildPlayerAsIsBatch",
                    "-buildPath", str(PLAYER), "-logFile", str(log)], check=True)
    print(f"built {PLAYER}")


def cmd_check(a) -> None:
    """The same seed, fast-forwarded by different amounts and twice at one: identical files?"""
    d = ROOT / "build" / "fleet" / "check"
    jobs = [(f, d / f"f{f}{tag}.json") for f, tag in [(1, ""), (20, ""), (20, "b"), (50, "")]]
    with ThreadPoolExecutor(len(jobs)) as ex:
        list(ex.map(lambda j: run_one(Path(a.player), a.buses, 7, 20.0, j[1], duration=a.duration, warmup=60, fast=j[0]), jobs))

    def load(p):
        r = json.loads(p.read_text())
        r.pop("wall_s")
        r["config"].pop("fast")
        return r
    base = load(jobs[1][1])
    same = all(load(p) == base for _, p in jobs)
    print(f"{a.buses} buses, seed 7, {a.duration:g} s at 1, 20, 20 and 50 ticks a frame: "
          f"{'identical' if same else 'DIFFERENT'} ({base['counters_window']['delivered']:g} delivered)")
    if not same:
        sys.exit(1)


def cmd_run(a) -> None:
    jobs = [(b, s, i) for i in a.intervals for b in a.buses for s in a.seeds if not run_path(i, b, s).exists()]
    print(f"{len(jobs)} runs to do, {a.workers} at a time")
    t0 = time.perf_counter()
    with ThreadPoolExecutor(a.workers) as ex:
        for k, (out, wall, rc) in enumerate(ex.map(lambda j: run_one(Path(a.player), j[0], j[1], j[2], run_path(j[2], j[0], j[1])), jobs)):
            ok = out.exists() and rc == 0
            print(f"[{k + 1}/{len(jobs)}] {out.relative_to(ROOT)} {wall:.0f} s{'' if ok else f' FAILED rc={rc}'}", flush=True)
    print(f"done in {time.perf_counter() - t0:.0f} s")


# ------------------------------------------------------------------ analysis

# Two-sided 95 % Student t quantiles, df 1..30.
T95 = [12.706, 4.303, 3.182, 2.776, 2.571, 2.447, 2.365, 2.306, 2.262, 2.228, 2.201, 2.179, 2.160, 2.145,
       2.131, 2.120, 2.110, 2.101, 2.093, 2.086, 2.080, 2.074, 2.069, 2.064, 2.060, 2.056, 2.052, 2.048, 2.045, 2.042]


def ci(xs: list[float]) -> dict:
    xs = [x for x in xs if x is not None and not math.isnan(x)]
    n = len(xs)
    if n == 0:
        return {"mean": None, "ci95": None, "n": 0}
    m = statistics.fmean(xs)
    h = T95[min(n - 1, 30) - 1] * statistics.stdev(xs) / math.sqrt(n) if n > 1 else None
    return {"mean": m, "ci95": h, "n": n}


def quantile(xs: list[float], q: float) -> float:
    """Linear interpolation between order statistics (numpy's default)."""
    s = sorted(xs)
    if not s:
        return float("nan")
    k = (len(s) - 1) * q
    f = math.floor(k)
    return s[f] + (s[min(f + 1, len(s) - 1)] - s[f]) * (k - f)


def run_metrics(r: dict) -> dict:
    """One run's numbers: requests raised in (start, end - TAIL]; times over the window."""
    start, end = r["window"]["start_s"], r["window"]["end_s"]
    hours = (end - start) / 3600.0
    reqs = [q for q in r["requests"] if not q["user"] and start < q["raised"] <= end - TAIL]
    served = [q for q in reqs if q["state"] == "Delivered"]
    gave_up = [q for q in reqs if q["state"] == "GaveUp"]
    waits = [q["picked"] - q["raised"] for q in served]
    rides = [q["dropped"] - q["picked"] for q in served]
    detours = [(q["dropped"] - q["picked"]) / q["direct_s"] for q in served if q["direct_s"] > 0]
    dropped_in = [q for q in r["requests"] if q["dropped"] is not None and start < q["dropped"] <= end]
    buses = r["buses"]
    bus_s = sum(b["time_s"] for b in buses)
    drive_s = sum(b["driving_s"] for b in buses)
    c = r["counters_window"]
    wh = sum(b["energy_wh"] for b in buses)
    return {
        "requests": len(reqs), "served": len(served), "gave_up": len(gave_up),
        "unresolved": len(reqs) - len(served) - len(gave_up),
        "wait_mean_s": statistics.fmean(waits) if waits else float("nan"),
        "wait_p90_s": quantile(waits, 0.9),
        "ride_mean_s": statistics.fmean(rides) if rides else float("nan"),
        "ride_over_direct": statistics.fmean(detours) if detours else float("nan"),
        "give_up_share": len(gave_up) / len(reqs) if reqs else float("nan"),
        "rides_per_hour": len(dropped_in) / hours,
        "occupancy": sum(b["seat_s"] for b in buses) / bus_s,
        "occupancy_driving": sum(b["seat_s_driving"] for b in buses) / drive_s if drive_s > 0 else float("nan"),
        "driving_share": drive_s / bus_s,
        "dwelling_share": sum(b["dwelling_s"] for b in buses) / bus_s,
        "idle_share": sum(b["idle_s"] for b in buses) / bus_s,
        "charging_share": sum(b["charging_s"] for b in buses) / bus_s,
        "towed_share": sum(b["towed_s"] for b in buses) / bus_s,
        "km": sum(b["metres"] for b in buses) / 1000.0,
        "energy_wh": wh,
        "wh_per_ride": wh / len(dropped_in) if dropped_in else float("nan"),
        "wh_per_km": wh / (sum(b["metres"] for b in buses) / 1000.0),
        # The traffic rules (ADR 0012) in the window.
        "give_way_stops_per_h": c["give_way_stops"] / hours,
        "held_s_per_bus_h": c["held_s"] / (bus_s / 3600.0),
        "crossings_per_h": c["crossings"] / hours,
        "cycles_broken": c["cycles_broken"], "made_room": c["made_room"],
        "served_from_behind": c["served_from_behind"],
        "stuck_tows": c["stuck_tows"], "overlaps": c["overlaps"],
        "leg_fail_tows": c["leg_fail_tows"], "flats": c["flats"], "kerb_strikes": c["kerb_strikes"],
        "wall_s": r["wall_s"],
    }


# ShuttleFleet logs each new overlap with every bus's pose and state; the touching
# pair is taken as the two closest centres (the footprint is 0.42 m by 0.275 m).
_BUS = re.compile(r"Bus (\d+) at \((-?[\d.]+), (-?[\d.]+)\) yaw (-?[\d.]+) v (-?[\d.]+), (\w+) \((.*?)\), (\w+)( turning)?, "
                  r"hold ([^:]+): (.*?)(?=; Bus \d+ at |$)")


def overlap_events(log: Path, start: float, end: float) -> list[dict]:
    out = []
    if not log.exists():
        return out
    for line in log.read_text(encoding="utf-8", errors="replace").splitlines():
        m = re.match(r"ShuttleFleet: buses overlap at ([\d.]+) s: (.*)", line.strip())
        if not m or not (start < float(m.group(1)) <= end):
            continue
        bs = [g.groups() for g in _BUS.finditer(m.group(2))]
        pairs = [(math.hypot(float(p[1]) - float(q[1]), float(p[2]) - float(q[2])), p, q)
                 for k, p in enumerate(bs) for q in bs[k + 1:]]
        if not pairs:
            continue
        d, p, q = min(pairs, key=lambda x: x[0])
        out.append({"t": float(m.group(1)), "centres_m": d, "speed_max": max(abs(float(p[4])), abs(float(q[4]))),
                    "turning": bool(p[8]) or bool(q[8])})
    return out


SUMS = ["requests", "served", "gave_up", "unresolved", "cycles_broken", "made_room", "served_from_behind",
        "stuck_tows", "overlaps", "leg_fail_tows", "flats", "kerb_strikes",
        "overlaps_logged", "overlaps_at_rest", "overlaps_turning"]


def cmd_analyse(a) -> None:
    runs, summary = [], []
    for i in a.intervals:
        for b in a.buses:
            ms = []
            for s in a.seeds:
                p = run_path(i, b, s)
                if not p.exists():
                    continue
                r = json.loads(p.read_text())
                m = run_metrics(r)
                ev = overlap_events(p.with_suffix(".log"), r["window"]["start_s"], r["window"]["end_s"])
                m["overlaps_logged"] = len(ev)
                m["overlaps_at_rest"] = sum(e["speed_max"] < 0.1 for e in ev)
                m["overlaps_turning"] = sum(e["turning"] for e in ev)
                runs.append({"interval_s": i, "buses": b, "seed": s, **m})
                ms.append(m)
            if not ms:
                continue
            row = {"interval_s": i, "buses": b, "seeds": len(ms)}
            for k in ms[0]:
                row[k] = ci([m[k] for m in ms])
            for k in SUMS:
                row[k]["total"] = sum(m[k] for m in ms)
            summary.append(row)
    commit = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "--short", "HEAD"], capture_output=True, text=True).stdout.strip()
    doc = {
        "what": "Fleet-level evaluation of the campus shuttle game: service against fleet size (Docs/fleet-results.md).",
        "protocol": {
            "player": "build/fleet/player/RcTwinCampus.exe (CampusSceneBuilder.BuildPlayerAsIsBatch), headless (-batchmode -nographics)",
            "fast_ticks_per_frame": FAST, "control_tick_s": 0.01, "core_dt_s": 0.001,
            "warmup_s": WARMUP, "duration_s": DURATION, "window_s": DURATION - WARMUP, "scored_requests": f"raised in (warmup, duration - {TAIL:g} s]",
            "buses": a.buses, "request_intervals_s": a.intervals, "seeds": a.seeds,
            "ci": "mean over seeds, 95 % Student t interval of the per-seed values (half width)",
            "game_defaults": "capacity 6, cruise 2 m/s, give up after 180 s, battery 5 min of driving, chargers AB1 and AB3, lane offset 0.2 m",
        },
        "commit": commit,
        "summary": summary,
        "runs": runs,
    }
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "fleet.json").write_text(json.dumps(doc, indent=1) + "\n")
    print(f"wrote {OUT / 'fleet.json'} ({len(runs)} runs)")
    print(table(summary))
    figure(summary)


def fmt(c: dict, scale=1.0, nd=1) -> str:
    if c["mean"] is None:
        return "-"
    s = f"{c['mean'] * scale:.{nd}f}"
    return s + (f" ± {c['ci95'] * scale:.{nd}f}" if c["ci95"] is not None else "")


def table(summary: list[dict]) -> str:
    head = ("| Interval | Buses | Wait mean [s] | Wait P90 [s] | Ride [s] | Rides/h | Gave up [%] | Occupancy | "
            "Charging [%] | Wh/ride | Give-way stops/h | Stuck tows | Overlaps |")
    lines = [head, "|" + "---|" * (head.count("|") - 1)]
    for r in summary:
        lines.append(f"| {r['interval_s']:g} s | {r['buses']} | {fmt(r['wait_mean_s'])} | {fmt(r['wait_p90_s'])} | "
                     f"{fmt(r['ride_mean_s'])} | {fmt(r['rides_per_hour'])} | {fmt(r['give_up_share'], 100)} | "
                     f"{fmt(r['occupancy'], 1, 2)} | {fmt(r['charging_share'], 100)} | {fmt(r['wh_per_ride'], 1, 3)} | "
                     f"{fmt(r['give_way_stops_per_h'])} | {r['stuck_tows']['total']:g} | {r['overlaps']['total']:g} |")
    return "\n".join(lines)


def figure(summary: list[dict]) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family": "serif", "mathtext.fontset": "cm", "font.size": 7.5,
                         "axes.linewidth": 0.6, "xtick.major.width": 0.6, "ytick.major.width": 0.6,
                         "axes.spines.top": False, "axes.spines.right": False})
    width = 122 / 25.4   # LNCS text width
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(width, 2.1))
    colours = {20.0: "#2a78d6", 10.0: "#eb6834"}   # categorical slots 1 and 2
    marks = {20.0: "o", 10.0: "s"}
    for i in sorted({r["interval_s"] for r in summary}, reverse=True):
        rows = [r for r in summary if r["interval_s"] == i]
        x = [r["buses"] for r in rows]
        lab = f"1 request / {i:g} s"
        for key, ls, fill in [("wait_mean_s", "-", True), ("wait_p90_s", "--", False)]:
            y = [r[key]["mean"] for r in rows]
            e = [r[key]["ci95"] or 0.0 for r in rows]
            a1.errorbar(x, y, yerr=e, color=colours[i], ls=ls, lw=1.2, marker=marks[i], ms=3.5, capsize=1.8,
                        elinewidth=0.6, mfc=colours[i] if fill else "white", mew=0.8,
                        label=f"{lab}, {'mean' if fill else 'P90'}")
        y = [100 * r["give_up_share"]["mean"] for r in rows]
        e = [100 * (r["give_up_share"]["ci95"] or 0.0) for r in rows]
        a2.errorbar(x, y, yerr=e, color=colours[i], lw=1.2, marker=marks[i], ms=3.5, capsize=1.8, elinewidth=0.6, label=lab)
    for ax in (a1, a2):
        ax.set_xlabel("buses")
        ax.set_xticks(range(1, 7))
        ax.grid(axis="y", lw=0.4, color="#d9d9d6")
        ax.set_ylim(bottom=0)
    a1.set_ylabel("wait for pick-up [s]")
    a2.set_ylabel("students who gave up [%]")
    a1.set_title("(a) wait of the students served", fontsize=7.5)
    a2.set_title("(b) gave up (after 180 s)", fontsize=7.5)
    h, l = a1.get_legend_handles_labels()
    a2.legend(h, l, frameon=False, fontsize=6, handlelength=2.6, loc="upper right")   # (b) is empty there
    fig.tight_layout(pad=0.3, w_pad=1.2)
    for ext in ("pdf", "png"):
        fig.savefig(OUT / f"fleet_wait.{ext}", dpi=300, metadata={"CreationDate": None} if ext == "pdf" else None)
    print(f"wrote {OUT / 'fleet_wait.pdf'} and .png")


def main() -> None:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=["build", "check", "run", "analyse"])
    ap.add_argument("--player", default=str(PLAYER))
    ap.add_argument("--buses", type=int, nargs="+", default=BUSES)
    ap.add_argument("--seeds", type=int, nargs="+", default=SEEDS)
    ap.add_argument("--intervals", type=float, nargs="+", default=INTERVALS)
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) // 2))
    ap.add_argument("--duration", type=float, default=300.0, help="check: simulated seconds")
    a = ap.parse_args()
    if a.command == "check":
        a.buses = a.buses[0] if len(a.buses) == 1 else 4
    {"build": cmd_build, "check": cmd_check, "run": cmd_run, "analyse": cmd_analyse}[a.command](a)


if __name__ == "__main__":
    main()
