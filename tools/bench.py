#!/usr/bin/env python3
"""Runtime benchmark of the twin (Docs/performance.md).

Measures, single-threaded, on an otherwise idle machine:

  native   ns per 1 kHz Core step through rc_twin at L0, L1 and L2, L2 on
           road ground, with the Unity car's features and with the campus
           walls; the Core called directly for the C ABI's overhead
           (native/bench/bench_step.cpp, built with -DRC_TWIN_BENCH=ON)
  python   environment steps per second and episode wall time for
           RcTwinEnv, CampusEnv and CampusEnv on the stop task
  profiler wall time for tools/edge_profiler.py over the 37 campus edges

Every measurement is repeated; results/reference/runtime.json keeps every
sample and the median and interquartile range. Before each measurement the
script waits until the machine's CPU load (all logical CPUs, GetSystemTimes)
falls below --idle-threshold, and it records the load seen during the
measurement; a run whose load from other processes exceeded the threshold is
retried, up to --retries times, and kept flagged if it never got below.

Usage (from the repo root, after building with -DCMAKE_BUILD_TYPE=Release
-DRC_TWIN_BENCH=ON):
    python tools/bench.py --out results/reference/runtime.json
    python tools/bench.py --quick       # a short smoke run, nothing written

Windows only: the load probe and the hardware record use Windows APIs.
Standard library only.
"""

from __future__ import annotations

import argparse
import ctypes
import glob
import importlib.util
import json
import math
import os
import pathlib
import platform
import statistics
import subprocess
import sys
import tempfile
import time

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent

NATIVE_RUNS = [  # (case, mode)
    ("l0", "core"), ("l0", "abi1"), ("l0", "abi10"),
    ("l1", "core"), ("l1", "abi1"), ("l1", "abi10"),
    ("l2", "core"), ("l2", "abi1"), ("l2", "abi10"),
    ("l2_road", "abi1"), ("l2_road", "abi10"),
    ("l2_unity", "abi1"), ("l2_unity", "abi10"),
    ("l2_unity_walls", "abi1"), ("l2_unity_walls", "abi10"),
]
PROFILER_SWEEP = ["--speeds", "1", "2", "3", "--payloads", "0", "1", "--soc", "0.8", "0.4"]


def _load(name: str, path: pathlib.Path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# ------------------------------------------------------------------ machine

class _FileTime(ctypes.Structure):
    _fields_ = [("lo", ctypes.c_uint32), ("hi", ctypes.c_uint32)]


def _system_times() -> tuple[int, int]:
    """(idle, total) 100 ns ticks over all logical CPUs since boot."""
    idle, kern, user = _FileTime(), _FileTime(), _FileTime()
    if not ctypes.windll.kernel32.GetSystemTimes(ctypes.byref(idle), ctypes.byref(kern),
                                                 ctypes.byref(user)):
        raise OSError("GetSystemTimes failed")
    v = lambda f: (f.hi << 32) | f.lo  # noqa: E731
    return v(idle), v(kern) + v(user)  # kernel time includes idle time


class LoadProbe:
    """CPU busy fraction (0..1, all logical CPUs) between start() and stop()."""

    def start(self):
        self._t0 = _system_times()
        return self

    def stop(self) -> float:
        i1, t1 = _system_times()
        di, dt = i1 - self._t0[0], t1 - self._t0[1]
        return 0.0 if dt <= 0 else max(0.0, 1.0 - di / dt)


def sample_load(seconds: float) -> float:
    p = LoadProbe().start()
    time.sleep(seconds)
    return p.stop()


def wait_idle(threshold: float, timeout: float, log) -> tuple[float, float]:
    """Wait until a 2 s load sample is below threshold. Returns (load, waited s)."""
    t0 = time.monotonic()
    while True:
        load = sample_load(2.0)
        waited = time.monotonic() - t0
        if load < threshold or waited > timeout:
            if load >= threshold:
                log(f"  machine still busy ({load:.0%}) after {waited:.0f} s; measuring anyway")
            return load, waited
        log(f"  busy ({load:.0%} >= {threshold:.0%}); waiting")
        time.sleep(10.0)


def measured(fn, args, log, own_threads: int = 1):
    """Run fn() once the machine is idle; retry if others loaded it meanwhile.
    Returns (fn's result, conditions dict)."""
    ncpu = os.cpu_count() or 1
    own = own_threads / ncpu
    tries = []
    for attempt in range(args.retries + 1):
        before, waited = wait_idle(args.idle_threshold, args.idle_timeout, log)
        probe = LoadProbe().start()
        result = fn()
        during = probe.stop()
        others = max(0.0, during - own)
        tries.append({"load_before": round(before, 4), "load_during": round(during, 4),
                      "load_others_during": round(others, 4), "waited_s": round(waited, 1)})
        if others < args.idle_threshold:
            break
        log(f"  others loaded the machine to {others:.0%} during the run; retrying")
    cond = dict(tries[-1], attempts=len(tries), idle=tries[-1]["load_others_during"] < args.idle_threshold)
    return result, cond


def stats(xs: list[float]) -> dict:
    xs = sorted(xs)
    if len(xs) >= 2:
        q1, _, q3 = statistics.quantiles(xs, n=4, method="inclusive")
    else:
        q1 = q3 = xs[0]
    return {"median": statistics.median(xs), "q1": q1, "q3": q3, "iqr": q3 - q1, "n": len(xs)}


def _ps_json(script: str):
    try:
        out = subprocess.run(["powershell", "-NoProfile", "-Command", script], capture_output=True,
                             text=True, timeout=60).stdout
        return json.loads(out)
    except (OSError, subprocess.SubprocessError, json.JSONDecodeError):
        return None


def hardware(build: pathlib.Path) -> dict:
    cpu = _ps_json("Get-CimInstance Win32_Processor | Select-Object Name,NumberOfCores,"
                   "NumberOfLogicalProcessors,MaxClockSpeed | ConvertTo-Json")
    gpus = _ps_json("@(Get-CimInstance Win32_VideoController | Select-Object Name,DriverVersion) "
                    "| ConvertTo-Json")
    osinfo = _ps_json("Get-CimInstance Win32_OperatingSystem | Select-Object Caption,Version,"
                      "BuildNumber | ConvertTo-Json")
    ram = _ps_json("(Get-CimInstance Win32_ComputerSystem).TotalPhysicalMemory | ConvertTo-Json")
    power = None
    try:
        power = subprocess.run(["powercfg", "/getactivescheme"], capture_output=True, text=True,
                               timeout=30).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        pass
    if isinstance(gpus, dict):
        gpus = [gpus]
    return {
        "cpu": cpu and cpu["Name"].strip(), "physical_cores": cpu and cpu["NumberOfCores"],
        "logical_processors": cpu and cpu["NumberOfLogicalProcessors"],
        "cpu_base_clock_mhz": cpu and cpu["MaxClockSpeed"],
        "ram_gib": ram and round(ram / 2 ** 30, 1),
        "gpus": [f"{g['Name']} (driver {g['DriverVersion']})" for g in gpus or []],
        "os": osinfo and f"{osinfo['Caption']} {osinfo['Version']} (build {osinfo['BuildNumber']})",
        "power_scheme": power,
        "python": f"{platform.python_implementation()} {platform.python_version()} ({sys.executable})",
        "compiler": compiler(build),
    }


def compiler(build: pathlib.Path) -> dict:
    cache = {}
    try:
        for line in (build / "CMakeCache.txt").read_text(encoding="utf-8").splitlines():
            if ":" in line and "=" in line and not line.startswith(("//", "#")):
                k, v = line.split("=", 1)
                cache[k.split(":")[0]] = v
    except OSError:
        pass
    cxx = cache.get("CMAKE_CXX_COMPILER", "")
    if not cxx and cache.get("CMAKE_TOOLCHAIN_FILE"):
        try:  # set by the toolchain file, so not in the cache
            for line in pathlib.Path(cache["CMAKE_TOOLCHAIN_FILE"]).read_text(encoding="utf-8").splitlines():
                if line.startswith("set(CMAKE_CXX_COMPILER "):
                    cxx = line.split('"')[1]
        except (OSError, IndexError):
            pass
    info = {"cmake_build_type": cache.get("CMAKE_BUILD_TYPE"),
            "cxx_flags_release": cache.get("CMAKE_CXX_FLAGS_RELEASE"),
            "cxx_compiler": cxx}
    # The toolchain wrapper (tools/bootstrap_toolchain.py) holds the target flags.
    try:
        wrapper = next(ln for ln in pathlib.Path(cxx).read_text(encoding="utf-8").splitlines() if "zig" in ln)
        info["wrapper"] = wrapper.strip()
        zig = wrapper.split('"')[1]
        info["zig"] = subprocess.run([zig, "version"], capture_output=True, text=True).stdout.strip()
        cc = subprocess.run([zig, "cc", "--version"], capture_output=True, text=True).stdout
        info["clang"] = cc.splitlines()[0].strip() if cc else None
    except (OSError, IndexError, StopIteration):
        pass
    ninja = cache.get("CMAKE_MAKE_PROGRAM")
    if ninja:
        for target, src in (("rc_twin", "rc_twin.cpp"), ("bench_step", "bench_step.cpp")):
            try:
                cmds = subprocess.run([ninja, "-C", str(build), "-t", "commands", target],
                                      capture_output=True, text=True).stdout.splitlines()
                line = next(c for c in cmds if c.rstrip().endswith(src))
                flags = [t for t in line.split() if t.startswith(("-O", "-D", "-std", "-f", "-W", "-m"))
                         and not t.startswith(("-MD", "-MT", "-MF"))]
                info[f"flags_{target}"] = " ".join(flags)
            except (OSError, StopIteration):
                pass
    return info


# ------------------------------------------------------------------- native

def write_scenario(path: pathlib.Path, envmod, campus) -> dict:
    """The longest stop-to-stop campus route, resampled at 5 cm, with the
    campus walls and the Unity car's options."""
    net = campus.CampusNetwork.load()
    best = None
    for a in sorted(net.stops):
        for b in sorted(net.stops):
            if net.stops[a]["node"] == net.stops[b]["node"]:
                continue
            pts = net.route_points(net.stops[a]["node"], net.stops[b]["node"])
            length = sum(math.hypot(q[0] - p[0], q[1] - p[1]) for p, q in zip(pts, pts[1:]))
            if best is None or length > best[0]:
                best = (length, a, b, pts)
    length, a, b, pts = best
    res = [pts[0]]
    for p, q in zip(pts, pts[1:]):
        seg = math.hypot(q[0] - p[0], q[1] - p[1])
        n = max(1, int(math.ceil(seg / 0.05)))
        for k in range(1, n + 1):
            t = k / n
            res.append([p[i] + t * (q[i] - p[i]) for i in range(3)])
    lib = envmod.dr.load_lib(ROOT / "build" / "bin" / "rc_twin.dll")
    mass = envmod.dr.car_params(lib, type("A", (), {**envmod.UNITY_CAR, "wheel_rate": 0.0,
                                                    "damping": 0.0, "payload": 0.0})()).mass
    u = envmod.unity_car(mass)
    lines = [f"route {len(res)}"] + [f"{p[0]!r} {p[1]!r} {p[2]!r}" for p in res]
    lines += [f"road {len(pts)}"] + [f"{p[0]!r} {p[1]!r} {p[2]!r}" for p in pts]
    lines.append(f"width {net.width!r}")
    lines.append("unity " + " ".join(repr(float(x)) for x in (
        u["wheel_inertia"], u["ackermann"], u["wheel_rate"], u["damping"], u["travel"],
        u["v_kinematic"], envmod.FOOTPRINT_MARGIN)))
    lines.append(f"walls {len(net.walls)}")
    segs = 0
    for w in net.walls:
        flat = " ".join(f"{p[0]!r} {p[1]!r}" for p in w["points"])
        lines.append(f"{1 if w.get('closed') else 0} {len(w['points'])} {flat}")
        segs += len(w["points"]) - 1 + (1 if w.get("closed") else 0)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return {"route": f"{a} -> {b}", "route_length_m": round(length, 3), "driver_path_points": len(res),
            "road_centreline_points": len(pts),
            "road_width_m": net.width, "walls": len(net.walls), "wall_segments_max": segs,
            "unity_car": u, "footprint_margin_m": envmod.FOOTPRINT_MARGIN,
            "controller": "pure pursuit, 0.6 m lookahead, speed hold at 2 m/s, updated every 10 steps",
            "dt_s": 0.001, "integrator": "RK4"}


def bench_native(args, exe: pathlib.Path, scenario: pathlib.Path, log) -> list[dict]:
    out = []
    for case, mode in NATIVE_RUNS:
        log(f"native {case} {mode}")

        def go():
            r = subprocess.run([str(exe), str(scenario), case, mode, str(args.steps), str(args.reps)],
                               capture_output=True, text=True, check=True)
            return json.loads(r.stdout)

        res, cond = measured(go, args, log)
        ns = res["ns_per_step"]
        rtf = [1e6 / x for x in ns]   # simulated s per wall s at dt = 1 ms
        out.append({"case": case, "mode": mode, "steps_per_run": res["steps"],
                    "ns_per_step": stats(ns), "real_time_factor": stats(rtf),
                    "samples_ns": ns, "resets_per_run": res["resets_per_run"],
                    "wall_contacts_per_run": res["wall_contacts_per_run"], "conditions": cond})
        log(f"  {statistics.median(ns):.1f} ns/step, RTF {statistics.median(rtf):.0f}, {cond}")
    return out


UNITY_FEATURES = {  # scenario "unity" fields each one-feature variant keeps
    "none": (), "wheel_inertia": (0,), "ackermann": (1,), "suspension": (2, 3),
    "v_kinematic": (5,), "all": (0, 1, 2, 3, 5),
}


def bench_ablation(args, exe: pathlib.Path, scenario: pathlib.Path, log) -> list[dict]:
    """L2 on road ground with one Unity car feature at a time, rc_twin n = 1."""
    lines = scenario.read_text(encoding="utf-8").splitlines()
    i = next(k for k, ln in enumerate(lines) if ln.startswith("unity "))
    full = lines[i].split()[1:]
    out = []
    for name, keep in UNITY_FEATURES.items():
        vals = [full[k] if (k in keep or k == 4) else "0.0" for k in range(len(full))]
        variant = scenario.with_name(f"scenario_{name}.txt")
        variant.write_text("\n".join(lines[:i] + ["unity " + " ".join(vals)] + lines[i + 1:]) + "\n",
                           encoding="utf-8")
        log(f"ablation l2 road ground + {name}")

        def go():
            r = subprocess.run([str(exe), str(variant), "l2_unity", "abi1", str(args.steps),
                                str(args.reps)], capture_output=True, text=True, check=True)
            return json.loads(r.stdout)

        res, cond = measured(go, args, log)
        ns = res["ns_per_step"]
        out.append({"features": name, "ns_per_step": stats(ns),
                    "real_time_factor": stats([1e6 / x for x in ns]), "samples_ns": ns,
                    "conditions": cond})
        log(f"  {statistics.median(ns):.1f} ns/step, {cond}")
    return out


# ------------------------------------------------------------------- python

def run_episodes(env, policy, seeds: list[int]) -> dict:
    steps, step_time, sim = 0, 0.0, 0.0
    episodes = []
    for seed in seeds:
        t0 = time.perf_counter()
        obs, _ = env.reset(seed=seed)
        n, ts = 0, 0.0
        while True:
            a = policy(obs)
            s0 = time.perf_counter()
            obs, _, term, trunc, info = env.step(a)
            ts += time.perf_counter() - s0
            n += 1
            if term or trunc:
                break
        wall = time.perf_counter() - t0
        episodes.append({"seed": seed, "steps": n, "sim_s": round(env.t, 4), "wall_s": wall,
                         "status": info.get("status")})
        steps += n
        step_time += ts
        sim += env.t
    return {"steps": steps, "step_time_s": step_time, "sim_s": sim, "episodes": episodes}


def bench_python(args, log) -> list[dict]:
    envmod = _load("rc_twin_env", ROOT / "sim_py" / "rc_twin_env.py")
    campus = _load("campus", ROOT / "sim_py" / "campus.py")
    lib = ROOT / "build" / "bin" / "rc_twin.dll"
    road = ROOT / "roads" / "samples" / "hill_breaker_bend.json"
    envs = [
        ("RcTwinEnv follow, roads/samples/hill_breaker_bend.json",
         lambda: envmod.RcTwinEnv(road, lib_path=lib)),
        ("CampusEnv follow, campus walls",
         lambda: campus.CampusEnv(seed=0, lib_path=lib)),
        ("CampusEnv stop task, campus walls",
         lambda: campus.CampusEnv(seed=0, lib_path=lib, task="stop")),
    ]
    seeds = list(range(args.episodes))
    out = []
    for name, make in envs:
        log(f"python {name}")
        env = make()
        run_episodes(env, envmod.pure_pursuit_policy, seeds[:1])  # warm-up

        def go():
            return [run_episodes(env, envmod.pure_pursuit_policy, seeds) for _ in range(args.py_reps)]

        rounds, cond = measured(go, args, log)
        env.close()
        sps = [r["steps"] / r["step_time_s"] for r in rounds]
        rtf = [r["sim_s"] / sum(e["wall_s"] for e in r["episodes"]) for r in rounds]
        eps = [e for r in rounds for e in r["episodes"]]
        statuses = {}
        for e in rounds[0]["episodes"]:
            statuses[e["status"]] = statuses.get(e["status"], 0) + 1
        out.append({
            "env": name, "policy": "rc_twin_env.pure_pursuit_policy", "ranges": "RANGES_TRAIN",
            "control_hz": 100, "core_steps_per_env_step": envmod.SUBSTEPS,
            "episodes_per_round": len(seeds), "rounds": len(rounds),
            "env_steps_per_s": stats(sps),
            "episode_real_time_factor": stats(rtf),
            "episode_wall_s": stats([e["wall_s"] for e in eps]),
            "episode_sim_s": stats([e["sim_s"] for e in rounds[0]["episodes"]]),
            "episode_env_steps": stats([e["steps"] for e in rounds[0]["episodes"]]),
            "statuses_round0": statuses, "conditions": cond,
        })
        log(f"  {statistics.median(sps):.0f} env steps/s, episode median "
            f"{statistics.median(e['wall_s'] for e in eps):.3f} s wall, {cond}")
    return out


# ----------------------------------------------------------------- profiler

def bench_profiler(args, log) -> dict:
    roads = sorted(glob.glob(str(ROOT / "roads" / "campus" / "*.json")))
    log(f"edge profiler, {len(roads)} edges, sweep {' '.join(PROFILER_SWEEP)}")
    times, entries = [], None
    tmp = pathlib.Path(tempfile.mkdtemp(prefix="rc_twin_bench_"))
    conds = []
    for i in range(args.profiler_reps):
        out = tmp / f"cache_{i}.json"

        def go():
            out.unlink(missing_ok=True)  # a retry must not reuse the last attempt's cache
            t0 = time.perf_counter()
            subprocess.run([sys.executable, "-I", str(ROOT / "tools" / "edge_profiler.py"), *roads,
                            "--out", str(out), *PROFILER_SWEEP,
                            "--lib", str(ROOT / "build" / "bin" / "rc_twin.dll")],
                           check=True, capture_output=True, text=True)
            return time.perf_counter() - t0

        t, cond = measured(go, args, log)
        times.append(t)
        conds.append(cond)
        cache = json.loads(out.read_text(encoding="utf-8"))["entries"]
        entries = len(cache)
        breakers = {json.loads(pathlib.Path(r).read_text(encoding="utf-8"))["id"]
                    for r in roads
                    if json.loads(pathlib.Path(r).read_text(encoding="utf-8")).get("speed_breakers")}
        # Simulated driving: each entry's run plus, on edges with breakers, the
        # run without them (time_s minus the breaker penalty).
        sim_s = sum((e["time_s"] or 0.0) + ((e["time_s"] - e["breaker_penalty_s"])
                                            if e["edge_id"] in breakers and e["time_s"] is not None
                                            and e["breaker_penalty_s"] is not None else 0.0)
                    for e in cache)
        statuses = {}
        for e in cache:
            statuses[e["status"]] = statuses.get(e["status"], 0) + 1
        log(f"  run {i}: {t:.2f} s, {cond}")
    sweep_combos = 3 * 2 * 2
    return {"command": "python -I tools/edge_profiler.py roads/campus/*.json --out <fresh file> "
                       + " ".join(PROFILER_SWEEP),
            "edges": len(roads), "sweep": "speeds 1 2 3 m/s x payloads 0 1 kg x soc 0.8 0.4",
            "combinations_per_edge": sweep_combos, "entries": entries,
            "wall_s": stats(times), "samples_s": times,
            "ms_per_entry": stats([1000 * t / entries for t in times]),
            "simulated_driving_s": round(sim_s, 3), "edges_with_breakers": len(breakers),
            "entry_statuses": statuses,
            "real_time_factor": stats([sim_s / t for t in times]),
            "note": "each entry is two drives of the edge (with and without its speed breakers) "
                    "where the edge has breakers; process start-up and JSON I/O are included",
            "conditions": conds}


# ------------------------------------------------------------------- report

CASE_NAMES = {
    "l0": "L0 kinematic", "l1": "L1 bicycle", "l2": "L2 double track",
    "l2_road": "L2 on road ground", "l2_unity": "L2, Unity car, road ground",
    "l2_unity_walls": "L2, Unity car, road ground, campus walls",
}


def _mi(s: dict, fmt: str) -> str:
    return f"{format(s['median'], fmt)} ({format(s['q1'], fmt)} to {format(s['q3'], fmt)})"


def report(d: dict) -> str:
    """Markdown tables from a runtime.json: median (interquartile range)."""
    out = []
    if "native" in d:
        rows = {(r["case"], r["mode"]): r for r in d["native"]}
        out += ["| Core step, 1 kHz | through rc_twin, n = 1 [ns] | real-time factor | "
                "rc_twin, n = 10 [ns] | Core direct [ns] |", "|---|---|---|---|---|"]
        for case, name in CASE_NAMES.items():
            a1, a10, c = rows.get((case, "abi1")), rows.get((case, "abi10")), rows.get((case, "core"))
            if not a1:
                continue
            out.append(f"| {name} | {_mi(a1['ns_per_step'], '.0f')} | {_mi(a1['real_time_factor'], '.0f')} | "
                       f"{_mi(a10['ns_per_step'], '.0f') if a10 else '-'} | "
                       f"{_mi(c['ns_per_step'], '.0f') if c else '-'} |")
        out.append("")
    if "native_unity_feature_ablation" in d:
        out += ["| L2 on road ground, Unity feature on | rc_twin, n = 1 [ns] |", "|---|---|"]
        for r in d["native_unity_feature_ablation"]:
            out.append(f"| {r['features']} | {_mi(r['ns_per_step'], '.0f')} |")
        out.append("")
    if "python" in d:
        out += ["| Python environment | env steps/s | episode wall [s] | episode sim [s] | "
                "real-time factor |", "|---|---|---|---|---|"]
        for r in d["python"]:
            out.append(f"| {r['env']} | {_mi(r['env_steps_per_s'], '.0f')} | "
                       f"{_mi(r['episode_wall_s'], '.3f')} | {_mi(r['episode_sim_s'], '.1f')} | "
                       f"{_mi(r['episode_real_time_factor'], '.0f')} |")
        out.append("")
    if "edge_profiler" in d:
        e = d["edge_profiler"]
        out += [f"Edge profiler, {e['edges']} edges x {e['combinations_per_edge']} "
                f"({e['sweep']}) = {e['entries']} entries: {_mi(e['wall_s'], '.2f')} s wall, "
                f"{_mi(e['ms_per_entry'], '.2f')} ms per entry"
                + (f", {e['simulated_driving_s']:.0f} s simulated, real-time factor "
                   f"{_mi(e['real_time_factor'], '.0f')}." if "real_time_factor" in e else "."), ""]
    return "\n".join(out)


# --------------------------------------------------------------------- main

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--out", type=pathlib.Path, default=ROOT / "results" / "reference" / "runtime.json")
    ap.add_argument("--build", type=pathlib.Path, default=ROOT / "build")
    ap.add_argument("--steps", type=int, default=200_000, help="Core steps per native run")
    ap.add_argument("--reps", type=int, default=9, help="native runs per case")
    ap.add_argument("--episodes", type=int, default=20, help="episodes per Python round")
    ap.add_argument("--py-reps", type=int, default=5, help="Python rounds per env")
    ap.add_argument("--profiler-reps", type=int, default=9)
    ap.add_argument("--idle-threshold", type=float, default=0.10,
                    help="CPU busy fraction, all logical CPUs, counted as idle")
    ap.add_argument("--idle-timeout", type=float, default=1800.0, help="longest wait for idle [s]")
    ap.add_argument("--retries", type=int, default=3)
    ap.add_argument("--only", choices=["native", "python", "profiler"], nargs="+")
    ap.add_argument("--quick", action="store_true", help="tiny smoke run; writes nothing")
    ap.add_argument("--report", type=pathlib.Path, metavar="JSON",
                    help="print Markdown tables from an existing results file and exit")
    args = ap.parse_args(argv)
    if args.report:
        print(report(json.loads(args.report.read_text(encoding="utf-8"))))
        return 0
    if args.quick:
        args.steps, args.reps, args.episodes, args.py_reps, args.profiler_reps = 20_000, 2, 2, 1, 1
        args.idle_threshold, args.idle_timeout, args.retries = 1.0, 0.0, 0
    if os.name != "nt":
        print("bench: Windows only", file=sys.stderr)
        return 1
    exe = args.build / "bin" / "bench_step.exe"
    only = set(args.only or ["native", "python", "profiler"])
    if "native" in only and not exe.exists():
        print(f"bench: {exe} missing; configure with -DRC_TWIN_BENCH=ON and build bench_step",
              file=sys.stderr)
        return 1

    def log(msg):
        print(msg, file=sys.stderr, flush=True)

    envmod = _load("rc_twin_env", ROOT / "sim_py" / "rc_twin_env.py")
    campus = _load("campus", ROOT / "sim_py" / "campus.py")
    started = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    result = {"schema": "rc-physics-twin runtime benchmark 1", "started": started,
              "machine": hardware(args.build),
              "settings": {k: getattr(args, k) for k in ("steps", "reps", "episodes", "py_reps",
                                                         "profiler_reps", "idle_threshold",
                                                         "idle_timeout", "retries")},
              "load_metric": "1 - idle/total over all logical CPUs (GetSystemTimes); "
                             "load_others_during subtracts this benchmark's one thread"}
    if "native" in only:
        with tempfile.TemporaryDirectory() as d:
            scen = pathlib.Path(d) / "scenario.txt"
            result["native_scenario"] = write_scenario(scen, envmod, campus)
            result["native"] = bench_native(args, exe, scen, log)
            result["native_unity_feature_ablation"] = bench_ablation(args, exe, scen, log)
    if "python" in only:
        result["python"] = bench_python(args, log)
    if "profiler" in only:
        result["edge_profiler"] = bench_profiler(args, log)
    result["finished"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    if args.quick:
        print(json.dumps(result, indent=2))
        return 0
    args.out.parent.mkdir(parents=True, exist_ok=True)
    if args.out.exists() and args.only:  # merge a partial rerun into the existing file
        old = json.loads(args.out.read_text(encoding="utf-8"))
        rerun = {k: v for k, v in result.items() if k not in ("started", "finished")}
        rerun.setdefault("section_times", {})
        old.setdefault("section_times", {})
        for k in only:
            old["section_times"][k] = [result["started"], result["finished"]]
        rerun.pop("section_times")
        old.update(rerun)
        result = old
    args.out.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    log(f"bench: wrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
