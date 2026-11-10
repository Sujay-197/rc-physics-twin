# Runtime performance

How fast the twin runs headless, single-threaded, on one desktop machine. The numbers are in [`results/reference/runtime.json`](../results/reference/runtime.json), every sample included, and [`tools/bench.py`](../tools/bench.py) reproduces them.

**Short answer.** One 1 ms Core step costs about 0.1 µs at L0, 0.18 µs at L1 and 1.2 µs at L2. At L2 the car runs about 800 times faster than real time, and about 730 times with the Unity car's features and the campus walls. The C ABI adds about 25 ns per call, 2 % at L2. Python environments manage 31,000 to 40,000 steps a second (100 Hz control), 300 to 390 times real time. The edge profiler builds all 444 cache entries for the 37 campus edges in 1.6 s.

Every value below is the median, with the interquartile range in brackets.

## Machine and build

| | |
|---|---|
| CPU | AMD Ryzen 7 9700X, 8 cores, 16 threads, 3.8 GHz base |
| RAM | 31.1 GiB usable |
| GPU | NVIDIA GeForce RTX 5060 Ti (not used by these measurements) |
| OS | Windows 11 Pro 10.0.26300, power scheme Balanced |
| Compiler | Zig 0.16.0 (Clang 21.1.0), `-target x86_64-windows-gnu -mcpu=x86_64_v2` (`tools/bootstrap_toolchain.py`) |
| Build | CMake `Release`: `-O3 -DNDEBUG -std=c++17 -ffp-contract=off`; `rc_twin` also `-fvisibility=hidden` |
| Python | CPython 3.14.6 |

The build targets `x86_64_v2`, not the host CPU. That gives up whatever AVX2 or AVX-512 would have bought, and keeps the binary, and so its results, the same on any x86-64-v2 machine (SlipX's determinism rules forbid `-march=native`).

**Conditions.** Two other experiments were running on the same machine. Before every measurement, `bench.py` waited until CPU load over all 16 logical CPUs was under 10 %. Afterwards it subtracted its own thread from the load measured during the run, and retried any run where the rest was over 10 %. Every reported run met that, one of them (L0, Core direct) after a retry. Load from other processes during the runs had a median of 9 % in the native set, 7 % for the edge profiler and 3 to 6 % in the Python set, so about one to one and a half other threads were busy. Measured on 2026-11-09: Python first, then the native cases and the edge profiler in a second pass (`section_times` in the JSON).

## Core step cost

Each run is 200,000 steps (200 s simulated) at 1 kHz, RK4, SlipX's provisional car, nine runs per case after a warm-up run. Every case drives the same closed loop: pure pursuit (0.6 m lookahead) with a speed hold at 2 m/s, updated every 10 steps from the state, along the longest stop-to-stop campus route (AB3 to S4, 69 m). The car goes back to the start at the end of the route (5 times a run). The controller and the restarts are inside the timed loop and cost a few ns per step. The real-time factor is simulated seconds per wall second: 10⁶ / (ns per step).

| Case | rc_twin, n = 1 [ns/step] | Real-time factor | rc_twin, n = 10 [ns/step] | Core direct [ns/step] |
|---|---|---|---|---|
| L0 kinematic, level ground | 99 (98 to 102) | 10,100 (9,840 to 10,200) | 84 (83 to 87) | 82 (82 to 82) |
| L1 bicycle, level ground | 175 (175 to 177) | 5,700 (5,640 to 5,710) | 167 (166 to 168) | 164 (163 to 167) |
| L2 double track, level ground | 1,214 (1,209 to 1,247) | 824 (802 to 827) | 1,256 (1,222 to 1,275) | 1,188 (1,184 to 1,189) |
| L2 on road ground | 1,484 (1,480 to 1,521) | 674 (657 to 675) | 1,493 (1,473 to 1,537) | |
| L2, Unity car, road ground | 1,234 (1,231 to 1,238) | 810 (808 to 813) | 1,238 (1,226 to 1,259) | |
| L2, Unity car, road ground, campus walls | 1,372 (1,361 to 1,378) | 729 (726 to 735) | 1,321 (1,316 to 1,324) | |

- **n = 1** calls `rc_twin_step` once per step. **n = 10** calls it once per 10 ms control tick, as `sim_py` does. **Core direct** calls `slipx::VehicleModel::step` with no C ABI in between. It runs on level ground only, since the road's ground query belongs to `rc_twin`.
- **Road ground** is `rc_twin_road_ground` on the route's own 28-point centreline. It finds the nearest centreline segment by scanning all of them, once per wheel per step. That adds about 270 ns per step here, and the cost grows with the length of the road. In a trial run (under load, not in the results file), the same route resampled to 5 cm, 1,397 points, cost about 14 µs per step.
- **The Unity car** is `TwinCar.cs`'s set-up: wheel inertia 10⁻⁵ kg m², ideal Ackermann, suspension at 3 Hz and damping ratio 0.3 with 15 mm travel, kinematic at and below 0.3 m/s.
- **Campus walls** are the 18 kerb and building outlines of `campus/campus_network.json`, up to 176 segments, with the footprint widened by 17.5 mm as in Unity. The pure pursuit kept the car off the kerbs, so the walls were never touched. The extra 80 to 140 ns per step is the bounding-box test against every segment. A step with a contact costs more.

**The C ABI's overhead.** At n = 1, `rc_twin` costs about 17 ns per call at L0 and 26 ns at L2: 2 % of an L2 step. Batching ten steps per call brings L0 and L1 within 2 to 3 ns per step of the Core called directly. At L2 the n = 1 and n = 10 columns are within each other's noise. The per-call cost is the argument checks, copying the state in and out (so a failed step changes nothing), and the reverse latch.

**Why the Unity car is faster than the plain car.** With no suspension, every derivative of every RK4 stage evaluates the tyre forces twice: once at the static loads, then at the load transfer those first accelerations imply. With suspension on, the spring loads are held for the step, so one pass is enough (`l2_double_track.cpp`, ADR 0008). Switching the Unity features on one at a time shows it (L2 on road ground, n = 1):

| Feature on | ns/step |
|---|---|
| none | 1,505 (1,491 to 1,511) |
| wheel inertia | 1,543 (1,542 to 1,584) |
| Ackermann | 1,591 (1,579 to 1,611) |
| suspension | 1,088 (1,086 to 1,089) |
| kinematic blend | 1,482 (1,481 to 1,483) |
| all of them | 1,217 (1,215 to 1,218) |

## Python environments

`rc_twin_env.pure_pursuit_policy` drives each environment, on the training randomisation (`RANGES_TRAIN`), 100 Hz control over 10 Core steps. Each set is 20 episodes (seeds 0 to 19) repeated 5 times, after one warm-up episode. Steps per second counts only the time inside `env.step`. Episode wall time includes the reset and the policy. The real-time factor is simulated over wall time for whole episodes.

| Environment | env steps/s | Episode wall [s] | Episode simulated [s] | Real-time factor |
|---|---|---|---|---|
| `RcTwinEnv`, follow, `roads/samples/hill_breaker_bend.json` | 40,000 (40,000 to 40,100) | 0.048 (0.048 to 0.048) | 18.8 | 391 (390 to 392) |
| `CampusEnv`, follow, campus walls | 37,200 (37,000 to 37,300) | 0.035 (0.017 to 0.048) | 12.6 (6.1 to 17.2) | 362 (360 to 363) |
| `CampusEnv`, stop task, campus walls | 30,700 (30,600 to 30,800) | 0.044 (0.021 to 0.059) | 13.0 (6.6 to 17.6) | 300 (299 to 300) |

Campus episode lengths vary with the trip drawn, so the episode-time ranges are the spread of the routes, not noise. The baseline policy does not brake for the stop: every stop-task episode ended `overshot`, more than 0.3 m past it, which ends the episode. An environment step takes 25 to 33 µs, of which the 10 Core steps on road ground take about 15 µs. The rest is Python: ctypes calls, the observation and the reward.

## Edge profiler

```
python -I tools/edge_profiler.py roads/campus/*.json --out <fresh file> --speeds 1 2 3 --payloads 0 1 --soc 0.8 0.4
```

That is the sweep in `campus/README.md`: 37 edges × 3 speeds × 2 payloads × 2 charge buckets, 444 entries, all `reached_end`. Nine runs, each starting from an empty cache, timed from process start to exit:

| | |
|---|---|
| Wall time | 1.61 s (1.60 to 1.62) |
| Per entry | 3.62 ms (3.60 to 3.65) |
| Simulated driving | 905 s |
| Real-time factor | 563 (559 to 567) |

No campus edge has speed breakers, so each entry is a single drive. On a road with breakers each entry drives twice.

## What this means

The L2 Core runs about 800 times faster than real time on one thread, and still about 730 times with everything the Unity twin switches on. That is what makes the edge profiler practical: profiling the whole campus over a 12-point sweep takes under two seconds, so the fleet sim's edge costs can be recomputed after any change to the car or the roads, and the fleet sim never needs the physics in its own loop. In Python, the environment's own code takes about half of each step, so RL training is bound by Python, not by the physics.

## Not measured

- **The Unity player's frame rate** with 1, 3 and 6 buses (`-buses`). A fresh worktree has no Unity `Library` import and no `Generated/Look` assets. Building the player there means a full import, which was not worth the time for this measurement, and an import running beside the timings would have spoiled them.
- **Multi-threaded throughput.** Everything here is one thread. The Core is stateless per step (SlipX ADR-0004), so independent cars should scale across cores, but that was not measured.
- **Other machines and compilers.** One machine, one toolchain. Bit-identical results are promised within one build, and these timings are only claimed for this one.

## Reproduce

```
.toolchain/venv/Scripts/cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DRC_TWIN_BENCH=ON -DCMAKE_TOOLCHAIN_FILE="$PWD/.toolchain/zig-toolchain.cmake"
.toolchain/venv/Scripts/cmake --build build
python tools/bench.py --out results/reference/runtime.json
python tools/bench.py --report results/reference/runtime.json
```

`--quick` is a short smoke run that writes nothing, `--only native python profiler` reruns some sections and merges them into an existing file, and `--idle-threshold` sets the load counted as idle. The bench executable (`native/bench/bench_step.cpp`) builds only with `-DRC_TWIN_BENCH=ON` and is not a ctest case.
