# rc-physics-twin

High-fidelity, physics-first digital twin of a 1/10-scale electric Ackermann RC car, built to plug into a fleet-management simulation.

> **Status:** Phase 1 (foundation). SlipX is vendored in `core/`, the `rc_twin` C ABI builds and passes its tests, and the Unity project drives the car on flat ground (Edit and Play Mode tests pass in the editor). See [`Docs/NEXT_STEPS.md`](Docs/NEXT_STEPS.md).

## Why

Most open-source sim engines trade accuracy for speed. This project goes the other way: **slow and accurate dynamics**, then caches the results so the fleet layer stays fast.

What it models (target):

- **Vehicle dynamics** at 1 kHz: double-track chassis, per-corner load transfer, Magic-Formula-style tires with combined slip and relaxation, wheel spin state, Ackermann steering with a servo model.
- **Electric powertrain:** motor torque-speed curve, current limit, thermal derating, battery sag and state of charge, regen.
- **Road reality:** grade (slope), speed breakers, surface friction, curvature, banking.
- **Suspension:** per-corner spring/damper so bumps and breakers act on the dynamics.
- **Control:** manual (keyboard/gamepad) and RL policy through the **same** action vector.
- **RL adaptation:** domain randomization (friction, mass, motor lag, tire stiffness) plus a short observation history so the policy adapts online.
- **Fleet coupling:** an *edge profiler* runs the high-fidelity sim once per road edge and caches traversal time and energy as weights for a fast, deterministic fleet sim (demand projection, reassignment, energy-aware dispatch).

## Architecture in one picture

```
                    +--------------------------+
  Unity (renderer,  |  ground-query callback   |   height, normal, friction per wheel
  road tiles, UI) <-+  (engine-agnostic seam)  +<-------------------------------+
                    +------------+-------------+                                |
                                 |                                              |
                          vehicle state / inputs                                |
                                 v                                              |
                    +--------------------------+     +-----------------------+  |
                    |  core/ (forked SlipX)    |<--->| gymkhana oracle (py)  |  |
                    |  C++17, deterministic    |     | cross-validation, RL  |  |
                    +------------+-------------+     +-----------------------+  |
                                 | edge profiles (time, energy)                 |
                                 v                                              |
                    +--------------------------+                                |
                    |  fleet sim (separate     |--------------------------------+
                    |  repo: campus_fleet_*)   |
                    +--------------------------+
```

## Build and run (Windows)

No admin rights or MSVC needed: the bootstrap puts Zig (Clang 21), CMake and Ninja in `.toolchain/`.

```bash
python tools/bootstrap_toolchain.py
```

```bash
.toolchain/venv/Scripts/cmake -S . -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=.toolchain/zig-toolchain.cmake
```

```bash
.toolchain/venv/Scripts/cmake --build build
```

```bash
.toolchain/venv/Scripts/ctest --test-dir build --output-on-failure
```

The vendored Core's own tests and the determinism check (the six conformance hashes this toolchain must reproduce, `conformance/reference_hashes.tsv`); CI runs all of this on every push (`.github/workflows/ci.yml`):

```bash
.toolchain/venv/Scripts/cmake -S core -B build-core -G Ninja -DCMAKE_TOOLCHAIN_FILE="$PWD/.toolchain/zig-toolchain.cmake" -DSLIPX_BUILD_TESTS=ON -DSLIPX_BUILD_PYTHON=OFF
```

```bash
python tools/check_determinism.py --build-dir build-core --require-row
```

Then copy the plugin into Unity (close the editor first; it keeps native plugins loaded):

```bash
.toolchain/venv/Scripts/cmake --build build --target unity_plugin
```

Drive a road file headlessly (no Unity), with a JSON report of time, energy and peak accelerations:

```bash
python tools/drive_road.py roads/samples/hill_breaker_bend.json --speed 2
```

Train on the campus: `sim_py/campus.py` runs the RL environment on stop-to-stop trips over the fleet sim's own road network (`campus/README.md`).

Profile road edges for the fleet sim (time, energy, breaker penalty per speed, payload and charge; reruns reuse what still matches):

```bash
python tools/edge_profiler.py roads/samples/*.json --out profiles/cache.json --speeds 1 2 3 --payloads 0 1 --soc 0.8 0.4 --breaker-speed 1
```

Open `unity/RcTwin` in Unity Hub with **6000.5.3f1**, run **RcTwin > Build Flat Ground Scene**, and press Play. W/S throttle and brake, A/D steer, R reset. **RcTwin > Build Campus Scene** builds the campus fleet sim's roads at 1/10 instead, the car at the S3 stop. The C# bindings can also be checked without Unity: `native/tests/csharp` (`dotnet run -- build/bin`).

**Autopilot.** In the campus scene, the panel at the top right drives the car to any stop you pick. It starts from rest at the stop it is at and stops at the destination, on the twin's own physics, along the route the fleet sim and the RL environment use. Holding a drive key takes the car back.

Where the route leaves the stop the way the car came in, the car first turns round, driven: a planner (`TurnAround.cs`) finds a multi-point turn of forward and reverse arcs within the steer limit, inside the road (the campus kerbs and buildings as walls, and the network's road width), and the car drives it leg by leg before the trip starts at rest. The panel shows each leg as it is driven, then the turn it made ("Turned round: three-point turn: forward 0.15 m, reverse 0.45 m, forward 0.90 m"). Every turn-round on the campus has a plan, checked in Edit Mode. If no turn can be planned, the car is turned at the stop instead and the panel says "not driven". Reversing is the ESC's: hold a negative throttle at rest and it engages (ADR 0010). **RcTwin > Build Campus Player** builds the campus scene as `build/campus/RcTwinCampus.exe`; `-goto AB1 -speed 3` starts a trip from the command line.

The autopilot's driver is an `IDriver` (`unity/RcTwin/Assets/RcTwin/Runtime/Drivers.cs`). Each control tick, the driver gets the RL environment's own `"stop"`-task observation (`sim_py/README.md`), computed in C# by `TwinObservation` on the native road. A trip ends as that task's episodes do: stopped within 0.3 m of the destination, or failed. Two drivers come with it, and the panel switches between them:

- the tuned pure pursuit, with its stop plan;
- a learned policy (`PolicyDriver`), loaded from a `tools/export_policy.py` export.

To add a controller, write another `IDriver`. Edit Mode tests check the C# side against drives recorded in Python (`tools/make_autopilot_fixture.py`): the observation, the pure pursuit and the campus routes match to 1e-9 in both RL tasks. A learned policy's actions match its recorded checks to 1e-4.

**RcTwin > Build Replay Scene** turns the campus into a replay of trained policies: pick any two stops, and every policy recorded by `tools/record_drives.py` drives that route with the same car at once, one coloured car each, with every route's outcome per policy in a grid. **RcTwin > Build Replay Player** builds it as `build/replay/RcTwinReplay.exe`, with `results/drives.json` beside it. See `sim_py/README.md`.

## Layout

```
core/            SlipX vendored at ebdbf3f (C++17); our changes listed in THIRD_PARTY.md
native/          rc_twin: C ABI over the Core (rc_twin.dll), with C, C++ and C# tests
unity/RcTwin/    Unity 6 project: P/Invoke bindings, keyboard car, scene builders (test ground; the campus)
tools/           toolchain bootstrap; drive_road.py, the headless harness; edge_profiler.py; campus_import.py
profiles/        edge profile cache schema (the fleet sim's input)
sim_py/          RL environment (rc_twin_env.py) and its campus trips (campus.py); later gymkhana, SB3 training
campus/          the campus fleet sim's road network at 1/10 (export, converted network)
cars/            versioned car directories (YAML), every set labeled measured|identified|provisional
roads/           road-feature schema, sample roads, the campus edges (roads/campus/)
Docs/            specs, ADRs, roadmap, references
third_party/     vendored/forked upstream code with licenses preserved
```

## Upstream projects and licenses

| Project | License | Role |
|---|---|---|
| [SlipX](https://github.com/ibrahimsel/slipx) | Apache-2.0 | Dynamics core (fork) |
| [Gym-Khana](https://github.com/TeoIlie/Gym-Khana) | MIT | Validation oracle, RL harness (fork or dependency) |
| [ApexSim](https://github.com/pzimbrod/apexsim) | Apache-2.0 | Reference only (track CSV with elevation and banking) |
| [TLabVehiclePhysics](https://github.com/tlabaltoh/tlabvehiclephysics) | MIT (per directory listing) | Reference only |

SlipX's licence was checked against its `LICENSE` and `NOTICE` files at the pinned commit. The others still come from PyPI metadata and GitHub badges: **verify each `LICENSE` before vendoring code.** See [`THIRD_PARTY.md`](THIRD_PARTY.md).

## License

Apache-2.0. See [`LICENSE`](LICENSE) and [`NOTICE`](NOTICE).

## Honesty policy

No parameter set in this repo is claimed as validated unless it carries a `measured` or `identified` provenance label with a validation report. Defaults are `provisional`.
