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

Then copy the plugin into Unity (close the editor first; it keeps native plugins loaded):

```bash
.toolchain/venv/Scripts/cmake --build build --target unity_plugin
```

Open `unity/RcTwin` in Unity Hub with **6000.5.3f1**, run **RcTwin > Build Flat Ground Scene**, and press Play. W/S throttle and brake, A/D steer, R reset. The C# bindings can also be checked without Unity: `native/tests/csharp` (`dotnet run -- build/bin`).

## Layout

```
core/            SlipX vendored at ebdbf3f (C++17); our changes listed in THIRD_PARTY.md
native/          rc_twin: C ABI over the Core (rc_twin.dll), with C, C++ and C# tests
unity/RcTwin/    Unity 6 project: P/Invoke bindings, keyboard car, flat-ground scene builder
sim_py/          Python bindings, gymkhana cross-validation, SB3 training
cars/            versioned car directories (YAML), every set labeled measured|identified|provisional
roads/           road-feature schema and sample data (grade, speed breakers, friction)
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
