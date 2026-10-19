# Next Steps

## Decisions needed before Phase 1

- [x] Sign off ADR 0003 (Unity instead of Isaac Sim for the twin). Accepted 2026-10-19.
- [x] Choose the target car: generic 1/10 electric Ackermann, `provisional`, until real numbers come from `hardware-bom.md`.
- [x] Fork `ibrahimsel/slipx` on GitHub and decide: vendor into `core/` or git submodule. Forked ([Sujay-197/slipx](https://github.com/Sujay-197/slipx)); **vendored** into `core/` at `ebdbf3f`.
- [ ] Verify upstream `LICENSE` files (SlipX, Gym-Khana) and fill `THIRD_PARTY.md`. SlipX done 2026-10-19; Gym-Khana still open.
- [ ] Confirm Unity version and whether it can load a C++ native plugin on the dev machine (Windows).
- [ ] Open `rc_transit/docs/hardware-bom.md` in the fleet repo; extract any real mass, wheelbase, motor, battery numbers.

## Phase 1: Foundation

- [x] Fork SlipX; pin a commit; build `slipx_core` standalone; run `slipx-conformance`. Pinned `ebdbf3f`; built and run with zig/Clang 21, see [`slipx-windows-build.md`](slipx-windows-build.md).
- [ ] Repeat that build with MSVC once the Windows SDK is installed, including the Python bindings and the `slipx-conformance` console script.
- [x] Wrap Core in a C ABI (`step`, `reset`, `set_params`, `get_state`). `native/`: `rc_twin.dll`, tested bit-identical against the Core. The ground-query callback moves to Phase 2, where the Core change that consumes it lands.
- [x] Unity: load the plugin, drive the reference car on flat ground with keyboard input. `unity/RcTwin`, scene `Assets/Scenes/FlatGround.unity`. Verified in Unity 6000.5.3f1 batch mode on 2026-10-19: 5/5 Edit Mode tests (plugin load, defaults, driving, frame mapping) and 2/2 Play Mode tests (simulated W then A drives forward then turns left on the real transform; R resets to rest).
- [x] Headless harness implementing the same ground query (for CI and the profiler). `tools/drive_road.py`: a road file becomes the ground through `rc_twin_road_*` (ABI 6); pure-pursuit and speed control; JSON report. Its tests run in ctest (`tools.drive_road`).

## Phase 2: Vehicle

- [x] Per-patch ground query (height, normal, friction scale). ADR 0005: the host samples the ground under each wheel per step and the Core takes the samples as arguments (`VehicleModel::step_on`), so `step` stays pure. L2 feels grade, banking and per-wheel grip; L0/L1 refuse anything but level ground. Level ground is bit-identical to before (216 SlipX core tests, 6 conformance hashes unchanged).
- [x] Ground-query callback in the C ABI (`rc_twin_set_ground_query`, ABI v2), and `RaycastGround` + `GroundSurface` in Unity. The FlatGround scene has a hill, a banked strip and an ice patch; Play Mode tests drive all three.
- [x] Ackermann steering geometry. ADR 0007: `ackermann` fraction of ideal (0 parallel, the default; 1 ideal; negative anti), per-wheel front angles everywhere the steer enters at L2. Measure it on the car: steer to a known angle, read both front wheels.
- [x] Per-wheel spin state (lock-up and wheelspin representable). ADR 0006: opt-in `wheel_inertia`; chassis holds each wheel's slip over the step, wheels advance by a closed-form implicit solve coupled through the motor curve; motor braking locks rather than reverses. Inertia 0 is SlipX's L2 bit for bit.
- [ ] Per-wheel height differences (twist, kerbs) and speed breakers need suspension: Phase 3 / L3.

## Phase 3: Road

- [x] Suspension (spring/damper per corner). ADR 0008: opt-in at L2 from measured wheel rates and damping (L3 stays SlipX's); body heave, pitch and roll; per-wheel ground heights act; bump stop. The Unity twin runs DEMO springs (3 Hz, 0.3 damping, 15 mm travel) and has a speed breaker. Replace the demo values with bench measurements.
- [x] Road-feature schema: elevation, banking, speed breakers, surface. `roads/road.schema.json` (0.1.0), two sample roads, and the road ground model in `native/src/rc_twin_road.cpp`.
- [x] The campus in the twin. `RoadGraph`, stops, road tiles and buildings exported from `campus_fleet_man_unity` (b745220) and converted at 1/10 (`campus/`, `tools/campus_import.py`): 37 edges as road files (`roads/campus/`), the network in `campus/campus_network.json`, and **RcTwin > Build Campus Scene** in Unity (asphalt the car drives on, kerbs, buildings, stops; starts at S3).
- [ ] Elevation and features in the fleet sim's `RoadGraph` itself (the campus is flat today; the polylines carry z, so grades come through the importer once the fleet sim has them).
- [ ] Validate against gymkhana (planar) plus coast-down and skidpad tests.

## Phase 4: Fleet coupling

- [x] Edge profiler and cache format. `tools/edge_profiler.py` sweeps speed x payload x charge per edge under the reference driver and writes `profiles/edge_profile.schema.json` caches with road, car and controller digests; reruns reuse what still matches. Tests in ctest (`tools.edge_profiler`).
- [ ] Hook cache into fleet sim edge costs (fleet items C, D). The campus edges are now road files: `python tools/edge_profiler.py roads/campus/*.json` makes their cache; the fleet sim still has to read it (scale times and energies back up from 1/10, or run its buses as 1/10 cars).
- [ ] Energy-aware dispatch; charging constraints.
- [ ] Demand forecasting and request reassignment (fleet items A, B), re-evaluated with physical costs.

## Phase 5: Learning

- [x] Gymnasium env around the Core. `sim_py/rc_twin_env.py`: road following at a target speed, standard library only, optional `gymnasium.Env` wrapper; tests in ctest (`sim_py.env`).
- [x] Domain randomization and history-conditioned observations (friction, payload, servo lag, motor torque, tyre stiffness, battery resistance; seeded). A trained history-conditioned policy is still to do: needs gymnasium, numpy and SB3 installed.
- [x] Train where the fleet runs. `sim_py/campus.py`: `CampusEnv` drives stop-to-stop trips on the campus network, drawn by demand weight and routed by shortest path as the fleet sim does; tests in ctest (`sim_py.campus`).
- [ ] Held-out-parameter and perturbation evaluation. The tools are in place (`RANGES_HELD_OUT`, mid-episode `perturb`, `evaluate`, a pure-pursuit baseline); evaluating a trained policy waits on training.

## Phase 6: Empirical (optional)

- [ ] Coast-down, skidpad and acceleration tests on the real RC car.
- [ ] Identify parameters; promote labels from `provisional` to `identified`.

## Known issues

- [x] **Vertical faces in the scene (ramp sides, plateau edges) were not obstacles.** Fixed by ADR 0009: walls are contact segments resolved after every Core step with SlipX's `segment_contact` (ABI v7, `rc_twin_add_wall`), and `RaycastGround` climbs only steps under half a wheel radius. The flat-ground scene walls the hill's sides and the bank; the campus walls its kerb lines and buildings. Reported 2026-10-19, fixed 2026-10-20.
- [x] Walls in the RL environment. `RcTwinEnv(walls=...)` gives the car walls and charges each strike (`REWARD["wall"]`, speed lost to the wall); `CampusEnv` uses the campus's kerbs and buildings by default, so a kerb strike bounces and costs reward instead of ending the episode. The kerb lines and sidewalk runs are computed once by `tools/campus_import.py` and shared with the Unity scene.
- [ ] Walls are planar and infinitely tall (ADR 0009): a car on the plateau is held in by the plateau's own side walls. A wall with a height range, and walls extracted from arbitrary colliders, are follow-ups.

## Housekeeping

- [ ] CI: build, unit tests, determinism hash check per build.
- [ ] Decide public/private and set GitHub repo description and topics.
