# Next Steps

## Decisions needed before Phase 1

- [x] Sign off ADR 0003 (Unity instead of Isaac Sim for the twin). Accepted 2026-10-19.
- [x] Choose the target car: generic 1/10 electric Ackermann, `provisional`, until real numbers come from `hardware-bom.md`.
- [x] Fork `ibrahimsel/slipx` on GitHub and decide: vendor into `core/` or git submodule. Forked ([Sujay-197/slipx](https://github.com/Sujay-197/slipx)); **vendored** into `core/` at `ebdbf3f`.
- [ ] Verify upstream `LICENSE` files (SlipX, Gym-Khana) and fill `THIRD_PARTY.md`. SlipX done 2026-10-19; Gym-Khana still open.
- [x] Confirm Unity version and whether it can load a C++ native plugin on the dev machine (Windows). Unity 6000.5.3f1 loads `rc_twin.dll`; Edit and Play Mode suites run in batch mode.
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
- [x] Per-wheel height differences (twist) and speed breakers: suspension, ADR 0008. Kerbs are walls, not heights: ADR 0009.

## Phase 3: Road

- [x] Suspension (spring/damper per corner). ADR 0008: opt-in at L2 from measured wheel rates and damping (L3 stays SlipX's); body heave, pitch and roll; per-wheel ground heights act; bump stop. The Unity twin runs DEMO springs (3 Hz, 0.3 damping, 15 mm travel) and has a speed breaker. Replace the demo values with bench measurements.
- [x] Road-feature schema: elevation, banking, speed breakers, surface. `roads/road.schema.json` (0.1.0), two sample roads, and the road ground model in `native/src/rc_twin_road.cpp`.
- [x] The campus in the twin. `RoadGraph`, stops, road tiles and buildings exported from `campus_fleet_man_unity` (b745220) and converted at 1/10 (`campus/`, `tools/campus_import.py`): 37 edges as road files (`roads/campus/`), the network in `campus/campus_network.json`, and **RcTwin > Build Campus Scene** in Unity (asphalt the car drives on, kerbs, buildings, stops; starts at S3).
- [x] The real campus around the roads: buildings, land use and side roads from OpenStreetMap (ODbL), fitted to the fleet sim's schematic network by a similarity and a thin-plate-spline warp. Trees and ground cover come from local map screenshots. Drawn only (`campus/README.md`, `tools/campus_scenery.py`).
- [x] Watch policies drive: the replay player (`build/replay/RcTwinReplay.exe`, `tools/record_drives.py --pairs`) shows every recorded policy on any stop-to-stop route.
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
- [x] Trained policies on the campus (30 unseen trips each, target 2 m/s):
  - **CEM, linear, standard library only** (`train_cem.py`, 3 min): 100 % success under both training and held-out randomisation, no wall contacts, 1.88 m/s, 6 cm mean offset.
  - **PPO with history** (`train_ppo.py`, 30M steps, 32 min across 14 CPU processes and the GPU): 100 % under training randomisation (2.26 m/s, 8 cm), but 97 % and 11 trips touching a wall under held-out randomisation. In this budget it does not beat the linear policy; history-conditioned adaptation is still to be earned (longer training, tuning, or a recurrent policy).
  - Pure pursuit still scores highest (it gets the car's wheelbase and steer limit built in).
  - Training also exposed three loopholes in the reward, now closed and tested: speeding, crawling, and leaving the road on purpose.
- [x] Held-out-parameter and perturbation evaluation: the policy study, [`policy-study.md`](policy-study.md). It compares tuned pure pursuit, Stanley and LQR with CEM, PPO and SAC with history, and residual PPO and SAC, at 4 m/s on 100 trips in each of three conditions, with seeds, validation-selected checkpoints and paired tests. The results:
  - Learning from scratch finishes a few more trips with unfamiliar cars, but drives three to four times further off the line.
  - Residual RL stays with pure pursuit.
  - The tuned pure pursuit drives the twin by default.
- [x] A stop-to-stop task (`task="stop"`): start at rest, finish stopped within 0.3 m of the destination. The study traced most held-out failures to episodes starting at 4 m/s just before a junction.
- [x] The twin drives itself: the campus autopilot picks the route, drives it and stops at the destination, through `IDriver` (pure pursuit, or a learned policy exported by `tools/export_policy.py`). C# and Python are checked against each other in tests. The player is `build/campus/RcTwinCampus.exe`.
- [x] Three-point turns. A route that leaves a stop the way the car came in starts with a driven turn: `TurnPlanner` (hybrid A* over forward and reverse arcs at up to 85 % of the steer limit, the footprint 4 cm clear of the campus walls and inside the road width) plans it off the main thread, and `TurnDriver` drives it at 0.7 m/s, replanning a leg that ends off its plan. Then the trip starts at rest, as the RL "stop" task's episodes do. All 23 distinct turn-rounds on the campus plan (Edit Mode), and each drives on the Core with no kerb contact (headless sweep, 3 to 8 s per turn). Play Mode drives S3 to AB2 with no reset. The teleport is kept only for when planning fails, and the panel says so. Reversing needed the ESC's reverse and steering in reverse in the Core: ADR 0010.
- [x] Match the RL environment's car to the Unity twin's: `car="unity"` (wheel inertia, Ackermann, DEMO suspension) or `"both"`, recorded with each model and checked on load. On the Unity car the stop task is much harder: the tuned pure pursuit finishes 59 % of held-out trips, against 92 % on the bare car. [`policy-study.md`](policy-study.md#closing-the-gap-to-the-unity-car).
- [x] The Unity autopilot on the Unity car: pure pursuit gains tuned for it (`policies/controllers_unity_stop.json`), and `KerbRecovery`, which backs off a kerb the car is pinned against and retries. On the stop task with held-out cars the autopilot now finishes 82 % of trips (98 / 82 / 86 % train / held-out / perturbed), against 61 % with the study's gains alone; the back-off reverses with the Core's ESC reverse (ADR 0010). [`policy-study.md`](policy-study.md#the-autopilot-on-the-unity-car).
- [x] A corner speed plan for the autopilot: speed planned along the whole route at the trip's start, so the car slows for each junction before it comes into view, and pulls away from each junction and from the stop gently (`CornerSpeed`, Python and C#). On the Unity car, on fresh trips, the autopilot now finishes every stop-task trip at 1 to 4 m/s with training cars, held-out cars and the mid-trip change, without touching a kerb; at 4 m/s it had finished 91 % and 87 % with held-out cars and the mid-trip change. Trips at 4 m/s take a quarter longer. Its gains are set by design, like the stop plan's. [`policy-study.md`](policy-study.md#a-corner-speed-plan).
- [x] The campus shuttle game: students request rides between stops, and the twin's car, as a white campus shuttle, serves them by itself. It pools students up to six seats, stops on its way, charges at AB1 and keeps a score. It is the campus player's default (`-goto` keeps the one-trip demo). [`specs/2026-10-21-campus-shuttle-design.md`](specs/2026-10-21-campus-shuttle-design.md), plan in [`plans/2026-10-21-campus-shuttle.md`](plans/2026-10-21-campus-shuttle.md).
- [ ] Several shuttles. The dispatcher already asks every shuttle for its insertion cost; what is left is spawning more cars, a second charger, and keeping shuttles from blocking each other at a stop.
- [ ] A charger chosen by the user, and more than one.
- [x] A low-speed model. Below `v_eps` (0.5 m/s) L2's floored slip angles turned the car at full lock as if it were doing `v_eps`, up to five times tighter than its geometry, and erratically in reverse. `VehicleParams::v_kinematic` now blends L2 into a kinematic single-track car below `v_eps`, as AMZ Driverless does (ADR 0011); the Unity car is kinematic at and below 0.3 m/s. It creeps round its geometry forwards and backwards and stops turning when it stops. Every campus turn-round drives clean at 0.2 to 0.7 m/s, and at 0.7 m/s ends within 1.9 cm and 4.9 degrees of its plan (4.8 cm and 11.7 degrees before). The shipped autopilot's stop-task results are unchanged (every trip, both trip sets). rc_twin ABI v8. [`adr/0011-kinematic-blend-at-low-speed.md`](adr/0011-kinematic-blend-at-low-speed.md).
- [x] A stop-task reward that keeps a tidy line without costing robustness (v3): offset from a drivable line that rounds the junction corners, and kerb contact charged per tick. On the Unity-car gains the residual now ties the autopilot (82 to 86 % held-out against 82 %) instead of losing to it. [`policy-study.md`](policy-study.md#a-second-round-on-the-unity-car-a-drivable-line-and-kerb-contact).
- [ ] A learned driver that beats the autopilot. The residual drifts after 1 to 3M steps and validation, on training cars, keeps its earliest checkpoint; PPO + history finishes 85 % of held-out trips but stops short of the destination with heavier, weaker cars. Validation on the hardest training cars (`--val-edge`) was tried and made the residual worse: the controller finishes nearly all of those cars, so the set cannot rank checkpoints ([`policy-study.md`](policy-study.md#a-third-round-selecting-on-the-hardest-training-cars)). Next: a creep-to-the-stop term or more budget and seeds for PPO + history; wider training ranges; tune the learners' hyperparameters, and give RecurrentPPO a fair, tuned run.

## Phase 6: Empirical (optional)

- [ ] Coast-down, skidpad and acceleration tests on the real RC car.
- [ ] Identify parameters; promote labels from `provisional` to `identified`.

## Known issues

- [x] **Vertical faces in the scene (ramp sides, plateau edges) were not obstacles.** Fixed by ADR 0009: walls are contact segments resolved after every Core step with SlipX's `segment_contact` (ABI v7, `rc_twin_add_wall`), and `RaycastGround` climbs only steps under half a wheel radius. The flat-ground scene walls the hill's sides and the bank; the campus walls its kerb lines and buildings. Reported 2026-10-19, fixed 2026-10-20.
- [x] Walls in the RL environment. `RcTwinEnv(walls=...)` gives the car walls and charges each strike (`REWARD["wall"]`, speed lost to the wall); `CampusEnv` uses the campus's kerbs and buildings by default, so a kerb strike bounces and costs reward instead of ending the episode. The kerb lines and sidewalk runs are computed once by `tools/campus_import.py` and shared with the Unity scene.
- [ ] Walls are planar and infinitely tall (ADR 0009): a car on the plateau is held in by the plateau's own side walls. A wall with a height range, and walls extracted from arbitrary colliders, are follow-ups.

## Housekeeping

- [x] CI: build, unit tests, determinism hash check per build. `.github/workflows/ci.yml` on the reference toolchain: rc_twin and its suites, the C# bindings, every SlipX layer's tests, and `tools/check_determinism.py` against `conformance/reference_hashes.tsv`. Unity's suites need a licensed editor and stay local.
- [ ] Decide public/private and set GitHub repo description and topics.
