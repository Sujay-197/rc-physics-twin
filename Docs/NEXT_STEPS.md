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
- [ ] Headless harness implementing the same ground query (for CI and the profiler).

## Phase 2: Vehicle

- [x] Per-patch ground query (height, normal, friction scale). ADR 0005: the host samples the ground under each wheel per step and the Core takes the samples as arguments (`VehicleModel::step_on`), so `step` stays pure. L2 feels grade, banking and per-wheel grip; L0/L1 refuse anything but level ground. Level ground is bit-identical to before (216 SlipX core tests, 6 conformance hashes unchanged).
- [x] Ground-query callback in the C ABI (`rc_twin_set_ground_query`, ABI v2), and `RaycastGround` + `GroundSurface` in Unity. The FlatGround scene has a hill, a banked strip and an ice patch; Play Mode tests drive all three.
- [x] Ackermann steering geometry. ADR 0007: `ackermann` fraction of ideal (0 parallel, the default; 1 ideal; negative anti), per-wheel front angles everywhere the steer enters at L2. Measure it on the car: steer to a known angle, read both front wheels.
- [x] Per-wheel spin state (lock-up and wheelspin representable). ADR 0006: opt-in `wheel_inertia`; chassis holds each wheel's slip over the step, wheels advance by a closed-form implicit solve coupled through the motor curve; motor braking locks rather than reverses. Inertia 0 is SlipX's L2 bit for bit.
- [ ] Per-wheel height differences (twist, kerbs) and speed breakers need suspension: Phase 3 / L3.

## Phase 3: Road

- [x] Suspension (spring/damper per corner). ADR 0008: opt-in at L2 from measured wheel rates and damping (L3 stays SlipX's); body heave, pitch and roll; per-wheel ground heights act; bump stop. The Unity twin runs DEMO springs (3 Hz, 0.3 damping, 15 mm travel) and has a speed breaker. Replace the demo values with bench measurements.
- [ ] Road-feature schema: elevation, banking, speed breakers, surface.
- [ ] Add elevation and features to `RoadGraph` and the Unity build tool.
- [ ] Validate against gymkhana (planar) plus coast-down and skidpad tests.

## Phase 4: Fleet coupling

- [ ] Edge profiler and cache format.
- [ ] Hook cache into fleet sim edge costs (fleet items C, D).
- [ ] Energy-aware dispatch; charging constraints.
- [ ] Demand forecasting and request reassignment (fleet items A, B), re-evaluated with physical costs.

## Phase 5: Learning

- [ ] Gymnasium env around the Core.
- [ ] Domain randomization and history-conditioned policy.
- [ ] Held-out-parameter and perturbation evaluation.

## Phase 6: Empirical (optional)

- [ ] Coast-down, skidpad and acceleration tests on the real RC car.
- [ ] Identify parameters; promote labels from `provisional` to `identified`.

## Known issues

- [ ] **Vertical faces in the scene (ramp sides, plateau edges) are not obstacles.** The ground query is height-only (ADR 0005): `RaycastGround` reports whatever is under each wheel, so driving into a ramp's side snaps that wheel up onto the top, and driving off an edge drops it. The Core has no wall or obstacle contact. Needs: a step-height limit in the ground query (a rise above some height is an obstacle, not ground) and an obstacle contact model in the Core. Reported 2026-10-19 while driving the twin; deferred.

## Housekeeping

- [ ] CI: build, unit tests, determinism hash check per build.
- [ ] Decide public/private and set GitHub repo description and topics.
