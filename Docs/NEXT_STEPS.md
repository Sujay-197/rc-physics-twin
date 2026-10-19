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
- [ ] Wrap Core in a C ABI (`step`, `reset`, `set_params`, `get_state`, ground-query callback).
- [ ] Unity: load the plugin, drive the reference car on flat ground with keyboard input.
- [ ] Headless harness implementing the same ground query (for CI and the profiler).

## Phase 2: Vehicle

- [ ] Ackermann steering geometry.
- [ ] Per-wheel spin state (lock-up and wheelspin representable).
- [ ] Per-patch ground query (height, normal, friction scale).

## Phase 3: Road

- [ ] Suspension tier (spring/damper per corner).
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

## Housekeeping

- [ ] CI: build, unit tests, determinism hash check per build.
- [ ] Decide public/private and set GitHub repo description and topics.
