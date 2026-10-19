# Physics Twin: Design Spec

**Date:** 2026-10-19  **Status:** Draft for review  **Owner:** SJ
**Related:** `campus_fleet_man_unity` (fleet sim, `IVehicleNavigator` seam), `rc_transit/` (physical RC project)

## 1. Goal

A slow, accurate, deterministic vehicle simulation of a 1/10 electric Ackermann RC car that (a) can be driven manually, (b) can be controlled by an RL policy that adapts to changing physical parameters, and (c) feeds physically grounded time and energy costs into a fleet-management sim.

Accuracy beats speed. Speed comes from caching (edge profiles), not from simplifying the physics.

## 2. Non-goals

- Photorealistic rendering or camera models (Unity renders; no sensor realism work here).
- Full-scale road vehicles or traffic.
- Aerodynamics beyond a drag term (negligible at 1/10 scale below about 15 m/s).
- Claiming validation without real-car data. Defaults are provisional.
- Running the Core inside the fleet sim's hot loop.

## 3. Decisions already made

See `Docs/adr/`:

1. ADR 0001: fork SlipX as the dynamics Core.
2. ADR 0002: Core is engine-agnostic; Unity supplies ground queries and rendering.
3. ADR 0003: build the twin in Unity (deviation from the `rc_transit` Phase-3 Isaac Sim plan).
4. ADR 0004: edge profiler decouples the fast fleet sim from the slow physics.

## 4. Physics model

### 4.1 Longitudinal (per vehicle, for sanity checks and the profiler)

$$m\,\dot v = F_x - \tfrac12 \rho C_d A v^2 - C_{rr}\, m g \cos\theta - m g \sin\theta$$

where $\theta$ is road grade. Slope is a first-class term, not a post-hoc correction. The full model computes per-wheel forces; this form is used to check energy balance on constant-grade segments.

### 4.2 Tires

Magic-Formula-style tire with load sensitivity, combined slip (friction ellipse) and a relaxation length for the transient response. Base: SlipX `MF-lite`. Extension: friction scale $\mu_s(x,y)$ from the ground query multiplies the peak friction.

### 4.3 Chassis and load transfer

Double-track, four contact patches, per-corner vertical load including longitudinal, lateral and grade-induced transfer. Tier L3 adds per-corner spring/damper so bumps and speed breakers change contact load over time.

### 4.4 Steering

Ackermann geometry (SlipX uses parallel steer today; replacing that is an extension) with a second-order, rate-limited servo so commanded and achieved steer are separate states.

### 4.5 Wheel spin state

Per-wheel rotational dynamics (SlipX has none today) so braking lock-up and wheelspin are representable:

$$I_w\,\dot\omega = T_{drive} - T_{brake} - R_w F_x$$

### 4.6 Electric powertrain and energy

Motor torque limited by a torque-speed curve and current limit, with thermal derating. Battery:

$$V_t = V_{oc}(\mathrm{SoC}) - I\,R_{int}, \qquad \dot{\mathrm{SoC}} = -\frac{I}{Q}$$

$$P_{batt} = \frac{T\,\omega}{\eta(T,\omega)} + P_{aux}$$

Regen is limited by the regen current cap. "Fuel" for this car is battery energy.

### 4.7 Integration

Fixed step, RK4 at 1 ms. No `-ffast-math`, no `-march=native`, no multithreading inside the integrator (determinism requirements inherited from SlipX).

## 5. Road features

Stored on `RoadGraph` edges (and mirrored in Unity collision geometry):

| Field | Unit | Notes |
|---|---|---|
| `elevation` per polyline point | m | Grade derived by finite difference, smoothed |
| `banking` | rad | Optional |
| `speed_breakers[]` | `{s, height, length, profile}` | `s` = arc length position; profile e.g. `trapezoid`, `round_hump` |
| `surface_id` | enum | Maps to `(compound, surface)` tire pair and friction scale |
| `curvature` | 1/m | Derived |

Ground query contract (host to Core), called per contact patch per step:

```
GroundSample query(double x, double y, double t)
  -> { double height; Vec3 normal; double mu_scale; uint32 surface_id; }
```

## 6. Interfaces

- **Action vector** (manual and RL identical): `[steer_cmd, throttle_cmd]` with brake via negative throttle (regen/motor braking first).
- **Observation** (RL): speed, yaw rate, slip angles, wheel speeds, SoC, achieved steer, grade, short history window.
- **Unity seam:** a `PhysicsNavigator : IVehicleNavigator` implementing the fleet sim's actuator seam.
- **Native plugin:** C ABI around the Core; Unity P/Invoke.

## 7. Edge profiler

For each road edge and each (speed target, payload mass, battery SoC bucket) combination, run the Core headlessly under a reference controller and record:

- traversal time
- energy used (Wh)
- peak lateral/longitudinal acceleration
- speed-breaker slow-down penalty

Output: a table keyed by `(edge_id, vehicle_class, payload, soc_bucket)` consumed by the fleet sim as time-varying edge costs (fleet roadmap items C and D) and for energy-aware dispatch. Cache is invalidated when car parameters or road features change.

## 8. RL adaptation

- Gymnasium-style env around the Core; SB3 (PPO/SAC).
- Domain randomization per episode: friction, mass, motor lag, tire stiffness, battery internal resistance.
- Policy input includes a short history of observations and actions so it can infer parameters online (RMA-style).
- Evaluate on held-out parameter ranges and on perturbation tests, not only on the training distribution.

## 9. Validation plan

| Layer | Test | Pass criterion |
|---|---|---|
| Analytical | Constant-grade coast-down, steady-state circle | Matches closed form within tolerance |
| Invariant | Energy balance, left/right mirror symmetry, determinism hash | Bit-identical replay on one build |
| Cross-tier | Core L1 vs L2 in low-g limit | Agree within upstream-reported bounds |
| Oracle | Step-steer and skidpad vs gymkhana std model | Yaw rate and path within agreed tolerance |
| Empirical | Real RC coast-down, skidpad, straight-line accel | Needed to move labels from `provisional` to `identified` |

## 10. Phases

1. **Foundation:** fork, build native plugin, conformance run, flat ground in Unity.
2. **Vehicle:** Ackermann, wheel spin, ground-query callback.
3. **Road:** suspension tier, grade, speed breakers, surface friction, oracle validation.
4. **Fleet coupling:** edge profiler, time-varying edge costs, energy-aware dispatch.
5. **Learning:** RL env, domain randomization, adaptation evaluation.
6. **Empirical:** real-car data and parameter identification (optional but required for any validation claim).

## 11. Risks

| Risk | Mitigation |
|---|---|
| SlipX is alpha (v0.2.x), single maintainer, no empirical validation | Own the validation; keep changes upstreamable; pin a commit |
| Native plugin build on Windows/Unity 6 | Prove in Phase 1 before any other work |
| Unity PhysX conflicts with Core contact | PhysX used for ground queries only; Core owns vehicle dynamics |
| Parameters are guesses | Provenance labels; randomization ranges; real-car identification later |
| Scope creep into rendering/sensors | Non-goals above; defer to `rc_transit` Phase 3 |
| Deviates from `rc_transit` spec (Isaac Sim digital twin) | ADR 0003 records the decision and its cost |

## 12. Open questions

1. Real-car target: generic 1/10 electric Ackermann (provisional) or a specific kit (Traxxas Slash-class)?
2. Should the Core be forked into this repo (`core/`) or consumed as a git submodule of a personal fork?
3. Is the physical RC car in `rc_transit` meant to be the identification target for this twin?
