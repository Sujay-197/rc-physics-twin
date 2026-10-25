# ADR 0011: L2 blends into a kinematic car at low speed

**Status:** Accepted  **Date:** 2026-10-25
**Related:** our ADR 0006 (wheel dynamics), ADR 0007 (Ackermann), ADR 0010 (reverse); SlipX ADR-0012 (`v_eps`); `TurnDriver` (`unity/RcTwin/Assets/RcTwin/Runtime/TurnAround.cs`)

## Context

L2's slip angles divide by the wheel's longitudinal speed, floored at `v_eps` (0.5 m/s) so the model has no singularity at a standstill. SlipX calls this a numerical mitigation and says "below roughly v_eps, use L0". The twin never switches tier: it starts every trip from rest, stops at every stop and turns round at full lock, all on L2.

Below `v_eps` the floor sets the turn. A wheel at zero slip satisfies `atan2(v_y, v_eps) = delta` instead of `atan2(v_y, v_x) = delta`, so the steady yaw rate at full lock is `v_eps tan(delta) / L` whatever the speed. Measured on the Unity car (`TwinCar`: wheel inertia, ideal Ackermann, suspension), at full lock:

| Target speed | Yaw rate | Kinematic `v_x tan(delta) / L` | Yaw rate spread |
|---|---|---|---|
| 0.1 m/s forwards | 0.705 rad/s | 0.144 rad/s | 0.018 |
| 0.3 m/s forwards | 0.709 | 0.378 | 0.031 |
| 0.3 m/s backwards | −0.738 | −0.248 | 1.213 |
| 0.5 m/s backwards | −1.297 | −0.401 | 1.353 |

So the car pivots up to five times tighter than its geometry allows and slides sideways faster than it moves forwards. Reversing slowly, the yaw rate wanders by more than its own mean. A car braked to rest at full lock keeps turning: 0.21 rad/s after four seconds held at zero speed. `TurnDriver` therefore drove turn-rounds at 0.7 m/s, and slower turns touched kerbs.

The literature does the same thing for the same reason. The F1TENTH gym, a simulator for this class of car, switches its single-track model to the kinematic one below 0.5 m/s (`vehicle_dynamics_st`: `if abs(x[3]) < 0.5`). AMZ Driverless blends the two models instead of switching (Kabzan et al., *AMZ Driverless: The Full Autonomous Racing System*, JFR 2020, section 5.1). Their dynamic model "is ill-defined for slow velocities due to the slip angles". Both models share one state, and the derivative is `λ f_dyn + (1 − λ) f_kin`, with `λ = min(max((v_x − v_min) / (v_max − v_min), 0), 1)`. The kinematic accelerations come from differentiating `r = tan(delta) v_x / L` and `v_y = l_R r`. For their car the window is 3 to 5 m/s.

## Decision

1. **An opt-in parameter, `VehicleParams::v_kinematic`** (m/s, default 0 = off, otherwise in `[0, v_eps)`). With it set, L2's derivative for `v_x`, `v_y` and `r` is AMZ's blend. The dynamic weight `λ` is 1 from `v_eps` up, 0 at and below `v_kinematic`, and linear in `|v_x|` between them. Off, or at or above `v_eps`, nothing changes, bit for bit: the published conformance hashes and every existing car are untouched. It is in the manifest digest when set. No effect below L2.
2. **The window ends at `v_eps`**, where the floor stops acting. That is also the F1TENTH gym's switching speed for this class of car. **The Unity car's `v_kinematic` is 0.3 m/s**, 0.6 `v_eps`: AMZ's window ratio (3 to 5 m/s), kept for this car. Like `v_eps` it is numerical, not measured.
3. **The kinematic derivative** is AMZ's, without their small-angle step:
   - `r_kin = v_x tan(delta) / L` and `v_y,kin = l_r r_kin`, differentiated with the achieved steer and its servo rate (the `1 / cos² delta` term included).
   - `v_x` is driven by the tyres' longitudinal forces with no lateral demand, plus drag, rolling resistance and grade.
   - **A settling term (ours):** `(r_kin − r)` and `(v_y,kin − v_y)` decay at `v_eps / sigma`, the rate a tyre's lagged slip settles at `v_eps`. That is 6.25 /s for the reference tyres. Differentiating the constraint alone keeps any yaw rate the car brings into the kinematic range, so a car stopped at full lock would keep turning for ever.
4. **The wheels and the body see the same blend.** Each tyre's lateral demand is the dynamic one weighted by `λ`. A kinematic car's driven wheels then pull away with their whole grip, instead of having the floored lateral force eat into their friction ellipses. The reported longitudinal forces, slip ratios and accelerations are blended the same way, so the suspension and the wheel update are driven by the motion the car actually makes.
5. **The parameter goes through `rc_twin`** as `rc_twin_params.v_kinematic`, which bumps the ABI to v8. It is mirrored in C# (`RcTwinNative.Params.vKinematic`, `TwinCar.vKinematic`, default 0.3) and Python (`tools/drive_road.py --v-kinematic`, `rc_twin_env.UNITY_CAR`). The RL environment's `"bare"` car keeps it off, as the policy study trained.

## Consequences

- **The Unity car creeps round its geometry.** At full lock, 0.1 to 0.3 m/s, forwards and backwards, the yaw rate is the kinematic one and holds steady: 0.120 rad/s at 0.091 m/s, against 0.705 before. Reversing at 0.5 m/s, the yaw rate spread falls from 1.35 to 0.11 rad/s. A car braked to rest at full lock stops turning.
- **Turn-rounds.** All 23 campus turn-rounds, driven by `TurnDriver` on the Core with the campus walls (headless sweep in Edit Mode):

  | Cruise | Blend | Kerb contacts | Handed over | Worst end offset | Worst end heading | Mean time |
  |---|---|---|---|---|---|---|
  | 0.7 m/s | off | 0 | 23 / 23 | 4.8 cm | 11.7° | 6.0 s |
  | 0.3 m/s | off | 2 turns | 20 / 23 | 21 cm | 99° | 11.2 s |
  | 0.7 m/s | **on** | 0 | 23 / 23 | 1.9 cm | 4.9° | 5.5 s |
  | 0.3 m/s | **on** | 0 | 23 / 23 | 1.1 cm | 1.5° | 11.0 s |

  With the blend every cruise from 0.2 to 0.7 m/s drives clean. `TurnDriver` stays at 0.7 m/s, which is now both faster and tighter than before; 0.3 m/s is tighter still at twice the time.
- **The campus autopilot.** Stop task, 4 m/s, Unity car, 100 trips per condition, trip by trip against the same trips without the blend. The shipped autopilot (pure pursuit, kerb recovery, corner plan) finishes every trip either way, with no kerb contact and the same trip time, on the evaluation trips (seed 20000) and on fresh ones (seed 50000). Without the corner plan, success goes from 98 / 82 / 86 to 100 / 88 / 88 % (seed 20000) and from 100 / 91 / 87 to 100 / 90 / 92 % (seed 50000). Trip by trip that is +9 / −3 held-out (p = 0.15) and +6 / −4 and +7 / −2 perturbed: no significant change.
- **In the window the dynamic model still dominates.** The kinematic part's settling is weak beside the tyres' stiffness, so between `v_kinematic` and `v_eps` the steady turn stays close to the floored one: 0.69 rad/s at 0.36 m/s, against 0.47 kinematic and 0.71 floored. The turn is kinematic only at or below `v_kinematic`. A wider window would hand over more of the range, at the cost of a car that slides less than real tyres let it before the floor stops.
- **What a kinematic car does not represent:**
  - Lateral tyre forces: the reported lateral force is the dynamic one weighted by `λ`, zero when fully kinematic. A kinematic car's constraint forces are not modelled.
  - Sideways sliding below `v_kinematic`, on a bank or on ice.
  - Lagged slip angles: they keep relaxing as before, so the dynamic model takes over from them at the window's top.
  - L1: it keeps its floor (the twin does not drive L1).
- **Verified by:**
  - 12 Core tests (`test_low_speed.cpp`):
    - The parameter's validation.
    - The floor's defect without the blend.
    - The kinematic turn at full lock, forwards and backwards, for three cars (reference, the twin, a CoG off the middle).
    - Steady reversing in the window.
    - A stopped car stops turning.
    - Steering while creeping keeps to the kinematic turn.
    - The derivatives blend linearly at a state in the window.
    - A kinematic all-wheel-drive car's steered wheels drive with their whole grip.
    - A kinematic car rolls downhill.
    - Nothing changes above `v_eps`, forwards and backwards.
    - Left and right mirror bit for bit.
    - No effect below L2.
  - The C API round trip and refusal (`test_capi.c`).
  - Core 587 / 587, conformance hashes unchanged.
  - Mutations tried, all caught:
    - No settling in `r`; no settling in `v_y`.
    - `l_f` for `l_r` in the `v_y` rate, and in its target.
    - No steer-rate term; no `1 / cos²`.
    - No grade in `v_x`.
    - Signed speed in `λ`; `λ` ignoring `v_kinematic`.
    - The lateral demand left unweighted.
    - The longitudinal pass with lateral demand.
    - The weights swapped.
    - The blend only below `λ` = 0.5.
    - `v_x` from the dynamic model.
  - Three escaped the first pass and became tests:
    - The lateral-velocity check while steering.
    - The derivative check in the window.
    - The all-wheel-drive pull-away.
