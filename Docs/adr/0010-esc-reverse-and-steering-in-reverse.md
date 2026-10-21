# ADR 0010: The ESC reverses from a stop, and L2 steers in reverse

**Status:** Accepted  **Date:** 2026-10-21
**Related:** our ADR 0006 (wheel dynamics, lock-up), ADR 0009 (walls); `TwinAutopilot` turning round at a stop (`unity/RcTwin/Assets/RcTwin/Runtime/TurnAround.cs`)

## Context

The campus autopilot has to turn the car round where a route leaves a stop the way the car came in, which takes reversing. Two things stopped the Unity car doing it.

1. **No reverse with wheel dynamics.** With `wheel_inertia > 0` (the Unity car's default, ADR 0006) a negative demand is the motor brake, which acts against the wheel's rotation and "stops a wheel rather than reversing it". Held at rest it holds the car there for ever. That rule is right for braking: a car spun round onto its back wheels must not have its brake drive them backwards (`BrakingASpunCarRollingBackwardsDoesNotRunAway`). Quasi-static wheels (`wheel_inertia == 0`, the RL environment's) have no such rule: a negative demand is a signed force, and the car reverses from rest.
2. **No steering in reverse.** L2 measured every slip angle as `atan2(vy, vx) - delta` from the wheel's forward direction. Rolling backwards that is near pi, far past the tyre's peak: the front wheels turned and the car went straight back, whatever its wheels said. SlipX documents L2 as valid for forward motion; nothing in this project reversed until now.

A hobby ESC (forward/brake/reverse) separates braking from reversing with state: the throttle pushed back brakes while the car moves, and once the car has stopped, held back, it drives the motor backwards. Which one the driver means depends on history, so it cannot come from the vehicle state alone (a car rolling backwards after a spin and a car reversing look the same). The Core's `step` is `const` and stateless (SlipX ADR-0004), so the history cannot live there.

## Decision

1. **Reverse is an input to the Core.** `DriveInput::reverse` (default false). With wheel dynamics and `reverse` set, a negative demand is drive, not the brake: the ESC runs the motor backwards on its own torque-speed curve, mirrored (`T = -(A - B (-omega))`, the same current caps), and the lock-up rule does not apply. Without wheel dynamics the flag is ignored: a negative demand already reverses. The default leaves every existing run bit for bit as it was.
2. **The latch is the ESC's, in `rc_twin`.** Per Core step, `rc_twin_step` engages reverse when the demand is negative and the car is stopped (body speed and every wheel's rim speed within 0.01 m/s), and releases it once the car rolls forward faster than that, or on `rc_twin_reset`. A car rolling backwards that never stopped is braked, as before. The latch is evaluated every step, so n steps remain exactly n single steps, and a failed call restores it with the state. No ABI change: a negative throttle held at rest now reverses with wheel dynamics as it always did without them.
3. **A wheel rolling backwards measures its slip from its backward direction:** `atan2(vy, |vx|) + delta`, so the tyre's force opposes the contact's lateral sliding `(vy - vx delta) / |vx|` in both directions. A wheel rolling forwards is unchanged bit for bit; the published conformance hashes are unchanged.

## Consequences

- The Unity car reverses: from the keyboard (hold S at rest), the autopilot's turns at stops, and anything else that holds a negative throttle at rest. The RL environment's quasi-static car reversed already and now also steers while doing so.
- Reversing yaws as the geometry says above `v_eps` (0.5 m/s): within 15 % of kinematic at 1 m/s, as forwards. Below `v_eps`, in either direction, L2 is outside its stated validity, and at full lock the yaw rate turns erratic. The autopilot therefore drives its turns at 0.7 m/s with short starts and stops. A low-speed tyre model (blending to kinematic below `v_eps`) would fix it properly and is not done.
- Reversing hard is traction-limited sooner than accelerating forwards: the load transfer unloads the driven rear axle, as on a real rear-drive car.
- Braking is unchanged: the lock-up rule and the spun car's brake hold, and their tests pass unchanged.
- **Verified by:** 6 Core tests (`test_wheels.cpp`: no reverse without the flag, reverse from rest, the mirrored curve at rest and on the curve at 12 m/s, a positive demand ending reverse, the flag ignored without wheels, steering in reverse with and without wheels) and 2 C tests (`test_capi.c`, `test_ground_capi.c`: the latch engages at rest, releases rolling forward, a reset leaves it, a spinning wheel is braked first, n steps equal n single steps). Mutations tried, all caught: the brake rule applied in reverse; the forward slip angle in reverse; the sign of delta in reverse; no mirroring of the budget; the curve at the forward speed; the latch never released; the reset keeping it; engaging without checking the wheels; latching once per call instead of per step.
