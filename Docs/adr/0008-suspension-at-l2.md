# ADR 0008: Suspension at L2, opt-in, from measured wheel rates

**Status:** Accepted  **Date:** 2026-10-19
**Related:** SlipX ADR-0005, ADR-0022 (`core/docs/adr/0022-load-transfer-is-quasi-static.md`), our ADR 0005 (ground), ADR 0006 (wheel dynamics); spec 4.3 and 5

## Context

Phase 3 needs bumps and speed breakers to change contact load over time (spec 4.3), which needs a body that heaves, pitches and rolls against springs. SlipX L2 deliberately has none. Its ADR-0022 makes load transfer an instantaneous function of acceleration, for two stated reasons:

1. **Timing.** A roll degree of freedom is what SRS 2.4 lists under L3; putting it at L2 "moves L3 down and leaves L3 empty".
2. **Distribution.** The front/rear split of lateral transfer follows the roll stiffness distribution, which a team with wheel encoders, an IMU and a car park cannot identify; a guessed split is worse than none.

SlipX ADR-0005 also forbids a partially built tier: L3 is "L2 plus thermal and suspension", so an L3 with suspension and no thermal model would be exactly that.

## Decision

1. **Opt-in at L2, like ADR 0006 and 0007, and L3 stays as SlipX defines it** (it still throws). In this project features are parameters with an off default rather than tiers; SlipX's tier map is left alone.
2. **The parameters are wheel rates, which are measured, not identified.** Per axle: `wheel_rate_front`, `wheel_rate_rear` [N/m, spring force at the wheel per metre of wheel travel], `damping_front`, `damping_rear` [N s/m at the wheel], and `suspension_travel` [m of compression from static before the bump stop]. A wheel rate is read on the bench: put a known load on one corner (a bathroom scale under the wheel) and measure the wheel's travel with a ruler. That meets ADR-0022's own standard ("a ruler and a set of bathroom scales"), and it makes the roll stiffness distribution a consequence of measured numbers (wheel rate times track squared per axle) rather than a guess. Damping is measured from a drop test's decay. Suspension is on when both wheel rates are positive; all five default to 0.
3. **The body has three degrees of freedom, absolute:** heave `pos.z` (world height of the point `h_cog` below the CoG), pitch (nose up) and roll (right side down) relative to the world horizontal, with their rates in `vel_body.z` and `rates`. Absolute rather than relative to the road, so a road that tilts under the car (onto a ramp) does not re-base the state.
4. **Springs at each corner, vertical, small angles.** A corner sits at `z_i = Z + x_i pitch + y_i roll`; its spring force is `k (s0 + g_i - z_i) - c dz_i/dt`, never negative (the wheel lifts), plus a bump stop ten times stiffer past `suspension_travel`. `g_i` is the ground height under that wheel (ADR 0005, so per-wheel bumps finally act). `s0` is the static sag that holds the car level at rest on level ground, so `VehicleState{}` starts in equilibrium.
5. **Body equations, moments about the point below the CoG:** `m Z'' = sum F - m g`; `I_yy pitch'' = sum x F + m h a_x + m g h pitch`; `I_xx roll'' = sum y F + m h a_y + m g h roll`, with `a_x`, `a_y` the CoG's kinematic accelerations (tyre forces, resistance and gravity along the road). On level ground this reproduces SlipX's quasi-static transfer `m a h / L` in steady state; on a grade it gives the grade transfer of a parked car and none for a freely coasting one, which is right.
6. **Tyre loads are the spring forces** (times the road normal's `n_z` on a grade), and replace the two-pass quasi-static loads when suspension is on.
7. **Integration, split like ADR 0006.** The chassis integrates with the loads from the suspension state at the step's start, held; then the body's six states advance by RK4 over the step with the tyre forces held from the end-of-step evaluation. The body modes are a few hertz (the bump stop tens), far slower than 1 kHz, so explicit RK4 is comfortably stable here, unlike the wheel mode.

## Consequences

- Load transfer now takes time, a car pitches and rolls, and a speed breaker under one axle lifts it, unloads the tyres and settles.
- **Limits.** Roll and pitch centres at the ground (no suspension geometry, anti-dive or anti-squat); no anti-roll bars (fold them into the wheel rates for roll, which then overstates heave stiffness); springs vertical and small angles (on a grade the vertical springs take the vertical part of the car's acceleration along the road, so a car speeding up downhill on a 10 % grade carries about 1.5 % less normal load than it should); the damper acts on the body's velocity only, not the road's (a sharp bump is felt through the spring alone); no unsprung mass or tyre vertical stiffness; the body is a rigid plate. Small angles throughout: there is no rollover or flip geometry, so a road that throws the car into the air and over (centimetres under each wheel at once on soft springs) winds the angles up without limit rather than flipping it; a host should treat large pitch or roll, or all four loads at zero, as the car having left the model. A vertical face in the scene is still a step in ground height, not an obstacle, and now loads the bump stop hard (see `NEXT_STEPS.md`, Known issues).
- **The Unity twin uses demo values**, not a car: wheel rates for a 3 Hz ride frequency and dampers for a damping ratio of 0.3, with 15 mm of travel. They are labelled as such where they are set and must be replaced by bench measurements.
- This changes vendored SlipX code (`THIRD_PARTY.md`), and reverses ADR-0022 only where the user opts in; its default behaviour is unchanged bit for bit.
