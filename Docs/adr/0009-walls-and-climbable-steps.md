# ADR 0009: Walls are contact segments; the ground query climbs only small steps

**Status:** Accepted  **Date:** 2026-10-20
**Related:** SlipX ADR-0043 and ADR-0055 (`core/docs/adr/0055-walls-are-immovable-contact-segments-in-the-sim.md`), our ADR 0005 (ground), ADR 0008 (suspension); `NEXT_STEPS.md`, Known issues

## Context

The ground query (ADR 0005) is height only: `RaycastGround` reports whatever is under each wheel, cast from 50 m up. A vertical face is therefore a step in ground height, not an obstacle. Driving into the side of the 0.6 m plateau snaps the wheel onto the top in one step (and, since ADR 0008, slams the bump stop), and the campus scene's kerbs and buildings had to be drawn without colliders for the same reason. The Core has no wall or obstacle contact at all.

SlipX already has the mechanics. `slipx/contact.hpp` (ADR-0055) resolves a car footprint against an immovable wall segment: a Liang-Barsky clip decides touching, one impulse with restitution and Coulomb friction changes velocity and yaw rate, and a positional projection removes the penetration toward the side of the wall the car's centre is on, so a car cannot be squeezed through. Its `Simulation` uses it for race-track walls. Our hosts drive `VehicleModel` directly through `rc_twin`, not SlipX's `Simulation`, so the wall pass has to live in `rc_twin`.

## Decision

1. **Walls are plan-view polylines on the vehicle handle** (`rc_twin_add_wall`, open or closed; `rc_twin_clear_walls`), kept across `rc_twin_reset` and `rc_twin_set_params` as SlipX keeps them across `reset()`. After every Core step, `rc_twin_step` checks the footprint against each wall segment in the order they were added, skipping segments whose bounding box misses the footprint's (comparisons only, so a rejected segment leaves no trace), and applies SlipX's `segment_contact` and `resolve_contact` unchanged, the wall having zero inverse mass and inertia. The impulse changes the body velocity (converted from the world frame to the body frame) and the yaw rate; the projection moves the CoG. Wheel speeds, steering, charge and the suspension state are not touched: the wheels keep spinning against the wall as a real car's do.
2. **No walls, no change.** With an empty wall list the pass touches nothing: every trajectory is bit-identical to before, held by test.
3. **The contact constants are SlipX's** (`rc_twin_contact`: restitution 0.3, friction 0.5, no restitution below 0.1 m/s) and as unfitted as SlipX says they are: plausible and deterministic, provisional by nature, not measurable in a car park.
4. **The footprint defaults to known geometry:** the wheelbase plus a wheel radius at each end, the wider track across, centred between the axles. The tyres' outer halves stick out past the track by a width the parameters do not have, so a host that knows its car's outline sets the footprint (`rc_twin_set_contact`); the Unity car adds half its drawn tyre width.
5. **Each call reports its contacts** (`rc_twin_get_wall_report`): how many segment contacts were resolved over the call's steps, the total normal impulse, and the highest approach speed. Hosts decide what a hit means; an RL reward can charge for it.
6. **The ground query stops seeing the tops of walls.** A host casts each wheel's ray from that wheel's last ground height plus a climbable step instead of from far above, so a face taller than the step is never sampled as ground: the wheel stays on the floor and the wall stops the car. A ramp rising millimetres per step still climbs, and driving off an edge still drops. The step is a host setting, by default half the wheel radius (2.5 cm on the provisional car): a rule of thumb, not a measurement, chosen so the 1.5 cm speed breaker is still ground and the 6 cm kerbs and 0.6 m plateau are not. A reset (a teleport) forgets the last heights.
7. **Scenes declare their walls.** The scene builders emit a wall wherever a face is taller than the step: the plateau's sides, each ramp's sides from where they pass the step, the bank's high edge and ends, and on the campus the kerb lines and the buildings' footprints. Walls are explicit geometry in the scene, not extracted from colliders.

## Consequences

- The plateau, the ramps' sides, the campus kerbs and the buildings stop the car instead of lifting it; the ramps and the speed breaker drive as before.
- Penetration cannot carry the car's centre across a wall unless it moves more than the footprint's smallest half-extent in one step: 0.12 m against 2 mm per step at 2 m/s and 1 kHz. A step-size change must re-check this, as SlipX ADR-0055 says.
- One contact point per segment and sequential resolution near polyline joints, as in SlipX: order-dependent but deterministic. A car resting against a wall chatters at the restitution floor, which SlipX's anti-jitter threshold covers.
- Walls are planar and infinitely tall: a car on the plateau is stopped by the plateau's own side walls from the top as well. Nothing in our scenes is driven along from above, so this is accepted; a wall with a height range is a new decision if one is needed.
- The headless road ground (`rc_twin_road_ground`) is unchanged: leaving a road file's width still ends a headless run, and the RL environment still ends an episode when the car leaves the asphalt. Kerbs as walls in the headless harness are a follow-up.
- Automatic wall extraction from arbitrary colliders (a height scan) is not done; it is the natural follow-up for scenes we did not build.
