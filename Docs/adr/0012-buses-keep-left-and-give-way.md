# ADR 0012: Buses keep left and give way; cars do not touch in the Core

**Status:** Accepted  **Date:** 2026-10-25
**Related:** our ADR 0009 (walls), ADR 0010 (reverse); the fleet design [`specs/2026-10-25-shuttle-fleet-design.md`](../specs/2026-10-25-shuttle-fleet-design.md); `TrafficRules` (`unity/RcTwin/Assets/RcTwin/Runtime/Shuttle/Traffic.cs`)

## Context

The shuttle game now runs several buses at once, each a `TwinCar`: its own `rc_twin` vehicle, stepped on its own. Nothing in the Core knows about a second car. Walls (ADR 0009) are immovable segments on one vehicle handle: they stop that car's footprint, and the wall feels nothing.

The autopilot drives every route on its centreline, and the campus is close to a tree: 36 nodes and 37 edges, so most stop pairs share their roads both ways. Two buses on centrelines would meet head-on on most roads, and at every stop a second bus would drive into the first.

There were three ways to stop that:

1. **Contact in the Core.** Two vehicles exchange impulses where their footprints touch. SlipX's `Simulation` resolves contact against immovable walls only, so this would be new two-body contact (masses, inertias, impulse exchange, ordering) in `rc_twin`, plus an ABI that steps several vehicles together. It is also only half a fix. Buses that bounce off each other still block each other, and a game where buses collide is not one that runs itself.
2. **Other buses as walls.** Each step, every car gets the others' current footprints as walls. This is one-sided: the car that hits feels a wall, the car that is hit feels nothing. It would rebuild a large wall list (the campus has 190 wall points) on every vehicle every step. And it still lets buses meet head-on on centrelines.
3. **Traffic rules above the Core.** Buses drive in lanes and give way to each other before they touch, as drivers do. The Core stays a single-car model.

## Decision

Option 3. Car-to-car contact is not modelled. Buses avoid each other with rules in the game, and an overlap is counted, never resolved.

1. **Keep left.** In the shuttle game the autopilot drives each route offset to the left by a quarter of the road width (`TwinAutopilot.laneOffset`, 0.35 m on the campus): the middle of the left half of the asphalt. The offset is mitred at the polyline's corners. Stops are at the end of the lane, kerb-side, and turn-rounds plan from lane to lane. With an offset of 0 (the default, and the one-car demo) the autopilot is unchanged.
   This was checked before it was built. The shipped driver (pure pursuit with the Unity-car gains, the corner speed plan and kerb recovery, on the Unity car, nominal parameters, 2 m/s) drives all 90 campus stop-to-stop trips at offsets of 0, 0.30 and 0.35 m. At each offset it finishes all 90 with no wall contact.
2. **Give way.** `TrafficRules` (plain C#, Edit Mode tested) runs once per control tick over every bus. It gets each bus's footprint and its path ahead (the remaining route, or the turn-round's remaining legs, then the route), sampled every 5 cm up to its braking distance at 1.2 m/s² plus 1 m. Footprints are inflated by a 5 cm clearance. It returns, for each bus, how far it may still drive:
   - A bus never drives into another bus's present footprint. If the two already overlap, it may only move so as not to get closer.
   - Where two paths cross, one bus gives way. A bus already standing in the other's path goes first, so that it clears. Next, a bus that cannot stop before the crossing goes first. Otherwise the earlier arrival goes first, at the bus's speed or 0.3 m/s if slower, and on a tie the lower bus number. The decision holds while the paths still cross. A cycle of give-ways is broken in favour of its lowest-numbered bus.
   The autopilot caps its speed at what stops it in that distance at 1.2 m/s², the stop plan's braking (`StopEarlyDecel`). While held, kerb recovery does not count a stall, and neither the trip's timeout nor the turn driver's stall timer runs.
3. **Overlaps are incidents.** The fleet counts footprint overlaps and stuck tows (a bus held to a standstill for 20 s by a bus that is itself held is towed to its stop). The HUD shows both counts, and the Play Mode fleet test expects zero of each.

## Consequences

- Several buses run with the Core unchanged: no ABI change, and the single-car hashes and conformance are untouched.
- Driving in a lane changes the line the autopilot drives in the game, though not the driver. Corners in the lane are tighter on left turns and wider on right turns. The sweep above covers every route at 2 m/s. A different cruise speed, road width or driver needs that sweep again.
- A bus cannot pass another in its lane. Behind a parked bus it waits, or the parked bus makes room if it is idle. The chargers are at stops that no route passes through, so charging blocks no through traffic.
- If the rules miss something (a pursuit line cutting a corner more than the clearance allows), buses overlap visually and nothing pushes them apart. The count makes that visible, and the clearance or the horizon is the fix.
- Car-to-car contact in the Core stays possible later (option 1) if a scenario needs collisions as physics, for example a crash study. It would sit under these rules, not replace them.
