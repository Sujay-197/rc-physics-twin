# ADR 0012: Buses keep left and give way; cars do not touch in the Core

**Status:** Accepted  **Date:** 2026-10-25
**Related:** our ADR 0009 (walls), ADR 0010 (reverse), ADR 0011 (low speed); the fleet design [`specs/2026-10-25-shuttle-fleet-design.md`](../specs/2026-10-25-shuttle-fleet-design.md); `TrafficRules` (`unity/RcTwin/Assets/RcTwin/Runtime/Shuttle/Traffic.cs`), `tools/lane_sweep.py`

## Context

The shuttle game now runs several buses at once, each a `TwinCar`: its own `rc_twin` vehicle, stepped on its own. Nothing in the Core knows about a second car. Walls (ADR 0009) are immovable segments on one vehicle handle: they stop that car's footprint, and the wall feels nothing.

The autopilot drives every route on its centreline, and the campus is close to a tree: 36 nodes and 37 edges, so most stop pairs share their roads both ways. Two buses on centrelines would meet head-on on most roads, and at every stop a second bus would drive into the first.

There were three ways to stop that:

1. **Contact in the Core.** Two vehicles exchange impulses where their footprints touch. SlipX's `Simulation` resolves contact against immovable walls only, so this would be new two-body contact (masses, inertias, impulse exchange, ordering) in `rc_twin`, plus an ABI that steps several vehicles together. It is also only half a fix. Buses that bounce off each other still block each other, and a game where buses collide is not one that runs itself.
2. **Other buses as walls.** Each step, every car gets the others' current footprints as walls. This is one-sided: the car that hits feels a wall, the car that is hit feels nothing. It would rebuild a large wall list (the campus has 190 wall points) on every vehicle every step. And it still lets buses meet head-on on centrelines.
3. **Traffic rules above the Core.** Buses drive in lanes and give way to each other before they touch, as drivers do. The Core stays a single-car model.

## Decision

Option 3. Car-to-car contact is not modelled. Buses avoid each other with rules in the game, and an overlap is counted, never resolved.

1. **Keep left, 0.2 m off the centreline** (`TwinAutopilot.laneOffset`; `ShuttleFleet.LaneOffset`). The offset is mitred at the polyline's corners. Stops are at the lane's end, kerb-side, and turn-rounds plan from lane to lane. At a stop on a junction (S3, where three roads meet), a bus waits short of the crossing road: half the road's width plus half its footprint back from the node (`junctionSetback`, 0.92 m). At the node, it would stand across the mouth of the next road, where a bus turning in cuts the corner. With an offset of 0 (the default, and the one-car demo) routes are driven as before.
   - **Why 0.2 m.** `tools/lane_sweep.py` drives every campus stop pair with the shipped driver: pure pursuit with the Unity-car gains, the corner speed plan and kerb recovery, on the Unity car with nominal parameters, at 2 m/s.
     - At 0.2 m (and 0.15 m), all 90 trips arrive with no wall contact.
     - At 0.25 m, 34 of the 90 touch a kerb, and more at 0.30 and 0.35 m: pure pursuit cuts corners by up to 0.2 m.
   - **Room to pass.** The footprint is 0.42 by 0.275 m (the provisional car's wheelbase plus a wheel radius each end, its track plus the tyres). Oncoming buses 0.4 m apart leave 0.125 m between them, more than the two 0.03 m clearances below, so each may stray 3 cm from its lane and still pass. A smaller offset would make every oncoming meeting a give-way.
2. **Line up before leaving.** A stop on a bend leaves a bus facing up to 60° off its way out, and from rest pure pursuit swings 0.15 m into the other lane while it straightens. So in a lane, a bus that would start further than the turn hand-over's tolerances from its lane (0.15 m, 15°) first lines up. It plans and drives the move as it does a turn-round (`TurnPlanner`, `TurnDriver`), and if no move is found it drives off as before. In a lane, turn-rounds and line-ups hand over within 0.75 m of the route's start rather than 1.5 m. A trip started from rest 1 to 1.25 m before the sharp bend below AB2 meets its kerb, in the lane and on the centreline alike. Every campus turn-round and line-up between lanes plans (Edit Mode).
3. **Give way.** `TrafficRules` (plain C#, Edit Mode tested) runs once per control tick over every bus. Each bus gives it:
   - its footprint, inflated by a 0.03 m clearance;
   - its path ahead, sampled every 5 cm: the whole rest of a turn-round or line-up (the last plan while one is being replanned), then the route up to its braking distance at 1.2 m/s² plus 1 m. Where the route turns more than 20° within 0.4 m, its samples get 0.14 m more room on the inside of the bend, where pure pursuit cuts. Pure pursuit strays up to 0.28 m from its lane on the campus (the largest 95th-percentile offset of a trip, `tools/lane_sweep.py`), so this is half of it for each of two buses;
   - its present arc: the curve its steering is on now, over the same distance. This catches a bus off its path, cutting a corner or settling after a turn, before its path does.

   From these it returns, for each bus, how far it may still drive:
   - **Bodies.** A bus never drives, along its path or its arc, into another bus's present footprint. Two buses already within the clearance may only move so that the gap between their footprints does not shrink (passing side by side keeps it), or so that at least one clearance stays between them (squeezing past a parked bus).
   - **Crossings.** Where two paths cross, one bus gives way. A bus already standing in the other's path goes first, so that it clears. Next, a bus that can no longer stop before the crossing goes first. Next, a bus turning round or lining up, a slow move in a tight spot. Otherwise the earlier arrival goes first, at the bus's speed or 0.3 m/s if slower, and on a tie the lower bus number. The decision holds while the paths still cross. A cycle of waits is broken by the lowest-numbered bus in it that is giving way at a crossing; a wait on a body is never lifted.

   The autopilot caps its speed at what stops it 5 cm short of that distance at 1.2 m/s², the stop plan's braking (`StopEarlyDecel`). A held turn-round stands still. While held, kerb recovery does not count a stall, and neither the trip's timeout nor the turn driver's stall timer runs.
4. **Turning among other buses.** A turn-round or line-up plans around the other buses' footprints as obstacles. A finished plan that another bus now stands in is planned again, after 1.5 s, up to four times. So is a turn held up for 3 s. Only then does the autopilot fall back to turning the car at the stop. Like a tow, that puts the bus only where no other bus stands or is about to drive (`ShuttleFleet.PoseFree`); otherwise the leg fails and the bus is towed.
5. **Parked buses stay put.** After a trip ends (arrived, failed, cancelled), the autopilot holds the car at rest with a proportional brake, as the turn driver does. A car released at walking pace keeps rolling, and creeps even from rest. The hold lasts until the next trip or until something else commands the car.
6. **Overlaps and stuck buses are incidents.** The fleet counts footprint overlaps. A bus held to a standstill for 20 s by another counts as stuck: the higher-numbered bus that is driving is towed to its stop. The HUD shows both counts, and the Play Mode fleet test expects zero of each.

## Consequences

- Several buses run with the Core unchanged: no ABI change, and the single-car hashes and conformance are untouched.
- In the game, the autopilot drives a lane, not the centreline, and lines up at stops on bends. The driver is the same. `tools/lane_sweep.py` is the check: a different cruise speed, road width or driver needs it run again. It found one weakness of the driver that the lane widens, the start right before the bend below AB2, now avoided by the shorter hand-over.
- A bus cannot pass another in its lane. Behind a parked bus it waits, or the parked bus makes room if it is idle. The chargers are at stops that no route passes through, so charging blocks no through traffic.
- Turning round and lining up reverse at walking pace, and the Core's motor draws far more current there than cruising. Turns cost a bus a noticeable share of its small game battery. The shuttle's energy estimate learns this per leg (`SocMeter.Weight` 0.2), and a bus with a low battery re-checks every tick that it still reaches its charger.
- If the rules miss something, buses overlap visually and nothing pushes them apart. The count makes that visible.
- Car-to-car contact in the Core stays possible later (option 1) if a scenario needs collisions as physics, for example a crash study. It would sit under these rules, not replace them.
