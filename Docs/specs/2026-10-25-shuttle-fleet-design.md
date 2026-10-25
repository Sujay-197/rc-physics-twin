# Campus shuttle fleet and camera modes

Status: design, 2026-10-25. Extends the campus shuttle game ([`2026-10-21-campus-shuttle-design.md`](2026-10-21-campus-shuttle-design.md)), whose rules, look and screen stay as they are unless said here. Car-to-car traffic is [ADR 0012](../adr/0012-buses-keep-left-and-give-way.md).

## Intent

The game had one shuttle and a seam for more: the dispatcher already asks every shuttle for its insertion cost. This fills the seam. Several buses serve the campus together and charge at two chargers. They keep out of each other's way on the roads and at the stops. The person watching can follow any bus, or watch the whole campus, from a choice of cameras.

## The fleet

- **N buses**, each a `TwinCar` with its own `TwinAutopilot` and `ShuttleBrain`, labelled Bus 1, Bus 2 and so on, each with its own stripe colour. The default is 3 and the maximum 6. N comes from `-buses <n>` on the command line and from the settings panel ("Buses", 1 to 6).
- **Spawning.** Bus 1 is the scene's car at S3. Each further bus is a copy of it, placed at the stop farthest by route from the stops already taken, facing along its node's first edge, with a full game battery.
- **Fewer buses at run time.** The highest-numbered bus *retires*: it takes no new requests, its pickups not yet aboard go back to waiting, and it disappears once it has delivered everyone aboard. More buses at run time spawn as at the start.
- **One game, several brains.** `ShuttleFleet` (a new MonoBehaviour) owns what is shared: the settings, the request spawner, the dispatcher, the score, the chargers, the traffic rules and the students waiting at stops. Each bus (`ShuttleGame`, now one per car) owns its brain, its battery and its leg, and is the brain's `IShuttleWorld`. The fleet ticks every bus, then the dispatcher, then the traffic, in that order and once per physics step, so a run is repeatable.
- **Dispatch** is unchanged: cheapest insertion across every bus that can take the request. An assigned request stays with its bus until it boards, gives up, or the bus releases it (low battery, retiring). Nothing reassigns it (that is the open "request reassignment" item).
- **Score.** One fleet score as before. Each bus also keeps its own count (delivered, kerb strikes, flats), and its events also count toward the fleet score.

## Chargers

- **Two chargers: AB1 and AB3** (`ShuttleSettings.Chargers`). AB3 is the campus's dead end, 64 m by road from AB1, on the far side. No stop-to-stop route passes through AB1 or AB3, so a bus charging at either blocks no through traffic.
- Everything that used "the charger" now uses *the nearest charger* by route: the energy rule (the way from the plan's last stop to a charger), the low-battery plans, idle top-ups and the tow after a flat battery.
- **One bus per charger, as a rule.** A charger is taken while another bus charges there or has it in its plan. A bus that needs to charge goes to the nearest free charger. If both are taken, it goes to the nearest one, waits behind the bus there (see the traffic rules) and charges from that second berth.

## Traffic (ADR 0012)

The Core has no car-to-car contact: each `TwinCar` is its own vehicle, and walls are immovable. Buses therefore keep out of each other's way by traffic rules, and an overlap is counted as an incident, never resolved by physics.

- **Keep left.** In the game, every bus drives its route 0.2 m to the left of the centreline, as on Indian roads. That is the largest offset at which the shipped autopilot drives all 90 campus trips without touching a kerb (`tools/lane_sweep.py`; at 0.25 m, 34 trips touch). It still leaves oncoming buses 0.125 m apart. Stops are kerb-side, at the end of the lane. At S3, a junction, a bus waits short of the crossing road, as a bus would at a stop line. The one-car demo (`-goto`) keeps the centreline.
- **Line up before leaving.** A stop on a bend leaves a bus facing up to 60° off its way out. Driving off from there, it would swing into the other lane. So a bus more than 15° or 0.15 m off its lane first lines up, planned and driven as a turn-round is.
- **Give way.** Each control tick, each bus's path ahead is checked against the other buses' footprints, inflated by a 0.03 m clearance. The path is the rest of its turn-round or line-up if it is in one, then its route; the arc its steering is on now is checked too. Where the route bends, the path gets 0.14 m more room on the inside of the bend: half of the most that pure pursuit cuts a corner on the campus.
  1. *Bodies.* No bus drives into another's footprint: it brakes to stop short of it. Two buses already within the clearance may only keep or widen the gap between them, so they can pass side by side, or keep at least one clearance between them, squeezing past a parked bus.
  2. *Paths.* Where two buses' paths cross ahead, one gives way. A bus already standing in the other's path goes first, so that it clears. Next, a bus that can no longer stop before the crossing goes first. Next, a bus turning round or lining up. Otherwise the bus that gets there first goes first, and on a tie the lower bus number. A decision holds until the two paths stop crossing, so the buses do not take turns at deciding. A cycle of buses waiting on each other is broken in favour of the lowest number.
  A bus giving way caps its speed so that it can stop at the stop plan's braking (1.2 m/s²) 5 cm short of the conflict. Its kerb recovery does not read that hold as being pinned against a kerb, and the trip's timeout does not run while it is held.
- **Queues at stops.** A bus whose stop is taken by a parked bus stops behind it. It counts as arrived and serves the stop from there, like a second bus at a real stop, if it is within a berth's length of the stop (1 m) or the parked bus is at the same stop.
- **Making room.** An idle bus with nothing to do waits at its stop, in its lane. If it holds another bus up for more than 3 s, it drives off to the nearest stop that is not on the held bus's way and has no bus.
- **Turning round** plans around the other buses' footprints as obstacles. A plan that another bus has since moved into, or a turn held up for 3 s, is planned again. Its legs are the bus's path ahead, so others give way to it.
- **Parked buses stay put.** After a trip, the autopilot holds the bus at rest until its next trip; released, the car would roll on.
- **Stuck.** If a bus has been held to a standstill for 20 s, the higher-numbered of it and the bus holding it, if driving, is towed to its leg's stop, as a failed leg is today. The HUD counts these. They are a fault: the Play Mode run expects none.
- **Tows** (failed legs, flat batteries) put the bus at a berth of the stop clear of every other bus and of where each is about to drive.

## Energy with several buses

Turning round and lining up reverse at walking pace, which costs far more charge per metre than cruising. A bus therefore learns its energy per metre faster: the average weights each leg by 0.2, up from 0.05. With a low battery, it re-checks every tick that its plan still reaches its charger. If not, it charges first, at the nearest charger.

## Camera modes

The campus player had one chase camera; the replay player has Pack, Chase, Route and Campus. The campus player now has these:

| Mode | View |
|---|---|
| **Bus k** | behind and above bus k (the chase camera), one mode per bus |
| **Route** | from above the followed bus's current leg, framing it whole |
| **Fleet** | above all the buses, far enough back to keep every one in view (the replay's Pack) |
| **Overview** | the whole campus, top-down (the replay's Campus) |
| **Orbit** | free: drag with the right mouse button to orbit the campus, scroll to zoom |
| **Stop** | from the kerb at a stop, looking along its berth; it opens on the stop with the most students waiting, and `[`/`]` step through the stops |

- **C** cycles forward through Bus 1, …, Bus N, Route, Fleet, Overview, Orbit, Stop; **Shift+C** cycles back; **1–6** follow that bus. Clicking a bus's row in the fleet panel follows it.
- A camera chip at the top of the screen shows the mode ("Camera · Bus 2", "Camera · Stop S3") between ◀ and ▶ buttons that cycle it.
- `-camera bus<k>|route|fleet|overview|orbit|stop[:<name>]` picks the starting mode (`-camera bus` is Bus 1), and `-screenshot` still works with it.
- The mode list and its parsing are plain C# (`CameraModes`); `FleetCamera` places the camera.

## Screen

- **Top-left:** the fleet score as before, then a fleet panel with one row per bus: its colour, name, battery, seats and what it is doing. The followed bus is highlighted.
- **Minimap:** every bus as an arrow in its colour with its number; the followed bus's planned path; both chargers in green.
- **Bottom bar:** the followed bus (Bus 1 when the camera follows none), as before, with its name.
- **Over each bus:** "Bus 2 · 3/6".
- **Settings:** a "Buses" slider is added.

## Tests

- Edit Mode: dispatch across buses (the nearer bus takes a request, a full or retiring bus does not, the free charger is preferred); the traffic rules (a follower holds behind, oncoming lanes pass, a crossing gives way by arrival and holds its decision, a bus parked at a stop blocks the next one, a give-way cycle breaks); lane offsetting; camera mode selection and parsing.
- Play Mode: three buses serve the campus for several minutes with every request delivered, no stuck tows, no overlap and no flat battery; two buses sent to one stop queue and both serve it; the camera cycles through every bus and the other modes and follows the bus it names.

## Not in this version

- Overtaking, lane changes and passing a parked bus: a bus behind a parked one waits or is let through by "making room".
- Car-to-car contact in the Core (ADR 0012 says why).
- Reassigning a request from one bus to another once assigned.
- A charger the user places.
