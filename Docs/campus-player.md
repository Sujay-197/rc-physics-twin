# The campus player

What the Unity twin does once it is built. [The README](../README.md) has the build steps.

**Autopilot.** In the campus scene, the panel at the top right drives the car to any stop you pick. It starts from rest at the stop it is at and stops at the destination, on the twin's own physics, along the route the fleet sim and the RL environment use. Holding a drive key takes the car back.

Where the route leaves the stop the way the car came in, the car first turns round, driven: a planner (`TurnAround.cs`) finds a multi-point turn of forward and reverse arcs within the steer limit, inside the road (the campus kerbs and buildings as walls, and the network's road width), and the car drives it leg by leg before the trip starts at rest. The panel shows each leg as it is driven, then the turn it made ("Turned round: three-point turn: forward 0.15 m, reverse 0.45 m, forward 0.90 m"). Every turn-round on the campus has a plan, checked in Edit Mode. If no turn can be planned, the car is turned at the stop instead and the panel says "not driven". Reversing is the ESC's: hold a negative throttle at rest and it engages (ADR 0010). **RcTwin > Build Campus Player** builds the campus scene as `build/campus/RcTwinCampus.exe`; `-goto AB1 -speed 3` starts a trip from the command line.

**The campus shuttle game.** The campus player starts as an autonomous shuttle game run by a fleet of buses (`Docs/specs/2026-10-21-campus-shuttle-design.md`, `Docs/specs/2026-10-25-shuttle-fleet-design.md`):
- **Requests.** Students appear at the stops, busier stops more often, each wanting a ride to another stop. You can add one: click a stop on the minimap or in the world, then pick where they go.
- **The buses.** Three by default (`-buses <1-6>`, or the Buses slider in Settings), each the twin's car as a white campus shuttle with six seats and a coloured roof sign: Bus 1, Bus 2, and so on. Each request goes to the bus that can fit it in most cheaply, and each bus decides the rest itself:
  - in what order to serve its students, pooling them along the way and stopping on its way when someone waits there;
  - when to charge: the battery is sized so that a charge lasts about five minutes of driving, and a bus goes to the nearest free charger, at AB1 or at AB3, before it runs low.
- **Traffic.** The buses keep left (0.2 m off the centreline) and give way to each other. A bus never drives into another. Where two paths cross, a bus turning round goes first, and otherwise the later arrival gives way. A bus waits behind one parked at its stop and serves the stop from there, and an idle bus in the way drives off to make room. Leaving a stop on a bend, a bus first lines up with its lane. The Core has no car-to-car contact; these rules keep the buses apart instead (ADR 0012).
- **The score** rewards quick pickups and direct rides; students who wait too long give up.
- **The screen.** The score and a row per bus are top left (click a row to follow that bus), the camera mode top centre, the minimap with every bus top right, and the followed bus's battery, seats and what it is doing, with the request queue, along the bottom. **Settings** changes the number of buses, the request rate, seats, battery and charging, pauses, and runs the game at ×2 or ×4. F1 shows the car's and the autopilot's debug panels.
- **Cameras.** Follow each bus (Bus 1, Bus 2, ...), frame the followed bus's route, the whole fleet or the whole campus from above, orbit freely (drag with the right mouse button, scroll to zoom), or watch a stop from its kerb (`[` and `]` step through the stops). **C** cycles the modes, **Shift+C** goes back, **1-6** follow a bus, and the ‹ › buttons at the top do the same. `-camera bus2|route|fleet|overview|orbit|stop[:S3]` picks the first one.

`-goto <stop>` runs the one-trip autopilot demo instead of the game. In the editor the game stays off unless `ShuttleGame.RunInEditor` is set (as its Play Mode tests do), so the scene's other tests keep the car. Its rules are plain C# in `Runtime/Shuttle/`, tested in Edit Mode on a fake world.

**The campus look.** A golden-hour sky and sun, hills rising to a mountain range in the haze, post-processing, PBR road, sidewalks and buildings, the campus dressed with about 1,600 trees, hedges and bushes, street lamps, rocks and benches, and a modelled pod shuttle and students (`tools/make_figures.py`; without the prepared look, the box shuttle and capsule students). Every prop is off the asphalt and none has a collider, so the car and the autopilot are unaffected (`Editor/CampusLook.cs`, `Editor/CampusDressing.cs`). Its CC0 models and textures and the Inter font (THIRD_PARTY.md) are not in git; to get them before building the scene:

```bash
python tools/fetch_assets.py
blender -b --factory-startup --python tools/prepare_assets.py
```

The first downloads about 530 MB into `assets_src/`; the second (Blender 5.2) writes the 1/10-scale, LOD'd copies to `unity/RcTwin/Assets/RcTwin/Generated/Look/`. Without them the scene keeps its light, a procedural sky, the mountains, haze and post-processing, with plain surfaces and no dressing.

The autopilot's driver is an `IDriver` (`unity/RcTwin/Assets/RcTwin/Runtime/Drivers.cs`). Each control tick, the driver gets the RL environment's own `"stop"`-task observation (`sim_py/README.md`), computed in C# by `TwinObservation` on the native road. A trip ends as that task's episodes do: stopped within 0.3 m of the destination, or failed. Two drivers come with it, and the panel switches between them:

- the tuned pure pursuit, with its stop plan;
- a learned policy (`PolicyDriver`), loaded from a `tools/export_policy.py` export.

To add a controller, write another `IDriver`. Edit Mode tests check the C# side against drives recorded in Python (`tools/make_autopilot_fixture.py`): the observation, the pure pursuit and the campus routes match to 1e-9 in both RL tasks. A learned policy's actions match its recorded checks to 1e-4.

**RcTwin > Build Replay Scene** turns the campus into a replay of trained policies: pick any two stops, and every policy recorded by `tools/record_drives.py` drives that route with the same car at once, one coloured car each, with every route's outcome per policy in a grid. **RcTwin > Build Replay Player** builds it as `build/replay/RcTwinReplay.exe`, with `results/drives.json` beside it. See `sim_py/README.md`.
