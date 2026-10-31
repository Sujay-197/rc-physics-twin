# Campus Shuttle Implementation Plan

**Goal:** Turn the campus player into an endless autonomous-shuttle sandbox: students request rides between stops, one shuttle plans, pools, drives itself with the existing autopilot, charges at AB1, and scores.

**Architecture:** The game's rules live in plain C# free of `UnityEngine` (requests, travel times, the shuttle's plan, battery policy, score, the shuttle's state machine), tested in Edit Mode like `Drivers.cs`. A `ShuttleGame` MonoBehaviour connects them to the scene: it drives `TwinAutopilot.Go`, reads and sets the car's battery through `TwinCar`, and moves the student figures. `ShuttleHud` draws layout C. `CampusSceneBuilder` adds the game, the shuttle body, shelters and the charger, so the campus player is the game.

**Tech Stack:** Unity 6000.5.3f1, C# (Runtime asmdef `RcTwin.Runtime`), NUnit Edit Mode and Play Mode tests, the `rc_twin` plugin (ABI 7) unchanged.

**Spec:** `Docs/specs/2026-10-21-campus-shuttle-design.md`

## Global Constraints

- The car's physics, parameters other than the game battery's pack capacity, and the autopilot's driving are unchanged; the shuttle body is visual only (no colliders, footprint unchanged).
- Stops and demand weights come from `campus/campus_network.json` (S3 6, AB4 4, the rest 1); the charger is at AB1.
- Defaults (all in a settings object, `ShuttleSettings`): request rate 1 per 20 s across the campus; capacity 6; cruise 2 m/s; give-up wait 180 s; battery lasts 300 s of driving at cruise; charge 10 → 95 % in 30 s, leaves at 95 %; reserve 10 % of charge; energy safety factor 1.15; idle top-up below 60 %.
- Score: delivered +max(20, 100 − wait seconds), +20 if ride ≤ 1.5 × direct; gave up −50; flat battery −500; kerb strike −5.
- Pure game logic files contain no `using UnityEngine`.
- Edit Mode tests live in `unity/RcTwin/Assets/RcTwin/Tests/Editor/`, Play Mode in `.../Tests/PlayMode/`; new `.cs` files get their `.meta` from Unity's import (commit them).
- Run Unity tests: `"/c/Program Files/Unity/Hub/Editor/6000.5.3f1/Editor/Unity.exe" -batchmode -nographics -projectPath <worktree>\unity\RcTwin -runTests -testPlatform EditMode -testResults <xml> -logFile <log>` (PlayMode: drop `-nographics`, `-testPlatform PlayMode`). The native plugin must be at `unity/RcTwin/Assets/Plugins/x86_64/rc_twin.dll` (gitignored; copy `build/bin/rc_twin.dll` and its `.meta`).
- After a scene or player build, revert regenerated tracked files: `git checkout -- unity/RcTwin/Assets/RcTwin/Generated unity/RcTwin/Assets/Scenes unity/RcTwin/Assets/Settings unity/RcTwin/ProjectSettings`.

## Review Focus

1. A request whose origin is the stop the shuttle is standing at: it boards there without driving (`TwinAutopilot.Go` refuses a trip to the stop the car is at). Test in Task 6.
2. The autopilot fails a leg (`Status.Failed`: left the route, timed out) or refuses to start (`Go` returns false): the shuttle must not freeze. It is towed to the leg's stop and carries on. Test in Task 6.
3. The battery turns low while the shuttle is on its way to a pickup: at that stop, a student boards only if the drop-everyone-then-charge plan stays feasible with them aboard; otherwise they stay waiting, unassigned. Test in Task 6.
4. More students waiting at a stop than free seats: as many board as fit, by request age; the rest stay waiting and are re-planned. Test in Task 3 and Task 6.
5. Requests arriving while the shuttle charges at AB1: students at AB1 may board; the rest queue, and the shuttle does not leave before 95 %. Test in Task 6.

---

### Task 1: Ride requests and the request spawner

**Files:**
- Create: `unity/RcTwin/Assets/RcTwin/Runtime/Shuttle/RideRequest.cs`
- Create: `unity/RcTwin/Assets/RcTwin/Runtime/Shuttle/RequestSpawner.cs`
- Create: `unity/RcTwin/Assets/RcTwin/Runtime/Shuttle/ShuttleSettings.cs`
- Test: `unity/RcTwin/Assets/RcTwin/Tests/Editor/ShuttleRequestTests.cs`

**Interfaces:**
- Produces:
  - `enum RequestState { Waiting, Assigned, Riding, Delivered, GaveUp }`
  - `sealed class RideRequest { int Id; string Origin, Destination; double RaisedAt; double PickedUpAt = NaN, DroppedAt = NaN; RequestState State; bool UserRaised; }`
  - `sealed class ShuttleSettings` with public fields for every default in Global Constraints (`RequestInterval = 20`, `Capacity = 6`, `Cruise = 2.0`, `GiveUpAfter = 180`, `BatteryMinutesOfDriving = 5`, `ChargeSeconds10To95 = 30`, `LeaveChargerAt = 0.95`, `Reserve = 0.10`, `EnergySafety = 1.15`, `IdleTopUpBelow = 0.60`, score constants `DeliverBase = 100`, `DeliverMin = 20`, `DirectBonus = 20`, `DirectFactor = 1.5`, `GaveUpPenalty = 50`, `FlatPenalty = 500`, `KerbPenalty = 5`, `ChargerStop = "AB1"`), plus `DwellBase = 3.0`, `DwellPerStudent = 1.5`.
  - `sealed class RequestSpawner(IReadOnlyList<(string name, int node, int weight)> stops, int seed)` with `double RatePerSecond`, `List<RideRequest> Tick(double now, double dt)`, `RideRequest Raise(string origin, string destination, double now)` (user; throws `ArgumentException` for an unknown stop or the same node), `IReadOnlyList<RideRequest> All`.

- [ ] **Step 1: Write the failing tests**

```csharp
[Test] public void OriginsFollowTheDemandWeights()
// 100 000 s at RatePerSecond 1/20 with seed 1: S3's share of origins is 6/19 ± 0.02, AB4's 4/19 ± 0.02.
[Test] public void ADestinationIsNeverTheOriginsNode()
// every spawned request: node(Origin) != node(Destination); destination shares by weight excluding the origin (S3 as a destination when AB2 is the origin: 6/18 ± 0.02).
[Test] public void TheSameSeedSpawnsTheSameRequests()
[Test] public void AUserRequestIsRaisedAtOnceAndChecked()
// Raise("AB2","S6",12.5): Id unique, State Waiting, UserRaised true, RaisedAt 12.5; Raise("AB2","AB2",0) and Raise("XX","S6",0) throw ArgumentException.
```

- [ ] **Step 2: Run the Edit Mode tests; expect compile failure (types missing)**
- [ ] **Step 3: Implement the three files.** Poisson arrivals: per tick, the number of new requests is drawn so that the expected count is `RatePerSecond * dt` (one Bernoulli per tick is enough for `dt` ≤ 0.1 s); origin and destination by weight with `System.Random(seed)`.
- [ ] **Step 4: Run the Edit Mode tests; expect PASS**
- [ ] **Step 5: Commit** `git add unity/RcTwin/Assets/RcTwin/Runtime/Shuttle unity/RcTwin/Assets/RcTwin/Tests/Editor/ShuttleRequestTests.cs* && git commit -m "Shuttle: ride requests and a demand-weighted spawner"`

### Task 2: Travel model from the campus routes

**Files:**
- Create: `unity/RcTwin/Assets/RcTwin/Runtime/Shuttle/TravelModel.cs`
- Test: `unity/RcTwin/Assets/RcTwin/Tests/Editor/ShuttleTravelTests.cs`

**Interfaces:**
- Consumes: `CampusRoutes.Route(string, string) -> double[]` (xyz triples), `CampusRoutes.Stops`.
- Produces: `interface ITravelModel { double Distance(string a, string b); double DriveTime(string a, string b); double TurnAllowance(string from, string via, string to); IReadOnlyList<(string stop, double s)> StopsOnRoute(string a, string b); }` (`StopsOnRoute`: the other stops whose road node lies on the route from a to b, with their arc length along it, in order) and `sealed class RouteTravelModel(CampusRoutes net, double cruise) : ITravelModel`. `Distance` is the route polyline length (0 for a == b), cached per pair. `DriveTime = Distance / cruise`. `TurnAllowance(from, via, to)` is 8 s when the heading leaving `via` toward `to` differs by more than 90° from the heading arriving at `via` from `from` (the autopilot turns round), else 0; `from` may be null (no allowance).

- [ ] **Step 1: Write the failing tests**

```csharp
[Test] public void DistancesAreTheRouteLengths()
// Distance("S3","AB1") equals the summed segment lengths of net.Route("S3","AB1") to 1e-9; Distance(a,a) == 0; Distance("S3","AB1") > 20 and < 35.
[Test] public void DriveTimeIsDistanceOverCruise()   // cruise 2: DriveTime == Distance / 2
[Test] public void AReversalAtAStopCostsATurnRound()
// TurnAllowance("AB1","S3","AB1") == 8 (arrive at S3 from AB1, leave back toward AB1); TurnAllowance(null,"S3","AB1") == 0.
[Test] public void StopsOnTheWayAreFound()
// StopsOnRoute("AB4","AB1") contains S3 and S1 in route order with 0 < s < Distance("AB4","AB1"); StopsOnRoute("S3","S1") excludes S3 and S1 themselves.
```
Load the network as `CampusDriveTests`/`AutopilotParityTests` do (`campus/campus_network.json` from the repo root).

- [ ] **Step 2: Run; expect FAIL** (types missing)
- [ ] **Step 3: Implement `RouteTravelModel`** (headings from the first and last segments of the route polylines).
- [ ] **Step 4: Run; expect PASS**
- [ ] **Step 5: Commit** `"Shuttle: travel times from the campus routes"`

### Task 3: The shuttle's plan and cheapest insertion

**Files:**
- Create: `unity/RcTwin/Assets/RcTwin/Runtime/Shuttle/ShuttlePlan.cs`
- Test: `unity/RcTwin/Assets/RcTwin/Tests/Editor/ShuttlePlanTests.cs`

**Interfaces:**
- Consumes: `RideRequest`, `ShuttleSettings`, `ITravelModel`.
- Produces:
  - `sealed class PlanStop { string Stop; List<RideRequest> Board; List<RideRequest> Alight; }`
  - `struct PlanContext { string From; string ArrivedFrom; double Now; int Onboard; double Soc; double SocPerMetre; }` (`From`: the stop the shuttle is at, or the committed next stop if driving; `ArrivedFrom`: the previous stop, for turn allowances; `Onboard`: students aboard on arrival at `From`, after its alighting/boarding).
  - `sealed class ShuttlePlan { List<PlanStop> Stops; double Cost(PlanContext c, ShuttleSettings s, ITravelModel t); double SocNeeded(PlanContext c, ShuttleSettings s, ITravelModel t); bool Feasible(PlanContext c, ShuttleSettings s, ITravelModel t); ShuttlePlan TryInsert(RideRequest r, PlanContext c, ShuttleSettings s, ITravelModel t, out double addedCost); }` (`TryInsert` returns a new plan or null).
- Rules:
  - Simulating a plan from `c`: time starts at `c.Now`; each leg adds `DriveTime + TurnAllowance`; each stop adds `DwellBase + DwellPerStudent × (boarding + alighting)`.
  - `Cost` = the plan's finishing time − `c.Now` + Σ over its requests of (pickup time − RaisedAt) + Σ of (drop time − pickup time); students already aboard count only their remaining ride.
  - `SocNeeded` = (Σ leg distances + Distance(last stop, ChargerStop)) × `SocPerMetre` × `EnergySafety`.
  - `Feasible` = load never exceeds `Capacity` and `SocNeeded ≤ c.Soc − Reserve`.
  - `TryInsert` tries every pickup position i and drop position j ≥ i (inserting new stops or merging into an existing stop with the same name at that position) and keeps the feasible plan with the least `Cost`.

- [ ] **Step 1: Write the failing tests**

```csharp
[Test] public void AnEmptyPlanTakesTheRequestDirectly()
// from S3, request AB2→S6: plan stops [AB2(board), S6(alight)], addedCost > 0.
[Test] public void ARequestOnTheWayIsPooledNotDetoured()
// plan S3→AB1 with one rider, new request whose origin and destination lie on that route order: inserted between, total stops ≤ 3 (merged where names match).
[Test] public void CapacityIsNeverExceeded()
// Capacity 2, two riders aboard to S6: a third request from From to S6 → TryInsert returns null; with Capacity 3 it succeeds.
[Test] public void ThePickupComesBeforeTheDropOff()   // over 200 random insertions: each request's board index < alight index
[Test] public void APlanThatCannotReachTheChargerIsRefused()
// Soc 0.12, SocPerMetre so that the plan plus the way to AB1 needs 0.05: refused (0.12 − 0.10 < 0.05); Soc 0.20: accepted.
```

- [ ] **Step 2: Run; expect FAIL**
- [ ] **Step 3: Implement `ShuttlePlan`**
- [ ] **Step 4: Run; expect PASS**
- [ ] **Step 5: Commit** `"Shuttle: the plan, cheapest insertion with capacity and battery"`

### Task 4: Battery policy and score

**Files:**
- Create: `unity/RcTwin/Assets/RcTwin/Runtime/Shuttle/BatteryPolicy.cs`
- Create: `unity/RcTwin/Assets/RcTwin/Runtime/Shuttle/ShuttleScore.cs`
- Test: `unity/RcTwin/Assets/RcTwin/Tests/Editor/ShuttleBatteryScoreTests.cs`

**Interfaces:**
- Consumes: `ShuttlePlan`, `PlanContext`, `ShuttleSettings`, `ITravelModel`, `RideRequest`.
- Produces:
  - `enum LowBattery { Fine, DropOffFirst, ChargeNow }`
  - `static class BatteryPolicy`:
    - `double GamePackCapacityAh(ShuttleSettings s, double whPerMetre, double packNominalV)` = `BatteryMinutesOfDriving × 60 × Cruise × whPerMetre / packNominalV`. The seed value is `SeedWhPerMetre = 0.00029` (twin, 2 m/s, training cars, `Docs/policy-study.md` fresh-trip table).
    - `double ChargeRatePerSecond(ShuttleSettings s)` = 0.85 / `ChargeSeconds10To95`.
    - `LowBattery Decide(ShuttlePlan plan, PlanContext c, ShuttleSettings s, ITravelModel t, out ShuttlePlan next)`: `Fine` if `plan.Feasible`. Otherwise build the plan keeping only the stops where riders aboard alight, in the plan's order, then the charger. `DropOffFirst` if its `SocNeeded` (charger leg included) ≤ `Soc − Reserve` with the reserve halved for this check. Otherwise `ChargeNow` with `next` = [charger, then those drop-offs]. Pickups not aboard are released (`State = Waiting`) in both cases.
  - `sealed class SocMeter` (`Observe(double metres, double socUsed)`; `double SocPerMetre`, exponential average with weight 0.05, seeded with the `SeedWhPerMetre` equivalent for the pack).
  - `sealed class ShuttleScore`: `int Points`, `int Delivered`, `int GaveUp`, `int Flats`, `int KerbStrikes`, `double AverageWait`; `OnDelivered(RideRequest r, double directDriveTime)`, `OnGaveUp(RideRequest r)`, `OnFlat()`, `OnKerbStrike()`.

- [ ] **Step 1: Write the failing tests**

```csharp
[Test] public void TheGameBatteryLastsTheSetMinutes()      // 5 min, 2 m/s, 0.00029 Wh/m, 11.1 V → 0.01568 Ah ± 1e-5
[Test] public void ChargingTakesTheSetTime()               // 30 s → 0.028333/s
[Test] public void LowBatteryDropsOffFirstWhenItCan()     // riders aboard to S6; Soc enough for S6 then AB1 → DropOffFirst, next = [S6, AB1]
[Test] public void LowBatteryChargesFirstWhenItMust()     // Soc only enough for AB1 → ChargeNow, next = [AB1, S6]
[Test] public void PickupsNotAboardAreReleased()          // an Assigned request in the plan → Waiting after Decide != Fine
[Test] public void ScoreFollowsTheRules()
// wait 30 s, ride ≤ 1.5× direct → +90; wait 95 s, slow ride → +20 (min); give-up −50; flat −500; kerb −5; AverageWait over delivered.
```

- [ ] **Step 2: Run; expect FAIL**
- [ ] **Step 3: Implement both files**
- [ ] **Step 4: Run; expect PASS**
- [ ] **Step 5: Commit** `"Shuttle: battery policy and score"`

### Task 5: Car hooks for the game

**Files:**
- Modify: `unity/RcTwin/Assets/RcTwin/Runtime/TwinCar.cs` (near `ResetTo`, line 186)
- Modify: `unity/RcTwin/Assets/RcTwin/Runtime/TwinAutopilot.cs` (`OnGUI`, line ~471; add a public flag)
- Test: `unity/RcTwin/Assets/RcTwin/Tests/PlayMode/ShuttleCarHookTests.cs`

**Interfaces:**
- Produces:
  - `TwinCar.SetPackCapacity(double ah)`: the vehicle's `Params` with `packCapacityAh = ah`.
  - `TwinCar.SetCharge(double soc)`: resets the Core to the current state with `soc` changed (clamped to [0, 1]). Only while parked (|vx| < 0.05); it throws `InvalidOperationException` otherwise.
  - `TwinCar.showPanel` and `TwinAutopilot.showPanel` (public bool, default true): their `OnGUI` draws nothing when false.
  - `TwinCar.KerbStrikes` (long): count of rising edges of the wall report's contacts (0 → > 0 between physics frames).
  - `TwinAutopilot.StopEarlyAt(string stop) -> bool`: while driving a trip, if `stop`'s road node lies on the route ahead at arc length `sStop` with `sStop − s ≥ v² / (2 × 1.2) + 0.5`, cut the trip's `TwinRoad` and corner plan at `sStop` (rebuild both from the route prefix) and set `Destination = stop`; the trip then arrives there as usual. Otherwise return false and change nothing.

- [ ] **Step 1: Write the failing Play Mode tests** on the Campus scene (load as `CampusDriveTests` does):

```csharp
[UnityTest] IEnumerator AChargeCanBeSetWhileParked()       // SetCharge(0.3) → State.soc == 0.3 ± 1e-9, pose unchanged ± 1e-9
[UnityTest] IEnumerator ASmallPackDrainsFast()            // SetPackCapacity(0.0157), drive S3→AB1 at 2 m/s: soc drop > 0.03 (it was < 0.001 with the stock pack)
[UnityTest] IEnumerator AKerbStrikeIsCountedOnce()        // ResetTo facing a kerb 0.3 m away, full throttle 1 s: KerbStrikes == 1 while WallContacts > 100
[UnityTest] IEnumerator ATripCanEndEarlyAtAStopOnTheWay()  // AB4→AB1 at 2 m/s; 1 s in, StopEarlyAt("S3") is true; the trip arrives at S3 (Arrived, CurrentStop "S3"); StopEarlyAt("S6") (not on the route) is false
[UnityTest] IEnumerator TooLateToBrakeIsRefused()          // StopEarlyAt for a stop 0.3 m ahead at 2 m/s returns false and the trip continues
```

- [ ] **Step 2: Run Play Mode; expect FAIL**
- [ ] **Step 3: Implement the hooks**
- [ ] **Step 4: Run Edit + Play Mode; expect PASS (and every existing test still passes)**
- [ ] **Step 5: Commit** `"TwinCar and TwinAutopilot: charge, pack capacity, kerb strikes, stopping early on the way, panel switches"`

### Task 6: The shuttle's brain

**Files:**
- Create: `unity/RcTwin/Assets/RcTwin/Runtime/Shuttle/ShuttleBrain.cs`
- Create: `unity/RcTwin/Assets/RcTwin/Runtime/Shuttle/Dispatcher.cs`
- Test: `unity/RcTwin/Assets/RcTwin/Tests/Editor/ShuttleBrainTests.cs`

**Interfaces:**
- Consumes: Tasks 1–4.
- Produces:
  - `interface IShuttleWorld { string AtStop { get; } bool Driving { get; } bool LegFailed { get; } string LegFrom { get; } double LegS { get; } double Speed { get; } double Soc { get; } double Now { get; } bool StartLeg(string stop); bool StopEarlyAt(string stop); void Tow(string stop); void SetCharging(bool on); }` (`LegS`: arc length along the current leg; `Speed`: m/s). This is what `ShuttleGame` implements over `TwinAutopilot` and `TwinCar`.
  - `enum ShuttleMode { Idle, Driving, Dwelling, Charging, Towed }`
  - `sealed class ShuttleBrain(IShuttleWorld w, ShuttleSettings s, ITravelModel t, ShuttleScore score)`: `ShuttleMode Mode`, `ShuttlePlan Plan`, `LowBattery Battery`, `List<RideRequest> Aboard`, `double? QuoteInsertion(RideRequest r)`, `void Accept(RideRequest r)`, `void Tick(double dt)`, `string Status` (the bottom bar's sentence, e.g. "To AB2: pick up 2", "Charging at AB1, 74 %", "Battery low: dropping off 2, then charging"), and `event Action<RideRequest> Boarded, Alighted` for the visuals.
  - `sealed class Dispatcher(IReadOnlyList<ShuttleBrain> shuttles)`: `void Offer(RideRequest r)` (the lowest quote wins; none → stays `Waiting`) and `void RetryWaiting(IEnumerable<RideRequest> waiting)` (called whenever a plan changes).
- Behaviour:
  - **At a stop:** `Dwelling` for the dwell time. Alight first, then board in order of request age, up to capacity. With `Battery != Fine`, a student boards only if `Decide` with them aboard still gives `DropOffFirst`. A request whose origin is `AtStop` boards without a leg.
  - **Next leg:** after dwelling, `StartLeg(next stop)`, which is then committed. If `StartLeg` returns false, retry each tick for 2 s, then `Tow` to that stop.
  - **Stopping on the way:** while driving, if a stop on the leg ahead (`StopsOnRoute(LegFrom, next)` beyond `LegS`) has a waiting request this shuttle can take (a seat free and `TryInsert` puts its pickup there), or a rider aboard alights there, call `StopEarlyAt(that stop)`. If it returns true, insert that stop as the new committed next stop. Otherwise the request is re-planned normally.
  - **A failed leg:** `LegFailed` → `Tow(that stop)`, then carry on as arrived.
  - **Battery:** each tick, `BatteryPolicy.Decide`; anything but `Fine` stops `QuoteInsertion` (it returns null).
  - **At the charger:** `Charging` until `LeaveChargerAt`. Students waiting at the charger may board.
  - **Idle top-up:** with an empty plan and `Soc < IdleTopUpBelow`, go to the charger.
  - **Flat:** `Soc ≤ 0.005` → `score.OnFlat()`, `Tow(ChargerStop)`, then `Charging`.
  - **Give-ups:** waiting requests older than `GiveUpAfter` → `GaveUp`, `score.OnGaveUp`.

- [ ] **Step 1: Write the failing tests** with a fake world (`FakeWorld : IShuttleWorld`, legs take `DriveTime` from the travel model, soc drains by `Distance × SocPerMetre`):

```csharp
[Test] public void ItDeliversARequest()                       // at S3, request AB2→S6: legs S3→AB2, AB2→S6; Delivered 1; Boarded/Alighted fire once each
[Test] public void ARequestAtItsOwnStopBoardsWithoutALeg()     // at S3, request S3→S6: no leg to S3, first leg is S3→S6
[Test] public void ItPoolsUpToCapacityThenLeavesTheRestWaiting() // 8 students at AB2 to S6, capacity 6: 6 board, 2 stay Waiting and are re-planned
[Test] public void LowBatteryOnTheWayToAPickupDoesNotStrand()  // battery drops on the leg to AB2: at AB2 nobody boards unless DropOffFirst holds; it then charges; Flats == 0
[Test] public void ItChargesToTheSetLevelAndLetsChargerStudentsBoard() // at AB1 charging from 0.2: leaves only at ≥ 0.95; a request AB1→S3 raised while charging is aboard when it leaves
[Test] public void AFailedLegIsTowedAndTheTripCompletes()     // FakeWorld fails the first leg: Tow called with that stop, request still delivered
[Test] public void ARefusedLegIsRetriedThenTowed()            // StartLeg returns false 3 s: Tow after 2 s
[Test] public void ItStopsForAStudentOnTheWay()                 // driving AB4→AB1 with seats free, a request S3→S6 appears while S3 is ahead: StopEarlyAt("S3") called, the student boards at S3
[Test] public void ItDrivesThroughWhenFull()                    // same, at capacity: no StopEarlyAt, the request waits
[Test] public void OverAnHourItNeverRunsFlat()                 // 3600 s simulated with the spawner at defaults: Flats == 0, Delivered > 50, no request in two states
```

- [ ] **Step 2: Run; expect FAIL**
- [ ] **Step 3: Implement `ShuttleBrain` and `Dispatcher`**
- [ ] **Step 4: Run; expect PASS**
- [ ] **Step 5: Commit** `"Shuttle: the shuttle's brain and the dispatcher seam"`

### Task 7: The game in the scene

**Files:**
- Create: `unity/RcTwin/Assets/RcTwin/Runtime/ShuttleGame.cs` (MonoBehaviour, `UnityEngine` allowed)
- Create: `unity/RcTwin/Assets/RcTwin/Runtime/StudentFigure.cs`
- Test: `unity/RcTwin/Assets/RcTwin/Tests/PlayMode/ShuttleGameTests.cs`

**Interfaces:**
- Consumes: `TwinAutopilot` (`Go`, `State`, `CurrentStop`, `targetSpeed`), `TwinCar` hooks (Task 5), `CampusRoutes`, Tasks 1–4 and 6.
- Produces:
  - `ShuttleGame : MonoBehaviour, IShuttleWorld` with `ShuttleSettings Settings`, `ShuttleBrain Brain`, `RequestSpawner Spawner`, `ShuttleScore Score`, `IReadOnlyList<RideRequest> Waiting`, and `RideRequest RaiseRequest(string origin, string destination)` (for the HUD).
  - `Tow(stop)`: `TwinCar.ResetTo` at the stop's road node, facing along the first segment of the route leaving it.
  - `SetCharging`: each physics tick, `SetCharge(soc + rate × dt)` while parked at AB1.
  - On start, `SetPackCapacity(GamePackCapacityAh(...))`; it hides `TwinCar`/`TwinAutopilot` panels (F1 toggles them) and feeds `SocMeter` from distance driven and soc used.
  - Game mode: the game runs in the player by default; `-goto <stop>` on the command line turns it off (the one-trip demo). In the editor it runs only when `ShuttleGame.RunInEditor` (a static bool, false by default) is set, so `CampusDriveTests` and the other Play Mode tests are unaffected; `ShuttleGameTests` set it before loading the scene.
  - `StudentFigure`: a capsule-and-sphere figure with a random clothing colour, idling at its stop's kerb. It walks to the shuttle's door and disappears on `Boarded`, and appears at the door and walks to the stop's kerb on `Alighted`. It has no colliders. 3 to 6 figures stand around a stop as a group.

- [ ] **Step 1: Write the failing Play Mode test**

```csharp
[UnityTest] IEnumerator TheShuttleServesTheCampus()
// Campus scene with ShuttleGame; Time.timeScale 5; raise AB2→S6 and S1→S3, run 180 game-seconds:
// Score.Delivered ≥ 2, Score.Flats == 0, every delivered request has PickedUpAt and DroppedAt set, the car's soc dropped.
[UnityTest] IEnumerator ALowBatteryShuttleChargesAtAB1()
// SetCharge(0.15) at start, one request far from AB1: within 120 game-seconds the brain charges at AB1, leaves at ≥ 0.95, Flats == 0.
```

- [ ] **Step 2: Run Play Mode; expect FAIL**
- [ ] **Step 3: Implement `ShuttleGame` and `StudentFigure`**; add `ShuttleGame` to the car in `CampusSceneBuilder` next to `TwinAutopilot` (line ~241)
- [ ] **Step 4: Run Edit + Play Mode; expect PASS**
- [ ] **Step 5: Commit** `"Shuttle game: the shuttle serves the campus in the scene"`

### Task 8: HUD, layout C

**Files:**
- Create: `unity/RcTwin/Assets/RcTwin/Runtime/ShuttleHud.cs`
- Test: `unity/RcTwin/Assets/RcTwin/Tests/Editor/ShuttleHudTests.cs`

**Interfaces:**
- Consumes: `ShuttleGame` (Task 7), `CampusRoutes` (edges, stops).
- Produces:
  - `ShuttleHud : MonoBehaviour`.
  - `static class Minimap { Rect Fit(CampusRoutes net, Rect area); Vector2 ToMap(double x, double y); string StopAt(Vector2 click, float radiusPx) }` (pure maths, tested).
- Layout:
  - Score top-left, then "Delivered n · Avg wait m:ss · Gave up n".
  - Minimap top-right, about 22 % of the screen width: roads, stop dots with waiting counts, the shuttle marker with heading, its planned path, a charger icon at AB1.
  - Bottom bar: battery gauge (percent, green/amber/red at 50/25 %, a charging pulse), seats n/6, `Brain.Status`, request chips "AB2 → S6 · 0:42" (assigned ones marked).
  - Click a stop (minimap or its 3D marker) → a destination picker → `RaiseRequest`.
  - Settings panel (gear button) for every `ShuttleSettings` default, plus Pause and a ×1/×2/×4 time scale.

- [ ] **Step 1: Write the failing tests:** `Minimap.Fit` keeps the campus aspect ratio inside the rect, `ToMap` of every stop lies inside it, `StopAt` returns the stop under a click within 8 px and null elsewhere.
- [ ] **Step 2: Run; expect FAIL**
- [ ] **Step 3: Implement `Minimap` and `ShuttleHud`** (immediate-mode GUI, as the existing panels; minimap lines with `GL` in `OnGUI` on `Repaint`)
- [ ] **Step 4: Run; expect PASS.** Build the player and take screenshots (`-goto` is replaced by the game: run `RcTwinCampus.exe -screenshot <png> -at 40`): the score, minimap and bottom bar are visible and not overlapping at 1280×720 and 1920×1080
- [ ] **Step 5: Commit** `"Shuttle game: HUD in layout C, click a stop to raise a request"`

### Task 9: The look, and the player

**Files:**
- Modify: `unity/RcTwin/Assets/RcTwin/Editor/CampusSceneBuilder.cs` (the car's body, stops, AB1)
- Create: `unity/RcTwin/Assets/RcTwin/Editor/ShuttleLook.cs` (editor-side builders for the meshes and materials)
- Test: `unity/RcTwin/Assets/RcTwin/Tests/Editor/ShuttleLookTests.cs`

**Interfaces:**
- Consumes: the car's `body` transform and `TwinCar.Params` (track, wheelbase), the stops from `CampusRoutes`.
- Produces: `ShuttleLook.BuildShuttleBody(Transform body, RcTwinNative.Params p)`, `ShuttleLook.BuildShelter(Vector3 at, float headingDeg)` and `ShuttleLook.BuildCharger(Vector3 at, float headingDeg)`.
  - The shuttle body is white, with a red waist stripe, dark tinted windows, a door on the kerb (left) side and lights. It is built from primitives with all colliders removed, with shadows on.

- [ ] **Step 1: Write the failing tests:** after `CampusSceneBuilder.Build()`, the car's body has no `Collider` in its children; the car's `TwinCar` contact half-width and `Params` are identical to before the build; every stop has a shelter; AB1 has a charger.
- [ ] **Step 2: Run; expect FAIL**
- [ ] **Step 3: Implement `ShuttleLook` and call it from `CampusSceneBuilder`**
- [ ] **Step 4: Run Edit + Play Mode; expect PASS.** Build the player (`CampusSceneBuilder.BuildPlayerBatch`) and take a screenshot mid-game: a white shuttle with students boarding at a shelter
- [ ] **Step 5: Revert the regenerated tracked files** (Global Constraints), then commit `"Shuttle game: a campus shuttle, stop shelters and the AB1 charger"`

### Task 10: Docs, and the pull request

**Files:**
- Modify: `README.md` (the campus player section: what the game is, the controls, F1 for the debug panels)
- Modify: `Docs/NEXT_STEPS.md` (a done item for the shuttle game; open items: several shuttles, user-chosen charger)

- [ ] **Step 1: Update both files**
- [ ] **Step 2: Run every suite:** `sim_py/test_*.py`, `tools/test_*.py`, Unity Edit Mode and Play Mode. All pass
- [ ] **Step 3: Commit and push `twin/shuttle`; open a PR to `main`** (it stacks on #24, which is in its history) with the screenshots
