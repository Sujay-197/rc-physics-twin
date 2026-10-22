// Edit Mode: the shuttle's brain on a fake world, where legs take their drive
// time and drain the battery by distance, with no physics
// (Docs/specs/2026-10-21-campus-shuttle-design.md, "The shuttle's plan", "Battery").
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;

namespace RcTwin.Tests
{
    public class ShuttleBrainTests
    {
        private static readonly CampusRoutes Net = ShuttleRequestTests.Network();

        internal sealed class FakeWorld : IShuttleWorld
        {
            private string _at;
            /// <summary>Rolling slowly past this stop: AtStop reads null, yet a leg starts from it.</summary>
            public string RollingAt;
            public string AtStop { get => RollingAt != null ? null : _at; set => _at = value; }
            public bool Driving { get; private set; }
            public bool LegFailed { get; private set; }
            public string LegFrom { get; private set; }
            public string LegTo { get; private set; }
            public double LegS { get; private set; }
            public double Speed => Driving ? Cruise : 0.0;
            public double Soc { get; set; } = 1.0;
            public double Now { get; private set; }
            public bool Charging { get; private set; }

            public double Cruise = 2.0, SocPerMetre = 1.0 / 600.0, ChargeRate = 0.85 / 30.0;
            public double RefuseLegsUntil = -1.0;
            public bool FailNextLeg;
            public readonly List<(string from, string to)> Legs = new List<(string, string)>();
            public readonly List<(string stop, double at)> Tows = new List<(string, double)>();
            public readonly List<string> EarlyStops = new List<string>();
            public double SocAtLegStart = double.NaN;

            private readonly ITravelModel _t;
            private double _length, _failAt;

            public FakeWorld(ITravelModel t, string at) { _t = t; AtStop = at; }

            public bool StartLeg(string stop)
            {
                string origin = RollingAt ?? AtStop;
                RollingAt = null;
                if (Now < RefuseLegsUntil || origin == null || Driving) return false;
                Legs.Add((origin, stop));
                LegFrom = origin;
                LegTo = stop;
                _length = _t.Distance(origin, stop);
                _failAt = FailNextLeg ? 0.5 * _length : double.PositiveInfinity;
                FailNextLeg = false;
                LegS = 0.0;
                Driving = true;
                AtStop = null;
                SocAtLegStart = Soc;
                return true;
            }

            public bool StopEarlyAt(string stop)
            {
                if (!Driving) return false;
                foreach (var (x, s) in _t.StopsOnRoute(LegFrom, LegTo))
                    if (x == stop && s - LegS >= Speed * Speed / 2.4 + 0.5)
                    {
                        LegTo = stop;
                        _length = s;
                        EarlyStops.Add(stop);
                        return true;
                    }
                return false;
            }

            public void Tow(string stop)
            {
                Tows.Add((stop, Now));
                Driving = false;
                LegFailed = false;
                AtStop = stop;
                LegS = 0.0;
            }

            public void SetCharging(bool on) => Charging = on;

            public void Step(double dt)
            {
                Now += dt;
                if (Driving)
                {
                    double d = Cruise * dt;
                    LegS += d;
                    Soc = System.Math.Max(0.0, Soc - d * SocPerMetre);
                    if (LegS >= _failAt) { Driving = false; LegFailed = true; }
                    else if (LegS >= _length) { Driving = false; AtStop = LegTo; }
                }
                else if (Charging) Soc = System.Math.Min(1.0, Soc + ChargeRate * dt);
            }
        }

        internal sealed class Sim
        {
            public readonly ShuttleSettings S;
            public readonly RouteTravelModel T;
            public readonly FakeWorld W;
            public readonly ShuttleScore Score;
            public readonly ShuttleBrain Brain;
            public readonly Dispatcher Disp;
            public readonly RequestSpawner Spawner;
            public bool Spawn;

            public Sim(string at, ShuttleSettings s = null, double soc = 1.0)
            {
                S = s ?? new ShuttleSettings();
                T = new RouteTravelModel(Net, S.Cruise);
                W = new FakeWorld(T, at) { Soc = soc };
                Score = new ShuttleScore(S);
                Brain = new ShuttleBrain(W, S, T, Score, W.SocPerMetre);
                Disp = new Dispatcher(new[] { Brain });
                Spawner = new RequestSpawner(ShuttleRequestTests.Stops(Net), 11) { RatePerSecond = 1.0 / S.RequestInterval };
            }

            public RideRequest Raise(string o, string d)
            {
                var r = Spawner.Raise(o, d, W.Now);
                Disp.Offer(r);
                return r;
            }

            public void Run(double seconds, System.Func<bool> until = null, double dt = 0.1)
            {
                for (double t = 0.0; t < seconds; t += dt)
                {
                    W.Step(dt);
                    if (Spawn) foreach (var r in Spawner.Tick(W.Now, dt)) Disp.Offer(r);
                    Brain.Tick(dt);
                    Disp.Tick(W.Now, Score);
                    if (until != null && until()) return;
                }
            }
        }

        [Test]
        public void ItDeliversARequest()
        {
            var sim = new Sim("S3");
            int boarded = 0, alighted = 0;
            sim.Brain.Boarded += _ => boarded++;
            sim.Brain.Alighted += _ => alighted++;
            var r = sim.Raise("AB2", "S6");
            Assert.AreEqual(RequestState.Assigned, r.State);
            sim.Run(200, () => r.State == RequestState.Delivered);
            Assert.AreEqual(RequestState.Delivered, r.State);
            CollectionAssert.AreEqual(new[] { ("S3", "AB2"), ("AB2", "S6") }, sim.W.Legs.Take(2).ToArray());
            Assert.AreEqual(1, sim.Score.Delivered);
            Assert.AreEqual((1, 1), (boarded, alighted));
            Assert.IsFalse(double.IsNaN(r.PickedUpAt) || double.IsNaN(r.DroppedAt));
        }

        [Test]
        public void ARequestAtItsOwnStopBoardsWithoutALeg()
        {
            var sim = new Sim("S3");
            var r = sim.Raise("S3", "S6");
            sim.Run(200, () => r.State == RequestState.Delivered);
            Assert.AreEqual(RequestState.Delivered, r.State);
            Assert.AreEqual(("S3", "S6"), sim.W.Legs[0], "no leg to S3, it was there");
        }

        [Test]
        public void ItPoolsUpToCapacityThenLeavesTheRestWaiting()
        {
            var sim = new Sim("S3");
            var rs = Enumerable.Range(0, 8).Select(_ => sim.Raise("AB2", "S6")).ToList();
            sim.Run(200, () => sim.W.Legs.Count >= 2);      // left AB2
            Assert.AreEqual(("AB2", "S6"), sim.W.Legs[1]);
            Assert.AreEqual(6, sim.Brain.Aboard.Count);
            var rest = rs.Where(r => r.State != RequestState.Riding).ToList();
            Assert.AreEqual(2, rest.Count);
            Assert.IsTrue(rest.All(r => r.State == RequestState.Waiting || r.State == RequestState.Assigned), "waiting, or planned for a later pickup");
            sim.Run(400, () => rs.All(r => r.State == RequestState.Delivered));
            Assert.IsTrue(rs.All(r => r.State == RequestState.Delivered), "the two are re-planned and delivered too");
        }

        [Test]
        public void LowBatteryOnTheWayToAPickupDoesNotStrand()
        {
            // Just enough charge to accept AB2 to S6 (and the way on to AB1); the car
            // uses 1.6 times what the shuttle expects, so the plan turns infeasible
            // on the way to the pickup.
            var t = new RouteTravelModel(Net, 2.0);
            var set = new ShuttleSettings();
            double metres = t.Distance("S3", "AB2") + t.Distance("AB2", "S6") + t.Distance("S6", "AB1");
            var sim = new Sim("S3", soc: metres / 600.0 * set.EnergySafety + set.Reserve + 0.005);
            sim.W.SocPerMetre *= 1.6;
            var r = sim.Raise("AB2", "S6");
            Assert.AreEqual(RequestState.Assigned, r.State);
            sim.Run(600, () => sim.W.Charging && sim.W.Soc > 0.9);
            Assert.AreEqual(0, sim.Score.Flats);
            Assert.IsTrue(sim.W.Charging, "it reached the charger");
            Assert.AreEqual("AB1", sim.W.AtStop);
            Assert.AreNotEqual(LowBattery.Fine, sim.Brain.Battery);
            Assert.IsTrue(r.State == RequestState.Waiting || r.State == RequestState.Delivered || r.State == RequestState.Riding);
        }

        [Test]
        public void ItChargesToTheSetLevelAndLetsChargerStudentsBoard()
        {
            var sim = new Sim("AB1", soc: 0.2);
            sim.Run(2);
            Assert.AreEqual(ShuttleMode.Charging, sim.Brain.Mode);
            var r = sim.Raise("AB1", "S3");
            sim.Run(200, () => sim.W.Legs.Count > 0);
            Assert.GreaterOrEqual(sim.W.SocAtLegStart, sim.S.LeaveChargerAt - 1e-9, "left only at 95 %");
            Assert.AreEqual(RequestState.Riding, r.State, "boarded while it charged");
            Assert.AreEqual(("AB1", "S3"), sim.W.Legs[0]);
        }

        [Test]
        public void AFailedLegIsTowedAndTheTripCompletes()
        {
            var sim = new Sim("S3");
            sim.W.FailNextLeg = true;
            var r = sim.Raise("AB2", "S6");
            sim.Run(300, () => r.State == RequestState.Delivered);
            Assert.AreEqual("AB2", sim.W.Tows[0].stop, "towed to the stop it was driving to");
            Assert.AreEqual(RequestState.Delivered, r.State);
        }

        [Test]
        public void ARefusedLegIsRetriedThenTowed()
        {
            var sim = new Sim("S3");
            sim.W.RefuseLegsUntil = 3.0;
            sim.Raise("AB2", "S6");
            sim.Run(5);
            Assert.AreEqual(1, sim.W.Tows.Count);
            Assert.AreEqual("AB2", sim.W.Tows[0].stop);
            Assert.That(sim.W.Tows[0].at, Is.InRange(1.9, 2.4), "after 2 s of refusals");
        }

        private static Sim OnTheWayToAB1(int capacity, out RideRequest rider)
        {
            var sim = new Sim("AB4", new ShuttleSettings { Capacity = capacity });
            rider = sim.Raise("AB4", "AB1");
            var rd = rider;
            sim.Run(30, () => sim.W.Driving && rd.State == RequestState.Riding);
            Assert.AreEqual(("AB4", "AB1"), sim.W.Legs[0]);
            return sim;
        }

        [Test]
        public void ItStopsForAStudentOnTheWay()
        {
            var sim = OnTheWayToAB1(6, out _);
            var r = sim.Raise("S3", "S6");
            sim.Run(100, () => r.State == RequestState.Riding);
            CollectionAssert.Contains(sim.W.EarlyStops, "S3");
            Assert.AreEqual(RequestState.Riding, r.State, "boarded at S3 on the way");
            Assert.AreEqual("S3", sim.W.AtStop, "stopped there");
            Assert.AreEqual(1, sim.W.Legs.Count, "within the first leg, cut short"); 
        }

        [Test]
        public void StoppingOnTheWayBoardsOnlyTheSeatsItHas()
        {
            // Two seats, one rider to AB1; three students at S3 (on the way) are
            // planned after AB1. Stopping early at S3 has one seat: one boards
            // there, the others keep their later pickup, and the battery is fine.
            var sim = OnTheWayToAB1(2, out _);
            var rs = Enumerable.Range(0, 3).Select(_ => sim.Raise("S3", "S6")).ToList();
            sim.Run(100, () => sim.W.EarlyStops.Contains("S3"));
            CollectionAssert.Contains(sim.W.EarlyStops, "S3");
            Assert.AreEqual("S3", sim.Brain.Plan.Stops[0].Stop);
            Assert.LessOrEqual(sim.Brain.Plan.Stops[0].Board.Count, 1, "one free seat at S3");
            sim.Run(5);
            Assert.AreEqual(LowBattery.Fine, sim.Brain.Battery, "a full shuttle is not a low battery");
            sim.Run(600, () => rs.All(r => r.State == RequestState.Delivered));
            Assert.IsTrue(rs.All(r => r.State == RequestState.Delivered));
        }

        [Test]
        public void ALegStartedWhileRollingKnowsWhereItLeft()
        {
            // The car creeps past S3 (AtStop null) when a request comes in; the
            // autopilot still starts the leg from S3.
            var sim = new Sim("S3");
            sim.W.RollingAt = "S3";
            var r = sim.Raise("AB2", "S6");
            Assert.DoesNotThrow(() => sim.Run(300, () => r.State == RequestState.Delivered));
            Assert.AreEqual(("S3", "AB2"), sim.W.Legs[0]);
            Assert.AreEqual(RequestState.Delivered, r.State);
        }

        [Test]
        public void ItDrivesThroughWhenFull()
        {
            var sim = OnTheWayToAB1(1, out var rider);
            var r = sim.Raise("S3", "S6");
            sim.Run(100, () => rider.State == RequestState.Delivered);
            CollectionAssert.DoesNotContain(sim.W.EarlyStops, "S3");
            Assert.AreEqual(RequestState.Delivered, rider.State);
            Assert.AreNotEqual(RequestState.Riding, r.State);
        }

        [Test]
        public void OverAnHourItNeverRunsFlat()
        {
            var sim = new Sim("S3") { Spawn = true };
            for (int k = 0; k < 36; k++)
            {
                sim.Run(100);
                var planned = new HashSet<RideRequest>(sim.Brain.Plan.Stops.SelectMany(st => st.Board));
                foreach (var r in sim.Spawner.All)
                {
                    bool aboard = sim.Brain.Aboard.Contains(r);
                    Assert.AreEqual(r.State == RequestState.Riding, aboard, $"{r}: riding iff aboard");
                    if (r.State == RequestState.Assigned) Assert.IsTrue(planned.Contains(r), $"{r}: assigned iff planned");
                    if (r.State == RequestState.Waiting || r.State == RequestState.Delivered || r.State == RequestState.GaveUp)
                        Assert.IsFalse(planned.Contains(r) || aboard, $"{r}: not planned");
                }
            }
            Assert.AreEqual(0, sim.Score.Flats);
            Assert.Greater(sim.Score.Delivered, 50);
        }
    }
}
