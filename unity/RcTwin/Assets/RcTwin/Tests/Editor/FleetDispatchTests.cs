// Edit Mode: several shuttles on fake worlds sharing one dispatcher
// (Docs/specs/2026-10-25-shuttle-fleet-design.md, "The fleet", "Chargers").
using System;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;

namespace RcTwin.Tests
{
    public class FleetDispatchTests
    {
        private static readonly CampusRoutes Net = ShuttleRequestTests.Network();

        private sealed class Fleet
        {
            public readonly ShuttleSettings S = new ShuttleSettings();
            public readonly RouteTravelModel T;
            public readonly ShuttleScore Score;
            public readonly List<ShuttleBrainTests.FakeWorld> W = new List<ShuttleBrainTests.FakeWorld>();
            public readonly List<ShuttleBrain> B = new List<ShuttleBrain>();
            public readonly List<ShuttleScore> BusScores = new List<ShuttleScore>();
            public readonly Dispatcher Disp = new Dispatcher(new ShuttleBrain[0]);
            public readonly RequestSpawner Spawner;

            public Fleet(params string[] at)
            {
                T = new RouteTravelModel(Net, S.Cruise);
                Score = new ShuttleScore(S);
                foreach (var stop in at)
                {
                    var w = new ShuttleBrainTests.FakeWorld(T, stop);
                    var own = new ShuttleScore(S, Score);
                    var b = new ShuttleBrain(w, S, T, own, w.SocPerMetre);
                    BusScores.Add(own);
                    var me = b;
                    b.PickCharger = from => BatteryPolicy.PickCharger(S, T, from,
                        B.Where(x => x != me && x.Charger != null).Select(x => x.Charger).ToList());
                    W.Add(w);
                    B.Add(b);
                    Disp.Add(b);
                }
                Spawner = new RequestSpawner(ShuttleRequestTests.Stops(Net), 5) { RatePerSecond = 1.0 / S.RequestInterval };
            }

            public RideRequest Raise(string o, string d)
            {
                var r = Spawner.Raise(o, d, W[0].Now);
                Disp.Offer(r);
                return r;
            }

            public int Holding(RideRequest r) => B.FindIndex(b => b.Plan.Stops.Any(st => st.Board.Contains(r) || st.Alight.Contains(r)) || b.Aboard.Contains(r));

            public void Run(double seconds, bool spawn = false, System.Func<bool> until = null, double dt = 0.1)
            {
                for (double t = 0.0; t < seconds; t += dt)
                {
                    foreach (var w in W) w.Step(dt);
                    if (spawn) foreach (var r in Spawner.Tick(W[0].Now, dt)) Disp.Offer(r);
                    foreach (var b in B) b.Tick(dt);
                    Disp.Tick(W[0].Now, Score);
                    if (until != null && until()) return;
                }
            }
        }

        [Test]
        public void TheNearerBusTakesTheRequest()
        {
            var f = new Fleet("S3", "AB2");
            var r = f.Raise("AB2", "S6");
            Assert.AreEqual(1, f.Holding(r), "the bus standing at AB2");
            var q = f.Raise("S3", "S1");
            Assert.AreEqual(0, f.Holding(q), "the bus standing at S3");
        }

        [Test]
        public void ARetiringBusTakesNothingAndGivesItsPickupsBack()
        {
            var f = new Fleet("AB2", "S3");
            var r = f.Raise("AB2", "S6");
            Assert.AreEqual(0, f.Holding(r));
            f.B[0].Retire();
            Assert.AreEqual(RequestState.Waiting, r.State, "back to waiting");
            Assert.IsNull(f.B[0].QuoteInsertion(f.Raise("AB2", "S1")), "a retiring bus quotes nothing");
            f.Run(2.0);                                         // the dispatcher retries each second
            Assert.AreEqual(1, f.Holding(r), "the other bus took it");
            Assert.IsTrue(f.B[0].Retired, "nothing aboard, nothing planned: it can leave");
        }

        [Test]
        public void ARetiringBusDeliversWhoIsAboardFirst()
        {
            var f = new Fleet("AB2", "S3");
            var r = f.Raise("AB2", "S6");
            f.Run(30.0, until: () => r.State == RequestState.Riding);
            Assert.AreEqual(RequestState.Riding, r.State);
            f.B[0].Retire();
            Assert.IsFalse(f.B[0].Retired);
            f.Run(200.0, until: () => r.State == RequestState.Delivered);
            Assert.AreEqual(RequestState.Delivered, r.State);
            f.Run(1.0);
            Assert.IsTrue(f.B[0].Retired);
        }

        [Test]
        public void TwoBusesServeTheCampusTogether()
        {
            var f = new Fleet("S3", "AB3");
            f.S.RequestInterval = 8.0;
            f.Spawner.RatePerSecond = 1.0 / f.S.RequestInterval;
            f.Run(1200.0, spawn: true);
            Assert.Greater(f.Score.Delivered, 100);
            Assert.Greater(f.BusScores[0].Delivered, 20, "Bus 1 works");
            Assert.Greater(f.BusScores[1].Delivered, 20, "Bus 2 works");
            Assert.AreEqual(f.BusScores[0].Delivered + f.BusScores[1].Delivered, f.Score.Delivered,
                            "each delivery counted once, and in the fleet's score too");
            Assert.AreEqual(f.BusScores[0].Points + f.BusScores[1].Points -
                            f.Score.GaveUp * f.S.GaveUpPenalty, f.Score.Points, "the fleet's points: the buses' and the give-ups'");
            Assert.AreEqual(0, f.Score.Flats);
        }

        [Test]
        public void ALowBatteryBusGoesToTheNearestCharger()
        {
            var f = new Fleet("AB2");
            f.W[0].Soc = 0.12;
            f.Run(120.0, until: () => f.B[0].Mode == ShuttleMode.Charging);
            Assert.AreEqual(ShuttleMode.Charging, f.B[0].Mode, f.B[0].Status);
            Assert.AreEqual("AB3", f.W[0].AtStop, "AB3 is 24 m from AB2, AB1 43 m");
            Assert.AreEqual("AB3", f.B[0].Charger);
        }

        [Test]
        public void ALowPlanThatNoLongerReachesItsChargerChargesFirst()
        {
            // Dropping off first was planned, then the charge fell faster than planned (a
            // turn round in reverse, a wait): it goes to the nearest charger before it runs flat.
            var f = new Fleet("S3");
            var r = f.Raise("S3", "S6");
            f.Run(10.0, until: () => r.State == RequestState.Riding);
            Assert.AreEqual(RequestState.Riding, r.State);
            var w = f.W[0];
            double need = (f.T.Distance("S3", "S6") + f.T.Distance("S6", "AB1")) * w.SocPerMetre * f.S.EnergySafety;
            w.Soc = need + 0.07;                                 // under the full reserve, within half of it
            f.Run(0.2);
            Assert.AreEqual(LowBattery.DropOffFirst, f.B[0].Battery, f.B[0].Status);
            CollectionAssert.AreEqual(new[] { "S6", "AB1" }, f.B[0].Plan.Stops.Select(st => st.Stop).ToArray());
            w.Soc = need - 0.02;                                 // no longer enough for S6, then AB1
            f.Run(0.2);
            Assert.AreEqual(LowBattery.ChargeNow, f.B[0].Battery, f.B[0].Status);
            var stops = f.B[0].Plan.Stops.Select(st => st.Stop).ToList();
            Assert.IsTrue(f.B[0].Plan.Stops.Exists(st => st.Charge), string.Join(", ", stops));
            Assert.Less(stops.IndexOf(f.B[0].Plan.Stops.Find(st => st.Charge).Stop), Math.Max(1, stops.IndexOf("S6") + 1),
                        "the charger before the drop-off, or the drop-off it is already committed to: " + string.Join(", ", stops));
            Assert.IsTrue(f.B[0].Plan.Stops.Any(st => st.Alight.Contains(r)), "the rider is still dropped off, later");
        }

        [Test]
        public void ATakenChargerIsPassedOver()
        {
            var t = new RouteTravelModel(Net, 2.0);
            var s = new ShuttleSettings();
            Assert.AreEqual("AB1", BatteryPolicy.PickCharger(s, t, "S4", new string[0]));
            Assert.AreEqual("AB3", BatteryPolicy.PickCharger(s, t, "S4", new[] { "AB1" }), "the free one, though farther");
            Assert.AreEqual("AB1", BatteryPolicy.PickCharger(s, t, "S4", new[] { "AB1", "AB3" }), "both taken: the nearest");
            Assert.AreEqual("AB3", BatteryPolicy.NearestCharger(s, t, "AB2"));
        }

        [Test]
        public void TwoLowBusesChargeAtDifferentChargers()
        {
            var f = new Fleet("S4", "S5");
            f.W[0].Soc = 0.3;
            f.W[1].Soc = 0.3;
            f.Run(200.0, until: () => f.B.All(b => b.Mode == ShuttleMode.Charging));
            Assert.IsTrue(f.B.All(b => b.Mode == ShuttleMode.Charging), string.Join("; ", f.B.Select(b => b.Status)));
            CollectionAssert.AreEquivalent(new[] { "AB1", "AB3" }, f.W.Select(w => w.AtStop).ToArray());
        }

        [Test]
        public void AnIdleBusMakesRoomOnlyWhenIdle()
        {
            var f = new Fleet("S3");
            Assert.IsTrue(f.B[0].MakeRoom("S1"));
            Assert.AreEqual("To S1: making room", f.B[0].Status);
            f.Run(60.0, until: () => f.W[0].AtStop == "S1");
            Assert.AreEqual("S1", f.W[0].AtStop);
            f.Run(5.0);
            Assert.AreEqual(ShuttleMode.Idle, f.B[0].Mode, "it waits there");
            var r = f.Raise("AB2", "S6");
            Assert.AreEqual(0, f.Holding(r));
            Assert.IsFalse(f.B[0].MakeRoom("S4"), "busy: it does not move for room");
        }
    }
}
