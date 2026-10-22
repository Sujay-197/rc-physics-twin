// Edit Mode: the shuttle's plan and cheapest insertion
// (Docs/specs/2026-10-21-campus-shuttle-design.md, "The shuttle's plan").
using System;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;

namespace RcTwin.Tests
{
    public class ShuttlePlanTests
    {
        private static readonly CampusRoutes Net = ShuttleRequestTests.Network();
        private static readonly RouteTravelModel T = new RouteTravelModel(Net, 2.0);
        private static int _ids = 1000;

        private static RideRequest Req(string o, string d, double at = 0.0, RequestState st = RequestState.Waiting) =>
            new RideRequest { Id = _ids++, Origin = o, Destination = d, RaisedAt = at, State = st };

        private static PlanContext At(string from, int onboard = 0, double soc = 1.0, double socPerMetre = 1e-4) =>
            new PlanContext { From = from, ArrivedFrom = null, Now = 0.0, Onboard = onboard, Soc = soc, SocPerMetre = socPerMetre };

        private static double Legs(ShuttlePlan p, string from)
        {
            double sum = 0.0;
            string at = from;
            foreach (var st in p.Stops) { sum += T.Distance(at, st.Stop); at = st.Stop; }
            return sum;
        }

        [Test]
        public void AnEmptyPlanTakesTheRequestDirectly()
        {
            var r = Req("AB2", "S6");
            var p = new ShuttlePlan().TryInsert(r, At("S3"), new ShuttleSettings(), T, out double added);
            Assert.IsNotNull(p);
            CollectionAssert.AreEqual(new[] { "AB2", "S6" }, p.Stops.Select(s => s.Stop).ToArray());
            CollectionAssert.Contains(p.Stops[0].Board, r);
            CollectionAssert.Contains(p.Stops[1].Alight, r);
            Assert.Greater(added, 0.0);
        }

        [Test]
        public void ARequestOnTheWayIsPooledNotDetoured()
        {
            var rider = Req("S3", "AB1", st: RequestState.Riding);
            var plan = new ShuttlePlan();
            plan.Stops.Add(new PlanStop { Stop = "AB1", Alight = { rider } });
            var on = T.StopsOnRoute("S3", "AB1").Select(x => x.stop).ToList();
            Assert.GreaterOrEqual(on.Count, 1, "S3 to AB1 passes another stop");
            var r = Req(on[0], on.Count > 1 ? on[1] : "AB1");
            var p = plan.TryInsert(r, At("S3", onboard: 1), new ShuttleSettings(), T, out _);
            Assert.IsNotNull(p);
            Assert.LessOrEqual(p.Stops.Count, 3);
            Assert.AreEqual(T.Distance("S3", "AB1"), Legs(p, "S3"), 1e-6, "no detour");
            Assert.AreEqual("AB1", p.Stops.Last().Stop);
        }

        [Test]
        public void CapacityIsNeverExceeded()
        {
            ShuttlePlan Full()
            {
                var plan = new ShuttlePlan();
                plan.Stops.Add(new PlanStop { Stop = "S6", Alight = { Req("S3", "S6", st: RequestState.Riding), Req("S3", "S6", st: RequestState.Riding) } });
                return plan;
            }
            var third = Req("S3", "S6");
            var two = Full().TryInsert(third, At("S3", onboard: 2), new ShuttleSettings { Capacity = 2 }, T, out _);
            Assert.IsNotNull(two, "it comes back for them once the riders are off");
            int board = two.Stops.FindIndex(s => s.Board.Contains(third));
            int firstDrop = two.Stops.FindIndex(s => s.Alight.Count == 2);
            Assert.Greater(board, firstDrop, "picked up only after the two riders got off");
            var three = Full().TryInsert(third, At("S3", onboard: 2), new ShuttleSettings { Capacity = 3 }, T, out _);
            Assert.IsNotNull(three);
            Assert.AreEqual("S3", three.Stops[0].Stop, "a seat free: it boards where the shuttle is");
            CollectionAssert.Contains(three.Stops[0].Board, third);
        }

        [Test]
        public void ThePickupComesBeforeTheDropOff()
        {
            var spawner = new RequestSpawner(ShuttleRequestTests.Stops(Net), 3);
            var names = Net.Stops.Keys.OrderBy(x => x, StringComparer.Ordinal).ToList();
            var rng = new Random(5);
            var s = new ShuttleSettings { Capacity = 4 };
            var plan = new ShuttlePlan();
            var c = At("S3");
            var aboard = new HashSet<RideRequest>();
            for (int k = 0; k < 200; k++)
            {
                string o = names[rng.Next(names.Count)], d;
                do d = names[rng.Next(names.Count)]; while (Net.Stops[d].node == Net.Stops[o].node);
                var r = spawner.Raise(o, d, k);
                var next = plan.TryInsert(r, c, s, T, out _);
                if (next != null) plan = next;
                for (int i = 0; i < plan.Stops.Count; i++)
                {
                    foreach (var b in plan.Stops[i].Board)
                        Assert.Greater(plan.Stops.FindIndex(x => x.Alight.Contains(b)), i, $"{b}: drop after pickup");
                    foreach (var a in plan.Stops[i].Alight)
                        Assert.IsTrue(aboard.Contains(a) || plan.Stops.Take(i).Any(x => x.Board.Contains(a)), $"{a}: aboard or picked up first");
                }
                Assert.IsTrue(plan.Feasible(c, s, T));
                if (plan.Stops.Count > 0)
                {
                    // The shuttle drives to the next stop.
                    var st = plan.Stops[0];
                    plan.Stops.RemoveAt(0);
                    foreach (var a in st.Alight) aboard.Remove(a);
                    foreach (var b in st.Board) aboard.Add(b);
                    c = new PlanContext { From = st.Stop, ArrivedFrom = c.From, Now = k + 1, Onboard = aboard.Count, Soc = 1.0, SocPerMetre = 1e-4 };
                }
            }
        }

        [Test]
        public void APlanThatCannotReachTheChargerIsRefused()
        {
            var s = new ShuttleSettings();
            double metres = T.Distance("S3", "AB2") + T.Distance("AB2", "S6") + T.Distance("S6", "AB1");
            double perMetre = 0.05 / (metres * s.EnergySafety);
            var low = new ShuttlePlan().TryInsert(Req("AB2", "S6"), At("S3", soc: 0.12, socPerMetre: perMetre), s, T, out _);
            Assert.IsNull(low, "0.12 − 0.10 reserve leaves 0.02, the trip and the way to AB1 need 0.05");
            var ok = new ShuttlePlan().TryInsert(Req("AB2", "S6"), At("S3", soc: 0.20, socPerMetre: perMetre), s, T, out _);
            Assert.IsNotNull(ok);
            Assert.AreEqual(0.05, ok.SocNeeded(At("S3", soc: 0.20, socPerMetre: perMetre), s, T), 1e-9);
        }
    }
}
