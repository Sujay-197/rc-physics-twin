// Edit Mode: the shuttle's battery policy and score
// (Docs/specs/2026-10-21-campus-shuttle-design.md, "Battery" and "Score").
using System.Linq;
using NUnit.Framework;

namespace RcTwin.Tests
{
    public class ShuttleBatteryScoreTests
    {
        private static readonly CampusRoutes Net = ShuttleRequestTests.Network();
        private static readonly RouteTravelModel T = new RouteTravelModel(Net, 2.0);
        private const double PerMetre = 1e-3;

        private static RideRequest Req(string o, string d, RequestState st) =>
            new RideRequest { Id = 1, Origin = o, Destination = d, State = st };

        private static PlanContext At(string from, int onboard, double soc) =>
            new PlanContext { From = from, Onboard = onboard, Soc = soc, SocPerMetre = PerMetre };

        // Riders aboard at S3, both getting off at S6.
        private static ShuttlePlan ToS6(out RideRequest rider)
        {
            rider = Req("S3", "S6", RequestState.Riding);
            var p = new ShuttlePlan();
            p.Stops.Add(new PlanStop { Stop = "S6", Alight = { rider } });
            return p;
        }

        private static double Need(params string[] stops)
        {
            double m = 0.0;
            for (int i = 1; i < stops.Length; i++) m += T.Distance(stops[i - 1], stops[i]);
            return m * PerMetre * new ShuttleSettings().EnergySafety;
        }

        [Test]
        public void TheGameBatteryLastsTheSetMinutes() =>
            Assert.AreEqual(0.01568, BatteryPolicy.GamePackCapacityAh(new ShuttleSettings(), BatteryPolicy.SeedWhPerMetre, 11.1), 1e-5);

        [Test]
        public void ChargingTakesTheSetTime() =>
            Assert.AreEqual(0.85 / 30.0, BatteryPolicy.ChargeRatePerSecond(new ShuttleSettings()), 1e-12);

        [Test]
        public void AFeasiblePlanIsFine()
        {
            var p = ToS6(out _);
            Assert.AreEqual(LowBattery.Fine, BatteryPolicy.Decide(p, At("S3", 1, 1.0), new ShuttleSettings(), T, out var next));
            Assert.AreSame(p, next);
        }

        [Test]
        public void LowBatteryDropsOffFirstWhenItCan()
        {
            var p = ToS6(out var rider);
            double soc = Need("S3", "S6", "AB1") + 0.07;   // under the full 10 % reserve, within half of it
            Assert.AreEqual(LowBattery.DropOffFirst, BatteryPolicy.Decide(p, At("S3", 1, soc), new ShuttleSettings(), T, out var next));
            CollectionAssert.AreEqual(new[] { "S6", "AB1" }, next.Stops.Select(s => s.Stop).ToArray());
            CollectionAssert.Contains(next.Stops[0].Alight, rider);
        }

        [Test]
        public void LowBatteryChargesFirstWhenItMust()
        {
            var p = ToS6(out var rider);
            double soc = Need("S3", "S6", "AB1") + 0.03;
            Assert.AreEqual(LowBattery.ChargeNow, BatteryPolicy.Decide(p, At("S3", 1, soc), new ShuttleSettings(), T, out var next));
            CollectionAssert.AreEqual(new[] { "AB1", "S6" }, next.Stops.Select(s => s.Stop).ToArray());
            CollectionAssert.Contains(next.Stops[1].Alight, rider);
        }

        [Test]
        public void PickupsNotAboardAreReleased()
        {
            var rider = Req("S3", "S6", RequestState.Riding);
            var waiting = Req("AB2", "AB4", RequestState.Assigned);
            var p = new ShuttlePlan();
            p.Stops.Add(new PlanStop { Stop = "AB2", Board = { waiting } });
            p.Stops.Add(new PlanStop { Stop = "S6", Alight = { rider } });
            p.Stops.Add(new PlanStop { Stop = "AB4", Alight = { waiting } });
            double soc = Need("S3", "S6", "AB1") + 0.07;
            Assert.AreNotEqual(LowBattery.Fine, BatteryPolicy.Decide(p, At("S3", 1, soc), new ShuttleSettings(), T, out var next));
            Assert.AreEqual(RequestState.Waiting, waiting.State);
            Assert.IsFalse(next.Stops.Any(s => s.Board.Contains(waiting) || s.Alight.Contains(waiting)));
            Assert.AreEqual(RequestState.Riding, rider.State);
        }

        [Test]
        public void TheSocMeterAveragesTowardWhatItSees()
        {
            var m = new SocMeter(1e-4);
            Assert.AreEqual(1e-4, m.SocPerMetre);
            for (int i = 0; i < 200; i++) m.Observe(1.0, 3e-4);
            Assert.AreEqual(3e-4, m.SocPerMetre, 1e-6);
            m.Observe(0.0, 0.5);
            Assert.AreEqual(3e-4, m.SocPerMetre, 1e-6, "standing still teaches nothing");
        }

        [Test]
        public void ScoreFollowsTheRules()
        {
            var s = new ShuttleScore(new ShuttleSettings());
            s.OnDelivered(new RideRequest { RaisedAt = 0, PickedUpAt = 30, DroppedAt = 40 }, directDriveTime: 10.0);   // direct ride
            Assert.AreEqual(90, s.Points);
            s.OnDelivered(new RideRequest { RaisedAt = 0, PickedUpAt = 95, DroppedAt = 200 }, directDriveTime: 10.0);  // late, slow
            Assert.AreEqual(110, s.Points);
            s.OnGaveUp(new RideRequest());
            s.OnFlat();
            s.OnKerbStrike();
            Assert.AreEqual(110 - 50 - 500 - 5, s.Points);
            Assert.AreEqual((2, 1, 1, 1), (s.Delivered, s.GaveUp, s.Flats, s.KerbStrikes));
            Assert.AreEqual(62.5, s.AverageWait, 1e-9);
        }
    }
}
