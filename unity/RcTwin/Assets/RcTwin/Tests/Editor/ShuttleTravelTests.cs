// Edit Mode: the shuttle's travel model, from the campus routes the autopilot drives.
using System;
using System.Linq;
using NUnit.Framework;

namespace RcTwin.Tests
{
    public class ShuttleTravelTests
    {
        private static readonly CampusRoutes Net = ShuttleRequestTests.Network();

        private static double PolylineLength(double[] xyz)
        {
            double sum = 0.0;
            for (int i = 3; i < xyz.Length; i += 3)
                sum += Math.Sqrt(Math.Pow(xyz[i] - xyz[i - 3], 2) + Math.Pow(xyz[i + 1] - xyz[i - 2], 2));
            return sum;
        }

        [Test]
        public void DistancesAreTheRouteLengths()
        {
            var t = new RouteTravelModel(Net, 2.0);
            Assert.AreEqual(PolylineLength(Net.Route("S3", "AB1")), t.Distance("S3", "AB1"), 1e-9);
            Assert.AreEqual(0.0, t.Distance("S3", "S3"));
            Assert.That(t.Distance("S3", "AB1"), Is.InRange(20.0, 35.0));
        }

        [Test]
        public void DriveTimeIsDistanceOverCruise()
        {
            var t = new RouteTravelModel(Net, 2.0);
            Assert.AreEqual(t.Distance("AB2", "S6") / 2.0, t.DriveTime("AB2", "S6"), 1e-12);
        }

        [Test]
        public void AReversalAtAStopCostsATurnRound()
        {
            var t = new RouteTravelModel(Net, 2.0);
            Assert.AreEqual(8.0, t.TurnAllowance("AB1", "S3", "AB1"), "arrive at S3 from AB1, leave back toward AB1");
            Assert.AreEqual(0.0, t.TurnAllowance(null, "S3", "AB1"));
        }

        [Test]
        public void StopsOnTheWayAreFound()
        {
            var t = new RouteTravelModel(Net, 2.0);
            var on = t.StopsOnRoute("AB4", "AB1");
            var names = on.Select(x => x.stop).ToList();
            CollectionAssert.IsSubsetOf(new[] { "S3", "S1" }, names);
            Assert.IsTrue(on.All(x => x.s > 0.0 && x.s < t.Distance("AB4", "AB1")));
            for (int i = 1; i < on.Count; i++) Assert.Greater(on[i].s, on[i - 1].s, "in route order");
            var s3s1 = t.StopsOnRoute("S3", "S1").Select(x => x.stop).ToList();
            CollectionAssert.DoesNotContain(s3s1, "S3");
            CollectionAssert.DoesNotContain(s3s1, "S1");
        }
    }
}
