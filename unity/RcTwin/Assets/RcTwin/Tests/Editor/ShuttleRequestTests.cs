// Edit Mode: ride requests for the campus shuttle (Docs/specs/2026-10-21-campus-shuttle-design.md).
// Origins and destinations follow the stops' demand weights in campus/campus_network.json.
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using NUnit.Framework;
using UnityEngine;

namespace RcTwin.Tests
{
    public class ShuttleRequestTests
    {
        internal static CampusRoutes Network() =>
            new CampusRoutes(File.ReadAllText(Path.GetFullPath(
                Path.Combine(Application.dataPath, "..", "..", "..", "campus", "campus_network.json"))));

        internal static List<(string name, int node, int weight)> Stops(CampusRoutes net) =>
            net.Stops.Values.OrderBy(s => s.name, StringComparer.Ordinal).Select(s => (s.name, s.node, s.weight)).ToList();

        private static List<RideRequest> Run(int seed, double seconds, double dt = 0.1)
        {
            var spawner = new RequestSpawner(Stops(Network()), seed) { RatePerSecond = 1.0 / 20.0 };
            var all = new List<RideRequest>();
            for (double t = 0.0; t < seconds; t += dt) all.AddRange(spawner.Tick(t, dt));
            return all;
        }

        [Test]
        public void OriginsFollowTheDemandWeights()
        {
            var all = Run(1, 100000.0);
            Assert.That(all.Count, Is.InRange(4500, 5500), "one request every 20 s");
            double share(string s) => all.Count(r => r.Origin == s) / (double)all.Count;
            Assert.AreEqual(6.0 / 19.0, share("S3"), 0.02);
            Assert.AreEqual(4.0 / 19.0, share("AB4"), 0.02);
        }

        [Test]
        public void ADestinationIsNeverTheOriginsNode()
        {
            var stops = Stops(Network()).ToDictionary(s => s.name, s => s.node);
            var all = Run(2, 100000.0);
            Assert.IsTrue(all.All(r => stops[r.Origin] != stops[r.Destination]));
            var fromAb2 = all.Where(r => r.Origin == "AB2").ToList();
            Assert.Greater(fromAb2.Count, 100);
            Assert.AreEqual(6.0 / 18.0, fromAb2.Count(r => r.Destination == "S3") / (double)fromAb2.Count, 0.05);
        }

        [Test]
        public void TheSameSeedSpawnsTheSameRequests()
        {
            string Key(List<RideRequest> l) => string.Join(";", l.Select(r => $"{r.Origin}>{r.Destination}@{r.RaisedAt:0.0}"));
            Assert.AreEqual(Key(Run(7, 2000.0)), Key(Run(7, 2000.0)));
            Assert.AreNotEqual(Key(Run(7, 2000.0)), Key(Run(8, 2000.0)));
        }

        [Test]
        public void AUserRequestIsRaisedAtOnceAndChecked()
        {
            var spawner = new RequestSpawner(Stops(Network()), 0);
            var a = spawner.Raise("AB2", "S6", 12.5);
            var b = spawner.Raise("S3", "AB1", 13.0);
            Assert.AreNotEqual(a.Id, b.Id);
            Assert.AreEqual(RequestState.Waiting, a.State);
            Assert.IsTrue(a.UserRaised);
            Assert.AreEqual(12.5, a.RaisedAt);
            Assert.IsTrue(double.IsNaN(a.PickedUpAt) && double.IsNaN(a.DroppedAt));
            Assert.AreEqual(2, spawner.All.Count);
            Assert.Throws<ArgumentException>(() => spawner.Raise("AB2", "AB2", 0.0));
            Assert.Throws<ArgumentException>(() => spawner.Raise("XX", "S6", 0.0));
        }
    }
}
