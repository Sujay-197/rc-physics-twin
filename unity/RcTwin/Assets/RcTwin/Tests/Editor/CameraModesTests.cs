// Edit Mode: choosing the campus player's camera mode (Docs/specs/2026-10-25-shuttle-fleet-design.md,
// "Camera modes"): the cycle, following a bus, the fleet changing size, -camera.
using System.Linq;
using NUnit.Framework;

namespace RcTwin.Tests
{
    public class CameraModesTests
    {
        [Test]
        public void CCyclesThroughEveryBusThenTheViews()
        {
            var m = new CameraModes(3);
            Assert.AreEqual("Bus 1", m.Current.ToString(), "it starts on Bus 1");
            var seen = new System.Collections.Generic.List<string> { m.Current.ToString() };
            for (int i = 0; i < 8; i++) { m.Next(); seen.Add(m.Current.ToString()); }
            CollectionAssert.AreEqual(
                new[] { "Bus 1", "Bus 2", "Bus 3", "Route", "Fleet", "Overview", "Orbit", "Stop", "Bus 1" }, seen);
            m.Previous();
            Assert.AreEqual(CameraKind.Stop, m.Current.Kind, "Shift+C goes back");
            m.Previous();
            Assert.AreEqual(CameraKind.Orbit, m.Current.Kind);
        }

        [Test]
        public void NumberKeysFollowABus()
        {
            var m = new CameraModes(3);
            Assert.IsTrue(m.Follow(2));
            Assert.AreEqual(CameraMode.Following(2), m.Current);
            Assert.IsFalse(m.Follow(4), "there is no Bus 4");
            Assert.AreEqual(CameraMode.Following(2), m.Current);
            m.Next();
            Assert.AreEqual(CameraKind.Bus, m.Current.Kind);
            Assert.AreEqual(3, m.Current.Bus);
            m.Next();
            Assert.AreEqual(CameraKind.Route, m.Current.Kind);
            Assert.AreEqual(3, m.FollowedBus, "the route view shows the bus last followed");
        }

        [Test]
        public void AFollowedBusThatLeavesHandsOverToTheLastOne()
        {
            var m = new CameraModes(3);
            m.Follow(3);
            m.SetBuses(2);
            Assert.AreEqual(CameraMode.Following(2), m.Current);
            Assert.AreEqual(2, m.FollowedBus);
            m.Set(new CameraMode(CameraKind.Overview));
            m.SetBuses(5);
            Assert.AreEqual(CameraKind.Overview, m.Current.Kind, "other views are kept");
            Assert.AreEqual(5 + CameraModes.Views.Length, m.Cycle().Count);
        }

        [Test]
        public void TheStopCamStepsThroughTheStops()
        {
            var m = new CameraModes(1);
            var stops = new[] { "AB1", "AB2", "S1" };
            m.StepStop(stops, "AB2", 1);
            Assert.AreEqual("Stop S1", m.Current.ToString());
            m.StepStop(stops, "S1", 1);
            Assert.AreEqual("Stop AB1", m.Current.ToString(), "round to the first");
            m.StepStop(stops, "AB1", -1);
            Assert.AreEqual("Stop S1", m.Current.ToString());
        }

        [TestCase("bus", CameraKind.Bus, 1, null)]
        [TestCase("Bus2", CameraKind.Bus, 2, null)]
        [TestCase("bus 3", CameraKind.Bus, 3, null)]
        [TestCase("route", CameraKind.Route, 0, null)]
        [TestCase("FLEET", CameraKind.Fleet, 0, null)]
        [TestCase("overview", CameraKind.Overview, 0, null)]
        [TestCase("orbit", CameraKind.Orbit, 0, null)]
        [TestCase("stop", CameraKind.Stop, 0, null)]
        [TestCase("stop:s3", CameraKind.Stop, 0, "S3")]
        public void TheCommandLineNamesAMode(string arg, CameraKind kind, int bus, string stop)
        {
            Assert.IsTrue(CameraModes.TryParse(arg, out var m), arg);
            Assert.AreEqual(new CameraMode(kind, bus, stop), m);
        }

        [TestCase("")]
        [TestCase("bus0")]
        [TestCase("busy")]
        [TestCase("chase")]
        public void ANameItDoesNotKnowIsRefused(string arg) => Assert.IsFalse(CameraModes.TryParse(arg, out _));

        [Test]
        public void AModeOnABusBeyondTheFleetFollowsTheLast()
        {
            var m = new CameraModes(2);
            Assert.IsTrue(CameraModes.TryParse("bus5", out var five));
            m.Set(five);
            Assert.AreEqual(CameraMode.Following(2), m.Current);
            Assert.IsTrue(m.Cycle().Take(2).All(x => x.Kind == CameraKind.Bus));
        }
    }
}
