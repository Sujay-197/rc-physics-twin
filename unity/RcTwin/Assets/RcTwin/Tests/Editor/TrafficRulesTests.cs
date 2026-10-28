// Edit Mode: buses keeping out of each other's way (ADR 0012): lanes, holding
// behind a bus, passing in the other lane, giving way at a crossing, and
// waiting behind a bus parked at a stop.
using System;
using System.Collections.Generic;
using NUnit.Framework;

namespace RcTwin.Tests
{
    public class TrafficRulesTests
    {
        private static readonly CampusRoutes Net = ShuttleRequestTests.Network();
        private static readonly double Lane = ShuttleFleet.LaneOffset;
        private static RcTwinNative.Contact _fp;

        [OneTimeSetUp]
        public void Footprint()
        {
            using (var v = new TwinVehicle(RcTwinNative.Tier.L2DoubleTrack, RcTwinNative.DefaultParams()))
            {
                _fp = v.Contact;
                _fp.halfWidth += 0.0175;   // TwinCar.footprintMargin
            }
        }

        // A bus at (x, y) heading yaw, driving `length` straight ahead at `speed` (0 length: parked).
        private static TrafficAgent Bus(int id, double x, double y, double yaw, double speed, double length)
        {
            var a = new TrafficAgent
            {
                Id = id, X = x, Y = y, Yaw = yaw, Speed = speed,
                HalfLength = _fp.halfLength, HalfWidth = _fp.halfWidth, CentreOffset = _fp.centreOffset,
            };
            a.Path.Add(new PathPose(x, y, yaw, 0.0));
            for (double s = 0.05; s <= length + 1e-9; s += 0.05)
                a.Path.Add(new PathPose(x + s * Math.Cos(yaw), y + s * Math.Sin(yaw), yaw, s));
            return a;
        }

        [Test]
        public void LanesAreTheRouteOffsetToTheLeft()
        {
            // East, then north: the left lane is north of the first leg and west of the second.
            var xyz = new double[] { 0, 0, 0, 2, 0, 0, 2, 2, 0 };
            var o = Lanes.Offset(xyz, 0.2);
            Assert.AreEqual(0.0, o[0], 1e-12);
            Assert.AreEqual(0.2, o[1], 1e-12);
            Assert.AreEqual(2.0 - 0.2, o[3], 1e-12, "the mitred corner: 0.2 off both legs");
            Assert.AreEqual(0.2, o[4], 1e-12);
            Assert.AreEqual(2.0 - 0.2, o[6], 1e-12);
            Assert.AreEqual(2.0, o[7], 1e-12);
            CollectionAssert.AreEqual(xyz, Lanes.Offset(xyz, 0.0), "offset 0: the centreline");
        }

        [Test]
        public void AFollowerHoldsBehindTheBusAhead()
        {
            var t = new TrafficRules();
            var ahead = Bus(1, 0.0, Lane, 0.0, 0.5, 2.0);
            var behind = Bus(2, -1.5, Lane, 0.0, 2.0, 3.0);
            t.Update(new[] { ahead, behind });
            Assert.IsTrue(double.IsPositiveInfinity(t.Hold[0]), "nothing in front of the leader");
            Assert.AreEqual(0, t.HeldBy[1]);
            double gap = 1.5 - 2.0 * _fp.halfLength - 2.0 * t.Clearance;
            Assert.That(t.Hold[1], Is.InRange(gap - 0.06, gap), "stops a clearance short of its tail");
        }

        [Test]
        public void OncomingBusesPassInTheirLanes()
        {
            Assert.Less(2.0 * (_fp.halfWidth + new TrafficRules().Clearance), 2.0 * Lane, "the lanes leave room to pass");
            var t = new TrafficRules();
            var east = Bus(1, -1.5, Lane, 0.0, 2.0, 3.0);
            var west = Bus(2, 1.5, -Lane, Math.PI, 2.0, 3.0);
            t.Update(new[] { east, west });
            Assert.IsTrue(double.IsPositiveInfinity(t.Hold[0]) && double.IsPositiveInfinity(t.Hold[1]),
                          $"holds {t.Hold[0]}, {t.Hold[1]}");
            var t2 = new TrafficRules();
            t2.Update(new[] { Bus(1, -1.5, 0.0, 0.0, 2.0, 3.0), Bus(2, 1.5, 0.0, Math.PI, 2.0, 3.0) });
            Assert.IsTrue(t2.Hold[0] < 3.0 || t2.Hold[1] < 3.0, "on one centreline they would meet");
        }

        [Test]
        public void AtACrossingTheLaterArrivalGivesWayAndKeepsGivingWay()
        {
            var t = new TrafficRules();
            var a = Bus(1, -1.0, 0.0, 0.0, 2.0, 3.0);               // reaches the crossing first
            var b = Bus(2, 0.0, -2.0, 0.5 * Math.PI, 2.0, 4.0);
            t.Update(new[] { a, b });
            Assert.IsTrue(double.IsPositiveInfinity(t.Hold[0]));
            Assert.AreEqual(0, t.HeldBy[1]);
            Assert.Less(t.Hold[1], 2.0 - _fp.halfLength, "stops short of the crossing");
            // Bus 1 slows right down: the decision holds while the paths still cross.
            a = Bus(1, -1.0, 0.0, 0.0, 0.05, 3.0);
            t.Update(new[] { a, b });
            Assert.IsTrue(double.IsPositiveInfinity(t.Hold[0]), "no taking turns at deciding");
            Assert.AreEqual(0, t.HeldBy[1]);
        }

        [Test]
        public void ABusAlreadyInTheWayGoesFirst()
        {
            var t = new TrafficRules();
            var inside = Bus(2, 0.0, 0.0, 0.0, 0.3, 2.0);          // slow, standing in the crossing
            var coming = Bus(1, -0.1, -2.5, 0.5 * Math.PI, 2.0, 4.0);
            t.Update(new[] { inside, coming });
            Assert.IsTrue(double.IsPositiveInfinity(t.Hold[0]), "the bus in the crossing clears it");
            Assert.AreEqual(0, t.HeldBy[1], "the lower number gives way: it is not in the crossing yet");
        }

        [Test]
        public void ABusTurningRoundHasTheWay()
        {
            // Slow and later to the crossing, it would give way; turning round, it does not
            // (the other bus can still stop).
            var t = new TrafficRules();
            var turning = Bus(2, 0.0, -1.0, 0.5 * Math.PI, 0.3, 2.0);
            turning.Manoeuvring = true;
            var driving = Bus(1, -2.5, 0.0, 0.0, 2.0, 3.0);
            t.Update(new[] { turning, driving });
            Assert.IsTrue(double.IsPositiveInfinity(t.Hold[0]), "the turning bus goes on");
            Assert.AreEqual(0, t.HeldBy[1], "the bus on its route waits");
        }

        [Test]
        public void ABusParkedAtAStopHoldsTheNextOneBehindIt()
        {
            var stop = Net.Stops["S1"];
            var xyz = Lanes.Offset(Net.Route("AB4", "S1"), Lane);
            int n = xyz.Length / 3;
            double yaw = Math.Atan2(xyz[3 * n - 2] - xyz[3 * n - 5], xyz[3 * n - 3] - xyz[3 * n - 6]);
            double ex = xyz[3 * n - 3], ey = xyz[3 * n - 2];
            var parked = Bus(1, ex, ey, yaw, 0.0, 0.0);              // at its berth: no path
            var arriving = Bus(2, ex - 1.6 * Math.Cos(yaw), ey - 1.6 * Math.Sin(yaw), yaw, 1.0, 1.6);
            var t = new TrafficRules();
            t.Update(new[] { parked, arriving });
            Assert.AreEqual(0, t.HeldBy[1]);
            double end = 1.6 - 2.0 * _fp.halfLength - 2.0 * t.Clearance;
            Assert.That(t.Hold[1], Is.InRange(end - 0.06, end), "it waits a bus length behind");
            Assert.LessOrEqual(1.6 - t.Hold[1], ShuttleFleet.Berth, "close enough to serve the stop from there");
            Assert.AreEqual(0, t.Overlaps);
            Assert.Less(Math.Sqrt((ex - stop.nodeX) * (ex - stop.nodeX) + (ey - stop.nodeY) * (ey - stop.nodeY)), Lane + 1e-6,
                        "the berth is the lane's end, at the kerb side of the stop");
        }

        [Test]
        public void ACornerIsGivenRoomOnItsInsideOnly()
        {
            var t = new TrafficRules();
            var parked = Bus(1, 0.0, 0.0, 0.0, 0.0, 0.0);
            double off = 2.0 * _fp.halfWidth + 2.0 * t.Clearance + 0.05;   // clear of each other
            var left = new PathPose(0.0, off, 0.0, 0.0, -0.14);      // bending right: room on its right, toward the parked bus
            var right = new PathPose(0.0, off, 0.0, 0.0, 0.14);      // bending left: room away from it
            Assert.IsFalse(TrafficRules.Overlap(new PathPose(0.0, off, 0.0, 0.0), parked, parked.Path[0], parked, t.Clearance));
            Assert.IsTrue(TrafficRules.Overlap(left, parked, parked.Path[0], parked, t.Clearance), "cutting toward it");
            Assert.IsFalse(TrafficRules.Overlap(right, parked, parked.Path[0], parked, t.Clearance), "cutting away from it");
        }

        [Test]
        public void OverlapsAreCountedOncePerMeeting()
        {
            var t = new TrafficRules();
            var a = Bus(1, 0.0, 0.0, 0.0, 0.0, 0.0);
            var b = Bus(2, 0.1, 0.0, 0.0, 0.0, 0.0);
            t.Update(new[] { a, b });
            t.Update(new[] { a, b });
            Assert.AreEqual(1, t.Overlaps);
            t.Update(new[] { a, Bus(2, 3.0, 0.0, 0.0, 0.0, 0.0) });
            t.Update(new[] { a, b });
            Assert.AreEqual(2, t.Overlaps);
        }

        [Test]
        public void BusesSideBySideInTheirLanesMayPass()
        {
            // Each a few centimetres off its lane toward the other: within the clearance, not
            // touching. Passing keeps the gap between them, so neither holds the other.
            var t = new TrafficRules();
            var east = Bus(1, 0.0, Lane - 0.07, 0.0, 0.5, 1.0);
            var west = Bus(2, 0.3, -Lane, Math.PI, 0.5, 1.0);
            Assert.IsTrue(TrafficRules.Overlap(east.Path[0], east, west.Path[0], west, t.Clearance), "within the clearance");
            Assert.Greater(TrafficRules.Gap(east.Path[0], east, west.Path[0], west), 0.0, "not touching");
            t.Update(new[] { east, west });
            Assert.IsTrue(double.IsPositiveInfinity(t.Hold[0]) && double.IsPositiveInfinity(t.Hold[1]),
                          $"holds {t.Hold[0]}, {t.Hold[1]}");

            // Drawing a little closer as it passes one parked (its lane bends that way) is a squeeze
            // it may make while half the clearance stays between them; touching it is not.
            var parked = Bus(1, 0.0, Lane - 0.03, 0.0, 0.0, 0.0);
            var squeeze = Bus(2, 0.8, -Lane, Math.PI, 0.5, 0.0);
            for (double s = 0.05; s <= 1.6; s += 0.05)
                squeeze.Path.Add(new PathPose(0.8 - s, -Lane + 0.02 * s / 1.6, Math.PI, s));
            t = new TrafficRules();
            t.Update(new[] { parked, squeeze });
            Assert.IsTrue(double.IsPositiveInfinity(t.Hold[1]), $"squeezes past: hold {t.Hold[1]}");
            var touch = Bus(2, 0.8, -Lane, Math.PI, 0.5, 0.0);
            for (double s = 0.05; s <= 1.6; s += 0.05)
                touch.Path.Add(new PathPose(0.8 - s, -Lane + 0.2 * s / 1.6, Math.PI, s));
            t = new TrafficRules();
            t.Update(new[] { parked, touch });
            Assert.Less(t.Hold[1], 1.6, "would touch: holds");
        }

        [Test]
        public void BusesAlreadyTooCloseMayOnlyDrawApart()
        {
            var t = new TrafficRules();
            var a = Bus(1, 0.0, 0.0, 0.0, 0.0, 0.0);
            var away = Bus(2, 2.0 * _fp.halfLength + 0.05, 0.0, 0.0, 0.5, 1.0);   // within the clearance, driving off
            t.Update(new[] { a, away });
            Assert.IsTrue(double.IsPositiveInfinity(t.Hold[1]), "driving away is allowed");
            var toward = Bus(2, -(2.0 * _fp.halfLength + 0.05), 0.0, 0.0, 0.5, 1.0);
            t.Update(new[] { a, toward });
            Assert.Less(t.Hold[1], 0.06, "closer is not");
        }
    }
}
