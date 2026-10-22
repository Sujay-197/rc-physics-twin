// Edit Mode: the shuttle HUD's minimap maths (layout C, top-right).
using System.Linq;
using NUnit.Framework;
using UnityEngine;

namespace RcTwin.Tests
{
    public class ShuttleHudTests
    {
        private static readonly CampusRoutes Net = ShuttleRequestTests.Network();

        [Test]
        public void TheMapKeepsTheCampusShapeInsideItsArea()
        {
            var area = new Rect(900f, 10f, 300f, 200f);
            var m = new Minimap(Net, area);
            Assert.IsTrue(area.Contains(m.Map.min) && area.Contains(m.Map.max - new Vector2(1e-3f, 1e-3f)), m.Map.ToString());
            Assert.AreEqual(m.CampusAspect, m.Map.width / m.Map.height, 1e-3, "east-west over north-south, unstretched");
            float pad = 2f * Minimap.Pad;
            Assert.IsTrue(Mathf.Abs(m.Map.width - (area.width - pad)) < 1e-3f || Mathf.Abs(m.Map.height - (area.height - pad)) < 1e-3f,
                          "as large as the area allows, inside its margin");
        }

        [Test]
        public void EveryStopIsOnTheMap()
        {
            var m = new Minimap(Net, new Rect(0f, 0f, 320f, 240f));
            var inside = new Rect(m.Map.x - 1e-3f, m.Map.y - 1e-3f, m.Map.width + 2e-3f, m.Map.height + 2e-3f);
            foreach (var s in Net.Stops.Values)
                Assert.IsTrue(inside.Contains(m.ToMap(s.nodeX, s.nodeY)), s.name + " (the outermost stops lie on the edge)");
        }

        [Test]
        public void AClickNearAStopPicksIt()
        {
            var m = new Minimap(Net, new Rect(0f, 0f, 400f, 300f));
            var s3 = Net.Stops["S3"];
            var at = m.ToMap(s3.nodeX, s3.nodeY);
            Assert.AreEqual("S3", m.StopAt(at, 8f));
            Assert.AreEqual("S3", m.StopAt(at + new Vector2(5f, -3f), 8f));
            Assert.IsNull(m.StopAt(new Vector2(-50f, -50f), 8f));
        }

        [Test]
        public void WaitingStudentsAreLabelledByDestinationAndWait()
        {
            RideRequest R(string d, double at) => new RideRequest { Origin = "AB2", Destination = d, RaisedAt = at };
            var label = ShuttleHud.WaitLabel(new[] { R("S1", 40.0), R("S6", 10.0), R("S6", 30.0) }, 52.0);
            Assert.AreEqual("2 \u2192 S6 \u00b7 0:42\n\u2192 S1 \u00b7 0:12", label);
            Assert.AreEqual("3/6", ShuttleHud.SeatLabel(3, 6));
        }

        [Test]
        public void NorthIsUpAndEastIsRight()
        {
            // ISO x is Unity's +z (into the screen from the chase camera's default),
            // ISO y is Unity's -x: on a top-down map, +x is up and +y is left.
            var m = new Minimap(Net, new Rect(0f, 0f, 400f, 300f));
            var a = m.ToMap(0.0, 0.0);
            Assert.Less(m.ToMap(1.0, 0.0).y, a.y, "+x up the screen (GUI y grows down)");
            Assert.Less(m.ToMap(0.0, 1.0).x, a.x, "+y to the left");
        }
    }
}
