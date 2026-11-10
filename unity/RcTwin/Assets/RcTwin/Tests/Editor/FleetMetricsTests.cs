// Edit Mode: what a fleet run measures (FleetMetrics, the fleet-level experiment):
// sums only inside the window, counters relative to the window's start, and JSON.
using System.Collections.Generic;
using NUnit.Framework;

namespace RcTwin.Tests
{
    public class FleetMetricsTests
    {
        [Test]
        public void SumsOnlyInsideTheWindow()
        {
            var m = new FleetMetrics(10.0, 20.0);
            for (int k = 1; k <= 300; k++)
                m.Observe(0.1 * k, 0.1, 2, k % 2 == 0 ? ShuttleMode.Driving : ShuttleMode.Charging, 3, 0.2, 0.01);
            Assert.AreEqual(1, m.Buses.Count);
            var b = m.Buses[0];
            Assert.AreEqual(2, b.Number);
            Assert.AreEqual(10.0, b.Time, 0.15, "ticks in (10, 20]");
            Assert.AreEqual(5.0, b.Driving, 0.15);
            Assert.AreEqual(5.0, b.Charging, 0.15);
            Assert.AreEqual(3.0 * b.Time, b.SeatSeconds, 1e-9);
            Assert.AreEqual(3.0 * b.Driving, b.SeatSecondsDriving, 1e-9);
            Assert.AreEqual(20.0, b.Metres, 0.4);
            Assert.AreEqual(1.0, b.EnergyWh, 0.02);
        }

        [Test]
        public void CountersAreTakenFromTheWindowStart()
        {
            var m = new FleetMetrics(10.0, 100.0);
            m.Mark(5.0, new Dictionary<string, double> { ["delivered"] = 1 });
            Assert.IsNull(m.AtStart, "not yet in the window");
            m.Mark(10.5, new Dictionary<string, double> { ["delivered"] = 4 });
            m.Mark(11.0, new Dictionary<string, double> { ["delivered"] = 6 });
            var r = m.Report(50.0, new Dictionary<string, double> { ["delivered"] = 9 }, new RideRequest[0],
                             _ => (0.0, 0.0), new LegRecord[0], 1.0);
            Assert.AreEqual(5.0, (double)((Dictionary<string, object>)r["counters_window"])["delivered"]);
            Assert.AreEqual(9.0, (double)((Dictionary<string, object>)r["counters_total"])["delivered"]);
        }

        [Test]
        public void WritesJson()
        {
            var r = new RideRequest { Id = 7, Origin = "S3", Destination = "AB1", RaisedAt = 1.5 };
            var m = new FleetMetrics(0.0, 10.0);
            m.Config["seed"] = 3;
            var rep = m.Report(10.0, new Dictionary<string, double>(), new[] { r }, _ => (12.5, 6.25), new LegRecord[0], 2.0);
            string json = FleetMetrics.ToJson(rep);
            StringAssert.Contains("\"seed\": 3", json);
            StringAssert.Contains("\"picked\": null", json, "NaN is null");
            StringAssert.Contains("\"direct_m\": 12.5", json);
            var back = (Dictionary<string, object>)MiniJson.Parse(json);
            Assert.AreEqual("rc-twin-fleet-run/1", back["schema"]);
            Assert.AreEqual("a\"b\\c\n", ((Dictionary<string, object>)MiniJson.Parse(FleetMetrics.ToJson(new Dictionary<string, object> { ["k"] = "a\"b\\c\n" })))["k"]);
        }
    }
}
