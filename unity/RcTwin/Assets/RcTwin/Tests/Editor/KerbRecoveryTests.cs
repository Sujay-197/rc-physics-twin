// Edit Mode: KerbRecovery backs off a kerb as sim_py/controllers.py's does.
// Fixtures/kerb_recovery.json (tools/make_autopilot_fixture.py) is a
// scripted trace: per tick the wrapped driver's action, the car's speed and
// what Python's recovery commanded. And the autopilot's pure pursuit gains
// are the ones tuned for the Unity car, policies/controllers_unity_stop.json.
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using NUnit.Framework;
using UnityEngine;

namespace RcTwin.Tests
{
    public class KerbRecoveryTests
    {
        private const double Tol = 1e-12;

        private static double D(object o) => Convert.ToDouble(o, CultureInfo.InvariantCulture);

        /// <summary>Says what the trace says the wrapped driver said.</summary>
        private sealed class Scripted : IDriver
        {
            public (double steer, double throttle) Next;
            public int Resets;
            public string Name => "scripted";
            public int History => 0;
            public void Reset() => Resets++;
            public (double steer, double throttle) Act(in DriveContext c) => Next;
        }

        [Test]
        public void ItBacksOffAsPythonDoes()
        {
            string path = Path.Combine(Application.dataPath, "RcTwin", "Tests", "Editor", "Fixtures", "kerb_recovery.json");
            var doc = (Dictionary<string, object>)MiniJson.Parse(File.ReadAllText(path));
            var inner = new Scripted();
            var rec = new KerbRecovery(inner, D(doc["target_speed"])) { Tick = D(doc["tick"]) };
            rec.Reset();
            Assert.AreEqual(1, inner.Resets, "a new trip resets the driver it wraps");
            var obs = new double[TwinObservation.RemainingIndex + 1];
            int k = 0, backing = 0;
            foreach (Dictionary<string, object> t in (List<object>)doc["ticks"])
            {
                var a = (List<object>)t["inner"];
                var want = (List<object>)t["action"];
                inner.Next = (D(a[0]), D(a[1]));
                obs[0] = D(t["speed_ratio"]);
                var (steer, throttle) = rec.Act(new DriveContext { Observation = obs });
                Assert.AreEqual(D(want[0]), steer, Tol, $"tick {k} steer");
                Assert.AreEqual(D(want[1]), throttle, Tol, $"tick {k} throttle");
                Assert.AreEqual(Convert.ToInt32(t["back_offs"]), rec.BackOffs, $"tick {k} back-offs");
                if (rec.Backing) backing++;
                k++;
            }
            Assert.AreEqual(2, rec.BackOffs, "the trace pins twice");
            Assert.Greater(backing, 100, "and backs off for a while each time");
        }

        [Test]
        public void TheAutopilotDrivesWithTheGainsTunedForTheUnityCar()
        {
            string path = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "..", "policies", "controllers_unity_stop.json"));
            var doc = (Dictionary<string, object>)MiniJson.Parse(File.ReadAllText(path));
            var pp = (Dictionary<string, object>)doc["pure_pursuit"];
            var g = (Dictionary<string, object>)pp["gains"];
            var tuning = (Dictionary<string, object>)pp["tuning"];
            Assert.AreEqual("stop", tuning["task"]);
            Assert.AreEqual("unity", tuning["car"]);
            var drv = PurePursuitDriver.TunedForTwin(RcTwinNative.DefaultParams(), 4.0);
            Assert.AreEqual(D(g["lookahead"]), drv.Lookahead);
            Assert.AreEqual(D(g["k_speed"]), drv.KSpeed);
            Assert.AreEqual(D(g["a_lat"]), drv.ALat);
            Assert.AreEqual(TwinObservation.RemainingIndex, drv.RemainingIndex, "the stop task");
        }
    }
}
