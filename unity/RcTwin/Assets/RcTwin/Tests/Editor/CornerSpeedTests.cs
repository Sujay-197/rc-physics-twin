// Edit Mode: the corner speed plan as sim_py/controllers.py's. Fixtures/
// corner_speed.json (tools/make_autopilot_fixture.py) has a campus route,
// Python's speed profile along it, and a scripted trace: per tick the car's
// arc length and speed, the wrapped driver's action and what Python's
// CornerSpeed commanded. And the autopilot's plan has the gains set in
// policies/controllers_unity_stop.json.
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using NUnit.Framework;
using UnityEngine;

namespace RcTwin.Tests
{
    public class CornerSpeedTests
    {
        private const double Tol = 1e-9;

        private static double D(object o) => Convert.ToDouble(o, CultureInfo.InvariantCulture);

        private static Dictionary<string, object> Fixture() =>
            (Dictionary<string, object>)MiniJson.Parse(File.ReadAllText(
                Path.Combine(Application.dataPath, "RcTwin", "Tests", "Editor", "Fixtures", "corner_speed.json")));

        private sealed class Scripted : IDriver
        {
            public (double steer, double throttle) Next;
            public string Name => "scripted";
            public int History => 0;
            public void Reset() { }
            public (double steer, double throttle) Act(in DriveContext c) => Next;
        }

        [Test]
        public void ItPlansAndCapsAsPythonDoes()
        {
            var doc = Fixture();
            var xyz = new List<double>();
            foreach (List<object> q in (List<object>)doc["xyz"]) { xyz.Add(D(q[0])); xyz.Add(D(q[1])); xyz.Add(D(q[2])); }
            using var road = new TwinRoad(xyz.ToArray(), D(doc["width"]));
            Assert.AreEqual(D(doc["length"]), road.Length, Tol);

            var inner = new Scripted();
            var cs = new CornerSpeed(inner, D(doc["target_speed"]), D(doc["k_speed"]))
            {
                ALat = D(doc["a_lat"]), ABrake = D(doc["a_brake"]), Window = D(doc["window"]), Step = D(doc["step"]),
            };
            cs.Plan(s => road.Point(s).heading, road.Length);
            var want = (List<object>)doc["profile"];
            Assert.AreEqual(want.Count, cs.Profile.Length);
            for (int i = 0; i < want.Count; i++) Assert.AreEqual(D(want[i]), cs.Profile[i], Tol, $"profile {i}");

            var obs = new double[TwinObservation.RemainingIndex + 1];
            int k = 0, braked = 0;
            foreach (Dictionary<string, object> t in (List<object>)doc["ticks"])
            {
                var a = (List<object>)t["inner"];
                var act = (List<object>)t["action"];
                inner.Next = (D(a[0]), D(a[1]));
                obs[0] = D(t["speed_ratio"]);
                var (steer, throttle) = cs.Act(new DriveContext { Observation = obs, S = D(t["s"]) });
                Assert.AreEqual(D(act[0]), steer, Tol, $"tick {k} steer");
                Assert.AreEqual(D(act[1]), throttle, Tol, $"tick {k} throttle");
                if (throttle < D(a[1])) braked++;
                k++;
            }
            Assert.Greater(braked, 20, "the trace runs over the plan at the junctions");
        }

        [Test]
        public void WithoutAPlanItIsTheDriverItWraps()
        {
            var inner = new Scripted { Next = (0.2, 0.9) };
            var cs = new CornerSpeed(inner, 4.0);
            var obs = new double[TwinObservation.RemainingIndex + 1];
            obs[0] = 1.0;
            Assert.AreEqual((0.2, 0.9), cs.Act(new DriveContext { Observation = obs, S = 3.0 }));
        }

        [Test]
        public void TheAutopilotPlansWithTheDesignGains()
        {
            string path = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "..", "policies", "controllers_unity_stop.json"));
            var doc = (Dictionary<string, object>)MiniJson.Parse(File.ReadAllText(path));
            var g = (Dictionary<string, object>)((Dictionary<string, object>)doc["corner_speed"])["gains"];
            Assert.AreEqual(D(g["a_lat"]), CornerSpeed.DesignALat);
            Assert.AreEqual(D(g["a_brake"]), CornerSpeed.DesignABrake);
            var pp = PurePursuitDriver.TunedForTwin(RcTwinNative.DefaultParams(), 4.0);
            var cs = new CornerSpeed(pp, 4.0);
            Assert.AreEqual(pp.KSpeed, cs.KSpeed, "the speed loop of the driver it wraps, as in Python");
            Assert.AreEqual(CornerSpeed.DesignALat, cs.ALat);
            Assert.AreEqual(CornerSpeed.DesignABrake, cs.ABrake);
        }
    }
}
