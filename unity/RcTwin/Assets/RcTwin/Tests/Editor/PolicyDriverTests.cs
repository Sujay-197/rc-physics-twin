// Edit Mode: PolicyDriver runs an exported network as Python does. The
// policies here are small and synthetic (tools/make_autopilot_fixture.py:
// seeded random weights, in tools/export_policy.py's format) with the
// actions Python computed for them; policies/release/*.json, trained ones,
// are replayed against their own recorded checks the same way.
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using NUnit.Framework;
using UnityEngine;

namespace RcTwin.Tests
{
    public class PolicyDriverTests
    {
        private const double Tol = 1e-4;   // float32 network against Python's float64 sums

        private static string Fixture(string name) =>
            Path.Combine(Application.dataPath, "RcTwin", "Tests", "Editor", "Fixtures", name);

        private static double D(object o) => Convert.ToDouble(o, CultureInfo.InvariantCulture);

        private static double[] Arr(object o)
        {
            var l = (List<object>)o;
            var a = new double[l.Count];
            for (int i = 0; i < a.Length; i++) a[i] = D(l[i]);
            return a;
        }

        private static void Replay(string path, double tol)
        {
            string json = File.ReadAllText(path);
            var drv = PolicyDriver.Load(json, RcTwinNative.DefaultParams());
            var doc = (Dictionary<string, object>)MiniJson.Parse(json);
            int n = 0;
            foreach (Dictionary<string, object> c in (List<object>)doc["checks"])
            {
                var want = Arr(c["action"]);
                var (steer, throttle) = drv.Act(new DriveContext { Observation = Arr(c["obs"]) });
                Assert.AreEqual(want[0], steer, tol, $"{Path.GetFileName(path)} check {n}: steer");
                Assert.AreEqual(want[1], throttle, tol, $"{Path.GetFileName(path)} check {n}: throttle");
                n++;
            }
            Assert.Greater(n, 10);
        }

        [Test]
        public void AResidualPolicyOnTheStopTask() => Replay(Fixture("policy_residual_stop.json"), Tol);

        [Test]
        public void APlainPolicyOnTheFollowTask() => Replay(Fixture("policy_follow.json"), Tol);

        [Test]
        public void ItSaysWhatItIs()
        {
            var r = PolicyDriver.Load(File.ReadAllText(Fixture("policy_residual_stop.json")), RcTwinNative.DefaultParams());
            Assert.IsTrue(r.Residual);
            Assert.AreEqual("stop", r.Task);
            Assert.AreEqual(4, r.History);
            Assert.AreEqual(4.0, r.TargetSpeed);
            var f = PolicyDriver.Load(File.ReadAllText(Fixture("policy_follow.json")), RcTwinNative.DefaultParams());
            Assert.IsFalse(f.Residual);
            Assert.AreEqual("follow", f.Task);
        }

        [Test]
        public void ItRefusesAnObservationOfTheWrongSize()
        {
            var f = PolicyDriver.Load(File.ReadAllText(Fixture("policy_follow.json")), RcTwinNative.DefaultParams());
            Assert.Throws<ArgumentException>(() => f.Act(new DriveContext { Observation = new double[5] }));
            Assert.Throws<FormatException>(() => PolicyDriver.Load("{\"format\":\"x\"}", RcTwinNative.DefaultParams()));
        }

        [Test]
        public void ReleasedPoliciesReplayTheirChecks()
        {
            string dir = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "..", "policies", "release"));
            if (!Directory.Exists(dir)) Assert.Ignore("no policies/release yet");
            int n = 0;
            foreach (var path in Directory.GetFiles(dir, "*.json"))
            {
                Replay(path, Tol);
                n++;
            }
            if (n == 0) Assert.Ignore("no released policies");
        }
    }
}
