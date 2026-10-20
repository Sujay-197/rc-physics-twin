// Edit Mode: the Unity autopilot computes what Python does. The fixture
// (Fixtures/autopilot_parity.json, tools/make_autopilot_fixture.py) holds
// drives recorded in sim_py, one per RL task ("follow" and "stop"): the Core
// state per control tick, where the car was along the route, the RL
// environment's frame and observation, and the tuned pure pursuit's action;
// and campus routes from sim_py/campus.py.
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using NUnit.Framework;
using UnityEngine;

namespace RcTwin.Tests
{
    public class AutopilotParityTests
    {
        private const double Tol = 1e-9;
        private Dictionary<string, object> _fx;

        [OneTimeSetUp]
        public void Load()
        {
            string path = Path.Combine(Application.dataPath, "RcTwin", "Tests", "Editor", "Fixtures", "autopilot_parity.json");
            _fx = (Dictionary<string, object>)MiniJson.Parse(File.ReadAllText(path));
        }

        private static double D(object o) => Convert.ToDouble(o, CultureInfo.InvariantCulture);

        private static double[] Arr(object o)
        {
            var l = (List<object>)o;
            var a = new double[l.Count];
            for (int i = 0; i < a.Length; i++) a[i] = D(l[i]);
            return a;
        }

        private List<Dictionary<string, object>> Episodes()
        {
            var o = new List<Dictionary<string, object>>();
            foreach (Dictionary<string, object> e in (List<object>)_fx["episodes"]) o.Add(e);
            return o;
        }

        private static bool Stop(Dictionary<string, object> ep) => (string)ep["task"] == "stop";

        private static TwinRoad Road(Dictionary<string, object> ep)
        {
            var road = (Dictionary<string, object>)ep["road"];
            var xyz = new List<double>();
            foreach (List<object> q in (List<object>)road["xyz"]) { xyz.Add(D(q[0])); xyz.Add(D(q[1])); xyz.Add(D(q[2])); }
            return new TwinRoad(xyz.ToArray(), D(road["width"]));
        }

        private RcTwinNative.Params Car()
        {
            var car = (Dictionary<string, object>)_fx["car"];
            var p = RcTwinNative.DefaultParams();
            p.wheelRadius = D(car["wheel_radius"]);
            p.steerMax = D(car["steer_max"]);
            p.lf = D(car["lf"]);
            p.lr = D(car["lr"]);
            return p;
        }

        // The fixture's state, in rc_twin_state field order (state_fields).
        private static RcTwinNative.State State(double[] v) => new RcTwinNative.State
        {
            x = v[0], y = v[1], z = v[2], yaw = v[3], pitch = v[4], roll = v[5], vx = v[6], vy = v[7], vz = v[8],
            rollRate = v[9], pitchRate = v[10], yawRate = v[11],
            omegaFL = v[12], omegaFR = v[13], omegaRL = v[14], omegaRR = v[15],
            steer = v[16], steerRate = v[17], soc = v[18], packV = v[19],
            fzFL = v[20], fzFR = v[21], fzRL = v[22], fzRR = v[23],
            alphaLagFL = v[24], alphaLagFR = v[25], alphaLagRL = v[26], alphaLagRR = v[27],
        };

        private static List<Dictionary<string, object>> Ticks(Dictionary<string, object> ep)
        {
            var o = new List<Dictionary<string, object>>();
            foreach (Dictionary<string, object> t in (List<object>)ep["ticks"]) o.Add(t);
            return o;
        }

        [Test]
        public void BothTasksAreRecorded()
        {
            var eps = Episodes();
            Assert.AreEqual(new[] { "follow", "stop" }, new[] { (string)eps[0]["task"], (string)eps[1]["task"] });
            Assert.AreEqual("reached_end", (string)eps[1]["status"], "the stop episode stopped");
            Assert.AreEqual(TwinObservation.RemainingScaleM, D(_fx["remaining_scale_m"]));
            var fields = (List<object>)eps[1]["frame_fields"];
            Assert.AreEqual(TwinObservation.StopFrameSize, fields.Count);
            Assert.AreEqual("remaining_ratio", fields[TwinObservation.RemainingIndex]);
            for (int i = 0; i < TwinObservation.FrameSize; i++) Assert.AreEqual(TwinObservation.Fields[i], fields[i]);
        }

        private static void Near(double[] want, double[] got, string what)
        {
            Assert.AreEqual(want.Length, got.Length, $"{what}: length");
            for (int i = 0; i < want.Length; i++)
                Assert.AreEqual(want[i], got[i], Tol, $"{what}[{i}]");
        }

        [Test]
        public void StateFieldsAreInBindingOrder()
        {
            var f = (List<object>)_fx["state_fields"];
            Assert.AreEqual(28, f.Count);
            Assert.AreEqual("lag_rr", f[27]);
            Assert.AreEqual(28 * 8, System.Runtime.InteropServices.Marshal.SizeOf<RcTwinNative.State>());
        }

        [Test]
        public void FramesMatchPython()
        {
            var car = Car();
            double v = D(_fx["target_speed"]);
            foreach (var ep in Episodes())
            {
                using var road = Road(ep);
                int k = 0;
                foreach (var t in Ticks(ep))
                {
                    var st = State(Arr(t["state"]));
                    var (s, d) = road.Locate(st.x, st.y);
                    string at = $"{ep["task"]} tick {t["tick"]}";
                    Assert.AreEqual(D(t["s"]), s, Tol, $"{at}: s");
                    Assert.AreEqual(D(t["d"]), d, Tol, $"{at}: d");
                    Near(Arr(t["frame"]), TwinObservation.Frame(road, st, car, v, s, d, Stop(ep)), $"{at} frame");
                    k++;
                }
                Assert.Greater(k, 50);
            }
        }

        [Test]
        public void ObservationHistoryMatchesPython()
        {
            int history = (int)D(_fx["history"]);
            foreach (var ep in Episodes())
            {
            var obs = new TwinObservation(history, Stop(ep));
            var ticks = Ticks(ep);
            for (int k = 0; k < ticks.Count && (int)D(ticks[k]["tick"]) == k; k++)   // the every-tick stretch
            {
                var frame = Arr(ticks[k]["frame"]);
                if (k == 0) obs.Reset(frame);
                else
                {
                    var a = Arr(ticks[k - 1]["action"]);
                    obs.Push(a[0], a[1], frame);
                }
                Near(Arr(ticks[k]["obs"]), obs.Vector(), $"{ep["task"]} tick {k} observation");
            }
            }
        }

        [Test]
        public void TunedPurePursuitMatchesPython()
        {
            var gains = (Dictionary<string, object>)_fx["pure_pursuit"];
            foreach (var ep in Episodes())
            {
                var drv = PurePursuitDriver.Tuned(Car(), D(_fx["target_speed"]), D(gains["lookahead"]), D(gains["k_speed"]),
                                                  D(gains["a_lat"]), stopTask: Stop(ep));
                bool turned = false, braked = false;
                foreach (var t in Ticks(ep))
                {
                    var want = Arr(t["action"]);
                    var (steer, throttle) = drv.Act(new DriveContext { Observation = Arr(t["obs"]) });
                    Assert.AreEqual(want[0], steer, Tol, $"{ep["task"]} tick {t["tick"]} steer");
                    Assert.AreEqual(want[1], throttle, Tol, $"{ep["task"]} tick {t["tick"]} throttle");
                    turned |= Math.Abs(steer) > 0.05;
                    braked |= throttle < -0.05;
                }
                Assert.IsTrue(turned, "the fixture's drive turns");
                if (Stop(ep)) Assert.IsTrue(braked, "and brakes for the stop");
            }
        }

        [Test]
        public void RoutesMatchPython()
        {
            string net = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "..", "campus", "campus_network.json"));
            var routes = new CampusRoutes(File.ReadAllText(net));
            foreach (Dictionary<string, object> r in (List<object>)_fx["routes"])
            {
                var want = new List<double>();
                foreach (List<object> q in (List<object>)r["xyz"]) { want.Add(D(q[0])); want.Add(D(q[1])); want.Add(D(q[2])); }
                var got = routes.Route((string)r["from"], (string)r["to"]);
                Near(want.ToArray(), got, $"route {r["from"]} -> {r["to"]}");
            }
        }

        [Test]
        public void ItStopsAtTheEndWithoutReversing()
        {
            var drv = PurePursuitDriver.Tuned(Car(), 4.0, stopTask: true);
            var obs = Arr(Ticks(Episodes()[1])[10]["obs"]);
            obs[0] = 1.0;                                 // at the target speed
            double[] straight = { 0.5, 0.0, 1.0, 0.0, 2.0, 0.0 };
            Array.Copy(straight, 0, obs, 16, 6);          // on a straight road
            int rem = TwinObservation.RemainingIndex;
            obs[rem] = 1.0;                               // the stop is far
            var (_, far) = drv.Act(new DriveContext { Observation = obs });
            obs[rem] = 0.05;                              // 0.5 m to go
            var (_, near) = drv.Act(new DriveContext { Observation = obs });
            Assert.Greater(far, near);
            Assert.Less(near, 0.0, "brakes for a stop close ahead");
            var still = (double[])obs.Clone();
            still[0] = 0.0;                               // at rest
            still[rem] = 0.0;
            var (_, atRest) = drv.Act(new DriveContext { Observation = still });
            Assert.AreEqual(0.0, atRest, "no brake at rest: it would reverse");
        }

        [Test]
        public void StopsAreFoundByTheirRoadNode()
        {
            string net = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "..", "campus", "campus_network.json"));
            var routes = new CampusRoutes(File.ReadAllText(net));
            foreach (var s in routes.Stops.Values)
            {
                var (name, d) = routes.Nearest(s.nodeX, s.nodeY);
                Assert.AreEqual(0.0, d, 1e-9, s.name);
                Assert.AreEqual(routes.Stops[name].node, s.node, s.name);
            }
            Assert.AreEqual("AB1", routes.StopNames()[0]);
        }
    }
}
