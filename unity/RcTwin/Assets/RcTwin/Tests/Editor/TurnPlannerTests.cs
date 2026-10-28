// Edit Mode: turning round at a stop (TurnAround.cs). The planner on the
// campus network's own walls and road width, and a planned turn driven by
// TurnDriver on the Core with the campus car's wheels (inertia on, so the
// reverse legs need the ESC's reverse, ADR 0010).
using System;
using System.Collections.Generic;
using System.IO;
using NUnit.Framework;
using UnityEngine;

namespace RcTwin.Tests
{
    public class TurnPlannerTests
    {
        // S3, as the campus scene starts the car there (CampusDriveTests).
        private const double HomeX = -0.137349153, HomeY = -2.5646368, HomeYaw = 3.1382891505603703;
        private const double FootprintMargin = 0.0175;   // TwinCar.footprintMargin
        private CampusRoutes _net;

        [OneTimeSetUp]
        public void Load()
        {
            string net = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "..", "campus", "campus_network.json"));
            _net = new CampusRoutes(File.ReadAllText(net));
        }

        // The campus car: TwinCar's wheel inertia, Ackermann and low-speed
        // kinematic blend (its suspension does not matter at walking pace on the
        // flat).
        private static RcTwinNative.Params Car()
        {
            var p = RcTwinNative.DefaultParams();
            p.wheelInertia = 1e-5;
            p.ackermann = 1.0;
            p.vKinematic = 0.3;
            return p;
        }

        private static RcTwinNative.Contact Footprint(TwinVehicle v)
        {
            var c = v.Contact;
            c.halfWidth += FootprintMargin;
            return c;
        }

        private IEnumerable<(double[], bool)> Walls()
        {
            foreach (var w in _net.Walls) yield return (w.xy, w.closed);
        }

        private TurnPlanner Planner(TwinVehicle v, IEnumerable<(double[], bool)> walls = null) =>
            new TurnPlanner(Car(), Footprint(v), walls ?? Walls(), _net.Centrelines(), 3, _net.RoadWidth);

        private static double Heading(double[] xyz) => Math.Atan2(xyz[4] - xyz[1], xyz[3] - xyz[0]);

        [Test]
        public void TheNetworkCarriesTheSceneWalls()
        {
            Assert.Greater(_net.Walls.Count, 10, "kerb lines and building footprints");
            Assert.IsTrue(_net.Walls.Exists(w => w.name.StartsWith("kerb")));
        }

        [Test]
        public void AtS3TheRouteToAB2LeavesBehindTheCar()
        {
            double h = Heading(_net.Route("S3", "AB2"));
            Assert.IsTrue(TurnPlanner.FacesAway(HomeYaw, h, TwinAutopilot.TurnAroundRad));
            Assert.IsFalse(TurnPlanner.FacesAway(HomeYaw, Heading(_net.Route("S3", "AB1")), TwinAutopilot.TurnAroundRad),
                           "S3 to AB1 leaves the way the car faces");
        }

        [Test]
        public void ATurnFitsInTheJunctionAtS3()
        {
            using var car = new TwinVehicle(RcTwinNative.Tier.L2DoubleTrack, Car());
            var planner = Planner(car);
            var plan = planner.Plan(HomeX, HomeY, HomeYaw, _net.Route("S3", "AB2"));
            Assert.IsNotNull(plan, "a turn fits");
            Assert.GreaterOrEqual(plan.Legs.Count, 2, plan.Describe());
            Assert.IsTrue(plan.Legs.Exists(g => g.Dir < 0), "it reverses");
            for (int k = 1; k < plan.Legs.Count; k++) Assert.AreNotEqual(plan.Legs[k - 1].Dir, plan.Legs[k].Dir, "legs alternate");
            foreach (var g in plan.Legs)
                foreach (var q in g.Path)
                {
                    Assert.IsTrue(planner.Free(q.x, q.y, q.yaw), $"clear at ({q.x:0.00}, {q.y:0.00})");
                    Assert.LessOrEqual(Math.Abs(q.steer), planner.SteerUse + 1e-12, "within the planned steer");
                }
            var end = plan.Legs[plan.Legs.Count - 1].End;
            Assert.IsTrue(planner.AtGoal(end.x, end.y, end.yaw), "it ends on the route, facing along it");
            var (s, _, _) = planner.OnRoute(end.x, end.y);
            Assert.GreaterOrEqual(s, 0.0, "at or past the stop, not behind it");
            StringAssert.Contains("-point turn: ", plan.Describe());
        }

        [Test]
        public void NoTurnFitsInABox()
        {
            // A closed wall just round the car: there is nowhere to go.
            using var car = new TwinVehicle(RcTwinNative.Tier.L2DoubleTrack, Car());
            double[] box = { HomeX - 0.4, HomeY - 0.3, HomeX + 0.4, HomeY - 0.3, HomeX + 0.4, HomeY + 0.3, HomeX - 0.4, HomeY + 0.3 };
            var walls = new List<(double[], bool)>(Walls()) { (box, true) };
            var planner = Planner(car, walls);
            planner.MaxExpansions = 2000;
            Assert.IsNull(planner.Plan(HomeX, HomeY, HomeYaw, _net.Route("S3", "AB2")));
        }

        [Test]
        public void TheCarDrivesTheTurnOnTheCore()
        {
            // The plan, driven by TurnDriver on L2 with the campus kerbs as walls:
            // it reverses (the ESC's reverse), keeps off the kerbs, and ends
            // where the autopilot hands over to the route driver.
            using var car = new TwinVehicle(RcTwinNative.Tier.L2DoubleTrack, Car());
            var footprint = Footprint(car);
            car.Contact = footprint;
            foreach (var w in _net.Walls) car.AddWall(w.xy, w.closed);
            var planner = Planner(car);
            var plan = planner.Plan(HomeX, HomeY, HomeYaw, _net.Route("S3", "AB2"));
            Assert.IsNotNull(plan);
            car.Reset(new RcTwinNative.State { x = HomeX, y = HomeY, yaw = HomeYaw, soc = 1.0, packV = 11.1 });
            var p = car.Params;
            var drv = new TurnDriver(plan, p.steerMax);
            long contacts = 0;
            double slowest = 0.0, t = 0.0;
            for (; !drv.Done && t < 20.0; t += TwinAutopilot.ControlTick)
            {
                var (u, th) = drv.Act(car.State, TwinAutopilot.ControlTick);
                car.Step(u * p.steerMax, th >= 0.0 ? th * p.accelMax : th * p.decelMax, 0.001, 10);
                contacts += car.LastWallReport.contacts;
                slowest = Math.Min(slowest, car.State.vx);
            }
            var st = car.State;
            Assert.IsTrue(drv.Done, $"all legs driven in {t:0.0} s");
            Assert.Less(slowest, -0.3, "it reversed");
            Assert.AreEqual(0, contacts, "no kerb contacts");
            Assert.LessOrEqual(Math.Abs(st.vx), TwinAutopilot.StopSpeed, "at rest");
            Assert.IsTrue(planner.Near(st.x, st.y, st.yaw, TwinAutopilot.HandOverOffset, TwinAutopilot.HandOverHeading),
                          $"ends on the route: ({st.x:0.000}, {st.y:0.000}), {st.yaw * 180 / Math.PI:0.0} deg");
        }

        [Test, Timeout(300000)]
        public void EveryTurnRoundOnTheCampusIsPlanned()
        {
            // Every stop, arrived at along any route, then left along any route
            // that leaves behind the car: a turn fits in the road there.
            using var car = new TwinVehicle(RcTwinNative.Tier.L2DoubleTrack, Car());
            var planner = Planner(car);
            var seen = new HashSet<string>();
            var missing = new List<string>();
            foreach (var o in _net.StopNames())
                foreach (var d in _net.StopNames())
                {
                    if (_net.Stops[o].node == _net.Stops[d].node) continue;
                    var outR = _net.Route(o, d);
                    double hOut = Heading(outR);
                    foreach (var a in _net.StopNames())
                    {
                        if (_net.Stops[a].node == _net.Stops[o].node) continue;
                        var inR = _net.Route(a, o);
                        int n = inR.Length;
                        double hIn = Math.Atan2(inR[n - 2] - inR[n - 5], inR[n - 3] - inR[n - 6]);
                        if (!TurnPlanner.FacesAway(hIn, hOut, TwinAutopilot.TurnAroundRad)) continue;
                        if (!seen.Add($"{o} {Math.Round(hIn, 1)} {outR[3]:0.0} {outR[4]:0.0}")) continue;
                        if (planner.Plan(_net.Stops[o].nodeX, _net.Stops[o].nodeY, hIn, outR) == null)
                            missing.Add($"{a} to {o} to {d}");
                    }
                }
            Assert.Greater(seen.Count, 10, "the campus has turn-rounds to plan");
            Assert.IsEmpty(missing, "no turn found");
        }

        [Test, Timeout(300000)]
        public void EveryTurnRoundAndLineUpBetweenTheLanesIsPlanned()
        {
            // The shuttle fleet keeps left (ADR 0012): it arrives at the end of the
            // left lane and turns into the left lane of the way out, or, where the
            // road bends at the stop, lines up with it first.
            double lane = ShuttleFleet.LaneOffset;
            using var car = new TwinVehicle(RcTwinNative.Tier.L2DoubleTrack, Car());
            var planner = Planner(car);
            planner.GoalSMax = TwinAutopilot.LaneGoalSMax;
            var seen = new HashSet<string>();
            var missing = new List<string>();
            foreach (var o in _net.StopNames())
                foreach (var d in _net.StopNames())
                {
                    if (_net.Stops[o].node == _net.Stops[d].node) continue;
                    var outR = Lanes.Offset(_net.Route(o, d), lane);
                    double hOut = Heading(outR);
                    foreach (var a in _net.StopNames())
                    {
                        if (_net.Stops[a].node == _net.Stops[o].node) continue;
                        var inR = Lanes.Offset(_net.Route(a, o), lane);
                        // At a junction stop the bus waits short of the crossing road (TwinAutopilot.junctionSetback).
                        if (_net.Roads(o) >= 3) inR = Lanes.Shorten(inR, 0.5 * _net.RoadWidth + Footprint(car).halfLength);
                        int n = inR.Length;
                        double hIn = Math.Atan2(inR[n - 2] - inR[n - 5], inR[n - 3] - inR[n - 6]);
                        if (!TurnPlanner.FacesAway(hIn, hOut, TwinAutopilot.HandOverHeading)) continue;
                        if (!seen.Add($"{o} {Math.Round(hIn, 1)} {outR[3]:0.0} {outR[4]:0.0}")) continue;
                        if (planner.Plan(inR[n - 3], inR[n - 2], hIn, outR) == null)
                            missing.Add($"{a} to {o} to {d}");
                    }
                }
            Assert.Greater(seen.Count, 10, "the campus has turn-rounds to plan");
            Assert.IsEmpty(missing, "no turn found");
        }
    }
}
