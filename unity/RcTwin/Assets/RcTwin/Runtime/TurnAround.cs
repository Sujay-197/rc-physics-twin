// Turning round at a stop. A route can leave a stop the way the car came in;
// the autopilot then drives a multi-point turn in the road before the trip.
//
// TurnPlanner finds the turn: a hybrid A* search over short forward and
// reverse arcs within the steer limit, on the kinematic bicycle about the
// CoG. Every pose on the way keeps the car's wall footprint (ADR 0009), plus
// a clearance, clear of the walls (the kerb lines and buildings) and inside
// the road corridor (within half the road width of a centreline). It ends on
// the route's centreline, facing along it, just past the stop. Changing
// direction costs extra, so it prefers the fewest legs.
//
// TurnDriver drives a plan leg by leg: set the steering at rest, roll along
// the leg's arcs at walking pace with feedback on the offset and heading
// from the planned path, and stop at the leg's end. Its command is the
// autopilot's, steer and throttle in [-1, 1]; a negative throttle at rest
// reverses in the Core, so the speed loop is signed.
//
// Free of UnityEngine, like RcTwinNative.
using System;
using System.Collections.Generic;

namespace RcTwin
{
    /// <summary>One point of a planned leg: the CoG pose, the steer (fraction
    /// of the steer limit, + left) to drive on from it, and the distance along the leg.</summary>
    public struct TurnPoint
    {
        public double x, y, yaw, steer, s;
    }

    /// <summary>One leg of a turn: forward (+1) or reverse (-1), at least two points.</summary>
    public sealed class TurnLeg
    {
        public int Dir;
        public readonly List<TurnPoint> Path = new List<TurnPoint>();
        public double Length => Path[Path.Count - 1].s;
        public TurnPoint End => Path[Path.Count - 1];
    }

    public sealed class TurnPlan
    {
        public readonly List<TurnLeg> Legs = new List<TurnLeg>();

        public double Length
        {
            get { double l = 0.0; foreach (var g in Legs) l += g.Length; return l; }
        }

        /// <summary>"three-point turn: forward 0.40 m, reverse 0.62 m, forward 0.95 m".</summary>
        public string Describe()
        {
            string[] words = { "", "one", "two", "three", "four", "five", "six" };
            int n = Legs.Count;
            var parts = new List<string>();
            foreach (var g in Legs) parts.Add($"{(g.Dir > 0 ? "forward" : "reverse")} {g.Length:0.00} m");
            return $"{(n < words.Length ? words[n] : n.ToString())}-point turn: {string.Join(", ", parts)}";
        }
    }

    public sealed class TurnPlanner
    {
        // The car (nominal), and its wall footprint.
        public readonly double Wheelbase, Lr, SteerMax;
        public readonly double HalfLength, HalfWidth, CentreOffset;

        /// <summary>Kept between the footprint and any wall or the road's edge [m].</summary>
        public double Clearance = 0.04;
        /// <summary>The largest steer planned, as a fraction of the limit: the rest is the tracker's.</summary>
        public double SteerUse = 0.85;
        /// <summary>Arc length of one search step, and of the collision checks along it [m].</summary>
        public double Step = 0.15, SubStep = 0.05;
        /// <summary>Search grid: position cell [m] and heading bins per turn.</summary>
        public double Cell = 0.1;
        public int YawBins = 36;
        /// <summary>Costs, in metres driven: reversing costs this much more per metre;
        /// each change of direction this much.</summary>
        public double ReverseCost = 0.5, SwitchCost = 1.0;
        public int MaxExpansions = 30000;
        /// <summary>The search stays within this far of the start [m].</summary>
        public double Reach = 3.0;
        /// <summary>Where the turn may end: off the centreline [m], off its heading [rad],
        /// and between these distances along the route [m].</summary>
        public double GoalOffset = 0.06, GoalHeading = 0.1, GoalSMin = 0.0, GoalSMax = 1.5;

        private readonly List<double[]> _walls = new List<double[]>();        // ax, ay, bx, by
        private readonly List<double[]> _centrelines = new List<double[]>();  // ax, ay, bx, by
        private readonly double _halfRoad;

        /// <param name="walls">Wall polylines as x, y pairs, and whether each is closed.</param>
        /// <param name="centrelines">Road centrelines as x, y pairs (or x, y, z triples with stride 3).</param>
        public TurnPlanner(in RcTwinNative.Params car, in RcTwinNative.Contact footprint,
                           IEnumerable<(double[] xy, bool closed)> walls,
                           IEnumerable<double[]> centrelines, int centrelineStride, double roadWidth)
        {
            Wheelbase = car.lf + car.lr;
            Lr = car.lr;
            SteerMax = car.steerMax;
            HalfLength = footprint.halfLength;
            HalfWidth = footprint.halfWidth;
            CentreOffset = footprint.centreOffset;
            _halfRoad = 0.5 * roadWidth;
            foreach (var (xy, closed) in walls)
            {
                int n = xy.Length / 2;
                for (int i = 0; i + 1 < n; i++) _walls.Add(new[] { xy[2 * i], xy[2 * i + 1], xy[2 * i + 2], xy[2 * i + 3] });
                if (closed && n > 2) _walls.Add(new[] { xy[2 * n - 2], xy[2 * n - 1], xy[0], xy[1] });
            }
            foreach (var pl in centrelines)
            {
                int k = centrelineStride, n = pl.Length / k;
                for (int i = 0; i + 1 < n; i++) _centrelines.Add(new[] { pl[k * i], pl[k * i + 1], pl[k * (i + 1)], pl[k * (i + 1) + 1] });
            }
        }

        // ------------------------------------------------------------ model

        /// <summary>The CoG's motion along an arc of length ds (>= 0) in direction
        /// dir at steer fraction u: kinematic bicycle, slip angle at the CoG.</summary>
        public void Move(ref double x, ref double y, ref double yaw, int dir, double u, double ds)
        {
            double beta = Math.Atan(Lr / Wheelbase * Math.Tan(u * SteerMax));
            double dyaw = dir * ds * Math.Sin(beta) / Lr;
            // Along the chord of the CoG's circle (radius lr / sin beta), exactly.
            double chord = Math.Abs(dyaw) > 1e-9 ? Math.Abs(2.0 * Lr / Math.Sin(beta) * Math.Sin(0.5 * dyaw)) : ds;
            double mid = yaw + beta + 0.5 * dyaw;
            x += dir * chord * Math.Cos(mid);
            y += dir * chord * Math.Sin(mid);
            yaw = TwinObservation.Wrap(yaw + dyaw);
        }

        // ---------------------------------------------------------- collision

        private List<double[]> _nearWalls, _nearRoad;

        /// <summary>The footprint at this CoG pose, with the clearance, is clear of
        /// every wall and inside the road corridor.</summary>
        public bool Free(double x, double y, double yaw)
        {
            var walls = _nearWalls ?? _walls;
            var road = _nearRoad ?? _centrelines;
            double c = Math.Cos(yaw), s = Math.Sin(yaw);
            double cx = x + c * CentreOffset, cy = y + s * CentreOffset;
            double hl = HalfLength + Clearance, hw = HalfWidth + Clearance;
            foreach (var w in walls)
            {
                // The wall segment in the footprint's frame, clipped to the box.
                double ax = w[0] - cx, ay = w[1] - cy, bx = w[2] - cx, by = w[3] - cy;
                double u0 = c * ax + s * ay, v0 = -s * ax + c * ay;
                double u1 = c * bx + s * by, v1 = -s * bx + c * by;
                if (SegmentHitsBox(u0, v0, u1, v1, hl, hw)) return false;
            }
            if (road.Count == 0) return true;
            // Corners and side midpoints of the footprint, each on the asphalt.
            for (int i = -1; i <= 1; i++)
                for (int j = -1; j <= 1; j++)
                {
                    if (i == 0 && j == 0) continue;
                    double px = cx + c * i * hl - s * j * hw, py = cy + s * i * hl + c * j * hw;
                    if (DistanceToRoad(px, py, road) > _halfRoad) return false;
                }
            return true;
        }

        private static double DistanceToRoad(double px, double py, List<double[]> road)
        {
            double best = double.PositiveInfinity;
            foreach (var e in road) best = Math.Min(best, SegmentDistance(px, py, e[0], e[1], e[2], e[3]));
            return best;
        }

        private static double SegmentDistance(double px, double py, double ax, double ay, double bx, double by)
        {
            double dx = bx - ax, dy = by - ay, l2 = dx * dx + dy * dy;
            double t = l2 > 0.0 ? Math.Max(0.0, Math.Min(1.0, ((px - ax) * dx + (py - ay) * dy) / l2)) : 0.0;
            double qx = ax + t * dx - px, qy = ay + t * dy - py;
            return Math.Sqrt(qx * qx + qy * qy);
        }

        // Liang-Barsky: does segment (u0, v0)-(u1, v1) touch |u| <= hl, |v| <= hw?
        private static bool SegmentHitsBox(double u0, double v0, double u1, double v1, double hl, double hw)
        {
            double t0 = 0.0, t1 = 1.0, du = u1 - u0, dv = v1 - v0;
            bool Clip(double p, double q)
            {
                if (Math.Abs(p) < 1e-15) return q >= 0.0;
                double r = q / p;
                if (p < 0.0) { if (r > t1) return false; if (r > t0) t0 = r; }
                else { if (r < t0) return false; if (r < t1) t1 = r; }
                return true;
            }
            return Clip(-du, u0 + hl) && Clip(du, hl - u0) && Clip(-dv, v0 + hw) && Clip(dv, hw - v0);
        }

        // -------------------------------------------------------------- route

        private double[] _route;   // x, y pairs of the route's first metres
        private double[] _routeS;

        private void SetRoute(double[] xyz)
        {
            var xy = new List<double> { xyz[0], xyz[1] };
            var ss = new List<double> { 0.0 };
            for (int i = 3; i + 2 < xyz.Length && ss[ss.Count - 1] < GoalSMax + 2.0; i += 3)
            {
                double dx = xyz[i] - xy[xy.Count - 2], dy = xyz[i + 1] - xy[xy.Count - 1];
                ss.Add(ss[ss.Count - 1] + Math.Sqrt(dx * dx + dy * dy));
                xy.Add(xyz[i]); xy.Add(xyz[i + 1]);
            }
            _route = xy.ToArray();
            _routeS = ss.ToArray();
        }

        /// <summary>The nearest point of the route's start: distance along it, signed
        /// offset (left positive) and the centreline's heading there.</summary>
        public (double s, double d, double heading) OnRoute(double x, double y)
        {
            double bestD2 = double.PositiveInfinity, bs = 0.0, bd = 0.0, bh = 0.0;
            for (int i = 0; i + 1 < _routeS.Length; i++)
            {
                double ax = _route[2 * i], ay = _route[2 * i + 1], dx = _route[2 * i + 2] - ax, dy = _route[2 * i + 3] - ay;
                double l = Math.Sqrt(dx * dx + dy * dy);
                if (l < 1e-9) continue;
                double tx = dx / l, ty = dy / l;
                // Before the first point, s goes negative: the car is behind the stop.
                double t = Math.Max(i == 0 ? double.NegativeInfinity : 0.0, Math.Min(l, (x - ax) * tx + (y - ay) * ty));
                double qx = ax + t * tx - x, qy = ay + t * ty - y, d2 = qx * qx + qy * qy;
                if (d2 < bestD2)
                {
                    bestD2 = d2;
                    bs = _routeS[i] + t;
                    bd = -(x - ax) * ty + (y - ay) * tx;
                    bh = Math.Atan2(ty, tx);
                }
            }
            return (bs, bd, bh);
        }

        /// <summary>A pose the route driver can take over from.</summary>
        public bool AtGoal(double x, double y, double yaw) => Near(x, y, yaw, GoalOffset, GoalHeading);

        /// <summary>On the route's start, within these of its centreline [m] and heading [rad],
        /// and no further along than GoalSMax (the last Plan's route).</summary>
        public bool Near(double x, double y, double yaw, double offset, double heading)
        {
            var (s, d, h) = OnRoute(x, y);
            return s >= GoalSMin && s <= GoalSMax && Math.Abs(d) <= offset
                   && Math.Abs(TwinObservation.Wrap(yaw - h)) <= heading;
        }

        /// <summary>Whether the route at its start points away from the car's
        /// heading by more than `limit` [rad]: the car must turn round first.</summary>
        public static bool FacesAway(double yaw, double routeHeading, double limit) =>
            Math.Abs(TwinObservation.Wrap(yaw - routeHeading)) > limit;

        // ------------------------------------------------------------- search

        private struct Node
        {
            public double x, y, yaw, g;
            public int parent, dir;
            public double steer;
        }

        /// <summary>Expansions the last Plan call made.</summary>
        public int Expansions { get; private set; }

        /// <summary>A turn from the CoG pose (x, y, yaw) onto the start of the route
        /// (ISO x, y, z triples), or null if none fits.</summary>
        public TurnPlan Plan(double x, double y, double yaw, double[] routeXyz)
        {
            SetRoute(routeXyz);
            double near = Reach + HalfLength + 1.0;
            _nearWalls = _walls.FindAll(w => SegmentDistance(x, y, w[0], w[1], w[2], w[3]) < near);
            _nearRoad = _centrelines.FindAll(e => SegmentDistance(x, y, e[0], e[1], e[2], e[3]) < near + _halfRoad);
            try { return Search(x, y, yaw); }
            finally { _nearWalls = _nearRoad = null; }
        }

        private TurnPlan Search(double x0, double y0, double yaw0)
        {
            // Aim the heuristic at a point on the centreline inside the goal span.
            double sAim = Math.Min(0.5 * (GoalSMin + GoalSMax), _routeS[_routeS.Length - 1]);
            double gx = 0, gy = 0, gh = 0;
            for (int i = 0; i + 1 < _routeS.Length; i++)
                if (_routeS[i + 1] >= sAim || i + 2 == _routeS.Length)
                {
                    double t = (sAim - _routeS[i]) / Math.Max(1e-9, _routeS[i + 1] - _routeS[i]);
                    gx = _route[2 * i] + t * (_route[2 * i + 2] - _route[2 * i]);
                    gy = _route[2 * i + 1] + t * (_route[2 * i + 3] - _route[2 * i + 1]);
                    gh = Math.Atan2(_route[2 * i + 3] - _route[2 * i + 1], _route[2 * i + 2] - _route[2 * i]);
                    break;
                }
            double rMin = Lr / Math.Sin(Math.Atan(Lr / Wheelbase * Math.Tan(SteerUse * SteerMax)));
            double H(double x, double y, double yaw) =>
                Math.Sqrt((x - gx) * (x - gx) + (y - gy) * (y - gy)) + 0.5 * rMin * Math.Abs(TwinObservation.Wrap(yaw - gh));

            var steers = new[] { -SteerUse, -0.5 * SteerUse, 0.0, 0.5 * SteerUse, SteerUse };
            var nodes = new List<Node> { new Node { x = x0, y = y0, yaw = yaw0, parent = -1 } };
            var best = new Dictionary<long, double>();
            var open = new SortedSet<(double f, int i)> { (H(x0, y0, yaw0), 0) };
            long Key(double x, double y, double yaw)
            {
                long ix = (long)Math.Floor((x - x0) / Cell) + 4096, iy = (long)Math.Floor((y - y0) / Cell) + 4096;
                long ib = ((long)Math.Floor(TwinObservation.Wrap(yaw) / (2 * Math.PI) * YawBins) % YawBins + YawBins) % YawBins;
                return (ix * 8192 + iy) * YawBins + ib;
            }
            best[Key(x0, y0, yaw0)] = 0.0;
            int subs = Math.Max(1, (int)Math.Round(Step / SubStep));
            Expansions = 0;
            while (open.Count > 0 && Expansions < MaxExpansions)
            {
                var top = open.Min;
                open.Remove(top);
                var n = nodes[top.i];
                Expansions++;
                foreach (int dir in new[] { 1, -1 })
                    foreach (double u in steers)
                    {
                        double x = n.x, y = n.y, yaw = n.yaw;
                        bool ok = true;
                        for (int k = 0; k < subs && ok; k++)
                        {
                            Move(ref x, ref y, ref yaw, dir, u, Step / subs);
                            ok = (x - x0) * (x - x0) + (y - y0) * (y - y0) <= Reach * Reach && Free(x, y, yaw);
                        }
                        if (!ok) continue;
                        double g = n.g + Step * (dir < 0 ? 1.0 + ReverseCost : 1.0)
                                   + (n.dir != 0 && n.dir != dir ? SwitchCost : 0.0)
                                   + 0.05 * Math.Abs(u - n.steer);
                        var child = new Node { x = x, y = y, yaw = yaw, g = g, parent = top.i, dir = dir, steer = u };
                        if (AtGoal(x, y, yaw))
                        {
                            nodes.Add(child);
                            return Build(nodes, nodes.Count - 1, subs);
                        }
                        long key = Key(x, y, yaw);
                        if (best.TryGetValue(key, out double gb) && gb <= g) continue;
                        best[key] = g;
                        nodes.Add(child);
                        open.Add((g + H(x, y, yaw), nodes.Count - 1));
                    }
            }
            return null;
        }

        // The chain of nodes as legs, each arc resampled at the collision step.
        private TurnPlan Build(List<Node> nodes, int last, int subs)
        {
            var chain = new List<Node>();
            for (int i = last; i >= 0; i = nodes[i].parent) chain.Add(nodes[i]);
            chain.Reverse();
            var plan = new TurnPlan();
            TurnLeg leg = null;
            for (int k = 1; k < chain.Count; k++)
            {
                var a = chain[k - 1];
                var b = chain[k];
                if (leg == null || leg.Dir != b.dir)
                {
                    leg = new TurnLeg { Dir = b.dir };
                    leg.Path.Add(new TurnPoint { x = a.x, y = a.y, yaw = a.yaw, steer = b.steer, s = 0.0 });
                    plan.Legs.Add(leg);
                }
                else
                {
                    var p = leg.Path[leg.Path.Count - 1];
                    p.steer = b.steer;   // from here on, this arc's steer
                    leg.Path[leg.Path.Count - 1] = p;
                }
                double x = a.x, y = a.y, yaw = a.yaw, s = leg.Length;
                for (int j = 0; j < subs; j++)
                {
                    Move(ref x, ref y, ref yaw, b.dir, b.steer, Step / subs);
                    s += Step / subs;
                    leg.Path.Add(new TurnPoint { x = x, y = y, yaw = yaw, steer = b.steer, s = s });
                }
            }
            return plan;
        }
    }

    /// <summary>Drives a TurnPlan through the car's command, leg by leg.</summary>
    public sealed class TurnDriver
    {
        /// <summary>Speed along a leg [m/s], braking to the leg's end [m/s^2], and throttle per m/s of speed error.
        /// The cruise trades time for precision. On TwinCar's Core, kinematic at 0.3 m/s and below
        /// (ADR 0011), every campus turn-round drives clean from 0.2 to 0.7 m/s; at 0.3 they end
        /// within 1.1 cm and 1.5 deg of the plan, at 0.7 within 1.9 cm and 4.9 deg in half the time
        /// (5.5 against 11 s a turn). Without that blend the car yawed erratically at full lock below
        /// 0.4 m/s and touched kerbs, which is why 0.7 was chosen. The braking stays inside what the
        /// motor brake gives (its regen current limit, about 2.3 m/s^2 for the reference car).</summary>
        public double Cruise = 0.7, Decel = 1.8, KSpeed = 1.5;
        /// <summary>Steer feedback [rad] per metre off the planned path and per radian off its heading.</summary>
        public double KOffset = 1.5, KHeading = 1.0;
        /// <summary>At rest below this [m/s]; the steering is set when within this of its target [rad].</summary>
        public double RestSpeed = 0.02, SteerSet = 0.03;

        private enum Phase { Settle, Move, Stop }

        private readonly double _steerMax;
        private Phase _phase = Phase.Settle;
        private int _i;
        private double _phaseTime, _lastSteer;

        public TurnPlan Plan { get; }
        /// <summary>The leg being driven; Plan.Legs.Count once all are done.</summary>
        public int Leg { get; private set; }
        public bool Done => Leg >= Plan.Legs.Count;
        /// <summary>Set for one tick when a leg has ended at rest.</summary>
        public bool LegEnded { get; private set; }
        /// <summary>Distance left on the current leg [m].</summary>
        public double LegRemaining { get; private set; }
        /// <summary>Not moving although it should be (against a wall, say) for this long [s].</summary>
        public double Stalled { get; private set; }

        public TurnDriver(TurnPlan plan, double steerMax)
        {
            Plan = plan;
            _steerMax = steerMax;
            LegRemaining = plan.Legs.Count > 0 ? plan.Legs[0].Length : 0.0;
        }

        public TurnLeg Current => Done ? null : Plan.Legs[Leg];

        /// <summary>The planned poses still to drive: the current leg from where the car is, then the rest.</summary>
        public IEnumerable<TurnPoint> Ahead()
        {
            for (int l = Leg; l < Plan.Legs.Count; l++)
            {
                var path = Plan.Legs[l].Path;
                for (int k = l == Leg ? _i : 0; k < path.Count; k++) yield return path[k];
            }
        }

        public (double steer, double throttle) Act(in RcTwinNative.State st, double dt)
        {
            LegEnded = false;
            if (Done) return (0.0, Hold(st.vx));
            var leg = Plan.Legs[Leg];
            _phaseTime += dt;
            if (_phase == Phase.Settle)
            {
                // Set the steering at rest, then go.
                double u0 = leg.Path[0].steer;
                _lastSteer = u0;
                if ((Math.Abs(st.steer - u0 * _steerMax) < SteerSet && Math.Abs(st.vx) < RestSpeed) || _phaseTime > 0.5)
                    Enter(Phase.Move);
                return (u0, Hold(st.vx));
            }
            if (_phase == Phase.Stop)
            {
                if (Math.Abs(st.vx) < RestSpeed)
                {
                    Leg++;
                    _i = 0;
                    LegEnded = true;
                    LegRemaining = Done ? 0.0 : Plan.Legs[Leg].Length;
                    Enter(Phase.Settle);
                }
                return (_lastSteer, Hold(st.vx));
            }

            // Move: the nearest planned point, a little way ahead of the last.
            var path = leg.Path;
            double bestD2 = double.PositiveInfinity;
            int bi = _i;
            for (int k = _i; k < Math.Min(path.Count, _i + 16); k++)
            {
                double dx = st.x - path[k].x, dy = st.y - path[k].y, d2 = dx * dx + dy * dy;
                if (d2 < bestD2) { bestD2 = d2; bi = k; }
            }
            _i = bi;
            var p = path[bi];
            double c = Math.Cos(p.yaw), s = Math.Sin(p.yaw);
            double ex = st.x - p.x, ey = st.y - p.y;
            double along = leg.Dir * (c * ex + s * ey);
            double offset = -s * ex + c * ey;                       // left of the path
            double headingErr = TwinObservation.Wrap(st.yaw - p.yaw);
            LegRemaining = leg.Length - (p.s + along);
            if (LegRemaining <= 0.005) { Enter(Phase.Stop); return (_lastSteer, Hold(st.vx)); }

            // Forward, steer against both errors; in reverse the heading term flips.
            double fb = leg.Dir > 0 ? -(KOffset * offset + KHeading * headingErr)
                                    : -KOffset * offset + KHeading * headingErr;
            _lastSteer = DriverMath.Clip(p.steer + fb / _steerMax);
            double v = Math.Max(0.06, Math.Min(Cruise, Math.Sqrt(2.0 * Decel * Math.Max(0.0, LegRemaining - 0.005))));
            Stalled = Math.Abs(st.vx) < RestSpeed && _phaseTime > 0.5 ? Stalled + dt : 0.0;
            return (_lastSteer, DriverMath.Clip(KSpeed * (leg.Dir * v - st.vx)));
        }

        private double Hold(double vx) => DriverMath.Clip(-KSpeed * vx);

        private void Enter(Phase p)
        {
            _phase = p;
            _phaseTime = 0.0;
            Stalled = 0.0;
        }
    }
}
