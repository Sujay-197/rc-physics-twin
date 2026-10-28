// Several buses on the campus (ADR 0012): the Core has no car-to-car contact,
// so buses keep out of each other's way by traffic rules. They drive in the
// left lane (Lanes.Offset), and each control tick TrafficRules tells every
// bus how far it may still drive: never into another bus, and giving way
// where two paths cross. Free of UnityEngine, like Drivers.cs.
using System;
using System.Collections.Generic;

namespace RcTwin
{
    public static class Lanes
    {
        /// <summary>A route's centreline offset d to the left of its direction of travel
        /// (x, y, z triples; z kept), mitred at the corners so each segment lies d off its own.</summary>
        public static double[] Offset(double[] xyz, double d)
        {
            int n = xyz.Length / 3;
            var o = (double[])xyz.Clone();
            if (d == 0.0 || n < 2) return o;
            for (int i = 0; i < n; i++)
            {
                double nx = 0.0, ny = 0.0, first = double.NaN;
                if (i > 0) Add(Math.Atan2(xyz[3 * i + 1] - xyz[3 * i - 2], xyz[3 * i] - xyz[3 * i - 3]));
                if (i < n - 1) Add(Math.Atan2(xyz[3 * i + 4] - xyz[3 * i + 1], xyz[3 * i + 3] - xyz[3 * i]));
                double l = Math.Sqrt(nx * nx + ny * ny);
                nx /= l; ny /= l;
                // Mitre: the offset along the mean normal that puts each segment d off.
                double k = d / Math.Max(0.5, nx * -Math.Sin(first) + ny * Math.Cos(first));
                o[3 * i] += nx * k;
                o[3 * i + 1] += ny * k;

                void Add(double h)
                {
                    if (double.IsNaN(first)) first = h;
                    nx += -Math.Sin(h);
                    ny += Math.Cos(h);
                }
            }
            return o;
        }

        /// <summary>The polyline (x, y, z triples) with `cut` metres taken off its end; at least
        /// one segment's start is kept.</summary>
        public static double[] Shorten(double[] xyz, double cut)
        {
            int n = xyz.Length / 3;
            if (cut <= 0.0 || n < 2) return (double[])xyz.Clone();
            for (int i = n - 1; i > 0; i--)
            {
                double dx = xyz[3 * i] - xyz[3 * i - 3], dy = xyz[3 * i + 1] - xyz[3 * i - 2], dz = xyz[3 * i + 2] - xyz[3 * i - 1];
                double len = Math.Sqrt(dx * dx + dy * dy);
                if (len > cut || i == 1)
                {
                    double k = Math.Max(0.05, len - cut) / Math.Max(len, 1e-9);
                    var o = new double[3 * (i + 1)];
                    Array.Copy(xyz, o, 3 * i);
                    o[3 * i] = xyz[3 * i - 3] + k * dx;
                    o[3 * i + 1] = xyz[3 * i - 2] + k * dy;
                    o[3 * i + 2] = xyz[3 * i - 1] + k * dz;
                    return o;
                }
                cut -= len;
            }
            return (double[])xyz.Clone();
        }
    }

    /// <summary>A pose along a bus's path ahead: the CoG and heading, how far along [m], and how
    /// far the bus may stray from it there [m]: where the route bends, pure pursuit cuts the corner,
    /// toward the inside of the bend (positive: to the left).</summary>
    public struct PathPose
    {
        public double X, Y, Yaw, S, Slack;
        public PathPose(double x, double y, double yaw, double s, double slack = 0.0) { X = x; Y = y; Yaw = yaw; S = s; Slack = slack; }
    }

    /// <summary>One bus, as the traffic rules see it.</summary>
    public sealed class TrafficAgent
    {
        /// <summary>The bus number: on a tie, the lower goes first.</summary>
        public int Id;
        public double X, Y, Yaw;
        /// <summary>Speed along its path [m/s] (forward or reverse).</summary>
        public double Speed;
        /// <summary>Turning round or lining up: a slow move in a tight spot, given the way.</summary>
        public bool Manoeuvring;
        /// <summary>The footprint (ADR 0009): half extents about a centre this far ahead of the CoG [m].</summary>
        public double HalfLength, HalfWidth, CentreOffset;
        /// <summary>Where it will drive, from its pose (S = 0) on; empty while parked.</summary>
        public readonly List<PathPose> Path = new List<PathPose>();
        /// <summary>Where it is heading now: its present arc (speed and yaw rate held), which leaves
        /// the planned path where the car is off it (cutting a corner, settling after a turn round).</summary>
        public readonly List<PathPose> Motion = new List<PathPose>();

        /// <summary>Fill Motion: the arc of curvature k [1/m] (as yaw per metre driven forward, so the
        /// steering's) at speed v [m/s] (negative: reversing), every `step` metres for `length` metres.</summary>
        public void Arc(double v, double k, double length, double step)
        {
            Motion.Clear();
            if (Math.Abs(v) < 0.05) return;
            double sign = Math.Sign(v);
            for (double s = step; s <= length + 1e-9; s += step)
            {
                // Along the signed distance sigma (negative reversing): yaw = Yaw + k sigma.
                double sigma = sign * s, yaw = Yaw + k * sigma;
                double dx, dy;
                if (Math.Abs(k) < 1e-6) { dx = sigma * Math.Cos(Yaw); dy = sigma * Math.Sin(Yaw); }
                else { dx = (Math.Sin(yaw) - Math.Sin(Yaw)) / k; dy = (Math.Cos(Yaw) - Math.Cos(yaw)) / k; }
                Motion.Add(new PathPose(X + dx, Y + dy, yaw, s));
            }
        }
    }

    public sealed class TrafficRules
    {
        /// <summary>Kept between buses: each footprint is inflated by this [m]. Lanes 0.4 m apart leave
        /// oncoming buses 0.125 m between them: each may stray this much from its lane and still pass.</summary>
        public double Clearance = 0.03;
        /// <summary>Braking a held bus plans with: the stop plan's (TwinAutopilot.StopEarlyDecel) [m/s^2].</summary>
        public double Decel = 1.2;
        /// <summary>Paths are looked at this far past the braking distance [m].</summary>
        public double Margin = 1.0;
        /// <summary>Arrival times use at least this speed [m/s], so a bus at rest still has one.</summary>
        public double MinSpeed = 0.3;

        /// <summary>How far each bus may still drive along its path [m]; +inf: free.</summary>
        public double[] Hold { get; private set; } = new double[0];
        /// <summary>The bus each one is held by (its index), or -1.</summary>
        public int[] HeldBy { get; private set; } = new int[0];
        /// <summary>New overlaps of two buses' bare footprints, since the start (an incident).</summary>
        public int Overlaps { get; private set; }

        private readonly Dictionary<(int, int), int> _yield = new Dictionary<(int, int), int>();   // (lo, hi) id pair -> the id that gives way
        private readonly HashSet<(int, int)> _touching = new HashSet<(int, int)>();

        /// <summary>How far ahead a bus's path is needed at this speed [m].</summary>
        public double Horizon(double speed) => speed * speed / (2.0 * Decel) + Margin;

        public void Update(IReadOnlyList<TrafficAgent> a)
        {
            int n = a.Count;
            Hold = new double[n];
            HeldBy = new int[n];
            for (int i = 0; i < n; i++) { Hold[i] = double.PositiveInfinity; HeldBy[i] = -1; }
            var waits = new List<(int who, int on, double s, bool path)>();

            for (int i = 0; i < n; i++)
                for (int j = i + 1; j < n; j++)
                {
                    var key = Key(a[i], a[j]);
                    bool touch = Overlap(Pose(a[i]), a[i], Pose(a[j]), a[j], 0.0);
                    if (touch && _touching.Add(key)) Overlaps++;
                    else if (!touch) _touching.Remove(key);

                    if (Body(a, i, j) is double si) waits.Add((i, j, si, false));
                    if (Body(a, j, i) is double sj) waits.Add((j, i, sj, false));

                    // Paths that cross ahead: one gives way, and keeps giving way while they cross.
                    if (!Crossing(a[i], a[j], out int ci, out int cj)) { _yield.Remove(key); continue; }
                    if (!_yield.TryGetValue(key, out int loserId))
                        _yield[key] = loserId = Loser(a[i], ci, a[j], cj);
                    int lo = loserId == a[i].Id ? i : j, c = lo == i ? ci : cj;
                    waits.Add((lo, lo == i ? j : i, Math.Max(0.0, a[lo].Path[c].S - Step(a[lo], c)), true));
                }

            // A cycle of buses waiting on each other: the lowest-numbered one in it
            // stops giving way (a wait on a body cannot be lifted).
            for (bool broke = true; broke;)
            {
                broke = false;
                var cycle = Cycle(n, waits);
                if (cycle == null) break;
                int first = -1;
                foreach (int m in cycle)
                    if (waits.Exists(w => w.who == m && w.path && cycle.Contains(w.on)) && (first < 0 || a[m].Id < a[first].Id)) first = m;
                if (first < 0) break;
                int k = waits.FindIndex(w => w.who == first && w.path && cycle.Contains(w.on));
                var lifted = waits[k];
                waits.RemoveAt(k);
                _yield[Key(a[first], a[lifted.on])] = a[lifted.on].Id;
                broke = true;
            }
            foreach (var w in waits)
                if (w.s < Hold[w.who]) { Hold[w.who] = w.s; HeldBy[w.who] = w.on; }
        }

        private static (int, int) Key(TrafficAgent x, TrafficAgent y) => (Math.Min(x.Id, y.Id), Math.Max(x.Id, y.Id));

        // A cycle in the waits (as bus indices), or null.
        private static List<int> Cycle(int n, List<(int who, int on, double s, bool path)> waits)
        {
            var state = new int[n];   // 0 new, 1 on the stack, 2 done
            var stack = new List<int>();
            List<int> found = null;
            void Visit(int u)
            {
                state[u] = 1;
                stack.Add(u);
                foreach (var w in waits)
                {
                    if (found != null) return;
                    if (w.who != u) continue;
                    if (state[w.on] == 1) found = stack.GetRange(stack.IndexOf(w.on), stack.Count - stack.IndexOf(w.on));
                    else if (state[w.on] == 0) Visit(w.on);
                }
                stack.RemoveAt(stack.Count - 1);
                state[u] = 2;
            }
            for (int i = 0; i < n && found == null; i++) if (state[i] == 0) Visit(i);
            return found;
        }

        // Never into another bus: i stops short of j's footprint where its path, or
        // the arc it is on now, reaches it. Already too close, i may move only so as
        // not to get closer.
        private double? Body(IReadOnlyList<TrafficAgent> a, int i, int j)
        {
            double? path = Body(a, i, j, a[i].Path), now = Body(a, i, j, a[i].Motion);
            return path.HasValue && now.HasValue ? Math.Min(path.Value, now.Value) : path ?? now;
        }

        private double? Body(IReadOnlyList<TrafficAgent> a, int i, int j, List<PathPose> p)
        {
            var bj = Pose(a[j]);
            var now = Pose(a[i]);
            bool close = Overlap(now, a[i], bj, a[j], Clearance);
            double g0 = Gap(now, a[i], bj, a[j]);
            for (int k = 0; k < p.Count; k++)
            {
                if (p[k].S <= 0.0) continue;
                if (!Overlap(p[k], a[i], bj, a[j], Clearance)) continue;
                if (close && Squeezes(Gap(p[k], a[i], bj, a[j]), g0)) continue;
                return Math.Max(0.0, p[k].S - (k > 0 ? p[k].S - p[k - 1].S : p[k].S));
            }
            return null;
        }

        // The first poses along each path that conflict with the other path
        // (index 0 is the bus where it stands).
        private bool Crossing(TrafficAgent x, TrafficAgent y, out int cx, out int cy)
        {
            cx = cy = int.MaxValue;
            if (x.Path.Count == 0 || y.Path.Count == 0) return false;
            double reach = 2.0 * Radius(x) + 2.0 * Radius(y);
            if (!Near(x.Path, y.Path, reach)) return false;
            double rr = Radius(x) + Radius(y) + 2.0 * Clearance;
            // Already too close: only poses that would bring them closer still count.
            var x0 = Pose(x);
            var y0 = Pose(y);
            bool close = Overlap(x0, x, y0, y, Clearance);
            double g0 = Gap(x0, x, y0, y);
            for (int k = 0; k < x.Path.Count; k++)
                for (int m = 0; m < y.Path.Count; m++)
                {
                    if (k >= cx && m >= cy) continue;
                    var p = x.Path[k];
                    var q = y.Path[m];
                    if ((p.X - q.X) * (p.X - q.X) + (p.Y - q.Y) * (p.Y - q.Y) > rr * rr) continue;
                    if (!Overlap(p, x, q, y, Clearance)) continue;
                    if (close && Squeezes(Gap(p, x, q, y), g0)) continue;
                    if (k < cx) cx = k;
                    if (m < cy) cy = m;
                }
            return cx != int.MaxValue;
        }

        // Who gives way at a crossing: not a bus standing in the other's path, nor one
        // that can no longer stop before it, nor one turning round or lining up; else
        // the later arrival; on a tie, the higher number.
        private int Loser(TrafficAgent x, int cx, TrafficAgent y, int cy)
        {
            bool xIn = cx == 0, yIn = cy == 0;
            if (xIn != yIn) return xIn ? y.Id : x.Id;
            if (!xIn)
            {
                bool xCommitted = x.Path[cx].S < x.Speed * x.Speed / (2.0 * Decel);
                bool yCommitted = y.Path[cy].S < y.Speed * y.Speed / (2.0 * Decel);
                if (xCommitted != yCommitted) return xCommitted ? y.Id : x.Id;
                if (x.Manoeuvring != y.Manoeuvring) return x.Manoeuvring ? y.Id : x.Id;
                double tx = x.Path[cx].S / Math.Max(Math.Abs(x.Speed), MinSpeed);
                double ty = y.Path[cy].S / Math.Max(Math.Abs(y.Speed), MinSpeed);
                if (Math.Abs(tx - ty) > 1e-6) return tx < ty ? y.Id : x.Id;
            }
            return x.Id < y.Id ? y.Id : x.Id;
        }

        // Two buses already within the clearance may move on if the gap between them does not
        // shrink, or if it keeps at least half the clearance (a bus squeezing past one parked).
        private bool Squeezes(double gap, double gap0) => gap >= gap0 - 1e-3 || gap >= Clearance;

        // ------------------------------------------------------------ geometry

        private static PathPose Pose(TrafficAgent a) => new PathPose(a.X, a.Y, a.Yaw, 0.0);

        private static double Step(TrafficAgent a, int k) =>
            k > 0 ? a.Path[k].S - a.Path[k - 1].S : 0.0;

        private double Radius(TrafficAgent a) =>
            Math.Sqrt(a.HalfLength * a.HalfLength + a.HalfWidth * a.HalfWidth) + Clearance;

        // Whether any pose of one path comes within reach of any pose of the other (bounding boxes).
        private static bool Near(List<PathPose> p, List<PathPose> q, double reach)
        {
            double ax0 = double.MaxValue, ay0 = double.MaxValue, ax1 = double.MinValue, ay1 = double.MinValue;
            foreach (var x in p) { ax0 = Math.Min(ax0, x.X); ay0 = Math.Min(ay0, x.Y); ax1 = Math.Max(ax1, x.X); ay1 = Math.Max(ay1, x.Y); }
            foreach (var y in q)
                if (y.X > ax0 - reach && y.X < ax1 + reach && y.Y > ay0 - reach && y.Y < ay1 + reach) return true;
            return false;
        }

        /// <summary>How far apart two bare footprints are along the axis that separates them best [m];
        /// negative: how deep they overlap. Two buses passing side by side keep the same gap.</summary>
        public static double Gap(PathPose p, TrafficAgent a, PathPose q, TrafficAgent b)
        {
            var (ax, ay) = Centre(p, a);
            var (bx, by) = Centre(q, b);
            double dx = bx - ax, dy = by - ay;
            double ca = Math.Cos(p.Yaw), sa = Math.Sin(p.Yaw), cb = Math.Cos(q.Yaw), sb = Math.Sin(q.Yaw);
            double gap = double.NegativeInfinity;
            foreach (var (ux, uy) in new[] { (ca, sa), (-sa, ca), (cb, sb), (-sb, cb) })
            {
                double ra = a.HalfLength * Math.Abs(ca * ux + sa * uy) + a.HalfWidth * Math.Abs(-sa * ux + ca * uy);
                double rb = b.HalfLength * Math.Abs(cb * ux + sb * uy) + b.HalfWidth * Math.Abs(-sb * ux + cb * uy);
                gap = Math.Max(gap, Math.Abs(dx * ux + dy * uy) - ra - rb);
            }
            return gap;
        }

        private static (double, double) Centre(PathPose p, TrafficAgent a) =>
            (p.X + a.CentreOffset * Math.Cos(p.Yaw), p.Y + a.CentreOffset * Math.Sin(p.Yaw));

        /// <summary>Whether two footprints, each inflated by c, overlap (separating axes). A pose's
        /// slack widens its footprint toward that side by the slack.</summary>
        public static bool Overlap(PathPose p, TrafficAgent a, PathPose q, TrafficAgent b, double c)
        {
            var (ax, ay) = Centre(p, a);
            var (bx, by) = Centre(q, b);
            double ca = Math.Cos(p.Yaw), sa = Math.Sin(p.Yaw), cb = Math.Cos(q.Yaw), sb = Math.Sin(q.Yaw);
            // Slack: half of it moves the centre to its side, half widens the footprint.
            ax += -sa * 0.5 * p.Slack; ay += ca * 0.5 * p.Slack;
            bx += -sb * 0.5 * q.Slack; by += cb * 0.5 * q.Slack;
            double dx = bx - ax, dy = by - ay;
            double al = a.HalfLength + c, aw = a.HalfWidth + c + 0.5 * Math.Abs(p.Slack);
            double bl = b.HalfLength + c, bw = b.HalfWidth + c + 0.5 * Math.Abs(q.Slack);
            // Axes: each box's length and width directions.
            foreach (var (ux, uy) in new[] { (ca, sa), (-sa, ca), (cb, sb), (-sb, cb) })
            {
                double ra = al * Math.Abs(ca * ux + sa * uy) + aw * Math.Abs(-sa * ux + ca * uy);
                double rb = bl * Math.Abs(cb * ux + sb * uy) + bw * Math.Abs(-sb * ux + cb * uy);
                if (Math.Abs(dx * ux + dy * uy) > ra + rb) return false;
            }
            return true;
        }
    }
}
