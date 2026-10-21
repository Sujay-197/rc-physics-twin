// The campus road network (campus/campus_network.json) and its stop-to-stop
// routes, as sim_py/campus.py finds them: Dijkstra by edge length from one
// stop's node to another's, ties broken the same way, so the twin drives the
// route the RL environment and the fleet sim drive.
//
// Free of UnityEngine, like RcTwinNative.
using System;
using System.Collections.Generic;
using System.Globalization;

namespace RcTwin
{
    public sealed class CampusRoutes
    {
        public sealed class Stop
        {
            public string name;
            public int node, weight;
            public double x, y;            // the stop's marker (for AB stops, the building)
            public double nodeX, nodeY;    // where the route starts and ends, on the road
        }

        private sealed class Edge
        {
            public int id, a, b;
            public double length;
            public List<double[]> polyline;   // x, y, z
        }

        public readonly double RoadWidth;
        public readonly Dictionary<string, Stop> Stops = new Dictionary<string, Stop>();
        /// <summary>The kerb lines and building footprints the scene builder hands
        /// to TwinWalls: name, closed, and the points as x, y pairs.</summary>
        public readonly List<(string name, bool closed, double[] xy)> Walls = new List<(string, bool, double[])>();
        private readonly Dictionary<int, Edge> _edges = new Dictionary<int, Edge>();
        private readonly Dictionary<int, List<(int to, int edge, double length)>> _adj =
            new Dictionary<int, List<(int, int, double)>>();

        public CampusRoutes(string json)
        {
            var net = (Dictionary<string, object>)MiniJson.Parse(json);
            RoadWidth = D(((Dictionary<string, object>)net["road"])["width"]);
            var nodeXY = new Dictionary<int, (double, double)>();
            foreach (Dictionary<string, object> n in (List<object>)net["nodes"])
            {
                _adj[I(n["id"])] = new List<(int, int, double)>();
                nodeXY[I(n["id"])] = (D(n["x"]), D(n["y"]));
            }
            foreach (Dictionary<string, object> e in (List<object>)net["edges"])
            {
                var pl = new List<double[]>();
                foreach (List<object> q in (List<object>)e["polyline"]) pl.Add(new[] { D(q[0]), D(q[1]), D(q[2]) });
                var edge = new Edge { id = I(e["id"]), a = I(e["a"]), b = I(e["b"]), length = D(e["length"]), polyline = pl };
                _edges[edge.id] = edge;
                _adj[edge.a].Add((edge.b, edge.id, edge.length));
                _adj[edge.b].Add((edge.a, edge.id, edge.length));
            }
            // Python iterates sorted((neighbour, edge id, length)) tuples.
            foreach (var list in _adj.Values) list.Sort((p, q) => p.to != q.to ? p.to.CompareTo(q.to)
                                                       : p.edge != q.edge ? p.edge.CompareTo(q.edge) : p.length.CompareTo(q.length));
            foreach (Dictionary<string, object> s in (List<object>)net["stops"])
            {
                var stop = new Stop { name = (string)s["name"], node = I(s["node"]), weight = I(s["weight"]), x = D(s["x"]), y = D(s["y"]) };
                (stop.nodeX, stop.nodeY) = nodeXY[stop.node];
                Stops[stop.name] = stop;
            }
            if (net.TryGetValue("walls", out object walls))
                foreach (Dictionary<string, object> w in (List<object>)walls)
                {
                    var pts = (List<object>)w["points"];
                    var xy = new double[2 * pts.Count];
                    for (int k = 0; k < pts.Count; k++)
                    {
                        var q = (List<object>)pts[k];
                        xy[2 * k] = D(q[0]);
                        xy[2 * k + 1] = D(q[1]);
                    }
                    Walls.Add(((string)w["name"], (bool)w["closed"], xy));
                }
        }

        /// <summary>Every edge's centreline, x, y, z triples: the asphalt is
        /// RoadWidth wide about them.</summary>
        public IEnumerable<double[]> Centrelines()
        {
            foreach (var e in _edges.Values)
            {
                var xyz = new double[3 * e.polyline.Count];
                for (int k = 0; k < e.polyline.Count; k++)
                    for (int j = 0; j < 3; j++) xyz[3 * k + j] = e.polyline[k][j];
                yield return xyz;
            }
        }

        /// <summary>Stop names, letters then number (AB1 ... S6).</summary>
        public List<string> StopNames()
        {
            var l = new List<string>(Stops.Keys);
            l.Sort(CompareStops);
            return l;
        }

        /// <summary>Stop names in order: letters, then number (AB2 before AB12, S1 before S2).</summary>
        public static int CompareStops(string a, string b)
        {
            static (string, int) Key(string s)
            {
                int i = s.Length;
                while (i > 0 && char.IsDigit(s[i - 1])) i--;
                return (s.Substring(0, i), i < s.Length ? int.Parse(s.Substring(i), CultureInfo.InvariantCulture) : 0);
            }
            var (pa, na) = Key(a);
            var (pb, nb) = Key(b);
            int c = string.CompareOrdinal(pa, pb);
            return c != 0 ? c : na.CompareTo(nb);
        }

        /// <summary>Edges from node a to node b as (edge id, reversed?), in travel order.</summary>
        public List<(int edge, bool reversed)> ShortestPath(int a, int b)
        {
            var dist = new Dictionary<int, double> { [a] = 0.0 };
            var prev = new Dictionary<int, (int node, int edge)>();
            // heapq of (distance, node): the same pop order as Python's.
            var heap = new SortedSet<(double d, int n, long seq)>();
            long seq = 0;
            heap.Add((0.0, a, seq++));
            while (heap.Count > 0)
            {
                var top = heap.Min;
                heap.Remove(top);
                var (d, n, _) = top;
                if (n == b) break;
                if (d > (dist.TryGetValue(n, out double dn) ? dn : double.PositiveInfinity)) continue;
                foreach (var (m, eid, length) in _adj[n])
                {
                    double nd = d + length;
                    if (nd < (dist.TryGetValue(m, out double dm) ? dm : double.PositiveInfinity))
                    {
                        dist[m] = nd;
                        prev[m] = (n, eid);
                        heap.Add((nd, m, seq++));
                    }
                }
            }
            if (!dist.ContainsKey(b)) throw new ArgumentException($"no route from node {a} to node {b}");
            var path = new List<(int, bool)>();
            for (int n = b; n != a;)
            {
                var (p, eid) = prev[n];
                path.Add((eid, _edges[eid].a != p));
                n = p;
            }
            path.Reverse();
            return path;
        }

        /// <summary>The route's centreline from one stop to another, ISO x, y, z
        /// triples, consecutive duplicates dropped.</summary>
        public double[] Route(string origin, string destination)
        {
            int a = Stops[origin].node, b = Stops[destination].node;
            if (a == b) throw new ArgumentException($"{origin} and {destination} share node {a}");
            var pts = new List<double>();
            foreach (var (eid, rev) in ShortestPath(a, b))
            {
                var pl = _edges[eid].polyline;
                for (int k = 0; k < pl.Count; k++)
                {
                    var p = pl[rev ? pl.Count - 1 - k : k];
                    int n = pts.Count;
                    if (n >= 3 && Math.Sqrt((p[0] - pts[n - 3]) * (p[0] - pts[n - 3]) + (p[1] - pts[n - 2]) * (p[1] - pts[n - 2])) < 1e-6)
                        continue;
                    pts.Add(p[0]); pts.Add(p[1]); pts.Add(p[2]);
                }
            }
            return pts.ToArray();
        }

        /// <summary>The stop whose node (on the road) is nearest (x, y), and how far it is [m].</summary>
        public (string name, double distance) Nearest(double x, double y)
        {
            string best = null;
            double bd = double.PositiveInfinity;
            foreach (var s in Stops.Values)
            {
                double d = Math.Sqrt((s.nodeX - x) * (s.nodeX - x) + (s.nodeY - y) * (s.nodeY - y));
                if (d < bd || (d == bd && string.CompareOrdinal(s.name, best) < 0)) { bd = d; best = s.name; }
            }
            return (best, bd);
        }

        private static double D(object o) => Convert.ToDouble(o, CultureInfo.InvariantCulture);
        private static int I(object o) => (int)Math.Round(D(o));
    }
}
