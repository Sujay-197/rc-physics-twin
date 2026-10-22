// How far and how long between campus stops, along the routes the autopilot
// drives (CampusRoutes.Route, the fleet sim's shortest paths): what the
// shuttle plans its stops with.
using System;
using System.Collections.Generic;

namespace RcTwin
{
    public interface ITravelModel
    {
        /// <summary>Route length from stop a to stop b [m]; 0 for a == b.</summary>
        double Distance(string a, string b);
        /// <summary>Driving time at the cruise speed [s].</summary>
        double DriveTime(string a, string b);
        /// <summary>Extra time at `via` when the shuttle arrives from `from` and
        /// leaves toward `to` the way it came in (the autopilot turns round) [s].</summary>
        double TurnAllowance(string from, string via, string to);
        /// <summary>The other stops on the route from a to b, with their arc length along it, in order.</summary>
        IReadOnlyList<(string stop, double s)> StopsOnRoute(string a, string b);
    }

    public sealed class RouteTravelModel : ITravelModel
    {
        /// <summary>A turn-round at a stop, planned and driven at 0.7 m/s: 3 to 8 s on the campus.</summary>
        public const double TurnRoundTime = 8.0;

        private readonly CampusRoutes _net;
        private readonly double _cruise;
        private readonly Dictionary<(string, string), Leg> _legs = new Dictionary<(string, string), Leg>();

        private sealed class Leg
        {
            public double length, startHeading, endHeading;
            public List<(string stop, double s)> on;
        }

        public RouteTravelModel(CampusRoutes net, double cruise)
        {
            _net = net;
            _cruise = cruise;
        }

        public double Distance(string a, string b) => a == b ? 0.0 : Get(a, b).length;

        public double DriveTime(string a, string b) => Distance(a, b) / _cruise;

        public double TurnAllowance(string from, string via, string to)
        {
            if (from == null || from == via || to == via) return 0.0;
            double d = Math.Abs(Wrap(Get(via, to).startHeading - Get(from, via).endHeading));
            return d > 0.5 * Math.PI ? TurnRoundTime : 0.0;
        }

        public IReadOnlyList<(string stop, double s)> StopsOnRoute(string a, string b) =>
            a == b ? new List<(string, double)>() : Get(a, b).on;

        private Leg Get(string a, string b)
        {
            if (_legs.TryGetValue((a, b), out var leg)) return leg;
            var xyz = _net.Route(a, b);
            int n = xyz.Length / 3;
            leg = new Leg { on = new List<(string, double)>() };
            var cum = new double[n];
            for (int i = 1; i < n; i++)
                cum[i] = cum[i - 1] + Math.Sqrt(Sq(xyz[3 * i] - xyz[3 * i - 3]) + Sq(xyz[3 * i + 1] - xyz[3 * i - 2]));
            leg.length = cum[n - 1];
            leg.startHeading = Math.Atan2(xyz[4] - xyz[1], xyz[3] - xyz[0]);
            leg.endHeading = Math.Atan2(xyz[3 * n - 2] - xyz[3 * n - 5], xyz[3 * n - 3] - xyz[3 * n - 6]);
            int na = _net.Stops[a].node, nb = _net.Stops[b].node;
            foreach (var s in _net.Stops.Values)
            {
                if (s.node == na || s.node == nb) continue;
                for (int i = 1; i < n - 1; i++)
                    if (Sq(xyz[3 * i] - s.nodeX) + Sq(xyz[3 * i + 1] - s.nodeY) < 1e-6)
                    {
                        leg.on.Add((s.name, cum[i]));
                        break;
                    }
            }
            leg.on.Sort((p, q) => p.s.CompareTo(q.s));
            _legs[(a, b)] = leg;
            return leg;
        }

        private static double Sq(double x) => x * x;

        private static double Wrap(double a)
        {
            double t = a + Math.PI, m = 2.0 * Math.PI;
            return t - m * Math.Floor(t / m) - Math.PI;
        }
    }
}
