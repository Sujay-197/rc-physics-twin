// Students appear at stops as a Poisson process, each stop at a rate in
// proportion to its demand weight, wanting to go to another stop drawn by
// weight (never one on the same road node), as sim_py/campus.py's
// CampusNetwork.sample_trip draws trips. The person watching can raise
// requests too.
using System;
using System.Collections.Generic;

namespace RcTwin
{
    public sealed class RequestSpawner
    {
        /// <summary>New requests per second, across the whole campus.</summary>
        public double RatePerSecond = 1.0 / 20.0;

        public IReadOnlyList<RideRequest> All => _all;

        private readonly List<(string name, int node, int weight)> _stops;
        private readonly Dictionary<string, int> _node = new Dictionary<string, int>();
        private readonly Random _rng;
        private readonly List<RideRequest> _all = new List<RideRequest>();
        private int _nextId = 1;

        public RequestSpawner(IReadOnlyList<(string name, int node, int weight)> stops, int seed)
        {
            _stops = new List<(string, int, int)>(stops);
            foreach (var s in _stops) _node[s.name] = s.node;
            _rng = new Random(seed);
        }

        /// <summary>The requests raised in (now, now + dt]: one Bernoulli draw per tick
        /// (dt of 0.1 s or less keeps two in one tick rare enough not to matter).</summary>
        public List<RideRequest> Tick(double now, double dt)
        {
            var made = new List<RideRequest>();
            if (_rng.NextDouble() < RatePerSecond * dt)
            {
                var origin = Draw(null);
                made.Add(Add(origin, Draw(_node[origin]), now + dt, user: false));
            }
            return made;
        }

        /// <summary>A request the person watching raised.</summary>
        public RideRequest Raise(string origin, string destination, double now)
        {
            if (origin == null || !_node.ContainsKey(origin)) throw new ArgumentException($"no stop {origin}");
            if (destination == null || !_node.ContainsKey(destination)) throw new ArgumentException($"no stop {destination}");
            if (_node[origin] == _node[destination]) throw new ArgumentException($"{origin} and {destination} are the same place");
            return Add(origin, destination, now, user: true);
        }

        private RideRequest Add(string origin, string destination, double at, bool user)
        {
            var r = new RideRequest { Id = _nextId++, Origin = origin, Destination = destination, RaisedAt = at, UserRaised = user };
            _all.Add(r);
            return r;
        }

        // A stop by demand weight; with excludeNode, never one on that node.
        private string Draw(int? excludeNode)
        {
            int total = 0;
            foreach (var s in _stops) if (s.node != excludeNode) total += s.weight;
            int pick = _rng.Next(total);
            foreach (var s in _stops)
            {
                if (s.node == excludeNode) continue;
                if (pick < s.weight) return s.name;
                pick -= s.weight;
            }
            throw new InvalidOperationException("no stop to draw");
        }
    }
}
