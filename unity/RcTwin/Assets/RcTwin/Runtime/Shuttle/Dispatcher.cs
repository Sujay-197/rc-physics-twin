// Who serves a request: each shuttle quotes the cost of fitting it into its
// plan, and the cheapest takes it. With one shuttle it simply goes to that
// one; a fleet adds and removes shuttles as it grows and shrinks
// (Docs/specs/2026-10-25-shuttle-fleet-design.md).
// It also keeps the open requests: retrying those still waiting, and letting
// students who waited too long give up.
using System.Collections.Generic;
using System.Linq;

namespace RcTwin
{
    public sealed class Dispatcher
    {
        public const double RetryEvery = 1.0;   // [s]

        /// <summary>Requests neither delivered nor given up.</summary>
        public IReadOnlyList<RideRequest> Open => _open;

        private readonly List<ShuttleBrain> _shuttles;
        private readonly List<RideRequest> _open = new List<RideRequest>();
        private double _lastRetry = double.NegativeInfinity;

        public IReadOnlyList<ShuttleBrain> Shuttles => _shuttles;

        public Dispatcher(IReadOnlyList<ShuttleBrain> shuttles)
        {
            _shuttles = new List<ShuttleBrain>();
            foreach (var b in shuttles) Add(b);
        }

        public void Add(ShuttleBrain b)
        {
            if (_shuttles.Contains(b)) return;
            _shuttles.Add(b);
            b.WaitingAt = stop => _open.Where(r => r.State == RequestState.Waiting && r.Origin == stop);
        }

        /// <summary>A shuttle leaving the fleet: whatever it still had planned waits again.</summary>
        public void Remove(ShuttleBrain b)
        {
            if (!_shuttles.Remove(b)) return;
            foreach (var r in b.Plan.Stops.SelectMany(st => st.Board).ToList())
            {
                b.Release(r);
                r.State = RequestState.Waiting;
            }
        }

        public void Offer(RideRequest r)
        {
            if (!_open.Contains(r)) _open.Add(r);
            Assign(r);
        }

        public void RetryWaiting(IEnumerable<RideRequest> waiting)
        {
            foreach (var r in waiting.Where(x => x.State == RequestState.Waiting).OrderBy(x => x.RaisedAt).ToList())
                Assign(r);
        }

        /// <summary>Give-ups, and a retry of the waiting requests every RetryEvery seconds.</summary>
        public void Tick(double now, ShuttleScore score)
        {
            if (_shuttles.Count == 0) return;
            double giveUp = _shuttles[0].Settings.GiveUpAfter;
            foreach (var r in _open)
            {
                if ((r.State == RequestState.Waiting || r.State == RequestState.Assigned) && now - r.RaisedAt > giveUp)
                {
                    if (r.State == RequestState.Assigned) foreach (var b in _shuttles) b.Release(r);
                    r.State = RequestState.GaveUp;
                    score.OnGaveUp(r);
                }
            }
            _open.RemoveAll(r => r.State == RequestState.Delivered || r.State == RequestState.GaveUp);
            if (now - _lastRetry >= RetryEvery)
            {
                _lastRetry = now;
                RetryWaiting(_open);
            }
        }

        private void Assign(RideRequest r)
        {
            ShuttleBrain best = null;
            double bestCost = double.PositiveInfinity;
            foreach (var b in _shuttles)
            {
                var q = b.QuoteInsertion(r);
                if (q.HasValue && q.Value < bestCost) { bestCost = q.Value; best = b; }
            }
            best?.Accept(r);
        }
    }
}
