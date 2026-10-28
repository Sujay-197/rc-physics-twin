// The shuttle's score (Docs/specs/2026-10-21-campus-shuttle-design.md, "Score"):
// quick pickups and direct rides pay; students giving up, a flat battery and
// kerb strikes cost. In a fleet each bus keeps its own, and every event in it
// also counts in the fleet's (its parent).
using System;

namespace RcTwin
{
    public sealed class ShuttleScore
    {
        public int Points { get; private set; }
        public int Delivered { get; private set; }
        public int GaveUp { get; private set; }
        public int Flats { get; private set; }
        public int KerbStrikes { get; private set; }
        /// <summary>Mean wait at the stop of the students delivered [s].</summary>
        public double AverageWait => Delivered == 0 ? 0.0 : _waits / Delivered;

        private readonly ShuttleSettings _s;
        private readonly ShuttleScore _parent;
        private double _waits;

        public ShuttleScore(ShuttleSettings s, ShuttleScore parent = null)
        {
            _s = s;
            _parent = parent;
        }

        /// <summary>A student dropped off; directDriveTime is the drive from their origin to their destination [s].</summary>
        public void OnDelivered(RideRequest r, double directDriveTime)
        {
            double wait = r.PickedUpAt - r.RaisedAt;
            int pts = Math.Max(_s.DeliverMin, (int)Math.Round(_s.DeliverBase - wait));
            if (r.DroppedAt - r.PickedUpAt <= _s.DirectFactor * directDriveTime) pts += _s.DirectBonus;
            Points += pts;
            Delivered++;
            _waits += wait;
            _parent?.OnDelivered(r, directDriveTime);
        }

        public void OnGaveUp(RideRequest r) { Points -= _s.GaveUpPenalty; GaveUp++; _parent?.OnGaveUp(r); }
        public void OnFlat() { Points -= _s.FlatPenalty; Flats++; _parent?.OnFlat(); }
        public void OnKerbStrike() { Points -= _s.KerbPenalty; KerbStrikes++; _parent?.OnKerbStrike(); }
    }
}
