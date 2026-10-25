// The shuttle's plan: the stops it will visit, in order, and who boards and
// alights at each. New requests go in by cheapest insertion, never past the
// seats or the battery (Docs/specs/2026-10-21-campus-shuttle-design.md).
using System.Collections.Generic;

namespace RcTwin
{
    public sealed class PlanStop
    {
        public string Stop;
        public List<RideRequest> Board = new List<RideRequest>();
        public List<RideRequest> Alight = new List<RideRequest>();
        /// <summary>A visit to the charger to charge, not only to pick up or drop off.</summary>
        public bool Charge;
        /// <summary>A visit only to wait there: an idle bus making room for another (fleet).</summary>
        public bool Park;

        public PlanStop Copy() =>
            new PlanStop { Stop = Stop, Board = new List<RideRequest>(Board), Alight = new List<RideRequest>(Alight), Charge = Charge, Park = Park };
    }

    /// <summary>Where a plan starts from.</summary>
    public struct PlanContext
    {
        /// <summary>The stop the shuttle is at, or the committed next stop while driving.</summary>
        public string From;
        /// <summary>The stop before From, for turn-round allowances (null: none).</summary>
        public string ArrivedFrom;
        /// <summary>Game time at From: now, or the estimated arrival there while driving [s].</summary>
        public double Now;
        /// <summary>Students aboard leaving From.</summary>
        public int Onboard;
        /// <summary>State of charge, 0 to 1, and its use per metre driven.</summary>
        public double Soc, SocPerMetre;
    }

    public sealed class ShuttlePlan
    {
        public List<PlanStop> Stops = new List<PlanStop>();

        public ShuttlePlan Copy()
        {
            var p = new ShuttlePlan();
            foreach (var st in Stops) p.Stops.Add(st.Copy());
            return p;
        }

        /// <summary>Time to finish the plan, plus every planned student's wait
        /// and ride (only the rest of the ride for those already aboard) [s].</summary>
        public double Cost(PlanContext c, ShuttleSettings s, ITravelModel t)
        {
            var arrive = Timeline(c, s, t, out double finish);
            var boarded = new Dictionary<RideRequest, double>();
            double cost = finish - c.Now;
            for (int i = 0; i < Stops.Count; i++)
            {
                foreach (var r in Stops[i].Board)
                {
                    boarded[r] = arrive[i];
                    cost += arrive[i] - r.RaisedAt;
                }
                foreach (var r in Stops[i].Alight)
                    cost += arrive[i] - (boarded.TryGetValue(r, out double tb) ? tb : c.Now);
            }
            return cost;
        }

        /// <summary>Charge the plan needs, with the way from its last stop to the nearest charger.</summary>
        public double SocNeeded(PlanContext c, ShuttleSettings s, ITravelModel t)
        {
            double metres = 0.0;
            string at = c.From;
            foreach (var st in Stops)
            {
                metres += t.Distance(at, st.Stop);
                at = st.Stop;
            }
            metres += t.Distance(at, BatteryPolicy.NearestCharger(s, t, at));
            return metres * c.SocPerMetre * s.EnergySafety;
        }

        public bool Feasible(PlanContext c, ShuttleSettings s, ITravelModel t)
        {
            int load = c.Onboard;
            if (load > s.Capacity) return false;
            foreach (var st in Stops)
            {
                load += st.Board.Count - st.Alight.Count;
                if (load > s.Capacity) return false;
            }
            return SocNeeded(c, s, t) <= c.Soc - s.Reserve;
        }

        /// <summary>The cheapest feasible plan with r's pickup and drop-off added,
        /// as new stops or merged into a stop already planned there; null if none.</summary>
        public ShuttlePlan TryInsert(RideRequest r, PlanContext c, ShuttleSettings s, ITravelModel t, out double addedCost)
        {
            double before = Cost(c, s, t);
            ShuttlePlan best = null;
            double bestCost = double.PositiveInfinity;
            int n = Stops.Count;
            // Pickup: a new stop before index p (p = 0..n), or merged into stop p (p = n+1.. as n+1+k).
            for (int pk = 0; pk <= 2 * n; pk++)
            {
                bool mergeP = pk > n;
                int p = mergeP ? pk - n - 1 : pk;
                if (mergeP && Stops[p].Stop != r.Origin) continue;
                var withPick = Copy();
                int ip;
                if (mergeP) { withPick.Stops[p].Board.Add(r); ip = p; }
                else { withPick.Stops.Insert(p, new PlanStop { Stop = r.Origin, Board = { r } }); ip = p; }
                int m = withPick.Stops.Count;
                for (int dk = ip + 1; dk <= 2 * m; dk++)
                {
                    bool mergeD = dk > m;
                    int q = mergeD ? dk - m - 1 : dk;
                    if (mergeD && (q <= ip || withPick.Stops[q].Stop != r.Destination)) continue;
                    if (!mergeD && q <= ip) continue;
                    var cand = withPick.Copy();
                    if (mergeD) cand.Stops[q].Alight.Add(r);
                    else cand.Stops.Insert(q, new PlanStop { Stop = r.Destination, Alight = { r } });
                    if (!cand.Feasible(c, s, t)) continue;
                    double cost = cand.Cost(c, s, t);
                    if (cost < bestCost) { bestCost = cost; best = cand; }
                }
            }
            addedCost = best == null ? double.PositiveInfinity : bestCost - before;
            return best;
        }

        // Arrival time at each stop, and when the plan finishes (after the last dwell).
        private double[] Timeline(PlanContext c, ShuttleSettings s, ITravelModel t, out double finish)
        {
            var arrive = new double[Stops.Count];
            double time = c.Now;
            string prev = c.ArrivedFrom, at = c.From;
            for (int i = 0; i < Stops.Count; i++)
            {
                var st = Stops[i];
                if (st.Stop != at)
                {
                    time += t.DriveTime(at, st.Stop) + t.TurnAllowance(prev, at, st.Stop);
                    prev = at;
                    at = st.Stop;
                }
                arrive[i] = time;
                time += s.DwellBase + s.DwellPerStudent * (st.Board.Count + st.Alight.Count);
            }
            finish = time;
            return arrive;
        }
    }
}
