// The shuttle's battery rules (Docs/specs/2026-10-21-campus-shuttle-design.md,
// "Battery"): a game battery small enough to matter, charging at the nearest
// charger, and what to do when the charge left no longer covers the plan.
using System;
using System.Collections.Generic;

namespace RcTwin
{
    public enum LowBattery { Fine, DropOffFirst, ChargeNow }

    public static class BatteryPolicy
    {
        /// <summary>The twin's energy per metre at 2 m/s with training cars, on the
        /// stop task (Docs/policy-study.md, fresh-trip table) [Wh/m].</summary>
        public const double SeedWhPerMetre = 0.00029;

        /// <summary>The pack capacity that lasts BatteryMinutesOfDriving at the cruise speed [Ah].</summary>
        public static double GamePackCapacityAh(ShuttleSettings s, double whPerMetre, double packNominalV) =>
            s.BatteryMinutesOfDriving * 60.0 * s.Cruise * whPerMetre / packNominalV;

        /// <summary>Charge added per second parked at the charger (10 to 95 % in ChargeSeconds10To95).</summary>
        public static double ChargeRatePerSecond(ShuttleSettings s) => 0.85 / s.ChargeSeconds10To95;

        /// <summary>The charger nearest `from` by route (ties: the first listed).</summary>
        public static string NearestCharger(ShuttleSettings s, ITravelModel t, string from)
        {
            string best = s.Chargers[0];
            double bd = double.PositiveInfinity;
            foreach (var c in s.Chargers)
            {
                double d = from == null ? 0.0 : t.Distance(from, c);
                if (d < bd) { bd = d; best = c; }
            }
            return best;
        }

        /// <summary>The charger to go to from `from` with several buses: the nearest one
        /// not taken (charged at or headed for by another bus); the nearest if all are.</summary>
        public static string PickCharger(ShuttleSettings s, ITravelModel t, string from, ICollection<string> taken)
        {
            string nearest = null, free = null;
            double dn = double.PositiveInfinity, df = double.PositiveInfinity;
            foreach (var c in s.Chargers)
            {
                double d = from == null ? 0.0 : t.Distance(from, c);
                if (d < dn) { dn = d; nearest = c; }
                if (d < df && !taken.Contains(c)) { df = d; free = c; }
            }
            return free ?? nearest;
        }

        /// <summary>Fine while the charge covers the plan (seats are not its
        /// business: a plan over capacity is no low battery). Otherwise the riders aboard
        /// are dropped off first if the charge allows (with half the reserve), or
        /// the shuttle charges first; pickups not yet aboard go back to waiting.
        /// pick chooses the charger to go to from a stop (by default the nearest).</summary>
        public static LowBattery Decide(ShuttlePlan plan, PlanContext c, ShuttleSettings s, ITravelModel t, out ShuttlePlan next,
                                        Func<string, string> pick = null)
        {
            pick = pick ?? (from => NearestCharger(s, t, from));
            if (plan.SocNeeded(c, s, t) <= c.Soc - s.Reserve) { next = plan; return LowBattery.Fine; }
            var boardsLater = new System.Collections.Generic.HashSet<RideRequest>();
            foreach (var st in plan.Stops)
                foreach (var r in st.Board) boardsLater.Add(r);
            var drops = new ShuttlePlan();
            foreach (var st in plan.Stops)
            {
                var keep = st.Alight.FindAll(r => !boardsLater.Contains(r));
                if (keep.Count > 0) drops.Stops.Add(new PlanStop { Stop = st.Stop, Alight = keep });
            }
            foreach (var r in boardsLater) r.State = RequestState.Waiting;

            var dropFirst = drops.Copy();
            string last = dropFirst.Stops.Count > 0 ? dropFirst.Stops[dropFirst.Stops.Count - 1].Stop : c.From;
            string after = pick(last);
            if (dropFirst.Stops.Count == 0 || last != after)
                dropFirst.Stops.Add(new PlanStop { Stop = after });
            dropFirst.Stops[dropFirst.Stops.Count - 1].Charge = true;
            if (dropFirst.SocNeeded(c, s, t) <= c.Soc - 0.5 * s.Reserve)
            {
                next = dropFirst;
                return LowBattery.DropOffFirst;
            }
            next = new ShuttlePlan();
            string now = pick(c.From);
            next.Stops.Add(new PlanStop { Stop = now, Charge = true });
            foreach (var st in drops.Stops)
            {
                if (st.Stop == now) next.Stops[0].Alight.AddRange(st.Alight);
                else next.Stops.Add(st);
            }
            return LowBattery.ChargeNow;
        }
    }

    /// <summary>The shuttle's own charge use per metre, an exponential average
    /// of what it measures as it drives.</summary>
    public sealed class SocMeter
    {
        /// <summary>Per leg. Fast enough to learn within a few legs what turning round and lining up
        /// cost a bus (reversing at walking pace draws far more current than cruising).</summary>
        public const double Weight = 0.2;
        public double SocPerMetre { get; private set; }

        public SocMeter(double seedSocPerMetre) => SocPerMetre = seedSocPerMetre;

        /// <summary>The pack was re-sized: the same energy is k times the charge.</summary>
        public void Scale(double k) => SocPerMetre *= k;

        public void Observe(double metres, double socUsed)
        {
            if (metres <= 1e-6) return;
            SocPerMetre = (1.0 - Weight) * SocPerMetre + Weight * Math.Max(0.0, socUsed / metres);
        }
    }
}
