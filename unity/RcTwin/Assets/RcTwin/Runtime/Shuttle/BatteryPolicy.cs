// The shuttle's battery rules (Docs/specs/2026-10-21-campus-shuttle-design.md,
// "Battery"): a game battery small enough to matter, charging at AB1, and
// what to do when the charge left no longer covers the plan.
using System;

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

        /// <summary>Fine while the charge covers the plan (seats are not its
        /// business: a plan over capacity is no low battery). Otherwise the riders aboard
        /// are dropped off first if the charge allows (with half the reserve), or
        /// the shuttle charges first; pickups not yet aboard go back to waiting.</summary>
        public static LowBattery Decide(ShuttlePlan plan, PlanContext c, ShuttleSettings s, ITravelModel t, out ShuttlePlan next)
        {
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
            if (dropFirst.Stops.Count == 0 || dropFirst.Stops[dropFirst.Stops.Count - 1].Stop != s.ChargerStop)
                dropFirst.Stops.Add(new PlanStop { Stop = s.ChargerStop });
            dropFirst.Stops[dropFirst.Stops.Count - 1].Charge = true;
            if (dropFirst.SocNeeded(c, s, t) <= c.Soc - 0.5 * s.Reserve)
            {
                next = dropFirst;
                return LowBattery.DropOffFirst;
            }
            next = new ShuttlePlan();
            next.Stops.Add(new PlanStop { Stop = s.ChargerStop, Charge = true });
            foreach (var st in drops.Stops)
            {
                if (st.Stop == s.ChargerStop) next.Stops[0].Alight.AddRange(st.Alight);
                else next.Stops.Add(st);
            }
            return LowBattery.ChargeNow;
        }
    }

    /// <summary>The shuttle's own charge use per metre, an exponential average
    /// of what it measures as it drives.</summary>
    public sealed class SocMeter
    {
        public const double Weight = 0.05;
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
