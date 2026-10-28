// The campus shuttle's settings: every default the design sets
// (Docs/specs/2026-10-21-campus-shuttle-design.md), in one place the HUD's
// settings panel edits. Free of UnityEngine, like Drivers.cs.
namespace RcTwin
{
    public sealed class ShuttleSettings
    {
        // Requests
        public double RequestInterval = 20.0;      // one new request this often across the campus [s]
        public double GiveUpAfter = 180.0;         // a student waits this long, then leaves [s]

        // The fleet (Docs/specs/2026-10-25-shuttle-fleet-design.md)
        public int Buses = 3;
        public const int MaxBuses = 6;

        // The shuttle
        public int Capacity = 6;
        public double Cruise = 2.0;                // [m/s]
        public double DwellBase = 3.0;             // at every stop [s]
        public double DwellPerStudent = 1.5;       // per student boarding or alighting [s]

        // Battery
        public double BatteryMinutesOfDriving = 5.0;
        public double ChargeSeconds10To95 = 30.0;
        public double LeaveChargerAt = 0.95;
        public double Reserve = 0.10;              // of a full charge, never planned into
        public double EnergySafety = 1.15;
        public double IdleTopUpBelow = 0.60;
        /// <summary>Stops with a charger, one bus at a time each: AB1, and AB3, the dead end on the far side.</summary>
        public string[] Chargers = { "AB1", "AB3" };

        // Score
        public int DeliverBase = 100, DeliverMin = 20, DirectBonus = 20;
        public double DirectFactor = 1.5;
        public int GaveUpPenalty = 50, FlatPenalty = 500, KerbPenalty = 5;
    }
}
