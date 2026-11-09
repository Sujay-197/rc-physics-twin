// What a fleet run measured, for the fleet-level evaluation (tools/fleet_experiment.py,
// Docs/fleet-results.md). The fleet (ShuttleFleet) keeps one only when the command line
// has -metrics <file.json>, and writes it when the run ends (-duration) or the player quits.
// Each bus's time in each mode, its seat-seconds, distance and energy are summed over
// the measuring window (warm-up, end]; the fleet's counters are kept at the window's
// start, so the report has them for the window and in total; every request and every
// leg is listed whole, and the statistics are left to the script. Free of UnityEngine.
using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text;

namespace RcTwin
{
    /// <summary>One stop-to-stop leg as a bus drove it.</summary>
    public sealed class LegRecord
    {
        public int Bus;
        public string From, To;
        /// <summary>The leg was cut short at a stop on its way (StopEarlyAt): To is that stop.</summary>
        public bool Early;
        public double Start, End = double.NaN;
        /// <summary>How the autopilot ended it (TwinAutopilot.Status), or null while it is driven.</summary>
        public string Outcome;
        /// <summary>The autopilot's own times for the trip [s]: held by the traffic rules, turning round or lining up.</summary>
        public double Held, Turn;
    }

    public sealed class FleetMetrics
    {
        public sealed class BusTally
        {
            public int Number;
            public double Time, Driving, Dwelling, Idle, Charging, Towed;
            /// <summary>Riders aboard, integrated over time [rider s]; and over the time driving.</summary>
            public double SeatSeconds, SeatSecondsDriving;
            public double Metres, EnergyWh;
        }

        public readonly double Warmup, End;
        public readonly Dictionary<string, object> Config = new Dictionary<string, object>();
        public IReadOnlyList<BusTally> Buses => _buses;
        /// <summary>The counters at the window's start (null before it).</summary>
        public Dictionary<string, double> AtStart { get; private set; }

        private readonly List<BusTally> _buses = new List<BusTally>();

        public FleetMetrics(double warmup, double end)
        {
            Warmup = warmup;
            End = end;
        }

        public bool InWindow(double now) => now > Warmup && now <= End;

        /// <summary>One bus over the tick ending at `now`, `dt` long: what it did, how many rode,
        /// how far it went and the energy it drew. Counted only inside the window.</summary>
        public void Observe(double now, double dt, int bus, ShuttleMode mode, int aboard, double metres, double wh)
        {
            if (!InWindow(now)) return;
            var t = _buses.FirstOrDefault(b => b.Number == bus);
            if (t == null)
            {
                t = new BusTally { Number = bus };
                _buses.Add(t);
                _buses.Sort((p, q) => p.Number.CompareTo(q.Number));
            }
            t.Time += dt;
            switch (mode)
            {
                case ShuttleMode.Driving: t.Driving += dt; t.SeatSecondsDriving += aboard * dt; break;
                case ShuttleMode.Dwelling: t.Dwelling += dt; break;
                case ShuttleMode.Idle: t.Idle += dt; break;
                case ShuttleMode.Charging: t.Charging += dt; break;
                case ShuttleMode.Towed: t.Towed += dt; break;
            }
            t.SeatSeconds += aboard * dt;
            t.Metres += metres;
            t.EnergyWh += wh;
        }

        /// <summary>The fleet's counters as they stand at `now`: kept once, at the window's start.</summary>
        public void Mark(double now, Dictionary<string, double> counters)
        {
            if (AtStart == null && now > Warmup) AtStart = new Dictionary<string, double>(counters);
        }

        /// <summary>The report: the config, the window, each bus's tally, the counters in the
        /// window and in total, every request and every leg.</summary>
        public Dictionary<string, object> Report(double now, Dictionary<string, double> counters,
                                                 IEnumerable<RideRequest> requests, Func<RideRequest, (double m, double s)> direct,
                                                 IEnumerable<LegRecord> legs, double wallSeconds)
        {
            var window = new Dictionary<string, object>();
            foreach (var kv in counters)
                window[kv.Key] = AtStart == null ? double.NaN : kv.Value - (AtStart.TryGetValue(kv.Key, out var v) ? v : 0.0);
            return new Dictionary<string, object>
            {
                ["schema"] = "rc-twin-fleet-run/1",
                ["config"] = Config,
                ["window"] = new Dictionary<string, object> { ["start_s"] = Warmup, ["end_s"] = Math.Min(End, now) },
                ["sim_s"] = now,
                ["wall_s"] = wallSeconds,
                ["buses"] = _buses.Select(b => (object)new Dictionary<string, object>
                {
                    ["bus"] = b.Number, ["time_s"] = b.Time, ["driving_s"] = b.Driving, ["dwelling_s"] = b.Dwelling,
                    ["idle_s"] = b.Idle, ["charging_s"] = b.Charging, ["towed_s"] = b.Towed,
                    ["seat_s"] = b.SeatSeconds, ["seat_s_driving"] = b.SeatSecondsDriving,
                    ["metres"] = b.Metres, ["energy_wh"] = b.EnergyWh,
                }).ToList(),
                ["counters_window"] = window,
                ["counters_total"] = counters.ToDictionary(kv => kv.Key, kv => (object)kv.Value),
                ["requests"] = requests.Select(r =>
                {
                    var (m, s) = direct(r);
                    return (object)new Dictionary<string, object>
                    {
                        ["id"] = r.Id, ["from"] = r.Origin, ["to"] = r.Destination, ["user"] = r.UserRaised,
                        ["raised"] = r.RaisedAt, ["picked"] = r.PickedUpAt, ["dropped"] = r.DroppedAt,
                        ["state"] = r.State.ToString(), ["direct_m"] = m, ["direct_s"] = s,
                    };
                }).ToList(),
                ["legs"] = legs.Select(l => (object)new Dictionary<string, object>
                {
                    ["bus"] = l.Bus, ["from"] = l.From, ["to"] = l.To, ["early"] = l.Early,
                    ["start"] = l.Start, ["end"] = l.End, ["outcome"] = l.Outcome, ["held"] = l.Held, ["turn"] = l.Turn,
                }).ToList(),
            };
        }

        // ------------------------------------------------------------ JSON

        /// <summary>JSON for dictionaries, lists, strings, numbers, booleans and null; a NaN or
        /// infinite number is null. Numbers round-trip (R).</summary>
        public static string ToJson(object o)
        {
            var sb = new StringBuilder();
            Write(sb, o);
            return sb.ToString();
        }

        private static void Write(StringBuilder sb, object o)
        {
            switch (o)
            {
                case null: sb.Append("null"); return;
                case string s: Str(sb, s); return;
                case bool b: sb.Append(b ? "true" : "false"); return;
                case int i: sb.Append(i.ToString(CultureInfo.InvariantCulture)); return;
                case long l: sb.Append(l.ToString(CultureInfo.InvariantCulture)); return;
                case float f: Num(sb, f); return;
                case double d: Num(sb, d); return;
                case IDictionary dict:
                {
                    sb.Append('{');
                    bool first = true;
                    foreach (DictionaryEntry kv in dict)
                    {
                        if (!first) sb.Append(", ");
                        first = false;
                        Str(sb, Convert.ToString(kv.Key, CultureInfo.InvariantCulture));
                        sb.Append(": ");
                        Write(sb, kv.Value);
                    }
                    sb.Append('}');
                    return;
                }
                case IEnumerable list:
                {
                    sb.Append('[');
                    bool first = true;
                    foreach (var x in list)
                    {
                        if (!first) sb.Append(", ");
                        first = false;
                        Write(sb, x);
                    }
                    sb.Append(']');
                    return;
                }
                default: Str(sb, Convert.ToString(o, CultureInfo.InvariantCulture)); return;
            }
        }

        private static void Num(StringBuilder sb, double d)
        {
            if (double.IsNaN(d) || double.IsInfinity(d)) sb.Append("null");
            else sb.Append(d.ToString("R", CultureInfo.InvariantCulture));
        }

        private static void Str(StringBuilder sb, string s)
        {
            sb.Append('"');
            foreach (char c in s)
            {
                switch (c)
                {
                    case '"': sb.Append("\\\""); break;
                    case '\\': sb.Append("\\\\"); break;
                    case '\n': sb.Append("\\n"); break;
                    case '\r': sb.Append("\\r"); break;
                    case '\t': sb.Append("\\t"); break;
                    default:
                        if (c < 0x20) sb.Append("\\u").Append(((int)c).ToString("x4"));
                        else sb.Append(c);
                        break;
                }
            }
            sb.Append('"');
        }
    }
}
