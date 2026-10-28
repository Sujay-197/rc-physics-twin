// The campus player's camera modes (Docs/specs/2026-10-25-shuttle-fleet-design.md,
// "Camera modes"): following each bus, its route, the fleet, the whole campus,
// a free orbit and a stop cam; which one is on, how C and 1-6 change it, and
// -camera on the command line. FleetCamera places the camera. Free of
// UnityEngine, like Drivers.cs.
using System;
using System.Collections.Generic;
using System.Globalization;

namespace RcTwin
{
    public enum CameraKind { Bus, Route, Fleet, Overview, Orbit, Stop }

    public readonly struct CameraMode : IEquatable<CameraMode>
    {
        public readonly CameraKind Kind;
        /// <summary>The bus followed, 1-based (Bus); 0 otherwise.</summary>
        public readonly int Bus;
        /// <summary>The stop watched (Stop), or null for the busiest.</summary>
        public readonly string Stop;

        public CameraMode(CameraKind kind, int bus = 0, string stop = null)
        {
            Kind = kind;
            Bus = kind == CameraKind.Bus ? Math.Max(1, bus) : 0;
            Stop = kind == CameraKind.Stop ? stop : null;
        }

        public static CameraMode Following(int bus) => new CameraMode(CameraKind.Bus, bus);

        /// <summary>What the screen calls it: "Bus 2", "Stop S3", "Overview".</summary>
        public override string ToString() =>
            Kind == CameraKind.Bus ? $"Bus {Bus}" : Kind == CameraKind.Stop && Stop != null ? $"Stop {Stop}" : Kind.ToString();

        public bool Equals(CameraMode o) => Kind == o.Kind && Bus == o.Bus && Stop == o.Stop;
        public override bool Equals(object o) => o is CameraMode m && Equals(m);
        public override int GetHashCode() => ((int)Kind * 31 + Bus) * 31 + (Stop?.GetHashCode() ?? 0);
    }

    public sealed class CameraModes
    {
        /// <summary>The modes after the buses, in cycle order.</summary>
        public static readonly CameraKind[] Views = { CameraKind.Route, CameraKind.Fleet, CameraKind.Overview, CameraKind.Orbit, CameraKind.Stop };

        public CameraMode Current { get; private set; } = CameraMode.Following(1);
        public int Buses { get; private set; }
        /// <summary>The bus the views that need one look at (Route, the bottom bar): the last one followed.</summary>
        public int FollowedBus { get; private set; } = 1;

        public CameraModes(int buses) => Buses = Math.Max(1, buses);

        /// <summary>Bus 1, ..., Bus N, then the views.</summary>
        public List<CameraMode> Cycle()
        {
            var l = new List<CameraMode>();
            for (int b = 1; b <= Buses; b++) l.Add(CameraMode.Following(b));
            foreach (var k in Views) l.Add(new CameraMode(k));
            return l;
        }

        /// <summary>C (forward) and Shift+C (back). A stop cam keeps its stop only while it is on.</summary>
        public void Step(int dir)
        {
            var l = Cycle();
            int i = l.FindIndex(m => m.Kind == Current.Kind && m.Bus == Current.Bus);
            Set(l[((i < 0 ? 0 : i) + (dir >= 0 ? 1 : -1) + l.Count) % l.Count]);
        }

        public void Next() => Step(1);
        public void Previous() => Step(-1);

        /// <summary>Follow bus k (1-based), as 1-6 do; false if there is no such bus.</summary>
        public bool Follow(int bus)
        {
            if (bus < 1 || bus > Buses) return false;
            Set(CameraMode.Following(bus));
            return true;
        }

        public void Set(CameraMode m)
        {
            if (m.Kind == CameraKind.Bus && m.Bus > Buses) m = CameraMode.Following(Buses);
            Current = m;
            if (m.Kind == CameraKind.Bus) FollowedBus = m.Bus;
        }

        /// <summary>The fleet changed size: a mode on a bus that has gone follows the last one.</summary>
        public void SetBuses(int n)
        {
            Buses = Math.Max(1, n);
            if (FollowedBus > Buses) FollowedBus = Buses;
            if (Current.Kind == CameraKind.Bus && Current.Bus > Buses) Current = CameraMode.Following(Buses);
        }

        /// <summary>The stop cam on the next or previous stop of `stops` (in order).</summary>
        public void StepStop(IReadOnlyList<string> stops, string current, int dir)
        {
            if (stops.Count == 0) return;
            int i = 0;
            for (int k = 0; k < stops.Count; k++) if (stops[k] == current) i = k;
            Set(new CameraMode(CameraKind.Stop, stop: stops[((i + (dir >= 0 ? 1 : -1)) % stops.Count + stops.Count) % stops.Count]));
        }

        /// <summary>-camera's value: bus, bus&lt;k&gt;, route, fleet, overview, orbit, stop or stop:&lt;name&gt; (any case).</summary>
        public static bool TryParse(string s, out CameraMode m)
        {
            m = CameraMode.Following(1);
            if (string.IsNullOrWhiteSpace(s)) return false;
            s = s.Trim();
            string lower = s.ToLowerInvariant();
            if (lower.StartsWith("bus"))
            {
                string n = lower.Substring(3).Trim();
                if (n.Length == 0) return true;
                if (!int.TryParse(n, NumberStyles.None, CultureInfo.InvariantCulture, out int k) || k < 1) return false;
                m = CameraMode.Following(k);
                return true;
            }
            if (lower == "stop" || lower.StartsWith("stop:"))
            {
                string name = lower.Length > 5 ? s.Substring(5).Trim().ToUpperInvariant() : null;
                m = new CameraMode(CameraKind.Stop, stop: string.IsNullOrEmpty(name) ? null : name);
                return true;
            }
            foreach (var k in Views)
                if (lower == k.ToString().ToLowerInvariant()) { m = new CameraMode(k); return true; }
            return false;
        }
    }
}
