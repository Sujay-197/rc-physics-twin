// Drivers: what turns an observation into a command. The autopilot
// (TwinAutopilot) asks one each control tick; a new controller or a learned
// policy is a new IDriver, nothing else changes.
//
// PurePursuitDriver is sim_py/controllers.py's PurePursuit, line for line,
// stop plan included (tests/Editor/AutopilotParityTests.cs checks it
// against Python in both RL tasks). PolicyDriver runs a trained network.
// KerbRecovery wraps either and backs the car off a kerb it is pinned against.
// CornerSpeed wraps pure pursuit and slows for the route's corners before
// they come into view (SpeedProfile).
//
// Free of UnityEngine, like RcTwinNative.
using System;

namespace RcTwin
{
    /// <summary>What a driver sees each control tick.</summary>
    public struct DriveContext
    {
        /// <summary>The observation vector: TwinObservation's current frame,
        /// then its history (sim_py/rc_twin_env.py's layout).</summary>
        public double[] Observation;
        /// <summary>Distance left to the end of the route [m], for display;
        /// drivers read it from the observation, as the RL policies do.</summary>
        public double Remaining;
        /// <summary>The car's arc length along the route [m]: the autopilot
        /// knows its route. CornerSpeed reads it; the RL policies do not.</summary>
        public double S;
    }

    public interface IDriver
    {
        string Name { get; }
        /// <summary>History frames the driver's observation carries.</summary>
        int History { get; }
        /// <summary>Start of a trip: forget any memory.</summary>
        void Reset();
        /// <summary>Steer (+ left) and throttle (- brake), each in [-1, 1].</summary>
        (double steer, double throttle) Act(in DriveContext c);
    }

    public static class DriverMath
    {
        public static double Clip(double x, double lo = -1.0, double hi = 1.0) => x < lo ? lo : x > hi ? hi : x;

        /// <summary>Signed curvature of the circle through three points (left
        /// positive); 0 if they are collinear (controllers.path_curvature).</summary>
        public static double PathCurvature(double x0, double y0, double x1, double y1, double x2, double y2)
        {
            double a = Hypot(x1 - x0, y1 - y0), b = Hypot(x2 - x1, y2 - y1), c = Hypot(x2 - x0, y2 - y0);
            double cross = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0);
            if (a * b * c < 1e-12) return 0.0;
            return 2.0 * cross / (a * b * c);
        }

        // Python's math.dist: sqrt of the sum of squares.
        private static double Hypot(double x, double y) => Math.Sqrt(x * x + y * y);
    }

    /// <summary>Pure pursuit toward the centreline at a lookahead distance,
    /// with the shared speed loop: hold the target speed, slower where the
    /// road ahead curves more than the cornering limit allows.</summary>
    public sealed class PurePursuitDriver : IDriver
    {
        public double Lookahead = 1.0;     // [m]
        public double KSpeed = 3.0;        // throttle per unit speed-ratio error
        public double ALat = double.NaN;   // cornering limit [m/s^2]; NaN: none
        public double Wheelbase;           // nominal car [m]
        public double SteerMax;            // nominal car [rad]
        public double TargetSpeed;         // the speed the observation is scaled by [m/s]
        /// <summary>The stop plan ("stop" task): braking [m/s^2] and how far
        /// short of the end to aim [m], as sim_py/controllers.py plans it.</summary>
        public double StopDecel = 1.2, StopMargin = 0.1;
        /// <summary>Where the observation carries remaining_ratio ("stop"
        /// task), or -1: the driver then drives on, as in the "follow" task.</summary>
        public int RemainingIndex = -1;
        public double RemainingScale = TwinObservation.RemainingScaleM;

        public string Name => $"pure pursuit (lookahead {Lookahead:0.##} m)";
        public int History => 0;
        public void Reset() { }

        /// <summary>The tuned pure pursuit (policies/controllers.json) for the nominal car.</summary>
        public static PurePursuitDriver Tuned(in RcTwinNative.Params nominal, double targetSpeed,
                                              double lookahead = 1.5, double kSpeed = 4.0, double aLat = 5.0,
                                              bool stopTask = false) =>
            new PurePursuitDriver
            {
                Lookahead = lookahead, KSpeed = kSpeed, ALat = aLat, TargetSpeed = targetSpeed,
                Wheelbase = nominal.lf + nominal.lr, SteerMax = nominal.steerMax,
                RemainingIndex = stopTask ? TwinObservation.RemainingIndex : -1,
            };

        /// <summary>Pure pursuit tuned for the Unity twin's car on the "stop" task
        /// (policies/controllers_unity_stop.json): what the autopilot drives with.
        /// Tuned's defaults are the study's bare-car gains, which the residual
        /// policies are built on.</summary>
        public static PurePursuitDriver TunedForTwin(in RcTwinNative.Params nominal, double targetSpeed) =>
            Tuned(nominal, targetSpeed, lookahead: 1.5, kSpeed: 2.0, aLat: 12.0, stopTask: true);

        public (double steer, double throttle) Act(in DriveContext c)
        {
            var o = c.Observation;
            double vRefRatio = o[22], vRatio = o[0];
            double p0x = o[16], p0y = o[17], p1x = o[18], p1y = o[19], p2x = o[20], p2y = o[21];

            double ax, ay, d = Lookahead;
            if (d <= 0.5) { ax = p0x; ay = p0y; }
            else if (d <= 1.0)
            {
                double t = (d - 0.5) / 0.5;
                ax = p0x + t * (p1x - p0x); ay = p0y + t * (p1y - p0y);
            }
            else
            {
                double t = Math.Min(1.0, (d - 1.0) / 1.0);
                ax = p1x + t * (p2x - p1x); ay = p1y + t * (p2y - p1y);
            }
            double ld2 = ax * ax + ay * ay;
            double delta = Math.Atan2(2.0 * Wheelbase * ay, ld2);

            // The distance left, when the observation says it is near (ratio below 1).
            double remaining = RemainingIndex >= 0 && o[RemainingIndex] < 1.0 ? o[RemainingIndex] * RemainingScale : double.NaN;
            double goal = vRefRatio;
            if (!double.IsNaN(remaining))
            {
                // Stop at the end: no faster than braking at StopDecel allows.
                double rem = Math.Max(0.0, remaining - StopMargin);
                goal = Math.Min(goal, Math.Sqrt(2.0 * StopDecel * rem) / TargetSpeed * vRefRatio);
            }
            if (!double.IsNaN(ALat) && ALat > 0.0)
            {
                double k = Math.Abs(DriverMath.PathCurvature(p0x, p0y, p1x, p1y, p2x, p2y));
                if (k > 1e-6) goal = Math.Min(goal, Math.Sqrt(ALat / k) / TargetSpeed * vRefRatio);
            }
            double throttle = DriverMath.Clip(KSpeed * (goal - vRatio));
            if (!double.IsNaN(remaining) && throttle < 0.0 && vRatio <= 0.01) throttle = 0.0;   // braking at rest would reverse
            return (DriverMath.Clip(delta / SteerMax), throttle);
        }
    }

    /// <summary>A speed cap along a route: sim_py/controllers.py's
    /// speed_profile, line for line.</summary>
    /// <remarks>
    /// Every Step along the route, curvature is the heading change across
    /// Window centred there (clamped to the route) over the window, the cap
    /// is sqrt(aLat / curvature), at most vMax, and a backward pass lowers
    /// it so that braking at aBrake reaches every corner's cap in time. With
    /// aAccel (not NaN), a forward pass then lets the cap rise after each
    /// corner only as fast as accelerating at aAccel allows: the car is still
    /// turning as it leaves a junction, and full throttle there spun low-grip cars.
    /// </remarks>
    public static class SpeedProfile
    {
        public static double[] Build(Func<double, double> headingAt, double length, double vMax, double aLat,
                                     double aBrake, double window = 1.5, double step = 0.1, double aAccel = double.NaN)
        {
            int n = (int)(length / step + 1e-9) + 1;
            double half = 0.5 * window;
            var cap = new double[n];
            for (int i = 0; i < n; i++)
            {
                double s = i * step;
                double lo = Math.Max(0.0, s - half), hi = Math.Min(length, s + half);
                double k = Math.Abs(Wrap(headingAt(hi) - headingAt(lo))) / Math.Max(hi - lo, 1e-9);
                cap[i] = k < 1e-9 ? vMax : Math.Min(vMax, Math.Sqrt(aLat / k));
            }
            for (int i = n - 2; i >= 0; i--)
                cap[i] = Math.Min(cap[i], Math.Sqrt(cap[i + 1] * cap[i + 1] + 2.0 * aBrake * step));
            if (!double.IsNaN(aAccel))
                for (int i = 1; i < n; i++)
                    cap[i] = Math.Min(cap[i], Math.Sqrt(cap[i - 1] * cap[i - 1] + 2.0 * aAccel * step));
            return cap;
        }

        // Python's (a + pi) % (2 pi) - pi: a floored modulo.
        private static double Wrap(double a)
        {
            double t = a + Math.PI, m = 2.0 * Math.PI;
            return t - m * Math.Floor(t / m) - Math.PI;
        }
    }

    /// <summary>Slows for the route's corners before they come into view:
    /// sim_py/controllers.py's CornerSpeed, line for line.</summary>
    /// <remarks>
    /// The observation sees the centreline 2 m ahead at most; braking from
    /// 4 m/s for a junction takes 4 to 5 m. The autopilot knows its route,
    /// so it plans speed along all of it (Plan, at each trip's start), and
    /// each tick this caps the driver's throttle by a speed loop on the plan
    /// at the car's arc length, where the plan is below the target speed:
    /// before each junction, and as the car pulls away from it. It
    /// only ever lowers the throttle. Most of the Unity autopilot's held-out
    /// failures were low-grip cars spinning at a junction they reached at
    /// full speed (Docs/policy-study.md, "A corner speed plan").
    /// </remarks>
    public sealed class CornerSpeed : IDriver
    {
        /// <summary>The plan's gains, set by design (policies/controllers_unity_stop.json "corner_speed").</summary>
        public const double DesignALat = 3.4, DesignABrake = 1.2, DesignAAccel = 1.2;   // [m/s^2]

        public double ALat = DesignALat, ABrake = DesignABrake, AAccel = DesignAAccel;   // [m/s^2]; AAccel NaN: none
        public double Window = 1.5, Step = 0.1;                    // [m]
        public double KSpeed;                                      // throttle per unit speed-ratio error

        public IDriver Inner { get; }
        /// <summary>The speed the observation is scaled by, and the plan's ceiling [m/s].</summary>
        public double TargetSpeed { get; }
        /// <summary>The plan for this trip [m/s every Step], or null before Plan.</summary>
        public double[] Profile { get; private set; }
        /// <summary>The last tick's cap [m/s], for display.</summary>
        public double Cap { get; private set; } = double.NaN;

        public CornerSpeed(IDriver inner, double targetSpeed, double kSpeed = 3.0)
        {
            Inner = inner;
            TargetSpeed = targetSpeed;
            KSpeed = inner is PurePursuitDriver pp ? pp.KSpeed : kSpeed;
        }

        public string Name => Inner.Name;
        public int History => Inner.History;
        public void Reset() => Inner.Reset();

        /// <summary>Plan a route from its heading at arc length s [rad] and its length [m].</summary>
        public void Plan(Func<double, double> headingAt, double length) =>
            Profile = SpeedProfile.Build(headingAt, length, TargetSpeed, ALat, ABrake, Window, Step, AAccel);

        /// <summary>The planned speed at arc length s, interpolated [m/s].</summary>
        public double CapAt(double s)
        {
            var p = Profile;
            double u = Math.Max(0.0, s) / Step;
            int i = Math.Min((int)u, p.Length - 1);
            int j = Math.Min(i + 1, p.Length - 1);
            return p[i] + (u - i) * (p[j] - p[i]);
        }

        public (double steer, double throttle) Act(in DriveContext c)
        {
            var (steer, throttle) = Inner.Act(c);
            if (Profile == null) { Cap = double.NaN; return (steer, throttle); }
            double cap = CapAt(c.S);
            Cap = cap;
            if (cap >= TargetSpeed) return (steer, throttle);   // no corner in reach: the driver as it is
            double v = c.Observation[0] * TargetSpeed;
            return (steer, Math.Min(throttle, DriverMath.Clip(KSpeed * (cap - v) / TargetSpeed)));
        }
    }

    /// <summary>Backs off a kerb the car is pinned against, then lets the
    /// driver it wraps retry: sim_py/controllers.py's KerbRecovery, line for line.</summary>
    /// <remarks>
    /// On the Unity twin's car a car that strikes a kerb at a junction nose
    /// first stays pinned against it at full throttle (Docs/policy-study.md,
    /// "Closing the gap to the Unity car"). The driver cannot tell: the
    /// observation has no wall contact. Pinned, the car is not still: it
    /// bounces off the kerb and back. So when the driver has asked for
    /// throttle above MinThrottle for StallTime and the car has moved less
    /// than StallDistance in that time (its speed, integrated), it reverses
    /// with the steering on the other lock, which swings the nose toward the
    /// way the driver was steering, for BackDistance or at most BackTime.
    /// </remarks>
    public sealed class KerbRecovery : IDriver
    {
        public double StallDistance = 0.05, StallTime = 0.5;   // [m], [s]
        public double MinThrottle = 0.3, BackThrottle = -0.6;
        public double BackDistance = 0.3, BackTime = 1.5;      // [m], [s]
        public double Tick = 0.01;                             // [s], the control tick (TwinAutopilot.ControlTick)

        public IDriver Inner { get; }
        /// <summary>The speed the observation is scaled by [m/s].</summary>
        public double TargetSpeed { get; }
        /// <summary>Back-offs this trip.</summary>
        public int BackOffs { get; private set; }
        public bool Backing => _backing;

        private double _stalled, _moved;   // how long the driver has asked for throttle, how far the car went in that time
        private bool _backing;
        private double _backSteer, _backT, _backS;

        public KerbRecovery(IDriver inner, double targetSpeed)
        {
            Inner = inner;
            TargetSpeed = targetSpeed;
        }

        public string Name => Inner.Name;
        public int History => Inner.History;

        public void Reset()
        {
            _stalled = _moved = 0.0;
            _backing = false;
            BackOffs = 0;
            Inner.Reset();
        }

        public (double steer, double throttle) Act(in DriveContext c)
        {
            var (steer, throttle) = Inner.Act(c);   // every tick, so a driver with memory keeps it
            double v = c.Observation[0] * TargetSpeed;
            if (_backing)
            {
                _backT += Tick;
                _backS += Math.Max(0.0, -v) * Tick;
                if (_backT < BackTime && _backS < BackDistance) return (_backSteer, BackThrottle);
                _backing = false;
            }
            if (throttle > MinThrottle)
            {
                _stalled += Tick;
                _moved += v * Tick;
                if (Math.Abs(_moved) > StallDistance) _stalled = _moved = 0.0;
            }
            else _stalled = _moved = 0.0;
            if (_stalled >= StallTime)
            {
                _stalled = _moved = 0.0;
                BackOffs++;
                _backSteer = Math.Abs(steer) < 0.05 ? 0.0 : steer > 0.0 ? -1.0 : 1.0;
                _backing = true;
                _backT = _backS = 0.0;
                return (_backSteer, BackThrottle);
            }
            return (steer, throttle);
        }
    }
}
