// Drivers: what turns an observation into a command. The autopilot
// (TwinAutopilot) asks one each control tick; a new controller or a learned
// policy is a new IDriver, nothing else changes.
//
// PurePursuitDriver is sim_py/controllers.py's PurePursuit, line for line,
// stop plan included (tests/Editor/AutopilotParityTests.cs checks it
// against Python in both RL tasks). PolicyDriver runs a trained network.
// KerbRecovery wraps either and backs the car off a kerb it is pinned against.
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
