// The RL environment's observation (sim_py/rc_twin_env.py, FRAME_FIELDS),
// computed here from the Core's state and a TwinRoad, so a controller or a
// learned policy drives the Unity twin from the numbers it was built on.
// Field for field, formula for formula; tests/Editor/AutopilotParityTests.cs
// checks it against frames Python recorded.
//
// Free of UnityEngine, like RcTwinNative.
using System;
using System.Collections.Generic;

namespace RcTwin
{
    public sealed class TwinObservation
    {
        public static readonly double[] LookaheadM = { 0.5, 1.0, 2.0 };

        public static readonly string[] Fields =
        {
            "speed_ratio", "lateral_velocity", "yaw_rate",
            "slip_angle_fl", "slip_angle_fr", "slip_angle_rl", "slip_angle_rr",
            "wheel_speed_ratio_fl", "wheel_speed_ratio_fr", "wheel_speed_ratio_rl", "wheel_speed_ratio_rr",
            "soc", "steer_ratio", "grade", "offset_ratio", "heading_error",
            "ahead_0.5_x", "ahead_0.5_y", "ahead_1.0_x", "ahead_1.0_y", "ahead_2.0_x", "ahead_2.0_y",
            "target_speed_ratio",
        };

        public static int FrameSize => Fields.Length;

        public static int Index(string field)
        {
            int i = Array.IndexOf(Fields, field);
            if (i < 0) throw new ArgumentException($"no observation field {field}");
            return i;
        }

        /// <summary>Python's a % b for floats: the sign of b.</summary>
        public static double Wrap(double a)
        {
            double r = (a + Math.PI) % (2.0 * Math.PI);
            if (r != 0.0 && r < 0.0) r += 2.0 * Math.PI;
            return r - Math.PI;
        }

        /// <summary>One frame: the car's state, measured along the road at arc
        /// length s with offset d (from TwinRoad.Locate).</summary>
        public static double[] Frame(TwinRoad road, in RcTwinNative.State st, in RcTwinNative.Params p,
                                     double targetSpeed, double s, double d)
        {
            double vRef = targetSpeed;
            var (_, _, _, h) = road.Point(s);
            double headingError = Wrap(st.yaw - h);
            double grade = 0.0;
            if (road.Ground(st.x, st.y, out var g) && g.nz > 0.0)
                grade = -(g.nx * Math.Cos(st.yaw) + g.ny * Math.Sin(st.yaw)) / g.nz;
            double cy = Math.Cos(st.yaw), sy = Math.Sin(st.yaw);
            var f = new double[FrameSize];
            double r = p.wheelRadius;
            f[0] = st.vx / vRef;
            f[1] = st.vy;
            f[2] = st.yawRate;
            f[3] = st.alphaLagFL; f[4] = st.alphaLagFR; f[5] = st.alphaLagRL; f[6] = st.alphaLagRR;
            f[7] = st.omegaFL * r / vRef; f[8] = st.omegaFR * r / vRef;
            f[9] = st.omegaRL * r / vRef; f[10] = st.omegaRR * r / vRef;
            f[11] = st.soc;
            f[12] = st.steer / p.steerMax;
            f[13] = grade;
            f[14] = d / (0.5 * road.Width);
            f[15] = headingError;
            for (int k = 0; k < LookaheadM.Length; k++)
            {
                var (ax, ay, _, _) = road.Point(s + LookaheadM[k]);
                double dx = ax - st.x, dy = ay - st.y;
                f[16 + 2 * k] = cy * dx + sy * dy;
                f[17 + 2 * k] = -sy * dx + cy * dy;
            }
            f[22] = 1.0;
            return f;
        }

        // ------------------------------------------------------- history

        private readonly int _history;
        private double[] _now;
        private readonly List<(double[] frame, double steer, double throttle)> _hist =
            new List<(double[], double, double)>();

        /// <param name="history">Previous frames carried, each with the action
        /// taken after it, newest first (the environment's `history`).</param>
        public TwinObservation(int history) => _history = history;

        public int Size => FrameSize + _history * (FrameSize + 2);

        /// <summary>Start of an episode: the history is this frame, repeated, with no action.</summary>
        public void Reset(double[] frame)
        {
            _now = frame;
            _hist.Clear();
            for (int i = 0; i < _history; i++) _hist.Add((frame, 0.0, 0.0));
        }

        /// <summary>After an action (clipped to [-1, 1]) and the step it drove:
        /// the frame it was taken on joins the history, the new one is current.</summary>
        public void Push(double steer, double throttle, double[] newFrame)
        {
            _hist.Insert(0, (_now, steer, throttle));
            while (_hist.Count > _history) _hist.RemoveAt(_hist.Count - 1);
            _now = newFrame;
        }

        public double[] Current => _now;

        public double[] Vector()
        {
            var o = new double[Size];
            Array.Copy(_now, o, FrameSize);
            int k = FrameSize;
            foreach (var (frame, steer, throttle) in _hist)
            {
                Array.Copy(frame, 0, o, k, FrameSize);
                k += FrameSize;
                o[k++] = steer;
                o[k++] = throttle;
            }
            return o;
        }
    }
}
