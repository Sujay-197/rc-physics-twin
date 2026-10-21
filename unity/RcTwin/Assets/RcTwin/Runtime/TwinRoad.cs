// A road centreline in the native layer (rc_twin_road_*): where the car is
// along it and how far off it, the centreline ahead, and the ground the road
// data gives. The RL environment measures its observation the same way
// (sim_py/rc_twin_env.py), so a driver sees the same numbers here.
//
// Free of UnityEngine, like RcTwinNative.
using System;
using System.Runtime.InteropServices;

namespace RcTwin
{
    public sealed class TwinRoad : IDisposable
    {
        private IntPtr _h;

        public double Length { get; }
        public double Width { get; }

        /// <param name="xyz">Centreline points as x, y, z triples, ISO world [m].</param>
        public TwinRoad(double[] xyz, double width)
        {
            if (xyz == null || xyz.Length < 6 || xyz.Length % 3 != 0)
                throw new ArgumentException("a road needs at least two x, y, z points");
            _h = RcTwinNative.rc_twin_road_create(xyz, null, (uint)(xyz.Length / 3), width, out var why);
            if (_h == IntPtr.Zero)
                throw new ArgumentException("rc_twin_road_create: " + Marshal.PtrToStringAnsi(why));
            Length = RcTwinNative.rc_twin_road_length(_h);
            Width = width;
        }

        /// <summary>Arc length of the nearest centreline point and the signed
        /// offset from it (left positive).</summary>
        public (double s, double d) Locate(double x, double y)
        {
            RcTwinNative.Check(RcTwinNative.rc_twin_road_locate(Handle, x, y, out double s, out double d), "rc_twin_road_locate");
            return (s, d);
        }

        /// <summary>The centreline at arc length s: position and heading.</summary>
        public (double x, double y, double z, double heading) Point(double s)
        {
            RcTwinNative.Check(RcTwinNative.rc_twin_road_point(Handle, s, out double x, out double y, out double z, out double h),
                               "rc_twin_road_point");
            return (x, y, z, h);
        }

        /// <summary>The road's ground under (x, y); false off its width.</summary>
        public bool Ground(double x, double y, out RcTwinNative.GroundSample g) =>
            RcTwinNative.rc_twin_road_ground(Handle, 0, x, y, out g) == 0;

        private IntPtr Handle => _h != IntPtr.Zero ? _h : throw new ObjectDisposedException(nameof(TwinRoad));

        public void Dispose()
        {
            if (_h != IntPtr.Zero) RcTwinNative.rc_twin_road_destroy(_h);
            _h = IntPtr.Zero;
        }

        ~TwinRoad() => Dispose();
    }
}
