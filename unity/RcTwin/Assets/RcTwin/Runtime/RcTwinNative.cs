// P/Invoke bindings for native/include/rc_twin.h (ABI version 1).
//
// Deliberately free of UnityEngine so native/tests/csharp can compile and run
// it against rc_twin.dll outside the editor. Field order here must match the
// C structs exactly; the struct_size check in rc_twin_create catches a size
// mismatch, and the binding test checks every default by value.
//
// Conventions are SlipX's: ISO 8855 (x forward, y LEFT, z up), SI units,
// wheels FL, FR, RL, RR.
using System;
using System.Runtime.InteropServices;

namespace RcTwin
{
    public static class RcTwinNative
    {
        public const string Lib = "rc_twin";
        public const uint AbiVersion = 1;

        public const int Ok = 0, ENull = -1, EInvalid = -2, EInternal = -3;

        public enum Tier { L0Kinematic = 0, L1Bicycle = 1, L2DoubleTrack = 2, L3Extended = 3 }
        public enum Integrator { Rk4 = 0, SemiImplicitEuler = 1 }

        [StructLayout(LayoutKind.Sequential)]
        public struct State
        {
            public double x, y, z;
            public double yaw, pitch, roll;
            public double vx, vy, vz;
            public double rollRate, pitchRate, yawRate;
            public double omegaFL, omegaFR, omegaRL, omegaRR;
            public double steer, steerRate, soc, packV;
            public double fzFL, fzFR, fzRL, fzRR;
            public double alphaLagFL, alphaLagFR, alphaLagRL, alphaLagRR;

            public double Speed => Math.Sqrt(vx * vx + vy * vy);
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct Tyre
        {
            public double muY0, muX0, kMu, relaxLength, shapeC, curvatureE;
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct Params
        {
            public uint structSize;
            public int provenance;  // 0 provisional, 1 identified, 2 measured

            public double mass, izz, ixx, iyy;
            public double lf, lr, trackFront, trackRear, hCog, wheelRadius;

            public double cAlphaF, cAlphaR, muClip;
            public Tyre tyreFront, tyreRear;
            public double cKappa;

            public double accelMax, decelMax, vMax;
            public int layout;        // 0 RWD, 1 FWD, 2 AWD
            public int differential;  // 0 open, 1 spool, 2 LSD
            public double lsdPreload;

            public double torqueStall, omegaFree, torquePerAmp, driveEfficiency;
            public double currentMax, regenCurrentMax;

            public double packNominalV, packVFull, packVEmpty, packCapacityAh,
                packInternalResistance;

            public double steerMax, steerRateMax, steerBandwidth, steerDamping;

            public double dragCoeff, rollResist, vEps;
        }

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        public static extern uint rc_twin_abi_version();

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr rc_twin_core_version();

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rc_twin_params_default(out Params p);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr rc_twin_create(int tier, int integrator, ref Params p,
                                                   out IntPtr reason);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        public static extern void rc_twin_destroy(IntPtr v);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rc_twin_set_params(IntPtr v, ref Params p, out IntPtr reason);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rc_twin_get_params(IntPtr v, out Params p);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rc_twin_reset(IntPtr v, ref State initial);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl, EntryPoint = "rc_twin_reset")]
        public static extern int rc_twin_reset_to_rest(IntPtr v, IntPtr mustBeZero);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rc_twin_get_state(IntPtr v, out State s);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rc_twin_step(IntPtr v, double steerCmd, double accelCmd,
                                              double dt, uint n);

        public static string CoreVersion => Marshal.PtrToStringAnsi(rc_twin_core_version());

        public static Params DefaultParams()
        {
            Check(rc_twin_params_default(out var p), "rc_twin_params_default");
            return p;
        }

        internal static void Check(int status, string what)
        {
            if (status != Ok) throw new InvalidOperationException($"{what} failed with status {status}");
        }
    }

    /// <summary>One simulated vehicle. Owns the native handle.</summary>
    public sealed class TwinVehicle : IDisposable
    {
        private IntPtr _handle;

        public TwinVehicle(RcTwinNative.Tier tier, RcTwinNative.Params p,
                           RcTwinNative.Integrator integrator = RcTwinNative.Integrator.Rk4)
        {
            if (RcTwinNative.rc_twin_abi_version() != RcTwinNative.AbiVersion)
                throw new InvalidOperationException(
                    $"rc_twin ABI {RcTwinNative.rc_twin_abi_version()} does not match bindings {RcTwinNative.AbiVersion}");
            _handle = RcTwinNative.rc_twin_create((int)tier, (int)integrator, ref p, out var reason);
            if (_handle == IntPtr.Zero)
                throw new ArgumentException("rc_twin_create: " + Marshal.PtrToStringAnsi(reason));
        }

        public RcTwinNative.State State
        {
            get { RcTwinNative.Check(RcTwinNative.rc_twin_get_state(Live, out var s), "get_state"); return s; }
        }

        public RcTwinNative.Params Params
        {
            get { RcTwinNative.Check(RcTwinNative.rc_twin_get_params(Live, out var p), "get_params"); return p; }
            set
            {
                if (RcTwinNative.rc_twin_set_params(Live, ref value, out var reason) != RcTwinNative.Ok)
                    throw new ArgumentException("rc_twin_set_params: " + Marshal.PtrToStringAnsi(reason));
            }
        }

        public void Step(double steerCmd, double accelCmd, double dt, uint n = 1) =>
            RcTwinNative.Check(RcTwinNative.rc_twin_step(Live, steerCmd, accelCmd, dt, n), "step");

        public void ResetToRest() =>
            RcTwinNative.Check(RcTwinNative.rc_twin_reset_to_rest(Live, IntPtr.Zero), "reset");

        public void Reset(RcTwinNative.State s) =>
            RcTwinNative.Check(RcTwinNative.rc_twin_reset(Live, ref s), "reset");

        private IntPtr Live => _handle != IntPtr.Zero ? _handle : throw new ObjectDisposedException(nameof(TwinVehicle));

        public void Dispose()
        {
            if (_handle != IntPtr.Zero) { RcTwinNative.rc_twin_destroy(_handle); _handle = IntPtr.Zero; }
            GC.SuppressFinalize(this);
        }

        ~TwinVehicle()
        {
            if (_handle != IntPtr.Zero) RcTwinNative.rc_twin_destroy(_handle);
        }
    }
}
