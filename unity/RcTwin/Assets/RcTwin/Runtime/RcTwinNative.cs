// P/Invoke bindings for native/include/rc_twin.h (ABI version 3).
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
        public const uint AbiVersion = 3;

        public const int Ok = 0, ENull = -1, EInvalid = -2, EInternal = -3, EGround = -4;

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

            public double wheelInertia;  // per wheel [kg m^2]; 0 = quasi-static wheels (ADR 0006)
        }

        /// <summary>Ground under one wheel, world frame, ISO 8855 (ADR 0005).</summary>
        [StructLayout(LayoutKind.Sequential)]
        public struct GroundSample
        {
            public double height;      // [m]
            public double nx, ny, nz;  // outward normal, any length, nz > 0
            public double muScale;     // multiplies peak tyre friction, >= 0

            public static GroundSample Level(double height = 0.0) =>
                new GroundSample { height = height, nz = 1.0, muScale = 1.0 };
        }

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        public delegate int GroundFn(IntPtr user, int wheel, double x, double y, ref GroundSample sample);

        [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
        public static extern int rc_twin_set_ground_query(IntPtr v, GroundFn fn, IntPtr user);

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

    /// <summary>
    /// Answers the Core's ground query for one wheel (0 FL, 1 FR, 2 RL, 3 RR) at
    /// world ISO (x, y). <paramref name="sample"/> arrives as level ground at
    /// height 0. Return false if there is no ground there; the step then fails.
    /// </summary>
    public interface IGroundProvider
    {
        bool Sample(int wheel, double x, double y, ref RcTwinNative.GroundSample sample);
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

        public void Step(double steerCmd, double accelCmd, double dt, uint n = 1)
        {
            int status = RcTwinNative.rc_twin_step(Live, steerCmd, accelCmd, dt, n);
            if (status == RcTwinNative.EGround)
            {
                string why = _groundBox?.error ?? "the samples were invalid or this tier cannot represent this ground";
                if (_groundBox != null) _groundBox.error = null;
                throw new GroundQueryException(why);
            }
            RcTwinNative.Check(status, "step");
        }

        // ------------------------------------------------------------ ground
        // The provider travels to native code as a GCHandle; the callback is one
        // static method, so it is AOT-safe (IL2CPP) and never collected. The box
        // holds no reference back to this vehicle, so an undisposed vehicle can
        // still be finalised.
        private GCHandle _groundHandle;
        private GroundBox _groundBox;
        private static readonly RcTwinNative.GroundFn GroundThunk = OnGround;

        /// <summary>Feed the Core this provider's ground; null restores level ground.</summary>
        public void SetGround(IGroundProvider provider)
        {
            if (provider != null)
            {
                var box = new GroundBox { provider = provider };
                var handle = GCHandle.Alloc(box);
                RcTwinNative.Check(RcTwinNative.rc_twin_set_ground_query(Live, GroundThunk, GCHandle.ToIntPtr(handle)),
                                   "set_ground_query");
                if (_groundHandle.IsAllocated) _groundHandle.Free();
                _groundHandle = handle;
                _groundBox = box;
            }
            else
            {
                RcTwinNative.Check(RcTwinNative.rc_twin_set_ground_query(Live, null, IntPtr.Zero), "set_ground_query");
                if (_groundHandle.IsAllocated) _groundHandle.Free();
                _groundBox = null;
            }
        }

        private sealed class GroundBox
        {
            public IGroundProvider provider;
            public string error;  // why the last query failed, for the exception
        }

        [MonoPInvokeCallback(typeof(RcTwinNative.GroundFn))]
        private static int OnGround(IntPtr user, int wheel, double x, double y, ref RcTwinNative.GroundSample sample)
        {
            GroundBox box = null;
            try
            {
                box = (GroundBox)GCHandle.FromIntPtr(user).Target;
                if (box.provider.Sample(wheel, x, y, ref sample)) return 0;
                box.error = $"no ground under wheel {wheel} at ({x:F2}, {y:F2})";
                return 1;
            }
            catch (Exception e)  // never let an exception unwind into native code
            {
                if (box != null) box.error = "ground provider threw: " + e.Message;
                return 1;
            }
        }

        public void ResetToRest() =>
            RcTwinNative.Check(RcTwinNative.rc_twin_reset_to_rest(Live, IntPtr.Zero), "reset");

        public void Reset(RcTwinNative.State s) =>
            RcTwinNative.Check(RcTwinNative.rc_twin_reset(Live, ref s), "reset");

        private IntPtr Live => _handle != IntPtr.Zero ? _handle : throw new ObjectDisposedException(nameof(TwinVehicle));

        public void Dispose()
        {
            if (_handle != IntPtr.Zero) { RcTwinNative.rc_twin_destroy(_handle); _handle = IntPtr.Zero; }
            if (_groundHandle.IsAllocated) _groundHandle.Free();
            GC.SuppressFinalize(this);
        }

        ~TwinVehicle()
        {
            if (_handle != IntPtr.Zero) RcTwinNative.rc_twin_destroy(_handle);
            if (_groundHandle.IsAllocated) _groundHandle.Free();
        }
    }

    /// <summary>The ground query reported no ground, bad ground, or ground the tier cannot represent.</summary>
    public sealed class GroundQueryException : InvalidOperationException
    {
        public GroundQueryException(string message) : base(message) { }
    }

    /// <summary>
    /// IL2CPP recognises this attribute by name; defined here so this file needs
    /// no UnityEngine reference.
    /// </summary>
    [AttributeUsage(AttributeTargets.Method)]
    internal sealed class MonoPInvokeCallbackAttribute : Attribute
    {
        public MonoPInvokeCallbackAttribute(Type type) { }
    }
}
