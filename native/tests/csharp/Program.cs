// Usage: dotnet run -- <dir containing rc_twin.dll>
// Checks the C# struct layouts against the native library by value, then
// drives a car. Exit code 0 means every check passed.
using System;
using System.IO;
using System.Runtime.InteropServices;
using RcTwin;

static class Program
{
    static int failures, checks;

    static void Check(bool ok, string what)
    {
        checks++;
        if (!ok) { failures++; Console.Error.WriteLine("FAIL: " + what); }
    }

    static int Main(string[] args)
    {
        string dir = args.Length > 0 ? args[0] : ".";
        NativeLibrary.SetDllImportResolver(typeof(RcTwinNative).Assembly, (name, asm, path) =>
            name == RcTwinNative.Lib ? NativeLibrary.Load(Path.Combine(dir, "rc_twin.dll")) : IntPtr.Zero);

        Check(RcTwinNative.rc_twin_abi_version() == RcTwinNative.AbiVersion, "ABI version");
        Check(RcTwinNative.CoreVersion == "0.2.0", "core version " + RcTwinNative.CoreVersion);
        Check(Marshal.SizeOf<RcTwinNative.State>() == 28 * 8, "State size");

        var p = RcTwinNative.DefaultParams();
        Check(p.structSize == Marshal.SizeOf<RcTwinNative.Params>(),
              $"Params size: native {p.structSize}, C# {Marshal.SizeOf<RcTwinNative.Params>()}");

        // Every default by value (SlipX params.hpp / tyre.hpp at ebdbf3f), so a
        // field that is out of order lands on a wrong value and fails here.
        void Eq(double got, double want, string n) => Check(got == want, $"{n}: got {got}, want {want}");
        Check(p.provenance == 0, "provenance");
        Eq(p.mass, 3.5, "mass"); Eq(p.izz, 0.05, "izz"); Eq(p.ixx, 0.02, "ixx"); Eq(p.iyy, 0.06, "iyy");
        Eq(p.lf, 0.16, "lf"); Eq(p.lr, 0.16, "lr"); Eq(p.trackFront, 0.24, "trackFront");
        Eq(p.trackRear, 0.24, "trackRear"); Eq(p.hCog, 0.06, "hCog"); Eq(p.wheelRadius, 0.05, "wheelRadius");
        Eq(p.cAlphaF, 420.0, "cAlphaF"); Eq(p.cAlphaR, 455.0, "cAlphaR"); Eq(p.muClip, 1.1, "muClip");
        foreach (var (t, n) in new[] { (p.tyreFront, "front"), (p.tyreRear, "rear") })
        {
            Eq(t.muY0, 1.10, n + ".muY0"); Eq(t.muX0, 1.20, n + ".muX0"); Eq(t.kMu, 0.15, n + ".kMu");
            Eq(t.relaxLength, 0.08, n + ".relaxLength"); Eq(t.shapeC, 1.68, n + ".shapeC");
            Eq(t.curvatureE, 0.42, n + ".curvatureE");
        }
        Eq(p.cKappa, 120.0, "cKappa");
        Eq(p.accelMax, 8.0, "accelMax"); Eq(p.decelMax, 12.0, "decelMax"); Eq(p.vMax, 20.0, "vMax");
        Check(p.layout == 0 && p.differential == 0, "layout/differential");
        Eq(p.lsdPreload, 0.0, "lsdPreload");
        Eq(p.torqueStall, 2.0, "torqueStall"); Eq(p.omegaFree, 480.0, "omegaFree");
        Eq(p.torquePerAmp, 0.01, "torquePerAmp"); Eq(p.driveEfficiency, 0.85, "driveEfficiency");
        Eq(p.currentMax, 120.0, "currentMax"); Eq(p.regenCurrentMax, 40.0, "regenCurrentMax");
        Eq(p.packNominalV, 11.1, "packNominalV"); Eq(p.packVFull, 12.6, "packVFull");
        Eq(p.packVEmpty, 9.9, "packVEmpty"); Eq(p.packCapacityAh, 5.2, "packCapacityAh");
        Eq(p.packInternalResistance, 0.020, "packInternalResistance");
        Eq(p.steerMax, 0.40, "steerMax"); Eq(p.steerRateMax, 10.0, "steerRateMax");
        Eq(p.steerBandwidth, 45.0, "steerBandwidth"); Eq(p.steerDamping, 0.7, "steerDamping");
        Eq(p.dragCoeff, 0.015, "dragCoeff"); Eq(p.rollResist, 0.015, "rollResist"); Eq(p.vEps, 0.5, "vEps");

        using (var car = new TwinVehicle(RcTwinNative.Tier.L2DoubleTrack, p))
        {
            car.Step(0.0, 2.0, 0.001, 1000);
            var s = car.State;
            Check(s.vx > 1.0 && s.vx < 2.0, $"straight-line vx {s.vx}");
            Check(Math.Abs(s.y) < 1e-9 && Math.Abs(s.yaw) < 1e-9, "goes straight");
            Check(s.soc < 1.0, "draws charge");
            car.Step(0.2, 0.0, 0.001, 1000);
            s = car.State;
            Check(s.yaw > 0.1 && s.y > 0.0, "left steer turns left");

            var heavier = p; heavier.mass = 5.0;
            car.Params = heavier;
            Check(car.Params.mass == 5.0, "set/get params");

            var snap = car.State;
            car.Step(0.1, 1.0, 0.001, 50);
            car.Reset(snap);
            Check(car.State.Equals(snap), "reset to snapshot");
            car.ResetToRest();
            Check(car.State.x == 0.0 && car.State.soc == 1.0, "reset to rest");
        }

        bool threw = false;
        var bad = p; bad.mass = -1.0;
        try { new TwinVehicle(RcTwinNative.Tier.L2DoubleTrack, bad).Dispose(); }
        catch (ArgumentException e) { threw = e.Message.Contains("mass"); }
        Check(threw, "invalid params raise with the Core's reason");

        Console.WriteLine($"{checks} checks, {failures} failed");
        return failures == 0 ? 0 : 1;
    }
}
