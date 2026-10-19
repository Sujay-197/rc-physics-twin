// Usage: dotnet run -- <dir containing rc_twin.dll>
// Checks the C# struct layouts against the native library by value, then
// drives a car. Exit code 0 means every check passed.
using System;
using System.IO;
using System.Runtime.InteropServices;
using RcTwin;

sealed class Slope : IGroundProvider
{
    public double nx;
    public int calls;
    public bool missing;
    public bool Sample(int wheel, double x, double y, ref RcTwinNative.GroundSample g)
    {
        calls++;
        if (missing) return false;
        g.nx = nx;
        return true;
    }
}

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
        Eq(p.wheelInertia, 0.0, "wheelInertia");
        Eq(p.ackermann, 0.0, "ackermann");
        Eq(p.wheelRateFront, 0.0, "wheelRateFront"); Eq(p.wheelRateRear, 0.0, "wheelRateRear");
        Eq(p.dampingFront, 0.0, "dampingFront"); Eq(p.dampingRear, 0.0, "dampingRear");
        Eq(p.suspensionTravel, 0.0, "suspensionTravel");

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

            var heavier = p; heavier.mass = 5.0; heavier.wheelInertia = 2e-5; heavier.ackermann = 0.8;
            heavier.wheelRateFront = 310; heavier.wheelRateRear = 290; heavier.dampingFront = 9;
            heavier.dampingRear = 8; heavier.suspensionTravel = 0.015;
            car.Params = heavier;
            Check(car.Params.mass == 5.0, "set/get params");
            Check(car.Params.wheelInertia == 2e-5, "wheel inertia round trip");
            Check(car.Params.ackermann == 0.8, "ackermann round trip");
            Check(car.Params.wheelRateFront == 310 && car.Params.wheelRateRear == 290, "wheel rates round trip");
            Check(car.Params.dampingFront == 9 && car.Params.dampingRear == 8, "damping round trip");
            Check(car.Params.suspensionTravel == 0.015, "travel round trip (last field)");

            var snap = car.State;
            car.Step(0.1, 1.0, 0.001, 50);
            car.Reset(snap);
            Check(car.State.Equals(snap), "reset to snapshot");
            car.ResetToRest();
            Check(car.State.x == 0.0 && car.State.soc == 1.0, "reset to rest");
        }

        // Ground query from C#: called 4 n times, a grade reaches the Core, and a
        // provider with no ground fails the step with its reason, keeping state.
        using (var flat = new TwinVehicle(RcTwinNative.Tier.L2DoubleTrack, p))
        using (var hill = new TwinVehicle(RcTwinNative.Tier.L2DoubleTrack, p))
        {
            var slope = new Slope { nx = 0.05 };
            hill.SetGround(slope);
            flat.Step(0.0, 3.0, 0.001, 500);
            hill.Step(0.0, 3.0, 0.001, 500);
            Check(slope.calls == 2000, $"ground calls {slope.calls}");
            Check(hill.State.vx > flat.State.vx + 0.1, "downhill is faster");
            var before = hill.State;
            slope.missing = true;
            bool groundThrew = false;
            try { hill.Step(0.0, 3.0, 0.001, 1); }
            catch (GroundQueryException e) { groundThrew = e.Message.Contains("no ground"); }
            Check(groundThrew, "missing ground raises with its reason");
            Check(hill.State.Equals(before), "failed step keeps state");
            hill.SetGround(null);
            hill.Step(0.0, 3.0, 0.001, 1);  // level again, no throw
        }

        bool threw = false;
        var bad = p; bad.mass = -1.0;
        try { new TwinVehicle(RcTwinNative.Tier.L2DoubleTrack, bad).Dispose(); }
        catch (ArgumentException e) { threw = e.Message.Contains("mass"); }
        Check(threw, "invalid params raise with the Core's reason");

        // Walls (ADR 0009): struct layouts, defaults, and a head-on rebound.
        Check(Marshal.SizeOf<RcTwinNative.Contact>() == 6 * 8, "Contact size");
        Check(Marshal.SizeOf<RcTwinNative.WallReport>() == 24, "WallReport size");
        using (var car = new TwinVehicle(RcTwinNative.Tier.L2DoubleTrack, RcTwinNative.DefaultParams()))
        {
            var dp = RcTwinNative.DefaultParams();
            Check(RcTwinNative.rc_twin_contact_default(ref dp, out var cd) == RcTwinNative.Ok, "contact_default");
            var c = car.Contact;
            Check(c.Equals(cd), "a new car has the default contact");
            Check(c.restitution == 0.3 && c.friction == 0.5 && c.restitutionMinSpeed == 0.1, "SlipX constants");
            Check(c.halfLength == 0.5 * (dp.lf + dp.lr) + dp.wheelRadius && c.halfWidth == 0.5 * dp.trackFront,
                  "footprint from geometry");
            c.halfWidth += 0.0175;
            car.Contact = c;
            Check(car.Contact.halfWidth == c.halfWidth, "contact round trip (last but one field)");
            Check(car.Contact.centreOffset == c.centreOffset, "contact round trip (last field)");
            double front = c.centreOffset + c.halfLength, wall = front + 0.05;
            car.AddWall(new[] { wall, -1.0, wall, 1.0 }, false);
            car.AddWall(new[] { 5.0, 5.0, 6.0, 5.0, 6.0, 6.0 }, true);
            Check(car.WallCount == 4, $"wall count {car.WallCount}");
            var s0 = new RcTwinNative.State { vx = 1.0, soc = 1.0, packV = dp.packVFull,
                                              omegaFL = 20, omegaFR = 20, omegaRL = 20, omegaRR = 20 };
            car.Reset(s0);
            bool hit = false;
            for (int k = 0; k < 300 && !hit; k++)
            {
                car.Step(0.0, 0.0, 0.001);
                var r = car.LastWallReport;
                if (r.contacts == 0) continue;
                hit = true;
                Check(r.contacts == 1 && r.maxApproach > 0.9 && r.impulse > 0.0, "report fields");
                Check(Math.Abs(car.State.vx + 0.3 * r.maxApproach) < 1e-9, $"rebound vx {car.State.vx}");
            }
            Check(hit, "the wall was hit");
            car.ClearWalls();
            Check(car.WallCount == 0, "clear walls");
            bool badWall = false;
            try { car.AddWall(new[] { 0.0, 0.0, 1.0 }, false); } catch (ArgumentException) { badWall = true; }
            Check(badWall, "odd coordinate count refused");
        }

        Console.WriteLine($"{checks} checks, {failures} failed");
        return failures == 0 ? 0 : 1;
    }
}
