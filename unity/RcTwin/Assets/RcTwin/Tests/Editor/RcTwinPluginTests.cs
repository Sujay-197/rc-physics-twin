// Edit Mode tests (Window > General > Test Runner): the plugin loads inside
// the Unity editor and behaves as it does in native/tests. The by-value
// layout check lives in native/tests/csharp, which runs without Unity.
using NUnit.Framework;
using System;

namespace RcTwin.Tests
{
    public class RcTwinPluginTests
    {
        [Test]
        public void PluginLoadsWithMatchingAbi()
        {
            Assert.AreEqual(RcTwinNative.AbiVersion, RcTwinNative.rc_twin_abi_version());
            Assert.AreEqual("0.2.0", RcTwinNative.CoreVersion);
        }

        [Test]
        public void DefaultsArriveIntact()
        {
            var p = RcTwinNative.DefaultParams();
            Assert.AreEqual(System.Runtime.InteropServices.Marshal.SizeOf<RcTwinNative.Params>(), (int)p.structSize);
            Assert.AreEqual(3.5, p.mass);
            Assert.AreEqual(0.5, p.vEps);  // last field: catches any shift
        }

        [Test]
        public void DrivesStraightThenTurnsLeft()
        {
            using var car = new TwinVehicle(RcTwinNative.Tier.L2DoubleTrack, RcTwinNative.DefaultParams());
            car.Step(0.0, 2.0, 0.001, 1000);
            var s = car.State;
            Assert.That(s.vx, Is.InRange(1.0, 2.0));
            Assert.That(Math.Abs(s.yaw), Is.LessThan(1e-9));
            car.Step(0.2, 0.0, 0.001, 1000);
            Assert.That(car.State.yaw, Is.GreaterThan(0.1));
        }

        [Test]
        public void IsoToUnityMapping()
        {
            // ISO forward is Unity forward; ISO left is Unity -x; ISO up is Unity up.
            Assert.AreEqual(new UnityEngine.Vector3(0, 0, 1), TwinCar.ToUnity(1, 0, 0));
            Assert.AreEqual(new UnityEngine.Vector3(-1, 0, 0), TwinCar.ToUnity(0, 1, 0));
            Assert.AreEqual(new UnityEngine.Vector3(0, 1, 0), TwinCar.ToUnity(0, 0, 1));
            // A left turn (ISO yaw +90 deg) faces Unity -x.
            var fwd = UnityEngine.Quaternion.Euler(0, TwinCar.YawToUnityDeg(Math.PI / 2), 0) * UnityEngine.Vector3.forward;
            Assert.That(fwd.x, Is.EqualTo(-1f).Within(1e-5f));
        }

        [Test]
        public void InvalidParamsAreRejectedWithReason()
        {
            var p = RcTwinNative.DefaultParams();
            p.mass = -1;
            var e = Assert.Throws<ArgumentException>(() => new TwinVehicle(RcTwinNative.Tier.L2DoubleTrack, p));
            StringAssert.Contains("mass", e.Message);
        }
    }
}
