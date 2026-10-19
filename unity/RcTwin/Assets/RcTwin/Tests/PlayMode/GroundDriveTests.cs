// Play Mode: the Core feeling the FlatGround scene's ground through
// RaycastGround (ADR 0005). Positions are ISO world coordinates: Unity
// (ux, uz) on the ground is ISO (x = uz, y = -ux).
using System.Collections;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;

namespace RcTwin.Tests
{
    public class GroundDriveTests : InputTestFixture
    {
        private TwinCar _car;
        private Keyboard _kb;

        private IEnumerator Load()
        {
            _kb = InputSystem.AddDevice<Keyboard>();
            SceneManager.LoadScene("FlatGround");
            yield return null;
            yield return null;
            _car = Object.FindFirstObjectByType<TwinCar>();
            Assert.IsNotNull(_car);
            Assert.IsNotNull(_car.GetComponent<RaycastGround>(), "the car reads the scene's ground");
        }

        // Holds W for the run if throttle is set. The caller releases it with
        // Lift(): code after the last yield of a nested coroutine is not run
        // by the test runner, so a release here would never happen.
        private IEnumerator Run(double x, double y, double yaw, double vx, float seconds, bool throttle)
        {
            _car.ResetTo(x, y, yaw, vx);
            if (throttle) Press(_kb.wKey);
            yield return new WaitForSeconds(seconds);
        }

        private void Lift()
        {
            if (_kb.wKey.isPressed) Release(_kb.wKey);
            if (_kb.sKey.isPressed) Release(_kb.sKey);
        }

        [UnityTest]
        public IEnumerator TheDownRampSpeedsTheCarUp()
        {
            yield return Load();
            yield return Run(5.0, -10.0, 0.0, 1.0, 1.0f, false);   // flat ground
            double flat = _car.State.vx;
            yield return Run(19.5, 0.0, 0.0, 1.0, 1.0f, false);    // the 10 % down ramp
            double ramp = _car.State.vx;
            Assert.Greater(ramp, flat + 0.6, $"ramp {ramp:F3} m/s vs flat {flat:F3} m/s (g sin 5.7 deg is 0.98 m/s^2)");
            Assert.Less(_car.State.z, 0.45, "it went down");
            Assert.IsFalse(_car.OffGround);
        }

        [UnityTest]
        public IEnumerator ThePlateauIsSixtyCentimetresUp()
        {
            yield return Load();
            yield return Run(16.0, 0.0, 0.0, 0.0, 0.1f, false);
            Assert.That(_car.State.z, Is.EqualTo(0.6).Within(1e-4));
            Assert.That(_car.transform.position.y, Is.EqualTo(0.6f + (float)_car.Params.hCog).Within(1e-3f));
        }

        [UnityTest]
        public IEnumerator IceCostsTraction()
        {
            yield return Load();
            yield return Run(2.0, -10.0, 0.0, 0.5, 1.0f, true);    // flat ground
            Lift();
            double grip = _car.State.vx;
            yield return null;
            yield return Run(2.0, 5.5, 0.0, 0.5, 1.0f, true);      // the ice patch
            Lift();
            double ice = _car.State.vx;
            Assert.Less(ice, 0.5 * grip, $"ice {ice:F3} m/s vs grip {grip:F3} m/s");
            Assert.Greater(ice, 0.5, "but some traction remains");
        }

        [UnityTest]
        public IEnumerator TheBankPushesTowardItsLowSide()
        {
            yield return Load();
            yield return Run(5.0, -5.5, 0.0, 2.0, 1.0f, false);    // on the bank, right side high
            // At 2 m/s the tyres hold the car at a sideslip of about
            // m g sin(phi) / C_alpha, about 8 mm/s sideways: 9.4 mm in 1 s natively.
            Assert.Greater(_car.State.y, -5.5 + 0.005, "drifted left, toward the low edge");
            Assert.Greater(_car.State.yaw, 0.0, "and turned toward it");
        }

        [UnityTest]
        public IEnumerator OnIceTheRearWheelsSpinAndThenLock()
        {
            yield return Load();
            Assert.Greater(_car.Params.wheelInertia, 0.0, "wheel dynamics on (ADR 0006)");
            yield return Run(2.0, 5.5, 0.0, 0.5, 0.5f, true);      // full throttle on ice
            Lift();
            Assert.Greater(_car.Slip(2), 0.5, "rear left spinning");
            Assert.Greater(_car.Slip(3), 0.5, "rear right spinning");
            Assert.Less(System.Math.Abs(_car.Slip(0)), 0.05, "front rolling");

            yield return null;                                       // the release lands
            Assert.IsFalse(_kb.wKey.isPressed);
            _car.ResetTo(2.0, 5.5, 0.0, 3.0);                       // rolling at 3 m/s
            Press(_kb.sKey);
            yield return new WaitForSeconds(0.3f);
            Lift();
            Assert.AreEqual(0.0, _car.State.omegaRL, "rear left locked");
            Assert.AreEqual(0.0, _car.State.omegaRR, "rear right locked");
            Assert.Greater(_car.State.vx, 1.5, "still sliding");
        }

        [UnityTest]
        public IEnumerator NoGroundStopsTheCarInsteadOfGuessing()
        {
            yield return Load();
            _car.GetComponent<RaycastGround>().groundMask = 0;
            LogAssert.Expect(LogType.Warning, new System.Text.RegularExpressions.Regex("TwinCar stopped: no ground"));
            yield return Run(0.0, 0.0, 0.0, 1.0, 0.2f, false);
            Assert.IsTrue(_car.OffGround);
            Assert.AreEqual(0.0, _car.State.x, "state kept as it was");
        }
    }
}
