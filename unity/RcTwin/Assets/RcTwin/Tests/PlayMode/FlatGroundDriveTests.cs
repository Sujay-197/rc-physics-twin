// Play Mode: the FlatGround scene, driven through a simulated keyboard. Covers
// what Edit Mode cannot: FixedUpdate stepping, input mapping, and the
// ISO-to-Unity pose on the real transform.
using System.Collections;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;

namespace RcTwin.Tests
{
    public class FlatGroundDriveTests : InputTestFixture
    {
        [UnityTest]
        public IEnumerator ThrottleDrivesForwardThenLeftSteerTurnsLeft()
        {
            var kb = InputSystem.AddDevice<Keyboard>();
            SceneManager.LoadScene("FlatGround");
            yield return null;
            yield return null;

            var car = Object.FindFirstObjectByType<TwinCar>();
            Assert.IsNotNull(car, "FlatGround has a TwinCar");
            Vector3 start = car.transform.position;

            Press(kb.wKey);
            yield return new WaitForSeconds(1.0f);
            var s = car.State;
            Assert.Greater(s.vx, 1.0, "W accelerates the Core car");
            Assert.Less(System.Math.Abs(s.yaw), 1e-6, "no steer, no yaw");
            Assert.Greater(car.transform.position.z - start.z, 0.3f, "ISO +x is Unity +z");
            Assert.Less(Mathf.Abs(car.transform.position.x - start.x), 1e-3f);

            Press(kb.aKey);
            yield return new WaitForSeconds(1.0f);
            Release(kb.aKey);
            Release(kb.wKey);
            s = car.State;
            Assert.Greater(s.yaw, 0.1, "A steers left: yaw increases (ISO CCW)");
            Assert.Less(car.transform.position.x, start.x - 0.05f, "a left turn moves toward Unity -x");
            Assert.Less(s.soc, 1.0, "driving drew charge");
        }

        [UnityTest]
        public IEnumerator ResetKeyReturnsToRest()
        {
            var kb = InputSystem.AddDevice<Keyboard>();
            SceneManager.LoadScene("FlatGround");
            yield return null;
            var car = Object.FindFirstObjectByType<TwinCar>();

            Press(kb.wKey);
            yield return new WaitForSeconds(0.5f);
            Release(kb.wKey);
            // Let the released throttle reach FixedUpdate before resetting:
            // input is sampled in Update and applied in the next physics step.
            yield return new WaitForSeconds(0.2f);
            Assert.Greater(car.State.x, 0.0);

            PressAndRelease(kb.rKey);
            for (int i = 0; i < 5; i++) yield return new WaitForFixedUpdate();
            var s = car.State;
            Assert.AreEqual(0.0, s.x, "back at the origin and staying there");
            Assert.AreEqual(0.0, s.vx);
            Assert.AreEqual(1.0, s.soc);
        }
    }
}
