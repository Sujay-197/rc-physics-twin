// Play Mode: the Campus scene (RcTwin > Build Campus Scene), the fleet sim's
// road network at 1/10 that sim_py/campus.py trains on. The car starts at the
// hub stop, S3: node 33, ISO (-0.137, -2.565), on a straight 4.2 m run toward
// -x (campus/campus_network.json).
using System.Collections;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;

namespace RcTwin.Tests
{
    public class CampusDriveTests : InputTestFixture
    {
        private const string Asphalt = "Campus asphalt";
        private const double HomeX = -0.137349153, HomeY = -2.5646368;
        // Edge 22's heading: 4.6 mm of lateral offset over 1.4 m in the fleet sim's graph.
        private const double HomeYaw = 3.1382891505603703;
        private TwinCar _car;
        private Keyboard _kb;

        private IEnumerator Load()
        {
            _kb = InputSystem.AddDevice<Keyboard>();
            SceneManager.LoadScene("Campus");
            yield return null;
            yield return null;
            _car = Object.FindFirstObjectByType<TwinCar>();
            Assert.IsNotNull(_car);
        }

        private static string GroundUnder(Vector3 p)
        {
            return Physics.Raycast(new Vector3(p.x, 10f, p.z), Vector3.down, out RaycastHit hit, 20f)
                ? hit.collider.name : "nothing";
        }

        [UnityTest]
        public IEnumerator EveryStopIsOnTheAsphalt()
        {
            yield return Load();
            var stops = GameObject.Find("Bus stops");
            Assert.IsNotNull(stops);
            Assert.AreEqual(10, stops.transform.childCount, "the fleet sim's ten stops");
            foreach (Transform s in stops.transform)
                Assert.AreEqual(Asphalt, GroundUnder(s.position), s.name);
        }

        [UnityTest]
        public IEnumerator TheCarStartsAtTheHubAndDrivesDownTheRoad()
        {
            yield return Load();
            Assert.That(_car.State.x, Is.EqualTo(HomeX).Within(1e-4));
            Assert.That(_car.State.y, Is.EqualTo(HomeY).Within(1e-4));
            Assert.That(_car.State.yaw, Is.EqualTo(HomeYaw).Within(1e-5), "facing -x, along edge 22");
            Assert.AreEqual(Asphalt, GroundUnder(_car.transform.position));

            Press(_kb.wKey);
            yield return new WaitForSeconds(0.6f);
            Release(_kb.wKey);
            yield return null;
            Assert.Less(_car.State.x, HomeX - 0.3, "it drove toward -x");
            Assert.That(_car.State.y, Is.EqualTo(HomeY).Within(0.05), "straight down the lane");
            Assert.AreEqual(Asphalt, GroundUnder(_car.transform.position));
            Assert.IsFalse(_car.OffGround);

            PressAndRelease(_kb.rKey);
            yield return null;
            yield return null;
            Assert.That(_car.State.x, Is.EqualTo(HomeX).Within(1e-3), "R returns to S3");
            Assert.That(_car.State.y, Is.EqualTo(HomeY).Within(1e-3));
        }
    }
}
