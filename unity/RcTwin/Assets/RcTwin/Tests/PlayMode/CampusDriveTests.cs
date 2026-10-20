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
        public IEnumerator TheAutopilotDrivesToAStopAndStopsThere()
        {
            // S3 to AB1 (about 25 m, two 90-degree corners) at 2 m/s with the
            // tuned pure pursuit, on the Unity twin's own physics.
            yield return Load();
            var pilot = _car.GetComponent<TwinAutopilot>();
            Assert.IsNotNull(pilot, "the campus car has an autopilot");
            Assert.AreEqual("S3", pilot.CurrentStop());
            pilot.targetSpeed = 2.0;
            Assert.IsTrue(pilot.Go("AB1"), pilot.Message);
            float scale = Time.timeScale;
            Time.timeScale = 5f;
            try
            {
                float t0 = Time.time;
                while (pilot.State == TwinAutopilot.Status.Driving && Time.time - t0 < 60f) yield return null;
            }
            finally { Time.timeScale = scale; }
            Assert.AreEqual(TwinAutopilot.Status.Arrived, pilot.State, pilot.Message);
            Assert.AreEqual("AB1", pilot.CurrentStop());
            Assert.Less(System.Math.Abs(_car.State.vx), 0.05, "stopped");
            Assert.Less(pilot.Remaining, 0.25, "at the stop");
            Assert.Less(pilot.TripTime, 30.0);
            Assert.AreEqual(0, pilot.TripWallContacts, "no kerb strikes at 2 m/s");
        }

        [UnityTest]
        public IEnumerator HoldingAKeyTakesTheCarBack()
        {
            yield return Load();
            var pilot = _car.GetComponent<TwinAutopilot>();
            Assert.IsTrue(pilot.Go("S1"), pilot.Message);
            yield return new WaitForSeconds(0.3f);
            Press(_kb.sKey);
            yield return null;                       // TwinCar reads the key in Update
            yield return new WaitForFixedUpdate();   // the autopilot sees it on its next tick
            yield return new WaitForFixedUpdate();
            Assert.AreEqual(TwinAutopilot.Status.Manual, pilot.State);
            Assert.IsNull(_car.Command, "the keyboard has the car");
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
        public IEnumerator TheKerbKeepsTheCarOnTheAsphalt()
        {
            // Edge 13 runs along y = -2.56 toward -x; its left kerb is at
            // y = -2.56 + 0.7045. Aim 29 degrees into it at 1.5 m/s and hold
            // the throttle: the kerb is a wall (ADR 0009), not a step.
            yield return Load();
            Assert.Greater(_car.WallSegments, 100u, "kerbs and buildings reached the Core");
            _car.ResetTo(-2.9, -2.56, System.Math.PI - 0.5, 1.5);
            Press(_kb.wKey);
            yield return new WaitForSeconds(1.0f);
            Release(_kb.wKey);
            yield return null;
            Assert.Greater(_car.WallContacts, 0, "it hit the kerb");
            Assert.Less(_car.State.y, -2.56 + 0.7045, "the CoG stayed on the road side");
            Assert.AreEqual(Asphalt, GroundUnder(_car.transform.position));
            Assert.That(_car.State.z, Is.EqualTo(0.0).Within(0.01), "not up on the sidewalk");
            Assert.IsFalse(_car.OffGround);
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
