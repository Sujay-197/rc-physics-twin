// Play Mode: what the campus shuttle game needs from the car and the
// autopilot (Docs/specs/2026-10-21-campus-shuttle-design.md): setting the
// charge at the charger, a small game battery, kerb strikes counted once per
// scrape, and ending a trip early at a stop on the way.
using System.Collections;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;

namespace RcTwin.Tests
{
    public class ShuttleCarHookTests : UnityEngine.InputSystem.InputTestFixture
    {
        private TwinCar _car;
        private TwinAutopilot _pilot;

        private IEnumerator Load()
        {
            SceneManager.LoadScene("Campus");
            yield return null;
            yield return null;
            _car = Object.FindFirstObjectByType<TwinCar>();
            _pilot = _car.GetComponent<TwinAutopilot>();
            Assert.IsNotNull(_pilot);
        }

        // Place the car at a stop's road node, facing along the route to `toward`.
        private void PlaceAt(string stop, string toward)
        {
            var xyz = _pilot.Routes.Route(stop, toward);
            _car.ResetTo(xyz[0], xyz[1], System.Math.Atan2(xyz[4] - xyz[1], xyz[3] - xyz[0]));
        }

        private static IEnumerator Fast(System.Func<bool> until, float seconds)
        {
            float scale = Time.timeScale;
            Time.timeScale = 5f;
            try
            {
                float t0 = Time.time;
                while (!until() && Time.time - t0 < seconds) yield return null;
            }
            finally { Time.timeScale = scale; }
        }

        [UnityTest]
        public IEnumerator AChargeCanBeSetWhileParked()
        {
            yield return Load();
            yield return new WaitForFixedUpdate();
            var before = _car.State;
            _car.SetCharge(0.3);
            var after = _car.State;
            Assert.AreEqual(0.3, after.soc, 1e-9);
            Assert.AreEqual(before.x, after.x, 1e-9);
            Assert.AreEqual(before.y, after.y, 1e-9);
            Assert.AreEqual(before.yaw, after.yaw, 1e-9);
            _car.SetCharge(1.7);
            Assert.AreEqual(1.0, _car.State.soc, 1e-12, "clamped");
        }

        [UnityTest]
        public IEnumerator ASmallPackDrainsFast()
        {
            yield return Load();
            _car.SetPackCapacity(0.0157);
            Assert.AreEqual(0.0157, _car.Params.packCapacityAh, 1e-12);
            double soc0 = _car.State.soc;
            _pilot.targetSpeed = 2.0;
            Assert.IsTrue(_pilot.Go("AB1"), _pilot.Message);
            yield return Fast(() => _pilot.State != TwinAutopilot.Status.Driving, 60f);
            Assert.AreEqual(TwinAutopilot.Status.Arrived, _pilot.State, _pilot.Message);
            Assert.Greater(soc0 - _car.State.soc, 0.03, "the stock pack loses under 0.1 % on this trip");
        }

        [UnityTest]
        public IEnumerator AKerbStrikeIsCountedOnce()
        {
            // As CampusDriveTests.TheKerbKeepsTheCarOnTheAsphalt: 29 degrees into the
            // left kerb of edge 13 at 1.5 m/s, throttle held.
            yield return Load();
            _car.ResetTo(-2.9, -2.56, System.Math.PI - 0.5, 1.5);
            _car.Command = (0.0, 1.0);
            yield return new WaitForSeconds(1.0f);
            _car.Command = (0.0, 0.0);
            Assert.Greater(_car.WallContacts, 0, "it hit the kerb");
            Assert.AreEqual(1, _car.KerbStrikes, "one scrape, one strike");
        }

        [UnityTest]
        public IEnumerator ATripCanEndEarlyAtAStopOnTheWay()
        {
            yield return Load();
            PlaceAt("AB4", "AB1");
            yield return new WaitForFixedUpdate();
            Assert.AreEqual("AB4", _pilot.CurrentStop());
            _pilot.targetSpeed = 2.0;
            Assert.IsTrue(_pilot.Go("AB1"), _pilot.Message);
            yield return new WaitForSeconds(1.0f);
            Assert.AreEqual(TwinAutopilot.Status.Driving, _pilot.State);
            Assert.IsFalse(_pilot.StopEarlyAt("S6"), "S6 is not on the way");
            Assert.IsTrue(_pilot.StopEarlyAt("S3"), _pilot.Message);
            Assert.AreEqual("S3", _pilot.Destination);
            yield return Fast(() => _pilot.State != TwinAutopilot.Status.Driving, 60f);
            Assert.AreEqual(TwinAutopilot.Status.Arrived, _pilot.State, _pilot.Message);
            Assert.AreEqual("S3", _pilot.CurrentStop());
        }

        [UnityTest]
        public IEnumerator TooLateToBrakeIsRefused()
        {
            yield return Load();
            PlaceAt("AB4", "AB1");
            yield return new WaitForFixedUpdate();
            _pilot.targetSpeed = 2.0;
            Assert.IsTrue(_pilot.Go("AB1"), _pilot.Message);
            var s3 = _pilot.Routes.Stops["S3"];
            yield return Fast(() =>
            {
                var st = _car.State;
                return System.Math.Sqrt((st.x - s3.nodeX) * (st.x - s3.nodeX) + (st.y - s3.nodeY) * (st.y - s3.nodeY)) < 1.2;
            }, 60f);
            Assert.AreEqual(TwinAutopilot.Status.Driving, _pilot.State);
            Assert.Greater(_car.State.vx, 1.0, "at speed past S3");
            Assert.IsFalse(_pilot.StopEarlyAt("S3"), "1.2 m is too late to stop from that speed (about 2 m needed)");
            Assert.AreEqual("AB1", _pilot.Destination);
        }
    }
}
