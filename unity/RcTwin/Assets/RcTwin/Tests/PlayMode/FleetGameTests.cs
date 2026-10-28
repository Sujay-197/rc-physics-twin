// Play Mode: several buses on the Campus scene, on the Unity twin's own physics
// (Docs/specs/2026-10-25-shuttle-fleet-design.md, ADR 0012): serving the campus
// together without blocking or touching each other, queueing at a stop, the
// fleet growing and shrinking, and the camera modes.
using System.Collections;
using System.Linq;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;

namespace RcTwin.Tests
{
    public class FleetGameTests : UnityEngine.InputSystem.InputTestFixture
    {
        private ShuttleFleet _fleet;

        private IEnumerator Load(int buses)
        {
            ShuttleGame.RunInEditor = true;
            ShuttleFleet.BusesOverride = buses;
            SceneManager.LoadScene("Campus");
            yield return null;
            yield return null;
            yield return new WaitForFixedUpdate();
            _fleet = Object.FindFirstObjectByType<ShuttleFleet>();
            Assert.IsNotNull(_fleet, "the campus runs the shuttle game");
            Assert.AreEqual(buses, _fleet.Buses.Count);
            _fleet.Spawner.RatePerSecond = 0.0;   // only the requests a test asks for
        }

        [TearDown]
        public void Off()
        {
            ShuttleGame.RunInEditor = false;
            ShuttleFleet.BusesOverride = 0;
            Time.timeScale = 1f;
        }

        private static IEnumerator Fast(System.Func<bool> until, float gameSeconds)
        {
            float scale = Time.timeScale;
            Time.timeScale = 5f;
            try
            {
                float t0 = Time.time;
                while (!until() && Time.time - t0 < gameSeconds) yield return null;
            }
            finally { Time.timeScale = scale; }
        }

        private string Report() =>
            string.Join("; ", _fleet.Buses.Select(b => $"{b.Label}: {b.Brain.Status} ({b.Pilot.Message})")) +
            $"; delivered {_fleet.Score.Delivered}, gave up {_fleet.Score.GaveUp}, kerb strikes {_fleet.Score.KerbStrikes}, stuck {_fleet.StuckTows}, " +
            $"overlaps {_fleet.Overlaps}, longest hold {_fleet.LongestHold:0.0} s, made room {_fleet.MadeRoom}";

        [UnityTest]
        public IEnumerator ThreeBusesServeTheCampusTogether()
        {
            yield return Load(3);
            var spots = _fleet.Buses.Select(b => _fleet.Routes.Nearest(b.Car.State.x, b.Car.State.y).name).ToList();
            Assert.AreEqual(3, spots.Distinct().Count(), $"the buses start at different stops: {string.Join(", ", spots)}");
            _fleet.Spawner.RatePerSecond = 1.0 / 8.0;   // per bus, about the one-bus game's load (a request every 20 s)
            yield return Fast(() => false, 300f);
            Debug.Log($"FleetGameTests: {Report()}");
            Assert.GreaterOrEqual(_fleet.Score.Delivered, 20, Report());
            Assert.GreaterOrEqual(_fleet.Buses.Count(b => b.BusScore.Delivered > 0), 2, "more than one bus works: " + Report());
            Assert.AreEqual(0, _fleet.StuckTows, "no bus stuck behind another: " + Report());
            Assert.AreEqual(0, _fleet.Overlaps, "no two buses touched: " + Report());
            Assert.Less(_fleet.LongestHold, ShuttleFleet.StuckAfter, Report());
            Assert.AreEqual(0, _fleet.Score.Flats, Report());
            Assert.AreEqual(0, _fleet.Score.KerbStrikes, "every bus keeps to its lane: " + Report());
            // A five-minute battery: in five minutes every bus charges once, and a student
            // may give up meanwhile; most are served.
            Assert.LessOrEqual(5 * _fleet.Score.GaveUp, _fleet.Score.Delivered, Report());
        }

        [UnityTest]
        public IEnumerator TheSecondBusAtAStopWaitsBehindTheFirst()
        {
            yield return Load(2);
            var first = _fleet.Buses[1];
            var second = _fleet.Buses[0];
            // Both come to AB1 along the same road: from S1 (18 m) and from S3 (30 m).
            first.Tow("S1");
            second.Tow("S3");
            yield return new WaitForFixedUpdate();
            Assert.IsTrue(first.Brain.MakeRoom("AB1") && second.Brain.MakeRoom("AB1"));
            yield return Fast(() => _fleet.Buses.All(b => b.Brain.Mode == ShuttleMode.Idle && b.AtStop == "AB1"), 90f);
            Assert.IsTrue(_fleet.Buses.All(b => b.AtStop == "AB1"), Report());
            StringAssert.Contains("behind Bus 2", second.Pilot.Message, "it served the stop from behind: " + Report());
            float gap = Vector3.Distance(first.transform.position, second.transform.position);
            Assert.Greater(gap, 2f * (float)first.Car.Footprint.halfLength, "nose to tail, not on top of each other");
            Assert.AreEqual(0, _fleet.Overlaps, Report());
            Assert.AreEqual(0, _fleet.StuckTows, Report());
            Assert.AreEqual(0, second.Car.KerbStrikes, Report());
        }

        [UnityTest]
        public IEnumerator TheFleetGrowsAndShrinks()
        {
            yield return Load(2);
            _fleet.SetBuses(3);
            Assert.AreEqual(3, _fleet.Buses.Count);
            Assert.AreEqual("Bus 3", _fleet.Buses[2].Label);
            yield return new WaitForFixedUpdate();
            Assert.AreEqual(3, _fleet.Cameras.Buses);
            _fleet.Cameras.Follow(3);
            _fleet.SetBuses(2);
            Assert.IsTrue(_fleet.Buses[2].Brain.Retiring);
            yield return Fast(() => _fleet.Buses.Count == 2, 10f);
            Assert.AreEqual(2, _fleet.Buses.Count, "an idle bus leaves at once");
            Assert.AreEqual(CameraMode.Following(2), _fleet.Cameras.Current, "the camera follows the last bus left");
            Assert.AreEqual(2, Object.FindObjectsByType<ShuttleGame>(FindObjectsSortMode.None).Length);
        }

        [UnityTest]
        public IEnumerator TheCameraCyclesThroughEveryBusAndTheViews()
        {
            yield return Load(3);
            var cam = Camera.main.GetComponent<FleetCamera>();
            Assert.IsNotNull(cam, "the game drives the camera");
            Assert.IsFalse(Camera.main.GetComponent<ChaseCamera>().enabled);
            var kb = InputSystem.AddDevice<Keyboard>();
            Assert.AreEqual("Bus 1", _fleet.Cameras.Current.ToString());
            foreach (var want in new[] { "Bus 2", "Bus 3", "Route", "Fleet", "Overview", "Orbit", "Stop", "Bus 1" })
            {
                Press(kb.cKey);
                yield return null;
                Release(kb.cKey);
                yield return null;
                Assert.AreEqual(want, _fleet.Cameras.Current.ToString());
            }
            Press(kb.digit2Key);
            yield return null;
            Release(kb.digit2Key);
            yield return new WaitForSeconds(2.5f);
            var at = cam.transform.position;
            var b1 = _fleet.Buses[0].transform.position;
            var b2 = _fleet.Buses[1].transform.position;
            Assert.Less(Vector3.Distance(at, b2), 2.5f, "behind Bus 2");
            Assert.Less(Vector3.Distance(at, b2), Vector3.Distance(at, b1));

            _fleet.Cameras.Set(new CameraMode(CameraKind.Overview));
            yield return new WaitForSeconds(2.5f);
            Assert.Greater(cam.transform.position.y, 10f, "high above the campus");
            Assert.Less(cam.transform.forward.y, -0.95f, "looking straight down");

            _fleet.Cameras.Set(new CameraMode(CameraKind.Stop, stop: "S3"));
            yield return null;
            Assert.AreEqual("S3", cam.Stop);
            Press(kb.rightBracketKey);
            yield return null;
            Release(kb.rightBracketKey);
            yield return null;
            Assert.AreEqual(CameraKind.Stop, _fleet.Cameras.Current.Kind);
            Assert.AreEqual("S4", _fleet.Cameras.Current.Stop, "] steps to the next stop");
        }
    }
}
