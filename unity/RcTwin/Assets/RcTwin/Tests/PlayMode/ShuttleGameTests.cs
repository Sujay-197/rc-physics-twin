// Play Mode: the campus shuttle game on the Campus scene, on the Unity twin's
// own physics (Docs/specs/2026-10-21-campus-shuttle-design.md).
using System.Collections;
using System.Linq;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;

namespace RcTwin.Tests
{
    public class ShuttleGameTests : UnityEngine.InputSystem.InputTestFixture
    {
        private ShuttleGame _game;
        private TwinCar _car;

        private IEnumerator Load()
        {
            ShuttleGame.RunInEditor = true;
            SceneManager.LoadScene("Campus");
            yield return null;
            yield return null;
            yield return new WaitForFixedUpdate();
            _game = Object.FindFirstObjectByType<ShuttleGame>();
            Assert.IsNotNull(_game, "the campus car runs the shuttle game");
            _car = _game.GetComponent<TwinCar>();
            _game.Spawner.RatePerSecond = 0.0;   // only the requests a test raises
        }

        [TearDown]
        public void Off() => ShuttleGame.RunInEditor = false;

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

        [UnityTest]
        public IEnumerator TheGameIsOffInTheEditorUnlessAsked()
        {
            ShuttleGame.RunInEditor = false;
            SceneManager.LoadScene("Campus");
            yield return null;
            yield return null;
            Assert.IsNull(Object.FindFirstObjectByType<ShuttleGame>(), "the autopilot demo and its tests keep the car");
        }

        [UnityTest]
        public IEnumerator TheShuttleServesTheCampus()
        {
            yield return Load();
            double soc0 = _car.State.soc;
            var a = _game.RaiseRequest("AB2", "S6");
            var b = _game.RaiseRequest("S1", "S3");
            yield return Fast(() => _game.Score.Delivered >= 2, 180f);
            Assert.GreaterOrEqual(_game.Score.Delivered, 2, _game.Brain.Status);
            Assert.AreEqual(0, _game.Score.Flats);
            foreach (var r in new[] { a, b })
            {
                Assert.AreEqual(RequestState.Delivered, r.State, r.ToString());
                Assert.IsFalse(double.IsNaN(r.PickedUpAt) || double.IsNaN(r.DroppedAt));
            }
            Assert.Less(_car.State.soc, soc0, "driving used the game battery");
        }

        [UnityTest]
        public IEnumerator ALowBatteryShuttleChargesAtAB1()
        {
            yield return Load();
            _car.SetCharge(0.15);
            _game.RaiseRequest("S6", "AB4");
            yield return Fast(() => _game.Brain.Mode == ShuttleMode.Charging, 120f);
            Assert.AreEqual(ShuttleMode.Charging, _game.Brain.Mode, _game.Brain.Status);
            Assert.AreEqual("AB1", _game.AtStop);
            double top = 0.0;
            yield return Fast(() =>
            {
                top = System.Math.Max(top, _car.State.soc);
                return _game.Brain.Mode != ShuttleMode.Charging;
            }, 120f);
            Assert.AreNotEqual(ShuttleMode.Charging, _game.Brain.Mode, "it finished charging");
            Assert.GreaterOrEqual(top, 0.95 - 1e-6, "charged to the set level before leaving");
            Assert.AreEqual(0, _game.Score.Flats);
        }

        [UnityTest]
        public IEnumerator StudentsAppearAtTheirStopAndLeaveWhenTheyBoard()
        {
            yield return Load();
            var r = _game.RaiseRequest("AB2", "S6");
            yield return null;
            var fig = Object.FindObjectsByType<StudentFigure>(FindObjectsSortMode.None).FirstOrDefault(f => f.Request == r);
            Assert.IsNotNull(fig, "a figure waits at AB2");
            Assert.IsEmpty(fig.GetComponentsInChildren<Collider>(), "figures never touch the physics");
            yield return Fast(() => r.State == RequestState.Riding, 120f);
            yield return new WaitForSeconds(3.5f);   // boarding walks take at most 3 s
            Assert.IsTrue(fig == null, "it walked to the door and boarded");
        }
    }
}
