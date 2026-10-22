// The campus shuttle game (Docs/specs/2026-10-21-campus-shuttle-design.md):
// students raise ride requests at stops, and the twin's car, as a shuttle,
// serves them by itself. This joins the game's rules (ShuttleBrain,
// Dispatcher, RequestSpawner, ShuttleScore: plain C#) to the scene: legs are
// TwinAutopilot trips, the battery is TwinCar's (a small game pack), and
// students are StudentFigures.
//
// Game mode: in the player the game runs unless the command line has -goto
// (the one-trip autopilot demo). In the editor it runs only when
// RunInEditor is set (tests), so the autopilot's own tests keep the car.
// It attaches itself to the scene's autopilot after a scene loads.
using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.SceneManagement;

namespace RcTwin
{
    [RequireComponent(typeof(TwinCar))]
    [RequireComponent(typeof(TwinAutopilot))]
    public sealed class ShuttleGame : MonoBehaviour, IShuttleWorld
    {
        /// <summary>Run the game in the editor too (Play Mode tests).</summary>
        public static bool RunInEditor;

        public int seed = 1;

        public ShuttleSettings Settings { get; } = new ShuttleSettings();
        public ShuttleBrain Brain { get; private set; }
        public RequestSpawner Spawner { get; private set; }
        public ShuttleScore Score { get; private set; }
        public Dispatcher Dispatcher { get; private set; }
        public CampusRoutes Routes => _pilot != null ? _pilot.Routes : null;
        /// <summary>Requests not yet picked up: waiting, or assigned to the shuttle.</summary>
        public IReadOnlyList<RideRequest> Waiting =>
            Dispatcher == null ? new List<RideRequest>()
                : Dispatcher.Open.Where(r => r.State == RequestState.Waiting || r.State == RequestState.Assigned).ToList();

        private TwinCar _car;
        private TwinAutopilot _pilot;
        private RouteTravelModel _travel;
        private double _now;
        private bool _charging;
        private double _soc;              // the charge before a reset (ResetTo fills the pack)
        private string _legFrom, _legTo;
        private long _strikes;
        private readonly Dictionary<RideRequest, StudentFigure> _figures = new Dictionary<RideRequest, StudentFigure>();
        private System.Random _look;

        // ----------------------------------------------------------- game mode

        public static bool ShouldRun() =>
            Application.isEditor ? RunInEditor : Array.IndexOf(Environment.GetCommandLineArgs(), "-goto") < 0;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Hook()
        {
            SceneManager.sceneLoaded -= Attach;
            SceneManager.sceneLoaded += Attach;
        }

        private static void Attach(Scene scene, LoadSceneMode mode)
        {
            if (!ShouldRun()) return;
            foreach (var pilot in FindObjectsByType<TwinAutopilot>(FindObjectsSortMode.None))
                if (pilot.network != null && pilot.GetComponent<ShuttleGame>() == null)
                    pilot.gameObject.AddComponent<ShuttleGame>();
        }

        // --------------------------------------------------------------- setup

        private void Start()
        {
            _car = GetComponent<TwinCar>();
            _pilot = GetComponent<TwinAutopilot>();
            if (_pilot.Routes == null) { enabled = false; return; }
            _pilot.targetSpeed = Settings.Cruise;
            _pilot.useLearned = false;
            _travel = new RouteTravelModel(_pilot.Routes, Settings.Cruise);
            var p = _car.Params;
            double ah = BatteryPolicy.GamePackCapacityAh(Settings, BatteryPolicy.SeedWhPerMetre, p.packNominalV);
            _car.SetPackCapacity(ah);
            double seedSpm = BatteryPolicy.SeedWhPerMetre / (ah * p.packNominalV);
            Score = new ShuttleScore(Settings);
            Brain = new ShuttleBrain(this, Settings, _travel, Score, seedSpm);
            Dispatcher = new Dispatcher(new[] { Brain });
            var stops = _pilot.Routes.Stops.Values.OrderBy(s => s.name, StringComparer.Ordinal)
                .Select(s => (s.name, s.node, s.weight)).ToList();
            Spawner = new RequestSpawner(stops, seed) { RatePerSecond = 1.0 / Settings.RequestInterval };
            Brain.Boarded += OnBoarded;
            Brain.Alighted += OnAlighted;
            _look = new System.Random(seed);
            _car.showHud = false;
            _pilot.showPanel = false;
            _strikes = _car.KerbStrikes;
            _soc = _car.State.soc;
            _car.Resetted += KeepCharge;
            if (GetComponent<ShuttleHud>() == null) gameObject.AddComponent<ShuttleHud>();
        }

        private void OnDestroy()
        {
            if (_car != null) _car.Resetted -= KeepCharge;
        }

        // A tow, a turn at a stop or R moves the shuttle; it never charges it.
        private void KeepCharge()
        {
            if (Math.Abs(_car.State.vx) < 0.05 && _car.State.soc != _soc) _car.SetCharge(_soc);
        }

        /// <summary>Re-size the game battery after BatteryMinutesOfDriving changed; the charge
        /// stays, and the shuttle's energy estimate scales with the pack.</summary>
        public void ResizeBattery()
        {
            double before = _car.Params.packCapacityAh;
            double ah = BatteryPolicy.GamePackCapacityAh(Settings, BatteryPolicy.SeedWhPerMetre, _car.Params.packNominalV);
            _car.SetPackCapacity(ah);
            Brain.Meter.Scale(before / ah);
        }

        /// <summary>A request the person watching raised.</summary>
        public RideRequest RaiseRequest(string origin, string destination)
        {
            var r = Spawner.Raise(origin, destination, _now);
            Offer(r);
            return r;
        }

        private void Offer(RideRequest r)
        {
            Dispatcher.Offer(r);
            var f = StudentFigure.Make(r, KerbSlot(r.Origin, WaitingCountAt(r.Origin)), Clothes());
            _figures[r] = f;
        }

        // ---------------------------------------------------------------- tick

        private void Update()
        {
            var kb = Keyboard.current;
            if (kb != null && kb.f1Key.wasPressedThisFrame)
            {
                _car.showHud = !_car.showHud;
                _pilot.showPanel = !_pilot.showPanel;
            }
        }

        private void FixedUpdate()
        {
            if (Brain == null) return;
            double dt = Time.fixedDeltaTime;
            _now += dt;
            if (_charging && Math.Abs(_car.State.vx) < 0.05 && _car.State.soc < 1.0)
                _car.SetCharge(Math.Min(1.0, _car.State.soc + BatteryPolicy.ChargeRatePerSecond(Settings) * dt));
            _soc = _car.State.soc;
            foreach (var r in Spawner.Tick(_now, dt)) Offer(r);
            Brain.Tick(dt);
            Dispatcher.Tick(_now, Score);
            long strikes = _car.KerbStrikes;
            for (long k = _strikes; k < strikes; k++) Score.OnKerbStrike();
            _strikes = strikes;
            foreach (var kv in _figures.Where(kv => kv.Key.State == RequestState.GaveUp).ToList())
            {
                if (kv.Value != null) kv.Value.WalkTo(kv.Value.transform.position + kv.Value.transform.right * 0.6f, vanish: true);
                _figures.Remove(kv.Key);
            }
        }

        // ---------------------------------------------------------- the world

        public string AtStop =>
            _pilot.State != TwinAutopilot.Status.Driving && _pilot.State != TwinAutopilot.Status.Paused &&
            Math.Abs(_car.State.vx) < 0.15 ? _pilot.CurrentStop() : null;

        public bool Driving => _pilot.State == TwinAutopilot.Status.Driving || _pilot.State == TwinAutopilot.Status.Paused;
        /// <summary>Failed, taken over by the keys (Manual), or cancelled by a car reset (Idle).</summary>
        public bool LegFailed => _pilot.State == TwinAutopilot.Status.Failed || _pilot.State == TwinAutopilot.Status.Manual ||
                                 _pilot.State == TwinAutopilot.Status.Idle;
        public string LegFrom => _legFrom;
        public double LegS
        {
            get
            {
                if (!Driving || _legFrom == null || double.IsNaN(_pilot.Remaining)) return 0.0;
                return Math.Max(0.0, _travel.Distance(_legFrom, _legTo) - _pilot.Remaining);
            }
        }
        public double Speed => _car.State.vx;
        public double Soc => _car.State.soc;
        public double Now => _now;

        public bool StartLeg(string stop)
        {
            string from = _pilot.CurrentStop();
            if (!_pilot.Go(stop)) return false;
            _legFrom = from;
            _legTo = stop;
            return true;
        }

        public bool StopEarlyAt(string stop)
        {
            if (!_pilot.StopEarlyAt(stop)) return false;
            _legTo = stop;
            return true;
        }

        public void Tow(string stop)
        {
            var net = _pilot.Routes;
            string toward = Brain != null && Brain.Plan.Stops.Count > 0 && Brain.Plan.Stops[0].Stop != stop
                ? Brain.Plan.Stops[0].Stop
                : net.StopNames().First(s => net.Stops[s].node != net.Stops[stop].node);
            var xyz = net.Route(stop, toward);
            _car.ResetTo(xyz[0], xyz[1], Math.Atan2(xyz[4] - xyz[1], xyz[3] - xyz[0]));
            _strikes = _car.KerbStrikes;   // a reset clears the car's count
        }

        public void SetCharging(bool on) => _charging = on;

        // ------------------------------------------------------------ figures

        private void OnBoarded(RideRequest r)
        {
            if (_figures.TryGetValue(r, out var f) && f != null) f.WalkTo(Door(), vanish: true, maxTime: 3f);
            _figures.Remove(r);
        }

        private void OnAlighted(RideRequest r)
        {
            var f = StudentFigure.Make(r, Door(), Clothes());
            f.WalkTo(KerbSlot(r.Destination, 0) + Away(r.Destination) * 0.5f, vanish: true);
        }

        private int WaitingCountAt(string stop) => _figures.Count(kv => kv.Key.Origin == stop && kv.Value != null);

        private Color Clothes()
        {
            var palette = new[]
            {
                new Color(0.17f, 0.29f, 0.55f), new Color(0.55f, 0.16f, 0.16f), new Color(0.2f, 0.45f, 0.3f),
                new Color(0.85f, 0.75f, 0.35f), new Color(0.35f, 0.35f, 0.38f), new Color(0.9f, 0.9f, 0.9f),
                new Color(0.45f, 0.3f, 0.55f),
            };
            return palette[_look.Next(palette.Length)];
        }

        // The shuttle's door: the left side, at the kerb.
        private Vector3 Door()
        {
            var t = _car.transform;
            return t.position - t.right * 0.2f;
        }

        // Where the i-th student waits at a stop: in its shelter (the campus scene's
        // "Stop <name> (...)" object, its post on the stop's side), in rows; without
        // the scene's stops, on the sidewalk left of the road leaving the stop.
        public Vector3 KerbSlot(string stop, int i)
        {
            if (StopFrame(stop) is (Transform frame, float postX))
            {
                float postSide = Mathf.Sign(postX);
                var local = new Vector3(postX + postSide * (0.03f * (i / 4)), 0.06f, -0.38f + 0.075f * (i % 4));
                var shelter = frame.TransformPoint(local);
                shelter.y = 0.06f;
                return shelter;
            }
            var net = _pilot.Routes;
            var s = net.Stops[stop];
            string toward = net.StopNames().First(x => net.Stops[x].node != s.node);
            var xyz = net.Route(stop, toward);
            double hx = xyz[3] - xyz[0], hy = xyz[4] - xyz[1], n = Math.Sqrt(hx * hx + hy * hy);
            hx /= n; hy /= n;
            double side = 0.5 * net.RoadWidth + 0.12;
            double along = 0.12 * (i % 6) - 0.3, back = 0.1 * (i / 6);
            double x = s.nodeX - hy * (side + back) + hx * along;
            double y = s.nodeY + hx * (side + back) + hy * along;
            var p = TwinCar.ToUnity(x, y, 0.0);
            p.y = 0.06f;   // on the sidewalk, at kerb height
            return p;
        }

        private readonly Dictionary<string, (Transform, float)?> _stopFrames = new Dictionary<string, (Transform, float)?>();

        // The scene's stop object and its post's sideways offset, if the scene has them.
        private (Transform, float)? StopFrame(string stop)
        {
            if (_stopFrames.TryGetValue(stop, out var f)) return f;
            f = null;
            var root = GameObject.Find("Bus stops");
            if (root != null)
                foreach (Transform c in root.transform)
                    if (c.name.StartsWith($"Stop {stop} ("))
                    {
                        var post = c.Find("Post");
                        if (post != null) f = (c, post.localPosition.x);
                    }
            _stopFrames[stop] = f;
            return f;
        }

        private Vector3 Away(string stop)
        {
            var slot = KerbSlot(stop, 0);
            var node = _pilot.Routes.Stops[stop];
            var road = TwinCar.ToUnity(node.nodeX, node.nodeY, 0.0);
            var d = slot - road;
            d.y = 0f;
            return d.normalized;
        }
    }
}
