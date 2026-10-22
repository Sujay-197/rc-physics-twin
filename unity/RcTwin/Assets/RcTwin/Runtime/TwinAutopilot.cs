// Autopilot for the twin on the campus: pick a stop and the car drives
// itself there on the Core's physics, from wherever it is stopped to a
// stop at the destination, along the route the fleet sim and the RL
// environment use (CampusRoutes).
//
// Each 10 ms control tick it measures the car along the route as the RL
// environment's "stop" task does (TwinObservation on a TwinRoad: the frame
// plus the distance left), asks its driver for a command, and hands it to
// TwinCar. The trip ends as that task's episodes do: stopped within 0.3 m of
// the end, or failed past it, off the route or out of time. Holding a drive
// key takes the car back.
//
// A route can leave the stop the way the car came in. The car then turns
// round first, driven: TurnPlanner plans a multi-point turn of forward and
// reverse arcs inside the road (the walls the scene gives the Core, and the
// network's road width), off the main thread, and TurnDriver drives it
// through TwinCar.Command (reverse is the ESC's, ADR 0010). A leg that ends
// off its plan is replanned from where the car stopped. The trip itself
// starts at rest once the car faces along the route, as the RL "stop" task's
// episodes do. Only if no turn can be planned is the car turned at the stop
// instead (a reset), and the panel says so.
//
// Drivers (IDriver): pure pursuit tuned for this car (TunedForTwin), or a
// learned policy exported by tools/export_policy.py (PolicyDriver) when
// `policy` is set; the panel switches between them. A new controller is a
// new IDriver. Either is wrapped in KerbRecovery: pinned against a kerb, the
// car backs off and retries. Pure pursuit is also wrapped in CornerSpeed: it
// plans speed along the whole route at the trip's start and slows for each
// junction before it comes into view.
//
// Keys: P stops or resumes the trip. The panel picks the destination and speed.
// Command line (the player): -goto <stop> [-speed <m/s>] [-driver learned|pure_pursuit] starts a trip;
// -screenshot <png> [-at <s>] saves a frame that long after the start and quits.
using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

namespace RcTwin
{
    [DefaultExecutionOrder(-50)]   // before TwinCar's step, so each step uses this tick's command
    [RequireComponent(typeof(TwinCar))]
    public class TwinAutopilot : MonoBehaviour
    {
        public enum Status { Idle, Driving, Paused, Arrived, Manual, Failed }

        [Tooltip("campus/campus_network.json, copied in by RcTwin > Build Campus Scene.")]
        public TextAsset network;
        [Tooltip("Cruise speed [m/s]. The tuned gains are for 4 m/s; lower is gentler.")]
        public double targetSpeed = 2.0;
        [Tooltip("A learned policy (tools/export_policy.py JSON) trained on the \"stop\" task; empty for pure pursuit only.")]
        public TextAsset policy;
        [Tooltip("Drive with the learned policy when there is one.")]
        public bool useLearned = true;
        [Tooltip("Back off a kerb the car is pinned against and retry (KerbRecovery).")]
        public bool kerbRecovery = true;
        [Tooltip("Pure pursuit slows for the route's corners before they come into view (CornerSpeed).")]
        public bool cornerSpeed = true;
        [Tooltip("Give up when further than this off the centreline [m].")]
        public double maxOffset = 1.2;
        [Tooltip("Give up after this long on one trip [s].")]
        public double timeout = 90.0;
        [Tooltip("Unlit material for the route line.")]
        public Material routeMaterial;
        public bool showPanel = true;

        public Status State { get; private set; } = Status.Idle;
        public string Origin { get; private set; }
        public string Destination { get; private set; }
        public string Message { get; private set; } = "Pick a destination.";
        public IDriver Driver { get; set; }
        public CampusRoutes Routes => _net;
        public double Remaining { get; private set; } = double.NaN;
        public double TripTime { get; private set; }
        public long TripWallContacts { get; private set; }
        public double LastOffset { get; private set; }
        /// <summary>The turn being driven before the trip, or the last one; null if none was needed.</summary>
        public TurnPlan Turn { get; private set; }
        /// <summary>What the car did to turn round for this trip, for the panel; null if it did not have to.</summary>
        public string TurnReport { get; private set; }
        /// <summary>Planning or driving the turn: the trip proper has not started.</summary>
        public bool TurningRound => _planning != null || _turnDriver != null;
        /// <summary>Time spent turning round [s], and how often the turn was replanned.</summary>
        public double TurnTime { get; private set; }
        public int TurnReplans { get; private set; }

        public const double ControlTick = 0.01;   // [s], sim_py/rc_twin_env.py TICK
        public const double StopTolerance = 0.3, StopSpeed = 0.1;   // rc_twin_env.py STOP_TOLERANCE_M, STOP_SPEED
        /// <summary>Facing further than this from the route's way out of the stop, the car is turned round.</summary>
        public const double TurnAroundRad = 1.0471975511965976;   // 60 degrees
        /// <summary>The route driver takes over from a turn ending within these of the
        /// route's centreline [m] and heading [rad].</summary>
        public const double HandOverOffset = 0.15, HandOverHeading = 0.26;
        /// <summary>A leg ending further than these from its planned end [m, rad] is replanned.</summary>
        public const double LegOffPlan = 0.08, LegOffPlanHeading = 0.15;
        public const int MaxReplans = 4;

        private TwinCar _car;
        private CampusRoutes _net;
        private TwinRoad _road;
        private TwinObservation _obs;
        private (double steer, double throttle) _last;
        private bool _first;
        private long _wallsAtStart;
        private LineRenderer _line;
        private PolicyDriver _learned;
        private string _learnedError;

        private float _savedFixedDelta = -1f;

        private double[] _route;
        private List<(double[] xy, bool closed)> _walls;
        private TurnPlanner _planner;
        private System.Threading.Tasks.Task<TurnPlan> _planning;
        private TurnDriver _turnDriver;
        private bool _resetting;
        private long _wallsCarried;   // kerb contacts before a reset this trip

        private void Awake()
        {
            _car = GetComponent<TwinCar>();
            if (network != null) _net = new CampusRoutes(network.text);
            if (Math.Abs(Time.fixedDeltaTime - ControlTick) > 1e-6)
            {
                _savedFixedDelta = Time.fixedDeltaTime;   // put back on the way out: the step is global
                Debug.Log($"TwinAutopilot: physics step {Time.fixedDeltaTime * 1000:0} ms -> {ControlTick * 1000:0} ms, the RL environment's control tick.");
                Time.fixedDeltaTime = (float)ControlTick;
            }
            _car.Resetted += OnReset;
            if (policy != null)
            {
                try
                {
                    _learned = PolicyDriver.Load(policy.text, RcTwinNative.DefaultParams());
                    if (_learned.Task != "stop") { _learnedError = $"{_learned.PolicyName} was trained to follow roads, not to stop at stops."; _learned = null; }
                }
                catch (Exception e) { _learnedError = e.Message; _learned = null; }
                if (_learnedError != null) Debug.LogWarning($"TwinAutopilot: learned policy not used: {_learnedError}");
            }
        }

        /// <summary>The learned policy, if one loaded.</summary>
        public PolicyDriver Learned => _learned;

        private IDriver ChooseDriver()
        {
            IDriver d;
            if (useLearned && _learned != null)
            {
                targetSpeed = _learned.TargetSpeed;   // it was trained at one speed
                d = _learned;
            }
            else
            {
                d = PurePursuitDriver.TunedForTwin(RcTwinNative.DefaultParams(), targetSpeed);
                if (cornerSpeed)
                {
                    var road = _road;
                    var cs = new CornerSpeed(d, targetSpeed);
                    cs.Plan(s => road.Point(s).heading, road.Length);
                    d = cs;
                }
            }
            return kerbRecovery ? new KerbRecovery(d, targetSpeed) : d;
        }

        /// <summary>The corner speed plan's cap at the car this tick [m/s], or NaN.</summary>
        public double CornerCap => ((Driver as KerbRecovery)?.Inner ?? Driver) is CornerSpeed cs ? cs.Cap : double.NaN;

        /// <summary>Times this trip the car backed off a kerb it was pinned against.</summary>
        public int TripBackOffs => (Driver as KerbRecovery)?.BackOffs ?? 0;

        private void OnDestroy()
        {
            if (_savedFixedDelta > 0f) Time.fixedDeltaTime = _savedFixedDelta;
            if (_car != null) _car.Resetted -= OnReset;
            _road?.Dispose();
        }

        private void Start()
        {
            var args = Environment.GetCommandLineArgs();
            string Arg(string name)
            {
                int i = Array.IndexOf(args, name);
                return i >= 0 && i + 1 < args.Length ? args[i + 1] : null;
            }
            var inv = System.Globalization.CultureInfo.InvariantCulture;
            if (double.TryParse(Arg("-speed"), System.Globalization.NumberStyles.Float, inv, out double v) && v > 0) targetSpeed = v;
            if (Arg("-driver") is string drv) useLearned = drv == "learned";
            if (Arg("-goto") is string dest) StartCoroutine(GoSoon(dest));
            if (Arg("-screenshot") is string png)
            {
                Application.runInBackground = true;
                float at = float.TryParse(Arg("-at"), System.Globalization.NumberStyles.Float, inv, out float a) ? a : 3f;
                StartCoroutine(Shoot(png, at));
            }
        }

        private System.Collections.IEnumerator GoSoon(string dest)
        {
            yield return new WaitForFixedUpdate();   // TwinCar has placed itself at home
            if (!Go(dest)) Debug.LogWarning($"-goto {dest}: {Message}");
        }

        private System.Collections.IEnumerator Shoot(string png, float at)
        {
            float t0 = Time.time;
            while (Time.time - t0 < at) yield return null;
            yield return new WaitForEndOfFrame();
            Debug.Log($"screenshot at {Time.time - t0:0.0} s: {State}, {Message}");
            ScreenCapture.CaptureScreenshot(png);
            yield return null;
            yield return null;
            Application.Quit();
        }

        private void OnReset()
        {
            if (_resetting) return;
            if (State == Status.Driving || State == Status.Paused) End(Status.Idle, "Car reset; trip cancelled.");
        }

        /// <summary>The stop the car is at (nearest, within 2 m), or null.</summary>
        public string CurrentStop()
        {
            if (_net == null) return null;
            var (name, d) = _net.Nearest(_car.State.x, _car.State.y);
            return d <= 2.0 ? name : null;
        }

        /// <summary>Braking StopEarlyAt plans with: the stop plan's [m/s^2].</summary>
        public const double StopEarlyDecel = 1.2;

        /// <summary>End the trip early at `stop`, a stop on the route ahead (the
        /// campus shuttle stops on its way for a student). Only if there is room to
        /// brake for it: v^2 / (2 StopEarlyDecel) + 0.5 m. The trip's road and the
        /// corner plan are cut at the stop; the driving is unchanged.</summary>
        public bool StopEarlyAt(string stop)
        {
            if (State != Status.Driving || TurningRound || _net == null || _road == null) return false;
            if (!_net.Stops.TryGetValue(stop, out var target) || stop == Destination) return false;
            var route = _route;
            int n = route.Length / 3, hit = -1;
            double cum = 0.0, sStop = 0.0;
            for (int i = 1; i < n - 1 && hit < 0; i++)
            {
                cum += Math.Sqrt((route[3 * i] - route[3 * i - 3]) * (route[3 * i] - route[3 * i - 3]) +
                                 (route[3 * i + 1] - route[3 * i - 2]) * (route[3 * i + 1] - route[3 * i - 2]));
                double dx = route[3 * i] - target.nodeX, dy = route[3 * i + 1] - target.nodeY;
                if (dx * dx + dy * dy < 1e-6) { hit = i; sStop = cum; }
            }
            if (hit < 0) return false;
            var st = _car.State;
            var (s, _) = _road.Locate(st.x, st.y);
            double v = Math.Max(0.0, st.vx);
            if (sStop - s < v * v / (2.0 * StopEarlyDecel) + 0.5) return false;
            var prefix = new double[3 * (hit + 1)];
            Array.Copy(route, prefix, prefix.Length);
            var road = new TwinRoad(prefix, _net.RoadWidth);
            _road.Dispose();
            _road = road;
            _route = prefix;
            Destination = stop;
            if (((Driver as KerbRecovery)?.Inner ?? Driver) is CornerSpeed cs) cs.Plan(x => road.Point(x).heading, road.Length);
            DrawRoute();
            Message = $"{Origin} to {Destination}, stopping on the way.";
            return true;
        }

        /// <summary>Drive from the stop the car is at to `destination`.</summary>
        public bool Go(string destination)
        {
            if (_net == null) { Message = "No road network: rebuild the campus scene."; return false; }
            string origin = CurrentStop();
            if (origin == null) { Message = "Drive (or press R) to a stop first: trips run stop to stop."; return false; }
            if (!_net.Stops.ContainsKey(destination)) { Message = $"No stop {destination}."; return false; }
            if (_net.Stops[origin].node == _net.Stops[destination].node) { Message = $"Already at {destination}."; return false; }
            var route = _net.Route(origin, destination);
            var road = new TwinRoad(route, _net.RoadWidth);
            var (_, _, _, sh) = road.Point(_car.Params.lr + 0.05);
            bool turn = TurnPlanner.FacesAway(_car.State.yaw, sh, TurnAroundRad);
            if (turn && Math.Abs(_car.State.vx) > StopSpeed)
            {
                road.Dispose();
                Message = "Stop first: the route leaves the other way.";
                return false;
            }
            _road?.Dispose();
            _road = road;
            _route = route;
            Origin = origin;
            Destination = destination;
            Turn = null;
            TurnReport = null;
            TurnTime = 0.0;
            TurnReplans = 0;
            _turnDriver = null;
            _planning = null;
            _wallsAtStart = _car.WallContacts;
            _wallsCarried = 0;
            TripWallContacts = 0;
            TripTime = 0.0;
            Remaining = double.NaN;
            State = Status.Driving;
            DrawRoute();
            if (turn) PlanTurn();
            else StartTrip();
            return true;
        }

        // The trip proper, from rest at the stop: the RL "stop" task's episode.
        private void StartTrip()
        {
            Driver = ChooseDriver();
            Driver.Reset();
            _obs = new TwinObservation(Driver.History, stopTask: true);
            _first = true;
            TripTime = 0.0;
            Message = $"{Origin} to {Destination}, {_road.Length:0.0} m.";
        }

        // ------------------------------------------------------- turning round

        // Plan a turn from where the car is, off the main thread.
        private void PlanTurn()
        {
            if (_walls == null)
            {
                // The walls the Core has (TwinCar hands it every TwinWalls); a scene
                // without any falls back to the network's own kerbs and buildings.
                _walls = new List<(double[], bool)>();
                foreach (var set in FindObjectsByType<TwinWalls>(FindObjectsSortMode.InstanceID))
                    foreach (var w in set.walls)
                        if (w.points != null && w.points.Length >= 2) _walls.Add((TwinWalls.Flatten(w), w.closed));
                if (_walls.Count == 0) foreach (var w in _net.Walls) _walls.Add((w.xy, w.closed));
            }
            var planner = new TurnPlanner(_car.Params, _car.Footprint, _walls, _net.Centrelines(), 3, _net.RoadWidth);
            var st = _car.State;
            var route = _route;
            _planner = planner;
            _turnDriver = null;
            _planning = System.Threading.Tasks.Task.Run(() => planner.Plan(st.x, st.y, st.yaw, route));
            Message = $"{Origin} to {Destination}: the route leaves the other way; planning a turn.";
        }

        private void TurnTick(in RcTwinNative.State st)
        {
            TurnTime += Time.fixedDeltaTime;
            if (_planning != null)
            {
                _car.Command = (0.0, Brake());
                if (!_planning.IsCompleted) return;
                TurnPlan plan = _planning.Status == System.Threading.Tasks.TaskStatus.RanToCompletion ? _planning.Result : null;
                string why = _planning.IsFaulted ? _planning.Exception.GetBaseException().Message : "no turn fits in the road here";
                _planning = null;
                if (plan == null) { TurnAtStop(why); return; }
                Turn = plan;
                _turnDriver = new TurnDriver(plan, _car.Params.steerMax);
                TurnReport = $"Turning round: {plan.Describe()}" + (TurnReplans > 0 ? $" (replanned {TurnReplans}x)." : ".");
            }

            var drv = _turnDriver;
            var leg = drv.Current;
            _car.Command = drv.Act(st, Time.fixedDeltaTime);
            if (drv.Done)
            {
                if (_planner.Near(st.x, st.y, st.yaw, HandOverOffset, HandOverHeading))
                {
                    _turnDriver = null;
                    TurnReport = $"Turned round: {Turn.Describe()}, {TurnTime:0.0} s" +
                                 (TurnReplans > 0 ? $", replanned {TurnReplans}x." : ".");
                    Debug.Log($"TwinAutopilot: {TurnReport} {TripWallContacts} kerb contacts.");
                    StartTrip();
                }
                else Replan("ended off the route");
                return;
            }
            if (drv.LegEnded)
            {
                var end = leg.End;
                double off = Math.Sqrt((st.x - end.x) * (st.x - end.x) + (st.y - end.y) * (st.y - end.y));
                if (off > LegOffPlan || Math.Abs(TwinObservation.Wrap(st.yaw - end.yaw)) > LegOffPlanHeading) Replan("a leg ended off its plan");
            }
            else if (drv.Stalled > 1.0) Replan("stalled");
            else Message = $"{Origin} to {Destination}: turning round, leg {drv.Leg + 1} of {Turn.Legs.Count} " +
                           $"({(drv.Current.Dir > 0 ? "forward" : "reverse")}), {Math.Max(0.0, drv.LegRemaining):0.00} m left.";
        }

        private void Replan(string why)
        {
            if (++TurnReplans > MaxReplans)
            {
                End(Status.Failed, $"Could not turn round ({why}, {MaxReplans} replans). Drive back or press R.");
                return;
            }
            Debug.Log($"TwinAutopilot: turn {why}; replanning ({TurnReplans}).");
            PlanTurn();
        }

        // No turn fits: turn the car at the stop instead, as a bus turns at a
        // terminus, where the RL environment starts its episodes (on the
        // centreline, just past the stop).
        private void TurnAtStop(string why)
        {
            var (sx, sy, _, sh) = _road.Point(_car.Params.lr + 0.05);
            _wallsCarried = TripWallContacts;   // the reset clears the car's count
            _resetting = true;
            try { _car.ResetTo(sx, sy, sh); }
            finally { _resetting = false; }
            _wallsAtStart = _car.WallContacts;
            TurnReport = $"Turned round at the stop (not driven): {why}.";
            Debug.LogWarning($"TwinAutopilot: {TurnReport}");
            StartTrip();
        }

        public void Pause()
        {
            if (State == Status.Driving) { State = Status.Paused; _car.Command = (0.0, Brake()); Message = "Paused (P to resume)."; }
            else if (State == Status.Paused) { State = Status.Driving; Message = $"{Origin} to {Destination}."; }
        }

        // Against the motion; at rest a brake would reverse.
        private double Brake() => _car.State.vx > 0.05 ? -0.6 : _car.State.vx < -0.05 ? 0.6 : 0.0;

        private void End(Status s, string message)
        {
            State = s;
            Message = message;
            _planning = null;
            _turnDriver = null;
            _car.Command = s == Status.Manual ? ((double, double)?)null : (0.0, 0.0);
        }

        private void Update()
        {
            var kb = Keyboard.current;
            if (kb != null && kb.pKey.wasPressedThisFrame) Pause();
            if (State == Status.Paused) _car.Command = (0.0, Brake());
        }

        private void FixedUpdate()
        {
            if (State != Status.Driving) return;
            if (_car.ManualOverride) { End(Status.Manual, "You have the car (keys held). Pick a destination to hand it back."); return; }
            var st = _car.State;
            TripWallContacts = _wallsCarried + _car.WallContacts - _wallsAtStart;
            if (TurningRound) { TurnTick(st); return; }
            var p = _car.Params;
            var (s, d) = _road.Locate(st.x, st.y);
            LastOffset = d;
            var frame = TwinObservation.Frame(_road, st, p, targetSpeed, s, d, stopTask: true);
            if (_first) { _obs.Reset(frame); _first = false; }
            else _obs.Push(_last.steer, _last.throttle, frame);

            Remaining = _road.Length - s;
            TripTime += Time.fixedDeltaTime;
            if (Remaining <= StopTolerance && Math.Abs(st.vx) <= StopSpeed)
            {
                End(Status.Arrived, $"Arrived at {Destination} in {TripTime:0.0} s" +
                                    (TripWallContacts > 0 ? $", {TripWallContacts} kerb contacts" : "") +
                                    (TripBackOffs > 0 ? $", backed off the kerb {TripBackOffs}x." : "."));
                return;
            }
            if (Math.Abs(d) > maxOffset) { End(Status.Failed, $"Left the route ({d:+0.00;-0.00} m off). Drive back or press R."); return; }
            if (PastEnd(st) > StopTolerance) { End(Status.Failed, $"Ran past {Destination}."); return; }
            if (TripTime > timeout) { End(Status.Failed, $"Gave up after {timeout:0} s."); return; }

            var a = Driver.Act(new DriveContext { Observation = _obs.Vector(), Remaining = Remaining, S = s });
            _last = (DriverMath.Clip(a.steer), DriverMath.Clip(a.throttle));
            _car.Command = _last;
        }

        // How far beyond the end of the route, along its last heading [m].
        private double PastEnd(in RcTwinNative.State st)
        {
            var (x, y, _, h) = _road.Point(_road.Length);
            return (st.x - x) * Math.Cos(h) + (st.y - y) * Math.Sin(h);
        }

        private void DrawRoute()
        {
            if (_line == null)
            {
                var go = new GameObject("Autopilot route");
                _line = go.AddComponent<LineRenderer>();
                _line.useWorldSpace = true;
                _line.widthMultiplier = 0.05f;
                _line.numCornerVertices = 2;
                _line.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
                if (routeMaterial != null) _line.material = new Material(routeMaterial) { color = new Color(0.3f, 0.75f, 1f) };
            }
            var pts = new List<Vector3>();
            for (double s = 0.0; s < _road.Length; s += 0.25)
            {
                var (x, y, z, _) = _road.Point(s);
                pts.Add(TwinCar.ToUnity(x, y, z + 0.006));
            }
            var (ex, ey, ez, _) = _road.Point(_road.Length);
            pts.Add(TwinCar.ToUnity(ex, ey, ez + 0.006));
            _line.positionCount = pts.Count;
            _line.SetPositions(pts.ToArray());
            _line.enabled = true;
        }

        // ------------------------------------------------------------- panel

        private GUIStyle _box, _title, _text, _btn, _on;

        private void OnGUI()
        {
            if (!showPanel || _net == null) return;
            if (_box == null)
            {
                var bg = new Texture2D(1, 1) { hideFlags = HideFlags.HideAndDontSave };
                bg.SetPixel(0, 0, new Color(0.07f, 0.08f, 0.1f, 0.88f)); bg.Apply();
                var on = new Texture2D(1, 1) { hideFlags = HideFlags.HideAndDontSave };
                on.SetPixel(0, 0, new Color(0.2f, 0.42f, 0.78f)); on.Apply();
                _box = new GUIStyle { normal = { background = bg }, padding = new RectOffset(12, 12, 10, 10) };
                _title = new GUIStyle(GUI.skin.label) { fontSize = 16, fontStyle = FontStyle.Bold, normal = { textColor = Color.white } };
                _text = new GUIStyle(GUI.skin.label) { fontSize = 12, wordWrap = true, normal = { textColor = new Color(0.85f, 0.87f, 0.9f) } };
                _btn = new GUIStyle(GUI.skin.button) { fontSize = 12 };
                _on = new GUIStyle(_btn) { normal = { background = on, textColor = Color.white } };
            }
            var r = new Rect(Screen.width - 350, 10, 340, 380);
            GUILayout.BeginArea(r, _box);
            GUILayout.Label("Autopilot", _title);
            GUILayout.Label("Driver", _text);
            GUILayout.BeginHorizontal();
            bool learned = useLearned && _learned != null;
            if (GUILayout.Button("Pure pursuit (tuned)", learned ? _btn : _on) && State != Status.Driving) useLearned = false;
            GUI.enabled = _learned != null;
            if (GUILayout.Button(_learned != null ? $"Learned: {_learned.PolicyName}" : "Learned: none", learned ? _on : _btn)
                && State != Status.Driving) useLearned = true;
            GUI.enabled = true;
            GUILayout.EndHorizontal();
            if (_learnedError != null) GUILayout.Label(_learnedError, _text);
            GUILayout.Label(learned ? $"Speed {_learned.TargetSpeed:0} m/s (the policy's)" : "Speed", _text);
            if (!learned)
            {
                GUILayout.BeginHorizontal();
                foreach (double v in new[] { 1.0, 2.0, 3.0, 4.0 })
                    if (GUILayout.Button($"{v:0} m/s", Math.Abs(targetSpeed - v) < 1e-6 ? _on : _btn) && State != Status.Driving)
                        targetSpeed = v;
                GUILayout.EndHorizontal();
            }
            GUILayout.Label($"At stop: {CurrentStop() ?? "none"}.", _text);
            GUILayout.Label("Drive to", _text);
            var names = _net.StopNames();
            for (int row = 0; row < names.Count; row += 5)
            {
                GUILayout.BeginHorizontal();
                for (int k = row; k < Math.Min(names.Count, row + 5); k++)
                {
                    bool here = names[k] == CurrentStop();
                    GUI.enabled = !here && State != Status.Driving;
                    if (GUILayout.Button(names[k], names[k] == Destination && State == Status.Driving ? _on : _btn)) Go(names[k]);
                    GUI.enabled = true;
                }
                GUILayout.EndHorizontal();
            }
            if (State == Status.Driving || State == Status.Paused)
            {
                if (GUILayout.Button(State == Status.Paused ? "Resume (P)" : "Pause (P)", _btn)) Pause();
                if (!TurningRound)
                    GUILayout.Label($"{Remaining:0.0} m to go, {_car.State.vx:0.00} m/s, {LastOffset * 100:+0;-0} cm off the line, " +
                                    $"{TripWallContacts} kerb contacts" +
                                    ((Driver as KerbRecovery)?.Backing == true ? ", backing off the kerb" :
                                     TripBackOffs > 0 ? $", {TripBackOffs} back-offs" : "") +
                                    (CornerCap < targetSpeed ? $", plan: {CornerCap:0.0} m/s" : ""), _text);
            }
            if (TurnReport != null) GUILayout.Label(TurnReport, _text);
            GUILayout.Label(Message, _text);
            GUILayout.EndArea();
        }
    }
}
