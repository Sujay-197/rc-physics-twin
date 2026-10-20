// Autopilot for the twin on the campus: pick a stop and the car drives
// itself there on the Core's physics, from wherever it is stopped to a
// stop at the destination, along the route the fleet sim and the RL
// environment use (CampusRoutes).
//
// Each 10 ms control tick (the RL environment's) it measures the car along
// the route as the environment does (TwinObservation on a TwinRoad), asks
// its driver (an IDriver: by default the tuned pure pursuit) for a command,
// and hands it to TwinCar. Holding a drive key takes the car back; the trip
// ends there. A driver is the only thing to change for a new controller or a
// learned policy.
//
// Keys: P stops or resumes the trip. The panel picks the destination and speed.
// Command line (the player): -goto <stop> [-speed <m/s>] starts a trip;
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
        [Tooltip("Planned braking for the stop at the destination [m/s^2].")]
        public double stopDecel = 2.0;
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

        public const double ControlTick = 0.01;   // [s], sim_py/rc_twin_env.py TICK

        private TwinCar _car;
        private CampusRoutes _net;
        private TwinRoad _road;
        private TwinObservation _obs;
        private (double steer, double throttle) _last;
        private bool _first;
        private long _wallsAtStart;
        private LineRenderer _line;

        private float _savedFixedDelta = -1f;

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
        }

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
            if (State == Status.Driving || State == Status.Paused) End(Status.Idle, "Car reset; trip cancelled.");
        }

        /// <summary>The stop the car is at (nearest, within 2 m), or null.</summary>
        public string CurrentStop()
        {
            if (_net == null) return null;
            var (name, d) = _net.Nearest(_car.State.x, _car.State.y);
            return d <= 2.0 ? name : null;
        }

        /// <summary>Drive from the stop the car is at to `destination`.</summary>
        public bool Go(string destination)
        {
            if (_net == null) { Message = "No road network: rebuild the campus scene."; return false; }
            string origin = CurrentStop();
            if (origin == null) { Message = "Drive (or press R) to a stop first: trips run stop to stop."; return false; }
            if (!_net.Stops.ContainsKey(destination)) { Message = $"No stop {destination}."; return false; }
            if (_net.Stops[origin].node == _net.Stops[destination].node) { Message = $"Already at {destination}."; return false; }
            _road?.Dispose();
            _road = new TwinRoad(_net.Route(origin, destination), _net.RoadWidth);
            Driver ??= PurePursuitDriver.Tuned(RcTwinNative.DefaultParams(), targetSpeed);
            if (Driver is PurePursuitDriver pp) { pp.TargetSpeed = targetSpeed; pp.StopDecel = stopDecel; }
            Driver.Reset();
            _obs = new TwinObservation(Driver.History);
            _first = true;
            Origin = origin;
            Destination = destination;
            TripTime = 0.0;
            _wallsAtStart = _car.WallContacts;
            State = Status.Driving;
            Message = $"{origin} to {destination}, {_road.Length:0.0} m.";
            DrawRoute();
            return true;
        }

        public void Pause()
        {
            if (State == Status.Driving) { State = Status.Paused; _car.Command = (0.0, Brake()); Message = "Paused (P to resume)."; }
            else if (State == Status.Paused) { State = Status.Driving; Message = $"{Origin} to {Destination}."; }
        }

        private double Brake() => _car.State.vx > 0.05 ? -0.6 : 0.0;   // at rest a brake would reverse

        private void End(Status s, string message)
        {
            State = s;
            Message = message;
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
            var p = _car.Params;
            var (s, d) = _road.Locate(st.x, st.y);
            LastOffset = d;
            var frame = TwinObservation.Frame(_road, st, p, targetSpeed, s, d);
            if (_first) { _obs.Reset(frame); _first = false; }
            else _obs.Push(_last.steer, _last.throttle, frame);

            Remaining = _road.Length - s;
            TripTime += Time.fixedDeltaTime;
            TripWallContacts = _car.WallContacts - _wallsAtStart;
            if (Remaining < 0.25 && Math.Abs(st.vx) < 0.05)
            {
                End(Status.Arrived, $"Arrived at {Destination} in {TripTime:0.0} s" +
                                    (TripWallContacts > 0 ? $", {TripWallContacts} kerb contacts." : "."));
                return;
            }
            if (Math.Abs(d) > maxOffset) { End(Status.Failed, $"Left the route ({d:+0.00;-0.00} m off). Drive back or press R."); return; }
            if (TripTime > timeout) { End(Status.Failed, $"Gave up after {timeout:0} s."); return; }

            var a = Driver.Act(new DriveContext { Observation = _obs.Vector(), Remaining = Remaining });
            _last = (DriverMath.Clip(a.steer), DriverMath.Clip(a.throttle));
            _car.Command = _last;
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
            var r = new Rect(Screen.width - 330, 10, 320, 300);
            GUILayout.BeginArea(r, _box);
            GUILayout.Label("Autopilot", _title);
            GUILayout.Label($"{(Driver ?? PurePursuitDriver.Tuned(RcTwinNative.DefaultParams(), targetSpeed)).Name}, tuned " +
                            $"(policies/controllers.json). At stop: {CurrentStop() ?? "none"}.", _text);
            GUILayout.Label("Speed", _text);
            GUILayout.BeginHorizontal();
            foreach (double v in new[] { 1.0, 2.0, 3.0, 4.0 })
                if (GUILayout.Button($"{v:0} m/s", Math.Abs(targetSpeed - v) < 1e-6 ? _on : _btn) && State != Status.Driving)
                    targetSpeed = v;
            GUILayout.EndHorizontal();
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
                GUILayout.Label($"{Remaining:0.0} m to go, {_car.State.vx:0.00} m/s, {LastOffset * 100:+0;-0} cm off the line, " +
                                $"{TripWallContacts} kerb contacts", _text);
            }
            GUILayout.Label(Message, _text);
            GUILayout.EndArea();
        }
    }
}
