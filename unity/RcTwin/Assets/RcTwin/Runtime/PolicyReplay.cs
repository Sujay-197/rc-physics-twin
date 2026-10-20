// The policy replay (RcTwin > Build Replay Scene): every recorded policy
// drives the same campus trip at once, one coloured car each, from
// tools/record_drives.py's drives.json. The drives are the Core's own states
// from the evaluation harness, sampled at 25 Hz and interpolated here, so what
// is on screen is what the study scored, not a re-simulation.
//
// Pick a route with the From and To stop buttons (every pair is there when
// the recording was made with --pairs), or step through trips in order.
//
// Keys: Space play/pause, Left/Right trip, V reverse the route, Up/Down condition, C camera,
// 1-9 follow a policy, [ and ] speed, R restart, A auto-advance, G trip
// grid, L labels, H hide the panels, F5 reload the file, Esc quit (player).
//
// Command line (the player): -drives <path>, -from <stop> -to <stop>, and for a still picture
// -condition <train|held_out|perturbed> -trip <1-based> -camera
// <pack|chase|route|campus> -follow <1-based> -at <seconds> -screenshot <png>,
// which pauses at that time, saves the frame and quits.
using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.Rendering;

namespace RcTwin
{
    public class PolicyReplay : MonoBehaviour
    {
        [Tooltip("Lit material the cars are tinted from.")]
        public Material bodyMaterial;
        [Tooltip("Unlit material the route and trails are tinted from.")]
        public Material lineMaterial;

        private enum CamMode { Pack, Chase, Route, Campus }

        private static readonly string[] ConditionNames =
            { "train", "Training cars", "held_out", "Held-out cars", "perturbed", "Mid-trip change" };

        private static readonly Color[] Palette =
        {
            Hex(0x9AA9BA), Hex(0x4CC3B5), Hex(0xA88BEB), Hex(0xF0B03F), Hex(0xF2643D),
            Hex(0xE35DB0), Hex(0x73C95B), Hex(0x4C9BF0), Hex(0xF5E04A), Hex(0xB8E36A),
        };

        private static readonly Color Finished = Hex(0x4FC46A), Bumped = Hex(0xE8B34A),
            OffRoad = Hex(0xE5534B), TimedOut = Hex(0x8C7FA8);

        private sealed class Ghost
        {
            public GameObject root;
            public Transform[] wheels = new Transform[4];
            public LineRenderer trail;
            public TextMesh label;
            public Color colour;
            public ReplayData.Drive drive;
            public Vector3[] path;
            public int trailCount = -1;
            public readonly List<GameObject> hits = new List<GameObject>();
        }

        private ReplayData _d;
        private string _path, _error;
        private int _cond, _trip, _follow;
        private float _t, _rate = 1f, _hold;
        private bool _playing = true, _auto = true, _panels = true, _grid = true, _labels = true;
        private bool[] _visible;
        private Ghost[] _ghosts = Array.Empty<Ghost>();
        private readonly List<GameObject> _marks = new List<GameObject>();
        private LineRenderer _route;
        private Camera _cam;
        private CamMode _mode = CamMode.Pack;
        private float _orbit, _zoom = 1f;
        private Bounds _routeBounds, _campusBounds;
        private Font _font;
        private Vector2 _gridScroll;
        private readonly List<Rect> _guiRects = new List<Rect>();

        private static readonly float[] Rates = { 0.25f, 0.5f, 1f, 2f, 4f };

        // ------------------------------------------------------------ setup

        private void Start()
        {
            _cam = Camera.main;
            if (_cam != null)
            {
                _cam.nearClipPlane = 0.02f;
                _cam.farClipPlane = 400f;
            }
            _font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            var asphalt = GameObject.Find("Campus asphalt");
            _campusBounds = asphalt != null && asphalt.TryGetComponent<Renderer>(out var r)
                ? r.bounds : new Bounds(Vector3.zero, new Vector3(60f, 1f, 40f));
            // The campus's stop names, turned to the camera so none reads mirrored.
            var stops = GameObject.Find("Bus stops");
            if (stops != null) _stopLabels.AddRange(stops.GetComponentsInChildren<TextMesh>());
            Load();
            CommandLine(Environment.GetCommandLineArgs());
        }

        private readonly List<TextMesh> _stopLabels = new List<TextMesh>();
        private string _shot;
        private float _shotAt;

        private void CommandLine(string[] args)
        {
            string Arg(string name)
            {
                int i = Array.IndexOf(args, name);
                return i >= 0 && i + 1 < args.Length ? args[i + 1] : null;
            }
            var inv = System.Globalization.CultureInfo.InvariantCulture;
            if (_d == null) return;
            if (Arg("-condition") is string c && _d.conditions.IndexOf(c) >= 0) SetCondition(_d.conditions.IndexOf(c));
            if (int.TryParse(Arg("-trip"), out int trip)) ShowTrip(trip - 1);
            if (Arg("-from") is string from && Arg("-to") is string to) PickRoute(from, to);
            if (int.TryParse(Arg("-follow"), out int f) && f >= 1 && f <= _ghosts.Length) Follow(f - 1);
            if (Enum.TryParse(Arg("-camera") ?? "", true, out CamMode m)) _mode = m;
            _shot = Arg("-screenshot");
            if (_shot != null)
            {
                Application.runInBackground = true;   // a player out of focus would pause and never shoot
                _auto = false;
                _shotAt = float.TryParse(Arg("-at"), System.Globalization.NumberStyles.Float, inv, out float at) ? at : 2f;
                StartCoroutine(Shoot());
            }
        }

        private System.Collections.IEnumerator Shoot()
        {
            // Run to the time, then let the camera settle with the clock held.
            while (_t < Mathf.Min(_shotAt, Duration)) yield return null;
            _playing = false;
            _t = Mathf.Min(_shotAt, Duration);
            for (float w = 0f; w < 2.5f; w += Time.deltaTime) yield return null;
            yield return new WaitForEndOfFrame();
            Debug.Log($"screenshot at {_t:0.00} s, camera {_mode} at {_cam?.transform.position}, campus bounds {_campusBounds}");
            ScreenCapture.CaptureScreenshot(_shot);
            yield return null;
            yield return null;
            Application.Quit();
        }

        private void Load()
        {
            string playerDir = Application.isEditor ? null : Path.GetDirectoryName(Application.dataPath);
            string repo = Application.isEditor
                ? Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "..", "results")) : null;
            _path = ReplayData.Find(Environment.GetCommandLineArgs(), playerDir, repo, Application.streamingAssetsPath);
            _error = null;
            _d = null;
            if (_path == null)
            {
                _error = "No recorded drives found.\n\nRecord some with\n" +
                         "    .venv/Scripts/python tools/record_drives.py --out results/drives.json\n\n" +
                         "then put drives.json beside RcTwinReplay.exe, or start it with -drives <path>, " +
                         "and press F5.";
                ClearGhosts();
                return;
            }
            try
            {
                _d = ReplayData.Parse(File.ReadAllText(_path));
            }
            catch (Exception e)
            {
                _error = $"Could not read {_path}:\n{e.Message}";
                ClearGhosts();
                return;
            }
            _visible = new bool[_d.policies.Count];
            for (int i = 0; i < _visible.Length; i++) _visible[i] = true;
            _cond = Mathf.Clamp(_cond, 0, _d.conditions.Count - 1);
            _follow = Mathf.Clamp(_follow, 0, _d.policies.Count - 1);
            BuildGhosts();
            ShowTrip(Mathf.Clamp(_trip, 0, Math.Max(0, _d.TripCount(Condition) - 1)));
        }

        private string Condition => _d.conditions[_cond];

        private static string ConditionLabel(string c)
        {
            for (int i = 0; i < ConditionNames.Length; i += 2)
                if (ConditionNames[i] == c) return ConditionNames[i + 1];
            return c;
        }

        private float Car(string key, float fallback) =>
            _d != null && _d.car.TryGetValue(key, out float v) ? v : fallback;

        private void ClearGhosts()
        {
            foreach (var g in _ghosts)
            {
                foreach (var h in g.hits) Destroy(h);
                Destroy(g.root);
                Destroy(g.trail.gameObject);
            }
            _ghosts = Array.Empty<Ghost>();
            foreach (var m in _marks) Destroy(m);
            _marks.Clear();
            if (_route != null) Destroy(_route.gameObject);
            _route = null;
        }

        private void BuildGhosts()
        {
            ClearGhosts();
            _ghosts = new Ghost[_d.policies.Count];
            for (int i = 0; i < _ghosts.Length; i++)
                _ghosts[i] = MakeGhost(_d.policies[i].label, Palette[i % Palette.Length]);
            _route = MakeLine("Route", new Color(0.92f, 0.92f, 0.86f), 0.03f);
        }

        private Ghost MakeGhost(string name, Color colour)
        {
            float hl = Car("half_length", 0.21f), hw = Car("half_width", 0.1375f), r = Car("wheel_radius", 0.05f);
            float lf = Car("lf", 0.16f), lr = Car("lr", 0.16f);
            float tf = Car("track_front", 0.25f), tr = Car("track_rear", 0.25f), co = Car("centre_offset", 0f);
            var g = new Ghost { colour = colour, root = new GameObject($"Car: {name}") };
            g.root.transform.SetParent(transform, false);
            var body = Tinted(Prim(PrimitiveType.Cube, g.root.transform, "Body"), colour);
            body.transform.localScale = new Vector3(1.7f * hw, 0.045f, 1.9f * hl);
            body.transform.localPosition = new Vector3(0f, r + 0.03f, co);
            // A darker nose block, so the heading reads from above.
            var nose = Tinted(Prim(PrimitiveType.Cube, g.root.transform, "Nose"), colour * 0.55f);
            nose.transform.localScale = new Vector3(1.2f * hw, 0.03f, 0.07f);
            nose.transform.localPosition = new Vector3(0f, r + 0.06f, co + 0.75f * hl);
            var tyre = new Color(0.07f, 0.07f, 0.08f);
            var pos = new[]
            {
                new Vector3(-0.5f * tf, r, lf), new Vector3(0.5f * tf, r, lf),
                new Vector3(-0.5f * tr, r, -lr), new Vector3(0.5f * tr, r, -lr),
            };
            for (int k = 0; k < 4; k++)
            {
                var pivot = new GameObject(k < 2 ? "Front wheel" : "Rear wheel").transform;
                pivot.SetParent(g.root.transform, false);
                pivot.localPosition = pos[k];
                var t = Tinted(Prim(PrimitiveType.Cylinder, pivot, "Tyre"), tyre);
                t.transform.localRotation = Quaternion.Euler(0f, 0f, 90f);
                t.transform.localScale = new Vector3(2f * r, 0.0175f, 2f * r);
                var spoke = Tinted(Prim(PrimitiveType.Cube, pivot, "Spoke"), Color.white * 0.85f);
                spoke.transform.localScale = new Vector3(0.037f, 1.8f * r, 0.012f);
                g.wheels[k] = pivot;
            }
            g.trail = MakeLine($"Trail: {name}", colour, 0.025f);
            var lab = new GameObject("Label");
            lab.transform.SetParent(g.root.transform, false);
            g.label = lab.AddComponent<TextMesh>();
            g.label.font = _font;
            g.label.fontSize = 48;
            g.label.characterSize = 0.012f;
            g.label.anchor = TextAnchor.LowerCenter;
            g.label.color = colour;
            g.label.text = name;
            lab.GetComponent<MeshRenderer>().sharedMaterial = _font.material;
            return g;
        }

        private static GameObject Prim(PrimitiveType type, Transform parent, string name)
        {
            var go = GameObject.CreatePrimitive(type);
            go.name = name;
            Destroy(go.GetComponent<Collider>());
            go.transform.SetParent(parent, false);
            return go;
        }

        private GameObject Tinted(GameObject go, Color c)
        {
            var mr = go.GetComponent<MeshRenderer>();
            mr.material = bodyMaterial != null ? new Material(bodyMaterial) { color = c } : mr.material;
            if (bodyMaterial == null) mr.material.color = c;
            return go;
        }

        private LineRenderer MakeLine(string name, Color c, float width)
        {
            var go = new GameObject(name);
            go.transform.SetParent(transform, false);
            var lr = go.AddComponent<LineRenderer>();
            lr.useWorldSpace = true;
            lr.widthMultiplier = width;
            lr.numCapVertices = 2;
            lr.numCornerVertices = 2;
            lr.alignment = LineAlignment.View;
            lr.shadowCastingMode = ShadowCastingMode.Off;
            lr.receiveShadows = false;
            if (lineMaterial != null) lr.material = new Material(lineMaterial) { color = c };
            lr.startColor = lr.endColor = c;
            lr.positionCount = 0;
            return lr;
        }

        private GameObject Disc(Vector2 iso, Color c, float diameter, double lift)
        {
            var go = Tinted(Prim(PrimitiveType.Cylinder, transform, "Marker"), c);
            go.transform.position = TwinCar.ToUnity(iso.x, iso.y, lift);
            go.transform.localScale = new Vector3(diameter, 0.002f, diameter);
            _marks.Add(go);
            return go;
        }

        // ------------------------------------------------------------- trips

        /// <summary>The trip from one stop to another; with no such trip, the
        /// first from that stop (to changed) or else to that stop.</summary>
        private void PickRoute(string from, string to)
        {
            int k = _d.FindTrip(Condition, from, to);
            if (k < 0)
            {
                int n = _d.TripCount(Condition);
                for (int j = 0; j < n && k < 0; j++)
                {
                    var d = FirstOf(j);
                    if (d != null && (d.from == from || (from == null && d.to == to))) k = j;
                }
            }
            if (k >= 0) ShowTrip(k);
        }

        private void Reverse()
        {
            var d = FirstDrive();
            if (d != null && _d.FindTrip(Condition, d.to, d.from) is int k && k >= 0) ShowTrip(k);
        }

        private void ShowTrip(int trip)
        {
            int n = _d.TripCount(Condition);
            if (n == 0) return;
            _trip = ((trip % n) + n) % n;
            _gridScroll.y = Mathf.Max(0f, (_trip - 6) * 21f);   // keep the row in view
            _t = 0f;
            _hold = 0f;
            foreach (var m in _marks) Destroy(m);
            _marks.Clear();

            Vector2[] route = null;
            for (int i = 0; i < _ghosts.Length; i++)
            {
                var g = _ghosts[i];
                foreach (var h in g.hits) Destroy(h);
                g.hits.Clear();
                g.drive = _d.Get(i, Condition, _trip);
                g.trailCount = -1;
                if (g.drive == null) continue;
                route ??= g.drive.route;
                var s = g.drive.samples;
                g.path = new Vector3[s.Length];
                for (int k = 0; k < s.Length; k++) g.path[k] = TwinCar.ToUnity(s[k].x, s[k].y, 0.008);
                foreach (var hit in g.drive.hits)
                {
                    var m = Tinted(Prim(PrimitiveType.Sphere, transform, "Wall contact"), g.colour);
                    m.transform.position = TwinCar.ToUnity(hit.y, hit.z, 0.05);
                    m.transform.localScale = Vector3.one * 0.07f;
                    m.SetActive(false);
                    g.hits.Add(m);
                }
            }
            if (route == null) return;

            _route.positionCount = route.Length;
            var lo = new Vector3(float.MaxValue, 0f, float.MaxValue);
            var hi = new Vector3(float.MinValue, 0f, float.MinValue);
            for (int k = 0; k < route.Length; k++)
            {
                var p = TwinCar.ToUnity(route[k].x, route[k].y, 0.004);
                _route.SetPosition(k, p);
                lo = Vector3.Min(lo, p);
                hi = Vector3.Max(hi, p);
            }
            _routeBounds = new Bounds(0.5f * (lo + hi), hi - lo);
            Disc(route[0], Finished, 0.45f, 0.003);
            Disc(route[route.Length - 1], Color.white, 0.45f, 0.003);
            if (Condition == "perturbed" && _d.perturbation.TryGetValue("at_s", out float atS))
                Disc(ReplayData.PointAt(route, atS), Bumped, 0.35f, 0.005);
        }

        // A drive that timed out is usually a car pinned against a kerb until
        // the 120 s limit; playback stops a few seconds after the last drive
        // that finished or left the road (the trip's "until", which the
        // recorder sets and records timed-out drives up to), and shows the
        // rest as timed out.
        private float Cap
        {
            get
            {
                var lead = FirstDrive();
                if (lead != null && lead.until > 0f) return lead.until;
                float c = 0f;
                foreach (var g in _ghosts)
                    if (g.drive != null && g.drive.status != "timeout") c = Mathf.Max(c, g.drive.time);
                return c > 0f ? c + 3f : 30f;
            }
        }

        private float End(ReplayData.Drive d) => d.status == "timeout" ? Mathf.Min(d.time, Cap) : d.time;

        private float Duration
        {
            get
            {
                float d = 0f;
                for (int i = 0; i < _ghosts.Length; i++)
                    if (_visible[i] && _ghosts[i].drive != null) d = Mathf.Max(d, End(_ghosts[i].drive));
                return d;
            }
        }

        // ------------------------------------------------------------ update

        private void Update()
        {
            HandleKeys();
            if (_d == null) return;
            float dur = Duration;
            if (_playing)
            {
                _t += Time.deltaTime * _rate;
                if (_t >= dur)
                {
                    _t = dur;
                    _hold += Time.deltaTime;
                    if (_hold > 1.5f)
                    {
                        if (_auto) ShowTrip(_trip + 1);
                        else _playing = false;
                    }
                }
            }
            for (int i = 0; i < _ghosts.Length; i++) Place(_ghosts[i], _visible[i]);
        }

        private void Place(Ghost g, bool visible)
        {
            bool on = visible && g.drive != null;
            g.root.SetActive(on);
            g.trail.gameObject.SetActive(on);
            for (int k = 0; k < g.hits.Count; k++) g.hits[k].SetActive(on && _t >= g.drive.hits[k].x);
            if (!on) return;
            int hz = _d.sampleHz;
            var p = ReplayData.At(g.drive, _t, hz);
            Vector3 at = TwinCar.ToUnity(p.x, p.y, 0.0);
            g.root.transform.SetPositionAndRotation(at, Quaternion.Euler(0f, TwinCar.YawToUnityDeg(p.yaw), 0f));
            float spin = p.odometer / Car("wheel_radius", 0.05f) * Mathf.Rad2Deg;
            float steer = TwinCar.YawToUnityDeg(p.steer * Car("steer_max", 0.4f));
            for (int k = 0; k < 4; k++)
                g.wheels[k].localRotation = Quaternion.Euler(0f, k < 2 ? steer : 0f, 0f) * Quaternion.Euler(spin, 0f, 0f);

            int n = ReplayData.SampleIndex(g.drive, _t, hz) + 1;
            if (n != g.trailCount)
            {
                g.trail.positionCount = n + 1;
                for (int k = 0; k < n; k++) g.trail.SetPosition(k, g.path[k]);
                g.trailCount = n;
            }
            g.trail.SetPosition(n, at + Vector3.up * 0.008f);

            g.label.gameObject.SetActive(_labels);
            if (_labels)
            {
                int idx = Array.IndexOf(_ghosts, g);
                // Readable at any zoom: size and stacking grow with distance.
                float far = _cam != null ? Mathf.Max(1f, Vector3.Distance(_cam.transform.position, at) / 2.5f) : 1f;
                g.label.characterSize = 0.008f * far;
                g.label.transform.position = at + Vector3.up * far * (0.2f + 0.05f * idx);
                if (_cam != null) g.label.transform.rotation = _cam.transform.rotation;
                g.label.text = p.ended || _t >= End(g.drive) ? $"{_d.policies[idx].label}  {Outcome(g.drive).ToLowerInvariant()}"
                                       : _d.policies[idx].label;
            }
        }

        private void LateUpdate()
        {
            if (_cam == null || _d == null) return;
            Vector3 want, look;
            float fov = _cam.fieldOfView * Mathf.Deg2Rad;
            var g = _follow < _ghosts.Length ? _ghosts[_follow] : null;
            if (_mode == CamMode.Chase && g != null && g.drive != null && _visible[_follow])
            {
                var tr = g.root.transform;
                Vector3 back = Quaternion.Euler(0f, _orbit, 0f) * -tr.forward;
                look = tr.position + Vector3.up * 0.05f;
                want = look + back * (1.6f * _zoom) + Vector3.up * (0.75f * _zoom);
            }
            else if (_mode == CamMode.Pack || _mode == CamMode.Chase)
            {
                // Above the cars still driving (all of them once every drive
                // has ended), far enough back to keep them all in view.
                var b = new Bounds();
                bool any = false;
                for (int pass = 0; pass < 2 && !any; pass++)
                    for (int i = 0; i < _ghosts.Length; i++)
                    {
                        var gi = _ghosts[i];
                        if (!_visible[i] || gi.drive == null || (pass == 0 && _t >= End(gi.drive))) continue;
                        var at = gi.root.transform.position;
                        if (!any) b = new Bounds(at, Vector3.zero); else b.Encapsulate(at);
                        any = true;
                    }
                if (!any) b = _routeBounds;
                float radius = 0.5f * Mathf.Max(b.size.x, b.size.z) + 1.2f;
                float dist = Mathf.Max(3f, radius / Mathf.Tan(0.5f * fov) * 1.3f) * _zoom;
                look = b.center;
                want = look + Quaternion.Euler(52f, _orbit, 0f) * Vector3.back * dist;
            }
            else
            {
                var b = _mode == CamMode.Campus ? _campusBounds : _routeBounds;
                float radius = 0.5f * Mathf.Max(b.size.x, b.size.z) + (_mode == CamMode.Campus ? 1f : 1.5f);
                float dist = radius / Mathf.Tan(0.5f * fov) * 1.3f * _zoom;
                float pitch = _mode == CamMode.Campus ? 89f : 58f;
                look = b.center;
                want = look + Quaternion.Euler(pitch, _orbit, 0f) * Vector3.back * dist;
            }
            float k = 1f - Mathf.Exp(-5f * Time.deltaTime);
            _cam.transform.position = Vector3.Lerp(_cam.transform.position, want, k);
            var rot = Quaternion.LookRotation(look - _cam.transform.position, Vector3.up);
            _cam.transform.rotation = Quaternion.Slerp(_cam.transform.rotation, rot, k);
            foreach (var label in _stopLabels) label.transform.rotation = _cam.transform.rotation;
        }

        private void HandleKeys()
        {
            var kb = Keyboard.current;
            if (kb != null)
            {
                if (kb.f5Key.wasPressedThisFrame) Load();
                if (kb.escapeKey.wasPressedThisFrame && !Application.isEditor) Application.Quit();
                if (kb.hKey.wasPressedThisFrame) _panels = !_panels;
                if (_d != null)
                {
                    if (kb.spaceKey.wasPressedThisFrame) TogglePlay();
                    if (kb.rightArrowKey.wasPressedThisFrame || kb.nKey.wasPressedThisFrame) ShowTrip(_trip + 1);
                    if (kb.leftArrowKey.wasPressedThisFrame || kb.pKey.wasPressedThisFrame) ShowTrip(_trip - 1);
                    if (kb.upArrowKey.wasPressedThisFrame) SetCondition(_cond - 1);
                    if (kb.downArrowKey.wasPressedThisFrame) SetCondition(_cond + 1);
                    if (kb.cKey.wasPressedThisFrame) _mode = (CamMode)(((int)_mode + 1) % 4);
                    if (kb.rKey.wasPressedThisFrame) { _t = 0f; _hold = 0f; _playing = true; }
                    if (kb.aKey.wasPressedThisFrame) _auto = !_auto;
                    if (kb.vKey.wasPressedThisFrame) Reverse();
                    if (kb.gKey.wasPressedThisFrame) _grid = !_grid;
                    if (kb.lKey.wasPressedThisFrame) _labels = !_labels;
                    if (kb.leftBracketKey.wasPressedThisFrame) StepRate(-1);
                    if (kb.rightBracketKey.wasPressedThisFrame) StepRate(1);
                    for (int i = 0; i < Math.Min(9, _ghosts.Length); i++)
                        if (kb[Key.Digit1 + i].wasPressedThisFrame) Follow(i);
                }
            }
            var mouse = Mouse.current;
            if (mouse == null) return;
            Vector2 mp = mouse.position.ReadValue();
            float s = GuiScale;
            var gui = new Vector2(mp.x / s, (Screen.height - mp.y) / s);
            foreach (var r in _guiRects)
                if (r.Contains(gui)) return;
            float wheel = mouse.scroll.ReadValue().y;
            if (Mathf.Abs(wheel) > 0.01f) _zoom = Mathf.Clamp(_zoom * (wheel > 0f ? 0.9f : 1.1f), 0.15f, 3f);
            if (mouse.rightButton.isPressed || mouse.leftButton.isPressed)
                _orbit += mouse.delta.ReadValue().x * 0.25f;
        }

        private void TogglePlay()
        {
            if (!_playing && _t >= Duration) { _t = 0f; _hold = 0f; }
            _playing = !_playing;
        }

        private void StepRate(int dir)
        {
            int i = Array.IndexOf(Rates, _rate);
            _rate = Rates[Mathf.Clamp((i < 0 ? 2 : i) + dir, 0, Rates.Length - 1)];
        }

        private void SetCondition(int c)
        {
            int n = _d.conditions.Count;
            _cond = ((c % n) + n) % n;
            ShowTrip(_trip);
        }

        private void Follow(int i)
        {
            _follow = i;
            _mode = CamMode.Chase;
            _visible[i] = true;
        }

        private static string Outcome(ReplayData.Drive d) =>
            d.status == "reached_end" ? (d.wallContacts > 0 ? "Finished, bumped" : "Finished")
            : d.status == "left_road" ? "Left road" : d.status == "timeout" ? "Timed out" : d.status;

        private static Color OutcomeColour(ReplayData.Drive d) =>
            d == null ? new Color(0.3f, 0.3f, 0.3f)
            : d.status == "reached_end" ? (d.wallContacts > 0 ? Bumped : Finished)
            : d.status == "left_road" ? OffRoad : TimedOut;

        // --------------------------------------------------------------- GUI

        private GUIStyle _panel, _title, _sub, _text, _small, _btn, _btnOn, _chip, _cellLabel;
        private Texture2D _panelTex, _btnTex, _btnHot, _btnOnTex;

        private static float GuiScale => Mathf.Max(0.7f, Screen.height / 900f);

        private void Styles()
        {
            if (_panel != null) return;
            _panelTex = Solid(new Color(0.07f, 0.08f, 0.1f, 0.88f));
            _btnTex = Solid(new Color(0.17f, 0.19f, 0.23f, 1f));
            _btnHot = Solid(new Color(0.24f, 0.27f, 0.32f, 1f));
            _btnOnTex = Solid(new Color(0.2f, 0.42f, 0.78f, 1f));
            _panel = new GUIStyle { normal = { background = _panelTex }, padding = new RectOffset(14, 14, 12, 12) };
            _title = new GUIStyle(GUI.skin.label) { fontSize = 20, fontStyle = FontStyle.Bold, normal = { textColor = Color.white } };
            _sub = new GUIStyle(GUI.skin.label) { fontSize = 12, wordWrap = true, normal = { textColor = new Color(0.68f, 0.71f, 0.76f) } };
            _text = new GUIStyle(GUI.skin.label) { fontSize = 13, normal = { textColor = new Color(0.92f, 0.93f, 0.95f) } };
            _small = new GUIStyle(_sub) { fontSize = 11 };
            _btn = new GUIStyle(GUI.skin.button)
            {
                fontSize = 12, normal = { background = _btnTex, textColor = Color.white },
                hover = { background = _btnHot, textColor = Color.white },
                active = { background = _btnOnTex, textColor = Color.white }, border = new RectOffset(0, 0, 0, 0),
            };
            _btnOn = new GUIStyle(_btn) { normal = { background = _btnOnTex, textColor = Color.white }, hover = { background = _btnOnTex, textColor = Color.white } };
            _chip = new GUIStyle(_text) { fontSize = 11, fontStyle = FontStyle.Bold, alignment = TextAnchor.MiddleLeft };
            _cellLabel = new GUIStyle(_small) { alignment = TextAnchor.MiddleLeft, wordWrap = false };
        }

        private static Texture2D Solid(Color c)
        {
            var t = new Texture2D(1, 1) { hideFlags = HideFlags.HideAndDontSave };
            t.SetPixel(0, 0, c);
            t.Apply();
            return t;
        }

        private static void Swatch(Rect r, Color c) =>
            GUI.DrawTexture(r, Texture2D.whiteTexture, ScaleMode.StretchToFill, false, 0f, c, 0f, 3f);

        private void OnGUI()
        {
            Styles();
            float s = GuiScale;
            GUI.matrix = Matrix4x4.Scale(new Vector3(s, s, 1f));
            float w = Screen.width / s, h = Screen.height / s;
            _guiRects.Clear();

            if (_d == null)
            {
                var r = new Rect(0.5f * w - 300f, 0.5f * h - 120f, 600f, 240f);
                _guiRects.Add(r);
                GUILayout.BeginArea(r, _panel);
                GUILayout.Label("Policy replay", _title);
                GUILayout.Label(_error ?? "Loading…", _sub);
                GUILayout.EndArea();
                return;
            }
            if (!_panels)
            {
                GUI.Label(new Rect(12f, h - 24f, 400f, 20f), "H shows the panels", _small);
                return;
            }
            LeftPanel(w, h);
            if (_grid) GridPanel(w, h);
            BottomBar(w, h);
            FollowHud(w);
        }

        private void LeftPanel(float w, float h)
        {
            var r = new Rect(16f, 16f, 480f, Mathf.Min(h - 120f, 150f + 26f * _ghosts.Length + 180f));
            _guiRects.Add(r);
            GUILayout.BeginArea(r, _panel);
            GUILayout.Label("Policy replay", _title);
            GUILayout.Label($"Campus at 1/10, target {_d.targetSpeed:0.#} m/s. Every policy drives the same trip " +
                            $"and the same car. Recorded {_d.created}.", _sub);
            GUILayout.Space(6f);

            GUILayout.BeginHorizontal();
            for (int i = 0; i < _d.conditions.Count; i++)
                if (GUILayout.Button(ConditionLabel(_d.conditions[i]), i == _cond ? _btnOn : _btn, GUILayout.Height(24f)))
                    SetCondition(i);
            GUILayout.EndHorizontal();

            var lead = FirstDrive();
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("<", _btn, GUILayout.Width(30f), GUILayout.Height(24f))) ShowTrip(_trip - 1);
            string trip = lead == null ? "" : $"Trip {_trip + 1} of {_d.TripCount(Condition)}:  {lead.from} → {lead.to}";
            GUILayout.Label(trip, new GUIStyle(_text) { alignment = TextAnchor.MiddleCenter, fontStyle = FontStyle.Bold }, GUILayout.Height(24f));
            if (GUILayout.Button(">", _btn, GUILayout.Width(30f), GUILayout.Height(24f))) ShowTrip(_trip + 1);
            bool canReverse = lead != null && _d.FindTrip(Condition, lead.to, lead.from) >= 0;
            GUI.enabled = canReverse;
            if (GUILayout.Button("Reverse", _btn, GUILayout.Width(64f), GUILayout.Height(24f))) Reverse();
            GUI.enabled = true;
            GUILayout.EndHorizontal();
            if (lead != null)
            {
                var stops = _d.Stops(Condition);
                StopRow("From", stops, lead.from, s => StartsATrip(s),
                        s => PickRoute(s, lead.to));
                StopRow("To", stops, lead.to, s => _d.FindTrip(Condition, lead.from, s) >= 0,
                        s => PickRoute(lead.from, s));
            }
            if (lead != null)
            {
                var c = lead.car;
                string Get(string k, string f) => c.TryGetValue(k, out float v) ? string.Format(f, v) : "";
                GUILayout.Label("This car: grip " + Get("friction", "×{0:0.00}") + ", payload " + Get("payload_kg", "+{0:0.00} kg") +
                                ", servo " + Get("servo_bandwidth", "×{0:0.00}") + ", motor " + Get("torque", "×{0:0.00}") +
                                ", tyres " + Get("tyre_stiffness", "×{0:0.00}") + ", battery R " + Get("battery_resistance", "×{0:0.00}"),
                                _small);
                if (Condition == "perturbed")
                {
                    var p = _d.perturbation;
                    GUILayout.Label($"At {p.GetValueOrDefault("at_s"):0.#} m the car changes: grip ×{p.GetValueOrDefault("friction"):0.##}, " +
                                    $"+{p.GetValueOrDefault("payload_kg"):0.#} kg, servo ×{p.GetValueOrDefault("servo_bandwidth"):0.##} (amber disc)",
                                    new GUIStyle(_small) { normal = { textColor = Bumped } });
                }
            }
            GUILayout.Space(8f);

            GUILayout.BeginHorizontal();
            GUILayout.Label("", _small, GUILayout.Width(36f));
            GUILayout.Label("policy", _small, GUILayout.Width(200f));
            GUILayout.Label("outcome", _small, GUILayout.Width(100f));
            GUILayout.Label("time", _small, GUILayout.Width(44f));
            GUILayout.Label("offset", _small, GUILayout.Width(50f));
            GUILayout.EndHorizontal();
            for (int i = 0; i < _ghosts.Length; i++)
            {
                var g = _ghosts[i];
                var d = g.drive;
                GUILayout.BeginHorizontal(GUILayout.Height(24f));
                _visible[i] = GUILayout.Toggle(_visible[i], "", GUILayout.Width(16f), GUILayout.Height(22f));
                var sw = GUILayoutUtility.GetRect(14f, 14f, GUILayout.Width(14f), GUILayout.Height(22f));
                Swatch(new Rect(sw.x, sw.y + 5f, 14f, 12f), g.colour);
                GUILayout.Space(4f);
                string name = (i < 9 ? $"{i + 1}  " : "   ") + _d.policies[i].label;
                if (GUILayout.Button(name, i == _follow && _mode == CamMode.Chase ? _btnOn : _btn, GUILayout.Width(200f), GUILayout.Height(22f)))
                    Follow(i);
                if (d == null)
                {
                    GUILayout.Label("no drive", _small);
                }
                else
                {
                    bool done = _t >= End(d);
                    var chip = new GUIStyle(_chip) { normal = { textColor = done ? OutcomeColour(d) : new Color(0.75f, 0.78f, 0.82f) } };
                    var p = ReplayData.At(d, _t, _d.sampleHz);
                    GUILayout.Label(done ? Outcome(d) : $"{p.speed:0.00} m/s", chip, GUILayout.Width(100f), GUILayout.Height(22f));
                    GUILayout.Label(done && d.status == "timeout" ? $"{d.time:0} s" : $"{Mathf.Min(_t, d.time):0.0} s", _text, GUILayout.Width(44f), GUILayout.Height(22f));
                    GUILayout.Label($"{100f * d.offsetMean:0.0} cm", _text, GUILayout.Width(50f), GUILayout.Height(22f));
                }
                GUILayout.EndHorizontal();
            }
            GUILayout.Space(4f);
            GUILayout.Label("Outcome shows when the car's drive ends. Offset: mean distance from the centreline. " +
                            "Spheres: wall contacts.", _small);
            GUILayout.EndArea();
        }

        // One row of stop buttons: the current one lit, the ones with no
        // recorded trip greyed out.
        private void StopRow(string what, List<string> stops, string current, Func<string, bool> available,
                             Action<string> pick)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(what, _small, GUILayout.Width(34f), GUILayout.Height(22f));
            float bw = Mathf.Max(30f, (440f - 34f) / Mathf.Max(1, stops.Count) - 4f);
            foreach (var stop in stops)
            {
                bool on = stop == current;
                GUI.enabled = on || available(stop);
                if (GUILayout.Button(stop, on ? _btnOn : _btn, GUILayout.Width(bw), GUILayout.Height(22f)) && !on)
                    pick(stop);
            }
            GUI.enabled = true;
            GUILayout.EndHorizontal();
        }

        private bool StartsATrip(string stop)
        {
            int n = _d.TripCount(Condition);
            for (int j = 0; j < n; j++)
                if (FirstOf(j) is ReplayData.Drive d && d.from == stop) return true;
            return false;
        }

        private ReplayData.Drive FirstDrive()
        {
            foreach (var g in _ghosts)
                if (g.drive != null) return g.drive;
            return null;
        }

        private void GridPanel(float w, float h)
        {
            int n = _d.TripCount(Condition), m = _ghosts.Length;
            const float cell = 18f, nameW = 96f;
            float pw = 28f + nameW + m * (cell + 4f);
            var r = new Rect(w - pw - 16f, 16f, pw, Mathf.Min(h - 120f, 112f + n * (cell + 3f)));
            _guiRects.Add(r);
            GUILayout.BeginArea(r, _panel);
            GUILayout.Label($"All trips: {ConditionLabel(Condition).ToLowerInvariant()}", _text);
            GUILayout.BeginHorizontal();
            GUILayout.Label("finished", _small, GUILayout.Width(nameW));
            for (int i = 0; i < m; i++)
            {
                int ok = 0;
                foreach (var d in _d.policies[i].runs[Condition]) ok += d != null && d.Finished ? 1 : 0;
                var col = GUILayoutUtility.GetRect(cell, 30f, GUILayout.Width(cell + 1f));
                Swatch(new Rect(col.x + 2f, col.y, cell - 4f, 6f), _ghosts[i].colour);
                GUI.Label(new Rect(col.x - 6f, col.y + 7f, cell + 12f, 22f), $"{ok}", new GUIStyle(_small) { alignment = TextAnchor.UpperCenter });
            }
            GUILayout.EndHorizontal();
            _gridScroll = GUILayout.BeginScrollView(_gridScroll, false, false, GUIStyle.none, GUI.skin.verticalScrollbar,
                                                    GUILayout.Height(Mathf.Max(60f, r.height - 140f)));
            for (int k = 0; k < n; k++)
            {
                GUILayout.BeginHorizontal(GUILayout.Height(cell + 1f));
                var any = _d.Get(0, Condition, k) ?? FirstOf(k);
                string label = any == null ? $"{k + 1}" : $"{k + 1}  {any.from}→{any.to}";
                if (GUILayout.Button(label, k == _trip ? _btnOn : _btn, GUILayout.Width(nameW), GUILayout.Height(cell)))
                    ShowTrip(k);
                for (int i = 0; i < m; i++)
                {
                    var c = GUILayoutUtility.GetRect(cell, cell, GUILayout.Width(cell + 1f), GUILayout.Height(cell));
                    Swatch(new Rect(c.x + 1f, c.y + 1f, cell - 2f, cell - 2f), OutcomeColour(_d.Get(i, Condition, k)));
                }
                GUILayout.EndHorizontal();
            }
            GUILayout.EndScrollView();
            GUILayout.BeginHorizontal();
            Key2(Finished, "finished");
            Key2(Bumped, "bumped");
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            Key2(OffRoad, "left road");
            Key2(TimedOut, "timed out");
            GUILayout.EndHorizontal();
            GUILayout.EndArea();
        }

        private ReplayData.Drive FirstOf(int trip)
        {
            for (int i = 0; i < _ghosts.Length; i++)
            {
                var d = _d.Get(i, Condition, trip);
                if (d != null) return d;
            }
            return null;
        }

        private void Key2(Color c, string what)
        {
            var r = GUILayoutUtility.GetRect(12f, 16f, GUILayout.Width(12f));
            Swatch(new Rect(r.x, r.y + 3f, 10f, 10f), c);
            GUILayout.Label(what, _small, GUILayout.Width(62f));
        }

        private void BottomBar(float w, float h)
        {
            float bw = Mathf.Min(w - 32f, 880f);
            var r = new Rect(0.5f * (w - bw), h - 92f, bw, 76f);
            _guiRects.Add(r);
            GUILayout.BeginArea(r, _panel);
            GUILayout.BeginHorizontal();
            if (GUILayout.Button(_playing ? "Pause" : "Play", _btn, GUILayout.Width(64f), GUILayout.Height(24f))) TogglePlay();
            if (GUILayout.Button("Restart", _btn, GUILayout.Width(64f), GUILayout.Height(24f))) { _t = 0f; _hold = 0f; _playing = true; }
            GUILayout.Space(6f);
            foreach (float rate in Rates)
                if (GUILayout.Button($"{rate:0.##}×", Mathf.Approximately(rate, _rate) ? _btnOn : _btn, GUILayout.Width(44f), GUILayout.Height(24f)))
                    _rate = rate;
            GUILayout.Space(6f);
            string cam = "Camera: " + _mode.ToString().ToLowerInvariant();
            if (GUILayout.Button(cam, _btn, GUILayout.Width(118f), GUILayout.Height(24f))) _mode = (CamMode)(((int)_mode + 1) % 4);
            if (GUILayout.Button(_auto ? "Next trip: auto" : "Next trip: manual", _auto ? _btnOn : _btn, GUILayout.Width(124f), GUILayout.Height(24f)))
                _auto = !_auto;
            GUILayout.FlexibleSpace();
            GUILayout.Label($"{_t:0.00} / {Duration:0.00} s", _text, GUILayout.Height(24f));
            GUILayout.EndHorizontal();
            float dur = Duration;
            float t = GUILayout.HorizontalSlider(_t, 0f, Mathf.Max(0.01f, dur));
            if (!Mathf.Approximately(t, _t)) { _t = t; _hold = 0f; }
            GUILayout.Label("Space play · ←/→ trip · V reverse · ↑/↓ condition · C camera · 1–9 follow · [ ] speed · " +
                            "drag to orbit, scroll to zoom · G grid · L labels · H hide · F5 reload", _small);
            GUILayout.EndArea();
        }

        private void FollowHud(float w)
        {
            if (_mode != CamMode.Chase || _follow >= _ghosts.Length) return;
            var d = _ghosts[_follow].drive;
            if (d == null) return;
            var p = ReplayData.At(d, _t, _d.sampleHz);
            var r = new Rect(0.5f * w - 330f, 16f, 660f, 44f);
            _guiRects.Add(r);
            GUILayout.BeginArea(r, _panel);
            GUILayout.Label($"Following {_d.policies[_follow].label}:  {p.speed:0.00} m/s,  steer {p.steer:+0.00;-0.00},  " +
                            $"throttle {p.throttle:+0.00;-0.00},  offset {100f * p.offset:+0.0;-0.0} cm",
                            new GUIStyle(_text) { normal = { textColor = _ghosts[_follow].colour } });
            GUILayout.EndArea();
        }

        private static Color Hex(int rgb) =>
            new Color(((rgb >> 16) & 255) / 255f, ((rgb >> 8) & 255) / 255f, (rgb & 255) / 255f);
    }
}
