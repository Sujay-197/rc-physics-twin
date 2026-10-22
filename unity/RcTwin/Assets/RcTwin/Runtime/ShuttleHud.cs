// The campus shuttle game's screen, layout C (Docs/specs/2026-10-21-campus-shuttle-design.md):
// the score top-left, a minimap top-right, a bottom bar with the battery, the
// seats, what the shuttle is doing and the request queue. Click a stop, on the
// minimap or in the world, to raise a request; the gear opens the settings.
using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.InputSystem;

namespace RcTwin
{
    /// <summary>The campus fitted into a screen rectangle, top-down, north (ISO +x) up.</summary>
    public sealed class Minimap
    {
        /// <summary>Margin inside the area, so stop dots at the edge stay whole [px].</summary>
        public const float Pad = 6f;

        public Rect Map { get; }
        /// <summary>The campus's width over its height on the map.</summary>
        public double CampusAspect { get; }

        private readonly CampusRoutes _net;
        private readonly double _minU, _maxV, _scale;

        public Minimap(CampusRoutes net, Rect area, float pad = Pad)
        {
            _net = net;
            double minU = double.MaxValue, maxU = double.MinValue, minV = double.MaxValue, maxV = double.MinValue;
            void Add(double x, double y)
            {
                double u = -y, v = x;   // screen right = ISO -y, screen up = ISO +x
                minU = Math.Min(minU, u); maxU = Math.Max(maxU, u);
                minV = Math.Min(minV, v); maxV = Math.Max(maxV, v);
            }
            foreach (var c in net.Centrelines())
                for (int i = 0; i + 1 < c.Length; i += 3) Add(c[i], c[i + 1]);
            foreach (var s in net.Stops.Values) Add(s.nodeX, s.nodeY);
            double du = Math.Max(1e-6, maxU - minU), dv = Math.Max(1e-6, maxV - minV);
            CampusAspect = du / dv;
            double w = area.width - 2 * pad, h = area.height - 2 * pad;
            _scale = Math.Min(w / du, h / dv);
            float mw = (float)(du * _scale), mh = (float)(dv * _scale);
            Map = new Rect(area.x + pad + (float)(w - mw) / 2f, area.y + pad + (float)(h - mh) / 2f, mw, mh);
            _minU = minU;
            _maxV = maxV;
        }

        /// <summary>ISO (x, y) to a GUI point (y down).</summary>
        public Vector2 ToMap(double x, double y) =>
            new Vector2(Map.x + (float)((-y - _minU) * _scale), Map.y + (float)((_maxV - x) * _scale));

        /// <summary>The stop whose road node is within radius of the click, nearest first; else null.</summary>
        public string StopAt(Vector2 click, float radiusPx)
        {
            string best = null;
            float bd = radiusPx;
            foreach (var s in _net.Stops.Values)
            {
                float d = Vector2.Distance(click, ToMap(s.nodeX, s.nodeY));
                if (d <= bd) { bd = d; best = s.name; }
            }
            return best;
        }
    }

    [RequireComponent(typeof(ShuttleGame))]
    public sealed class ShuttleHud : MonoBehaviour
    {
        private ShuttleGame _game;
        private TwinCar _car;
        private Minimap _map;
        private Rect _mapArea;
        private Material _lines;
        private GUIStyle _box, _big, _text, _small, _chip, _chipAssigned, _btn, _btnOn, _tag;
        private Texture2D _white;
        private string _pickFrom;                 // destination picker open for this origin
        private bool _settings;
        private float _scale = 1f;
        private bool _paused;

        private void Start()
        {
            _game = GetComponent<ShuttleGame>();
            _car = GetComponent<TwinCar>();
        }

        private void Update()
        {
            var m = Mouse.current;
            if (_game.Routes == null || Camera.main == null || m == null || !m.leftButton.wasPressedThisFrame) return;
            Vector2 screen = m.position.ReadValue();
            var mouse = new Vector2(screen.x, Screen.height - screen.y);
            if (_mapArea.Contains(mouse) || mouse.y > Screen.height - BarHeight || _pickFrom != null || _settings) return;
            // A click on the ground near a stop, in the 3D view.
            var ray = Camera.main.ScreenPointToRay(screen);
            if (Mathf.Abs(ray.direction.y) < 1e-4f) return;
            float t = (0.03f - ray.origin.y) / ray.direction.y;
            if (t <= 0f) return;
            var p = ray.origin + t * ray.direction;
            double x = p.z, y = -p.x;   // Unity to ISO
            var (stop, d) = _game.Routes.Nearest(x, y);
            if (d < 1.2) _pickFrom = stop;
        }

        private float BarHeight => Mathf.Max(64f, Screen.height * 0.09f);

        private void OnGUI()
        {
            if (_game == null || _game.Brain == null || _game.Routes == null) return;
            Styles();
            float w = Screen.width, h = Screen.height;
            float mapW = Mathf.Max(220f, w * 0.22f), mapH = mapW * 0.75f;
            var area = new Rect(w - mapW - 12f, 12f, mapW, mapH);
            if (_map == null || area != _mapArea) { _mapArea = area; _map = new Minimap(_game.Routes, area); }

            WorldLabels();
            ScorePanel();
            MinimapPanel();
            BottomBar(w, h);
            if (_pickFrom != null) Picker(w, h);
            if (_settings) SettingsPanel(w, h);
        }

        // ------------------------------------------------------ in the world

        // Over each stop's waiting students, their destinations and wait; over the shuttle, its seats.
        private void WorldLabels()
        {
            var cam = Camera.main;
            if (cam == null || Event.current.type != EventType.Repaint) return;
            foreach (var g in _game.Waiting.GroupBy(r => r.Origin))
                Tag(cam, _game.KerbSlot(g.Key, 0) + Vector3.up * 0.3f, WaitLabel(g, _game.Now));
            Tag(cam, transform.position + Vector3.up * 0.28f, SeatLabel(_game.Brain.Aboard.Count, _game.Settings.Capacity));
        }

        private void Tag(Camera cam, Vector3 world, string text)
        {
            var sp = cam.WorldToScreenPoint(world);
            if (sp.z <= 0f || string.IsNullOrEmpty(text)) return;
            var content = new GUIContent(text);
            var size = _tag.CalcSize(content);
            var r = new Rect(sp.x - size.x / 2f, Screen.height - sp.y - size.y, size.x, size.y);
            if (r.Overlaps(_mapArea) || r.yMax > Screen.height - BarHeight) return;
            GUI.Label(r, content, _tag);
        }

        // ------------------------------------------------------------- score

        private void ScorePanel()
        {
            var s = _game.Score;
            GUILayout.BeginArea(new Rect(12f, 12f, 330f, 76f), _box);
            GUILayout.Label($"Score {s.Points:N0}", _big);
            GUILayout.Label($"Delivered {s.Delivered} · Avg wait {Clock(s.AverageWait)} · Gave up {s.GaveUp}", _text);
            GUILayout.EndArea();
        }

        // ----------------------------------------------------------- minimap

        private void MinimapPanel()
        {
            GUI.Box(_mapArea, GUIContent.none, _box);
            var net = _game.Routes;
            if (Event.current.type == EventType.Repaint)
            {
                Lines(() =>
                {
                    GL.Color(new Color(0.55f, 0.57f, 0.6f));
                    foreach (var c in net.Centrelines())
                        for (int i = 3; i + 1 < c.Length; i += 3)
                        {
                            GL.Vertex(_map.ToMap(c[i - 3], c[i - 2]));
                            GL.Vertex(_map.ToMap(c[i], c[i + 1]));
                        }
                    // The shuttle's planned path, stop to stop along the routes.
                    GL.Color(new Color(0.35f, 0.75f, 1f));
                    var st = _car.State;
                    string at = _game.AtStop ?? _game.LegFrom;
                    foreach (var ps in _game.Brain.Plan.Stops)
                    {
                        if (at != null && at != ps.Stop && net.Stops[at].node != net.Stops[ps.Stop].node)
                        {
                            var r = net.Route(at, ps.Stop);
                            for (int i = 3; i + 1 < r.Length; i += 3)
                            {
                                GL.Vertex(_map.ToMap(r[i - 3], r[i - 2]));
                                GL.Vertex(_map.ToMap(r[i], r[i + 1]));
                            }
                        }
                        at = ps.Stop;
                    }
                    // The shuttle: an arrow along its heading.
                    var p = _map.ToMap(st.x, st.y);
                    var f = _map.ToMap(st.x + Math.Cos(st.yaw) * 1.2, st.y + Math.Sin(st.yaw) * 1.2) - p;
                    var side = new Vector2(-f.y, f.x) * 0.5f;
                    GL.Color(Color.white);
                    GL.Vertex(p + f); GL.Vertex(p - f * 0.6f + side);
                    GL.Vertex(p + f); GL.Vertex(p - f * 0.6f - side);
                    GL.Vertex(p - f * 0.6f + side); GL.Vertex(p - f * 0.6f - side);
                });
            }
            var waiting = _game.Waiting;
            foreach (var s in net.Stops.Values)
            {
                var q = _map.ToMap(s.nodeX, s.nodeY);
                int n = waiting.Count(r => r.Origin == s.name);
                bool charger = s.name == _game.Settings.ChargerStop;
                var dot = new Rect(q.x - 6f, q.y - 6f, 12f, 12f);
                GUI.color = charger ? new Color(0.35f, 0.85f, 0.45f) : new Color(0.25f, 0.5f, 0.95f);
                GUI.DrawTexture(dot, _white);
                GUI.color = Color.white;
                string label = s.name + (n > 0 ? $" ·{n}" : "");
                GUI.Label(new Rect(q.x + 7f, q.y - 9f, 70f, 18f), label, _small);
            }
            // Legend: the charger's dot is green.
            GUI.color = new Color(0.35f, 0.85f, 0.45f);
            GUI.DrawTexture(new Rect(_mapArea.x + 8f, _mapArea.yMax - 17f, 10f, 10f), _white);
            GUI.color = Color.white;
            GUI.Label(new Rect(_mapArea.x + 22f, _mapArea.yMax - 22f, 120f, 18f), $"charger ({_game.Settings.ChargerStop})", _small);
            var e = Event.current;
            if (e.type == EventType.MouseDown && e.button == 0 && _mapArea.Contains(e.mousePosition))
            {
                var stop = _map.StopAt(e.mousePosition, 10f);
                if (stop != null) { _pickFrom = stop; e.Use(); }
            }
        }

        // -------------------------------------------------------- bottom bar

        private void BottomBar(float w, float h)
        {
            float bh = BarHeight;
            var bar = new Rect(0f, h - bh, w, bh);
            GUI.Box(bar, GUIContent.none, _box);
            float x = 14f, y = h - bh + 10f;
            // Battery gauge, its label on the line above.
            double soc = _car.State.soc;
            GUI.Label(new Rect(x, y - 4f, 220f, 18f), $"Battery {100 * soc:0} %" + (_game.Brain.Mode == ShuttleMode.Charging ? " · charging" : ""), _small);
            var g = new Rect(x, y + 16f, 150f, 14f);
            GUI.color = new Color(0.2f, 0.22f, 0.26f);
            GUI.DrawTexture(g, _white);
            var c = soc > 0.5 ? new Color(0.3f, 0.78f, 0.42f) : soc > 0.25 ? new Color(0.95f, 0.7f, 0.2f) : new Color(0.92f, 0.3f, 0.25f);
            if (_game.Brain.Mode == ShuttleMode.Charging) c.a = 0.6f + 0.4f * Mathf.Sin(Time.unscaledTime * 4f);
            GUI.color = c;
            GUI.DrawTexture(new Rect(g.x, g.y, g.width * (float)Math.Max(0.0, Math.Min(1.0, soc)), g.height), _white);
            GUI.color = Color.white;
            x += 168f;
            GUI.Label(new Rect(x, y, 90f, 28f), $"Seats {_game.Brain.Aboard.Count}/{_game.Settings.Capacity}", _text);
            x += 96f;
            GUI.Label(new Rect(x, y, Mathf.Max(260f, w * 0.26f), 28f), _game.Brain.Status, _text);
            x += Mathf.Max(260f, w * 0.26f) + 8f;
            // The request queue, oldest first.
            float chipsEnd = w - 96f;
            foreach (var r in _game.Waiting.OrderBy(r => r.RaisedAt))
            {
                string t = $"{r.Origin} → {r.Destination} · {Clock(_game.Now - r.RaisedAt)}";
                float cw = _chip.CalcSize(new GUIContent(t)).x + 14f;
                if (x + cw > chipsEnd) { GUI.Label(new Rect(x, y + 4f, 40f, 20f), "…", _text); break; }
                GUI.Label(new Rect(x, y + 4f, cw, 22f), t, r.State == RequestState.Assigned ? _chipAssigned : _chip);
                x += cw + 6f;
            }
            if (GUI.Button(new Rect(w - 86f, y, 74f, 28f), "Settings", _btn)) _settings = !_settings;
        }

        // ------------------------------------------------- destination picker

        private void Picker(float w, float h)
        {
            var net = _game.Routes;
            var names = net.StopNames().Where(s => net.Stops[s].node != net.Stops[_pickFrom].node).ToList();
            float pw = 320f, ph = 70f + 34f * Mathf.Ceil(names.Count / 4f);
            GUILayout.BeginArea(new Rect((w - pw) / 2f, (h - ph) / 2f, pw, ph), _box);
            GUILayout.Label($"A student at {_pickFrom} wants to go to…", _text);
            for (int i = 0; i < names.Count; i += 4)
            {
                GUILayout.BeginHorizontal();
                foreach (var n in names.Skip(i).Take(4))
                    if (GUILayout.Button(n, _btn, GUILayout.Width(68f))) { _game.RaiseRequest(_pickFrom, n); _pickFrom = null; }
                GUILayout.EndHorizontal();
                if (_pickFrom == null) break;
            }
            if (_pickFrom != null && GUILayout.Button("Cancel", _btn)) _pickFrom = null;
            GUILayout.EndArea();
        }

        // ------------------------------------------------------------ settings

        private void SettingsPanel(float w, float h)
        {
            var s = _game.Settings;
            float pw = 340f, ph = 330f;
            GUILayout.BeginArea(new Rect(w - pw - 12f, h - BarHeight - ph - 8f, pw, ph), _box);
            GUILayout.Label("Settings", _big);
            s.RequestInterval = Slider($"A request every {s.RequestInterval:0} s", s.RequestInterval, 5.0, 60.0);
            _game.Spawner.RatePerSecond = 1.0 / s.RequestInterval;
            s.Capacity = (int)Math.Round(Slider($"Seats {s.Capacity}", s.Capacity, 1, 12));
            double minutes = Slider($"A charge lasts {s.BatteryMinutesOfDriving:0.0} min of driving", s.BatteryMinutesOfDriving, 1.0, 15.0);
            if (Math.Abs(minutes - s.BatteryMinutesOfDriving) > 1e-9) { s.BatteryMinutesOfDriving = minutes; _game.ResizeBattery(); }
            s.ChargeSeconds10To95 = Slider($"Charging 10 → 95 % takes {s.ChargeSeconds10To95:0} s", s.ChargeSeconds10To95, 5.0, 120.0);
            s.GiveUpAfter = Slider($"Students give up after {s.GiveUpAfter:0} s", s.GiveUpAfter, 30.0, 600.0);
            GUILayout.Space(6f);
            GUILayout.BeginHorizontal();
            if (GUILayout.Button(_paused ? "Resume" : "Pause", _btn)) { _paused = !_paused; Apply(); }
            foreach (var k in new[] { 1f, 2f, 4f })
                if (GUILayout.Button($"×{k:0}", Mathf.Approximately(_scale, k) ? _btnOn : _btn)) { _scale = k; Apply(); }
            GUILayout.EndHorizontal();
            GUILayout.Label("F1: the car's and the autopilot's debug panels", _small);
            if (GUILayout.Button("Close", _btn)) _settings = false;
            GUILayout.EndArea();
        }

        private void Apply() => Time.timeScale = _paused ? 0f : _scale;

        private double Slider(string label, double v, double lo, double hi)
        {
            GUILayout.Label(label, _small);
            return GUILayout.HorizontalSlider((float)v, (float)lo, (float)hi);
        }

        // ------------------------------------------------------------- helpers

        /// <summary>The label over a stop's waiting students: a line per destination,
        /// oldest first, with how many and the longest wait.</summary>
        public static string WaitLabel(IEnumerable<RideRequest> waiting, double now) =>
            string.Join("\n", waiting.GroupBy(r => r.Destination)
                .Select(g => (dest: g.Key, n: g.Count(), first: g.Min(r => r.RaisedAt)))
                .OrderBy(g => g.first)
                .Select(g => $"{(g.n > 1 ? g.n + " " : "")}\u2192 {g.dest} \u00b7 {Clock(now - g.first)}"));

        /// <summary>The seat count over the shuttle.</summary>
        public static string SeatLabel(int aboard, int capacity) => $"{aboard}/{capacity}";

        private static string Clock(double seconds)
        {
            int s = Math.Max(0, (int)Math.Round(seconds));
            return $"{s / 60}:{s % 60:00}";
        }

        private void Lines(Action draw)
        {
            if (_lines == null)
            {
                _lines = new Material(Shader.Find("Hidden/Internal-Colored")) { hideFlags = HideFlags.HideAndDontSave };
                _lines.SetInt("_ZTest", (int)UnityEngine.Rendering.CompareFunction.Always);
            }
            _lines.SetPass(0);
            GL.PushMatrix();
            GL.LoadPixelMatrix(0f, Screen.width, Screen.height, 0f);
            GL.Begin(GL.LINES);
            draw();
            GL.End();
            GL.PopMatrix();
        }

        private void Styles()
        {
            if (_box != null) return;
            _white = Texture2D.whiteTexture;
            Texture2D Tex(Color c)
            {
                var t = new Texture2D(1, 1) { hideFlags = HideFlags.HideAndDontSave };
                t.SetPixel(0, 0, c);
                t.Apply();
                return t;
            }
            _box = new GUIStyle { normal = { background = Tex(new Color(0.08f, 0.09f, 0.11f, 0.86f)) }, padding = new RectOffset(12, 12, 8, 8) };
            _big = new GUIStyle(GUI.skin.label) { fontSize = 20, fontStyle = FontStyle.Bold, normal = { textColor = Color.white } };
            _text = new GUIStyle(GUI.skin.label) { fontSize = 14, normal = { textColor = new Color(0.92f, 0.93f, 0.95f) } };
            _small = new GUIStyle(GUI.skin.label) { fontSize = 12, normal = { textColor = new Color(0.85f, 0.87f, 0.9f) } };
            _chip = new GUIStyle(GUI.skin.label)
            {
                fontSize = 12, alignment = TextAnchor.MiddleCenter, padding = new RectOffset(7, 7, 2, 2),
                normal = { background = Tex(new Color(0.22f, 0.25f, 0.32f)), textColor = Color.white },
            };
            _chipAssigned = new GUIStyle(_chip) { normal = { background = Tex(new Color(0.2f, 0.42f, 0.78f)), textColor = Color.white } };
            _tag = new GUIStyle(GUI.skin.label)
            {
                fontSize = 11, alignment = TextAnchor.MiddleCenter, padding = new RectOffset(6, 6, 2, 2), wordWrap = false,
                normal = { background = Tex(new Color(0.08f, 0.09f, 0.11f, 0.7f)), textColor = Color.white },
            };
            _btn = new GUIStyle(GUI.skin.button) { fontSize = 13 };
            _btnOn = new GUIStyle(_btn) { normal = { background = Tex(new Color(0.2f, 0.42f, 0.78f)), textColor = Color.white } };
        }
    }
}
