// The campus shuttle game's screen, layout C (Docs/specs/2026-10-21-campus-shuttle-design.md),
// for a fleet (Docs/specs/2026-10-25-shuttle-fleet-design.md): the score and a
// row per bus top-left, the camera mode top centre, a minimap top-right with
// every bus, a bottom bar with the followed bus's battery, seats and what it
// is doing, and the request queue. Click a stop, on the minimap or in the
// world, to raise a request; click a bus's row to follow it; the Settings
// button opens the settings. Its look (Inter, rounded translucent panels,
// icons) is HudSkin's.
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

    [RequireComponent(typeof(ShuttleFleet))]
    public sealed class ShuttleHud : MonoBehaviour
    {
        private ShuttleFleet _game;
        private Minimap _map;
        private Rect _mapArea;
        private Material _lines;
        private GUIStyle _box, _big, _text, _small, _chip, _chipAssigned, _btn, _btnOn, _tag, _gauge, _caption;
        private GUIStyle _textCentred, _smallCentred, _captionCentred;
        private Texture2D _white, _dot;
        private string _pickFrom;                 // destination picker open for this origin
        private bool _settings;
        private float _scale = 1f;
        private bool _paused;

        private ShuttleGame Bus => _game.Followed;


        private void Start() => _game = GetComponent<ShuttleFleet>();

        private void Update()
        {
            var m = Mouse.current;
            if (_game.Routes == null || Camera.main == null || m == null || !m.leftButton.wasPressedThisFrame) return;
            Vector2 screen = m.position.ReadValue();
            var mouse = new Vector2(screen.x, Screen.height - screen.y);
            if (_mapArea.Contains(mouse) || mouse.y > Screen.height - BarHeight || _pickFrom != null || _settings ||
                _fleetArea.Contains(mouse) || _chipArea.Contains(mouse)) return;
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
            if (_game == null || _game.Routes == null || _game.Buses.Count == 0) return;
            Styles();
            float w = Screen.width, h = Screen.height;
            float mapW = Mathf.Max(220f, w * 0.22f), mapH = mapW * 0.75f;
            var area = new Rect(w - mapW - 12f, 12f, mapW, mapH);
            if (_map == null || area != _mapArea) { _mapArea = area; _map = new Minimap(_game.Routes, area); }

            WorldLabels();
            ScorePanel();
            FleetPanel();
            CameraChip(w);
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
            foreach (var b in _game.Buses)
                Tag(cam, b.transform.position + Vector3.up * 0.28f, $"{b.Label} · {SeatLabel(b.Brain.Aboard.Count, _game.Settings.Capacity)}");
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
            var r = new Rect(12f, 12f, 350f, 82f);
            GUI.Box(r, GUIContent.none, _box);
            Icon(HudIcon.Score, new Rect(r.x + 16f, r.y + 15f, 26f, 26f), HudSkin.Gold);
            GUI.Label(new Rect(r.x + 50f, r.y + 9f, r.width - 60f, 36f), $"Score {s.Points:N0}", _big);
            Icon(HudIcon.Clock, new Rect(r.x + 19f, r.y + 53f, 14f, 14f), HudSkin.Muted);
            GUI.Label(new Rect(r.x + 40f, r.y + 48f, r.width - 50f, 24f),
                      $"Delivered {s.Delivered} · Avg wait {Clock(s.AverageWait)} · Gave up {s.GaveUp}", _text);
        }

        // ------------------------------------------------------------- fleet

        private Rect _fleetArea, _chipArea;

        // A row per bus: its colour, name, battery, seats, deliveries and what it is doing.
        // Click one to follow it.
        private void FleetPanel()
        {
            const float rowH = 22f;
            var buses = _game.Buses;
            bool faults = _game.StuckTows > 0 || _game.Overlaps > 0;
            _fleetArea = new Rect(12f, 102f, 440f, 16f + rowH * buses.Count + (faults ? rowH : 0f));
            GUI.Box(_fleetArea, GUIContent.none, _box);
            float y = _fleetArea.y + 8f;
            for (int i = 0; i < buses.Count; i++, y += rowH)
            {
                var b = buses[i];
                var row = new Rect(_fleetArea.x + 6f, y, _fleetArea.width - 12f, rowH);
                if (b == Bus && _game.Cameras.Current.Kind == CameraKind.Bus)
                {
                    GUI.color = new Color(1f, 1f, 1f, 0.08f);
                    GUI.DrawTexture(row, _white);
                }
                GUI.color = b.Colour;
                GUI.DrawTexture(new Rect(row.x + 6f, y + 6f, 10f, 10f), _dot);
                GUI.color = Color.white;
                double soc = b.Car.State.soc;
                string text = $"{b.Label}   {100 * soc:0} %   {b.Brain.Aboard.Count}/{_game.Settings.Capacity}   {b.BusScore.Delivered} delivered   " +
                              (b.Brain.Retiring ? "retiring · " : "") + b.Brain.Status;
                GUI.Label(new Rect(row.x + 22f, y + 2f, row.width - 24f, rowH), text, _small);
                if (GUI.Button(row, GUIContent.none, GUIStyle.none)) _game.Cameras.Follow(i + 1);
            }
            if (faults)
                GUI.Label(new Rect(_fleetArea.x + 28f, y + 2f, _fleetArea.width - 30f, rowH),
                          $"Stuck tows {_game.StuckTows} · overlaps {_game.Overlaps}", _small);
        }

        // The camera's mode, between buttons that step it (C and Shift+C do the same).
        private void CameraChip(float w)
        {
            var modes = _game.Cameras;
            var m = modes.Current;
            string name = m.Kind == CameraKind.Bus ? _game.Buses[Math.Min(m.Bus, _game.Buses.Count) - 1].Label
                : m.Kind == CameraKind.Stop ? $"Stop {Camera.main?.GetComponent<FleetCamera>()?.Stop ?? m.Stop}"
                : m.Kind == CameraKind.Route ? $"Route ({Bus.Label})" : m.ToString();
            string label = $"Camera · {name}";
            float lw = Mathf.Max(170f, _text.CalcSize(new GUIContent(label)).x + 24f);
            _chipArea = new Rect((w - lw) / 2f - 40f, 12f, lw + 80f, 36f);
            GUI.Box(_chipArea, GUIContent.none, _box);
            if (GUI.Button(new Rect(_chipArea.x + 6f, _chipArea.y + 5f, 30f, 26f), "‹", _btn)) modes.Previous();
            GUI.Label(new Rect(_chipArea.x + 40f, _chipArea.y + 7f, lw, 24f), label, _textCentred);
            if (GUI.Button(new Rect(_chipArea.xMax - 36f, _chipArea.y + 5f, 30f, 26f), "›", _btn)) modes.Next();
            GUI.Label(new Rect(_chipArea.x, _chipArea.yMax + 2f, _chipArea.width, 16f), "C · Shift+C · 1–6 follow a bus", _captionCentred);
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
                    GL.Color(new Color(0.62f, 0.66f, 0.72f));
                    foreach (var c in net.Centrelines())
                        for (int i = 3; i + 1 < c.Length; i += 3)
                        {
                            GL.Vertex(_map.ToMap(c[i - 3], c[i - 2]));
                            GL.Vertex(_map.ToMap(c[i], c[i + 1]));
                        }
                    // The followed bus's planned path, stop to stop along the routes.
                    GL.Color(Bus.Colour);
                    string at = Bus.AtStop ?? Bus.LegFrom;
                    foreach (var ps in Bus.Brain.Plan.Stops)
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
                    // Each bus: an arrow along its heading, in its colour.
                    foreach (var b in _game.Buses)
                    {
                        var st = b.Car.State;
                        var p = _map.ToMap(st.x, st.y);
                        var f = _map.ToMap(st.x + Math.Cos(st.yaw) * 1.2, st.y + Math.Sin(st.yaw) * 1.2) - p;
                        var side = new Vector2(-f.y, f.x) * 0.5f;
                        GL.Color(b.Colour);
                        GL.Vertex(p + f); GL.Vertex(p - f * 0.6f + side);
                        GL.Vertex(p + f); GL.Vertex(p - f * 0.6f - side);
                        GL.Vertex(p - f * 0.6f + side); GL.Vertex(p - f * 0.6f - side);
                    }
                });
            }
            foreach (var b in _game.Buses)
            {
                var q = _map.ToMap(b.Car.State.x, b.Car.State.y);
                GUI.Label(new Rect(q.x - 18f, q.y + 4f, 36f, 16f), b.Number.ToString(), _smallCentred);
            }
            var waiting = _game.Waiting;
            foreach (var s in net.Stops.Values)
            {
                var q = _map.ToMap(s.nodeX, s.nodeY);
                int n = waiting.Count(r => r.Origin == s.name);
                bool charger = Array.IndexOf(_game.Settings.Chargers, s.name) >= 0;
                GUI.color = new Color(1f, 1f, 1f, 0.9f);
                GUI.DrawTexture(new Rect(q.x - 7f, q.y - 7f, 14f, 14f), _dot);
                GUI.color = charger ? HudSkin.Good : HudSkin.Accent;
                GUI.DrawTexture(new Rect(q.x - 5f, q.y - 5f, 10f, 10f), _dot);
                GUI.color = Color.white;
                string label = s.name + (n > 0 ? $" ·{n}" : "");
                GUI.Label(new Rect(q.x + 7f, q.y - 9f, 70f, 18f), label, _small);
            }
            // Legend: the charger's dot is green.
            Icon(HudIcon.Bolt, new Rect(_mapArea.x + 10f, _mapArea.yMax - 20f, 12f, 12f), HudSkin.Good);
            GUI.Label(new Rect(_mapArea.x + 24f, _mapArea.yMax - 23f, 180f, 18f), $"chargers ({string.Join(", ", _game.Settings.Chargers)})", _small);
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
            var bar = new Rect(12f, h - bh + 2f, w - 24f, bh - 12f);   // floating inside the bar's band
            GUI.Box(bar, GUIContent.none, _box);
            float x = 28f, y = bar.y + (bar.height - 40f) / 2f;
            // Battery: icon, gauge, its label on the line above.
            double soc = Bus.Car.State.soc;
            bool charging = Bus.Brain.Mode == ShuttleMode.Charging;
            var c = soc > 0.5 ? HudSkin.Good : soc > 0.25 ? HudSkin.Warn : HudSkin.Bad;
            Icon(charging ? HudIcon.Bolt : HudIcon.Battery, new Rect(x, y + 9f, 24f, 24f), charging ? HudSkin.Good : HudSkin.Text);
            GUI.Label(new Rect(x + 32f, y - 2f, 220f, 18f), $"Battery {100 * soc:0} %" + (charging ? " · charging" : ""), _caption);
            var g = new Rect(x + 32f, y + 20f, 150f, 12f);
            GUI.color = new Color(1f, 1f, 1f, 0.14f);
            GUI.Box(g, GUIContent.none, _gauge);
            if (charging) c.a = 0.6f + 0.4f * Mathf.Sin(Time.unscaledTime * 4f);
            GUI.color = c;
            GUI.Box(new Rect(g.x, g.y, Mathf.Max(g.height, g.width * (float)Math.Max(0.0, Math.Min(1.0, soc))), g.height), GUIContent.none, _gauge);
            GUI.color = Color.white;
            x += 200f;
            Divider(x - 10f, bar);
            // Seats: icon, the count, its label above.
            Icon(HudIcon.Seat, new Rect(x + 4f, y + 8f, 24f, 24f), HudSkin.Text);
            GUI.Label(new Rect(x + 34f, y - 2f, 80f, 18f), "Seats", _caption);
            GUI.Label(new Rect(x + 34f, y + 14f, 80f, 24f), $"{Bus.Brain.Aboard.Count}/{_game.Settings.Capacity}", _text);
            x += 106f;
            Divider(x - 10f, bar);
            GUI.Label(new Rect(x, y - 2f, Mathf.Max(260f, w * 0.26f), 18f), Bus.Label, _caption);
            GUI.Label(new Rect(x, y + 14f, Mathf.Max(260f, w * 0.26f), 24f), Bus.Brain.Status, _text);
            x += Mathf.Max(260f, w * 0.26f) + 8f;
            y += 10f;
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
            if (GUI.Button(new Rect(w - 108f, y - 4f, 80f, 30f), "Settings", _btn)) _settings = !_settings;
        }

        private void Divider(float x, Rect bar)
        {
            GUI.color = new Color(1f, 1f, 1f, 0.1f);
            GUI.DrawTexture(new Rect(x, bar.y + 12f, 1f, bar.height - 24f), _white);
            GUI.color = Color.white;
        }

        private static void Icon(HudIcon icon, Rect r, Color c)
        {
            var was = GUI.color;
            GUI.color = c;
            GUI.DrawTexture(r, HudSkin.Icon(icon), ScaleMode.ScaleToFit);
            GUI.color = was;
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
            float pw = 340f, ph = 372f;
            GUILayout.BeginArea(new Rect(w - pw - 12f, h - BarHeight - ph - 8f, pw, ph), _box);
            GUILayout.Label("Settings", _big);
            int buses = (int)Math.Round(Slider($"Buses {s.Buses}", s.Buses, 1, ShuttleSettings.MaxBuses));
            if (buses != s.Buses) _game.SetBuses(buses);
            s.RequestInterval = Slider($"A request every {s.RequestInterval:0} s", s.RequestInterval, 5.0, 60.0);
            _game.Spawner.RatePerSecond = 1.0 / s.RequestInterval;
            s.Capacity = (int)Math.Round(Slider($"Seats {s.Capacity}", s.Capacity, 1, 12));
            double minutes = Slider($"A charge lasts {s.BatteryMinutesOfDriving:0.0} min of driving", s.BatteryMinutesOfDriving, 1.0, 15.0);
            if (Math.Abs(minutes - s.BatteryMinutesOfDriving) > 1e-9)
            {
                s.BatteryMinutesOfDriving = minutes;
                foreach (var b in _game.Buses) b.ResizeBattery();
            }
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
            _dot = HudSkin.Rounded(8, Color.white, Color.clear);
            Font regular = HudSkin.Font("Regular"), medium = HudSkin.Font("Medium"), semi = HudSkin.Font("SemiBold"), bold = HudSkin.Font("Bold");
            GUIStyle Label(Font f, int size, Color c, FontStyle fallback = FontStyle.Normal) => new GUIStyle(GUI.skin.label)
            {
                font = f, fontSize = size, fontStyle = f != null ? FontStyle.Normal : fallback, normal = { textColor = c },
            };
            GUIStyle Pill(Texture2D bg, int radius, Font f, int size) => new GUIStyle(GUI.skin.label)
            {
                font = f, fontSize = size, alignment = TextAnchor.MiddleCenter, wordWrap = false,
                border = new RectOffset(radius + 1, radius + 1, radius + 1, radius + 1), padding = new RectOffset(9, 9, 3, 3),
                normal = { background = bg, textColor = Color.white },
            };
            _box = new GUIStyle
            {
                normal = { background = HudSkin.Rounded(14, HudSkin.Panel, HudSkin.Edge) },
                border = new RectOffset(15, 15, 15, 15), padding = new RectOffset(14, 14, 10, 10),
            };
            _big = Label(bold, 22, Color.white, FontStyle.Bold);
            _text = Label(medium, 14, HudSkin.Text);
            _small = Label(semi, 11, HudSkin.Text);
            _caption = Label(regular, 11, HudSkin.Muted);
            _textCentred = new GUIStyle(_text) { alignment = TextAnchor.MiddleCenter };
            _smallCentred = new GUIStyle(_small) { alignment = TextAnchor.UpperCenter };
            _captionCentred = new GUIStyle(_caption) { alignment = TextAnchor.UpperCenter };
            _chip = Pill(HudSkin.Rounded(10, new Color(0.22f, 0.27f, 0.35f, 0.92f), new Color(1f, 1f, 1f, 0.1f)), 10, medium, 12);
            _chipAssigned = Pill(HudSkin.Rounded(10, new Color(0.2f, 0.46f, 0.86f, 0.95f), new Color(1f, 1f, 1f, 0.2f)), 10, medium, 12);
            _tag = Pill(HudSkin.Rounded(8, new Color(0.05f, 0.07f, 0.1f, 0.78f), new Color(1f, 1f, 1f, 0.12f)), 8, semi, 11);
            _tag.padding = new RectOffset(8, 8, 3, 3);
            _gauge = new GUIStyle { normal = { background = HudSkin.Rounded(5, Color.white, Color.clear) }, border = new RectOffset(6, 6, 6, 6) };
            _btn = new GUIStyle(GUI.skin.button)
            {
                font = medium, fontSize = 13, border = new RectOffset(9, 9, 9, 9), padding = new RectOffset(10, 10, 5, 5),
                normal = { background = HudSkin.Rounded(8, new Color(1f, 1f, 1f, 0.1f), new Color(1f, 1f, 1f, 0.16f)), textColor = HudSkin.Text },
                hover = { background = HudSkin.Rounded(8, new Color(1f, 1f, 1f, 0.18f), new Color(1f, 1f, 1f, 0.22f)), textColor = Color.white },
                active = { background = HudSkin.Rounded(8, HudSkin.Accent, new Color(1f, 1f, 1f, 0.25f)), textColor = Color.white },
            };
            _btnOn = new GUIStyle(_btn) { normal = { background = HudSkin.Rounded(8, HudSkin.Accent, new Color(1f, 1f, 1f, 0.25f)), textColor = Color.white } };
        }
    }
}
