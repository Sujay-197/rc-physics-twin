// The campus shuttle fleet (Docs/specs/2026-10-25-shuttle-fleet-design.md):
// N buses, each a ShuttleGame on its own TwinCar and TwinAutopilot, sharing
// the requests, the dispatcher, the score, two chargers and the road. Each
// physics step it ticks every bus, then the dispatcher, then the traffic rules
// (ADR 0012), which hold each bus back from the others before its autopilot
// steps. It also keeps the students waiting at the stops.
//
// Bus 1 is the scene's car; the others are copies of it, placed at the stops
// farthest from the buses already placed. -buses <n> on the command line (or
// the settings panel at run time) sets how many; fewer retires the
// highest-numbered bus once it has delivered everyone aboard.
using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace RcTwin
{
    [DefaultExecutionOrder(-60)]   // before the autopilots (-50): each tick's holds reach their command
    public sealed class ShuttleFleet : MonoBehaviour
    {
        /// <summary>Tests: start with this many buses, whatever the command line says (0: not set).</summary>
        public static int BusesOverride;

        /// <summary>Each bus's colour, Bus 1 first: the HUD's accent blue, then distinct hues.</summary>
        public static readonly Color[] Palette =
        {
            new Color(0.31f, 0.6f, 1f), new Color(1f, 0.6f, 0.2f), new Color(0.36f, 0.8f, 0.45f),
            new Color(0.75f, 0.45f, 0.95f), new Color(0.95f, 0.35f, 0.4f), new Color(0.2f, 0.8f, 0.8f),
        };

        public static Color Colour(int number) => Palette[(Math.Max(1, number) - 1) % Palette.Length];

        /// <summary>Buses keep left, this far off the centreline (ADR 0012) [m]: the largest offset, in
        /// 5 cm steps, at which the shipped driver touches no kerb on any campus trip (at 0.25 it touches
        /// one on 34 of the 90). Oncoming buses pass 0.4 m apart, 0.125 m between their footprints.</summary>
        public const double LaneOffset = 0.2;
        /// <summary>A bus held behind a parked one within this of its stop serves the stop from there [m].</summary>
        public const double Berth = 1.0;
        /// <summary>Held this long by an idle bus, that bus makes room [s].</summary>
        public const double MakeRoomAfter = 3.0;
        /// <summary>Held to a standstill this long, a bus is stuck and towed [s].</summary>
        public const double StuckAfter = 20.0;
        /// <summary>A bus's present arc is checked this far past its braking distance [m], as its path is (TrafficRules.Margin).</summary>
        public const double MotionMargin = 1.0;
        /// <summary>Spacing of the path samples the traffic rules check [m].</summary>
        public const double PathStep = 0.05;

        public int seed = 1;

        public ShuttleSettings Settings { get; } = new ShuttleSettings();
        public RequestSpawner Spawner { get; private set; }
        public ShuttleScore Score { get; private set; }
        public Dispatcher Dispatcher { get; private set; }
        public TrafficRules Traffic { get; } = new TrafficRules();
        public RouteTravelModel Travel { get; private set; }
        public CampusRoutes Routes { get; private set; }
        public CameraModes Cameras { get; private set; }
        /// <summary>The buses, Bus 1 first.</summary>
        public IReadOnlyList<ShuttleGame> Buses => _buses;
        public double Now { get; private set; }
        /// <summary>Buses towed because they were stuck behind another (a fault, ADR 0012).</summary>
        public int StuckTows { get; private set; }
        /// <summary>Times an idle bus drove off to make room for another.</summary>
        public int MadeRoom { get; private set; }
        /// <summary>Times two buses' footprints overlapped (a fault, ADR 0012).</summary>
        public int Overlaps => Traffic.Overlaps;
        /// <summary>The longest any bus has been held to a standstill [s].</summary>
        public double LongestHold { get; private set; }

        /// <summary>The bus the camera follows (or last followed): the bottom bar's.</summary>
        public ShuttleGame Followed =>
            _buses.Count == 0 ? null : _buses[Math.Min(_buses.Count, Math.Max(1, Cameras?.FollowedBus ?? 1)) - 1];

        /// <summary>Requests not yet picked up: waiting, or assigned to a bus.</summary>
        public IReadOnlyList<RideRequest> Waiting =>
            Dispatcher == null ? new List<RideRequest>()
                : Dispatcher.Open.Where(r => r.State == RequestState.Waiting || r.State == RequestState.Assigned).ToList();

        private readonly List<ShuttleGame> _buses = new List<ShuttleGame>();
        private readonly List<TrafficAgent> _agents = new List<TrafficAgent>();
        private readonly List<double> _heldFor = new List<double>();
        private readonly Dictionary<RideRequest, StudentFigure> _figures = new Dictionary<RideRequest, StudentFigure>();
        private GameObject _template;
        private System.Random _look;

        // ----------------------------------------------------------- game mode

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Hook()
        {
            SceneManager.sceneLoaded -= Attach;
            SceneManager.sceneLoaded += Attach;
        }

        private static void Attach(Scene scene, LoadSceneMode mode)
        {
            if (!ShuttleGame.ShouldRun() || FindFirstObjectByType<ShuttleFleet>() != null) return;
            if (FindObjectsByType<TwinAutopilot>(FindObjectsSortMode.None).Any(p => p.network != null))
                new GameObject("Shuttle fleet").AddComponent<ShuttleFleet>();
        }

        // --------------------------------------------------------------- setup

        private void Awake()
        {
            var first = FindObjectsByType<TwinAutopilot>(FindObjectsSortMode.InstanceID)
                .FirstOrDefault(p => p.network != null && p.GetComponent<ShuttleGame>() == null);
            if (first == null || first.Routes == null) { enabled = false; return; }
            Routes = first.Routes;
            Travel = new RouteTravelModel(Routes, Settings.Cruise);
            Score = new ShuttleScore(Settings);
            Dispatcher = new Dispatcher(new ShuttleBrain[0]);
            var stops = Routes.Stops.Values.OrderBy(s => s.name, StringComparer.Ordinal).Select(s => (s.name, s.node, s.weight)).ToList();
            Spawner = new RequestSpawner(stops, seed) { RatePerSecond = 1.0 / Settings.RequestInterval };
            _look = new System.Random(seed);

            int n = BusesOverride > 0 ? BusesOverride : Arg("-buses") is string a &&
                    int.TryParse(a, NumberStyles.Integer, CultureInfo.InvariantCulture, out int k) ? k : Settings.Buses;
            Settings.Buses = Math.Max(1, Math.Min(ShuttleSettings.MaxBuses, n));

            // The other buses are copies of the scene's car as it was, kept inactive.
            var holder = new GameObject("Bus template");
            holder.SetActive(false);
            holder.transform.SetParent(transform, false);
            _template = Instantiate(first.gameObject, holder.transform);
            _template.name = "Bus template";

            Join(first, CampusSceneHome);
            while (_buses.Count < Settings.Buses) Spawn();

            Cameras = new CameraModes(_buses.Count);
            if (Arg("-camera") is string cam && CameraModes.TryParse(cam, out var m)) Cameras.Set(m);
            gameObject.AddComponent<ShuttleHud>();
            var c = Camera.main;
            if (c != null && c.GetComponent<FleetCamera>() == null) c.gameObject.AddComponent<FleetCamera>().Fleet = this;
        }

        /// <summary>The stop the campus scene puts the car at (CampusSceneBuilder.HomeStop).</summary>
        public const string CampusSceneHome = "S3";

        private static string Arg(string name)
        {
            var args = Environment.GetCommandLineArgs();
            int i = Array.IndexOf(args, name);
            return i >= 0 && i + 1 < args.Length ? args[i + 1] : null;
        }

        // A bus joins at `stop`, in its lane, facing along the stop's first road.
        private void Join(TwinAutopilot pilot, string stop)
        {
            var bus = pilot.gameObject.AddComponent<ShuttleGame>();
            bus.Setup(this, _buses.Count == 0 ? 1 : _buses.Max(b => b.Number) + 1, Travel);
            string toward = Routes.StopNames().First(s => Routes.Stops[s].node != Routes.Stops[stop].node);
            var (x, y, yaw) = FreeBerth(bus, stop, toward);
            var car = bus.Car;
            car.homeX = x;
            car.homeY = y;
            car.homeYaw = yaw;
            if (car.State.x != 0.0 || car.State.y != 0.0 || Time.frameCount > 0) car.ResetTo(x, y, yaw);
            _buses.Add(bus);
            _heldFor.Add(0.0);
            _agents.Add(new TrafficAgent());
            Dispatcher.Add(bus.Brain);
        }

        // One more bus, a copy of the scene's car, at the stop farthest from the others.
        private ShuttleGame Spawn()
        {
            var go = Instantiate(_template);
            go.transform.SetParent(null);
            go.GetComponent<TwinAutopilot>().readCommandLine = false;
            go.SetActive(true);
            Join(go.GetComponent<TwinAutopilot>(), SpawnStop());
            return _buses[_buses.Count - 1];
        }

        // The stop farthest by route from every stop a bus already stands at or heads to.
        private string SpawnStop()
        {
            var taken = new List<string>();
            foreach (var b in _buses)
            {
                var (near, d) = Routes.Nearest(b.Car.homeX, b.Car.homeY);
                if (b.Car.State.x != 0.0 || b.Car.State.y != 0.0) (near, d) = Routes.Nearest(b.Car.State.x, b.Car.State.y);
                taken.Add(near);
                if (b.Brain.Plan.Stops.Count > 0) taken.Add(b.Brain.Plan.Stops[0].Stop);
            }
            string best = null;
            double bd = double.NegativeInfinity;
            foreach (var s in Routes.StopNames())
            {
                double d = taken.Count == 0 ? 0.0 : taken.Min(t => Routes.Stops[t].node == Routes.Stops[s].node ? 0.0 : Travel.Distance(t, s));
                if (d > bd) { bd = d; best = s; }
            }
            return best;
        }

        /// <summary>Run with n buses: more spawn at once; fewer retire the highest-numbered
        /// ones, which leave once everyone aboard is delivered.</summary>
        public void SetBuses(int n)
        {
            n = Math.Max(1, Math.Min(ShuttleSettings.MaxBuses, n));
            Settings.Buses = n;
            int active = _buses.Count(b => !b.Brain.Retiring);
            foreach (var b in _buses.Where(b => b.Brain.Retiring))
                if (active < n) { b.Brain.Unretire(); active++; }
            while (active < n) { Spawn(); active++; }
            for (int i = _buses.Count - 1; i >= 0 && active > n; i--)
                if (!_buses[i].Brain.Retiring) { _buses[i].Brain.Retire(); active--; }
            Cameras.SetBuses(_buses.Count);
        }

        private void RemoveRetired()
        {
            for (int i = _buses.Count - 1; i >= 0; i--)
            {
                var b = _buses[i];
                if (!b.Brain.Retired) continue;
                Dispatcher.Remove(b.Brain);
                _buses.RemoveAt(i);
                _heldFor.RemoveAt(i);
                _agents.RemoveAt(i);
                Destroy(b.gameObject);
                Cameras.SetBuses(_buses.Count);
            }
        }

        // ---------------------------------------------------------------- tick

        private void FixedUpdate()
        {
            if (Dispatcher == null || _buses.Count == 0) return;
            double dt = Time.fixedDeltaTime;
            Now += dt;
            foreach (var r in Spawner.Tick(Now, dt)) Offer(r);
            foreach (var b in _buses) b.Step(dt);
            Dispatcher.Tick(Now, Score);
            TrafficTick(dt);
            RemoveRetired();
            foreach (var kv in _figures.Where(kv => kv.Key.State == RequestState.GaveUp).ToList())
            {
                if (kv.Value != null) kv.Value.WalkTo(kv.Value.transform.position + kv.Value.transform.right * 0.6f, vanish: true);
                _figures.Remove(kv.Key);
            }
        }

        // The traffic rules over every bus, then what follows from a long hold:
        // serving a stop from behind a parked bus, an idle bus making room, a stuck tow.
        private void TrafficTick(double dt)
        {
            for (int i = 0; i < _buses.Count; i++)
            {
                var b = _buses[i];
                var a = _agents[i];
                var st = b.Car.State;
                var fp = b.Car.Footprint;
                a.Id = b.Number;
                a.X = st.x; a.Y = st.y; a.Yaw = st.yaw;
                a.Speed = Math.Abs(st.vx);
                a.Manoeuvring = b.Pilot.TurningRound;
                a.HalfLength = fp.halfLength; a.HalfWidth = fp.halfWidth; a.CentreOffset = fp.centreOffset;
                b.Pilot.PathAhead(Traffic.Horizon(a.Speed), PathStep, a.Path);
                // The arc the steering is on now (kinematic): it turns before the yaw rate does.
                var p = b.Car.Params;
                a.Arc(st.vx, Math.Tan(st.steer) / (p.lf + p.lr), a.Speed * a.Speed / (2.0 * Traffic.Decel) + MotionMargin, PathStep);
            }
            int overlaps = Traffic.Overlaps;
            Traffic.Update(_agents);
            if (Traffic.Overlaps > overlaps)
                Debug.LogWarning($"ShuttleFleet: buses overlap at {Now:0.0} s: " + string.Join("; ", _buses.Select(Describe)));
            for (int i = 0; i < _buses.Count; i++)
            {
                var b = _buses[i];
                var pilot = b.Pilot;
                pilot.HoldShort = Traffic.Hold[i];
                bool standing = pilot.State == TwinAutopilot.Status.Driving && Math.Abs(b.Car.State.vx) < 0.05 &&
                                Traffic.Hold[i] < TwinAutopilot.TurnHoldDistance;
                _heldFor[i] = standing ? _heldFor[i] + dt : 0.0;
                LongestHold = Math.Max(LongestHold, _heldFor[i]);
                int j = Traffic.HeldBy[i];
                if (!standing || j < 0) continue;
                var other = _buses[j];
                if (_heldFor[i] > 0.5 && !pilot.TurningRound && !other.Driving &&
                    (pilot.Remaining <= Berth || other.AtStop == pilot.Destination && pilot.CurrentStop() == pilot.Destination))
                {
                    pilot.ArriveHere($"Arrived at {pilot.Destination}, behind {other.Label}.");
                    _heldFor[i] = 0.0;
                }
                else if (_heldFor[i] > MakeRoomAfter && other.Brain.Mode == ShuttleMode.Idle && other.Brain.Plan.Stops.Count == 0)
                {
                    if (RoomFor(other, b) is string stop && other.Brain.MakeRoom(stop)) MadeRoom++;
                }
                else if (_heldFor[i] > StuckAfter)
                {
                    var tow = other.Driving && other.Number > b.Number ? other : b;
                    var by = tow == b ? other : b;
                    tow.Pilot.Abandon($"Stuck behind {by.Label}: towed.");
                    StuckTows++;
                    _heldFor[i] = 0.0;
                    _heldFor[j] = 0.0;
                    Debug.LogWarning($"ShuttleFleet: {tow.Label} stuck behind {by.Label} for {StuckAfter:0} s; towed to its stop. " +
                                     $"{Describe(b)}; {Describe(other)}; held by {(Traffic.HeldBy[j] >= 0 ? _buses[Traffic.HeldBy[j]].Label : "none")}");
                }
            }
        }

        private static string Describe(ShuttleGame b)
        {
            var st = b.Car.State;
            return $"{b.Label} at ({st.x:0.00}, {st.y:0.00}) yaw {st.yaw:0.00} v {st.vx:0.00}, {b.Brain.Mode} ({b.Brain.Status}), " +
                   $"{b.Pilot.State}{(b.Pilot.TurningRound ? " turning" : "")}, hold {b.Pilot.HoldShort:0.00}: {b.Pilot.Message}";
        }

        // Where an idle bus can wait out of the held bus's way: the nearest stop
        // not on that bus's way, with no bus at it or bound for it.
        private string RoomFor(ShuttleGame idle, ShuttleGame held)
        {
            string from = idle.AtStop;
            if (from == null) return null;
            var avoid = new HashSet<string> { from };
            if (held.LegTo != null)
            {
                avoid.Add(held.LegTo);
                if (held.LegFrom != null) foreach (var (x, _) in Travel.StopsOnRoute(held.LegFrom, held.LegTo)) avoid.Add(x);
            }
            foreach (var ps in held.Brain.Plan.Stops) avoid.Add(ps.Stop);
            foreach (var b in _buses)
            {
                if (b == idle) continue;
                if (b.AtStop != null) avoid.Add(b.AtStop);
                if (b.Brain.Plan.Stops.Count > 0) avoid.Add(b.Brain.Plan.Stops[0].Stop);
            }
            return Routes.StopNames()
                .Where(s => !avoid.Contains(s) && Routes.Stops[s].node != Routes.Stops[from].node)
                .OrderBy(s => Travel.Distance(from, s)).FirstOrDefault();
        }

        // ------------------------------------------------------------ chargers

        /// <summary>The charger for `bus` from `from`: the nearest no other bus is
        /// charging at or bound for; the nearest if every one is.</summary>
        public string PickCharger(ShuttleGame bus, string from) =>
            BatteryPolicy.PickCharger(Settings, Travel, from,
                _buses.Where(b => b != bus && b.Brain != null && b.Brain.Charger != null).Select(b => b.Brain.Charger).ToList());

        // --------------------------------------------------------------- space

        /// <summary>The other buses' footprints, with the traffic clearance, for planning a turn round them.</summary>
        public IEnumerable<(double[] xy, bool closed)> ObstaclesFor(ShuttleGame bus)
        {
            foreach (var b in _buses)
            {
                if (b == bus) continue;
                var st = b.Car.State;
                var fp = b.Car.Footprint;
                double c = Math.Cos(st.yaw), s = Math.Sin(st.yaw);
                double cx = st.x + fp.centreOffset * c, cy = st.y + fp.centreOffset * s;
                double l = fp.halfLength + Traffic.Clearance, w = fp.halfWidth + Traffic.Clearance;
                yield return (new[]
                {
                    cx + l * c - w * s, cy + l * s + w * c, cx - l * c - w * s, cy - l * s + w * c,
                    cx - l * c + w * s, cy - l * s - w * c, cx + l * c + w * s, cy + l * s - w * c,
                }, true);
            }
        }

        /// <summary>Whether `bus` could be put at ISO (x, y, yaw) clear of every other bus and of
        /// where each is about to drive (its path and its arc, as the traffic rules last saw
        /// them), with the traffic clearance: a bus put there must not land in another's way.</summary>
        public bool PoseFree(ShuttleGame bus, double x, double y, double yaw)
        {
            var fp = bus.Car.Footprint;
            var me = new TrafficAgent { HalfLength = fp.halfLength, HalfWidth = fp.halfWidth, CentreOffset = fp.centreOffset };
            var pose = new PathPose(x, y, yaw, 0.0);
            for (int i = 0; i < _buses.Count; i++)
            {
                if (_buses[i] == bus) continue;
                var st = _buses[i].Car.State;
                var other = _agents[i];
                var ofp = _buses[i].Car.Footprint;
                other.HalfLength = ofp.halfLength; other.HalfWidth = ofp.halfWidth; other.CentreOffset = ofp.centreOffset;
                if (TrafficRules.Overlap(pose, me, new PathPose(st.x, st.y, st.yaw, 0.0), other, Traffic.Clearance)) return false;
                foreach (var q in other.Path) if (TrafficRules.Overlap(pose, me, q, other, Traffic.Clearance)) return false;
                foreach (var q in other.Motion) if (TrafficRules.Overlap(pose, me, q, other, Traffic.Clearance)) return false;
            }
            return true;
        }

        /// <summary>Where to put `bus` at `stop` (a tow, a new bus): where a bus arriving at the stop
        /// would stand (at the end of a lane into it, short of the crossing road at a junction), or a
        /// berth or two further back along that lane; the way in that faces `toward` first; the first
        /// clear of the other buses and of where they are about to drive (PoseFree).</summary>
        public (double x, double y, double yaw) FreeBerth(ShuttleGame bus, string stop, string toward)
        {
            double lane = bus.Pilot.laneOffset;
            double setback = Routes.Roads(stop) >= 3 ? bus.Pilot.junctionSetback : 0.0;
            var o = Routes.Route(stop, toward);
            double hOut = Math.Atan2(o[4] - o[1], o[3] - o[0]);
            var ways = new List<(double[] lane, double heading)>();
            foreach (var w in Routes.StopNames())
            {
                if (Routes.Stops[w].node == Routes.Stops[stop].node) continue;
                var l = Lanes.Offset(Routes.Route(w, stop), lane);
                int n = l.Length / 3;
                double h = Math.Atan2(l[3 * n - 2] - l[3 * n - 5], l[3 * n - 3] - l[3 * n - 6]);
                if (!ways.Any(x => Math.Abs(TwinObservation.Wrap(x.heading - h)) < 0.1)) ways.Add((l, h));
            }
            ways.Sort((p, q) => Math.Abs(TwinObservation.Wrap(p.heading - hOut)).CompareTo(Math.Abs(TwinObservation.Wrap(q.heading - hOut))));
            (double, double, double)? first = null;
            foreach (double back in new[] { 0.0, 0.6, 1.2 })
                foreach (var (l, _) in ways)
                {
                    var e = Lanes.Shorten(l, setback + back);
                    int n = e.Length / 3;
                    double x = e[3 * n - 3], y = e[3 * n - 2];
                    double yaw = Math.Atan2(e[3 * n - 2] - e[3 * n - 5], e[3 * n - 3] - e[3 * n - 6]);
                    first = first ?? (x, y, yaw);
                    if (PoseFree(bus, x, y, yaw)) return (x, y, yaw);
                }
            return first ?? (o[0], o[1], hOut);
        }

        // ------------------------------------------------------------ requests

        /// <summary>A request the person watching raised.</summary>
        public RideRequest RaiseRequest(string origin, string destination)
        {
            var r = Spawner.Raise(origin, destination, Now);
            Offer(r);
            return r;
        }

        private void Offer(RideRequest r)
        {
            Dispatcher.Offer(r);
            int n = _figures.Count(kv => kv.Key.Origin == r.Origin && kv.Value != null);
            _figures[r] = StudentFigure.Make(r, KerbSlot(r.Origin, n), Clothes());
        }

        internal void OnBoarded(ShuttleGame bus, RideRequest r)
        {
            if (_figures.TryGetValue(r, out var f) && f != null) f.WalkTo(bus.Door(), vanish: true, maxTime: 3f);
            _figures.Remove(r);
        }

        internal void OnAlighted(ShuttleGame bus, RideRequest r)
        {
            var f = StudentFigure.Make(r, bus.Door(), Clothes());
            f.WalkTo(KerbSlot(r.Destination, 0) + Away(r.Destination) * 0.5f, vanish: true);
        }

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
            var net = Routes;
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
            var node = Routes.Stops[stop];
            var road = TwinCar.ToUnity(node.nodeX, node.nodeY, 0.0);
            var d = slot - road;
            d.y = 0f;
            return d.normalized;
        }
    }
}
