// One bus of the campus shuttle game (Docs/specs/2026-10-21-campus-shuttle-design.md,
// Docs/specs/2026-10-25-shuttle-fleet-design.md): the twin's car as a shuttle.
// It is its ShuttleBrain's world: legs are TwinAutopilot trips (in the left
// lane, held back by the fleet's traffic rules, ADR 0012), the battery is
// TwinCar's (a small game pack), and students board at its door. ShuttleFleet
// owns what the buses share (requests, dispatcher, score, chargers, traffic)
// and ticks every bus once per physics step.
//
// Game mode: in the player the game runs unless the command line has -goto
// (the one-trip autopilot demo). In the editor it runs only when
// RunInEditor is set (tests), so the autopilot's own tests keep the car.
// ShuttleFleet attaches itself to the scene's autopilot after a scene loads.
using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace RcTwin
{
    [RequireComponent(typeof(TwinCar))]
    [RequireComponent(typeof(TwinAutopilot))]
    public sealed class ShuttleGame : MonoBehaviour, IShuttleWorld
    {
        /// <summary>Run the game in the editor too (Play Mode tests).</summary>
        public static bool RunInEditor;

        public ShuttleFleet Fleet { get; private set; }
        /// <summary>1-based: Bus 1, Bus 2, ...</summary>
        public int Number { get; private set; }
        public string Label => $"Bus {Number}";
        public Color Colour => ShuttleFleet.Colour(Number);
        public ShuttleBrain Brain { get; private set; }
        /// <summary>This bus's own deliveries, flats and kerb strikes; they count in the fleet's score too.</summary>
        public ShuttleScore BusScore { get; private set; }
        public TwinCar Car => _car;
        public TwinAutopilot Pilot => _pilot;

        // The fleet's, for one bus's point of view.
        public ShuttleSettings Settings => Fleet.Settings;
        public RequestSpawner Spawner => Fleet.Spawner;
        public ShuttleScore Score => Fleet.Score;
        public Dispatcher Dispatcher => Fleet.Dispatcher;
        public CampusRoutes Routes => _pilot != null ? _pilot.Routes : null;
        public IReadOnlyList<RideRequest> Waiting => Fleet.Waiting;
        public RideRequest RaiseRequest(string origin, string destination) => Fleet.RaiseRequest(origin, destination);
        public Vector3 KerbSlot(string stop, int i) => Fleet.KerbSlot(stop, i);

        private TwinCar _car;
        private TwinAutopilot _pilot;
        private bool _charging;
        private double _soc = double.NaN; // the charge before a reset (ResetTo fills the pack); NaN before the first step
        private string _legFrom, _legTo;
        private long _strikes;
        private readonly Queue<string> _log = new Queue<string>();   // the last few events, for a flat battery's warning

        private void Note(string what)
        {
            _log.Enqueue($"{Fleet.Now:0.0} s {what} (soc {_car.State.soc:0.000})");
            while (_log.Count > 12) _log.Dequeue();
        }

        public static bool ShouldRun() =>
            Application.isEditor ? RunInEditor : Array.IndexOf(Environment.GetCommandLineArgs(), "-goto") < 0;

        // --------------------------------------------------------------- setup

        /// <summary>Joins the fleet as bus `number`: the game battery, a brain, the lane.</summary>
        internal void Setup(ShuttleFleet fleet, int number, RouteTravelModel travel)
        {
            Fleet = fleet;
            Number = number;
            name = Label;
            _car = GetComponent<TwinCar>();
            _pilot = GetComponent<TwinAutopilot>();
            _pilot.targetSpeed = Settings.Cruise;
            _pilot.useLearned = false;
            _pilot.laneOffset = ShuttleFleet.LaneOffset;
            // Out of the crossing road: its half width, and half the car's footprint.
            _pilot.junctionSetback = 0.5 * _pilot.Routes.RoadWidth + _car.Footprint.halfLength;
            _pilot.routeColour = Color.Lerp(Colour, Color.white, 0.25f);
            _pilot.Obstacles = () => fleet.ObstaclesFor(this);
            _pilot.PoseFree = (x, y, yaw) => fleet.PoseFree(this, x, y, yaw);
            var p = _car.Params;
            double ah = BatteryPolicy.GamePackCapacityAh(Settings, BatteryPolicy.SeedWhPerMetre, p.packNominalV);
            _car.SetPackCapacity(ah);
            double seedSpm = BatteryPolicy.SeedWhPerMetre / (ah * p.packNominalV);
            BusScore = new ShuttleScore(Settings, fleet.Score);
            Brain = new ShuttleBrain(this, Settings, travel, BusScore, seedSpm) { PickCharger = from => fleet.PickCharger(this, from) };
            Brain.Boarded += r => fleet.OnBoarded(this, r);
            Brain.Alighted += r => fleet.OnAlighted(this, r);
            _car.showHud = false;
            _pilot.showPanel = false;
            _strikes = _car.KerbStrikes;
            _car.Resetted += KeepCharge;
            RoofSign();
        }

        private void OnDestroy()
        {
            if (_car != null) _car.Resetted -= KeepCharge;
        }

        // A tow, a turn at a stop or R moves the shuttle; it never charges it.
        // (The car's own first reset, before the game has stepped, is the full pack it starts with.)
        private void KeepCharge()
        {
            if (!double.IsNaN(_soc) && Math.Abs(_car.State.vx) < 0.05 && _car.State.soc != _soc) _car.SetCharge(_soc);
            if (Fleet != null) Note($"reset, charge kept at {_soc:0.000}");
            _strikes = _car.KerbStrikes;   // a reset clears the car's count
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

        // A square of the bus's colour on its roof, to tell the buses apart from above.
        private void RoofSign()
        {
            var body = _car.body != null ? _car.body : transform;
            var b = new Bounds();
            bool any = false;
            foreach (var r in body.GetComponentsInChildren<Renderer>())
            {
                if (!r.enabled) continue;   // the boxes, hidden under the shuttle model
                var lb = r.bounds;
                var p = body.InverseTransformPoint(lb.center + Vector3.up * lb.extents.y);
                if (!any) { b = new Bounds(p, Vector3.zero); any = true; } else b.Encapsulate(p);
            }
            var sign = GameObject.CreatePrimitive(PrimitiveType.Cube);
            sign.name = "Roof sign";
            Destroy(sign.GetComponent<Collider>());
            sign.transform.SetParent(body, false);
            sign.transform.localPosition = new Vector3(0f, (any ? b.max.y : 0.1f) + 0.002f, 0f);
            sign.transform.localScale = new Vector3(0.12f, 0.004f, 0.16f);
            var mr = sign.GetComponent<Renderer>();
            var shader = Shader.Find("Universal Render Pipeline/Unlit") ?? Shader.Find("Unlit/Color");
            if (shader != null) mr.material = new Material(shader) { color = Colour };
            else mr.material.color = Colour;
            mr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        }

        // ---------------------------------------------------------------- tick

        /// <summary>One physics step: charging, the brain, kerb strikes (the fleet calls it).</summary>
        internal void Step(double dt)
        {
            if (_charging && Math.Abs(_car.State.vx) < 0.05 && _car.State.soc < 1.0)
                _car.SetCharge(Math.Min(1.0, _car.State.soc + BatteryPolicy.ChargeRatePerSecond(Settings) * dt));
            _soc = _car.State.soc;
            var mode = Brain.Mode;
            Brain.Tick(dt);
            if (Brain.Mode == ShuttleMode.Towed && mode != ShuttleMode.Towed)
                Debug.LogWarning($"ShuttleGame: {Label} ran flat at {Fleet.Now:0.0} s ({Brain.Status}); pack {_car.Params.packCapacityAh:0.0000} Ah, " +
                                 $"estimate {Brain.Meter.SocPerMetre:0.000000}/m. Before: {string.Join("; ", _log)}");
            long strikes = _car.KerbStrikes;
            for (long k = _strikes; k < strikes; k++) BusScore.OnKerbStrike();
            if (strikes > _strikes)
            {
                var st = _car.State;
                Debug.Log($"ShuttleGame: {Label} struck a kerb at {Fleet.Now:0.0} s, ({st.x:0.00}, {st.y:0.00}) yaw {st.yaw:0.00} " +
                          $"v {st.vx:0.00}, {(_pilot.TurningRound ? "turning round, " : "")}{_pilot.Message}");
            }
            _strikes = strikes;
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
        public string LegTo => _legTo;
        public double LegS
        {
            get
            {
                if (!Driving || _legFrom == null || double.IsNaN(_pilot.Remaining)) return 0.0;
                return Math.Max(0.0, Fleet.Travel.Distance(_legFrom, _legTo) - _pilot.Remaining);
            }
        }
        public double Speed => _car.State.vx;
        public double Soc => _car.State.soc;
        public double Now => Fleet.Now;
        public bool Charging => _charging;

        public bool StartLeg(string stop)
        {
            string from = _pilot.CurrentStop();
            if (!_pilot.Go(stop)) return false;
            _legFrom = from;
            _legTo = stop;
            Note($"leg {from} to {stop}");
            return true;
        }

        public bool StopEarlyAt(string stop)
        {
            if (!_pilot.StopEarlyAt(stop)) return false;
            _legTo = stop;
            return true;
        }

        /// <summary>Put the bus at `stop`, facing its next stop, at a berth no other bus stands in.</summary>
        public void Tow(string stop)
        {
            var net = _pilot.Routes;
            string toward = Brain != null && Brain.Plan.Stops.Count > 0 && Brain.Plan.Stops[0].Stop != stop
                ? Brain.Plan.Stops[0].Stop
                : net.StopNames().First(s => net.Stops[s].node != net.Stops[stop].node);
            var (x, y, yaw) = Fleet.FreeBerth(this, stop, toward);
            _soc = _car.State.soc;
            Note($"tow to {stop}");
            _car.ResetTo(x, y, yaw);
            _strikes = _car.KerbStrikes;
        }

        public void SetCharging(bool on) => _charging = on;

        /// <summary>The shuttle's door: the left side, at the kerb.</summary>
        public Vector3 Door()
        {
            var t = _car.transform;
            return t.position - t.right * 0.2f;
        }
    }
}
