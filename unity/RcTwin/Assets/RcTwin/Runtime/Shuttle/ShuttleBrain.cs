// One shuttle's mind (Docs/specs/2026-10-21-campus-shuttle-design.md): it
// keeps a plan of stops, takes requests by cheapest insertion, drives each
// leg through the world (ShuttleGame: TwinAutopilot and TwinCar), stops on
// the way for students when it can, boards and drops off, and keeps its
// battery: charging at a charger, going there first when it must, never
// running flat on purpose. In a fleet it may retire, and an idle one makes
// room for another (Docs/specs/2026-10-25-shuttle-fleet-design.md). Free of
// UnityEngine; tested on a fake world.
using System;
using System.Collections.Generic;
using System.Linq;

namespace RcTwin
{
    /// <summary>What the shuttle drives with: one stop-to-stop leg at a time.</summary>
    public interface IShuttleWorld
    {
        /// <summary>The stop the shuttle is stopped at, or null.</summary>
        string AtStop { get; }
        bool Driving { get; }
        /// <summary>The current leg ended away from its stop (left the route, timed out).</summary>
        bool LegFailed { get; }
        string LegFrom { get; }
        /// <summary>Arc length along the current leg [m].</summary>
        double LegS { get; }
        /// <summary>[m/s]</summary>
        double Speed { get; }
        double Soc { get; }
        double Now { get; }
        bool StartLeg(string stop);
        bool StopEarlyAt(string stop);
        void Tow(string stop);
        void SetCharging(bool on);
    }

    public enum ShuttleMode { Idle, Driving, Dwelling, Charging, Towed }

    public sealed class ShuttleBrain
    {
        public const double FlatSoc = 0.005, TowSeconds = 3.0, RefuseSeconds = 2.0;

        public ShuttleMode Mode { get; private set; } = ShuttleMode.Idle;
        /// <summary>The stops ahead; while driving, the first is the leg's stop.</summary>
        public ShuttlePlan Plan { get; private set; } = new ShuttlePlan();
        public LowBattery Battery { get; private set; } = LowBattery.Fine;
        public List<RideRequest> Aboard { get; } = new List<RideRequest>();
        public ShuttleSettings Settings { get; }
        public SocMeter Meter { get; }
        /// <summary>Students waiting (unassigned) at a stop: set by the Dispatcher.</summary>
        public Func<string, IEnumerable<RideRequest>> WaitingAt = _ => Enumerable.Empty<RideRequest>();
        /// <summary>The charger to go to from a stop: by default the nearest; a fleet passes over the taken ones.</summary>
        public Func<string, string> PickCharger;
        /// <summary>The charger it is charging at or going to, or null.</summary>
        public string Charger => Mode == ShuttleMode.Charging || Mode == ShuttleMode.Towed ? _charger
            : Plan.Stops.FirstOrDefault(st => st.Charge)?.Stop;
        /// <summary>Taking no new requests; it leaves the fleet once everyone aboard is delivered.</summary>
        public bool Retiring { get; private set; }
        public bool Retired => Retiring && Aboard.Count == 0 && Plan.Stops.Count == 0 &&
                               Mode != ShuttleMode.Driving && Mode != ShuttleMode.Towed;

        public event Action<RideRequest> Boarded, Alighted;
        public event Action PlanChanged;

        private readonly IShuttleWorld _w;
        private readonly ITravelModel _t;
        private readonly ShuttleScore _score;
        private string _legFrom, _arrivedFrom;
        private double _legStartSoc, _dwellLeft, _refusedFor, _towLeft;
        private bool _chargeHere, _towedThisLeg;
        private string _charger;

        public ShuttleBrain(IShuttleWorld w, ShuttleSettings s, ITravelModel t, ShuttleScore score, double seedSocPerMetre)
        {
            _w = w;
            Settings = s;
            _t = t;
            _score = score;
            Meter = new SocMeter(seedSocPerMetre);
            PickCharger = from => BatteryPolicy.NearestCharger(Settings, _t, from);
        }

        // The charger to head for from `from` with `soc` left there: PickCharger's choice
        // if the charge reaches it (with half the reserve, as the low-battery plans
        // allow), else the nearest; queueing at a taken charger beats running flat.
        private string ChargerFrom(string from, double soc)
        {
            string c = PickCharger(from);
            string n = BatteryPolicy.NearestCharger(Settings, _t, from);
            if (c == n || from == null) return c;
            return _t.Distance(from, c) * Meter.SocPerMetre * Settings.EnergySafety <= soc - 0.5 * Settings.Reserve ? c : n;
        }

        /// <summary>The bottom bar's sentence.</summary>
        public string Status
        {
            get
            {
                var s = Settings;
                switch (Mode)
                {
                    case ShuttleMode.Towed: return $"Flat battery: towing to {_charger}";
                    case ShuttleMode.Charging: return $"Charging at {_charger}, {100 * _w.Soc:0} %";
                }
                if (Battery == LowBattery.DropOffFirst)
                    return $"Battery low: dropping off {Aboard.Count}, then charging";
                if (Battery == LowBattery.ChargeNow) return "Battery low: charging first";
                if (Plan.Stops.Count == 0)
                    return Retiring ? "Leaving the fleet" : _w.AtStop != null ? $"Waiting at {_w.AtStop}" : "Waiting";
                var next = Plan.Stops[0];
                if (next.Charge) return $"To {next.Stop}: charge";
                if (next.Park && next.Board.Count == 0 && next.Alight.Count == 0) return $"To {next.Stop}: making room";
                var parts = new List<string>();
                if (next.Board.Count > 0) parts.Add($"pick up {next.Board.Count}");
                if (next.Alight.Count > 0) parts.Add($"drop off {next.Alight.Count}");
                return Mode == ShuttleMode.Dwelling && _w.AtStop == next.Stop
                    ? $"At {next.Stop}" : $"To {next.Stop}: {string.Join(", ", parts)}";
            }
        }

        // ------------------------------------------------------------ requests

        /// <summary>The added cost of taking r [s], or null if it cannot.</summary>
        public double? QuoteInsertion(RideRequest r)
        {
            if (Battery != LowBattery.Fine || Mode == ShuttleMode.Towed || Retiring) return null;
            var c = Context(out var sub, out _);
            DropParking(sub);
            return sub.TryInsert(r, c, Settings, _t, out double added) == null ? (double?)null : added;
        }

        public void Accept(RideRequest r)
        {
            var c = Context(out var sub, out var committed);
            DropParking(sub);
            var cand = sub.TryInsert(r, c, Settings, _t, out _);
            if (cand == null) return;
            Plan = Merge(committed, cand);
            r.State = RequestState.Assigned;
            PlanChanged?.Invoke();
        }

        /// <summary>Take no new requests and give back the pickups not yet aboard
        /// (they wait again); deliver everyone aboard, then leave the fleet.</summary>
        public void Retire()
        {
            if (Retiring) return;
            Retiring = true;
            foreach (var r in Plan.Stops.SelectMany(st => st.Board).ToList())
            {
                foreach (var st in Plan.Stops) { st.Board.Remove(r); st.Alight.Remove(r); }
                r.State = RequestState.Waiting;
            }
            foreach (var st in Plan.Stops) st.Park = false;
            Prune();
            PlanChanged?.Invoke();
        }

        /// <summary>Back in service after Retire.</summary>
        public void Unretire() => Retiring = false;

        /// <summary>Idle with nothing to do, drive to `stop` and wait there: another bus needs the way.</summary>
        public bool MakeRoom(string stop)
        {
            if (Mode != ShuttleMode.Idle || Plan.Stops.Count > 0 || stop == _w.AtStop) return false;
            Plan.Stops.Add(new PlanStop { Stop = stop, Park = true });
            PlanChanged?.Invoke();
            return true;
        }

        /// <summary>Drop r from the plan (it gave up).</summary>
        public void Release(RideRequest r)
        {
            foreach (var st in Plan.Stops) { st.Board.Remove(r); st.Alight.Remove(r); }
            Prune();
        }

        // ---------------------------------------------------------------- tick

        public void Tick(double dt)
        {
            var s = Settings;
            if (Mode == ShuttleMode.Towed)
            {
                _towLeft -= dt;
                if (_towLeft <= 0.0)
                {
                    _w.Tow(_charger);
                    _arrivedFrom = null;
                    if (Plan.Stops.Count > 0 && Plan.Stops[0].Stop == _charger) Plan.Stops.RemoveAt(0);
                    BeginCharging();
                }
                return;
            }
            if (Mode != ShuttleMode.Charging && _w.Soc <= FlatSoc)
            {
                _score.OnFlat();
                var c0 = Context(out var sub0, out var committed0);
                var keep = new ShuttlePlan();
                _charger = BatteryPolicy.NearestCharger(Settings, _t, _w.AtStop ?? (Plan.Stops.Count > 0 ? Plan.Stops[0].Stop : _legFrom));
                keep.Stops.Add(new PlanStop { Stop = _charger, Charge = true });
                foreach (var st in Plan.Stops)
                    foreach (var r in st.Board) r.State = RequestState.Waiting;
                foreach (var st in Plan.Stops)
                {
                    var al = st.Alight.FindAll(r => Aboard.Contains(r));
                    if (al.Count > 0) keep.Stops.Add(new PlanStop { Stop = st.Stop, Alight = al });
                }
                Plan = keep;
                Battery = LowBattery.ChargeNow;
                Mode = ShuttleMode.Towed;
                _towLeft = TowSeconds;
                _w.SetCharging(false);
                PlanChanged?.Invoke();
                return;
            }
            if (Mode != ShuttleMode.Charging && Battery == LowBattery.Fine) CheckBattery();
            else if (Mode != ShuttleMode.Charging && Battery != LowBattery.Fine) RecheckLow();

            switch (Mode)
            {
                case ShuttleMode.Idle:
                    if (Plan.Stops.Count > 0) StartNext(dt);
                    else if (_w.Soc < s.IdleTopUpBelow && _w.AtStop != null && !Retiring)
                    {
                        string charger = ChargerFrom(_w.AtStop, _w.Soc);
                        if (_w.AtStop == charger) { _charger = charger; BeginCharging(); }
                        else
                        {
                            Plan.Stops.Add(new PlanStop { Stop = charger, Charge = true });
                            StartNext(dt);
                        }
                    }
                    break;
                case ShuttleMode.Driving:
                    if (_w.LegFailed)
                    {
                        _w.Tow(Plan.Stops[0].Stop);
                        _towedThisLeg = true;
                        Arrive();
                    }
                    else if (!_w.Driving && _w.AtStop == Plan.Stops[0].Stop) Arrive();
                    else CheckOnTheWay();
                    break;
                case ShuttleMode.Dwelling:
                    if (Plan.Stops.Count > 0 && Plan.Stops[0].Stop == _w.AtStop) ServeHere();
                    else BoardWaitingHere();
                    _dwellLeft -= dt;
                    if (_dwellLeft <= 0.0)
                    {
                        if (_chargeHere && _w.Soc < s.LeaveChargerAt) BeginCharging();
                        else if (Plan.Stops.Count > 0) StartNext(dt);
                        else Mode = ShuttleMode.Idle;
                    }
                    break;
                case ShuttleMode.Charging:
                    _w.SetCharging(true);
                    if (Plan.Stops.Count > 0 && Plan.Stops[0].Stop == _w.AtStop) ServeHere();
                    else BoardWaitingHere();
                    if (_w.Soc >= s.LeaveChargerAt)
                    {
                        _w.SetCharging(false);
                        Battery = LowBattery.Fine;
                        _chargeHere = false;
                        Mode = ShuttleMode.Idle;
                        PlanChanged?.Invoke();
                    }
                    break;
            }
        }

        // --------------------------------------------------------------- legs

        private void StartNext(double dt)
        {
            var next = Plan.Stops[0];
            if (next.Stop == _w.AtStop)
            {
                _legFrom = _w.AtStop;
                _towedThisLeg = true;      // no leg driven: nothing to measure
                Arrive();
                return;
            }
            string from = _w.AtStop;
            if (_w.StartLeg(next.Stop))
            {
                _legFrom = _w.LegFrom ?? from;   // the car may roll at a stop: the world knows where it left
                _legStartSoc = _w.Soc;
                _refusedFor = 0.0;
                _towedThisLeg = false;
                Mode = ShuttleMode.Driving;
                return;
            }
            _refusedFor += dt;
            if (_refusedFor >= RefuseSeconds)
            {
                _refusedFor = 0.0;
                _legFrom = from;
                _w.Tow(next.Stop);
                _towedThisLeg = true;
                Arrive();
            }
        }

        private void Arrive()
        {
            var stop = Plan.Stops[0];
            if (!_towedThisLeg && _legFrom != null)
                Meter.Observe(_t.Distance(_legFrom, stop.Stop), _legStartSoc - _w.Soc);
            _arrivedFrom = _legFrom == stop.Stop ? _arrivedFrom : _legFrom;
            _chargeHere = false;
            Mode = ShuttleMode.Dwelling;
            _dwellLeft = Settings.DwellBase;
            ServeHere();
        }

        // Alight and board at the stop the shuttle is at (Plan.Stops[0] there).
        private void ServeHere()
        {
            var s = Settings;
            var stop = Plan.Stops[0];
            Plan.Stops.RemoveAt(0);
            _chargeHere |= stop.Charge;
            if (stop.Charge) _charger = stop.Stop;
            int moved = 0;
            foreach (var r in stop.Alight)
            {
                if (!Aboard.Remove(r)) continue;
                r.State = RequestState.Delivered;
                r.DroppedAt = _w.Now;
                _score.OnDelivered(r, _t.DriveTime(r.Origin, r.Destination));
                Alighted?.Invoke(r);
                moved++;
            }
            foreach (var r in stop.Board.OrderBy(x => x.RaisedAt).ToList())
            {
                if (Aboard.Count < s.Capacity && CanBoardWithBattery(r)) { BoardNow(r); moved++; }
                else
                {
                    // No seat, or not the charge for it: back to waiting, re-planned.
                    foreach (var st in Plan.Stops) { st.Board.Remove(r); st.Alight.Remove(r); }
                    r.State = RequestState.Waiting;
                }
            }
            moved += BoardWaitingHere();
            _dwellLeft += s.DwellPerStudent * moved;
            Prune();
            PlanChanged?.Invoke();
        }

        // Unassigned students waiting where the shuttle stands: board those it can take now.
        private int BoardWaitingHere()
        {
            string here = _w.AtStop;
            if (here == null || Retiring) return 0;
            int n = 0;
            foreach (var r in WaitingAt(here).Where(x => x.State == RequestState.Waiting).OrderBy(x => x.RaisedAt).ToList())
            {
                if (Aboard.Count >= Settings.Capacity) break;
                if (Battery != LowBattery.Fine && !CanBoardWithBattery(r)) continue;
                var c = Context(out var sub, out var committed);
                var cand = sub.TryInsert(r, c, Settings, _t, out _);
                if (cand == null && Battery != LowBattery.Fine)
                {
                    // Low battery: the plan is already infeasible by design; add the drop-off before the charger.
                    cand = Plan.Copy();
                    int ci = cand.Stops.FindIndex(st => st.Charge);
                    cand.Stops.Insert(ci < 0 ? cand.Stops.Count : ci, new PlanStop { Stop = here, Board = { r } });
                    cand.Stops.Insert(ci < 0 ? cand.Stops.Count : ci + 1, new PlanStop { Stop = r.Destination, Alight = { r } });
                }
                if (cand == null || cand.Stops.Count == 0 || cand.Stops[0].Stop != here || !cand.Stops[0].Board.Contains(r))
                    continue;   // it would rather come back for them: leave them to the dispatcher
                cand.Stops[0].Board.Remove(r);
                if (cand.Stops[0].Board.Count == 0 && cand.Stops[0].Alight.Count == 0 && !cand.Stops[0].Charge)
                    cand.Stops.RemoveAt(0);
                Plan = Merge(committed, cand);
                BoardNow(r);
                n++;
            }
            if (n > 0) { _dwellLeft += Settings.DwellPerStudent * n; PlanChanged?.Invoke(); }
            return n;
        }

        private void BoardNow(RideRequest r)
        {
            r.State = RequestState.Riding;
            r.PickedUpAt = _w.Now;
            Aboard.Add(r);
            Boarded?.Invoke(r);
        }

        // With the battery low, r boards only if dropping everyone off, r included, then charging still fits.
        private bool CanBoardWithBattery(RideRequest r)
        {
            if (Battery == LowBattery.Fine) return true;
            var trial = new ShuttlePlan();
            foreach (var st in Plan.Stops)
            {
                var al = st.Alight.FindAll(x => Aboard.Contains(x));
                if (al.Count > 0) trial.Stops.Add(new PlanStop { Stop = st.Stop, Alight = al });
            }
            trial.Stops.Add(new PlanStop { Stop = r.Destination, Alight = { r } });
            trial.Stops.Add(new PlanStop { Stop = BatteryPolicy.NearestCharger(Settings, _t, r.Destination), Charge = true });
            var c = new PlanContext
            {
                From = _w.AtStop, ArrivedFrom = _arrivedFrom, Now = _w.Now, Onboard = Aboard.Count + 1,
                Soc = _w.Soc, SocPerMetre = Meter.SocPerMetre,
            };
            return trial.SocNeeded(c, Settings, _t) <= _w.Soc - 0.5 * Settings.Reserve;
        }

        private void BeginCharging()
        {
            Mode = ShuttleMode.Charging;
            _chargeHere = true;
            _w.SetCharging(true);
            PlanChanged?.Invoke();
        }

        // ------------------------------------------------------- the battery

        private void CheckBattery()
        {
            var c = Context(out var sub, out var committed);
            DropParking(sub);
            var d = BatteryPolicy.Decide(sub, c, Settings, _t, out var next, from => ChargerFrom(from, c.Soc - sub.SocNeeded(c, Settings, _t)));
            if (d == LowBattery.Fine) return;
            Battery = d;
            Plan = Merge(committed, next);
            PlanChanged?.Invoke();
        }

        // With the battery low, the plan must still reach its charger with what is left (the
        // reserve may go): the estimate per metre grows as the shuttle measures turns and
        // waits. When it no longer does, it charges first, at the nearest charger.
        private void RecheckLow()
        {
            var c = Context(out var sub, out var committed);
            if (sub.Stops.Count == 0 || sub.SocNeeded(c, Settings, _t) <= c.Soc) return;
            string charger = BatteryPolicy.NearestCharger(Settings, _t, c.From);
            if (Battery == LowBattery.ChargeNow && sub.Stops[0].Charge && sub.Stops[0].Stop == charger) return;
            var next = new ShuttlePlan();
            next.Stops.Add(new PlanStop { Stop = charger, Charge = true });
            foreach (var st in sub.Stops)
            {
                foreach (var r in st.Board) r.State = RequestState.Waiting;   // pickups go back to waiting
                var al = st.Alight.FindAll(r => Aboard.Contains(r));
                if (al.Count == 0) continue;
                if (st.Stop == charger) next.Stops[0].Alight.AddRange(al);
                else next.Stops.Add(new PlanStop { Stop = st.Stop, Alight = al });
            }
            Battery = LowBattery.ChargeNow;
            Plan = Merge(committed, next);
            PlanChanged?.Invoke();
        }

        // --------------------------------------------------- stopping on the way

        private void CheckOnTheWay()
        {
            if (Plan.Stops.Count == 0 || _legFrom == null) return;
            var next = Plan.Stops[0];
            foreach (var (x, sx) in _t.StopsOnRoute(_legFrom, next.Stop))
            {
                if (sx <= _w.LegS) continue;
                bool alight = Aboard.Any(r => r.Destination == x);
                int seats = Settings.Capacity - Aboard.Count + Aboard.Count(r => r.Destination == x);
                bool board = Battery == LowBattery.Fine && !Retiring && seats > 0 &&
                             (WaitingAt(x).Any(r => r.State == RequestState.Waiting) ||
                              Plan.Stops.Skip(1).Any(st => st.Stop == x && st.Board.Count > 0));
                if (!(alight || board) || !_w.StopEarlyAt(x)) continue;
                var here = new PlanStop { Stop = x };
                foreach (var st in Plan.Stops)
                    foreach (var r in st.Alight.Where(r => r.Destination == x && Aboard.Contains(r)).ToList())
                    {
                        st.Alight.Remove(r);
                        here.Alight.Add(r);
                    }
                // Pickups planned later at x board here, oldest first, as many as the seats free here.
                foreach (var r in Plan.Stops.Where(st => st.Stop == x).SelectMany(st => st.Board)
                             .OrderBy(r => r.RaisedAt).Take(seats).ToList())
                {
                    foreach (var st in Plan.Stops) st.Board.Remove(r);
                    here.Board.Add(r);
                }
                Plan.Stops.Insert(0, here);
                Prune();
                PlanChanged?.Invoke();
                return;
            }
        }

        // ----------------------------------------------------------- helpers

        // Where a plan starts from: while driving, the committed stop is fixed and
        // the rest of the plan (sub) is planned from it.
        private PlanContext Context(out ShuttlePlan sub, out PlanStop committed)
        {
            var s = Settings;
            double spm = Meter.SocPerMetre;
            if (Mode == ShuttleMode.Driving && Plan.Stops.Count > 0)
            {
                committed = Plan.Stops[0];
                sub = new ShuttlePlan { Stops = Plan.Stops.Skip(1).Select(x => x.Copy()).ToList() };
                double remaining = Math.Max(0.0, _t.Distance(_legFrom, committed.Stop) - _w.LegS);
                int onboard = Aboard.Count - committed.Alight.Count(r => Aboard.Contains(r)) + committed.Board.Count;
                return new PlanContext
                {
                    From = committed.Stop, ArrivedFrom = _legFrom,
                    Now = _w.Now + remaining / s.Cruise + s.DwellBase + s.DwellPerStudent * (committed.Board.Count + committed.Alight.Count),
                    Onboard = onboard, Soc = _w.Soc - remaining * spm * s.EnergySafety, SocPerMetre = spm,
                };
            }
            committed = null;
            sub = Plan.Copy();
            double extra = Mode == ShuttleMode.Charging
                ? Math.Max(0.0, s.LeaveChargerAt - _w.Soc) / BatteryPolicy.ChargeRatePerSecond(s) : 0.0;
            return new PlanContext
            {
                From = _w.AtStop ?? (Plan.Stops.Count > 0 ? Plan.Stops[0].Stop : _legFrom ?? s.Chargers[0]),
                ArrivedFrom = _arrivedFrom, Now = _w.Now + extra, Onboard = Aboard.Count,
                Soc = Mode == ShuttleMode.Charging ? Math.Max(_w.Soc, s.LeaveChargerAt) : _w.Soc, SocPerMetre = spm,
            };
        }

        private static ShuttlePlan Merge(PlanStop committed, ShuttlePlan sub)
        {
            if (committed == null) return sub;
            var p = new ShuttlePlan();
            p.Stops.Add(committed);
            int i = 0;
            if (sub.Stops.Count > 0 && sub.Stops[0].Stop == committed.Stop)
            {
                committed.Board.AddRange(sub.Stops[0].Board.Where(r => !committed.Board.Contains(r)));
                committed.Alight.AddRange(sub.Stops[0].Alight.Where(r => !committed.Alight.Contains(r)));
                committed.Charge |= sub.Stops[0].Charge;
                i = 1;
            }
            for (; i < sub.Stops.Count; i++) p.Stops.Add(sub.Stops[i]);
            return p;
        }

        // Drop planned stops with nothing to do (the committed one stays while driving).
        private void Prune()
        {
            for (int i = Plan.Stops.Count - 1; i >= 0; i--)
            {
                var st = Plan.Stops[i];
                if (i == 0 && Mode == ShuttleMode.Driving) continue;
                if (st.Board.Count == 0 && st.Alight.Count == 0 && !st.Charge && !st.Park) Plan.Stops.RemoveAt(i);
            }
        }

        // A stop only to make room is given up for real work.
        private static void DropParking(ShuttlePlan p)
        {
            foreach (var st in p.Stops) st.Park = false;
            p.Stops.RemoveAll(st => st.Board.Count == 0 && st.Alight.Count == 0 && !st.Charge);
        }
    }
}
