// Drives one rc_twin vehicle from the keyboard and shows it.
//
// The Core owns the dynamics (ADR 0002): Unity only reads input, steps the
// Core at a fixed 1 ms in FixedUpdate, and draws the result. No PhysX body is
// involved. With a RaycastGround on the same object, the Core feels the
// scene's ground (grade, banking, GroundSurface grip; ADR 0005); without one
// it drives on level ground at height 0.
//
// Frames: the Core is ISO 8855 (x forward, y LEFT, z up, yaw CCW from above);
// Unity is left-handed y-up. World ISO (x, y, z) maps to Unity (-y, z, x) and
// ISO yaw maps to a Unity Y rotation of -yaw.
//
// Keys: W/S or Up/Down throttle and brake, A/D or Left/Right steer, R reset.
// An autopilot (TwinAutopilot) drives it through Command; holding any drive
// key takes over from it.
using System;
using UnityEngine;
using UnityEngine.InputSystem;

namespace RcTwin
{
    public class TwinCar : MonoBehaviour
    {
        [Tooltip("SlipX fidelity tier. L2 is the full double-track model and the only one that feels grade and grip.")]
        public RcTwinNative.Tier tier = RcTwinNative.Tier.L2DoubleTrack;

        [Tooltip("Core step [s]. SlipX is specified at 1 kHz.")]
        public double coreDt = 0.001;

        [Tooltip("Wheel pivots in Core order: FL, FR, RL, RR.")]
        public Transform[] wheels = new Transform[4];

        public bool showHud = true;

        [Header("Start pose (ISO world; R resets here)")]
        public double homeX;
        public double homeY;
        [Tooltip("Heading [rad], counter-clockwise from ISO +x seen from above.")]
        public double homeYaw;

        [Tooltip("Feed the Core the ground from a RaycastGround on this object, if present.")]
        public bool useGroundQuery = true;

        [Header("Walls (ADR 0009)")]
        [Tooltip("Hand every TwinWalls in the scene to the Core.")]
        public bool useWalls = true;
        [Tooltip("Added to the footprint's half width [m]: half the drawn tyre width, since the tyres' " +
                 "outer halves stick out past the track. Matches the drawn car, not a measurement.")]
        public double footprintMargin = 0.0175;

        [Tooltip("Per-wheel rotational inertia [kg m^2] (ADR 0006). 0 keeps quasi-static wheels. " +
                 "The default, 1e-5, is SlipX ADR-0027's order of magnitude for a 1/10-scale wheel: " +
                 "PROVISIONAL until a spin-down test on the real car measures it.")]
        public double wheelInertia = 1e-5;

        [Tooltip("Fraction of ideal Ackermann steering (ADR 0007): 0 parallel, 1 ideal, negative anti. " +
                 "The default, 1, is the geometric ideal: PROVISIONAL until both front wheel angles " +
                 "are measured on the real car at a known steer.")]
        [Range(-1f, 1f)] public double ackermann = 1.0;

        [Header("Suspension (ADR 0008)")]
        [Tooltip("Body heave, pitch and roll on springs. The three values below are DEMO TARGETS, " +
                 "not a car: replace them with wheel rates measured on the bench.")]
        public bool suspension = true;
        [Tooltip("DEMO: heave ride frequency [Hz] the wheel rates are set from.")]
        public double rideFrequencyHz = 3.0;
        [Tooltip("DEMO: heave damping ratio the dampers are set from.")]
        public double dampingRatio = 0.3;
        [Tooltip("DEMO: compression from static to the bump stop [m].")]
        public double suspensionTravel = 0.015;
        [Tooltip("The body mesh, drawn heaving, pitching and rolling over the wheels.")]
        public Transform body;

        private TwinVehicle _car;
        private RcTwinNative.Params _p;
        private RcTwinNative.State _prev, _curr;
        private readonly float[] _spinDeg = new float[4];
        private float _steerIn, _throttleIn;
        private RaycastGround _ground;
        private Vector3 _prevUp = Vector3.up, _currUp = Vector3.up;
        private bool _offGround;
        private Vector3 _bodyBase;
        private Vector3 _prevBodyN, _currBodyN;  // ground mean, body pitch, roll rel. to road

        public RcTwinNative.State State => _curr;

        /// <summary>Steer (+ left) and throttle (- brake) in [-1, 1] from an
        /// autopilot, applied each physics step; null for the keyboard.</summary>
        public (double steer, double throttle)? Command { get; set; }

        /// <summary>A drive key is held: the keyboard has the car, whatever Command says.</summary>
        public bool ManualOverride { get; private set; }

        /// <summary>Raised after a reset (R or ResetTo).</summary>
        public event Action Resetted;

        /// <summary>Wall contacts of the last physics frame (ADR 0009).</summary>
        public RcTwinNative.WallReport LastWallReport { get; private set; }

        /// <summary>Wall contacts since the last reset.</summary>
        public long WallContacts { get; private set; }

        /// <summary>Kerb strikes since the last reset: a scrape counts once, and a
        /// new strike needs StrikeGap without contact first (a pinned car bounces
        /// off the kerb and back).</summary>
        public long KerbStrikes { get; private set; }
        public const double StrikeGap = 0.5;   // [s]
        private double _sinceContact = double.PositiveInfinity;

        public uint WallSegments => _car != null ? _car.WallCount : 0u;

        /// <summary>The footprint the walls stop (ADR 0009), tyre margin included.</summary>
        public RcTwinNative.Contact Footprint => _car.Contact;

        /// <summary>Slip ratio of wheel i (0 FL .. 3 RR) along the body axis: 0 rolling, > 0 spinning, -1 locked.</summary>
        public double Slip(int i)
        {
            double v = _curr.vx;
            return (Omega(_curr, i) * _p.wheelRadius - v) / Math.Max(Math.Abs(v), _p.vEps);
        }
        public RcTwinNative.Params Params => _p;
        public bool OffGround => _offGround;

        public static Vector3 ToUnity(double x, double y, double z) =>
            new Vector3(-(float)y, (float)z, (float)x);

        public static float YawToUnityDeg(double yaw) => -(float)(yaw * Mathf.Rad2Deg);

        private void Awake()
        {
            _p = RcTwinNative.DefaultParams();
            _p.wheelInertia = wheelInertia;
            _p.ackermann = ackermann;
            if (suspension)
            {
                // Per wheel: four equal springs carry the car at the target
                // heave frequency; dampers at the target fraction of critical.
                double k = DemoWheelRate(_p.mass, rideFrequencyHz);
                double c = DemoDamping(_p.mass, k, dampingRatio);
                _p.wheelRateFront = _p.wheelRateRear = k;
                _p.dampingFront = _p.dampingRear = c;
                _p.suspensionTravel = suspensionTravel;
            }
            if (body != null) _bodyBase = body.localPosition;
            _car = new TwinVehicle(tier, _p);
            _ground = useGroundQuery ? GetComponent<RaycastGround>() : null;
            if (_ground != null)
            {
                if (_ground.climbableStep <= 0f) _ground.climbableStep = 0.5f * (float)_p.wheelRadius;
                _car.SetGround(_ground);
            }
            var contact = _car.Contact;
            contact.halfWidth += footprintMargin;
            _car.Contact = contact;
            if (useWalls)
            {
                foreach (var set in FindObjectsByType<TwinWalls>(FindObjectsSortMode.InstanceID))
                    foreach (var w in set.walls)
                        if (w.points != null && w.points.Length >= 2)
                            _car.AddWall(TwinWalls.Flatten(w), w.closed);
            }
            _prev = _curr = _car.State;
            Debug.Log($"rc_twin ABI {RcTwinNative.rc_twin_abi_version()}, SlipX {RcTwinNative.CoreVersion}, " +
                      $"tier {tier}, ground {(_ground != null ? "raycast" : "level")}, " +
                      $"{_car.WallCount} wall segments. " +
                      "Parameters are PROVISIONAL (SlipX defaults), not a measured car.");
        }

        private void Start()
        {
            if (homeX != 0.0 || homeY != 0.0 || homeYaw != 0.0) ResetTo(homeX, homeY, homeYaw);
        }

        private void OnDestroy()
        {
            _car?.Dispose();
            _car = null;
        }

        /// <summary>Per-wheel rate [N/m] for a heave frequency f on four equal springs.</summary>
        public static double DemoWheelRate(double mass, double f) =>
            mass * (2.0 * Math.PI * f) * (2.0 * Math.PI * f) / 4.0;

        /// <summary>Per-wheel damping [N s/m] for heave damping ratio zeta.</summary>
        public static double DemoDamping(double mass, double k, double zeta) =>
            2.0 * zeta * Math.Sqrt(4.0 * k * mass) / 4.0;

        /// <summary>Body tilt in the car's frame: pitch nose up, roll right side down (ISO).</summary>
        public static Quaternion BodyTilt(double pitchNoseUp, double rollRightDown) =>
            Quaternion.Euler(-(float)(pitchNoseUp * Mathf.Rad2Deg), 0f, -(float)(rollRightDown * Mathf.Rad2Deg));

        /// <summary>Place the car at ISO (x, y), heading yaw, at rest or rolling at speed vx.</summary>
        /// <summary>The pack's capacity [Ah]: the campus shuttle game's battery
        /// is a small one, so a charge lasts minutes (ShuttleGame). Nothing else changes.</summary>
        public void SetPackCapacity(double ah)
        {
            var p = _car.Params;
            p.packCapacityAh = ah;
            _car.Params = p;
            _p = _car.Params;
        }

        /// <summary>Set the state of charge, 0 to 1, keeping the rest of the state:
        /// charging at a charger. Only while parked.</summary>
        public void SetCharge(double soc)
        {
            var s = _car.State;
            if (Math.Abs(s.vx) >= 0.05) throw new InvalidOperationException("charge only while parked");
            s.soc = Math.Max(0.0, Math.Min(1.0, soc));
            _car.Reset(s);
            _prev = _curr = _car.State;
        }

        public void ResetTo(double x, double y, double yaw, double vx = 0.0)
        {
            double w = vx / _p.wheelRadius;  // wheels rolling with the car
            _ground?.Forget();               // a teleport: cast from above again
            WallContacts = 0;
            KerbStrikes = 0;
            _sinceContact = double.PositiveInfinity;
            var s = new RcTwinNative.State
            {
                x = x, y = y, yaw = yaw, vx = vx, soc = 1.0, packV = 11.1,
                omegaFL = w, omegaFR = w, omegaRL = w, omegaRR = w,
            };
            if (_p.wheelRateFront > 0.0 && _ground != null)
            {
                // Lay the sprung body on the ground where it lands, so a reset
                // onto a plateau or a ramp does not fire the springs.
                Plane(x, y, yaw, out double h, out double pitch, out double roll);
                s.z = h;
                s.pitch = pitch;
                s.roll = roll;
            }
            _car.Reset(s);
            _offGround = false;
            _prev = _curr = _car.State;
            _prevUp = _currUp = Vector3.up;
            Resetted?.Invoke();
        }

        private void Update()
        {
            ReadInput();
            DrawPose();
        }

        private void ReadInput()
        {
            var kb = Keyboard.current;
            if (kb == null) { _steerIn = _throttleIn = 0f; return; }

            float left = (kb.aKey.isPressed || kb.leftArrowKey.isPressed) ? 1f : 0f;
            float right = (kb.dKey.isPressed || kb.rightArrowKey.isPressed) ? 1f : 0f;
            float up = (kb.wKey.isPressed || kb.upArrowKey.isPressed) ? 1f : 0f;
            float down = (kb.sKey.isPressed || kb.downArrowKey.isPressed) ? 1f : 0f;
            _steerIn = left - right;   // + is left, as in the Core
            _throttleIn = up - down;
            ManualOverride = left + right + up + down > 0f;

            if (kb.rKey.wasPressedThisFrame) ResetTo(homeX, homeY, homeYaw);
        }

        private void FixedUpdate()
        {
            if (_offGround) return;

            uint n = (uint)Mathf.Max(1, Mathf.RoundToInt((float)(Time.fixedDeltaTime / coreDt)));
            double steerIn = _steerIn, throttleIn = _throttleIn;
            if (Command.HasValue && !ManualOverride)
            {
                var (cs, ct) = Command.Value;
                steerIn = Math.Max(-1.0, Math.Min(1.0, cs));
                throttleIn = Math.Max(-1.0, Math.Min(1.0, ct));
            }
            double steer = steerIn * _p.steerMax;
            double accel = throttleIn >= 0.0 ? throttleIn * _p.accelMax : throttleIn * _p.decelMax;
            try
            {
                _car.Step(steer, accel, coreDt, n);
            }
            catch (GroundQueryException e)
            {
                // The state is unchanged; hold here rather than drive on a guess.
                _offGround = true;
                Debug.LogWarning($"TwinCar stopped: {e.Message}. Press R to reset.");
                return;
            }

            LastWallReport = _car.LastWallReport;
            WallContacts += LastWallReport.contacts;
            if (LastWallReport.contacts > 0)
            {
                if (_sinceContact >= StrikeGap) KerbStrikes++;
                _sinceContact = 0.0;
            }
            else _sinceContact += n * coreDt;
            _prev = _curr;
            _curr = _car.State;
            _prevUp = _currUp;
            _currUp = _ground != null ? _ground.MeanUp : Vector3.up;
            _prevBodyN = _currBodyN;
            _currBodyN = BodyRelative(_curr, _currUp);

            // Wheel visuals. L0/L1 have no wheel states, so roll them from vx.
            double dt = n * coreDt;
            for (int i = 0; i < 4; i++)
            {
                double omega = tier >= RcTwinNative.Tier.L2DoubleTrack
                    ? Omega(_curr, i)
                    : _curr.vx / _p.wheelRadius;
                _spinDeg[i] = Mathf.Repeat(_spinDeg[i] + (float)(omega * dt * Mathf.Rad2Deg), 360f);
            }
        }

        private static double Omega(in RcTwinNative.State s, int i) =>
            i == 0 ? s.omegaFL : i == 1 ? s.omegaFR : i == 2 ? s.omegaRL : s.omegaRR;

        // The car's heading laid onto the road plane.
        /// <summary>
        /// A front wheel's road wheel angle for reference angle delta, the Core's
        /// formula (ADR 0007), for drawing. Positive is left.
        /// </summary>
        public static double FrontWheelAngle(in RcTwinNative.Params p, double delta, bool left)
        {
            if (p.ackermann == 0.0) return delta;
            double wb = p.lf + p.lr;
            double ht = 0.5 * p.trackFront;
            double tn = Math.Tan(delta);
            double ideal = Math.Atan(wb * tn / (left ? wb - ht * tn : wb + ht * tn));
            return delta + p.ackermann * (ideal - delta);
        }

        // The ground plane under the four wheels at ISO (x, y, yaw): mean height,
        // and the pitch (nose up) and roll (right down) a body lying on it has.
        private void Plane(double x, double y, double yaw, out double h, out double pitch, out double roll)
        {
            double[] bx = { _p.lf, _p.lf, -_p.lr, -_p.lr };
            double[] by = { _p.trackFront / 2, -_p.trackFront / 2, _p.trackRear / 2, -_p.trackRear / 2 };
            double cy = Math.Cos(yaw), sy = Math.Sin(yaw);
            double hs = 0, nx = 0, ny = 0, nz = 0;
            for (int i = 0; i < 4; i++)
            {
                var g = RcTwinNative.GroundSample.Level();
                if (!_ground.Sample(i, x + cy * bx[i] - sy * by[i], y + sy * bx[i] + cy * by[i], ref g)) continue;
                double len = Math.Sqrt(g.nx * g.nx + g.ny * g.ny + g.nz * g.nz);
                hs += g.height; nx += g.nx / len; ny += g.ny / len; nz += g.nz / len;
            }
            double nl = Math.Sqrt(nx * nx + ny * ny + nz * nz);
            nx /= nl; ny /= nl;
            h = hs / 4.0;
            pitch = -Math.Asin(nx * cy + ny * sy);   // forward . n
            roll = -Math.Asin(-nx * sy + ny * cy);   // left . n
        }

        // Body pose relative to the road under it (ADR 0008 states are absolute):
        // x = ground mean height, y = pitch, z = roll, both relative to the road.
        private Vector3 BodyRelative(in RcTwinNative.State s, Vector3 upUnity)
        {
            double h = 0.0;
            if (_ground != null)
                h = ((_ground.Last[0].height + _ground.Last[1].height) +
                     (_ground.Last[2].height + _ground.Last[3].height)) / 4.0;
            // Road attitude from its normal, in the car's frame.
            Vector3 fwd = ToUnity(Math.Cos(s.yaw), Math.Sin(s.yaw), 0.0);
            Vector3 left = ToUnity(-Math.Sin(s.yaw), Math.Cos(s.yaw), 0.0);
            double roadPitch = -Math.Asin(Vector3.Dot(fwd, upUnity));
            double roadRoll = -Math.Asin(Vector3.Dot(left, upUnity));
            return new Vector3((float)h, (float)(s.pitch - roadPitch), (float)(s.roll - roadRoll));
        }

        private static Quaternion OnPlane(double yaw, Vector3 up)
        {
            Vector3 fwd = Vector3.ProjectOnPlane(ToUnity(Math.Cos(yaw), Math.Sin(yaw), 0.0), up);
            return fwd.sqrMagnitude > 1e-12f ? Quaternion.LookRotation(fwd, up)
                                              : Quaternion.Euler(0f, YawToUnityDeg(yaw), 0f);
        }

        // Interpolate between the last two Core states so motion is smooth at
        // render rate while the physics stays at its fixed step. The body sits
        // h_cog above the mean contact point along the road normal.
        private void DrawPose()
        {
            float a = Time.fixedDeltaTime > 0f
                ? Mathf.Clamp01((Time.time - Time.fixedTime) / Time.fixedDeltaTime)
                : 1f;
            bool sprung = _p.wheelRateFront > 0.0;
            // The car's frame follows the ground; with suspension the body
            // moves on it (heave, pitch, roll), and z is the body's, so the
            // ground height comes from the ground query instead.
            double z0 = sprung ? _prevBodyN.x : _prev.z;
            double z1 = sprung ? _currBodyN.x : _curr.z;
            Vector3 p0 = ToUnity(_prev.x, _prev.y, z0) + _prevUp * (float)_p.hCog;
            Vector3 p1 = ToUnity(_curr.x, _curr.y, z1) + _currUp * (float)_p.hCog;
            transform.SetPositionAndRotation(
                Vector3.Lerp(p0, p1, a),
                Quaternion.Slerp(OnPlane(_prev.yaw, _prevUp), OnPlane(_curr.yaw, _currUp), a));
            if (body != null && sprung)
            {
                float heave = Mathf.Lerp((float)(_prev.z - z0), (float)(_curr.z - z1), a);
                body.localPosition = _bodyBase + Vector3.up * heave;
                body.localRotation = Quaternion.Slerp(BodyTilt(_prevBodyN.y, _prevBodyN.z),
                                                      BodyTilt(_currBodyN.y, _currBodyN.z), a);
            }

            float leftDeg = -(float)(FrontWheelAngle(_p, _curr.steer, true) * Mathf.Rad2Deg);
            float rightDeg = -(float)(FrontWheelAngle(_p, _curr.steer, false) * Mathf.Rad2Deg);
            for (int i = 0; i < 4 && i < wheels.Length; i++)
            {
                if (wheels[i] == null) continue;
                float yaw = i == 0 ? leftDeg : i == 1 ? rightDeg : 0f;
                wheels[i].localRotation = Quaternion.Euler(0f, yaw, 0f) * Quaternion.Euler(_spinDeg[i], 0f, 0f);
            }
        }

        private void OnGUI()
        {
            if (!showHud) return;
            var s = _curr;
            double mu = 1.0;
            if (_ground != null)
                mu = ((_ground.Last[0].muScale + _ground.Last[1].muScale) +
                      (_ground.Last[2].muScale + _ground.Last[3].muScale)) / 4.0;
            string wheels = _p.wheelInertia > 0.0
                ? $"rear slip  L {Slip(2):+0.00;-0.00}  R {Slip(3):+0.00;-0.00}\n"
                : "wheels quasi-static (inertia 0)\n";
            string susp = _p.wheelRateFront > 0.0
                ? $"body  pitch {_currBodyN.y * Mathf.Rad2Deg:+0.0;-0.0} deg  roll {_currBodyN.z * Mathf.Rad2Deg:+0.0;-0.0} deg (DEMO springs)\n"
                : "";
            GUI.Box(new Rect(10, 10, 330, 232), GUIContent.none);
            GUI.Label(new Rect(20, 15, 320, 222),
                $"SlipX {RcTwinNative.CoreVersion}  {tier}\n" +
                $"speed  {s.Speed * 3.6:F1} km/h  ({s.vx:F2} m/s)\n" +
                $"yaw rate  {s.yawRate:F2} rad/s\n" +
                $"steer  {s.steer * Mathf.Rad2Deg:F1} deg (achieved)\n" +
                $"SoC  {s.soc * 100.0:F2} %   pack {s.packV:F2} V\n" +
                $"ground  z {s.z:F2} m   grip x{mu:F2}\n" +
                wheels + susp +
                (_car.WallCount > 0 ? $"walls  {WallContacts} contacts (provisional restitution, friction)\n" : "") +
                (_offGround ? "OFF THE GROUND: press R\n" : "") +
                "PROVISIONAL parameters, not validated\n" +
                "W/S throttle/brake  A/D steer  R reset");
        }
    }
}
