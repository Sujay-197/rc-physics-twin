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

        [Tooltip("Feed the Core the ground from a RaycastGround on this object, if present.")]
        public bool useGroundQuery = true;

        [Tooltip("Per-wheel rotational inertia [kg m^2] (ADR 0006). 0 keeps quasi-static wheels. " +
                 "The default, 1e-5, is SlipX ADR-0027's order of magnitude for a 1/10-scale wheel: " +
                 "PROVISIONAL until a spin-down test on the real car measures it.")]
        public double wheelInertia = 1e-5;

        [Tooltip("Fraction of ideal Ackermann steering (ADR 0007): 0 parallel, 1 ideal, negative anti. " +
                 "The default, 1, is the geometric ideal: PROVISIONAL until both front wheel angles " +
                 "are measured on the real car at a known steer.")]
        [Range(-1f, 1f)] public double ackermann = 1.0;

        private TwinVehicle _car;
        private RcTwinNative.Params _p;
        private RcTwinNative.State _prev, _curr;
        private readonly float[] _spinDeg = new float[4];
        private float _steerIn, _throttleIn;
        private RaycastGround _ground;
        private Vector3 _prevUp = Vector3.up, _currUp = Vector3.up;
        private bool _offGround;

        public RcTwinNative.State State => _curr;

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
            _car = new TwinVehicle(tier, _p);
            _ground = useGroundQuery ? GetComponent<RaycastGround>() : null;
            if (_ground != null) _car.SetGround(_ground);
            _prev = _curr = _car.State;
            Debug.Log($"rc_twin ABI {RcTwinNative.rc_twin_abi_version()}, SlipX {RcTwinNative.CoreVersion}, " +
                      $"tier {tier}, ground {(_ground != null ? "raycast" : "level")}. " +
                      "Parameters are PROVISIONAL (SlipX defaults), not a measured car.");
        }

        private void OnDestroy()
        {
            _car?.Dispose();
            _car = null;
        }

        /// <summary>Place the car at ISO (x, y), heading yaw, at rest or rolling at speed vx.</summary>
        public void ResetTo(double x, double y, double yaw, double vx = 0.0)
        {
            double w = vx / _p.wheelRadius;  // wheels rolling with the car
            var s = new RcTwinNative.State
            {
                x = x, y = y, yaw = yaw, vx = vx, soc = 1.0, packV = 11.1,
                omegaFL = w, omegaFR = w, omegaRL = w, omegaRR = w,
            };
            _car.Reset(s);
            _offGround = false;
            _prev = _curr = _car.State;
            _prevUp = _currUp = Vector3.up;
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

            if (kb.rKey.wasPressedThisFrame) ResetTo(0.0, 0.0, 0.0);
        }

        private void FixedUpdate()
        {
            if (_offGround) return;

            uint n = (uint)Mathf.Max(1, Mathf.RoundToInt((float)(Time.fixedDeltaTime / coreDt)));
            double steer = _steerIn * _p.steerMax;
            double accel = _throttleIn >= 0f ? _throttleIn * _p.accelMax : _throttleIn * _p.decelMax;
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

            _prev = _curr;
            _curr = _car.State;
            _prevUp = _currUp;
            _currUp = _ground != null ? _ground.MeanUp : Vector3.up;

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
            Vector3 p0 = ToUnity(_prev.x, _prev.y, _prev.z) + _prevUp * (float)_p.hCog;
            Vector3 p1 = ToUnity(_curr.x, _curr.y, _curr.z) + _currUp * (float)_p.hCog;
            transform.SetPositionAndRotation(
                Vector3.Lerp(p0, p1, a),
                Quaternion.Slerp(OnPlane(_prev.yaw, _prevUp), OnPlane(_curr.yaw, _currUp), a));

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
            GUI.Box(new Rect(10, 10, 280, 200), GUIContent.none);
            GUI.Label(new Rect(20, 15, 270, 190),
                $"SlipX {RcTwinNative.CoreVersion}  {tier}\n" +
                $"speed  {s.Speed * 3.6:F1} km/h  ({s.vx:F2} m/s)\n" +
                $"yaw rate  {s.yawRate:F2} rad/s\n" +
                $"steer  {s.steer * Mathf.Rad2Deg:F1} deg (achieved)\n" +
                $"SoC  {s.soc * 100.0:F2} %   pack {s.packV:F2} V\n" +
                $"ground  z {s.z:F2} m   grip x{mu:F2}\n" +
                wheels +
                (_offGround ? "OFF THE GROUND: press R\n" : "") +
                "PROVISIONAL parameters, not validated\n" +
                "W/S throttle/brake  A/D steer  R reset");
        }
    }
}
