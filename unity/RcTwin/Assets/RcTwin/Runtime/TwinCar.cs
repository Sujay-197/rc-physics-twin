// Drives one rc_twin vehicle from the keyboard and shows it.
//
// The Core owns the dynamics (ADR 0002): Unity only reads input, steps the
// Core at a fixed 1 ms in FixedUpdate, and draws the result. No PhysX body is
// involved. Flat ground only until the Phase 2 ground query exists.
//
// Frames: the Core is ISO 8855 (x forward, y LEFT, z up, yaw CCW from above);
// Unity is left-handed y-up. World ISO (x, y, z) maps to Unity (-y, z, x) and
// ISO yaw maps to a Unity Y rotation of -yaw.
//
// Keys: W/S or Up/Down throttle and brake, A/D or Left/Right steer, R reset.
using UnityEngine;
using UnityEngine.InputSystem;

namespace RcTwin
{
    public class TwinCar : MonoBehaviour
    {
        [Tooltip("SlipX fidelity tier. L2 is the full double-track model.")]
        public RcTwinNative.Tier tier = RcTwinNative.Tier.L2DoubleTrack;

        [Tooltip("Core step [s]. SlipX is specified at 1 kHz.")]
        public double coreDt = 0.001;

        [Tooltip("Wheel pivots in Core order: FL, FR, RL, RR.")]
        public Transform[] wheels = new Transform[4];

        public bool showHud = true;

        private TwinVehicle _car;
        private RcTwinNative.Params _p;
        private RcTwinNative.State _prev, _curr;
        private readonly float[] _spinDeg = new float[4];
        private float _steerIn, _throttleIn;

        public RcTwinNative.State State => _curr;
        public RcTwinNative.Params Params => _p;

        public static Vector3 ToUnity(double x, double y, double z) =>
            new Vector3(-(float)y, (float)z, (float)x);

        public static float YawToUnityDeg(double yaw) => -(float)(yaw * Mathf.Rad2Deg);

        private void Awake()
        {
            _p = RcTwinNative.DefaultParams();
            _car = new TwinVehicle(tier, _p);
            _prev = _curr = _car.State;
            Debug.Log($"rc_twin ABI {RcTwinNative.rc_twin_abi_version()}, SlipX {RcTwinNative.CoreVersion}, " +
                      $"tier {tier}. Parameters are PROVISIONAL (SlipX defaults), not a measured car.");
        }

        private void OnDestroy()
        {
            _car?.Dispose();
            _car = null;
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

            if (kb.rKey.wasPressedThisFrame)
            {
                _car.ResetToRest();
                _prev = _curr = _car.State;
            }
        }

        private void FixedUpdate()
        {
            uint n = (uint)Mathf.Max(1, Mathf.RoundToInt((float)(Time.fixedDeltaTime / coreDt)));
            double steer = _steerIn * _p.steerMax;
            double accel = _throttleIn >= 0f ? _throttleIn * _p.accelMax : _throttleIn * _p.decelMax;
            _car.Step(steer, accel, coreDt, n);

            _prev = _curr;
            _curr = _car.State;

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

        // Interpolate between the last two Core states so motion is smooth at
        // render rate while the physics stays at its fixed step.
        private void DrawPose()
        {
            float a = Time.fixedDeltaTime > 0f
                ? Mathf.Clamp01((Time.time - Time.fixedTime) / Time.fixedDeltaTime)
                : 1f;
            Vector3 p0 = ToUnity(_prev.x, _prev.y, _prev.z + _p.hCog);
            Vector3 p1 = ToUnity(_curr.x, _curr.y, _curr.z + _p.hCog);
            transform.SetPositionAndRotation(
                Vector3.Lerp(p0, p1, a),
                Quaternion.Euler(0f, Mathf.LerpAngle(YawToUnityDeg(_prev.yaw), YawToUnityDeg(_curr.yaw), a), 0f));

            float steerDeg = -(float)(_curr.steer * Mathf.Rad2Deg);
            for (int i = 0; i < 4 && i < wheels.Length; i++)
            {
                if (wheels[i] == null) continue;
                float yaw = i < 2 ? steerDeg : 0f;  // parallel steer, as SlipX today
                wheels[i].localRotation = Quaternion.Euler(0f, yaw, 0f) * Quaternion.Euler(_spinDeg[i], 0f, 0f);
            }
        }

        private void OnGUI()
        {
            if (!showHud) return;
            var s = _curr;
            GUI.Box(new Rect(10, 10, 270, 150), GUIContent.none);
            GUI.Label(new Rect(20, 15, 260, 140),
                $"SlipX {RcTwinNative.CoreVersion}  {tier}\n" +
                $"speed  {s.Speed * 3.6:F1} km/h  ({s.vx:F2} m/s)\n" +
                $"yaw rate  {s.yawRate:F2} rad/s\n" +
                $"steer  {s.steer * Mathf.Rad2Deg:F1} deg (achieved)\n" +
                $"SoC  {s.soc * 100.0:F2} %   pack {s.packV:F2} V\n" +
                "PROVISIONAL parameters, not validated\n" +
                "W/S throttle/brake  A/D steer  R reset");
        }
    }
}
