// The campus player's camera in the shuttle game (Docs/specs/2026-10-25-shuttle-fleet-design.md,
// "Camera modes"): it follows a bus, frames its leg, the fleet or the whole
// campus, orbits freely, or watches a stop, as ShuttleFleet.Cameras says.
// It takes over from the scene's ChaseCamera.
//
// Keys: C next mode (Shift+C back), 1-6 follow that bus, [ and ] the stop cam's
// stop. Mouse: scroll zooms; in Orbit, drag with the right button to turn.
using System;
using System.Linq;
using UnityEngine;
using UnityEngine.InputSystem;

namespace RcTwin
{
    [RequireComponent(typeof(Camera))]
    public sealed class FleetCamera : MonoBehaviour
    {
        public ShuttleFleet Fleet;
        [Tooltip("How fast the camera settles on its mode's view [1/s].")]
        public float stiffness = 5f;

        /// <summary>The stop the stop cam watches.</summary>
        public string Stop { get; private set; }

        private Camera _cam;
        private Bounds _campus;
        private float _zoom = 1f, _orbitYaw, _orbitPitch = 45f, _orbitDist = -1f;
        private CameraMode _last;

        private void Start()
        {
            _cam = GetComponent<Camera>();
            var chase = GetComponent<ChaseCamera>();
            if (chase != null) chase.enabled = false;
            _cam.farClipPlane = Mathf.Max(_cam.farClipPlane, 400f);
            var asphalt = GameObject.Find("Campus asphalt");
            _campus = asphalt != null && asphalt.TryGetComponent<Renderer>(out var r) ? r.bounds : new Bounds(Vector3.zero, Vector3.one * 40f);
        }

        private void Update()
        {
            if (Fleet == null || Fleet.Cameras == null) return;
            var modes = Fleet.Cameras;
            var kb = Keyboard.current;
            if (kb != null)
            {
                bool shift = kb.leftShiftKey.isPressed || kb.rightShiftKey.isPressed;
                if (kb.cKey.wasPressedThisFrame) modes.Step(shift ? -1 : 1);
                var digits = new[] { kb.digit1Key, kb.digit2Key, kb.digit3Key, kb.digit4Key, kb.digit5Key, kb.digit6Key };
                for (int k = 0; k < digits.Length; k++) if (digits[k].wasPressedThisFrame) modes.Follow(k + 1);
                if (modes.Current.Kind == CameraKind.Stop && (kb.leftBracketKey.wasPressedThisFrame || kb.rightBracketKey.wasPressedThisFrame))
                    modes.StepStop(Fleet.Routes.StopNames(), Stop, kb.rightBracketKey.wasPressedThisFrame ? 1 : -1);
            }
            var m = Mouse.current;
            if (m != null)
            {
                float scroll = m.scroll.ReadValue().y;
                if (Mathf.Abs(scroll) > 0.01f) _zoom = Mathf.Clamp(_zoom * Mathf.Pow(0.9f, Mathf.Sign(scroll)), 0.35f, 3f);
                if (modes.Current.Kind == CameraKind.Orbit && m.rightButton.isPressed)
                {
                    var d = m.delta.ReadValue();
                    _orbitYaw += d.x * 0.25f;
                    _orbitPitch = Mathf.Clamp(_orbitPitch - d.y * 0.25f, 10f, 89f);
                }
            }
        }

        /// <summary>Where the camera wants to be and look, for the current mode.</summary>
        public (Vector3 at, Vector3 look) Want()
        {
            var mode = Fleet.Cameras.Current;
            float fov = _cam.fieldOfView * Mathf.Deg2Rad;
            if (!mode.Equals(_last))
            {
                if (mode.Kind == CameraKind.Stop) Stop = mode.Stop ?? Busiest();
                if (mode.Kind == CameraKind.Orbit && _orbitDist < 0f) _orbitDist = Fit(_campus, fov, 1f);
                _last = mode;
            }
            switch (mode.Kind)
            {
                case CameraKind.Bus:
                {
                    var t = Fleet.Buses[Math.Min(mode.Bus, Fleet.Buses.Count) - 1].transform;
                    var look = t.position + Vector3.up * 0.05f;
                    return (look - t.forward * (1.6f * _zoom) + Vector3.up * (0.6f * _zoom), look);
                }
                case CameraKind.Route:
                {
                    var bus = Fleet.Followed;
                    var b = new Bounds(bus.transform.position, Vector3.one * 2f);
                    if (bus.Driving && bus.LegFrom != null && bus.LegTo != null &&
                        Fleet.Routes.Stops[bus.LegFrom].node != Fleet.Routes.Stops[bus.LegTo].node)
                    {
                        var xyz = Fleet.Routes.Route(bus.LegFrom, bus.LegTo);
                        for (int i = 0; i + 2 < xyz.Length; i += 3) b.Encapsulate(TwinCar.ToUnity(xyz[i], xyz[i + 1], xyz[i + 2]));
                    }
                    return Above(b, 58f, 0f, Fit(b, fov, 1.5f) * _zoom);
                }
                case CameraKind.Fleet:
                {
                    var b = new Bounds(Fleet.Buses[0].transform.position, Vector3.zero);
                    foreach (var bus in Fleet.Buses) b.Encapsulate(bus.transform.position);
                    return Above(b, 52f, 0f, Mathf.Max(3f, Fit(b, fov, 1.2f)) * _zoom);
                }
                case CameraKind.Overview:
                    return Above(_campus, 89f, 0f, Fit(_campus, fov, 1f) * _zoom);
                case CameraKind.Orbit:
                    return Above(_campus, _orbitPitch, _orbitYaw, _orbitDist * _zoom);
                default:
                {
                    // From the kerb at the stop, a little back along its road, looking along the berth.
                    var s = Fleet.Routes.Stops[Stop];
                    var node = TwinCar.ToUnity(s.nodeX, s.nodeY, 0.0);
                    string toward = Fleet.Routes.StopNames().First(x => Fleet.Routes.Stops[x].node != s.node);
                    var xyz = Fleet.Routes.Route(Stop, toward);
                    var along = (TwinCar.ToUnity(xyz[3], xyz[4], 0.0) - node).normalized;
                    var kerb = Fleet.KerbSlot(Stop, 0);
                    kerb.y = 0f;
                    var side = (kerb - node).normalized;
                    var at = node + side * (1.1f * _zoom) - along * (1.4f * _zoom) + Vector3.up * (0.45f * _zoom);
                    return (at, node + along * 0.3f + Vector3.up * 0.08f);
                }
            }
        }

        private void LateUpdate()
        {
            if (Fleet == null || Fleet.Cameras == null || Fleet.Buses.Count == 0) return;
            var (want, look) = Want();
            float k = 1f - Mathf.Exp(-stiffness * Time.unscaledDeltaTime);
            transform.position = Vector3.Lerp(transform.position, want, k);
            var dir = look - transform.position;
            if (dir.sqrMagnitude > 1e-8f)
                transform.rotation = Quaternion.Slerp(transform.rotation, Quaternion.LookRotation(dir, Vector3.up), k);
        }

        // The stop with the most students waiting (the first by name on a tie).
        private string Busiest()
        {
            var w = Fleet.Waiting;
            return Fleet.Routes.StopNames().OrderByDescending(s => w.Count(r => r.Origin == s)).First();
        }

        private static float Fit(Bounds b, float fov, float pad) =>
            (0.5f * Mathf.Max(b.size.x, b.size.z) + pad) / Mathf.Tan(0.5f * fov) * 1.3f;

        private static (Vector3, Vector3) Above(Bounds b, float pitch, float yaw, float dist) =>
            (b.center + Quaternion.Euler(pitch, yaw, 0f) * Vector3.back * dist, b.center);
    }
}
