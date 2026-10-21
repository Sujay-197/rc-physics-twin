// Answers the Core's ground query by raycasting down onto the scene's
// colliders (ADR 0005). Unity only reports the ground; the Core does the
// dynamics. Frames: ISO (x, y, z) is Unity (-y, z, x), so a Unity vector
// (ux, uy, uz) is ISO (uz, -ux, uy).
//
// Each wheel's ray starts its climbable step above that wheel's last ground
// height (ADR 0009), so a face taller than the step is never read as ground:
// the wheel stays on the floor and the scene's walls (TwinWalls) stop the
// car. Ramps still climb, and edges still drop. A ray that starts inside a
// collider does not hit it. Forget() (on every reset) casts from castHeight
// again.
using UnityEngine;

namespace RcTwin
{
    public class RaycastGround : MonoBehaviour, IGroundProvider
    {
        [Tooltip("Colliders that count as ground.")]
        public LayerMask groundMask = ~0;

        [Tooltip("Rays start this far above world zero [m].")]
        public float castHeight = 50f;

        public float castDistance = 100f;

        [Tooltip("Tallest rise a wheel climbs in one step [m]; taller faces are walls. " +
                 "0: half the wheel radius, set by TwinCar (ADR 0009, a rule of thumb).")]
        public float climbableStep;

        private readonly double[] _lastHeight = { double.NaN, double.NaN, double.NaN, double.NaN };

        /// <summary>Forget the last heights: the next ray per wheel starts at castHeight.</summary>
        public void Forget()
        {
            for (int i = 0; i < _lastHeight.Length; i++) _lastHeight[i] = double.NaN;
        }

        /// <summary>The last sample per wheel (FL, FR, RL, RR), ISO frame.</summary>
        public readonly RcTwinNative.GroundSample[] Last = new RcTwinNative.GroundSample[4];

        /// <summary>Mean ground normal of the last samples, in Unity's frame.</summary>
        public Vector3 MeanUp
        {
            get
            {
                Vector3 sum = Vector3.zero;
                foreach (var g in Last) sum += new Vector3(-(float)g.ny, (float)g.nz, (float)g.nx);
                return sum.sqrMagnitude > 0f ? sum.normalized : Vector3.up;
            }
        }

        private static double Snap(float c) => Mathf.Abs(c) < 1e-6f ? 0.0 : c;

        public bool Sample(int wheel, double x, double y, ref RcTwinNative.GroundSample g)
        {
            bool known = wheel >= 0 && wheel < _lastHeight.Length && !double.IsNaN(_lastHeight[wheel]);
            double top = known ? _lastHeight[wheel] + climbableStep : castHeight;
            Vector3 origin = TwinCar.ToUnity(x, y, top);
            if (!Physics.Raycast(origin, Vector3.down, out RaycastHit hit, castDistance, groundMask,
                                 QueryTriggerInteraction.Ignore))
                return false;
            if (wheel >= 0 && wheel < _lastHeight.Length) _lastHeight[wheel] = hit.point.y;
            g.height = hit.point.y;
            // Float raycast normals carry noise of order 1e-8 on level colliders;
            // snapping it to zero keeps level ground exactly level for the Core
            // (a slope of 1e-6 rad is below anything a float normal resolves).
            g.nx = Snap(hit.normal.z);
            g.ny = Snap(-hit.normal.x);
            g.nz = hit.normal.y;
            var surface = hit.collider.GetComponentInParent<GroundSurface>();
            g.muScale = surface != null ? surface.muScale : 1.0;
            if (wheel >= 0 && wheel < Last.Length) Last[wheel] = g;
            return true;
        }
    }
}
