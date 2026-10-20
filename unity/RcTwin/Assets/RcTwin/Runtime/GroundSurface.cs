using UnityEngine;

namespace RcTwin
{
    /// <summary>
    /// Grip of the surface this collider represents, as a multiplier on the
    /// tyre's peak friction (1 = the surface the tyre coefficients describe).
    /// Read by RaycastGround from the hit collider or its parents.
    /// </summary>
    public class GroundSurface : MonoBehaviour
    {
        [Min(0f)] public float muScale = 1f;
    }
}
