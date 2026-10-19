using UnityEngine;

namespace RcTwin
{
    /// <summary>Follows a target from behind and above, smoothed.</summary>
    public class ChaseCamera : MonoBehaviour
    {
        public Transform target;
        public float distance = 1.6f;
        public float height = 0.6f;
        public float stiffness = 6f;

        private void LateUpdate()
        {
            if (target == null) return;
            Vector3 want = target.position - target.forward * distance + Vector3.up * height;
            float k = 1f - Mathf.Exp(-stiffness * Time.deltaTime);
            transform.position = Vector3.Lerp(transform.position, want, k);
            transform.LookAt(target.position + Vector3.up * 0.05f);
        }
    }
}
