// Turns a label to face the main camera every frame, so text never reads
// mirrored (the campus's stop names).
using UnityEngine;

namespace RcTwin
{
    public class FaceCamera : MonoBehaviour
    {
        private void LateUpdate()
        {
            var cam = Camera.main;
            if (cam != null) transform.rotation = cam.transform.rotation;
        }
    }
}
