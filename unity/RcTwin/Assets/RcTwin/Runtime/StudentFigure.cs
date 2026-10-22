// A student in the campus shuttle game: a simple figure at 1/10 scale that
// waits at its stop's kerb, walks to the shuttle's door and boards, or steps
// off and walks away. Visual only: no colliders, nothing the car can touch.
using System;
using UnityEngine;

namespace RcTwin
{
    public sealed class StudentFigure : MonoBehaviour
    {
        public const float WalkSpeed = 0.3f;   // [m/s] at the twin's 1/10 scale, brisk enough to watch

        public RideRequest Request;

        private Vector3 _target;
        private bool _walking, _vanish;
        private float _maxTime;

        private static Material _base;

        public static StudentFigure Make(RideRequest r, Vector3 at, Color clothes)
        {
            var root = new GameObject($"Student {r.Origin}→{r.Destination}");
            root.transform.position = at;
            var body = Part(PrimitiveType.Capsule, root.transform, new Vector3(0f, 0.065f, 0f), new Vector3(0.055f, 0.065f, 0.045f), clothes);
            body.name = "Body";
            var head = Part(PrimitiveType.Sphere, root.transform, new Vector3(0f, 0.152f, 0f), Vector3.one * 0.042f,
                            new Color(0.78f, 0.6f, 0.48f));
            head.name = "Head";
            var f = root.AddComponent<StudentFigure>();
            f.Request = r;
            return f;
        }

        private static GameObject Part(PrimitiveType type, Transform parent, Vector3 local, Vector3 scale, Color c)
        {
            var go = GameObject.CreatePrimitive(type);
            var col = go.GetComponent<Collider>();
            if (col != null) DestroyImmediate(col);
            go.transform.SetParent(parent, false);
            go.transform.localPosition = local;
            go.transform.localScale = scale;
            var r = go.GetComponent<MeshRenderer>();
            r.sharedMaterial = Tinted(c);
            return go;
        }

        private static Material Tinted(Color c)
        {
            if (_base == null)
            {
                var sh = Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
                _base = new Material(sh);
            }
            var m = new Material(_base) { color = c };
            if (m.HasProperty("_BaseColor")) m.SetColor("_BaseColor", c);
            return m;
        }

        /// <summary>Walk to p; vanish there (or after `maxTime` seconds) if asked.</summary>
        public void WalkTo(Vector3 p, bool vanish, float maxTime = 6f)
        {
            _target = p;
            _walking = true;
            _vanish = vanish;
            _maxTime = maxTime;
        }

        private void Update()
        {
            if (!_walking) return;
            _maxTime -= Time.deltaTime;
            var d = _target - transform.position;
            d.y = 0f;
            float step = WalkSpeed * Time.deltaTime;
            if (d.magnitude <= step || _maxTime <= 0f)
            {
                transform.position = new Vector3(_target.x, transform.position.y, _target.z);
                _walking = false;
                if (_vanish) Destroy(gameObject);
                return;
            }
            transform.position += d.normalized * step;
            transform.rotation = Quaternion.LookRotation(d.normalized, Vector3.up);
        }
    }
}
