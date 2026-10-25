// A student in the campus shuttle game: a figure at 1/10 scale that waits at
// its stop's kerb, walks to the shuttle's door and boards, or steps off and
// walks away. The modelled student (tools/make_figures.py, in
// Generated/Look/Resources/RcTwinLook/) when it is there, its shirt in the
// student's colour and its legs and arms swinging as it walks; else a capsule
// and a sphere. Visual only: no colliders, nothing the car can touch.
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
        private float _maxTime, _stride;
        private Transform _legL, _legR, _armL, _armR;

        private static Material _base;
        private static GameObject _model;
        private static bool _looked;
        private static readonly Color[] Skins =
        {
            new Color(0.62f, 0.43f, 0.32f), new Color(0.48f, 0.32f, 0.23f), new Color(0.76f, 0.57f, 0.45f),
            new Color(0.38f, 0.25f, 0.18f), new Color(0.84f, 0.67f, 0.55f),
        };

        /// <summary>The modelled student, or null without the prepared look.</summary>
        public static GameObject Model()
        {
            if (!_looked) { _model = Resources.Load<GameObject>("RcTwinLook/student"); _looked = true; }
            return _model;
        }

        public static StudentFigure Make(RideRequest r, Vector3 at, Color clothes)
        {
            var root = new GameObject($"Student {r.Origin}→{r.Destination}");
            root.transform.position = at;
            var f = root.AddComponent<StudentFigure>();
            f.Request = r;
            var model = Model();
            if (model != null)
            {
                var fig = Instantiate(model, root.transform, false);
                fig.name = "Figure";
                foreach (var c in fig.GetComponentsInChildren<Collider>(true)) DestroyImmediate(c);
                var skin = Skins[(int)((uint)(r.RaisedAt * 7919.0 + (r.Destination ?? "").Length * 31) % Skins.Length)];
                var block = new MaterialPropertyBlock();
                foreach (var rend in fig.GetComponentsInChildren<Renderer>(true))
                {
                    var mats = rend.sharedMaterials;
                    for (int i = 0; i < mats.Length; i++)
                    {
                        if (mats[i] == null) continue;
                        bool shirt = mats[i].name.StartsWith("student_shirt"), face = mats[i].name.StartsWith("student_skin");
                        if (!shirt && !face) continue;
                        block.Clear();
                        block.SetColor("_BaseColor", shirt ? clothes : skin);
                        rend.SetPropertyBlock(block, i);
                    }
                }
                f._legL = fig.transform.Find("student_leg_l");
                f._legR = fig.transform.Find("student_leg_r");
                f._armL = fig.transform.Find("student_arm_l");
                f._armR = fig.transform.Find("student_arm_r");
                return f;
            }
            var body = Part(PrimitiveType.Capsule, root.transform, new Vector3(0f, 0.065f, 0f), new Vector3(0.055f, 0.065f, 0.045f), clothes);
            body.name = "Body";
            var head = Part(PrimitiveType.Sphere, root.transform, new Vector3(0f, 0.152f, 0f), Vector3.one * 0.042f,
                            new Color(0.78f, 0.6f, 0.48f));
            head.name = "Head";
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

        // Legs and arms swing with the stride while walking, and rest when standing.
        private void Swing(float deg)
        {
            if (_legL == null) return;
            _legL.localRotation = Quaternion.Euler(deg, 0f, 0f);
            _legR.localRotation = Quaternion.Euler(-deg, 0f, 0f);
            if (_armL != null) _armL.localRotation = Quaternion.Euler(-0.8f * deg, 0f, 0f);
            if (_armR != null) _armR.localRotation = Quaternion.Euler(0.8f * deg, 0f, 0f);
        }

        private void Update()
        {
            if (!_walking) { Swing(0f); return; }
            _stride += Time.deltaTime * WalkSpeed / 0.045f;   // a step every 4.5 cm
            Swing(26f * Mathf.Sin(_stride));
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
