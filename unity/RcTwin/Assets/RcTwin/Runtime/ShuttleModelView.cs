// The campus shuttle's model (tools/make_figures.py, prepared into the
// gitignored Generated/Look/) on the car's sprung body. ShuttleLook builds
// the shuttle from boxes and adds this with the model when it is there;
// when the scene loads, the model is placed, fitted to the boxes' size, and
// the boxes are hidden. Without the model the boxes stay. Drawn only: the
// model has no colliders, and none is added.
using UnityEngine;
using UnityEngine.Rendering;

namespace RcTwin
{
    public sealed class ShuttleModelView : MonoBehaviour
    {
        public const string ModelName = "Shuttle model";

        public GameObject model;
        /// <summary>The size to fit [m]: width, height (scaled with the plan), length.</summary>
        public Vector3 size = new Vector3(0.25f, 0f, 0.42f);

        private void Awake() => Spawn();

        /// <summary>Places the model under the body, once; returns it, or null without one.</summary>
        public GameObject Spawn()
        {
            if (model == null) return null;
            var existing = transform.Find(ModelName);
            if (existing != null) return existing.gameObject;
            foreach (var r in GetComponentsInChildren<Renderer>(true)) r.enabled = false;   // the boxes
            var go = Instantiate(model, transform, false);
            go.name = ModelName;
            var b = Bounds(go);
            float sx = size.x / Mathf.Max(1e-6f, b.size.x), sz = size.z / Mathf.Max(1e-6f, b.size.z);
            go.transform.localScale = new Vector3(sx, 0.5f * (sx + sz), sz);
            foreach (var r in go.GetComponentsInChildren<Renderer>(true)) r.shadowCastingMode = ShadowCastingMode.On;
            foreach (var c in go.GetComponentsInChildren<Collider>(true)) DestroyImmediate(c);
            float front = b.max.z * sz;
            foreach (float s in new[] { -1f, 1f })
            {
                var lamp = new GameObject(s < 0f ? "Headlight left" : "Headlight right");
                lamp.transform.SetParent(go.transform.parent, false);
                lamp.transform.localPosition = new Vector3(s * size.x * 0.38f, 0.03f, front);
                lamp.transform.localRotation = Quaternion.Euler(8f, 0f, 0f);
                var l = lamp.AddComponent<Light>();
                l.type = LightType.Spot;
                l.color = new Color(1f, 0.95f, 0.85f);
                l.spotAngle = 70f;
                l.innerSpotAngle = 35f;
                l.range = 1.6f;
                l.intensity = 0.5f;
                l.shadows = LightShadows.None;
            }
            return go;
        }

        // The model's bounds in the body's frame, from its meshes.
        private Bounds Bounds(GameObject go)
        {
            var b = new Bounds();
            bool first = true;
            foreach (var mf in go.GetComponentsInChildren<MeshFilter>(true))
            {
                if (mf.sharedMesh == null) continue;
                var mb = mf.sharedMesh.bounds;
                for (int i = 0; i < 8; i++)
                {
                    var corner = mb.center + Vector3.Scale(mb.extents, new Vector3((i & 1) == 0 ? -1 : 1, (i & 2) == 0 ? -1 : 1, (i & 4) == 0 ? -1 : 1));
                    var p = transform.InverseTransformPoint(mf.transform.TransformPoint(corner));
                    if (first) { b = new Bounds(p, Vector3.zero); first = false; }
                    else b.Encapsulate(p);
                }
            }
            return b;
        }
    }
}
