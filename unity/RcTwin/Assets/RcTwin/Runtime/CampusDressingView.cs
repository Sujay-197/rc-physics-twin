// The campus dressing (Editor/CampusDressing.cs) as the scene keeps it: the
// prepared models and where each copy stands. The copies are made when the
// scene loads, so the scene stays small and opens cleanly without the
// gitignored models (the dressing is then empty). Drawn only: the models
// have no colliders, and none is added.
using System;
using UnityEngine;
using UnityEngine.Rendering;

namespace RcTwin
{
    /// <summary>What a dressing prop is (the order of CampusDressing.Kind).</summary>
    public enum DressingKind { Tree, Shrub, Plant, Rock, Lamp, Bench }

    public sealed class CampusDressingView : MonoBehaviour
    {
        [Serializable]
        public struct Model
        {
            public GameObject prefab;
            public DressingKind kind;
            public float height;   // lamps: where the light goes
        }

        public Model[] models = Array.Empty<Model>();
        /// <summary>Per copy: its model, (x, y, z, yaw in degrees) and scale.</summary>
        public int[] which = Array.Empty<int>();
        public Vector4[] at = Array.Empty<Vector4>();
        public float[] scale = Array.Empty<float>();

        private void Awake() => Spawn();

        /// <summary>Makes the copies, once; returns how many.</summary>
        public int Spawn()
        {
            if (transform.childCount > 0) return 0;
            var groups = new Transform[Enum.GetValues(typeof(DressingKind)).Length];
            for (int k = 0; k < groups.Length; k++)
            {
                groups[k] = new GameObject($"{(DressingKind)k}s").transform;
                groups[k].SetParent(transform, false);
            }
            int n = 0;
            for (int i = 0; i < which.Length; i++)
            {
                var m = models[which[i]];
                if (m.prefab == null) continue;
                var go = Instantiate(m.prefab, groups[(int)m.kind]);
                go.transform.SetPositionAndRotation(new Vector3(at[i].x, at[i].y, at[i].z), Quaternion.Euler(0f, at[i].w, 0f));
                go.transform.localScale = Vector3.one * scale[i];
                bool small = m.kind == DressingKind.Plant || (m.kind == DressingKind.Rock && scale[i] < 0.5f);
                foreach (var r in go.GetComponentsInChildren<Renderer>(true))
                {
                    r.shadowCastingMode = small ? ShadowCastingMode.Off : ShadowCastingMode.On;
                    r.lightProbeUsage = LightProbeUsage.Off;
                }
                if (go.TryGetComponent<LODGroup>(out var lod)) Transitions(lod, m.kind);
                if (m.kind == DressingKind.Lamp) LampLight(go.transform, m.height);
                n++;
            }
            return n;
        }

        /// <summary>Removes the copies (the scene builder bakes reflections with them, then saves without).</summary>
        public void Clear()
        {
            for (int i = transform.childCount - 1; i >= 0; i--)
            {
                var c = transform.GetChild(i).gameObject;
                if (Application.isPlaying) Destroy(c);
                else DestroyImmediate(c);
            }
        }

        // Screen heights where each LOD hands over, and where the prop is culled.
        private static void Transitions(LODGroup group, DressingKind kind)
        {
            float[] h = kind switch
            {
                DressingKind.Tree => new[] { 0.18f, 0.06f, 0.004f },
                DressingKind.Lamp => new[] { 0.2f, 0.06f, 0.006f },
                DressingKind.Plant => new[] { 0.08f, 0.03f, 0.01f },
                _ => new[] { 0.15f, 0.05f, 0.006f },
            };
            var lods = group.GetLODs();
            for (int i = 0; i < lods.Length && i < h.Length; i++) lods[i].screenRelativeTransitionHeight = h[i];
            group.SetLODs(lods);
        }

        private static void LampLight(Transform lamp, float height)
        {
            var go = new GameObject("Lamp light");
            go.transform.SetParent(lamp, false);
            go.transform.localPosition = new Vector3(0f, height * 0.9f, 0f);
            var light = go.AddComponent<Light>();
            light.type = LightType.Point;
            light.color = new Color(1f, 0.72f, 0.42f);
            light.range = 0.9f;
            light.intensity = 0.14f;
            light.shadows = LightShadows.None;
        }
    }
}
