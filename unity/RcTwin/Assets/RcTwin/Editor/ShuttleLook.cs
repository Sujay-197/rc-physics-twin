// The campus shuttle's look (Docs/specs/2026-10-21-campus-shuttle-design.md,
// "Look"): a white campus shuttle over the twin's car, a shelter at every
// stop, a charging post at the charger. Primitives only, every collider
// removed: the car's footprint, walls and physics are untouched.
using UnityEngine;
using UnityEngine.Rendering;

namespace RcTwin.EditorTools
{
    public static class ShuttleLook
    {
        private static readonly Color White = new Color(0.95f, 0.95f, 0.93f), Red = new Color(0.72f, 0.12f, 0.14f),
            Glass = new Color(0.12f, 0.16f, 0.22f), Door = new Color(0.32f, 0.36f, 0.42f), Roof = new Color(0.82f, 0.83f, 0.84f),
            Lamp = new Color(1f, 0.97f, 0.82f), Tail = new Color(0.85f, 0.08f, 0.06f), Frame = new Color(0.25f, 0.27f, 0.3f),
            Panel = new Color(0.62f, 0.72f, 0.8f), Bench = new Color(0.45f, 0.32f, 0.22f), Green = new Color(0.2f, 0.7f, 0.35f);

        /// <summary>Dress the car's sprung body as a campus shuttle. The body keeps its
        /// position (TwinCar moves it on the springs); its box is hidden and the
        /// shuttle's parts, at the car's 1/10 scale, are its children.</summary>
        public static void BuildShuttleBody(Transform body, RcTwinNative.Params p)
        {
            var box = body.GetComponent<Renderer>();
            if (box != null) box.enabled = false;
            body.localScale = Vector3.one;
            float L = (float)(p.lf + p.lr) + 0.1f, W = (float)System.Math.Min(p.trackFront, p.trackRear) * 0.85f;
            float cabinL = L - 0.04f;
            Part(body, "Shell", new Vector3(W, 0.07f, L - 0.01f), new Vector3(0f, 0.035f, 0f), White);
            Part(body, "Stripe", new Vector3(W + 0.004f, 0.012f, L - 0.006f), new Vector3(0f, 0.076f, 0f), Red);
            Part(body, "Cabin", new Vector3(W, 0.075f, cabinL), new Vector3(0f, 0.12f, -0.005f), White);
            Part(body, "Roof", new Vector3(W - 0.012f, 0.008f, cabinL - 0.02f), new Vector3(0f, 0.161f, -0.005f), Roof);
            Part(body, "Window left", new Vector3(0.003f, 0.036f, cabinL - 0.1f), new Vector3(-W / 2f - 0.0015f, 0.125f, -0.02f), Glass);
            Part(body, "Window right", new Vector3(0.003f, 0.036f, cabinL - 0.05f), new Vector3(W / 2f + 0.0015f, 0.125f, -0.005f), Glass);
            Part(body, "Window front", new Vector3(W - 0.025f, 0.046f, 0.003f), new Vector3(0f, 0.122f, cabinL / 2f - 0.004f), Glass);
            Part(body, "Window rear", new Vector3(W - 0.04f, 0.03f, 0.003f), new Vector3(0f, 0.13f, -cabinL / 2f - 0.006f), Glass);
            Part(body, "Door", new Vector3(0.004f, 0.105f, 0.07f), new Vector3(-W / 2f - 0.002f, 0.088f, L * 0.3f), Door);
            foreach (float s in new[] { -1f, 1f })
            {
                Part(body, $"Headlight {(s < 0 ? "left" : "right")}", new Vector3(0.032f, 0.014f, 0.004f),
                     new Vector3(s * W * 0.32f, 0.04f, (L - 0.01f) / 2f + 0.002f), Lamp);
                Part(body, $"Tail light {(s < 0 ? "left" : "right")}", new Vector3(0.022f, 0.016f, 0.004f),
                     new Vector3(s * W * 0.36f, 0.045f, -(L - 0.01f) / 2f - 0.002f), Tail);
            }
            // The modelled shuttle, when tools/prepare_assets.py has made it: it takes the
            // boxes' place on load, as wide as the car over its tyres, as long as the boxes.
            var model = CampusLook.Model("shuttle");
            if (model != null)
            {
                var view = body.gameObject.AddComponent<ShuttleModelView>();
                view.model = model;
                view.size = new Vector3((float)System.Math.Max(p.trackFront, p.trackRear) + 0.01f, 0f, L);
            }
        }

        /// <summary>A bus shelter at `at`, its open side facing `headingDeg` (Unity yaw).</summary>
        public static GameObject BuildShelter(Vector3 at, float headingDeg)
        {
            var root = new GameObject("Shelter");
            root.transform.SetPositionAndRotation(at, Quaternion.Euler(0f, headingDeg, 0f));
            var t = root.transform;
            Part(t, "Back panel", new Vector3(0.3f, 0.19f, 0.008f), new Vector3(0f, 0.105f, -0.065f), Panel);
            Part(t, "Roof", new Vector3(0.34f, 0.012f, 0.16f), new Vector3(0f, 0.215f, -0.005f), White);
            foreach (float s in new[] { -1f, 1f })
            {
                Part(t, "Post", new Vector3(0.012f, 0.21f, 0.012f), new Vector3(s * 0.16f, 0.105f, 0.065f), Frame);
                Part(t, "Post", new Vector3(0.012f, 0.21f, 0.012f), new Vector3(s * 0.16f, 0.105f, -0.065f), Frame);
                Part(t, "Side panel", new Vector3(0.006f, 0.16f, 0.1f), new Vector3(s * 0.16f, 0.11f, -0.015f), Panel);
            }
            Part(t, "Bench", new Vector3(0.22f, 0.012f, 0.045f), new Vector3(0f, 0.05f, -0.035f), Bench);
            return root;
        }

        /// <summary>A charging post at `at`, its screen facing `headingDeg` (Unity yaw).</summary>
        public static GameObject BuildCharger(Vector3 at, float headingDeg)
        {
            var root = new GameObject("Charger");
            root.transform.SetPositionAndRotation(at, Quaternion.Euler(0f, headingDeg, 0f));
            var t = root.transform;
            Part(t, "Post", new Vector3(0.06f, 0.15f, 0.04f), new Vector3(0f, 0.075f, 0f), White);
            Part(t, "Band", new Vector3(0.062f, 0.025f, 0.042f), new Vector3(0f, 0.13f, 0f), Green);
            Part(t, "Screen", new Vector3(0.036f, 0.028f, 0.003f), new Vector3(0f, 0.095f, 0.0215f), Glass);
            Part(t, "Cable", new Vector3(0.008f, 0.07f, 0.008f), new Vector3(0.034f, 0.06f, 0.01f), Frame);
            return root;
        }

        private static GameObject Part(Transform parent, string name, Vector3 size, Vector3 at, Color c)
        {
            var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
            go.name = name;
            Object.DestroyImmediate(go.GetComponent<Collider>());
            go.transform.SetParent(parent, false);
            go.transform.localPosition = at;
            go.transform.localScale = size;
            var r = go.GetComponent<Renderer>();
            r.sharedMaterial = FlatGroundSceneBuilder.ColourMaterial("Shuttle" + Key(c), c);
            r.shadowCastingMode = ShadowCastingMode.On;
            return go;
        }

        // One material per colour.
        private static string Key(Color c) => $"{(int)(c.r * 255):X2}{(int)(c.g * 255):X2}{(int)(c.b * 255):X2}";
    }
}
