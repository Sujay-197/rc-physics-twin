// The shuttle HUD's look (ShuttleHud), after the concept render: Inter
// (SIL OFL 1.1, THIRD_PARTY.md; Generated/Look/Resources/RcTwinLook from
// tools/prepare_assets.py, the built-in font without it), rounded
// translucent panels and simple icons. Panels and icons are drawn here in
// code, anti-aliased from signed distances: no texture assets.
using System.Collections.Generic;
using UnityEngine;

namespace RcTwin
{
    public enum HudIcon { Battery, Seat, Score, Clock, Bolt }

    public static class HudSkin
    {
        public static readonly Color Panel = new Color(0.05f, 0.07f, 0.1f, 0.7f);
        public static readonly Color Edge = new Color(1f, 1f, 1f, 0.12f);
        public static readonly Color Text = new Color(0.95f, 0.96f, 0.98f);
        public static readonly Color Muted = new Color(0.68f, 0.73f, 0.8f);
        public static readonly Color Accent = new Color(0.27f, 0.6f, 1f);
        public static readonly Color Gold = new Color(1f, 0.78f, 0.3f);
        public static readonly Color Good = new Color(0.3f, 0.82f, 0.48f);
        public static readonly Color Warn = new Color(0.98f, 0.72f, 0.22f);
        public static readonly Color Bad = new Color(0.95f, 0.32f, 0.28f);

        private static readonly Dictionary<string, Font> Fonts = new Dictionary<string, Font>();
        private static readonly Dictionary<HudIcon, Texture2D> Icons = new Dictionary<HudIcon, Texture2D>();

        /// <summary>Inter at a weight (Regular, Medium, SemiBold, Bold), or null without it.</summary>
        public static Font Font(string weight)
        {
            if (!Fonts.TryGetValue(weight, out var f)) Fonts[weight] = f = Resources.Load<Font>($"RcTwinLook/Inter-{weight}");
            return f;
        }

        /// <summary>A rounded rectangle with a one-pixel edge, for a GUIStyle background
        /// 9-sliced with a border of radius + 1.</summary>
        public static Texture2D Rounded(int radius, Color fill, Color edge)
        {
            int size = 2 * radius + 4;
            var t = new Texture2D(size, size, TextureFormat.RGBA32, false) { hideFlags = HideFlags.HideAndDontSave, wrapMode = TextureWrapMode.Clamp };
            var px = new Color[size * size];
            float half = size / 2f;
            for (int y = 0; y < size; y++)
                for (int x = 0; x < size; x++)
                {
                    var p = new Vector2(x + 0.5f - half, y + 0.5f - half);
                    float d = RoundBox(p, new Vector2(half - 0.5f, half - 0.5f), radius);   // px, negative inside
                    float inside = Mathf.Clamp01(0.5f - d);
                    float ring = Mathf.Clamp01(1f - Mathf.Abs(d + 0.75f)) * edge.a;   // a soft pixel just inside the edge
                    var c = Color.Lerp(fill, new Color(edge.r, edge.g, edge.b, 1f), ring);
                    c.a = Mathf.Lerp(fill.a, 1f, ring) * inside;
                    px[y * size + x] = c;
                }
            t.SetPixels(px);
            t.Apply();
            return t;
        }

        /// <summary>The icon in white on clear, size x size, to tint with GUI.color.</summary>
        public static Texture2D Icon(HudIcon icon, int size = 64)
        {
            if (Icons.TryGetValue(icon, out var cached) && cached != null) return cached;
            var t = new Texture2D(size, size, TextureFormat.RGBA32, true) { hideFlags = HideFlags.HideAndDontSave, wrapMode = TextureWrapMode.Clamp };
            var px = new Color[size * size];
            float pixel = 2f / size;
            for (int y = 0; y < size; y++)
                for (int x = 0; x < size; x++)
                {
                    var p = new Vector2((x + 0.5f) * pixel - 1f, (y + 0.5f) * pixel - 1f);   // y up
                    px[y * size + x] = new Color(1f, 1f, 1f, Mathf.Clamp01(0.5f - Distance(icon, p) / pixel));
                }
            t.SetPixels(px);
            t.Apply();
            Icons[icon] = t;
            return t;
        }

        /// <summary>Signed distance from p (the icon's square is -1..1, y up) to its
        /// shape: negative inside.</summary>
        public static float Distance(HudIcon icon, Vector2 p)
        {
            switch (icon)
            {
                case HudIcon.Battery:
                {
                    float body = Mathf.Abs(RoundBox(p - new Vector2(-0.08f, 0f), new Vector2(0.78f, 0.46f), 0.14f)) - 0.07f;
                    float nub = RoundBox(p - new Vector2(0.82f, 0f), new Vector2(0.09f, 0.2f), 0.05f);
                    float cells = Mathf.Min(RoundBox(p - new Vector2(-0.48f, 0f), new Vector2(0.17f, 0.25f), 0.04f),
                                            RoundBox(p - new Vector2(-0.08f, 0f), new Vector2(0.17f, 0.25f), 0.04f));
                    return Mathf.Min(body, Mathf.Min(nub, cells));
                }
                case HudIcon.Seat:   // a person: head and shoulders
                {
                    float head = (p - new Vector2(0f, 0.4f)).magnitude - 0.32f;
                    float shoulders = RoundBox(p - new Vector2(0f, -0.64f), new Vector2(0.72f, 0.44f), 0.4f);
                    return Mathf.Min(head, shoulders);
                }
                case HudIcon.Score:   // a five-pointed star
                {
                    var star = new Vector2[10];
                    for (int i = 0; i < 10; i++)
                    {
                        float a = Mathf.PI / 2f + i * Mathf.PI / 5f, r = i % 2 == 0 ? 0.98f : 0.42f;
                        star[i] = new Vector2(Mathf.Cos(a), Mathf.Sin(a)) * r - new Vector2(0f, 0.05f);
                    }
                    return Polygon(p, star);
                }
                case HudIcon.Clock:
                {
                    float ring = Mathf.Abs(p.magnitude - 0.8f) - 0.09f;
                    float hands = Mathf.Min(Segment(p, Vector2.zero, new Vector2(0f, 0.52f)), Segment(p, Vector2.zero, new Vector2(0.38f, 0f))) - 0.08f;
                    return Mathf.Min(ring, hands);
                }
                default:   // Bolt
                    return Polygon(p, new[]
                    {
                        new Vector2(0.18f, 1f), new Vector2(-0.55f, -0.12f), new Vector2(-0.04f, -0.12f),
                        new Vector2(-0.22f, -1f), new Vector2(0.58f, 0.16f), new Vector2(0.06f, 0.16f),
                    });
            }
        }

        private static float RoundBox(Vector2 p, Vector2 half, float r)
        {
            var q = new Vector2(Mathf.Abs(p.x) - half.x + r, Mathf.Abs(p.y) - half.y + r);
            return new Vector2(Mathf.Max(q.x, 0f), Mathf.Max(q.y, 0f)).magnitude + Mathf.Min(Mathf.Max(q.x, q.y), 0f) - r;
        }

        private static float Segment(Vector2 p, Vector2 a, Vector2 b)
        {
            Vector2 ab = b - a;
            float t = Mathf.Clamp01(Vector2.Dot(p - a, ab) / ab.sqrMagnitude);
            return (p - (a + t * ab)).magnitude;
        }

        private static float Polygon(Vector2 p, Vector2[] v)
        {
            float d = float.MaxValue;
            bool inside = false;
            for (int i = 0, j = v.Length - 1; i < v.Length; j = i++)
            {
                d = Mathf.Min(d, Segment(p, v[j], v[i]));
                if ((v[i].y > p.y) != (v[j].y > p.y) && p.x < (v[j].x - v[i].x) * (p.y - v[i].y) / (v[j].y - v[i].y) + v[i].x)
                    inside = !inside;
            }
            return inside ? -d : d;
        }
    }
}
