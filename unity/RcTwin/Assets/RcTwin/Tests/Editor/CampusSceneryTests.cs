// Edit Mode: the campus scenery's roofs (CampusScenery.EarClip) cover each
// footprint exactly once and face up in Unity, concave footprints included.
using System.Collections.Generic;
using NUnit.Framework;
using RcTwin.EditorTools;
using UnityEngine;

namespace RcTwin.Tests
{
    public class CampusSceneryTests
    {
        private static float Area(List<Vector2> p)
        {
            float a = 0f;
            for (int i = 0; i < p.Count; i++) a += p[i].x * p[(i + 1) % p.Count].y - p[(i + 1) % p.Count].x * p[i].y;
            return 0.5f * a;
        }

        private static void Covers(List<Vector2> poly)
        {
            var tris = CampusScenery.EarClip(poly);
            Assert.AreEqual(3 * (poly.Count - 2), tris.Count, "n - 2 triangles");
            float total = 0f;
            for (int k = 0; k < tris.Count; k += 3)
            {
                Vector3 a = TwinCar.ToUnity(poly[tris[k]].x, poly[tris[k]].y, 0.0);
                Vector3 b = TwinCar.ToUnity(poly[tris[k + 1]].x, poly[tris[k + 1]].y, 0.0);
                Vector3 c = TwinCar.ToUnity(poly[tris[k + 2]].x, poly[tris[k + 2]].y, 0.0);
                Vector3 n = Vector3.Cross(b - a, c - a);           // Unity's front face: this way
                Assert.Greater(n.y, 0f, "every roof triangle faces up");
                total += 0.5f * n.magnitude;
            }
            Assert.AreEqual(Area(poly), total, 1e-4f, "the triangles add up to the footprint");
        }

        [Test]
        public void ARectangle() =>
            Covers(new List<Vector2> { new Vector2(0, 0), new Vector2(2, 0), new Vector2(2, 1), new Vector2(0, 1) });

        [Test]
        public void AnLShape() =>
            Covers(new List<Vector2> { new Vector2(0, 0), new Vector2(3, 0), new Vector2(3, 1), new Vector2(1, 1),
                                       new Vector2(1, 3), new Vector2(0, 3) });

        [Test]
        public void ACross() =>
            Covers(new List<Vector2> { new Vector2(1, 0), new Vector2(2, 0), new Vector2(2, 1), new Vector2(3, 1),
                                       new Vector2(3, 2), new Vector2(2, 2), new Vector2(2, 3), new Vector2(1, 3),
                                       new Vector2(1, 2), new Vector2(0, 2), new Vector2(0, 1), new Vector2(1, 1) });
    }
}
