// Edit Mode: the campus look (CampusLook, CampusDressing). The dressing's plan
// needs only the committed campus data: every prop keeps off the asphalt
// (lamps and the stops' benches on the sidewalks, the rest beyond them), out
// of the buildings and clear of the stops, the same each build. The prepared
// models (tools/prepare_assets.py, gitignored) are checked when present:
// three LODs named for Unity's LODGroup, the sizes the plan assumes, and no
// colliders once placed.
using System.Collections.Generic;
using System.IO;
using System.Linq;
using NUnit.Framework;
using RcTwin.EditorTools;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace RcTwin.Tests
{
    public class CampusLookTests
    {
        private static CampusLayout _layout;
        private static List<CampusDressing.Placement> _plan;

        private static CampusLayout Layout => _layout ??= CampusLayout.Load(CampusSceneBuilder.NetworkPath, CampusScenery.SceneryPath);
        private static List<CampusDressing.Placement> Plan => _plan ??= CampusDressing.Plan(Layout);

        [Test]
        public void TheDressingKeepsOffTheRoads()
        {
            var L = Layout;
            foreach (var p in Plan)
            {
                Assert.IsTrue(!p.OnSidewalk || p.Kind == CampusDressing.Kind.Lamp || p.Kind == CampusDressing.Kind.Bench,
                              $"{p.Prop} on a sidewalk");
                foreach (var (c, r) in p.Footprint)
                {
                    float d = L.RoadDistance(c);
                    Assert.GreaterOrEqual(d, L.HalfWidth + r + CampusDressing.RoadMargin - 1e-4f, $"{p.Prop} at {c} reaches the asphalt");
                    if (!p.OnSidewalk)
                        Assert.GreaterOrEqual(d, L.HalfWidth + L.Walk + r - 1e-4f, $"{p.Prop} at {c} stands on the sidewalk");
                }
            }
        }

        [Test]
        public void TheDressingKeepsOutOfBuildingsAndClearOfTheStops()
        {
            var L = Layout;
            foreach (var p in Plan)
                foreach (var (c, r) in p.Footprint)
                {
                    Assert.GreaterOrEqual(L.BuildingClearance(c), r, $"{p.Prop} at {c} in a building");
                    Assert.GreaterOrEqual(L.SideRoadClearance(c), r - 1e-4f, $"{p.Prop} at {c} on a side road");
                    foreach (var s in L.Stops)
                    {
                        Assert.GreaterOrEqual(Vector2.Distance(c, s.node), CampusDressing.StopClear + r - 1e-4f, $"{p.Prop} at stop {s.name}");
                        Assert.GreaterOrEqual(Vector2.Distance(c, L.Shelter(s)), CampusDressing.ShelterClear + r - 1e-4f, $"{p.Prop} in {s.name}'s shelter");
                    }
                }
        }

        [Test]
        public void TheDressingIsTheSameEachBuildAndFillsTheCampus()
        {
            var again = CampusDressing.Plan(Layout);
            Assert.AreEqual(Plan.Count, again.Count);
            for (int i = 0; i < Plan.Count; i++)
            {
                Assert.AreEqual(Plan[i].Prop, again[i].Prop);
                Assert.AreEqual(Plan[i].At, again[i].At);
            }
            int Count(CampusDressing.Kind k) => Plan.Count(p => p.Kind == k);
            Assert.Greater(Count(CampusDressing.Kind.Tree), 200, "trees");
            Assert.Greater(Count(CampusDressing.Kind.Lamp), 40, "lamps");
            Assert.Greater(Count(CampusDressing.Kind.Shrub), 100, "shrubs");
            Assert.GreaterOrEqual(Count(CampusDressing.Kind.Bench), Layout.Stops.Count, "a bench at every stop, and more");
        }

        [Test]
        public void ThePreparedModelsHaveThreeLodsOfTheAssumedSize()
        {
            if (!CampusLook.Available) Assert.Ignore("no prepared look (tools/fetch_assets.py, tools/prepare_assets.py)");
            var doc = (Dictionary<string, object>)MiniJson.Parse(File.ReadAllText(CampusLook.ManifestFile));
            var props = (Dictionary<string, object>)doc["props"];
            foreach (var prop in CampusDressing.Props)
            {
                var m = (Dictionary<string, object>)props[prop.Name];
                float radius = System.Convert.ToSingle(m["radius"]), height = System.Convert.ToSingle(m["height"]);
                float reach = prop.Kind == CampusDressing.Kind.Bench ? prop.HalfLength + prop.Base : Mathf.Max(prop.Base, prop.Crown);
                Assert.LessOrEqual(radius, reach, $"{prop.Name}: the plan's radius covers the model");
                Assert.AreEqual(prop.Height, height, 0.05f * prop.Height, $"{prop.Name}: height");

                var model = CampusLook.Model(prop.Name);
                Assert.IsNotNull(model, prop.Name);
                var group = model.GetComponent<LODGroup>();
                Assert.IsNotNull(group, $"{prop.Name}: Unity built a LODGroup from the _LOD names");
                var lods = group.GetLODs();
                Assert.AreEqual(3, lods.Length, prop.Name);
                for (int i = 0; i < 3; i++)
                    Assert.IsTrue(lods[i].renderers.Length > 0 && lods[i].renderers.All(r => r.name == $"{prop.Name}_LOD{i}"),
                                  $"{prop.Name}: LOD {i} is {prop.Name}_LOD{i}");
                var placed = Object.Instantiate(model);
                var b = placed.GetComponent<LODGroup>().GetLODs()[0].renderers[0].bounds;
                Object.DestroyImmediate(placed);
                Assert.AreEqual(height, b.max.y, 0.05f * height + 0.002f, $"{prop.Name}: imported at the twin's scale, standing on its origin");
            }
        }

        [Test]
        public void ThePlacedDressingHasNoColliders()
        {
            if (!CampusLook.Available) Assert.Ignore("no prepared look (tools/fetch_assets.py, tools/prepare_assets.py)");
            EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var root = CampusDressing.Build(Plan, Layout);
            Assert.IsNotNull(root);
            Assert.AreEqual(0, root.GetComponentsInChildren<Collider>(true).Length, "drawn only");
            Assert.Greater(root.GetComponentsInChildren<LODGroup>(true).Length, 200);
            Assert.AreEqual(Plan.Count(p => p.Kind == CampusDressing.Kind.Lamp), root.GetComponentsInChildren<Light>(true).Length, "a light per lamp");
            Object.DestroyImmediate(root);
        }

        [Test]
        public void TheSkysSunIsWhereTheLightComesFrom()
        {
            // Skybox/Panoramic draws direction d from the texel at ToRadialCoords(rotate(d, -rotation)).
            static Vector2 Radial(Vector3 d)
            {
                d.Normalize();
                return new Vector2(0.5f - Mathf.Atan2(d.z, d.x) / (2f * Mathf.PI), 1f - Mathf.Acos(d.y) / Mathf.PI);
            }
            foreach (var (u, v) in new[] { (0.6f, 0.514f), (0.1f, 0.7f), (0.93f, 0.55f) })
            {
                var d = CampusLook.PanoramaDirection(u, v, 0f);
                var back = Radial(d);
                Assert.AreEqual(u, Mathf.Repeat(back.x, 1f), 1e-4f);
                Assert.AreEqual(v, back.y, 1e-4f);
                foreach (var toward in new[] { Vector3.forward, new Vector3(1f, 0f, -1f).normalized, Vector3.left })
                {
                    var sun = CampusLook.PanoramaDirection(u, v, CampusLook.PanoramaRotation(u, toward));
                    var plan = new Vector3(sun.x, 0f, sun.z).normalized;
                    Assert.Greater(Vector3.Dot(plan, toward), 0.9999f, $"sun at {u},{v} turned toward {toward}");
                }
            }
        }

        [Test]
        public void BoxUVsTileTheWorldAlongEachFace()
        {
            var m = new Mesh
            {
                vertices = new[] { new Vector3(1f, 0f, 2f), new Vector3(0.5f, 0.3f, 4f) },
                normals = new[] { Vector3.up, Vector3.right },
            };
            m.triangles = new[] { 0, 1, 0 };
            CampusLook.BoxUV(m, 0.5f);
            Assert.AreEqual(new Vector2(2f, 4f), m.uv[0], "a top face: plan x, z");
            Assert.AreEqual(new Vector2(8f, 0.6f), m.uv[1], "a wall facing x: z across, y up");
        }

        [Test]
        public void TheHudsPanelsAndIconsAreDrawn()
        {
            var panel = HudSkin.Rounded(10, new Color(0f, 0f, 0f, 0.7f), new Color(1f, 1f, 1f, 0.2f));
            Assert.AreEqual(0f, panel.GetPixel(0, 0).a, 0.01f, "rounded: the corner is clear");
            Assert.AreEqual(0.7f, panel.GetPixel(panel.width / 2, panel.height / 2).a, 0.02f, "translucent middle");
            foreach (HudIcon icon in System.Enum.GetValues(typeof(HudIcon)))
            {
                var t = HudSkin.Icon(icon);
                var a = t.GetPixels().Select(c => c.a).ToArray();
                float ink = a.Average();
                Assert.That(ink, Is.InRange(0.08f, 0.7f), $"{icon}: some ink, not a block");
                Assert.AreEqual(0f, t.GetPixel(0, t.height - 1).a, 0.01f, $"{icon}: a clear corner");
                Assert.Greater(a.Max(), 0.99f, $"{icon}: solid strokes");
            }
        }
    }
}
