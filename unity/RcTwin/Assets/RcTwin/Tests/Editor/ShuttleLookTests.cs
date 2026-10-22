// Edit Mode: the campus shuttle's look (Docs/specs/2026-10-21-campus-shuttle-design.md,
// "Look"): a white shuttle body, stop shelters and the AB1 charger, all visual
// only. Built in a fresh scene, so no scene or asset on disk changes.
using System.Linq;
using NUnit.Framework;
using RcTwin.EditorTools;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace RcTwin.Tests
{
    public class ShuttleLookTests
    {
        [SetUp]
        public void Fresh() => EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

        // The car as FlatGroundSceneBuilder.BuildCar makes it: a root at the CoG and a Body cube.
        private static (GameObject car, Transform body) Car(RcTwinNative.Params p)
        {
            var car = new GameObject("TwinCar");
            var twin = car.AddComponent<TwinCar>();
            var body = GameObject.CreatePrimitive(PrimitiveType.Cube);
            body.name = "Body";
            Object.DestroyImmediate(body.GetComponent<Collider>());
            body.transform.SetParent(car.transform, false);
            float lf = (float)p.lf, lr = (float)p.lr;
            body.transform.localScale = new Vector3((float)p.trackFront * 0.75f, 0.05f, lf + lr + 0.1f);
            body.transform.localPosition = new Vector3(0f, 0.01f, (lf - lr) * 0.5f);
            twin.body = body.transform;
            return (car, body.transform);
        }

        [Test]
        public void TheShuttleIsVisualOnlyAndFitsTheCar()
        {
            var p = RcTwinNative.DefaultParams();
            var (car, body) = Car(p);
            var twin = car.GetComponent<TwinCar>();
            double margin = twin.footprintMargin;
            var bodyAt = body.localPosition;
            ShuttleLook.BuildShuttleBody(body, p);
            Assert.IsEmpty(car.GetComponentsInChildren<Collider>(), "nothing the physics can touch");
            Assert.AreEqual(margin, twin.footprintMargin, "the footprint is unchanged");
            Assert.AreEqual(bodyAt, body.localPosition, "the sprung body stays where the car moves it");
            var parts = body.GetComponentsInChildren<Renderer>().Where(r => r.transform != body).ToList();
            Assert.GreaterOrEqual(parts.Count, 6, "shell, stripe, windows, door, lights");
            var b = parts[0].bounds;
            foreach (var r in parts) b.Encapsulate(r.bounds);
            float length = (float)(p.lf + p.lr) + 0.1f + 0.02f, width = (float)System.Math.Max(p.trackFront, p.trackRear) + 0.035f;
            Assert.LessOrEqual(b.size.z, length + 1e-4f, "no longer than the car's footprint");
            Assert.LessOrEqual(b.size.x, width + 1e-4f, "no wider than the car with its tyres");
            Assert.IsTrue(parts.Any(r => r.name == "Stripe"));
            Assert.IsTrue(parts.Any(r => r.name.StartsWith("Window")));
            Assert.IsTrue(parts.Any(r => r.name == "Door" && r.transform.localPosition.x < 0f), "the door is on the left, the kerb side");
            Assert.IsTrue(parts.All(r => r.shadowCastingMode != UnityEngine.Rendering.ShadowCastingMode.Off));
            var shell = body.GetComponent<Renderer>();
            Assert.IsTrue(shell == null || !shell.enabled, "the old box is hidden");
        }

        [Test]
        public void TheModelledShuttleFitsTheCarFacesForwardAndHidesTheBoxes()
        {
            if (CampusLook.Model("shuttle") == null) Assert.Ignore("no prepared look (tools/prepare_assets.py)");
            var p = RcTwinNative.DefaultParams();
            var (car, body) = Car(p);
            ShuttleLook.BuildShuttleBody(body, p);
            var view = body.GetComponent<ShuttleModelView>();
            Assert.IsNotNull(view, "the model is offered when it is there");
            var model = view.Spawn();
            Assert.IsNotNull(model);
            Assert.IsEmpty(car.GetComponentsInChildren<Collider>(), "nothing the physics can touch");
            var shown = body.GetComponentsInChildren<Renderer>().Where(r => r.enabled).ToList();
            Assert.IsTrue(shown.All(r => r.transform.IsChildOf(model.transform)), "the boxes are hidden");
            var b = shown[0].bounds;
            foreach (var r in shown) b.Encapsulate(r.bounds);
            float length = (float)(p.lf + p.lr) + 0.1f + 0.02f, width = (float)System.Math.Max(p.trackFront, p.trackRear) + 0.035f;
            Assert.LessOrEqual(b.size.z, length + 1e-3f, "no longer than the car's footprint");
            Assert.LessOrEqual(b.size.x, width + 1e-3f, "no wider than the car with its tyres");
            // Headlights at the front (+z, the car's forward), tail lights at the back.
            float Centre(string material)
            {
                var r = shown.First(x => x.sharedMaterials.Any(m => m != null && m.name == material));
                var mesh = r.GetComponent<MeshFilter>().sharedMesh;
                int i = System.Array.FindIndex(r.sharedMaterials, m => m != null && m.name == material);
                var v = mesh.vertices;
                var idx = mesh.GetIndices(i);
                return idx.Average(k => body.InverseTransformPoint(r.transform.TransformPoint(v[k])).z);
            }
            Assert.Greater(Centre("shuttle_head"), 0.15f);
            Assert.Less(Centre("shuttle_tail"), -0.15f);
            Assert.IsTrue(body.GetComponentsInChildren<Light>().All(l => body.InverseTransformPoint(l.transform.position).z > 0.15f),
                          "headlights shine forward");
        }

        [Test]
        public void TheModelledStudentIsVisualOnlyAndDressedInItsColour()
        {
            if (StudentFigure.Model() == null) Assert.Ignore("no prepared look (tools/prepare_assets.py)");
            var f = StudentFigure.Make(new RideRequest { Origin = "AB1", Destination = "S3", RaisedAt = 12.0 }, Vector3.zero, Color.red);
            Assert.IsEmpty(f.GetComponentsInChildren<Collider>(true));
            Assert.IsNotNull(f.transform.Find("Figure/student_leg_l"), "legs of their own, to swing");
            Assert.IsNotNull(f.transform.Find("Figure/student_arm_r"));
            Assert.IsTrue(f.GetComponentsInChildren<Renderer>().Any(r => r.HasPropertyBlock()), "the shirt in the student's colour");
            var b = f.GetComponentsInChildren<Renderer>()[0].bounds;
            foreach (var r in f.GetComponentsInChildren<Renderer>()) b.Encapsulate(r.bounds);
            Assert.AreEqual(0.17f, b.size.y, 0.02f, "1.7 m at full scale");
            Object.DestroyImmediate(f.gameObject);
        }

        [Test]
        public void SheltersAndTheChargerAreVisualOnly()
        {
            var shelter = ShuttleLook.BuildShelter(new Vector3(1f, 0.06f, 2f), 90f);
            var charger = ShuttleLook.BuildCharger(new Vector3(3f, 0.06f, 2f), 0f);
            foreach (var go in new[] { shelter, charger })
            {
                Assert.IsEmpty(go.GetComponentsInChildren<Collider>(), go.name);
                Assert.Greater(go.GetComponentsInChildren<Renderer>().Length, 1, go.name);
            }
            Assert.AreEqual(90f, shelter.transform.eulerAngles.y, 1e-3f);
            StringAssert.StartsWith("Shelter", shelter.name);
            StringAssert.StartsWith("Charger", charger.name);
        }
    }
}
