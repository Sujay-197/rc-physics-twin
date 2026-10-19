// Menu: RcTwin > Build Flat Ground Scene.
//
// Builds Assets/Scenes/FlatGround.unity from code so the scene is reviewable
// as a diff: a checkered ground plane, the reference car (proportions from the
// Core's provisional defaults) and a chase camera.
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace RcTwin.EditorTools
{
    public static class FlatGroundSceneBuilder
    {
        private const string ScenePath = "Assets/Scenes/FlatGround.unity";
        private const string GenDir = "Assets/RcTwin/Generated";

        [MenuItem("RcTwin/Build Flat Ground Scene")]
        public static void Build()
        {
            if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo()) return;
            var scene = EditorSceneManager.NewScene(NewSceneSetup.DefaultGameObjects, NewSceneMode.Single);

            var p = RcTwinNative.DefaultParams();

            // Ground: 500 m square with 1 m checks, so speed and turning read.
            var ground = GameObject.CreatePrimitive(PrimitiveType.Plane);
            ground.name = "Ground (flat, z = 0)";
            ground.transform.localScale = new Vector3(50f, 1f, 50f);
            ground.GetComponent<Renderer>().sharedMaterial = CheckerMaterial(250f);
            Object.DestroyImmediate(ground.GetComponent<Collider>());  // the Core owns contact

            // Car root sits at the CoG; children are placed from the params.
            var car = new GameObject("TwinCar");
            var twin = car.AddComponent<TwinCar>();
            float lf = (float)p.lf, lr = (float)p.lr, r = (float)p.wheelRadius, h = (float)p.hCog;
            float tf = (float)p.trackFront, tr = (float)p.trackRear;

            var body = GameObject.CreatePrimitive(PrimitiveType.Cube);
            body.name = "Body";
            Object.DestroyImmediate(body.GetComponent<Collider>());
            body.transform.SetParent(car.transform, false);
            body.transform.localScale = new Vector3(Mathf.Min(tf, tr) * 0.75f, 0.05f, lf + lr + 0.1f);
            body.transform.localPosition = new Vector3(0f, r - h + 0.04f, (lf - lr) * 0.5f);
            body.GetComponent<Renderer>().sharedMaterial = ColourMaterial("Body", new Color(0.85f, 0.25f, 0.1f));

            // Core order FL, FR, RL, RR. ISO y is left, Unity x is right.
            var wheelPos = new[]
            {
                new Vector3(-tf * 0.5f, r - h, lf), new Vector3(tf * 0.5f, r - h, lf),
                new Vector3(-tr * 0.5f, r - h, -lr), new Vector3(tr * 0.5f, r - h, -lr),
            };
            var names = new[] { "FL", "FR", "RL", "RR" };
            var tyre = ColourMaterial("Tyre", new Color(0.08f, 0.08f, 0.08f));
            for (int i = 0; i < 4; i++)
            {
                var pivot = new GameObject("Wheel " + names[i]);
                pivot.transform.SetParent(car.transform, false);
                pivot.transform.localPosition = wheelPos[i];
                var mesh = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                mesh.name = "Tyre";
                Object.DestroyImmediate(mesh.GetComponent<Collider>());
                mesh.transform.SetParent(pivot.transform, false);
                mesh.transform.localRotation = Quaternion.Euler(0f, 0f, 90f);  // axle along x
                mesh.transform.localScale = new Vector3(2f * r, 0.0175f, 2f * r);
                mesh.GetComponent<Renderer>().sharedMaterial = tyre;
                // A spoke so wheel spin is visible.
                var spoke = GameObject.CreatePrimitive(PrimitiveType.Cube);
                spoke.name = "Spoke";
                Object.DestroyImmediate(spoke.GetComponent<Collider>());
                spoke.transform.SetParent(pivot.transform, false);
                spoke.transform.localScale = new Vector3(0.04f, 2f * r * 0.9f, 0.012f);
                spoke.GetComponent<Renderer>().sharedMaterial = ColourMaterial("Spoke", Color.white);
                twin.wheels[i] = pivot.transform;
            }
            car.transform.position = new Vector3(0f, h, 0f);

            var cam = Camera.main;
            if (cam != null)
            {
                cam.nearClipPlane = 0.02f;
                cam.transform.position = new Vector3(0f, 0.8f, -1.8f);
                cam.gameObject.AddComponent<ChaseCamera>().target = car.transform;
            }

            Directory.CreateDirectory(Path.GetDirectoryName(ScenePath));
            EditorSceneManager.SaveScene(scene, ScenePath);
            EditorBuildSettings.scenes = new[] { new EditorBuildSettingsScene(ScenePath, true) };
            Debug.Log($"Built {ScenePath}. Press Play; W/S throttle, A/D steer, R reset.");
        }

        private static Material ColourMaterial(string name, Color c)
        {
            string path = $"{GenDir}/{name}.mat";
            var m = AssetDatabase.LoadAssetAtPath<Material>(path);
            if (m == null)
            {
                Directory.CreateDirectory(GenDir);
                m = new Material(LitShader());
                AssetDatabase.CreateAsset(m, path);
            }
            m.color = c;
            if (m.HasProperty("_BaseColor")) m.SetColor("_BaseColor", c);
            EditorUtility.SetDirty(m);
            return m;
        }

        private static Material CheckerMaterial(float tiles)
        {
            Directory.CreateDirectory(GenDir);
            string texPath = $"{GenDir}/Checker.png";
            if (!File.Exists(texPath))
            {
                var t = new Texture2D(2, 2, TextureFormat.RGBA32, false);
                var a = new Color(0.55f, 0.57f, 0.6f);
                var b = new Color(0.42f, 0.44f, 0.47f);
                t.SetPixels(new[] { a, b, b, a });
                File.WriteAllBytes(texPath, t.EncodeToPNG());
                Object.DestroyImmediate(t);
                AssetDatabase.ImportAsset(texPath);
                var imp = (TextureImporter)AssetImporter.GetAtPath(texPath);
                imp.filterMode = FilterMode.Point;
                imp.mipmapEnabled = false;
                imp.SaveAndReimport();
            }
            var tex = AssetDatabase.LoadAssetAtPath<Texture2D>(texPath);
            var m = ColourMaterial("Ground", Color.white);
            m.mainTexture = tex;
            m.mainTextureScale = new Vector2(tiles, tiles);
            if (m.HasProperty("_BaseMap"))
            {
                m.SetTexture("_BaseMap", tex);
                m.SetTextureScale("_BaseMap", new Vector2(tiles, tiles));
            }
            EditorUtility.SetDirty(m);
            AssetDatabase.SaveAssets();
            return m;
        }

        private static Shader LitShader() =>
            Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
    }
}
