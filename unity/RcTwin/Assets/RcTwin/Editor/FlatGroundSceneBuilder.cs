// Menu: RcTwin > Build Flat Ground Scene.
//
// Builds Assets/Scenes/FlatGround.unity from code so the scene is reviewable
// as a diff: a checkered ground plane, the reference car (proportions from the
// Core's provisional defaults), a chase camera, and ground features the Core
// feels through RaycastGround (ADR 0005): a hill (10 % up, 0.6 m plateau,
// 10 % down) straight ahead, a 10 % banked strip to the right and an ice
// patch (grip x0.25) to the left. Unity coordinates: the car starts at the
// origin facing +z.
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
            // Colliders here only answer the ground query; the Core owns contact.

            // Hill: up ramp z 8..14 to 0.6 m, plateau z 14..18, down ramp z 18..24.
            var hillMat = ColourMaterial("Hill", new Color(0.62f, 0.55f, 0.45f));
            Wedge("Hill up (10 %)", "HillUp", 3f, 6f, 0.6f, new Vector3(0f, 0f, 8f), 0f, hillMat);
            var plateau = GameObject.CreatePrimitive(PrimitiveType.Cube);
            plateau.name = "Hill plateau (0.6 m)";
            plateau.transform.position = new Vector3(0f, 0.3f, 16f);
            plateau.transform.localScale = new Vector3(3f, 0.6f, 4f);
            plateau.GetComponent<Renderer>().sharedMaterial = hillMat;
            Wedge("Hill down (10 %)", "HillDown", 3f, 6f, 0.6f, new Vector3(0f, 0f, 24f), 180f, hillMat);

            // Speed breaker: a 1.5 cm round hump, 0.3 m long, across the lane at z 4.
            Hump("Speed breaker (1.5 cm)", "SpeedBreaker", 3f, 0.3f, 0.015f, new Vector3(0f, 0f, 4f),
                 ColourMaterial("Breaker", new Color(0.95f, 0.8f, 0.1f)));

            // Bank: x 4..7 rising 0.3 m toward +x (the car's right), z 0..20.
            Wedge("Bank (10 %, right side high)", "Bank", 20f, 3f, 0.3f, new Vector3(4f, 0f, 10f), 90f,
                  ColourMaterial("Bank", new Color(0.45f, 0.6f, 0.45f)));

            // Ice: x -7..-4, z 0..20, 1 mm proud of the ground, grip x0.25.
            var ice = GameObject.CreatePrimitive(PrimitiveType.Cube);
            ice.name = "Ice patch (grip x0.25)";
            ice.transform.position = new Vector3(-5.5f, 0.0005f, 10f);
            ice.transform.localScale = new Vector3(3f, 0.001f, 20f);
            ice.GetComponent<Renderer>().sharedMaterial = ColourMaterial("Ice", new Color(0.7f, 0.85f, 1f));
            ice.AddComponent<GroundSurface>().muScale = 0.25f;

            var car = BuildCar(p);
            AttachChaseCamera(car.transform);

            Directory.CreateDirectory(Path.GetDirectoryName(ScenePath));
            EditorSceneManager.SaveScene(scene, ScenePath);
            RegisterScene(ScenePath);
            Debug.Log($"Built {ScenePath}. Press Play; W/S throttle, A/D steer, R reset.");
        }

        // The reference car, proportions from the Core's provisional defaults,
        // at the origin facing +z. Shared by every scene builder.
        internal static TwinCar BuildCar(RcTwinNative.Params p)
        {
            // Car root sits at the CoG; children are placed from the params.
            var car = new GameObject("TwinCar");
            car.AddComponent<RaycastGround>();
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
            twin.body = body.transform;

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
            return twin;
        }

        internal static void AttachChaseCamera(Transform target)
        {
            var cam = Camera.main;
            if (cam == null) return;
            cam.nearClipPlane = 0.02f;
            cam.transform.position = new Vector3(target.position.x, 0.8f, target.position.z - 1.8f);
            cam.gameObject.AddComponent<ChaseCamera>().target = target;
        }

        // Adds the scene to the build list if missing, keeping the others.
        internal static void RegisterScene(string path)
        {
            var list = new System.Collections.Generic.List<EditorBuildSettingsScene>(EditorBuildSettings.scenes);
            if (list.Exists(s => s.path == path)) return;
            list.Add(new EditorBuildSettingsScene(path, true));
            EditorBuildSettings.scenes = list.ToArray();
        }

        // A ramp: width across local x, rising from height 0 at local z = 0 to
        // `height` at z = length, placed with its low edge's centre at `origin`
        // and turned by `yawDeg` about +y. Triangles wind clockwise seen from
        // outside, so every face normal points out and a downward ray hits the
        // slope's front face.
        private static void Wedge(string name, string assetName, float width, float length, float height,
                                  Vector3 origin, float yawDeg, Material mat)
        {
            float w = width * 0.5f;
            var v = new[]
            {
                new Vector3(-w, 0, 0), new Vector3(w, 0, 0),                     // 0 1 low edge
                new Vector3(-w, 0, length), new Vector3(w, 0, length),           // 2 3 back bottom
                new Vector3(-w, height, length), new Vector3(w, height, length), // 4 5 top edge
            };
            var tris = new[]
            {
                0, 4, 1, 1, 4, 5,   // slope
                2, 3, 4, 3, 5, 4,   // back wall
                0, 1, 2, 1, 3, 2,   // bottom
                0, 2, 4,            // left side
                1, 5, 3,            // right side
            };
            // One vertex per corner per face, so each face keeps a flat normal.
            var verts = new Vector3[tris.Length];
            var idx = new int[tris.Length];
            for (int i = 0; i < tris.Length; i++) { verts[i] = v[tris[i]]; idx[i] = i; }
            var mesh = new Mesh { name = assetName };
            mesh.vertices = verts;
            mesh.triangles = idx;
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            Directory.CreateDirectory(GenDir);
            string path = $"{GenDir}/{assetName}.asset";
            AssetDatabase.DeleteAsset(path);
            AssetDatabase.CreateAsset(mesh, path);

            var go = new GameObject(name);
            go.transform.SetPositionAndRotation(origin, Quaternion.Euler(0f, yawDeg, 0f));
            go.AddComponent<MeshFilter>().sharedMesh = mesh;
            go.AddComponent<MeshRenderer>().sharedMaterial = mat;
            go.AddComponent<MeshCollider>().sharedMesh = mesh;
        }

        // A round hump across the lane: height h sin^2(pi z / length), 24 strips,
        // centred on `centre`. Top faces only plus a floor, collider on the mesh.
        private static void Hump(string name, string assetName, float width, float length, float h,
                                 Vector3 centre, Material mat)
        {
            const int n = 24;
            float w = width * 0.5f;
            var verts = new System.Collections.Generic.List<Vector3>();
            var tris = new System.Collections.Generic.List<int>();
            for (int i = 0; i < n; i++)
            {
                float z0 = length * i / n, z1 = length * (i + 1) / n;
                float y0 = h * Mathf.Pow(Mathf.Sin(Mathf.PI * z0 / length), 2f);
                float y1 = h * Mathf.Pow(Mathf.Sin(Mathf.PI * z1 / length), 2f);
                int b = verts.Count;
                verts.Add(new Vector3(-w, y0, z0 - length / 2)); verts.Add(new Vector3(w, y0, z0 - length / 2));
                verts.Add(new Vector3(-w, y1, z1 - length / 2)); verts.Add(new Vector3(w, y1, z1 - length / 2));
                tris.AddRange(new[] { b, b + 2, b + 1, b + 1, b + 2, b + 3 });  // up-facing
            }
            var mesh = new Mesh { name = assetName };
            mesh.SetVertices(verts);
            mesh.SetTriangles(tris, 0);
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            Directory.CreateDirectory(GenDir);
            string path = $"{GenDir}/{assetName}.asset";
            AssetDatabase.DeleteAsset(path);
            AssetDatabase.CreateAsset(mesh, path);
            var go = new GameObject(name);
            go.transform.position = centre;
            go.AddComponent<MeshFilter>().sharedMesh = mesh;
            go.AddComponent<MeshRenderer>().sharedMaterial = mat;
            go.AddComponent<MeshCollider>().sharedMesh = mesh;
        }

        internal static Material ColourMaterial(string name, Color c)
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

        internal static Shader LitShader() =>
            Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
    }
}
