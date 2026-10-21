// The real campus around the fleet sim's roads (tools/campus_scenery.py),
// drawn only: no colliders, nothing the car's physics or the RL environment
// feels. CampusSceneBuilder calls Build after laying the roads.
//   - buildings: campus/scenery.json (OpenStreetMap, ODbL), footprints fitted
//     round the roads and extruded to their heights; a real building standing
//     where a fleet sim building is hides that box (its walls stay);
//   - ground: Generated/Campus/Local/Ground.png laid over the grass (land
//     use, side roads and paths, and with the screenshots, grass, bare earth
//     and tree cover), the grass plane's own drawing switched off;
//   - trees: campus/reference/vegetation.json, from the screenshots.
// Ground and trees come from local files (map screenshots are third-party
// imagery, kept out of git); without them the campus keeps its plain grass.
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEditor;
using UnityEngine;
using Object = UnityEngine.Object;

namespace RcTwin.EditorTools
{
    public static class CampusScenery
    {
        private const string GenDir = "Assets/RcTwin/Generated/Campus";
        private const string LocalDir = GenDir + "/Local";
        public const string RootName = "Scenery (drawn only: OpenStreetMap, local imagery)";

        private static string Repo => Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", ".."));
        public static string SceneryPath => Path.Combine(Repo, "campus", "scenery.json");
        public static string VegetationPath => Path.Combine(Repo, "campus", "reference", "vegetation.json");

        private static readonly Dictionary<string, Color> Walls = new Dictionary<string, Color>
        {
            { "academic", new Color(0.86f, 0.80f, 0.70f) },
            { "hostel", new Color(0.90f, 0.88f, 0.82f) },
            { "other", new Color(0.78f, 0.74f, 0.68f) },
        };

        /// <summary>Adds the scenery to the open scene; the grass plane and
        /// the fleet sim's building boxes are passed so it can hide their
        /// drawing where it replaces them.</summary>
        public static void Build(GameObject grass, Transform fleetBuildings)
        {
            if (!File.Exists(SceneryPath))
            {
                Debug.Log($"No {SceneryPath}: the campus keeps its plain grass (tools/campus_scenery.py build).");
                return;
            }
            var doc = (Dictionary<string, object>)MiniJson.Parse(File.ReadAllText(SceneryPath));
            var root = new GameObject(RootName);
            int nb = Buildings(doc, root.transform, fleetBuildings);
            bool ground = Ground(root.transform, grass);
            int nt = Trees(root.transform);
            Debug.Log($"Campus scenery: {nb} buildings, {nt} trees, ground texture {(ground ? "on" : "off (no local Ground.png)")}. " +
                      (string)doc["attribution"]);
        }

        // ------------------------------------------------------- buildings

        private static int Buildings(Dictionary<string, object> doc, Transform root, Transform fleetBuildings)
        {
            var walls = new Dictionary<string, MeshParts>();
            var roof = new MeshParts();
            var hide = new HashSet<string>();
            int n = 0;
            foreach (Dictionary<string, object> b in (List<object>)doc["buildings"])
            {
                var fp = Points((List<object>)b["footprint"]);
                if (fp.Count < 3) continue;
                string kind = (string)b["kind"];
                if (!walls.TryGetValue(kind, out var mp)) walls[kind] = mp = new MeshParts();
                float h = F(b["height"]);
                mp.Prism(fp, h);
                roof.Cap(fp, h + 0.002f);
                foreach (var s in (List<object>)b["stands_for"]) hide.Add((string)s);
                n++;
            }
            var parent = new GameObject("Buildings").transform;
            parent.SetParent(root, false);
            foreach (var kv in walls)
                MeshObject($"Buildings, {kv.Key}", $"SceneryBuildings_{kv.Key}", kv.Value, parent,
                           FlatGroundSceneBuilder.ColourMaterial($"Scenery_{kv.Key}", Walls[kv.Key]), GenDir);
            MeshObject("Roofs", "SceneryRoofs", roof, parent,
                       FlatGroundSceneBuilder.ColourMaterial("Scenery_roof", new Color(0.62f, 0.60f, 0.58f)), GenDir);
            if (fleetBuildings != null)
                foreach (Transform box in fleetBuildings)
                    foreach (var name in hide)
                        if (box.name.StartsWith(name + " ", StringComparison.Ordinal) && box.TryGetComponent<MeshRenderer>(out var r))
                        {
                            r.enabled = false;   // the real building stands here; the box's walls stay in TwinWalls
                            box.name += " (drawn by the real building)";
                        }
            return n;
        }

        // ---------------------------------------------------------- ground

        private static bool Ground(Transform root, GameObject grass)
        {
            string png = $"{LocalDir}/Ground.png";
            string pngFile = Path.GetFullPath(Path.Combine(Application.dataPath, "..", png));
            string metaJson = Path.ChangeExtension(pngFile, ".json");
            if (!File.Exists(pngFile) || !File.Exists(metaJson)) return false;
            AssetDatabase.ImportAsset(png, ImportAssetOptions.ForceSynchronousImport);
            if (AssetImporter.GetAtPath(png) is TextureImporter ti)
            {
                ti.wrapMode = TextureWrapMode.Clamp;
                ti.maxTextureSize = 4096;
                ti.mipmapEnabled = true;
                ti.anisoLevel = 4;
                ti.SaveAndReimport();
            }
            var tex = AssetDatabase.LoadAssetAtPath<Texture2D>(png);
            var m = (Dictionary<string, object>)MiniJson.Parse(File.ReadAllText(metaJson));
            float x0 = F(m["x_min"]), x1 = F(m["x_max"]), y0 = F(m["y_min"]), y1 = F(m["y_max"]);

            // A quad over the texture's extent at the grass's height, the
            // texture's top-left at (x_max, y_max) seen from above.
            var mesh = new Mesh { name = "SceneryGround" };
            Vector3 P(float x, float y) => TwinCar.ToUnity(x, y, 0.0) + Vector3.up * -0.001f;
            mesh.vertices = new[] { P(x0, y1), P(x0, y0), P(x1, y0), P(x1, y1) };
            mesh.uv = new[] { new Vector2(0f, 0f), new Vector2(1f, 0f), new Vector2(1f, 1f), new Vector2(0f, 1f) };
            mesh.triangles = new[] { 0, 2, 1, 0, 3, 2 };
            if (Vector3.Dot(Vector3.Cross(mesh.vertices[2] - mesh.vertices[0], mesh.vertices[1] - mesh.vertices[0]), Vector3.up) < 0f)
                mesh.triangles = new[] { 0, 1, 2, 0, 2, 3 };
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            Save(mesh, $"{LocalDir}/SceneryGround.asset");
            var mat = Material($"{LocalDir}/Ground.mat");
            mat.mainTexture = tex;
            mat.color = Color.white;
            if (mat.HasProperty("_Smoothness")) mat.SetFloat("_Smoothness", 0.05f);
            var go = new GameObject("Ground (local imagery and OpenStreetMap)");
            go.transform.SetParent(root, false);
            go.AddComponent<MeshFilter>().sharedMesh = mesh;
            go.AddComponent<MeshRenderer>().sharedMaterial = mat;
            if (grass != null && grass.TryGetComponent<MeshRenderer>(out var gr))
            {
                // The grass keeps its collider (the car can still roll onto
                // it); a wider plane a little lower draws the land beyond.
                gr.enabled = false;
                var far = GameObject.CreatePrimitive(PrimitiveType.Plane);
                far.name = "Land beyond (drawn only)";
                Object.DestroyImmediate(far.GetComponent<Collider>());
                far.transform.SetParent(root, false);
                far.transform.position = grass.transform.position + Vector3.down * 0.02f;
                far.transform.localScale = new Vector3(40f, 1f, 40f);
                far.GetComponent<Renderer>().sharedMaterial =
                    FlatGroundSceneBuilder.ColourMaterial("Scenery_land", new Color(0.40f, 0.48f, 0.30f));
            }
            return true;
        }

        // ----------------------------------------------------------- trees

        private static int Trees(Transform root)
        {
            if (!File.Exists(VegetationPath)) return 0;
            var doc = (Dictionary<string, object>)MiniJson.Parse(File.ReadAllText(VegetationPath));
            var trees = (List<object>)doc["trees"];
            var trunks = new MeshParts();
            var crowns = new[] { new MeshParts(), new MeshParts(), new MeshParts() };
            var rng = new System.Random(11);
            foreach (List<object> t in trees)
            {
                float x = F(t[0]), y = F(t[1]), r = F(t[2]), h = F(t[3]);
                Vector3 at = TwinCar.ToUnity(x, y, 0.0);
                float trunkH = h - 1.4f * r;
                trunks.Cylinder(at, 0.12f * r + 0.012f, Mathf.Max(0.05f, trunkH), 6);
                crowns[rng.Next(3)].Blob(at + Vector3.up * (h - r), r, 0.85f + 0.3f * (float)rng.NextDouble(), rng);
            }
            var parent = new GameObject("Trees").transform;
            parent.SetParent(root, false);
            MeshObject("Trunks", "SceneryTrunks", trunks, parent,
                       Material($"{LocalDir}/Trunk.mat", new Color(0.36f, 0.28f, 0.20f)), LocalDir);
            var greens = new[] { new Color(0.22f, 0.36f, 0.18f), new Color(0.28f, 0.42f, 0.20f), new Color(0.19f, 0.31f, 0.17f) };
            for (int i = 0; i < 3; i++)
                MeshObject($"Crowns {i + 1}", $"SceneryCrowns{i + 1}", crowns[i], parent,
                           Material($"{LocalDir}/Crown{i + 1}.mat", greens[i]), LocalDir);
            return trees.Count;
        }

        // ------------------------------------------------------------ mesh

        private sealed class MeshParts
        {
            public readonly List<Vector3> V = new List<Vector3>();
            public readonly List<Vector3> N = new List<Vector3>();
            public readonly List<int> T = new List<int>();

            private void Quad(Vector3 a, Vector3 b, Vector3 c, Vector3 d, Vector3 n)
            {
                int k = V.Count;
                V.AddRange(new[] { a, b, c, d });
                N.AddRange(new[] { n, n, n, n });
                T.AddRange(new[] { k, k + 1, k + 2, k, k + 2, k + 3 });
            }

            // Walls of a footprint (ISO, counter-clockwise from above) from the
            // ground to height h, facing out.
            public void Prism(List<Vector2> fp, float h)
            {
                for (int i = 0; i < fp.Count; i++)
                {
                    Vector3 a = TwinCar.ToUnity(fp[i].x, fp[i].y, 0.0), b = TwinCar.ToUnity(fp[(i + 1) % fp.Count].x, fp[(i + 1) % fp.Count].y, 0.0);
                    Vector3 up = Vector3.up * h;
                    Vector3 n = Vector3.Cross(Vector3.up, b - a).normalized;   // Unity is left-handed: this points out of a CCW (ISO) ring
                    Quad(a, a + up, b + up, b, n);
                }
            }

            // The footprint's flat roof at height h, ear-clipped.
            public void Cap(List<Vector2> fp, float h)
            {
                int k = V.Count;
                foreach (var p in fp)
                {
                    V.Add(TwinCar.ToUnity(p.x, p.y, h));
                    N.Add(Vector3.up);
                }
                foreach (int i in EarClip(fp)) T.Add(k + i);
            }

            public void Cylinder(Vector3 at, float r, float h, int sides)
            {
                for (int i = 0; i < sides; i++)
                {
                    float a0 = 2f * Mathf.PI * i / sides, a1 = 2f * Mathf.PI * (i + 1) / sides;
                    Vector3 p0 = at + new Vector3(Mathf.Cos(a0), 0f, Mathf.Sin(a0)) * r;
                    Vector3 p1 = at + new Vector3(Mathf.Cos(a1), 0f, Mathf.Sin(a1)) * r;
                    Vector3 n = new Vector3(Mathf.Cos(0.5f * (a0 + a1)), 0f, Mathf.Sin(0.5f * (a0 + a1)));
                    Quad(p0, p0 + Vector3.up * h, p1 + Vector3.up * h, p1, n);
                }
            }

            private static readonly Vector3[] Ico = Icosahedron();

            // A lumpy low-poly crown: an icosahedron, each vertex pushed in or
            // out a little, squashed to `flat`.
            public void Blob(Vector3 c, float r, float flat, System.Random rng)
            {
                int k = V.Count;
                var verts = new Vector3[12];
                for (int i = 0; i < 12; i++)
                {
                    float s = r * (0.85f + 0.3f * (float)rng.NextDouble());
                    verts[i] = new Vector3(Ico[i].x * s, Ico[i].y * s * flat, Ico[i].z * s);
                }
                foreach (var f in IcoFaces)
                {
                    Vector3 a = verts[f[0]], b = verts[f[1]], d = verts[f[2]];
                    Vector3 n = Vector3.Cross(b - a, d - a).normalized;
                    if (Vector3.Dot(n, a + b + d) < 0f) { (b, d) = (d, b); n = -n; }
                    int j = V.Count;
                    V.AddRange(new[] { c + a, c + b, c + d });
                    N.AddRange(new[] { n, n, n });
                    T.AddRange(new[] { j, j + 1, j + 2 });
                }
            }

            private static readonly int[][] IcoFaces =
            {
                new[] { 0, 11, 5 }, new[] { 0, 5, 1 }, new[] { 0, 1, 7 }, new[] { 0, 7, 10 }, new[] { 0, 10, 11 },
                new[] { 1, 5, 9 }, new[] { 5, 11, 4 }, new[] { 11, 10, 2 }, new[] { 10, 7, 6 }, new[] { 7, 1, 8 },
                new[] { 3, 9, 4 }, new[] { 3, 4, 2 }, new[] { 3, 2, 6 }, new[] { 3, 6, 8 }, new[] { 3, 8, 9 },
                new[] { 4, 9, 5 }, new[] { 2, 4, 11 }, new[] { 6, 2, 10 }, new[] { 8, 6, 7 }, new[] { 9, 8, 1 },
            };

            private static Vector3[] Icosahedron()
            {
                float t = (1f + Mathf.Sqrt(5f)) / 2f;
                var v = new[]
                {
                    new Vector3(-1, t, 0), new Vector3(1, t, 0), new Vector3(-1, -t, 0), new Vector3(1, -t, 0),
                    new Vector3(0, -1, t), new Vector3(0, 1, t), new Vector3(0, -1, -t), new Vector3(0, 1, -t),
                    new Vector3(t, 0, -1), new Vector3(t, 0, 1), new Vector3(-t, 0, -1), new Vector3(-t, 0, 1),
                };
                for (int i = 0; i < v.Length; i++) v[i] = v[i].normalized;
                return v;
            }

            public Mesh ToMesh(string name)
            {
                var m = new Mesh { name = name, indexFormat = UnityEngine.Rendering.IndexFormat.UInt32 };
                m.SetVertices(V);
                m.SetNormals(N);
                m.SetTriangles(T, 0);
                m.RecalculateBounds();
                return m;
            }
        }

        /// <summary>Triangles of a simple polygon (counter-clockwise in ISO),
        /// as indices into it, wound to face up in Unity.</summary>
        public static List<int> EarClip(List<Vector2> poly)
        {
            var idx = new List<int>();
            for (int i = 0; i < poly.Count; i++) idx.Add(i);
            var tris = new List<int>();
            int guard = 0;
            while (idx.Count > 3 && guard++ < 10000)
            {
                bool clipped = false;
                for (int i = 0; i < idx.Count; i++)
                {
                    int ia = idx[(i + idx.Count - 1) % idx.Count], ib = idx[i], ic = idx[(i + 1) % idx.Count];
                    Vector2 a = poly[ia], b = poly[ib], c = poly[ic];
                    if (Cross(b - a, c - b) <= 1e-9f) continue;            // reflex: not an ear
                    bool inside = false;
                    foreach (int j in idx)
                    {
                        if (j == ia || j == ib || j == ic) continue;
                        if (InTriangle(poly[j], a, b, c)) { inside = true; break; }
                    }
                    if (inside) continue;
                    tris.AddRange(new[] { ia, ic, ib });                      // ISO CCW is clockwise seen from above in Unity
                    idx.RemoveAt(i);
                    clipped = true;
                    break;
                }
                if (!clipped) break;                                         // degenerate: fan the rest
            }
            for (int i = 1; i + 1 < idx.Count; i++) tris.AddRange(new[] { idx[0], idx[i + 1], idx[i] });
            return tris;
        }

        private static float Cross(Vector2 a, Vector2 b) => a.x * b.y - a.y * b.x;

        private static bool InTriangle(Vector2 p, Vector2 a, Vector2 b, Vector2 c)
        {
            float d1 = Cross(b - a, p - a), d2 = Cross(c - b, p - b), d3 = Cross(a - c, p - c);
            return d1 >= 0f && d2 >= 0f && d3 >= 0f;
        }

        // ---------------------------------------------------------- assets

        private static void MeshObject(string name, string asset, MeshParts parts, Transform parent, Material mat, string dir)
        {
            if (parts.V.Count == 0) return;
            var mesh = parts.ToMesh(asset);
            Save(mesh, $"{dir}/{asset}.asset");
            var go = new GameObject(name);
            go.transform.SetParent(parent, false);
            go.AddComponent<MeshFilter>().sharedMesh = mesh;
            go.AddComponent<MeshRenderer>().sharedMaterial = mat;
            GameObjectUtility.SetStaticEditorFlags(go, StaticEditorFlags.BatchingStatic);
        }

        private static void Save(Object asset, string path)
        {
            Directory.CreateDirectory(Path.GetDirectoryName(path));
            AssetDatabase.DeleteAsset(path);
            AssetDatabase.CreateAsset(asset, path);
        }

        private static Material Material(string path, Color? colour = null)
        {
            var m = AssetDatabase.LoadAssetAtPath<Material>(path);
            if (m == null)
            {
                Directory.CreateDirectory(Path.GetDirectoryName(path));
                m = new Material(FlatGroundSceneBuilder.LitShader());
                AssetDatabase.CreateAsset(m, path);
            }
            if (colour.HasValue) m.color = colour.Value;
            if (m.HasProperty("_Smoothness")) m.SetFloat("_Smoothness", 0.1f);
            EditorUtility.SetDirty(m);
            return m;
        }

        private static List<Vector2> Points(List<object> pts)
        {
            var o = new List<Vector2>();
            foreach (List<object> q in pts) o.Add(new Vector2(F(q[0]), F(q[1])));
            return o;
        }

        private static float F(object o) => (float)Convert.ToDouble(o, CultureInfo.InvariantCulture);
    }
}
