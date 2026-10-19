// Menu: RcTwin > Build Campus Scene.
//
// Builds Assets/Scenes/Campus.unity from campus/campus_network.json, the
// campus fleet sim's road network at 1/10 scale (tools/campus_import.py,
// campus/README.md), so the car drives the area the RL environment
// (sim_py/campus.py) trains on:
//   - asphalt along every edge's centreline, the measured 1.409 m wide, with
//     a collider: it is what the Core feels through RaycastGround;
//   - the sidewalks, 6 cm kerbs either side, and the buildings, drawn
//     without colliders; the kerb lines and the buildings' footprints are
//     walls instead (ADR 0009), so the car stays on the asphalt. Sidewalk
//     runs and walls come from the data (tools/campus_import.py), the same
//     geometry the RL environment's walls use;
//   - the grass beyond, 1 mm below the asphalt: driveable here, but leaving
//     the asphalt ends an RL episode;
//   - each bus stop as a stop line across its node and a post at the kerb;
//   - the car at the hub stop, S3, facing along its first edge (R returns it).
// Unity coordinates: ISO (x, y, z) is Unity (-y, z, x).
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using Object = UnityEngine.Object;

namespace RcTwin.EditorTools
{
    public static class CampusSceneBuilder
    {
        public const string ScenePath = "Assets/Scenes/Campus.unity";
        public const string AsphaltName = "Campus asphalt";
        public const string HomeStop = "S3";
        private const string GenDir = "Assets/RcTwin/Generated/Campus";

        public static string NetworkPath =>
            Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "..", "campus", "campus_network.json"));

        private sealed class Edge
        {
            public int id, a, b;
            public List<Vector3> pts;   // Unity frame
        }

        [MenuItem("RcTwin/Build Campus Scene")]
        public static void Build()
        {
            if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo()) return;
            var net = (Dictionary<string, object>)MiniJson.Parse(File.ReadAllText(NetworkPath));
            var road = (Dictionary<string, object>)net["road"];
            float width = F(road["width"]), walk = F(road["sidewalk_width"]), kerb = F(road["kerb_height"]);

            var nodes = new Dictionary<int, Vector3>();
            foreach (Dictionary<string, object> n in (List<object>)net["nodes"])
                nodes[I(n["id"])] = TwinCar.ToUnity(D(n["x"]), D(n["y"]), D(n["z"]));
            var edges = new List<Edge>();
            var degree = new Dictionary<int, int>();
            foreach (Dictionary<string, object> e in (List<object>)net["edges"])
            {
                var pts = new List<Vector3>();
                foreach (List<object> q in (List<object>)e["polyline"])
                    pts.Add(TwinCar.ToUnity(D(q[0]), D(q[1]), D(q[2])));
                var edge = new Edge { id = I(e["id"]), a = I(e["a"]), b = I(e["b"]), pts = pts };
                edges.Add(edge);
                degree[edge.a] = degree.TryGetValue(edge.a, out int da) ? da + 1 : 1;
                degree[edge.b] = degree.TryGetValue(edge.b, out int db) ? db + 1 : 1;
            }

            var scene = EditorSceneManager.NewScene(NewSceneSetup.DefaultGameObjects, NewSceneMode.Single);
            Directory.CreateDirectory(GenDir);

            // Grass under the whole campus, 1 mm below the asphalt.
            Vector3 lo = Vector3.positiveInfinity, hi = Vector3.negativeInfinity;
            foreach (var p in nodes.Values) { lo = Vector3.Min(lo, p); hi = Vector3.Max(hi, p); }
            var grass = GameObject.CreatePrimitive(PrimitiveType.Plane);
            grass.name = "Grass (off the road: drivable here, ends an RL episode)";
            grass.transform.position = new Vector3(0.5f * (lo.x + hi.x), -0.001f, 0.5f * (lo.z + hi.z));
            grass.transform.localScale = new Vector3((hi.x - lo.x + 12f) / 10f, 1f, (hi.z - lo.z + 12f) / 10f);
            grass.GetComponent<Renderer>().sharedMaterial =
                FlatGroundSceneBuilder.ColourMaterial("Grass", new Color(0.36f, 0.5f, 0.3f));

            // Asphalt: one ribbon per edge, plus a disc wherever the road turns
            // at a node, to fill the outside of the corner.
            var asphalt = new MeshBuilder();
            foreach (var e in edges) asphalt.Band(e.pts, -0.5f * width, 0.5f * width, 0f);
            foreach (var kv in nodes)
            {
                if (TurnsAt(kv.Key, edges)) asphalt.Disc(kv.Value, 0.5f * width, 32);
            }
            MeshObject(AsphaltName, "CampusAsphalt", asphalt,
                                       FlatGroundSceneBuilder.ColourMaterial("Asphalt", new Color(0.23f, 0.23f, 0.25f)),
                                       collider: true);

            // Sidewalks: the data's runs (left out at junctions), a band either
            // side kerb_height up with the kerb face toward the road. Walls:
            // the data's kerb lines and building footprints, in ISO.
            var walks = new MeshBuilder();
            float inner = 0.5f * width, outer = 0.5f * width + walk, mid = 0.5f * (inner + outer);
            foreach (Dictionary<string, object> run in (List<object>)net["sidewalks"])
            {
                var pts = new List<Vector3>();
                foreach (List<object> q in (List<object>)run["points"])
                    pts.Add(TwinCar.ToUnity(D(q[0]), D(q[1]), D(q[2])));
                int side = I(run["side"]);
                if (side < 0) walks.Band(pts, -outer, -inner, kerb);
                else walks.Band(pts, inner, outer, kerb);
                walks.Kerb(pts, side * inner, kerb, side);
            }
            var walls = new GameObject("Walls (kerbs and buildings, ADR 0009)").AddComponent<TwinWalls>();
            foreach (Dictionary<string, object> w in (List<object>)net["walls"])
            {
                var pts = (List<object>)w["points"];
                var xy = new Vector2[pts.Count];
                for (int k = 0; k < pts.Count; k++)
                {
                    var q = (List<object>)pts[k];
                    xy[k] = new Vector2(F(q[0]), F(q[1]));
                }
                walls.Add((string)w["name"], (bool)w["closed"], xy);
            }
            MeshObject("Campus sidewalks (drawn only)", "CampusSidewalks", walks,
                       FlatGroundSceneBuilder.ColourMaterial("Sidewalk", new Color(0.62f, 0.6f, 0.56f)),
                       collider: false);

            // Buildings: footprint boxes, drawn only.
            var buildingParent = new GameObject("Buildings (drawn only)");
            var buildingMat = FlatGroundSceneBuilder.ColourMaterial("Building", new Color(0.74f, 0.7f, 0.64f));
            foreach (Dictionary<string, object> b in (List<object>)net["buildings"])
            {
                float h = F(b["height"]);
                var box = GameObject.CreatePrimitive(PrimitiveType.Cube);
                box.name = $"{b["name"]} ({b["prefab"]})";
                Object.DestroyImmediate(box.GetComponent<Collider>());
                box.transform.SetParent(buildingParent.transform, false);
                box.transform.SetPositionAndRotation(
                    TwinCar.ToUnity(D(b["x"]), D(b["y"]), 0.5 * h),
                    Quaternion.Euler(0f, TwinCar.YawToUnityDeg(D(b["yaw"])), 0f));
                box.transform.localScale = new Vector3(2f * F(b["half_width"]), h, 2f * F(b["half_length"]));
                box.GetComponent<Renderer>().sharedMaterial = buildingMat;
            }

            // Stops: a stop line across the road at the stop's node and a post
            // on the kerb on the stop's side, named with the demand weight.
            var stopParent = new GameObject("Bus stops");
            var lineMat = FlatGroundSceneBuilder.ColourMaterial("StopLine", new Color(0.95f, 0.8f, 0.1f));
            var postMat = FlatGroundSceneBuilder.ColourMaterial("StopPost", new Color(0.1f, 0.35f, 0.85f));
            var font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            Dictionary<string, object> home = null;
            foreach (Dictionary<string, object> s in (List<object>)net["stops"])
            {
                string name = (string)s["name"];
                if (name == HomeStop) home = s;
                int node = I(s["node"]);
                Vector3 at = nodes[node];
                Vector3 dir = Direction(node, edges);
                var stop = new GameObject($"Stop {name} (node {node}, weight {I(s["weight"])})");
                stop.transform.SetParent(stopParent.transform, false);
                stop.transform.SetPositionAndRotation(at, Quaternion.LookRotation(dir, Vector3.up));

                var line = GameObject.CreatePrimitive(PrimitiveType.Cube);
                line.name = "Stop line";
                Object.DestroyImmediate(line.GetComponent<Collider>());
                line.transform.SetParent(stop.transform, false);
                line.transform.localPosition = new Vector3(0f, 0.001f, 0f);
                line.transform.localScale = new Vector3(width, 0.001f, 0.05f);
                line.GetComponent<Renderer>().sharedMaterial = lineMat;

                Vector3 toStop = TwinCar.ToUnity(D(s["x"]), D(s["y"]), 0.0) - at;
                float sideSign = Vector3.Dot(toStop, Vector3.Cross(Vector3.up, dir)) >= 0f ? 1f : -1f;
                var post = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                post.name = "Post";
                Object.DestroyImmediate(post.GetComponent<Collider>());
                post.transform.SetParent(stop.transform, false);
                post.transform.localPosition = new Vector3(sideSign * mid, kerb + 0.2f, 0f);
                post.transform.localScale = new Vector3(0.04f, 0.2f, 0.04f);
                post.GetComponent<Renderer>().sharedMaterial = postMat;

                var label = new GameObject("Label");
                label.transform.SetParent(stop.transform, false);
                label.transform.localPosition = new Vector3(sideSign * mid, kerb + 0.55f, 0f);
                var text = label.AddComponent<TextMesh>();
                text.text = name;
                text.font = font;
                text.characterSize = 0.05f;
                text.fontSize = 48;
                text.anchor = TextAnchor.MiddleCenter;
                text.color = Color.white;
                label.GetComponent<MeshRenderer>().sharedMaterial = font.material;
            }

            // The car at the hub, facing along its node's first edge.
            var car = FlatGroundSceneBuilder.BuildCar(RcTwinNative.DefaultParams());
            if (home == null) throw new InvalidOperationException($"no stop {HomeStop} in {NetworkPath}");
            int homeNode = I(home["node"]);
            Vector3 hp = nodes[homeNode], hd = Direction(homeNode, edges);
            // Unity (ux, uy, uz) is ISO (uz, -ux, uy).
            car.homeX = hp.z;
            car.homeY = -hp.x;
            car.homeYaw = Math.Atan2(-hd.x, hd.z);
            car.transform.SetPositionAndRotation(
                hp + Vector3.up * (float)RcTwinNative.DefaultParams().hCog,
                Quaternion.Euler(0f, TwinCar.YawToUnityDeg(car.homeYaw), 0f));
            FlatGroundSceneBuilder.AttachChaseCamera(car.transform);
            var cam = Camera.main;
            if (cam != null) cam.transform.position = car.transform.position - hd * 1.8f + Vector3.up * 0.8f;

            Directory.CreateDirectory(Path.GetDirectoryName(ScenePath));
            EditorSceneManager.SaveScene(scene, ScenePath);
            FlatGroundSceneBuilder.RegisterScene(ScenePath);
            AssetDatabase.SaveAssets();
            Debug.Log($"Built {ScenePath} from {NetworkPath}: {edges.Count} edges, {nodes.Count} nodes. " +
                      $"The car starts at stop {HomeStop}; W/S throttle, A/D steer, R back to {HomeStop}.");
        }

        // Unit vectors to the right of the line in plan, mitred at corners so
        // a band keeps its width round them.
        private static Vector3[] MiterNormals(List<Vector3> pts)
        {
            int n = pts.Count;
            var seg = new Vector3[n - 1];
            for (int i = 0; i < n - 1; i++)
            {
                Vector3 d = pts[i + 1] - pts[i];
                seg[i] = new Vector3(d.z, 0f, -d.x).normalized;   // right of +z is +x
            }
            var o = new Vector3[n];
            o[0] = seg[0];
            o[n - 1] = seg[n - 2];
            for (int i = 1; i < n - 1; i++)
            {
                Vector3 m = (seg[i - 1] + seg[i]).normalized;
                float c = Vector3.Dot(m, seg[i]);
                o[i] = c > 0.2f ? m / c : seg[i];
            }
            return o;
        }

        // True where the road changes direction at the node, by more than a degree.
        private static bool TurnsAt(int node, List<Edge> edges)
        {
            var dirs = new List<Vector3>();
            foreach (var e in edges)
            {
                if (e.a == node) dirs.Add((e.pts[1] - e.pts[0]).normalized);
                if (e.b == node) dirs.Add((e.pts[e.pts.Count - 2] - e.pts[e.pts.Count - 1]).normalized);
            }
            if (dirs.Count != 2) return dirs.Count > 2;
            return Vector3.Dot(dirs[0], -dirs[1]) < Mathf.Cos(Mathf.Deg2Rad);
        }

        // Direction (Unity, in plan) of the lowest-numbered edge leaving the node.
        private static Vector3 Direction(int node, List<Edge> edges)
        {
            foreach (var e in edges)
            {
                if (e.a == node) return Flat(e.pts[1] - e.pts[0]);
                if (e.b == node) return Flat(e.pts[e.pts.Count - 2] - e.pts[e.pts.Count - 1]);
            }
            throw new InvalidOperationException($"node {node} has no edge");
        }

        private static Vector3 Flat(Vector3 v) => new Vector3(v.x, 0f, v.z).normalized;

        private static GameObject MeshObject(string name, string asset, MeshBuilder mb, Material mat, bool collider)
        {
            var mesh = mb.ToMesh(asset);
            string path = $"{GenDir}/{asset}.asset";
            AssetDatabase.DeleteAsset(path);
            AssetDatabase.CreateAsset(mesh, path);
            var go = new GameObject(name);
            go.AddComponent<MeshFilter>().sharedMesh = mesh;
            go.AddComponent<MeshRenderer>().sharedMaterial = mat;
            if (collider) go.AddComponent<MeshCollider>().sharedMesh = mesh;
            return go;
        }

        private sealed class MeshBuilder
        {
            private readonly List<Vector3> _v = new List<Vector3>();
            private readonly List<int> _t = new List<int>();

            // Triangle a b c, wound so its face looks along `up`.
            private void Tri(Vector3 a, Vector3 b, Vector3 c, Vector3 up)
            {
                if (Vector3.Dot(Vector3.Cross(b - a, c - a), up) < 0f) (b, c) = (c, b);
                int k = _v.Count;
                _v.Add(a); _v.Add(b); _v.Add(c);
                _t.Add(k); _t.Add(k + 1); _t.Add(k + 2);
            }

            // The strip between lateral offsets d0 < d1 (right positive), dy up
            // from the line.
            public void Band(List<Vector3> pts, float d0, float d1, float dy)
            {
                var n = MiterNormals(pts);
                var up = Vector3.up * dy;
                for (int i = 1; i < pts.Count; i++)
                {
                    Vector3 l0 = pts[i - 1] + n[i - 1] * d0 + up, r0 = pts[i - 1] + n[i - 1] * d1 + up;
                    Vector3 l1 = pts[i] + n[i] * d0 + up, r1 = pts[i] + n[i] * d1 + up;
                    Tri(l0, l1, r1, Vector3.up);
                    Tri(l0, r1, r0, Vector3.up);
                }
            }

            // The kerb face at lateral offset d, from the road up to h, facing
            // the road (side +1: a kerb on the right, facing left).
            public void Kerb(List<Vector3> pts, float d, float h, int side)
            {
                var n = MiterNormals(pts);
                for (int i = 1; i < pts.Count; i++)
                {
                    Vector3 b0 = pts[i - 1] + n[i - 1] * d, b1 = pts[i] + n[i] * d;
                    Vector3 t0 = b0 + Vector3.up * h, t1 = b1 + Vector3.up * h;
                    Vector3 face = -n[i] * side;
                    Tri(b0, b1, t1, face);
                    Tri(b0, t1, t0, face);
                }
            }

            public void Disc(Vector3 c, float r, int segments)
            {
                for (int i = 0; i < segments; i++)
                {
                    float a0 = 2f * Mathf.PI * i / segments, a1 = 2f * Mathf.PI * (i + 1) / segments;
                    Tri(c, c + new Vector3(Mathf.Cos(a0), 0f, Mathf.Sin(a0)) * r,
                        c + new Vector3(Mathf.Cos(a1), 0f, Mathf.Sin(a1)) * r, Vector3.up);
                }
            }

            public Mesh ToMesh(string name)
            {
                var m = new Mesh { name = name, indexFormat = UnityEngine.Rendering.IndexFormat.UInt32 };
                m.SetVertices(_v);
                m.SetTriangles(_t, 0);
                m.RecalculateNormals();
                m.RecalculateBounds();
                return m;
            }
        }

        private static double D(object o) => Convert.ToDouble(o, CultureInfo.InvariantCulture);
        private static float F(object o) => (float)D(o);
        private static int I(object o) => (int)Math.Round(D(o));

        // Just enough JSON for campus_network.json: objects, arrays, numbers,
        // strings, true, false, null.
        internal static class MiniJson
        {
            public static object Parse(string s)
            {
                int i = 0;
                object v = Value(s, ref i);
                Ws(s, ref i);
                if (i != s.Length) throw new FormatException($"trailing characters at {i}");
                return v;
            }

            private static void Ws(string s, ref int i)
            {
                while (i < s.Length && char.IsWhiteSpace(s[i])) i++;
            }

            private static object Value(string s, ref int i)
            {
                Ws(s, ref i);
                if (i >= s.Length) throw new FormatException("unexpected end");
                char c = s[i];
                if (c == '{')
                {
                    var o = new Dictionary<string, object>();
                    i++; Ws(s, ref i);
                    if (s[i] == '}') { i++; return o; }
                    while (true)
                    {
                        Ws(s, ref i);
                        string k = Str(s, ref i);
                        Ws(s, ref i);
                        Expect(s, ref i, ':');
                        o[k] = Value(s, ref i);
                        Ws(s, ref i);
                        if (s[i] == ',') { i++; continue; }
                        Expect(s, ref i, '}');
                        return o;
                    }
                }
                if (c == '[')
                {
                    var a = new List<object>();
                    i++; Ws(s, ref i);
                    if (s[i] == ']') { i++; return a; }
                    while (true)
                    {
                        a.Add(Value(s, ref i));
                        Ws(s, ref i);
                        if (s[i] == ',') { i++; continue; }
                        Expect(s, ref i, ']');
                        return a;
                    }
                }
                if (c == '"') return Str(s, ref i);
                if (string.CompareOrdinal(s, i, "true", 0, 4) == 0) { i += 4; return true; }
                if (string.CompareOrdinal(s, i, "false", 0, 5) == 0) { i += 5; return false; }
                if (string.CompareOrdinal(s, i, "null", 0, 4) == 0) { i += 4; return null; }
                int start = i;
                while (i < s.Length && "+-0123456789.eE".IndexOf(s[i]) >= 0) i++;
                if (i == start) throw new FormatException($"unexpected '{c}' at {i}");
                return double.Parse(s.Substring(start, i - start), NumberStyles.Float, CultureInfo.InvariantCulture);
            }

            private static void Expect(string s, ref int i, char c)
            {
                if (i >= s.Length || s[i] != c) throw new FormatException($"expected '{c}' at {i}");
                i++;
            }

            private static string Str(string s, ref int i)
            {
                Expect(s, ref i, '"');
                var sb = new StringBuilder();
                while (s[i] != '"')
                {
                    if (s[i] == '\\')
                    {
                        i++;
                        char e = s[i];
                        if (e == 'u') { sb.Append((char)Convert.ToInt32(s.Substring(i + 1, 4), 16)); i += 4; }
                        else sb.Append(e == 'n' ? '\n' : e == 't' ? '\t' : e == 'r' ? '\r' : e == 'b' ? '\b' : e == 'f' ? '\f' : e);
                    }
                    else sb.Append(s[i]);
                    i++;
                }
                i++;
                return sb.ToString();
            }
        }
    }
}
