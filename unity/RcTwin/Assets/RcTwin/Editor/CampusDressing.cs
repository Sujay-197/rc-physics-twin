// The campus dressed (CampusLook): trees, hedges, street lamps, rocks and
// benches along the roads and at the stops, from tools/prepare_assets.py's
// Poly Haven models. Drawn only: every placement keeps clear of the asphalt
// (lamps and the stops' benches stand on the sidewalks, everything else on
// the grass beyond), of the buildings, side roads and water, and nothing has
// a collider, so the car's footprint, its walls and the autopilot are as
// before. The plan is deterministic and needs only the committed campus
// data (campus/campus_network.json, campus/scenery.json); the models it
// places come from the gitignored Generated/Look/.
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;
using Object = UnityEngine.Object;

namespace RcTwin.EditorTools
{
    /// <summary>The campus in plan (Unity x, z): what the dressing keeps clear of.</summary>
    public sealed class CampusLayout
    {
        public float HalfWidth, Walk, Kerb;
        public readonly List<Vector2[]> Roads = new List<Vector2[]>();
        public readonly List<(Vector2[] pts, float halfWidth)> SideRoads = new List<(Vector2[], float)>();
        public readonly List<Vector2[]> Buildings = new List<Vector2[]>();
        public readonly List<Vector2[]> NoPlanting = new List<Vector2[]>();   // water, pitches
        /// <summary>Junctions and corners: where sidewalks stop and the car turns.</summary>
        public readonly List<Vector2> Corners = new List<Vector2>();
        /// <summary>Each stop: its node, the road's direction there, the shelter's side (+1 right).</summary>
        public readonly List<(string name, Vector2 node, Vector2 dir, float side)> Stops = new List<(string, Vector2, Vector2, float)>();

        public static Vector2 Plan(Vector3 unity) => new Vector2(unity.x, unity.z);

        /// <summary>Where CampusSceneBuilder puts a stop's shelter: mid-sidewalk, 0.3 m back from the node.</summary>
        public Vector2 Shelter((string name, Vector2 node, Vector2 dir, float side) s) =>
            s.node + new Vector2(s.dir.y, -s.dir.x) * s.side * (HalfWidth + 0.5f * Walk) - s.dir * 0.3f;
        private static Vector2 Iso(object x, object y) => Plan(TwinCar.ToUnity(D(x), D(y), 0.0));

        public static CampusLayout Load(string networkPath, string sceneryPath)
        {
            var net = (Dictionary<string, object>)MiniJson.Parse(File.ReadAllText(networkPath));
            var road = (Dictionary<string, object>)net["road"];
            var L = new CampusLayout
            {
                HalfWidth = 0.5f * F(road["width"]), Walk = F(road["sidewalk_width"]), Kerb = F(road["kerb_height"]),
            };
            var nodes = new Dictionary<int, Vector2>();
            foreach (Dictionary<string, object> n in (List<object>)net["nodes"])
                nodes[I(n["id"])] = Iso(n["x"], n["y"]);
            var ends = new Dictionary<int, List<Vector2>>();   // node -> directions of the edges leaving it
            var firstDir = new Dictionary<int, Vector2>();
            foreach (Dictionary<string, object> e in (List<object>)net["edges"])
            {
                var pts = new List<Vector2>();
                foreach (List<object> q in (List<object>)e["polyline"]) pts.Add(Iso(q[0], q[1]));
                L.Roads.Add(pts.ToArray());
                int a = I(e["a"]), b = I(e["b"]);
                Vector2 da = (pts[1] - pts[0]).normalized, db = (pts[pts.Count - 2] - pts[pts.Count - 1]).normalized;
                if (!ends.ContainsKey(a)) ends[a] = new List<Vector2>();
                if (!ends.ContainsKey(b)) ends[b] = new List<Vector2>();
                ends[a].Add(da);
                ends[b].Add(db);
                if (!firstDir.ContainsKey(a)) firstDir[a] = da;   // as CampusSceneBuilder.Direction: the first edge in the file
                if (!firstDir.ContainsKey(b)) firstDir[b] = db;
            }
            foreach (var kv in ends)
                if (kv.Value.Count > 2 || (kv.Value.Count == 2 && Vector2.Dot(kv.Value[0], -kv.Value[1]) < Mathf.Cos(Mathf.Deg2Rad)))
                    L.Corners.Add(nodes[kv.Key]);
            foreach (Dictionary<string, object> b in (List<object>)net["buildings"])
            {
                Vector2 c = Iso(b["x"], b["y"]);
                float yaw = TwinCar.YawToUnityDeg(D(b["yaw"])) * Mathf.Deg2Rad;
                float hw = F(b["half_width"]), hl = F(b["half_length"]);
                // Unity yaw turns +z (length) toward +x (width).
                Vector2 ax = new Vector2(Mathf.Cos(yaw), -Mathf.Sin(yaw)) * hw, az = new Vector2(Mathf.Sin(yaw), Mathf.Cos(yaw)) * hl;
                L.Buildings.Add(new[] { c - ax - az, c + ax - az, c + ax + az, c - ax + az });
            }
            foreach (Dictionary<string, object> s in (List<object>)net["stops"])
            {
                int node = I(s["node"]);
                Vector2 at = nodes[node], dir = firstDir[node];
                Vector2 toStop = Iso(s["x"], s["y"]) - at;
                Vector2 right = new Vector2(dir.y, -dir.x);   // right of +z is +x
                L.Stops.Add(((string)s["name"], at, dir, Vector2.Dot(toStop, right) >= 0f ? 1f : -1f));
            }
            if (sceneryPath != null && File.Exists(sceneryPath))
            {
                var sc = (Dictionary<string, object>)MiniJson.Parse(File.ReadAllText(sceneryPath));
                foreach (Dictionary<string, object> b in (List<object>)sc["buildings"])
                    L.Buildings.Add(Points((List<object>)b["footprint"]));
                foreach (Dictionary<string, object> r in (List<object>)sc["side_roads"])
                    L.SideRoads.Add((Points((List<object>)r["points"]), 0.5f * F(r["width"])));
                foreach (Dictionary<string, object> l in (List<object>)sc["land"])
                {
                    string kind = (string)l["kind"];
                    if (kind == "water" || kind == "pitch") L.NoPlanting.Add(Points((List<object>)l["polygon"]));
                }
            }
            return L;
        }

        /// <summary>Distance in plan from p to the nearest road centreline.</summary>
        public float RoadDistance(Vector2 p)
        {
            float best = float.MaxValue;
            foreach (var r in Roads) best = Mathf.Min(best, PolylineDistance(p, r));
            return best;
        }

        /// <summary>How far p is outside the nearest side road's edge (negative: on it).</summary>
        public float SideRoadClearance(Vector2 p)
        {
            float best = float.MaxValue;
            foreach (var (pts, hw) in SideRoads) best = Mathf.Min(best, PolylineDistance(p, pts) - hw);
            return best;
        }

        /// <summary>How far p is outside the nearest building (negative: inside one).</summary>
        public float BuildingClearance(Vector2 p) => PolygonClearance(p, Buildings);

        public bool Plantable(Vector2 p, float r) => PolygonClearance(p, NoPlanting) >= r;

        public float CornerDistance(Vector2 p)
        {
            float best = float.MaxValue;
            foreach (var c in Corners) best = Mathf.Min(best, Vector2.Distance(p, c));
            return best;
        }

        private static float PolygonClearance(Vector2 p, List<Vector2[]> polys)
        {
            float best = float.MaxValue;
            foreach (var poly in polys)
            {
                float d = float.MaxValue;
                bool inside = false;
                for (int i = 0, j = poly.Length - 1; i < poly.Length; j = i++)
                {
                    d = Mathf.Min(d, SegmentDistance(p, poly[j], poly[i]));
                    if ((poly[i].y > p.y) != (poly[j].y > p.y) &&
                        p.x < (poly[j].x - poly[i].x) * (p.y - poly[i].y) / (poly[j].y - poly[i].y) + poly[i].x)
                        inside = !inside;
                }
                best = Mathf.Min(best, inside ? -d : d);
            }
            return best;
        }

        public static float PolylineDistance(Vector2 p, Vector2[] pts)
        {
            float best = float.MaxValue;
            for (int i = 1; i < pts.Length; i++) best = Mathf.Min(best, SegmentDistance(p, pts[i - 1], pts[i]));
            return best;
        }

        private static float SegmentDistance(Vector2 p, Vector2 a, Vector2 b)
        {
            Vector2 ab = b - a;
            float t = ab.sqrMagnitude < 1e-12f ? 0f : Mathf.Clamp01(Vector2.Dot(p - a, ab) / ab.sqrMagnitude);
            return Vector2.Distance(p, a + t * ab);
        }

        private static Vector2[] Points(List<object> pts)
        {
            var o = new Vector2[pts.Count];
            for (int i = 0; i < pts.Count; i++)
            {
                var q = (List<object>)pts[i];
                o[i] = Iso(q[0], q[1]);
            }
            return o;
        }

        private static double D(object o) => Convert.ToDouble(o, CultureInfo.InvariantCulture);
        private static float F(object o) => (float)D(o);
        private static int I(object o) => (int)Math.Round(D(o));
    }

    public static class CampusDressing
    {
        public const string RootName = "Dressing (drawn only: Poly Haven CC0, no colliders)";

        public enum Kind { Tree, Shrub, Plant, Rock, Lamp, Bench }

        /// <summary>A prop as tools/prepare_assets.py makes it, at scale 1 [m]: the
        /// radius of its base (what must stay off the road), of its crown, and the
        /// room it wants from its neighbours at the ground (trees also keep their
        /// crowns apart). A test checks look.json against these.</summary>
        public sealed class Prop
        {
            public string Name;
            public Kind Kind;
            public float Base, Crown, Room, Height;
            public float HalfLength;   // benches: along the seat
        }

        public static readonly Prop[] Props =
        {
            new Prop { Name = "tree_small_02", Kind = Kind.Tree, Base = 0.03f, Crown = 0.30f, Room = 0.06f, Height = 0.454f },
            new Prop { Name = "jacaranda_tree", Kind = Kind.Tree, Base = 0.04f, Crown = 0.66f, Room = 0.08f, Height = 0.968f },
            new Prop { Name = "shrub_02_a", Kind = Kind.Shrub, Base = 0.08f, Crown = 0.08f, Room = 0.06f, Height = 0.136f },
            new Prop { Name = "shrub_02_b", Kind = Kind.Shrub, Base = 0.07f, Crown = 0.07f, Room = 0.055f, Height = 0.122f },
            new Prop { Name = "shrub_02_c", Kind = Kind.Shrub, Base = 0.105f, Crown = 0.105f, Room = 0.08f, Height = 0.105f },
            new Prop { Name = "shrub_02_d", Kind = Kind.Shrub, Base = 0.065f, Crown = 0.065f, Room = 0.05f, Height = 0.098f },
            new Prop { Name = "shrub_03_a", Kind = Kind.Plant, Base = 0.012f, Crown = 0.012f, Room = 0.012f, Height = 0.040f },
            new Prop { Name = "shrub_03_b", Kind = Kind.Plant, Base = 0.012f, Crown = 0.012f, Room = 0.012f, Height = 0.035f },
            new Prop { Name = "shrub_03_c", Kind = Kind.Plant, Base = 0.01f, Crown = 0.01f, Room = 0.01f, Height = 0.030f },
            new Prop { Name = "boulder_01", Kind = Kind.Rock, Base = 0.1f, Crown = 0.1f, Room = 0.09f, Height = 0.093f },
            new Prop { Name = "street_lamp_01", Kind = Kind.Lamp, Base = 0.036f, Crown = 0.036f, Room = 0.05f, Height = 0.387f },
            new Prop { Name = "bench", Kind = Kind.Bench, Base = 0.04f, Crown = 0.04f, Room = 0.13f, Height = 0.087f, HalfLength = 0.125f },
        };

        public static Prop Find(string name) => Array.Find(Props, p => p.Name == name);

        /// <summary>One prop placed: where (plan), its yaw (Unity, degrees), its scale,
        /// whether it stands on a sidewalk (at kerb height), and the circles its base
        /// covers in plan (a bench: its middle and both ends).</summary>
        public struct Placement
        {
            public string Prop;
            public Kind Kind;
            public Vector2 At;
            public float Yaw, Scale;
            public bool OnSidewalk;
            public (Vector2 c, float r)[] Footprint;
        }

        // Keep-outs [m, twin scale].
        public const float RoadMargin = 0.03f;      // past the asphalt's edge, everything
        public const float StopClear = 0.45f;       // round a stop's node: students walk there
        public const float ShelterClear = 0.2f;     // round a stop's shelter (and the charger beside it)
        public const float CornerClear = 0.35f;     // sidewalk props: past the asphalt round a corner

        public static List<Placement> Plan(CampusLayout L, int seed = 20261022)
        {
            var rng = new System.Random(seed);
            string[] shrubs = { "shrub_02_a", "shrub_02_b", "shrub_02_c", "shrub_02_d" };
            string[] plants = { "shrub_03_a", "shrub_03_b", "shrub_03_c" };
            float R() => (float)rng.NextDouble();
            var placed = new List<Placement>();
            var room = new List<(Vector2 c, float r)>();
            var crowns = new List<(Vector2 c, float r)>();
            float beyond = L.HalfWidth + L.Walk;   // the sidewalk's outer edge

            bool Fits(Placement p, Prop prop)
            {
                foreach (var (c, r) in p.Footprint)
                {
                    float d = L.RoadDistance(c);
                    if (d < L.HalfWidth + r + RoadMargin) return false;
                    if (!p.OnSidewalk && d < beyond + r) return false;
                    if (L.BuildingClearance(c) < r + 0.02f) return false;
                    if (L.SideRoadClearance(c) < r) return false;
                    if (!L.Plantable(c, r)) return false;
                    foreach (var s in L.Stops)
                        if (Vector2.Distance(c, s.node) < StopClear + r || Vector2.Distance(c, L.Shelter(s)) < ShelterClear + r)
                            return false;
                }
                if (prop.Kind == Kind.Tree)
                {
                    // The crown may reach over the sidewalk but not far over the road,
                    // into a wall, or far into the next crown.
                    float crown = prop.Crown * p.Scale;
                    if (L.RoadDistance(p.At) < L.HalfWidth + 0.7f * crown) return false;
                    if (L.BuildingClearance(p.At) < 0.45f * crown) return false;
                    foreach (var (c, r) in crowns)
                        if (Vector2.Distance(c, p.At) < 0.55f * (crown + r)) return false;
                }
                if (p.OnSidewalk && L.CornerDistance(p.At) < beyond + CornerClear) return false;
                float mine = prop.Room * p.Scale;
                foreach (var (c, r) in room)
                    if (Vector2.Distance(c, p.At) < mine + r) return false;
                return true;
            }

            void Try(string name, Vector2 at, float yaw, float scale, bool onSidewalk, Vector2 along = default)
            {
                var prop = Find(name);
                var foot = prop.Kind == Kind.Bench
                    ? new[] { (at, prop.Base * scale), (at + along * prop.HalfLength * scale, prop.Base * scale),
                              (at - along * prop.HalfLength * scale, prop.Base * scale) }
                    : new[] { (at, prop.Base * scale) };
                var p = new Placement { Prop = name, Kind = prop.Kind, At = at, Yaw = yaw, Scale = scale, OnSidewalk = onSidewalk, Footprint = foot };
                if (!Fits(p, prop)) return;
                placed.Add(p);
                room.Add((at, prop.Room * scale));
                if (prop.Kind == Kind.Tree) crowns.Add((at, prop.Crown * scale));
            }

            void Tree(Vector2 at, Vector2 dir, Vector2 right, bool roadside)
            {
                bool big = R() < (roadside ? 0.3f : 0.4f);
                string name = big ? "jacaranda_tree" : "tree_small_02";
                float scale = big ? 0.75f + 0.3f * R() : 0.85f + 0.4f * R();
                Try(name, at, 360f * R(), scale, false);
                if (R() < 0.2f)
                {
                    int n = 1 + rng.Next(3);
                    for (int k = 0; k < n; k++)
                        Try("boulder_01", at + right * (0.12f + 0.15f * R()) + dir * (0.25f * (R() - 0.5f)), 360f * R(), 0.35f + 0.6f * R(), false);
                    for (int k = 0; k < 2 + rng.Next(3); k++)
                        Try(plants[rng.Next(plants.Length)], at + new Vector2(R() - 0.5f, R() - 0.5f) * 0.3f, 360f * R(), 0.9f + 0.5f * R(), false);
                }
            }

            // The stops first: a bench on the sidewalk beside each shelter, facing the road.
            foreach (var s in L.Stops)
            {
                Vector2 right = new Vector2(s.dir.y, -s.dir.x) * s.side;
                Vector2 at = s.node + right * (L.HalfWidth + 0.5f * L.Walk) - s.dir * 0.72f;
                Try("bench", at, Yaw(-right), 1f, onSidewalk: true, along: s.dir);
            }

            foreach (var road in L.Roads)
            {
                var stations = Stations(road);
                float length = stations.Count > 0 ? stations[stations.Count - 1].s : 0f;
                foreach (float side in new[] { 1f, -1f })
                {
                    // Lamps on the sidewalk, staggered either side.
                    for (float s = side > 0 ? 0.7f : 1.9f; s < length; s += 2.4f)
                    {
                        var (p, dir) = At(stations, s);
                        Vector2 right = new Vector2(dir.y, -dir.x) * side;
                        Try("street_lamp_01", p + right * (L.HalfWidth + 0.72f * L.Walk), Yaw(dir), 1f, onSidewalk: true);
                    }
                    // Trees behind the sidewalk, a second row further back now and then.
                    for (float s = 0.5f * R(); s < length; s += 0.8f + 0.7f * R())
                    {
                        if (R() > 0.85f) continue;
                        var (p, dir) = At(stations, s);
                        Vector2 right = new Vector2(dir.y, -dir.x) * side;
                        Tree(p + right * (beyond + 0.14f + 0.45f * R()), dir, right, true);
                        if (R() < 0.5f) Tree(p + right * (beyond + 0.9f + 0.8f * R()) + dir * (0.4f * R()), dir, right, true);
                    }
                    // Hedges: runs along the sidewalk's back edge, round the trunks.
                    for (float s = 0.3f; s < length;)
                    {
                        if (R() < 0.35f)
                        {
                            float end = Mathf.Min(length, s + 1.2f + 1.8f * R());
                            for (; s < end; s += 0.1f + 0.03f * R())
                            {
                                var (p, dir) = At(stations, s);
                                Vector2 right = new Vector2(dir.y, -dir.x) * side;
                                Try(shrubs[rng.Next(shrubs.Length)], p + right * (beyond + 0.085f), 360f * R(), 0.85f + 0.3f * R(), false);
                            }
                        }
                        s += 1.5f + 2.5f * R();
                    }
                    // Now and then a bench on the grass behind the sidewalk, facing the road.
                    for (float s = 2.5f + 3f * R(); s < length; s += 5f + 4f * R())
                    {
                        var (p, dir) = At(stations, s);
                        Vector2 right = new Vector2(dir.y, -dir.x) * side;
                        Try("bench", p + right * (beyond + 0.08f), Yaw(-right), 1f, false, along: dir);
                    }
                }
            }
            // Groves in the open ground between and beyond the roads: depth for the views.
            Vector2 lo = new Vector2(float.MaxValue, float.MaxValue), hi = -lo;
            foreach (var r in L.Roads)
                foreach (var q in r) { lo = Vector2.Min(lo, q); hi = Vector2.Max(hi, q); }
            lo -= Vector2.one * 6f;
            hi += Vector2.one * 6f;
            for (int i = 0; i < 2500; i++)
            {
                var at = new Vector2(Mathf.Lerp(lo.x, hi.x, R()), Mathf.Lerp(lo.y, hi.y, R()));
                float d = L.RoadDistance(at);
                if (d < beyond + 1.2f || d > 8f || R() > 1.2f - d / 8f) continue;
                Tree(at, Vector2.up, Vector2.right, false);
            }
            return placed;
        }

        // Unity yaw (degrees) that turns +z to face along d.
        private static float Yaw(Vector2 d) => Mathf.Atan2(d.x, d.y) * Mathf.Rad2Deg;

        private static List<(float s, Vector2 p)> Stations(Vector2[] pts)
        {
            var o = new List<(float, Vector2)>();
            float s = 0f;
            for (int i = 0; i < pts.Length; i++)
            {
                if (i > 0) s += Vector2.Distance(pts[i - 1], pts[i]);
                o.Add((s, pts[i]));
            }
            return o;
        }

        private static (Vector2 p, Vector2 dir) At(List<(float s, Vector2 p)> st, float s)
        {
            for (int i = 1; i < st.Count; i++)
                if (st[i].s >= s || i == st.Count - 1)
                {
                    float span = Mathf.Max(1e-6f, st[i].s - st[i - 1].s);
                    float t = Mathf.Clamp01((s - st[i - 1].s) / span);
                    return (Vector2.Lerp(st[i - 1].p, st[i].p, t), (st[i].p - st[i - 1].p).normalized);
                }
            return (st[0].p, Vector2.up);
        }

        // ------------------------------------------------------------ building

        /// <summary>Instantiates the plan's models under a new root; returns it, or
        /// null without the prepared models. Each lamp gets a warm light.</summary>
        public static GameObject Build(List<Placement> plan, CampusLayout L)
        {
            var models = new Dictionary<string, GameObject>();
            foreach (var prop in Props)
            {
                var m = CampusLook.Model(prop.Name);
                if (m != null) models[prop.Name] = m;
            }
            if (models.Count == 0) return null;
            var root = new GameObject(RootName);
            var groups = new Dictionary<Kind, Transform>();
            foreach (Kind k in Enum.GetValues(typeof(Kind)))
            {
                groups[k] = new GameObject($"{k}s").transform;
                groups[k].SetParent(root.transform, false);
            }
            int n = 0;
            foreach (var p in plan)
            {
                if (!models.TryGetValue(p.Prop, out var model)) continue;
                var go = (GameObject)PrefabUtility.InstantiatePrefab(model, groups[p.Kind]);
                go.transform.SetPositionAndRotation(new Vector3(p.At.x, p.OnSidewalk ? L.Kerb : 0f, p.At.y),
                                                    Quaternion.Euler(0f, p.Yaw, 0f));
                go.transform.localScale = Vector3.one * p.Scale;
                foreach (var c in go.GetComponentsInChildren<Collider>(true)) Object.DestroyImmediate(c);
                bool smallThing = p.Kind == Kind.Plant || (p.Kind == Kind.Rock && p.Scale < 0.5f);
                foreach (var r in go.GetComponentsInChildren<Renderer>(true))
                {
                    r.shadowCastingMode = smallThing ? ShadowCastingMode.Off : ShadowCastingMode.On;
                    r.lightProbeUsage = LightProbeUsage.Off;
                }
                if (go.TryGetComponent<LODGroup>(out var lod)) Transitions(lod, p.Kind);
                if (p.Kind == Kind.Lamp) LampLight(go.transform, Find(p.Prop).Height);
                n++;
            }
            Debug.Log($"Campus dressing: {n} props ({Count(plan, Kind.Tree)} trees, {Count(plan, Kind.Shrub)} shrubs, " +
                      $"{Count(plan, Kind.Lamp)} lamps, {Count(plan, Kind.Bench)} benches, {Count(plan, Kind.Rock)} rocks, {Count(plan, Kind.Plant)} plants).");
            return root;
        }

        private static int Count(List<Placement> plan, Kind k) => plan.FindAll(p => p.Kind == k).Count;

        // Screen heights where each LOD hands over, and where the prop is culled.
        private static void Transitions(LODGroup group, Kind kind)
        {
            float[] at = kind switch
            {
                Kind.Tree => new[] { 0.32f, 0.11f, 0.008f },
                Kind.Lamp => new[] { 0.2f, 0.06f, 0.006f },
                Kind.Plant => new[] { 0.08f, 0.03f, 0.01f },
                _ => new[] { 0.15f, 0.05f, 0.006f },
            };
            var lods = group.GetLODs();
            for (int i = 0; i < lods.Length && i < at.Length; i++) lods[i].screenRelativeTransitionHeight = at[i];
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
