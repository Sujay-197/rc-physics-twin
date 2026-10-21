// TEMPORARY (rc-physics-twin): exports the campus geometry the physics twin needs,
// as plain numbers. Run in batch mode, then delete this file:
//   Unity -batchmode -quit -projectPath <campus> -executeMethod TwinCampusExporter.Export -exportPath <file>
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

public static class TwinCampusExporter
{
    static readonly CultureInfo Inv = CultureInfo.InvariantCulture;
    static string F(float v) => v.ToString("R", Inv);
    static string V(Vector3 v) => $"[{F(v.x)},{F(v.y)},{F(v.z)}]";

    public static void Export()
    {
        string path = "campus_export.json";
        var args = System.Environment.GetCommandLineArgs();
        for (int i = 0; i + 1 < args.Length; i++) if (args[i] == "-exportPath") path = args[i + 1];

        EditorSceneManager.OpenScene("Assets/Scenes/SampleScene.unity");
        var sb = new StringBuilder();
        sb.Append("{\n\"units\":\"unity metres, Unity frame (x right, y up, z forward)\",\n");

        var graph = Object.FindFirstObjectByType<BusSystem.RoadGraph>();
        sb.Append("\"nodes\":[");
        for (int i = 0; i < graph.Nodes.Count; i++)
            sb.Append(i > 0 ? "," : "").Append($"{{\"id\":{graph.Nodes[i].Id},\"p\":{V(graph.Nodes[i].Position)}}}");
        sb.Append("],\n\"edges\":[");
        for (int i = 0; i < graph.Edges.Count; i++)
        {
            var e = graph.Edges[i];
            sb.Append(i > 0 ? ",\n" : "").Append($"{{\"id\":{e.Id},\"a\":{e.NodeA},\"b\":{e.NodeB},\"length\":{F(e.Length)},\"polyline\":[");
            for (int k = 0; k < e.Polyline.Count; k++) sb.Append(k > 0 ? "," : "").Append(V(e.Polyline[k]));
            sb.Append("]}");
        }
        sb.Append("],\n");

        var stops = Object.FindObjectsByType<BusSystem.BusStop>(FindObjectsSortMode.None);
        sb.Append("\"stops\":[");
        for (int i = 0; i < stops.Length; i++)
            sb.Append(i > 0 ? ",\n" : "").Append($"{{\"name\":\"{stops[i].name}\",\"stop_id\":{stops[i].StopId},\"node\":{stops[i].NearestNodeIndex},\"weight\":{F(stops[i].Weight)},\"p\":{V(stops[i].transform.position)}}}");
        sb.Append("],\n");

        sb.Append("\"road_tiles\":[");
        var roads = GameObject.Find("Roads");
        bool first = true;
        if (roads != null)
            foreach (Transform t in roads.transform)
            {
                var src = PrefabUtility.GetCorrespondingObjectFromSource(t.gameObject);
                sb.Append(first ? "" : ",\n");
                first = false;
                sb.Append($"{{\"name\":\"{t.name}\",\"prefab\":\"{(src ? src.name : "")}\",\"p\":{V(t.position)},\"euler\":{V(t.eulerAngles)},\"scale\":{V(t.lossyScale)},\"parts\":[");
                bool pf = true;
                foreach (var mf in t.GetComponentsInChildren<MeshFilter>())
                {
                    var mr = mf.GetComponent<MeshRenderer>();
                    var mesh = mf.sharedMesh;
                    if (mr == null || mesh == null) continue;
                    for (int sm = 0; sm < mesh.subMeshCount && sm < mr.sharedMaterials.Length; sm++)
                    {
                        var idx = mesh.GetIndices(sm);
                        var verts = mesh.vertices;
                        // Submesh bounds in the TILE's local frame (scaled), so widths read directly.
                        Vector3 lo = Vector3.positiveInfinity, hi = Vector3.negativeInfinity;
                        foreach (int vi in idx)
                        {
                            var w = mf.transform.TransformPoint(verts[vi]);
                            var l = t.InverseTransformPoint(w);
                            l = Vector3.Scale(l, t.lossyScale);
                            lo = Vector3.Min(lo, l); hi = Vector3.Max(hi, l);
                        }
                        var mat = mr.sharedMaterials[sm] ? mr.sharedMaterials[sm].name : "";
                        sb.Append(pf ? "" : ",");
                        pf = false;
                        sb.Append($"{{\"material\":\"{mat}\",\"lo\":{V(lo)},\"hi\":{V(hi)}}}");
                    }
                }
                sb.Append("]}");
            }
        sb.Append("],\n");

        sb.Append("\"buildings\":[");
        var bld = GameObject.Find("Buildings");
        first = true;
        if (bld != null)
            foreach (Transform t in bld.transform)
            {
                // Oriented footprint: every renderer's local-bounds corners, in the building's frame.
                Vector3 lo = Vector3.positiveInfinity, hi = Vector3.negativeInfinity;
                foreach (var r in t.GetComponentsInChildren<Renderer>())
                {
                    var b = r.localBounds;
                    for (int c = 0; c < 8; c++)
                    {
                        var corner = b.center + Vector3.Scale(b.extents,
                            new Vector3((c & 1) == 0 ? -1 : 1, (c & 2) == 0 ? -1 : 1, (c & 4) == 0 ? -1 : 1));
                        var w = r.transform.TransformPoint(corner);
                        var l = Quaternion.Inverse(t.rotation) * (w - t.position);
                        lo = Vector3.Min(lo, l); hi = Vector3.Max(hi, l);
                    }
                }
                if (float.IsInfinity(lo.x)) continue;
                var src = PrefabUtility.GetCorrespondingObjectFromSource(t.gameObject);
                sb.Append(first ? "" : ",\n");
                first = false;
                sb.Append($"{{\"name\":\"{t.name}\",\"prefab\":\"{(src ? src.name : "")}\",\"p\":{V(t.position)},\"yaw_deg\":{F(t.eulerAngles.y)},\"lo\":{V(lo)},\"hi\":{V(hi)}}}");
            }
        sb.Append("]\n}\n");
        File.WriteAllText(path, sb.ToString());
        Debug.Log($"TwinCampusExporter: wrote {path}");
    }
}
