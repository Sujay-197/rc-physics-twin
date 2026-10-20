// Menu: RcTwin > Build Replay Scene, RcTwin > Build Replay Player.
//
// The replay scene is the Campus scene (RcTwin > Build Campus Scene) with the
// driveable car and its chase camera taken out and a PolicyReplay put in: the
// campus's own meshes and materials, unchanged, so Campus.unity is untouched.
// PolicyReplay plays tools/record_drives.py's drives.json (see there).
//
// The player is that one scene as a Windows build at build/replay/
// RcTwinReplay.exe, with results/drives.json copied beside it when it exists.
// Record again and copy (or start the player with -drives <path>) to refresh.
//
// Batch: Unity -batchmode -quit -projectPath unity/RcTwin
//              -executeMethod RcTwin.EditorTools.ReplaySceneBuilder.BuildPlayerBatch
using System;
using System.IO;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEditor.SceneManagement;
using UnityEngine;
using Object = UnityEngine.Object;

namespace RcTwin.EditorTools
{
    public static class ReplaySceneBuilder
    {
        public const string ScenePath = "Assets/Scenes/Replay.unity";
        private const string GenDir = "Assets/RcTwin/Generated";

        private static string Repo => Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", ".."));
        public static string PlayerPath => Path.Combine(Repo, "build", "replay", "RcTwinReplay.exe");
        public static string DrivesPath => Path.Combine(Repo, "results", "drives.json");

        [MenuItem("RcTwin/Build Replay Scene")]
        public static void Build()
        {
            if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo()) return;
            BuildScene();
        }

        private static void BuildScene()
        {
            if (!File.Exists(CampusSceneBuilder.ScenePath)) CampusSceneBuilder.Build();
            var scene = EditorSceneManager.OpenScene(CampusSceneBuilder.ScenePath, OpenSceneMode.Single);
            foreach (var car in Object.FindObjectsByType<TwinCar>(FindObjectsSortMode.None))
                Object.DestroyImmediate(car.gameObject);
            var cam = Camera.main;
            if (cam != null)
            {
                var chase = cam.GetComponent<ChaseCamera>();
                if (chase != null) Object.DestroyImmediate(chase);
                cam.transform.SetPositionAndRotation(new Vector3(0f, 40f, -30f), Quaternion.Euler(55f, 0f, 0f));
            }
            var replay = new GameObject("Policy replay").AddComponent<PolicyReplay>();
            replay.bodyMaterial = FlatGroundSceneBuilder.ColourMaterial("ReplayBody", Color.white);
            replay.lineMaterial = UnlitMaterial("ReplayLine");
            EditorSceneManager.SaveScene(scene, ScenePath);
            FlatGroundSceneBuilder.RegisterScene(ScenePath);
            AssetDatabase.SaveAssets();
            Debug.Log($"Built {ScenePath} from {CampusSceneBuilder.ScenePath}. Press Play; it reads {DrivesPath}.");
        }

        private static Material UnlitMaterial(string name)
        {
            string path = $"{GenDir}/{name}.mat";
            var m = AssetDatabase.LoadAssetAtPath<Material>(path);
            if (m == null)
            {
                var shader = Shader.Find("Universal Render Pipeline/Unlit") ?? Shader.Find("Unlit/Color");
                m = new Material(shader);
                AssetDatabase.CreateAsset(m, path);
            }
            m.color = Color.white;
            EditorUtility.SetDirty(m);
            return m;
        }

        [MenuItem("RcTwin/Build Replay Player")]
        public static void BuildPlayer()
        {
            if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo()) return;
            BuildPlayerBatch();
        }

        /// <summary>Builds the scene and the player; throws on failure, so a
        /// batch run exits non-zero.</summary>
        public static void BuildPlayerBatch()
        {
            BuildScene();
            Directory.CreateDirectory(Path.GetDirectoryName(PlayerPath));
            var report = BuildPipeline.BuildPlayer(new BuildPlayerOptions
            {
                scenes = new[] { ScenePath },
                locationPathName = PlayerPath,
                target = BuildTarget.StandaloneWindows64,
                options = BuildOptions.None,
            });
            if (report.summary.result != BuildResult.Succeeded)
                throw new Exception($"replay player build {report.summary.result}: {report.summary.totalErrors} errors");
            if (File.Exists(DrivesPath))
                File.Copy(DrivesPath, Path.Combine(Path.GetDirectoryName(PlayerPath), "drives.json"), true);
            Debug.Log($"Built {PlayerPath} ({report.summary.totalSize / (1024 * 1024)} MB)" +
                      (File.Exists(DrivesPath) ? $" with {DrivesPath} beside it." : "; no results/drives.json to copy."));
        }
    }
}
