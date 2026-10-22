// The campus look, applied by CampusSceneBuilder (THIRD_PARTY.md, "Campus look"):
//   - a golden-hour sky (Poly Haven HDRI) and a low warm sun from where the
//     sky's sun is, soft shadows, warm haze, and hills rising to a mountain
//     range round the campus, highest toward the sun;
//   - post-processing: ACES tonemapping, bloom, SSAO (the renderer's
//     feature, tuned to the 1/10 scale), a warm grade and a vignette;
//   - PBR surfaces (URP Lit, smoothness = 1 - roughness): asphalt with a wet
//     sheen, concrete sidewalks and walls, grass;
//   - the campus dressed with trees, hedges, lamps, rocks and benches
//     (CampusDressing).
// Textures and models come from tools/fetch_assets.py and
// tools/prepare_assets.py (Generated/Look/, gitignored). Without them the
// light, sky (procedural), haze and post-processing still apply and the
// surfaces keep their plain colours. Drawn only: nothing here has a collider.
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;
using Object = UnityEngine.Object;

namespace RcTwin.EditorTools
{
    public static class CampusLook
    {
        public const string LookDir = "Assets/RcTwin/Generated/Look";
        private const string MatDir = LookDir + "/Materials";
        public const string ProfilePath = "Assets/RcTwin/Generated/CampusLook.asset";
        private const string ImportStamp = "rc-physics-twin look 1";

        /// <summary>The sun's elevation [deg]: the sky's sun sits on the horizon; a little
        /// higher keeps the shadows long but the roads lit.</summary>
        public const float SunElevation = 14f;
        /// <summary>The sun's bearing from the camera's first view [deg, clockwise from above]:
        /// ahead and to the side, so the scene is half backlit.</summary>
        public const float SunBearing = 58f;

        public static string ManifestFile => Path.GetFullPath(Path.Combine(Application.dataPath, "..", LookDir, "look.json"));
        public static bool Available => File.Exists(ManifestFile);

        private static Dictionary<string, object> _manifest;
        private static Dictionary<string, object> Manifest()
        {
            if (_manifest == null && Available) _manifest = (Dictionary<string, object>)MiniJson.Parse(File.ReadAllText(ManifestFile));
            return _manifest;
        }

        /// <summary>What the scene builder made, for the look to dress.</summary>
        public sealed class Parts
        {
            public GameObject Asphalt, Sidewalks, Grass;
            public Transform FleetBuildings, Scenery;
            public CampusLayout Layout;
            public Vector3 View = Vector3.forward;   // the camera's first view, in plan
        }

        public static void Apply(Parts parts)
        {
            _manifest = null;
            bool assets = Available;
            if (assets) ImportAll();
            else Debug.Log($"No {ManifestFile}: the campus keeps plain surfaces (tools/fetch_assets.py, then tools/prepare_assets.py in Blender).");
            var sun = SkyAndSun(parts.View);
            Mountains(parts.Layout, -sun.transform.forward);
            Pipeline();
            PostProcessing();
            CameraSetup(Camera.main);
            if (assets)
            {
                Surfaces(parts);
                var dressing = CampusDressing.Build(CampusDressing.Plan(parts.Layout), parts.Layout);
                // The dressing's trees replace the local imagery's low-poly ones (CampusScenery).
                var blobs = dressing != null && parts.Scenery != null ? parts.Scenery.Find("Trees") : null;
                if (blobs != null)
                {
                    blobs.gameObject.SetActive(false);
                    blobs.name = "Trees (local imagery; off: the dressing's trees stand instead)";
                }
            }
            // Reflections baked with the dressing standing; the scene keeps only its list.
            var view = Object.FindFirstObjectByType<CampusDressingView>();
            view?.Spawn();
            Reflections(parts.Layout);
            view?.Clear();
            RenderSettings.sun = sun;
        }

        /// <summary>A prepared prop's model (its FBX, LODGroup and materials), or null.</summary>
        public static GameObject Model(string prop) =>
            AssetDatabase.LoadAssetAtPath<GameObject>($"{LookDir}/Models/{prop}.fbx");

        // ------------------------------------------------------------ import

        private static void ImportAll()
        {
            AssetDatabase.Refresh();
            var sets = (Dictionary<string, object>)Manifest()["texture_sets"];
            var props = (Dictionary<string, object>)Manifest()["props"];
            var cutout = new HashSet<string>();
            foreach (Dictionary<string, object> p in props.Values)
                foreach (Dictionary<string, object> m in ((Dictionary<string, object>)p["materials"]).Values)
                    if ((string)m["mode"] == "cutout") cutout.Add((string)m["set"]);

            AssetDatabase.StartAssetEditing();
            try
            {
                foreach (var set in sets.Keys)
                {
                    Texture($"{LookDir}/Textures/{set}_albedo.png", ti =>
                    {
                        ti.sRGBTexture = true;
                        ti.alphaIsTransparency = cutout.Contains(set);
                        ti.mipMapsPreserveCoverage = cutout.Contains(set);   // leaves stay full in the distance
                        ti.alphaTestReferenceValue = 0.5f;
                    });
                    Texture($"{LookDir}/Textures/{set}_normal.png", ti => ti.textureType = TextureImporterType.NormalMap);
                    Texture($"{LookDir}/Textures/{set}_mask.png", ti => ti.sRGBTexture = false);
                }
                Texture($"{LookDir}/{(string)Manifest()["sky"]}", ti =>
                {
                    ti.textureShape = TextureImporterShape.Texture2D;
                    ti.mipmapEnabled = false;   // no seam where the panorama wraps
                    ti.wrapModeU = TextureWrapMode.Repeat;
                    ti.wrapModeV = TextureWrapMode.Clamp;
                    ti.maxTextureSize = 4096;
                    ti.textureCompression = TextureImporterCompression.CompressedHQ;
                });
            }
            finally { AssetDatabase.StopAssetEditing(); }

            AssetDatabase.StartAssetEditing();
            try
            {
                foreach (var kv in props)
                {
                    var p = (Dictionary<string, object>)kv.Value;
                    string path = $"{LookDir}/{(string)p["fbx"]}";
                    if (!(AssetImporter.GetAtPath(path) is ModelImporter mi)) continue;
                    var mats = (Dictionary<string, object>)p["materials"];
                    foreach (var m in mats)
                    {
                        var d = (Dictionary<string, object>)m.Value;
                        mi.AddRemap(new AssetImporter.SourceAssetIdentifier(typeof(Material), m.Key),
                                    PropMaterial(m.Key, (string)d["set"], (string)d["mode"]));
                    }
                    if (mi.userData == ImportStamp) continue;
                    mi.globalScale = 1f;
                    mi.useFileScale = true;
                    mi.importCameras = false;
                    mi.importLights = false;
                    mi.importVisibility = false;
                    mi.importAnimation = false;
                    mi.animationType = ModelImporterAnimationType.None;
                    mi.addCollider = false;
                    mi.isReadable = false;
                    mi.importNormals = ModelImporterNormals.Import;
                    mi.importTangents = ModelImporterTangents.CalculateMikk;
                    mi.materialImportMode = ModelImporterMaterialImportMode.ImportViaMaterialDescription;
                    mi.materialLocation = ModelImporterMaterialLocation.InPrefab;
                    mi.userData = ImportStamp;
                    mi.SaveAndReimport();
                }
            }
            finally { AssetDatabase.StopAssetEditing(); }
        }

        private static void Texture(string path, Action<TextureImporter> set)
        {
            if (!(AssetImporter.GetAtPath(path) is TextureImporter ti) || ti.userData == ImportStamp) return;
            ti.mipmapEnabled = true;
            ti.anisoLevel = 8;
            ti.maxTextureSize = 2048;
            set(ti);
            ti.userData = ImportStamp;
            ti.SaveAndReimport();
        }

        private static Texture2D Tex(string set, string map) =>
            AssetDatabase.LoadAssetAtPath<Texture2D>($"{LookDir}/Textures/{set}_{map}.png");

        // --------------------------------------------------------- materials

        private static Material PropMaterial(string name, string set, string mode)
        {
            var m = Lit(name, set, Color.white);
            switch (mode)
            {
                case "cutout":
                    m.SetFloat("_Smoothness", 0.15f);   // leaves: a soft sheen, not the sky's glint
                    // Light through the leaves, faked: a little of their own colour glows,
                    // so a crown against the sun is deep green, not black.
                    m.EnableKeyword("_EMISSION");
                    m.SetTexture("_EmissionMap", Tex(set, "albedo"));
                    m.SetColor("_EmissionColor", new Color(0.55f, 0.6f, 0.38f));
                    // A crown's leaves don't take shadows (their own, mostly): its sunny side lights
                    // whole, as a canopy reads; they still cast them.
                    m.SetFloat("_ReceiveShadows", 0f);
                    m.EnableKeyword("_RECEIVE_SHADOWS_OFF");
                    m.globalIlluminationFlags = MaterialGlobalIlluminationFlags.RealtimeEmissive;   // URP keeps _EMISSION only for an emissive flag
                    m.SetFloat("_AlphaClip", 1f);
                    m.SetFloat("_Cutoff", 0.5f);
                    m.EnableKeyword("_ALPHATEST_ON");
                    m.SetFloat("_Cull", (float)CullMode.Off);   // leaves seen from both sides
                    if (m.HasProperty("_AlphaToMask")) m.SetFloat("_AlphaToMask", 1f);
                    m.SetOverrideTag("RenderType", "TransparentCutout");
                    m.renderQueue = (int)RenderQueue.AlphaTest;
                    break;
                case "glass":
                    m.SetFloat("_Surface", 1f);
                    m.SetFloat("_Blend", 0f);
                    m.EnableKeyword("_SURFACE_TYPE_TRANSPARENT");
                    m.SetFloat("_SrcBlend", (float)BlendMode.SrcAlpha);
                    m.SetFloat("_DstBlend", (float)BlendMode.OneMinusSrcAlpha);
                    m.SetFloat("_SrcBlendAlpha", (float)BlendMode.One);
                    m.SetFloat("_DstBlendAlpha", (float)BlendMode.OneMinusSrcAlpha);
                    m.SetFloat("_ZWrite", 0f);
                    m.SetOverrideTag("RenderType", "Transparent");
                    m.renderQueue = (int)RenderQueue.Transparent;
                    m.SetShaderPassEnabled("ShadowCaster", false);
                    m.SetShaderPassEnabled("DepthOnly", false);
                    break;
                case "emissive":
                    m.EnableKeyword("_EMISSION");
                    m.SetColor("_EmissionColor", new Color(1f, 0.74f, 0.46f) * 5f);
                    m.globalIlluminationFlags = MaterialGlobalIlluminationFlags.RealtimeEmissive;   // URP keeps _EMISSION only for an emissive flag
                    break;
            }
            EditorUtility.SetDirty(m);
            return m;
        }

        /// <summary>URP Lit from a prepared texture set: albedo, normal, metallic with
        /// smoothness in alpha. Reset each build, so the look is what this code says.</summary>
        private static Material Lit(string name, string set, Color tint)
        {
            string path = $"{MatDir}/{name}.mat";
            var m = AssetDatabase.LoadAssetAtPath<Material>(path);
            if (m == null)
            {
                Directory.CreateDirectory(MatDir);
                m = new Material(FlatGroundSceneBuilder.LitShader());
                AssetDatabase.CreateAsset(m, path);
            }
            m.shader = FlatGroundSceneBuilder.LitShader();
            m.shaderKeywords = Array.Empty<string>();
            m.renderQueue = -1;
            var albedo = Tex(set, "albedo");
            m.SetTexture("_BaseMap", albedo);
            m.mainTexture = albedo;
            m.SetColor("_BaseColor", tint);
            var nor = Tex(set, "normal");
            if (nor != null)
            {
                m.SetTexture("_BumpMap", nor);
                m.SetFloat("_BumpScale", 1f);
                m.EnableKeyword("_NORMALMAP");
            }
            var mask = Tex(set, "mask");
            if (mask != null)
            {
                m.SetTexture("_MetallicGlossMap", mask);
                m.EnableKeyword("_METALLICSPECGLOSSMAP");
                m.SetFloat("_SmoothnessTextureChannel", 0f);
                m.SetFloat("_Smoothness", 1f);   // with the map, a scale on its alpha
            }
            m.enableInstancing = true;
            EditorUtility.SetDirty(m);
            return m;
        }

        // ------------------------------------------------------- sky and sun

        /// <summary>World direction to a texel of an equirectangular sky drawn by
        /// Skybox/Panoramic turned by rotationDeg (its ToRadialCoords, inverted).</summary>
        public static Vector3 PanoramaDirection(float u, float v, float rotationDeg)
        {
            float lon = (0.5f - u) * 2f * Mathf.PI, lat = (1f - v) * Mathf.PI;
            var t = new Vector3(Mathf.Sin(lat) * Mathf.Cos(lon), Mathf.Cos(lat), Mathf.Sin(lat) * Mathf.Sin(lon));
            float a = rotationDeg * Mathf.Deg2Rad;
            return new Vector3(Mathf.Cos(a) * t.x - Mathf.Sin(a) * t.z, t.y, Mathf.Sin(a) * t.x + Mathf.Cos(a) * t.z);
        }

        /// <summary>The panorama's rotation that puts the texel (u, v) at the plan
        /// bearing of `toward` (x, z).</summary>
        public static float PanoramaRotation(float u, Vector3 toward)
        {
            float lon = (0.5f - u) * 2f * Mathf.PI;
            float want = Mathf.Atan2(toward.z, toward.x);
            return Mathf.Repeat((want - lon) * Mathf.Rad2Deg, 360f);
        }

        private static Light SkyAndSun(Vector3 view)
        {
            view = new Vector3(view.x, 0f, view.z).normalized;
            Vector3 toward = Quaternion.Euler(0f, SunBearing, 0f) * view;   // the sun's bearing in plan
            Material sky;
            var doc = Manifest();
            var hdr = doc != null ? AssetDatabase.LoadAssetAtPath<Texture>($"{LookDir}/{(string)doc["sky"]}") : null;
            if (hdr != null)
            {
                var s = (Dictionary<string, object>)doc["sun"];
                float u = F(s["u"]);
                sky = SkyMaterial("Sky", "Skybox/Panoramic");
                sky.SetTexture("_MainTex", hdr);
                sky.SetFloat("_Mapping", 1f);     // latitude-longitude
                sky.SetFloat("_ImageType", 0f);   // 360 degrees
                sky.SetFloat("_Exposure", 1f);
                sky.SetFloat("_Rotation", PanoramaRotation(u, toward));
                sky.SetColor("_Tint", new Color(0.5f, 0.5f, 0.5f));
            }
            else
            {
                sky = SkyMaterial("SkyProcedural", "Skybox/Procedural");
                sky.SetFloat("_SunSize", 0.035f);
                sky.SetFloat("_SunSizeConvergence", 4f);
                sky.SetFloat("_AtmosphereThickness", 1.3f);
                sky.SetColor("_SkyTint", new Color(0.55f, 0.52f, 0.52f));
                sky.SetColor("_GroundColor", new Color(0.38f, 0.34f, 0.3f));
                sky.SetFloat("_Exposure", 1f);
            }
            RenderSettings.skybox = sky;

            var light = Object.FindFirstObjectByType<Light>();
            if (light == null || light.type != LightType.Directional)
                light = new GameObject("Directional Light").AddComponent<Light>();
            light.name = "Sun (golden hour)";
            light.type = LightType.Directional;
            Vector3 toSun = Quaternion.AngleAxis(-SunElevation, Vector3.Cross(Vector3.up, toward)) * toward;
            light.transform.rotation = Quaternion.LookRotation(-toSun);
            light.color = new Color(1f, 0.85f, 0.68f);
            light.intensity = 2.3f;
            light.shadows = LightShadows.Soft;
            light.shadowStrength = 0.82f;

            RenderSettings.ambientMode = AmbientMode.Trilight;
            RenderSettings.ambientSkyColor = new Color(0.7f, 0.77f, 0.92f);
            RenderSettings.ambientEquatorColor = new Color(0.72f, 0.64f, 0.58f);
            RenderSettings.ambientGroundColor = new Color(0.3f, 0.28f, 0.24f);
            RenderSettings.fog = true;
            RenderSettings.fogMode = FogMode.ExponentialSquared;
            RenderSettings.fogDensity = 0.0032f;   // the mountains, 2-7 km off at full scale, half in the haze
            RenderSettings.fogColor = new Color(0.82f, 0.8f, 0.8f);   // distance haze, a little warm
            return light;
        }

        private static Material SkyMaterial(string name, string shader)
        {
            string path = $"Assets/RcTwin/Generated/{name}.mat";
            if (name == "Sky") path = $"{MatDir}/{name}.mat";
            var m = AssetDatabase.LoadAssetAtPath<Material>(path);
            if (m == null)
            {
                Directory.CreateDirectory(Path.GetDirectoryName(path));
                m = new Material(Shader.Find(shader));
                AssetDatabase.CreateAsset(m, path);
            }
            m.shader = Shader.Find(shader);
            EditorUtility.SetDirty(m);
            return m;
        }

        // --------------------------------------------------------- mountains

        public const float HillsFrom = 230f, HillsTo = 680f, PeakHeight = 95f;

        /// <summary>A ring of hills round the campus rising to ridged mountains, highest
        /// toward the sun so the range stands backlit against the bright sky; drawn only,
        /// far beyond the shadows. Deterministic.</summary>
        private static void Mountains(CampusLayout L, Vector3 toSun)
        {
            Vector2 lo = new Vector2(float.MaxValue, float.MaxValue), hi = -lo;
            foreach (var r in L.Roads)
                foreach (var q in r) { lo = Vector2.Min(lo, q); hi = Vector2.Max(hi, q); }
            Vector2 centre = 0.5f * (lo + hi), sun = new Vector2(toSun.x, toSun.z).normalized;
            const int seg = 900, rings = 70;
            var v = new Vector3[(seg + 1) * (rings + 1)];
            var uv = new Vector2[v.Length];
            for (int i = 0; i <= seg; i++)
            {
                float a = 2f * Mathf.PI * i / seg;
                var dir = new Vector2(Mathf.Cos(a), Mathf.Sin(a));
                float range = 0.3f + 0.7f * Mathf.SmoothStep(0f, 1f, Mathf.InverseLerp(-0.3f, 0.85f, Vector2.Dot(dir, sun)));
                for (int j = 0; j <= rings; j++)
                {
                    float t = (float)j / rings, r = Mathf.Lerp(HillsFrom, HillsTo, t);
                    var p = centre + dir * r;
                    float ridge = 0f, amp = 1f, f = 0.0045f;
                    for (int o = 0; o < 5; o++)   // ridged fractal noise
                    {
                        float n = 1f - Mathf.Abs(2f * Mathf.PerlinNoise(p.x * f + 31.7f * o, p.y * f - 17.3f * o) - 1f);
                        ridge += amp * n * n;
                        amp *= 0.5f;
                        f *= 2.1f;
                    }
                    float rise = Mathf.SmoothStep(0f, 1f, Mathf.InverseLerp(0f, 0.55f, t));
                    float hills = 9f * Mathf.PerlinNoise(p.x * 0.02f + 5f, p.y * 0.02f + 9f) * Mathf.SmoothStep(0f, 1f, t * 6f);
                    float h = PeakHeight * range * rise * ridge / 1.6f + hills;
                    v[i * (rings + 1) + j] = new Vector3(p.x, h - 0.05f, p.y);
                    uv[i * (rings + 1) + j] = new Vector2(i * 28f / seg, Mathf.Clamp01(h / PeakHeight));
                }
            }
            var tris = new List<int>(seg * rings * 6);
            for (int i = 0; i < seg; i++)
                for (int j = 0; j < rings; j++)
                {
                    int a = i * (rings + 1) + j, b = a + rings + 1;
                    tris.AddRange(new[] { a, a + 1, b, b, a + 1, b + 1 });
                }
            var mesh = new Mesh { name = "Mountains", indexFormat = IndexFormat.UInt32 };
            mesh.vertices = v;
            mesh.uv = uv;
            mesh.SetTriangles(tris, 0);
            if (Vector3.Dot(Vector3.Cross(v[1] - v[0], v[rings + 1] - v[0]), Vector3.up) < 0f) { tris.Reverse(); mesh.SetTriangles(tris, 0); }
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            Directory.CreateDirectory(Path.Combine(Application.dataPath, "..", LookDir));
            string path = $"{LookDir}/Mountains.asset";
            AssetDatabase.DeleteAsset(path);
            AssetDatabase.CreateAsset(mesh, path);

            var m = new Material(FlatGroundSceneBuilder.LitShader()) { name = "Mountains" };
            string matPath = $"{LookDir}/Mountains.mat";
            AssetDatabase.DeleteAsset(matPath);
            AssetDatabase.CreateAsset(m, matPath);
            m.SetTexture("_BaseMap", MountainTexture());
            m.SetColor("_BaseColor", Color.white);
            m.SetFloat("_Smoothness", 0.08f);
            EditorUtility.SetDirty(m);
            var go = new GameObject("Mountains (drawn only, in the haze)");
            go.AddComponent<MeshFilter>().sharedMesh = mesh;
            var mr = go.AddComponent<MeshRenderer>();
            mr.sharedMaterial = m;
            mr.shadowCastingMode = ShadowCastingMode.Off;
            mr.receiveShadows = false;
        }

        // Forest below, scrub and grass above, rock on the tops; mottled. v is height.
        private static Texture2D MountainTexture()
        {
            string path = $"{LookDir}/Textures/mountains_albedo.png";
            const int w = 512, h = 256;
            var px = new Color[w * h];
            Color forest = new Color(0.12f, 0.19f, 0.1f), scrub = new Color(0.3f, 0.34f, 0.19f), rock = new Color(0.42f, 0.39f, 0.34f);
            for (int y = 0; y < h; y++)
                for (int x = 0; x < w; x++)
                {
                    float t = y / (h - 1f);
                    float n = Mathf.PerlinNoise(x * 0.06f, y * 0.11f), n2 = Mathf.PerlinNoise(x * 0.23f + 40f, y * 0.31f);
                    float tt = t + 0.25f * (n - 0.5f);
                    var c = tt < 0.45f ? Color.Lerp(forest, scrub, Mathf.SmoothStep(0f, 1f, tt / 0.45f) * (0.4f + 0.6f * n2))
                                       : Color.Lerp(scrub, rock, Mathf.SmoothStep(0f, 1f, (tt - 0.45f) / 0.45f));
                    c *= 0.85f + 0.3f * n2;
                    c.a = 1f;
                    px[y * w + x] = c;
                }
            var tex = new Texture2D(w, h, TextureFormat.RGBA32, true);
            tex.SetPixels(px);
            tex.Apply();
            Directory.CreateDirectory(Path.Combine(Application.dataPath, "..", LookDir, "Textures"));
            File.WriteAllBytes(Path.Combine(Application.dataPath, "..", path), tex.EncodeToPNG());
            Object.DestroyImmediate(tex);
            AssetDatabase.ImportAsset(path, ImportAssetOptions.ForceSynchronousImport);
            var ti = (TextureImporter)AssetImporter.GetAtPath(path);
            ti.wrapModeU = TextureWrapMode.Repeat;
            ti.wrapModeV = TextureWrapMode.Clamp;
            ti.SaveAndReimport();
            return AssetDatabase.LoadAssetAtPath<Texture2D>(path);
        }

        // ----------------------------------------------- pipeline and post

        private static void Pipeline()
        {
            if (!((QualitySettings.renderPipeline ?? GraphicsSettings.defaultRenderPipeline) is UniversalRenderPipelineAsset rp)) return;
            rp.shadowDistance = 32f;
            rp.shadowCascadeCount = 4;
            rp.supportsHDR = true;
            rp.msaaSampleCount = 4;
            var so = new SerializedObject(rp);
            so.FindProperty("m_MainLightShadowmapResolution").intValue = 4096;
            so.FindProperty("m_SoftShadowQuality").intValue = 3;   // high
            so.ApplyModifiedPropertiesWithoutUndo();
            EditorUtility.SetDirty(rp);
            var list = so.FindProperty("m_RendererDataList");
            if (list == null || list.arraySize == 0 || !(list.GetArrayElementAtIndex(0).objectReferenceValue is ScriptableRendererData data)) return;
            foreach (var f in data.rendererFeatures)
            {
                if (!(f is ScreenSpaceAmbientOcclusion ssao)) continue;
                var s = new SerializedObject(ssao);
                s.FindProperty("m_Settings.Intensity").floatValue = 1.1f;
                s.FindProperty("m_Settings.Radius").floatValue = 0.07f;            // 0.7 m at full scale
                s.FindProperty("m_Settings.DirectLightingStrength").floatValue = 0.3f;
                s.FindProperty("m_Settings.Falloff").floatValue = 60f;
                s.FindProperty("m_Settings.Samples").intValue = 0;                  // high
                s.FindProperty("m_Settings.BlurQuality").intValue = 0;              // high
                s.ApplyModifiedPropertiesWithoutUndo();
                ssao.SetActive(true);
                EditorUtility.SetDirty(ssao);
            }
            EditorUtility.SetDirty(data);
        }

        private static void PostProcessing()
        {
            AssetDatabase.DeleteAsset(ProfilePath);
            var profile = ScriptableObject.CreateInstance<VolumeProfile>();
            AssetDatabase.CreateAsset(profile, ProfilePath);
            T Add<T>() where T : VolumeComponent
            {
                var c = profile.Add<T>(true);
                c.name = typeof(T).Name;
                AssetDatabase.AddObjectToAsset(c, profile);
                return c;
            }
            Add<Tonemapping>().mode.Override(TonemappingMode.ACES);
            var bloom = Add<Bloom>();
            bloom.threshold.Override(0.95f);
            bloom.intensity.Override(0.55f);
            bloom.scatter.Override(0.7f);
            bloom.tint.Override(new Color(1f, 0.86f, 0.72f));
            var grade = Add<ColorAdjustments>();
            grade.postExposure.Override(0.3f);
            grade.contrast.Override(12f);
            grade.saturation.Override(12f);
            grade.colorFilter.Override(new Color(1f, 0.985f, 0.96f));
            Add<WhiteBalance>().temperature.Override(4f);
            var smh = Add<ShadowsMidtonesHighlights>();   // teal shadows, warm highlights
            smh.shadows.Override(new Vector4(0.94f, 0.99f, 1.08f, 0f));
            smh.highlights.Override(new Vector4(1.05f, 1f, 0.94f, 0f));
            var vignette = Add<Vignette>();
            vignette.intensity.Override(0.24f);
            vignette.smoothness.Override(0.45f);
            EditorUtility.SetDirty(profile);

            var go = new GameObject("Post-processing (golden hour)");
            var volume = go.AddComponent<Volume>();
            volume.isGlobal = true;
            volume.sharedProfile = profile;
        }

        private static void CameraSetup(Camera cam)
        {
            if (cam == null) return;
            cam.allowHDR = true;
            cam.allowMSAA = true;
            cam.farClipPlane = 1000f;
            var data = cam.GetUniversalAdditionalCameraData();
            data.renderPostProcessing = true;
            data.antialiasing = AntialiasingMode.SubpixelMorphologicalAntiAliasing;
            data.antialiasingQuality = AntialiasingQuality.High;
        }

        private static void Reflections(CampusLayout L)
        {
            Vector2 lo = new Vector2(float.MaxValue, float.MaxValue), hi = -lo;
            foreach (var r in L.Roads)
                foreach (var p in r) { lo = Vector2.Min(lo, p); hi = Vector2.Max(hi, p); }
            // Baked here, once the scene is dressed: the wet road and the windows
            // reflect the sky, the trees and the buildings.
            var go = new GameObject("Reflections (baked by the scene builder)");
            go.transform.position = new Vector3(0.5f * (lo.x + hi.x), 1.2f, 0.5f * (lo.y + hi.y));
            var probe = go.AddComponent<ReflectionProbe>();
            probe.mode = ReflectionProbeMode.Custom;
            probe.size = new Vector3(hi.x - lo.x + 30f, 20f, hi.y - lo.y + 30f);
            probe.resolution = 256;
            probe.hdr = true;
            probe.clearFlags = ReflectionProbeClearFlags.Skybox;
            probe.nearClipPlane = 0.05f;
            probe.farClipPlane = 300f;
            probe.shadowDistance = 20f;
            string path = $"{LookDir}/CampusReflections.exr";
            Directory.CreateDirectory(Path.Combine(Application.dataPath, "..", LookDir));
            if (Lightmapping.BakeReflectionProbe(probe, path))
                probe.customBakedTexture = AssetDatabase.LoadAssetAtPath<Cubemap>(path);
            else
                Debug.LogWarning("Campus look: the reflection probe did not bake (no graphics device?); reflections fall back to the sky.");
        }

        // ---------------------------------------------------------- surfaces

        private static void Surfaces(Parts parts)
        {
            // The road: asphalt with a wet sheen (its lane lines are plain paint).
            Paint(parts.Asphalt, Lit("Road", "asphalt_02", new Color(0.9f, 0.9f, 0.93f)), 0.3f);
            Paint(parts.Sidewalks, Lit("Sidewalk", "concrete_pavement", new Color(0.82f, 0.84f, 0.87f)), 0.18f);

            var grass = Lit("Grass", "leafy_grass", new Color(0.86f, 0.9f, 0.8f));
            if (parts.Grass != null && parts.Grass.TryGetComponent<MeshRenderer>(out var gr))
            {
                var tiled = Lit("GrassNear", "leafy_grass", new Color(0.86f, 0.9f, 0.8f));
                Vector3 size = parts.Grass.transform.localScale * 10f;   // a plane is 10 m across
                tiled.SetTextureScale("_BaseMap", new Vector2(size.x / 0.25f, size.z / 0.25f));
                gr.sharedMaterial = tiled;
            }

            // Buildings: rendered walls with windows in each kind's colour, concrete roofs.
            Facade();
            var fleetWall = FacadeMaterial("BuildingBox", new Color(0.95f, 0.93f, 0.88f));
            fleetWall.SetTextureScale("_BaseMap", new Vector2(2f, 4f));   // a cube's faces are 0..1
            if (parts.FleetBuildings != null)
                foreach (var r in parts.FleetBuildings.GetComponentsInChildren<MeshRenderer>(true)) r.sharedMaterial = fleetWall;
            if (parts.Scenery == null) return;
            var tints = new Dictionary<string, Color>
            {
                { "academic", new Color(1f, 0.95f, 0.88f) },
                { "hostel", new Color(1f, 1f, 0.97f) },
                { "other", new Color(0.95f, 0.93f, 0.9f) },
            };
            foreach (Transform t in parts.Scenery.GetComponentsInChildren<Transform>(true))
            {
                if (t.name.StartsWith("Buildings, ", StringComparison.Ordinal))
                {
                    string kind = t.name.Substring("Buildings, ".Length);
                    var wall = FacadeMaterial($"Wall_{kind}", tints.TryGetValue(kind, out var c) ? c : Color.white);
                    wall.SetTextureScale("_BaseMap", new Vector2(1f / (FacadeBays * BayWidth), 1f / (FacadeFloors * FloorHeight)));
                    Paint(t.gameObject, wall, 1f);
                }
                else if (t.name == "Roofs")
                    Paint(t.gameObject, Lit("Roof", "concrete_pavement", new Color(0.8f, 0.8f, 0.82f)), 0.3f);
                else if (t.name.StartsWith("Ground (", StringComparison.Ordinal) && t.TryGetComponent<MeshRenderer>(out var ground))
                    ground.sharedMaterial = GroundImagery(ground, t.GetComponent<MeshFilter>().sharedMesh);
                else if (t.name.StartsWith("Land beyond", StringComparison.Ordinal) && t.TryGetComponent<MeshRenderer>(out var land))
                {
                    var far = Lit("GrassFar", "leafy_grass", new Color(0.78f, 0.8f, 0.6f));
                    t.localScale = new Vector3(150f, 1f, 150f);   // to where the haze is whole
                    Vector3 size = t.localScale * 10f;
                    far.SetTextureScale("_BaseMap", new Vector2(size.x / 0.4f, size.z / 0.4f));
                    land.sharedMaterial = far;
                }
            }
        }

        // ----------------------------------------------------------- facades

        // The facade texture: FacadeBays window bays across, FacadeFloors floors up [m at 1/10].
        public const int FacadeBays = 4, FacadeFloors = 3;
        public const float BayWidth = 0.34f, FloorHeight = 0.34f;
        private const int BayPx = 192, FloorPx = 192;

        /// <summary>Writes the facade's albedo, normal, metallic-smoothness and emission maps
        /// (concrete wall, framed windows with sills, a slab line at each floor, a few
        /// windows lit warm), once; deterministic.</summary>
        private static void Facade()
        {
            string dir = $"{LookDir}/Textures";
            if (File.Exists(Path.Combine(Application.dataPath, "..", dir, "facade_albedo.png"))) return;
            int w = FacadeBays * BayPx, h = FacadeFloors * FloorPx;
            var albedo = new Color[w * h];
            var height = new float[w * h];
            var mask = new Color[w * h];
            var glow = new Color[w * h];
            var rng = new System.Random(3);
            var tint = new Color[FacadeBays * FacadeFloors];
            var lit = new bool[FacadeBays * FacadeFloors];
            for (int i = 0; i < tint.Length; i++)
            {
                double t = rng.NextDouble();
                tint[i] = t < 0.25 ? new Color(0.62f, 0.58f, 0.52f)            // blinds drawn
                        : Color.Lerp(new Color(0.1f, 0.13f, 0.17f), new Color(0.2f, 0.26f, 0.32f), (float)rng.NextDouble());
                lit[i] = rng.NextDouble() < 0.22;
            }
            for (int y = 0; y < h; y++)
                for (int x = 0; x < w; x++)
                {
                    int k = y * w + x;
                    float bx = (x % BayPx) / (float)BayPx, fy = (y % FloorPx) / (float)FloorPx;
                    int win = (y / FloorPx) * FacadeBays + x / BayPx;
                    float n = Mathf.PerlinNoise(x * 0.05f, y * 0.05f) * 0.6f + Mathf.PerlinNoise(x * 0.31f, y * 0.29f) * 0.4f;
                    var wall = new Color(0.8f, 0.78f, 0.74f) * (0.93f + 0.1f * n);
                    float hgt = 1f, smooth = 0.18f;
                    Color e = Color.black;
                    bool inWin = bx > 0.2f && bx < 0.8f && fy > 0.3f && fy < 0.8f;
                    bool frame = bx > 0.18f && bx < 0.82f && fy > 0.28f && fy < 0.82f && !inWin;
                    if (fy < 0.06f) { wall *= 0.86f; hgt = 1.02f; }                                     // slab line
                    else if (bx > 0.16f && bx < 0.84f && fy > 0.24f && fy <= 0.28f) { wall *= 1.08f; hgt = 1.04f; }   // sill
                    if (frame) { wall = new Color(0.86f, 0.86f, 0.84f); hgt = 0.99f; smooth = 0.4f; }
                    if (inWin)
                    {
                        float sky = Mathf.Lerp(0.85f, 1.15f, fy);   // a little brighter toward the top
                        wall = tint[win] * sky;
                        if (Mathf.Abs(bx - 0.5f) < 0.012f) { wall = new Color(0.86f, 0.86f, 0.84f); hgt = 0.97f; smooth = 0.4f; }   // mullion
                        else
                        {
                            hgt = 0.92f;
                            smooth = 0.92f;
                            if (lit[win]) e = new Color(1f, 0.72f, 0.42f) * (0.7f + 0.3f * fy);
                        }
                    }
                    wall.a = 1f;
                    albedo[k] = wall;
                    height[k] = hgt;
                    mask[k] = new Color(0f, 1f, 0f, smooth);
                    e.a = 1f;
                    glow[k] = e;
                }
            var normal = new Color[w * h];
            for (int y = 0; y < h; y++)
                for (int x = 0; x < w; x++)
                {
                    float dx = height[y * w + (x + 1) % w] - height[y * w + (x + w - 1) % w];
                    float dy = height[((y + 1) % h) * w + x] - height[((y + h - 1) % h) * w + x];
                    var nv = new Vector3(-dx * 6f, -dy * 6f, 1f).normalized;
                    normal[y * w + x] = new Color(nv.x * 0.5f + 0.5f, nv.y * 0.5f + 0.5f, nv.z * 0.5f + 0.5f, 1f);
                }
            void Save(string map, Color[] px, bool linear, bool isNormal)
            {
                var tex = new Texture2D(w, h, TextureFormat.RGBA32, false, linear);
                tex.SetPixels(px);
                tex.Apply();
                string path = $"{dir}/facade_{map}.png";
                File.WriteAllBytes(Path.Combine(Application.dataPath, "..", path), tex.EncodeToPNG());
                Object.DestroyImmediate(tex);
                AssetDatabase.ImportAsset(path, ImportAssetOptions.ForceSynchronousImport);
                var ti = (TextureImporter)AssetImporter.GetAtPath(path);
                ti.sRGBTexture = !linear;
                if (isNormal) ti.textureType = TextureImporterType.NormalMap;
                ti.anisoLevel = 8;
                ti.SaveAndReimport();
            }
            Save("albedo", albedo, false, false);
            Save("normal", normal, true, true);
            Save("mask", mask, true, false);
            Save("emission", glow, false, false);
        }

        private static Material FacadeMaterial(string name, Color tint)
        {
            var m = Lit(name, "facade", tint);
            m.SetTexture("_EmissionMap", Tex("facade", "emission"));
            m.SetColor("_EmissionColor", Color.white * 1.4f);
            m.EnableKeyword("_EMISSION");
            m.globalIlluminationFlags = MaterialGlobalIlluminationFlags.RealtimeEmissive;   // URP keeps _EMISSION only for an emissive flag
            EditorUtility.SetDirty(m);
            return m;
        }

        /// <summary>The local ground imagery (land use, paths) with grass's relief over it;
        /// its bare-earth browns turned toward dry grass, the campus as after the rains.</summary>
        private static Material GroundImagery(MeshRenderer r, Mesh mesh)
        {
            var imagery = r.sharedMaterial != null ? r.sharedMaterial.mainTexture : null;
            string source = imagery != null ? AssetDatabase.GetAssetPath(imagery) : null;
            if (!string.IsNullOrEmpty(source))
            {
                var tex = new Texture2D(2, 2, TextureFormat.RGBA32, false);
                tex.LoadImage(File.ReadAllBytes(Path.Combine(Application.dataPath, "..", source)));
                var px = tex.GetPixels32();
                var dry = new Color(0.47f, 0.52f, 0.3f);
                for (int i = 0; i < px.Length; i++)
                {
                    Color c = px[i];
                    float brown = Mathf.Clamp01((c.r - c.g) * 8f);   // red over green: earth, fields
                    float grey = 1f - Mathf.Clamp01((Mathf.Max(c.r, c.g, c.b) - Mathf.Min(c.r, c.g, c.b)) * 6f);
                    var o = Color.Lerp(c, dry * (0.8f + 0.4f * c.grayscale / 0.45f), 0.75f * brown * (1f - grey));
                    o.a = 1f;
                    px[i] = o;
                }
                tex.SetPixels32(px);
                tex.Apply();
                string path = $"{LookDir}/Textures/ground_look.png";
                File.WriteAllBytes(Path.Combine(Application.dataPath, "..", path), tex.EncodeToPNG());
                Object.DestroyImmediate(tex);
                AssetDatabase.ImportAsset(path, ImportAssetOptions.ForceSynchronousImport);
                var ti = (TextureImporter)AssetImporter.GetAtPath(path);
                ti.wrapMode = TextureWrapMode.Clamp;
                ti.maxTextureSize = 4096;
                ti.anisoLevel = 8;
                ti.SaveAndReimport();
                imagery = AssetDatabase.LoadAssetAtPath<Texture2D>(path);
            }
            var m = Lit("GroundImagery", "leafy_grass", Color.white);
            m.SetTexture("_BaseMap", imagery);
            m.mainTexture = imagery;
            m.SetTexture("_MetallicGlossMap", null);
            m.DisableKeyword("_METALLICSPECGLOSSMAP");
            m.SetFloat("_Smoothness", 0.12f);
            m.SetTexture("_BumpMap", null);
            m.DisableKeyword("_NORMALMAP");
            m.SetTexture("_DetailNormalMap", Tex("leafy_grass", "normal"));
            m.SetFloat("_DetailNormalMapScale", 0.9f);
            Vector3 size = mesh.bounds.size;
            m.SetTextureScale("_DetailAlbedoMap", new Vector2(size.x / 0.25f, size.z / 0.25f));   // the detail maps' tiling
            m.EnableKeyword("_DETAIL_MULX2");
            m.EnableKeyword("_NORMALMAP");   // URP Lit blends the detail normal only with a normal map (here the flat default)
            EditorUtility.SetDirty(m);
            return m;
        }

        // A generated mesh: world box-mapped UVs (one tile per `tile` metres), then the material.
        private static void Paint(GameObject go, Material m, float tile)
        {
            if (go == null) return;
            if (go.TryGetComponent<MeshFilter>(out var mf) && mf.sharedMesh != null)
            {
                BoxUV(mf.sharedMesh, tile);
                EditorUtility.SetDirty(mf.sharedMesh);
            }
            if (go.TryGetComponent<MeshRenderer>(out var r)) r.sharedMaterial = m;
        }

        /// <summary>UVs from each vertex's position, projected along its normal's
        /// main axis, one texture tile per `tile` metres; tangents to match.</summary>
        public static void BoxUV(Mesh m, float tile)
        {
            var v = m.vertices;
            var n = m.normals;
            var uv = new Vector2[v.Length];
            for (int i = 0; i < v.Length; i++)
            {
                Vector3 a = n.Length == v.Length ? n[i] : Vector3.up;
                float ax = Mathf.Abs(a.x), ay = Mathf.Abs(a.y), az = Mathf.Abs(a.z);
                uv[i] = ay >= ax && ay >= az ? new Vector2(v[i].x, v[i].z)
                      : ax >= az ? new Vector2(v[i].z, v[i].y)
                      : new Vector2(v[i].x, v[i].y);
                uv[i] /= tile;
            }
            m.uv = uv;
            m.RecalculateTangents();
        }

        private static float F(object o) => (float)Convert.ToDouble(o, CultureInfo.InvariantCulture);
    }
}
