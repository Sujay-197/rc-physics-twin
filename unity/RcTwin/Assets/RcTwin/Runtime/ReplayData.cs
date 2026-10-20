// Recorded drives for the policy replay (tools/record_drives.py): which
// policies drove, in which conditions, over which campus trips (each trip's
// stops, car and route stored once, shared by every policy's drive), and
// every drive sampled at a fixed rate. Positions are ISO world [m]; the replay
// converts with TwinCar.ToUnity.
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;

namespace RcTwin
{
    public sealed class ReplayData
    {
        public const string Format = "rc-physics-twin drives 0.2.0";

        public sealed class Policy
        {
            public string spec, label;
            /// <summary>condition -> drives, in trip order (trip k is evaluation seed eval_seed + k).</summary>
            public Dictionary<string, List<Drive>> runs = new Dictionary<string, List<Drive>>();
        }

        public sealed class Drive
        {
            public string status, from, to;
            public int tripSeed, wallContacts;
            public float time, ret, offsetMean, energyWh, distance;
            /// <summary>When the replay stops showing this trip [s]; a timed-out
            /// drive is recorded up to then and at its end.</summary>
            public float until;
            public Vector2[] route;
            public Dictionary<string, float> car = new Dictionary<string, float>();
            /// <summary>(time, x, y) of each new wall contact.</summary>
            public List<Vector3> hits = new List<Vector3>();
            /// <summary>(time, x, y) where the mid-trip change struck, or null.</summary>
            public Vector3? perturbed;
            public Sample[] samples;
            public float[] odometer;   // distance driven to each sample [m]

            public bool Finished => status == "reached_end";
        }

        public struct Sample
        {
            public float x, y, yaw, speed, steer, throttle, offset;
        }

        public struct Pose
        {
            public float x, y, yaw, speed, steer, throttle, offset, odometer;
            public bool ended;
        }

        public string created;
        /// <summary>"pairs": every stop-to-stop route; "evaluation": the harness's draws.</summary>
        public string tripSet;
        public float targetSpeed;
        public int sampleHz;
        public List<string> conditions = new List<string>();
        public Dictionary<string, float> perturbation = new Dictionary<string, float>();
        public List<Policy> policies = new List<Policy>();
        public Dictionary<string, float> car = new Dictionary<string, float>();
        public float roadWidth;

        public int TripCount(string condition)
        {
            int n = 0;
            foreach (var p in policies)
                if (p.runs.TryGetValue(condition, out var r)) n = Math.Max(n, r.Count);
            return n;
        }

        public Drive Get(int policy, string condition, int trip)
        {
            if (!policies[policy].runs.TryGetValue(condition, out var r) || trip >= r.Count) return null;
            return r[trip];
        }

        /// <summary>The first trip from one stop to another in the condition, or -1.</summary>
        public int FindTrip(string condition, string from, string to)
        {
            foreach (var p in policies)
            {
                if (!p.runs.TryGetValue(condition, out var r)) continue;
                for (int k = 0; k < r.Count; k++)
                    if (r[k] != null && r[k].from == from && r[k].to == to) return k;
            }
            return -1;
        }

        /// <summary>Every stop that starts or ends a trip in the condition,
        /// letters then number (AB1, ..., S1, S2, ...), as record_drives.py orders them.</summary>
        public List<string> Stops(string condition)
        {
            var set = new HashSet<string>();
            foreach (var p in policies)
                if (p.runs.TryGetValue(condition, out var r))
                    foreach (var d in r)
                        if (d != null) { set.Add(d.from); set.Add(d.to); }
            var list = new List<string>(set);
            list.Sort(CompareStops);
            return list;
        }

        public static int CompareStops(string a, string b)
        {
            static (string, int) Key(string s)
            {
                int i = s.Length;
                while (i > 0 && char.IsDigit(s[i - 1])) i--;
                return (s.Substring(0, i), i < s.Length ? int.Parse(s.Substring(i), CultureInfo.InvariantCulture) : 0);
            }
            var (pa, na) = Key(a);
            var (pb, nb) = Key(b);
            int c = string.CompareOrdinal(pa, pb);
            return c != 0 ? c : na.CompareTo(nb);
        }

        public static ReplayData Parse(string json)
        {
            var root = (Dictionary<string, object>)MiniJson.Parse(json);
            if (!root.TryGetValue("format", out var fmt) || (string)fmt != Format)
                throw new FormatException($"not a drives file (expected format \"{Format}\")");
            var d = new ReplayData
            {
                created = root.TryGetValue("created", out var c) ? (string)c : "",
                tripSet = root.TryGetValue("trip_set", out var ts) ? (string)ts : "",
                targetSpeed = F(root["target_speed"]),
                sampleHz = (int)Math.Round(D(root["sample_hz"])),
            };
            foreach (var x in (List<object>)root["conditions"]) d.conditions.Add((string)x);
            if (root.TryGetValue("perturbation", out var pert) && pert is Dictionary<string, object> pd)
                foreach (var kv in pd) d.perturbation[kv.Key] = F(kv.Value);
            var campus = (Dictionary<string, object>)root["campus"];
            d.roadWidth = F(campus["road_width"]);
            foreach (var kv in (Dictionary<string, object>)campus["car"]) d.car[kv.Key] = F(kv.Value);

            var tripTable = (Dictionary<string, object>)root["trips"];
            var runs = (Dictionary<string, object>)root["runs"];
            // One route and car per trip, shared by every policy's drive.
            var shared = new Dictionary<string, Dictionary<int, (Vector2[], Dictionary<string, float>)>>();
            foreach (var cond in d.conditions) shared[cond] = new Dictionary<int, (Vector2[], Dictionary<string, float>)>();
            foreach (Dictionary<string, object> po in (List<object>)root["policies"])
            {
                var p = new Policy { spec = (string)po["spec"], label = (string)po["label"] };
                var byCond = (Dictionary<string, object>)runs[p.spec];
                foreach (var cond in d.conditions)
                {
                    var list = new List<Drive>();
                    var trips = (List<object>)tripTable[cond];
                    if (byCond.TryGetValue(cond, out var rs))
                    {
                        var rl = (List<object>)rs;
                        for (int k = 0; k < rl.Count; k++)
                            list.Add(rl[k] is Dictionary<string, object> r
                                ? ParseDrive(r, (Dictionary<string, object>)trips[k], shared[cond], k) : null);
                    }
                    p.runs[cond] = list;
                }
                d.policies.Add(p);
            }
            return d;
        }

        private static Drive ParseDrive(Dictionary<string, object> r, Dictionary<string, object> trip,
                                        Dictionary<int, (Vector2[], Dictionary<string, float>)> shared, int k)
        {
            var dr = new Drive
            {
                status = (string)r["status"],
                from = (string)trip["from"],
                to = (string)trip["to"],
                tripSeed = (int)Math.Round(D(trip["seed"])),
                until = trip.TryGetValue("until", out var u) ? F(u) : 0f,
                wallContacts = (int)Math.Round(D(r["wall_contacts"])),
                time = F(r["time"]),
                ret = F(r["return"]),
                offsetMean = F(r["offset_mean"]),
                energyWh = F(r["energy_wh"]),
                distance = F(r["s"]),
            };
            if (!shared.TryGetValue(k, out var rc))
            {
                var route = (List<object>)trip["route"];
                var pts = new Vector2[route.Count];
                for (int i = 0; i < route.Count; i++)
                {
                    var q = (List<object>)route[i];
                    pts[i] = new Vector2(F(q[0]), F(q[1]));
                }
                var car = new Dictionary<string, float>();
                foreach (var kv in (Dictionary<string, object>)trip["car"]) car[kv.Key] = F(kv.Value);
                shared[k] = rc = (pts, car);
            }
            (dr.route, dr.car) = rc;
            foreach (List<object> h in (List<object>)r["hits"]) dr.hits.Add(new Vector3(F(h[0]), F(h[1]), F(h[2])));
            if (r["perturbed"] is List<object> pt) dr.perturbed = new Vector3(F(pt[0]), F(pt[1]), F(pt[2]));
            var frames = (List<object>)r["frames"];
            dr.samples = new Sample[frames.Count];
            dr.odometer = new float[frames.Count];
            for (int i = 0; i < frames.Count; i++)
            {
                var f = (List<object>)frames[i];
                dr.samples[i] = new Sample
                {
                    x = F(f[0]), y = F(f[1]), yaw = F(f[2]), speed = F(f[3]),
                    steer = F(f[4]), throttle = F(f[5]), offset = F(f[6]),
                };
                if (i > 0)
                    dr.odometer[i] = dr.odometer[i - 1] + Vector2.Distance(
                        new Vector2(dr.samples[i].x, dr.samples[i].y), new Vector2(dr.samples[i - 1].x, dr.samples[i - 1].y));
            }
            if (dr.samples.Length == 0) throw new FormatException("a drive with no samples");
            return dr;
        }

        /// <summary>Time of sample i: every 1/hz, except the last, which is
        /// where the drive ended.</summary>
        public static float SampleTime(Drive dr, int i, int hz) =>
            i >= dr.samples.Length - 1 ? dr.time : (float)i / hz;

        /// <summary>The car at time t, interpolated between samples; after the
        /// drive ended, where it ended.</summary>
        public static Pose At(Drive dr, float t, int hz)
        {
            int n = dr.samples.Length;
            if (t >= dr.time || n == 1) return Of(dr, n - 1, n - 1, 0f, t >= dr.time);
            if (t <= 0f) return Of(dr, 0, 0, 0f, false);
            int i0 = Math.Min(n - 2, (int)Math.Floor(t * hz));
            float t0 = SampleTime(dr, i0, hz), t1 = SampleTime(dr, i0 + 1, hz);
            float a = t1 > t0 ? Mathf.Clamp01((t - t0) / (t1 - t0)) : 1f;
            return Of(dr, i0, i0 + 1, a, false);
        }

        /// <summary>Index of the last sample at or before time t.</summary>
        public static int SampleIndex(Drive dr, float t, int hz)
        {
            if (t >= dr.time) return dr.samples.Length - 1;
            return Mathf.Clamp((int)Math.Floor(t * hz), 0, dr.samples.Length - 1);
        }

        private static Pose Of(Drive dr, int i0, int i1, float a, bool ended)
        {
            Sample s0 = dr.samples[i0], s1 = dr.samples[i1];
            float dyaw = Mathf.DeltaAngle(s0.yaw * Mathf.Rad2Deg, s1.yaw * Mathf.Rad2Deg) * Mathf.Deg2Rad;
            return new Pose
            {
                x = Mathf.Lerp(s0.x, s1.x, a),
                y = Mathf.Lerp(s0.y, s1.y, a),
                yaw = s0.yaw + a * dyaw,
                speed = Mathf.Lerp(s0.speed, s1.speed, a),
                steer = Mathf.Lerp(s0.steer, s1.steer, a),
                throttle = Mathf.Lerp(s0.throttle, s1.throttle, a),
                offset = Mathf.Lerp(s0.offset, s1.offset, a),
                odometer = Mathf.Lerp(dr.odometer[i0], dr.odometer[i1], a),
                ended = ended,
            };
        }

        /// <summary>The point arc length s along a polyline (clamped to its ends).</summary>
        public static Vector2 PointAt(Vector2[] line, float s)
        {
            if (line.Length == 0) return Vector2.zero;
            for (int i = 1; i < line.Length; i++)
            {
                float l = Vector2.Distance(line[i - 1], line[i]);
                if (s <= l) return l > 0f ? Vector2.Lerp(line[i - 1], line[i], s / l) : line[i];
                s -= l;
            }
            return line[line.Length - 1];
        }

        /// <summary>Where to read drives from, first found wins: a
        /// "-drives path" argument, drives.json beside the player, the
        /// repository's results/drives.json (the editor), StreamingAssets.</summary>
        public static string Find(string[] args, string playerDir, string repoResults, string streaming)
        {
            for (int i = 0; i + 1 < args.Length; i++)
                if (args[i] == "-drives" && File.Exists(args[i + 1])) return Path.GetFullPath(args[i + 1]);
            foreach (var dir in new[] { playerDir, repoResults, streaming })
            {
                if (string.IsNullOrEmpty(dir)) continue;
                string p = Path.Combine(dir, "drives.json");
                if (File.Exists(p)) return Path.GetFullPath(p);
            }
            return null;
        }

        private static double D(object o) => Convert.ToDouble(o, CultureInfo.InvariantCulture);
        private static float F(object o) => (float)D(o);
    }
}
