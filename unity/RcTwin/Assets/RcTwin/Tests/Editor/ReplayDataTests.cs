// Edit Mode: the policy replay's drives file (tools/record_drives.py) parses,
// and poses interpolate between its 25 Hz samples the way PolicyReplay
// plays them.
using System;
using System.IO;
using NUnit.Framework;
using UnityEngine;

namespace RcTwin.Tests
{
    public class ReplayDataTests
    {
        // Two policies, one condition, two trips. Trip 0, S1 to AB4: a 0.1 s
        // drive sampled at 25 Hz (t 0, 0.04, 0.08) that ends at 0.1 s, and a
        // left-road drive. Trip 1, AB4 to S1: pure pursuit only.
        private const string Sample = @"{""format"":""rc-physics-twin drives 0.2.0"",""created"":""2026-10-20 15:00"",
 ""trip_set"":""pairs"",""target_speed"":4.0,""sample_hz"":25,""conditions"":[""train""],
 ""perturbation"":{""at_s"":4.0,""friction"":0.6},
 ""policies"":[{""spec"":""controller:pure_pursuit"",""label"":""pure pursuit""},{""spec"":""sb3:x"",""label"":""PPO""}],
 ""campus"":{""road_width"":1.409,""car"":{""half_length"":0.21,""wheel_radius"":0.05}},
 ""trips"":{""train"":[
   {""from"":""S1"",""to"":""AB4"",""seed"":50000,""until"":3.1,""car"":{""friction"":0.9},""route"":[[0,0],[3,0],[3,4]]},
   {""from"":""AB4"",""to"":""S1"",""seed"":50001,""car"":{""friction"":1.1},""route"":[[3,4],[3,0],[0,0]]}]},
 ""runs"":{
  ""controller:pure_pursuit"":{""train"":[{""status"":""reached_end"",""time"":0.1,""return"":1.5,""offset_mean"":0.01,
    ""wall_contacts"":0,""energy_wh"":0.001,""s"":0.4,""hits"":[],""perturbed"":null,
    ""frames"":[[0,0,0,4,0,0,0],[0.16,0,0.1,4,0.5,1,0.01],[0.32,0,0.2,4,0,0,0],[0.4,0,3.1,4,0,0,0.02]]},
   {""status"":""timeout"",""time"":120,""return"":-50,""offset_mean"":0.2,""wall_contacts"":1,""energy_wh"":0,""s"":1,
    ""hits"":[],""perturbed"":null,""frames"":[[3,4,0,4,0,0,0],[3,3,0,0,0,1,0]]}]},
  ""sb3:x"":{""train"":[{""status"":""left_road"",""time"":0.04,""return"":-9,""offset_mean"":0.3,
    ""wall_contacts"":2,""energy_wh"":0,""s"":0.1,""hits"":[[0.04,0.1,0.6]],""perturbed"":[0.02,0.05,0],
    ""frames"":[[0,0,0,4,0,0,0],[0.1,0.05,0,4,1,1,0.3]]},null]}}}";

        [Test]
        public void ParsesEveryField()
        {
            var d = ReplayData.Parse(Sample);
            Assert.AreEqual(4f, d.targetSpeed);
            Assert.AreEqual(25, d.sampleHz);
            Assert.AreEqual("pairs", d.tripSet);
            Assert.AreEqual(2, d.policies.Count);
            Assert.AreEqual("PPO", d.policies[1].label);
            Assert.AreEqual(2, d.TripCount("train"));
            Assert.AreEqual(0.6f, d.perturbation["friction"]);
            var pp = d.Get(0, "train", 0);
            Assert.IsTrue(pp.Finished);
            Assert.AreEqual(("S1", "AB4"), (pp.from, pp.to));
            Assert.AreEqual(50000, pp.tripSeed);
            Assert.AreEqual(3.1f, pp.until);
            Assert.AreEqual(0f, d.Get(0, "train", 1).until, "absent: the replay works it out");
            Assert.AreEqual(0.9f, pp.car["friction"]);
            Assert.AreEqual(4, pp.samples.Length);
            Assert.AreEqual(3, pp.route.Length);
            Assert.IsNull(pp.perturbed);
            Assert.AreEqual(0.4f, pp.odometer[3], 1e-6f);
            var ppo = d.Get(1, "train", 0);
            Assert.IsFalse(ppo.Finished);
            Assert.AreSame(pp.route, ppo.route, "a trip's route is stored once for every policy");
            Assert.AreSame(pp.car, ppo.car);
            Assert.AreEqual(new Vector3(0.04f, 0.1f, 0.6f), ppo.hits[0]);
            Assert.AreEqual(0.02f, ppo.perturbed.Value.x, 1e-6f);
            Assert.AreEqual(("AB4", "S1"), (d.Get(0, "train", 1).from, d.Get(0, "train", 1).to));
            Assert.AreEqual(1.1f, d.Get(0, "train", 1).car["friction"]);
            Assert.IsNull(d.Get(1, "train", 1), "a policy with no drive on a trip");
            Assert.IsNull(d.Get(1, "train", 2));
        }

        [Test]
        public void FindsRoutesByStops()
        {
            var d = ReplayData.Parse(Sample);
            Assert.AreEqual(0, d.FindTrip("train", "S1", "AB4"));
            Assert.AreEqual(1, d.FindTrip("train", "AB4", "S1"));
            Assert.AreEqual(-1, d.FindTrip("train", "S1", "S2"));
            Assert.AreEqual(-1, d.FindTrip("held_out", "S1", "AB4"));
            CollectionAssert.AreEqual(new[] { "AB4", "S1" }, d.Stops("train"));
            var names = new System.Collections.Generic.List<string> { "S10", "S2", "AB1", "S1", "AB12", "AB2" };
            names.Sort(ReplayData.CompareStops);
            CollectionAssert.AreEqual(new[] { "AB1", "AB2", "AB12", "S1", "S2", "S10" }, names);
        }

        [Test]
        public void RejectsOtherFiles()
        {
            Assert.Throws<FormatException>(() => ReplayData.Parse(@"{""format"":""something else""}"));
        }

        [Test]
        public void PosesInterpolateAndHoldAtTheEnd()
        {
            var dr = ReplayData.Parse(Sample).Get(0, "train", 0);
            var p = ReplayData.At(dr, 0.02f, 25);          // half way from sample 0 to 1
            Assert.AreEqual(0.08f, p.x, 1e-5f);
            Assert.AreEqual(0.05f, p.yaw, 1e-5f);
            Assert.AreEqual(0.25f, p.steer, 1e-5f);
            Assert.IsFalse(p.ended);
            // The last sample is at the drive's end, 0.1 s, not 0.12 s.
            Assert.AreEqual(0.1f, ReplayData.SampleTime(dr, 3, 25));
            p = ReplayData.At(dr, 0.09f, 25);
            Assert.AreEqual(0.36f, p.x, 1e-5f);
            // 0.2 to 3.1 rad is less than pi apart: the plain difference.
            Assert.AreEqual(0.2f + 0.5f * 2.9f, p.yaw, 1e-4f);
            p = ReplayData.At(dr, 5f, 25);
            Assert.IsTrue(p.ended);
            Assert.AreEqual(0.4f, p.x, 1e-6f);
            Assert.AreEqual(3, ReplayData.SampleIndex(dr, 5f, 25));
            Assert.AreEqual(1, ReplayData.SampleIndex(dr, 0.05f, 25));
        }

        [Test]
        public void YawWrapsTheShortWay()
        {
            var dr = new ReplayData.Drive
            {
                time = 0.04f,
                samples = new[] { new ReplayData.Sample { yaw = 3.1f }, new ReplayData.Sample { yaw = -3.1f } },
                odometer = new float[2],
            };
            var p = ReplayData.At(dr, 0.02f, 25);
            Assert.AreEqual(Mathf.PI, Mathf.Abs(p.yaw), 1e-3f, "through pi, not through zero");
        }

        [Test]
        public void PointsAlongTheRoute()
        {
            var line = new[] { Vector2.zero, new Vector2(3f, 0f), new Vector2(3f, 4f) };
            Assert.AreEqual(new Vector2(2f, 0f), ReplayData.PointAt(line, 2f));
            Assert.AreEqual(new Vector2(3f, 1f), ReplayData.PointAt(line, 4f));
            Assert.AreEqual(new Vector2(3f, 4f), ReplayData.PointAt(line, 99f));
        }

        [Test]
        public void FindsDrivesInOrder()
        {
            string root = Path.Combine(Path.GetTempPath(), "rc_twin_replay_find");
            string a = Path.Combine(root, "a"), b = Path.Combine(root, "b"), arg = Path.Combine(root, "arg.json");
            Directory.CreateDirectory(a);
            Directory.CreateDirectory(b);
            File.WriteAllText(Path.Combine(b, "drives.json"), "{}");
            File.WriteAllText(arg, "{}");
            try
            {
                Assert.AreEqual(Path.Combine(b, "drives.json"), ReplayData.Find(new string[0], a, b, null));
                Assert.AreEqual(arg, ReplayData.Find(new[] { "x.exe", "-drives", arg }, a, b, null));
                File.WriteAllText(Path.Combine(a, "drives.json"), "{}");
                Assert.AreEqual(Path.Combine(a, "drives.json"), ReplayData.Find(new string[0], a, b, null));
                Assert.IsNull(ReplayData.Find(new string[0], null, Path.Combine(root, "none"), null));
            }
            finally
            {
                Directory.Delete(root, true);
            }
        }
    }
}
