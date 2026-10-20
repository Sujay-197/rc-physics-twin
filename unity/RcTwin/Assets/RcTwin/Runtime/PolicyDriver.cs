// A trained policy as a driver: the network tools/export_policy.py writes
// out of a sim_py/train_rl.py model, run here in float32 as it trained, with
// no ML runtime. The observation is normalised as in training, the actor's
// layers run, and the head makes an action (PPO: clipped; SAC: tanh). A
// residual policy adds its scaled output to the tuned pure pursuit it was
// trained on. tests/Editor/PolicyDriverTests.cs replays the export's own
// check set, recorded from the policy driving in Python.
//
// Free of UnityEngine, like RcTwinNative.
using System;
using System.Collections.Generic;
using System.Globalization;

namespace RcTwin
{
    public sealed class PolicyDriver : IDriver
    {
        public const string Format = "rc-physics-twin policy 0.1.0";

        private sealed class Layer
        {
            public float[,] w;    // out x in
            public float[] b;
            public string activation;
        }

        private readonly List<Layer> _layers = new List<Layer>();
        private double[] _mean, _var;
        private double _eps, _clip;
        private bool _tanhHead;
        private PurePursuitDriver _base;
        private double _residualScale;

        public string PolicyName { get; private set; }
        public string Algo { get; private set; }
        public string Task { get; private set; }
        public double TargetSpeed { get; private set; }
        public int History { get; private set; }
        public int InputSize => _mean.Length;
        public bool Residual => _base != null;
        public string Name => $"learned: {PolicyName} ({Algo}{(Residual ? ", residual on pure pursuit" : "")})";

        public void Reset() { }

        /// <summary>A policy from tools/export_policy.py's JSON, for the nominal car
        /// (the residual's pure pursuit uses its geometry).</summary>
        public static PolicyDriver Load(string json, in RcTwinNative.Params nominal)
        {
            var d = (Dictionary<string, object>)MiniJson.Parse(json);
            if (!d.TryGetValue("format", out var f) || (string)f != Format)
                throw new FormatException($"not a policy export (expected format \"{Format}\")");
            var p = new PolicyDriver
            {
                PolicyName = (string)d["name"], Algo = (string)d["algo"], Task = (string)d["task"],
                TargetSpeed = D(d["target_speed"]), History = (int)Math.Round(D(d["history"])),
                _tanhHead = (string)d["head"] == "tanh",
            };
            var n = (Dictionary<string, object>)d["normalise"];
            p._mean = Arr(n["mean"]);
            p._var = Arr(n["var"]);
            p._eps = D(n["epsilon"]);
            p._clip = D(n["clip"]);
            foreach (Dictionary<string, object> l in (List<object>)d["layers"])
            {
                var rows = (List<object>)l["weight"];
                int outN = rows.Count, inN = ((List<object>)rows[0]).Count;
                var w = new float[outN, inN];
                for (int i = 0; i < outN; i++)
                {
                    var row = (List<object>)rows[i];
                    for (int j = 0; j < inN; j++) w[i, j] = (float)D(row[j]);
                }
                var b = Arr(l["bias"]);
                var fb = new float[b.Length];
                for (int i = 0; i < b.Length; i++) fb[i] = (float)b[i];
                p._layers.Add(new Layer { w = w, b = fb, activation = (string)l["activation"] });
            }
            if (d["residual"] is Dictionary<string, object> r)
            {
                if ((string)r["kind"] != "pure_pursuit")
                    throw new NotSupportedException($"a residual on {r["kind"]}: only pure pursuit is ported to C#");
                var g = (Dictionary<string, object>)r["gains"];
                p._base = PurePursuitDriver.Tuned(nominal, p.TargetSpeed, D(g["lookahead"]), D(g["k_speed"]), D(g["a_lat"]),
                                                  stopTask: p.Task == "stop");
                p._residualScale = D(r["scale"]);
            }
            return p;
        }

        public (double steer, double throttle) Act(in DriveContext c)
        {
            var o = c.Observation;
            (double steer, double throttle) b = default;
            int n = o.Length + (Residual ? 2 : 0);
            if (n != _mean.Length)
                throw new ArgumentException($"{PolicyName} takes {_mean.Length} inputs; the observation gives {n}");
            if (Residual) b = _base.Act(c);
            // Normalised in float64, then float32, as LoadedPolicy (train_rl.py) does.
            var x = new float[n];
            for (int i = 0; i < n; i++)
            {
                double v = i < o.Length ? o[i] : (i == o.Length ? b.steer : b.throttle);
                double z = (v - _mean[i]) / Math.Sqrt(_var[i] + _eps);
                x[i] = (float)(z < -_clip ? -_clip : z > _clip ? _clip : z);
            }
            foreach (var l in _layers)
            {
                int outN = l.b.Length, inN = x.Length;
                var y = new float[outN];
                for (int i = 0; i < outN; i++)
                {
                    float sum = l.b[i];
                    for (int j = 0; j < inN; j++) sum += l.w[i, j] * x[j];
                    y[i] = l.activation == "tanh" ? (float)Math.Tanh(sum)
                         : l.activation == "relu" ? Math.Max(0f, sum) : sum;
                }
                x = y;
            }
            double a0 = _tanhHead ? Math.Tanh(x[0]) : DriverMath.Clip(x[0]);
            double a1 = _tanhHead ? Math.Tanh(x[1]) : DriverMath.Clip(x[1]);
            if (Residual)
                return (DriverMath.Clip(b.steer + _residualScale * a0), DriverMath.Clip(b.throttle + _residualScale * a1));
            return (a0, a1);
        }

        private static double D(object o) => Convert.ToDouble(o, CultureInfo.InvariantCulture);

        private static double[] Arr(object o)
        {
            var l = (List<object>)o;
            var a = new double[l.Count];
            for (int i = 0; i < a.Length; i++) a[i] = D(l[i]);
            return a;
        }
    }
}
