#!/usr/bin/env python3
"""Export a trained policy (sim_py/train_rl.py) for the Unity twin.

Writes one JSON file the C# PolicyDriver (unity/.../Runtime/PolicyDriver.cs)
runs with no ML runtime: the observation normalisation, the actor network's
layers (weights, biases, activations), how its output becomes an action, and
for a residual policy the controller it corrects. Plus a check set: inputs
recorded from the policy driving in Python and the actions it chose, which
Unity's Edit Mode tests replay through the C# driver.

    .venv/Scripts/python tools/export_policy.py policies/study/ppo-res-s0 --out policies/release/ppo-res.json

PPO and residual PPO (MlpPolicy: tanh layers, a linear head, clipped) and
SAC and residual SAC (ReLU layers, a linear head, squashed by tanh) are
supported; recurrent policies are not.
"""

from __future__ import annotations

import argparse
import json
import os
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim_py"))

FORMAT = "rc-physics-twin policy 0.1.0"


def layers_of(model, algo: str):
    """The deterministic actor as [(W, b, activation)], last layer first-to-last."""
    sd = {k: v.detach().cpu().double().numpy() for k, v in model.policy.state_dict().items()}
    if algo == "ppo":
        net = model.policy.mlp_extractor.policy_net
        act = type(model.policy.activation_fn()).__name__.lower()       # tanh by default
        hidden = sorted({int(k.split(".")[2]) for k in sd if k.startswith("mlp_extractor.policy_net.")})
        out = [(sd[f"mlp_extractor.policy_net.{i}.weight"], sd[f"mlp_extractor.policy_net.{i}.bias"], act) for i in hidden]
        out.append((sd["action_net.weight"], sd["action_net.bias"], "linear"))
        return out, "clip"
    if algo == "sac":
        act = type(model.actor.activation_fn()).__name__.lower() if hasattr(model.actor, "activation_fn") else "relu"
        hidden = sorted({int(k.split(".")[2]) for k in sd if k.startswith("actor.latent_pi.")})
        out = [(sd[f"actor.latent_pi.{i}.weight"], sd[f"actor.latent_pi.{i}.bias"], act) for i in hidden]
        out.append((sd["actor.mu.weight"], sd["actor.mu.bias"], "linear"))
        return out, "tanh"
    raise ValueError(f"{algo}: only ppo and sac policies export")


def export(prefix: pathlib.Path, lib: str, check_ticks: int = 60) -> dict:
    import pickle
    import numpy as np
    import train_rl as rl
    meta = json.loads(prefix.with_suffix(".json").read_text(encoding="utf-8"))
    cfg = meta["config"]
    model = rl.load_model(prefix, meta["algo"])
    with open(prefix.with_suffix(".vecnormalize.pkl"), "rb") as f:
        vn = pickle.load(f)
    layers, head = layers_of(model, meta["algo"])
    tuned = json.loads((ROOT / "policies" / "controllers.json").read_text(encoding="utf-8"))
    doc = {
        "format": FORMAT, "name": prefix.name, "algo": meta["algo"], "selected": meta.get("selected"),
        "task": cfg.get("task", "follow"), "target_speed": cfg["target_speed"], "history": cfg["history"],
        "residual": ({"kind": cfg["residual"], "scale": cfg["residual_scale"],
                      "gains": tuned[cfg["residual"]]["gains"]} if cfg["residual"] else None),
        "normalise": {"mean": vn.obs_rms.mean.tolist(), "var": vn.obs_rms.var.tolist(),
                      "epsilon": float(vn.epsilon), "clip": float(vn.clip_obs)},
        "layers": [{"weight": W.tolist(), "bias": b.tolist(), "activation": a} for W, b, a in layers],
        "head": head,
        "note": "network in float32 as trained; the C# driver computes in float32 too",
    }
    # The check set: the policy driving one campus trip in Python.
    c = rl._campus()
    env = c.CampusEnv(seed=0, ranges=c.envmod.RANGES_NOMINAL, lib_path=lib, target_speed=cfg["target_speed"],
                      history=cfg["history"], task=cfg.get("task", "follow"))
    policy = rl.load_policy(prefix, env)
    obs, _ = env.reset(seed=0, options={"trip": ("AB2", "S3")})
    checks = []
    for _ in range(check_ticks):
        a = policy(obs)
        checks.append({"obs": list(obs), "action": [float(a[0]), float(a[1])]})
        obs, r, te, tr, info = env.step(a)
        if te or tr:
            break
    env.close()
    doc["checks"] = checks
    return doc


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("prefix", type=pathlib.Path, help="a train_rl.py model, without extension")
    ap.add_argument("--out", type=pathlib.Path, required=True)
    ap.add_argument("--lib")
    args = ap.parse_args(argv)
    lib = os.path.abspath(args.lib) if args.lib else (os.environ.get("RC_TWIN_LIB") or str(ROOT / "build" / "bin" / "rc_twin.dll"))
    doc = export(args.prefix, lib)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(doc, separators=(",", ":")) + "\n", encoding="utf-8")
    n = sum(len(l["bias"]) * (len(l["weight"][0]) + 1) for l in doc["layers"])
    print(f"wrote {args.out}: {doc['algo']}{' residual on ' + doc['residual']['kind'] if doc['residual'] else ''}, "
          f"{len(doc['layers'])} layers, {n} parameters, {len(doc['checks'])} check ticks")
    return 0


if __name__ == "__main__":
    sys.exit(main())
