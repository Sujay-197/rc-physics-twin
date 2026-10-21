"""A stable-baselines3 VecEnv that runs several environments per worker
process: workers x per_worker environments, one message per worker per step.

SB3's SubprocVecEnv runs one environment per process and exchanges a message
per environment per step. Our environment steps in tens of microseconds, so
on Windows the pipe round trip, not the physics, sets the pace. Here each
worker steps its own DummyVecEnv of per_worker environments (with SB3's own
auto-reset and terminal-observation handling) and answers for all of them at
once.
"""

from __future__ import annotations

import multiprocessing as mp
import traceback

import numpy as np
from stable_baselines3.common.vec_env import DummyVecEnv, VecEnv
from stable_baselines3.common.vec_env.base_vec_env import CloudpickleWrapper


def _worker(remote, parent, fns_wrapper):
    parent.close()
    try:
        venv = DummyVecEnv(fns_wrapper.var)
    except Exception:
        remote.send(_Error(traceback.format_exc()))
        return
    while True:
        try:
            cmd, data = remote.recv()
        except (EOFError, KeyboardInterrupt):
            return
        try:
            if cmd == "step":
                obs, rews, dones, infos = venv.step(data)
                remote.send((obs, rews, dones, infos))
            elif cmd == "reset":
                venv._seeds = data
                obs = venv.reset()
                remote.send((obs, venv.reset_infos))
            elif cmd == "get_attr":
                remote.send(venv.get_attr(*data))
            elif cmd == "set_attr":
                remote.send(venv.set_attr(*data))
            elif cmd == "env_method":
                name, args, kwargs, idx = data
                remote.send(venv.env_method(name, *args, indices=idx, **kwargs))
            elif cmd == "is_wrapped":
                remote.send(venv.env_is_wrapped(*data))
            elif cmd == "spaces":
                remote.send((venv.observation_space, venv.action_space))
            elif cmd == "close":
                venv.close()
                remote.close()
                return
        except Exception:
            # Back to the parent, which raises it: a dead worker would only
            # show up there as a broken pipe.
            remote.send(_Error(traceback.format_exc()))


class _Error:
    def __init__(self, text: str):
        self.text = text


def _recv(remote):
    out = remote.recv()
    if isinstance(out, _Error):
        raise RuntimeError("environment worker failed:\n" + out.text)
    return out


class BatchedSubprocVecEnv(VecEnv):
    def __init__(self, env_fns, per_worker: int, start_method: str = "spawn"):
        n = len(env_fns)
        groups = [env_fns[i:i + per_worker] for i in range(0, n, per_worker)]
        self.sizes = [len(g) for g in groups]
        ctx = mp.get_context(start_method)
        self.remotes, work_remotes = zip(*[ctx.Pipe() for _ in groups])
        self.processes = []
        for wr, r, g in zip(work_remotes, self.remotes, groups):
            p = ctx.Process(target=_worker, args=(wr, r, CloudpickleWrapper(g)), daemon=True)
            p.start()
            self.processes.append(p)
            wr.close()
        self.remotes[0].send(("spaces", None))
        obs_space, act_space = _recv(self.remotes[0])
        self.closed = False
        super().__init__(n, obs_space, act_space)

    # Which worker and local index each global env index maps to.
    def _split(self, indices):
        idx = range(self.num_envs) if indices is None else ([indices] if isinstance(indices, int) else indices)
        out, base = [], 0
        for w, size in enumerate(self.sizes):
            local = [i - base for i in idx if base <= i < base + size]
            if local:
                out.append((w, local))
            base += size
        return out

    def step_async(self, actions):
        base = 0
        for remote, size in zip(self.remotes, self.sizes):
            remote.send(("step", actions[base:base + size]))
            base += size

    def step_wait(self):
        obs, rews, dones, infos = [], [], [], []
        for remote in self.remotes:
            o, r, d, i = _recv(remote)
            obs.append(o)
            rews.append(r)
            dones.append(d)
            infos.extend(i)
        return np.concatenate(obs), np.concatenate(rews), np.concatenate(dones), infos

    def reset(self):
        base = 0
        for remote, size in zip(self.remotes, self.sizes):
            remote.send(("reset", self._seeds[base:base + size]))
            base += size
        obs, self.reset_infos = [], []
        for remote in self.remotes:
            o, infos = _recv(remote)
            obs.append(o)
            self.reset_infos.extend(infos)
        self._reset_seeds()
        return np.concatenate(obs)

    def close(self):
        if self.closed:
            return
        for remote in self.remotes:
            try:
                remote.send(("close", None))
            except (BrokenPipeError, OSError):
                pass   # that worker already ended
        for p in self.processes:
            p.join(timeout=10)
            if p.is_alive():
                p.terminate()
        self.closed = True

    def get_attr(self, attr_name, indices=None):
        out = []
        for w, local in self._split(indices):
            self.remotes[w].send(("get_attr", (attr_name, local)))
            out.extend(_recv(self.remotes[w]))
        return out

    def set_attr(self, attr_name, value, indices=None):
        for w, local in self._split(indices):
            self.remotes[w].send(("set_attr", (attr_name, value, local)))
            _recv(self.remotes[w])

    def env_method(self, method_name, *method_args, indices=None, **method_kwargs):
        out = []
        for w, local in self._split(indices):
            self.remotes[w].send(("env_method", (method_name, method_args, method_kwargs, local)))
            out.extend(_recv(self.remotes[w]))
        return out

    def env_is_wrapped(self, wrapper_class, indices=None):
        out = []
        for w, local in self._split(indices):
            self.remotes[w].send(("is_wrapped", (wrapper_class, local)))
            out.extend(_recv(self.remotes[w]))
        return out
