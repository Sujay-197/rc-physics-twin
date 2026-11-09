#!/usr/bin/env python3
"""Cross-check the twin's vehicle dynamics against independent references.

The question this answers: is the Core only self-consistent, or does it
agree with vehicle dynamics that someone else wrote down? It drives the Core
through rc_twin (L0, L1, L2, and L2 with the Unity car's options) and three
independent references through the same manoeuvres, from the same initial
state and with the same steering input, and reports how far apart they are.

References (Docs/validation.md has the citations and the parameter mapping):

  analytic     Linear single-track ("bicycle") theory, closed form: yaw-rate
               gain, understeer gradient, natural frequency and damping of the
               yaw response, from the twin's mass, yaw inertia, CoG position
               and axle cornering stiffnesses (Rajamani 2012, ch. 2 and 3;
               Pacejka 2012, ch. 1). Transients are the exact response of the
               linear system (scipy.signal.lsim), with and without a
               first-order tyre relaxation length.
  CommonRoad   The CommonRoad vehicle models (Althoff and Wuersching 2020),
               package commonroad-vehicle-models 3.0.2 (BSD-3-Clause),
               installed, not vendored: kinematic single-track (KS),
               single-track with linear tyres (ST) and single-track drift with
               Pacejka tyres and wheel spin (STD). The package's own dynamics
               functions are called unchanged; this script only integrates
               them (RK4) and maps the twin's parameters into them.
  kinematic    v tan(delta) / L, for the low-speed case.

The multi-body model (MB) is not run: it needs about 40 parameters (sprung
and unsprung masses, roll-axis heights, suspension and tyre compliances)
that the twin does not have and no cited source gives at 1/10 scale.

Manoeuvres (constants below):
  speed sweep   steady-state circular driving at a small steer, 1 to 10 m/s:
                yaw-rate gain against speed
  understeer    steady-state circular driving at 4 m/s, steer swept to the
                limit (ISO 4138, constant speed, variable steer)
  step steer    5 m/s, linear (0.03 rad) and nonlinear (0.10 rad) step
                (after ISO 7401)
  sine steer    5 m/s, 0.03 rad at 1 Hz, three periods
  low speed     0.2 and 0.4 m/s at full lock: the kinematic blend of ADR 0011

Inputs: every model holds its own body-x speed with the same PI controller at
100 Hz; steering is the twin's achieved road-wheel angle, recorded every 1 ms
and fed to the references (through the steering-rate input CommonRoad takes),
so the servo of L2 is not counted as a chassis disagreement. Core steps 1 ms.

Usage (the validation venv, see sim_py/requirements-validation.txt):
    .venv-validation/Scripts/python tools/cross_check.py

Writes results/reference/cross_check.json and the figures
results/reference/cross_check_{steady,transient}.{pdf,png}. Deterministic: the
same build gives the same file. Nothing here is a measured car: every number
is model against model.
"""

from __future__ import annotations

import argparse
import copy
import ctypes
import datetime
import importlib.metadata
import importlib.util
import json
import math
import pathlib
import platform
import subprocess
import sys

import numpy as np
from scipy import signal

from vehiclemodels.vehicle_parameters import setup_vehicle_parameters
from vehiclemodels.vehicle_dynamics_ks import vehicle_dynamics_ks
from vehiclemodels.vehicle_dynamics_st import vehicle_dynamics_st
from vehiclemodels.vehicle_dynamics_std import vehicle_dynamics_std

ROOT = pathlib.Path(__file__).resolve().parent.parent


def _load(name: str, path: pathlib.Path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


dr = _load("drive_road", ROOT / "tools" / "drive_road.py")
env = _load("rc_twin_env", ROOT / "sim_py" / "rc_twin_env.py")

DT = 1e-3                 # Core step and recording interval [s]
CTRL_EVERY = 10           # speed controller at 100 Hz
KP, KI = 4.0, 4.0         # speed PI gains [1/s], [1/s^2]; the same for every model
G_CR = 9.81               # CommonRoad's gravity (vehicle_dynamics_st.py) [m/s^2]

SPEED_SWEEP = {"steer": 0.01, "speeds": [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 10.0]}
UNDERSTEER = {"speed": 4.0, "steers": [0.01, 0.02, 0.04, 0.06, 0.08, 0.10, 0.12, 0.14, 0.16,
                                       0.18, 0.20, 0.22, 0.24, 0.27, 0.30, 0.35, 0.40]}
STEADY_RAMP, STEADY_HOLD, STEADY_AVG = 1.0, 3.0, 1.0   # [s]
STEADY_TOL = 0.01         # steady: yaw-rate spread over the last STEADY_AVG under 1 % of its mean
LINEAR_AY = 4.0           # "linear range" for the summary: |a_y| below this [m/s^2], about 0.37 g
STEP = {"speed": 5.0, "steers": [0.03, 0.10], "pre": 1.5, "after": 3.0}
SINE = {"speed": 5.0, "amp": 0.03, "freq": 1.0, "periods": 3, "pre": 1.5}
LOW = {"speeds": [0.2, 0.4], "steer": None, "pre": 1.0, "after": 10.0}   # steer None: full lock
PATH_N = 3.0              # path deviation reported this long after the manoeuvre starts [s]

CONFIGS = ("L1", "L2", "L2-unity")   # twin configurations compared against every reference
CONFIGS_EQ = ("L1-eqC", "L2-eqC")    # the same with equal axle stiffness (CommonRoad's mapping)


# ----------------------------------------------------------------- the twin

def twin_params(lib, config: str) -> "dr.Params":
    """SlipX's provisional defaults, through rc_twin; "unity" adds TwinCar's
    options (sim_py/rc_twin_env.py UNITY_CAR); "eqC" sets both axles to the
    mean cornering stiffness, the one CommonRoad 3.0.2 can represent."""
    p = dr.Params()
    lib.rc_twin_params_default(ctypes.byref(p))
    if config.endswith("unity"):
        u = env.unity_car(p.mass)
        p.wheel_inertia, p.ackermann = u["wheel_inertia"], u["ackermann"]
        p.wheel_rate_front = p.wheel_rate_rear = u["wheel_rate"]
        p.damping_front = p.damping_rear = u["damping"]
        p.suspension_travel, p.v_kinematic = u["travel"], u["v_kinematic"]
    if config.endswith("eqC"):
        c = 0.5 * (p.c_alpha_f + p.c_alpha_r)
        p.c_alpha_f = p.c_alpha_r = c
    return p


def tier_of(config: str) -> int:
    return {"L0": 0, "L1": 1}.get(config.split("-")[0], 2)


class PI:
    def __init__(self):
        self.i = 0.0

    def __call__(self, err: float) -> float:
        self.i += err * DT * CTRL_EVERY
        return KP * err + KI * self.i


def run_twin(lib, config: str, steer_cmd, v0: float, t_end: float) -> dict:
    """Drive one twin configuration: steer_cmd(t) [rad], speed held at v0."""
    p = twin_params(lib, config)
    why = ctypes.c_char_p()
    car = lib.rc_twin_create(tier_of(config), 0, ctypes.byref(p), ctypes.byref(why))
    if not car:
        raise RuntimeError(f"rc_twin_create {config}: {why.value}")
    w0 = v0 / p.wheel_radius
    st = dr.State(vx=v0, soc=0.8, pack_v=11.1, w_fl=w0, w_fr=w0, w_rl=w0, w_rr=w0)
    lib.rc_twin_reset(car, ctypes.byref(st))
    n = int(round(t_end / DT))
    out = {k: np.empty(n + 1) for k in ("t", "x", "y", "yaw", "vx", "vy", "r", "steer")}
    pi, accel = PI(), 0.0

    def rec(k):
        lib.rc_twin_get_state(car, ctypes.byref(st))
        for key, val in (("t", k * DT), ("x", st.x), ("y", st.y), ("yaw", st.yaw), ("vx", st.vx),
                         ("vy", st.vy), ("r", st.yaw_rate), ("steer", st.steer)):
            out[key][k] = val

    try:
        rec(0)
        for k in range(n):
            if k % CTRL_EVERY == 0:
                accel = pi(v0 - out["vx"][k])
            status = lib.rc_twin_step(car, steer_cmd(k * DT), accel, DT, 1)
            if status != 0:
                raise RuntimeError(f"rc_twin_step {config}: status {status}")
            rec(k + 1)
    finally:
        lib.rc_twin_destroy(car)
    out["yaw"] = np.unwrap(out["yaw"])
    return out


# --------------------------------------------------------------- CommonRoad

def cr_params(p) -> object:
    """The twin's parameters in CommonRoad's VehicleParameters. Every field
    the KS, ST and STD models read is set here from the twin; only the
    combined-slip shape coefficients of the Pacejka tyre (r_bx*, r_cx1,
    r_ex1, r_by*, r_cy1, r_ey1), which MF-lite has no counterpart for, keep
    CommonRoad's defaults (parameters_tire.yaml)."""
    q = setup_vehicle_parameters(1)
    mu = p.tyre_front.mu_y0
    q.a, q.b, q.m, q.I_z = p.lf, p.lr, p.mass, p.izz
    q.h_s = q.h_cg = p.h_cog
    q.R_w = p.wheel_radius
    # STD needs wheel inertia; the bare car's wheels are quasi-static, so the
    # Unity car's per-wheel value, two wheels per axle.
    q.I_y_w = 2.0 * env.UNITY_CAR["wheel_inertia"]
    q.T_sb, q.T_se = 0.0, 0.0   # RWD; the car brakes through its driven axle (l2_double_track.cpp)
    # The input is the twin's achieved steer, already inside its travel, so
    # CommonRoad's own limits are opened: steering_constraints() is evaluated
    # at RK4's stage points, and a limit at exactly steer_max zeroes the
    # last stage of a step that ends there.
    q.steering.min, q.steering.max = -1.5, 1.5
    q.steering.v_min, q.steering.v_max = -1e6, 1e6
    q.longitudinal.a_max, q.longitudinal.v_switch = 1e3, 1e3   # no power limit: the PI drives
    q.longitudinal.v_min, q.longitudinal.v_max = -50.0, 50.0
    # Tyre. CommonRoad 3.0.2 derives one normalised cornering stiffness for
    # both axles, C_S = -p_ky1 / p_dy1 (vehicle_dynamics_st.py); the twin
    # has two. Mapped so the total is kept: C_S mu m g = c_alpha_f + c_alpha_r.
    c_s = (p.c_alpha_f + p.c_alpha_r) / (mu * p.mass * G_CR)
    t = q.tire
    t.p_dy1, t.p_ky1 = mu, -c_s * mu
    t.p_cy1, t.p_ey1 = p.tyre_front.shape_c, p.tyre_front.curvature_e   # MF-lite's C and E
    t.p_dy3 = 0.0
    t.p_hy1 = t.p_hy3 = t.p_vy1 = t.p_vy3 = 0.0    # MF-lite has no shifts
    # Longitudinal: twin c_kappa per tyre at its static load, two per axle.
    fz_axle = p.mass * G_CR * p.lr / (p.lf + p.lr)
    t.p_dx1, t.p_kx1 = p.tyre_front.mu_x0, 2.0 * p.c_kappa / fz_axle
    t.p_cx1, t.p_ex1 = p.tyre_front.shape_c, p.tyre_front.curvature_e
    t.p_dx3 = t.p_hx1 = t.p_vx1 = 0.0
    t.r_hx1 = t.r_hy1 = t.r_vy1 = t.r_vy3 = 0.0     # combined-slip shifts: none in MF-lite
    return q


def cr_mapping(p, q) -> dict:
    t = q.tire
    return {
        "a": q.a, "b": q.b, "m": q.m, "I_z": q.I_z, "h_s": q.h_s, "R_w": q.R_w, "I_y_w": q.I_y_w,
        "T_sb": q.T_sb, "T_se": q.T_se,
        "tire": {k: getattr(t, k) for k in ("p_dy1", "p_ky1", "p_cy1", "p_ey1", "p_dx1", "p_kx1",
                                             "p_cx1", "p_ex1")},
        "C_S_normalised": -t.p_ky1 / t.p_dy1,
        "axle_cornering_stiffness_cr": -t.p_ky1 * q.m * G_CR * q.b / (q.a + q.b),
        "axle_cornering_stiffness_twin": {"front": p.c_alpha_f, "rear": p.c_alpha_r},
        "combined_slip_coefficients": "CommonRoad defaults (parameters_tire.yaml), no MF-lite counterpart",
    }


def rk4(f, x, u, h):
    k1 = np.asarray(f(list(x), u))
    k2 = np.asarray(f(list(x + 0.5 * h * k1), u))
    k3 = np.asarray(f(list(x + 0.5 * h * k2), u))
    k4 = np.asarray(f(list(x + h * k3), u))
    return x + h / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


def run_cr(model: str, q, steer: np.ndarray, v0: float) -> dict:
    """Integrate a CommonRoad model along the steer samples (every DT),
    speed held at v0 with the same PI. Positions are the CoG's."""
    p_l = q.a + q.b
    if model == "KS":
        f, x = vehicle_dynamics_ks, np.array([-q.b, 0.0, 0.0, v0, 0.0])   # rear axle at -lr
    elif model == "ST":
        f, x = vehicle_dynamics_st, np.array([0.0, 0.0, 0.0, v0, 0.0, 0.0, 0.0])
    else:
        f = vehicle_dynamics_std
        x = np.array([0.0, 0.0, 0.0, v0, 0.0, 0.0, 0.0, v0 / q.R_w, v0 / q.R_w])
    n = len(steer) - 1
    out = {k: np.empty(n + 1) for k in ("t", "x", "y", "yaw", "vx", "vy", "r", "steer")}
    pi, accel = PI(), 0.0

    def body(x):
        if model == "KS":
            yaw, v = x[4], x[3]
            return (x[0] + q.b * math.cos(yaw), x[1] + q.b * math.sin(yaw), yaw, v,
                    v * math.tan(x[2]) * q.b / p_l, v * math.tan(x[2]) / p_l)
        beta, v = x[6], x[3]
        return x[0], x[1], x[4], v * math.cos(beta), v * math.sin(beta), x[5]

    def rec(k, x):
        cx, cy, yaw, vx, vy, r = body(x)
        for key, val in (("t", k * DT), ("x", cx), ("y", cy), ("yaw", yaw), ("vx", vx),
                         ("vy", vy), ("r", r), ("steer", x[2])):
            out[key][k] = val

    rec(0, x)
    for k in range(n):
        if k % CTRL_EVERY == 0:
            accel = pi(v0 - out["vx"][k])
        u = [(steer[k + 1] - steer[k]) / DT, accel]
        if model == "STD":
            # The wheel-speed states are stiff: R^2 K_x / (I_y_w v). Substep so
            # RK4 stays well inside its stability limit.
            lam = q.R_w ** 2 * q.tire.p_kx1 * q.m * G_CR / 2.0 / (q.I_y_w * max(abs(x[3]), 0.1))
            sub = max(1, int(math.ceil(DT * lam / 1.0)))
        else:
            sub = 1
        h = DT / sub
        for _ in range(sub):
            x = rk4(lambda xx, uu: f(xx, uu, q), x, u, h)
        rec(k + 1, x)
    out["yaw"] = np.unwrap(out["yaw"])
    return out


# ---------------------------------------------------------- analytic theory

def linear_bicycle(p, v: float) -> dict:
    """Closed-form linear single-track quantities at speed v (Rajamani 2012,
    ch. 2 and 3; Pacejka 2012, ch. 1). States (v_y, r)."""
    m, iz, a, b, cf, cr = p.mass, p.izz, p.lf, p.lr, p.c_alpha_f, p.c_alpha_r
    big_l = a + b
    a11 = -(cf + cr) / (m * v)
    a12 = -v - (a * cf - b * cr) / (m * v)
    a21 = -(a * cf - b * cr) / (iz * v)
    a22 = -(a * a * cf + b * b * cr) / (iz * v)
    b1, b2 = cf / m, a * cf / iz
    det, tr = a11 * a22 - a12 * a21, a11 + a22
    k_us = m / big_l * (b / cf - a / cr)          # understeer gradient [rad/(m/s^2)]
    gain = v / (big_l + k_us * v * v)             # yaw-rate gain r / delta [1/s]
    wn = math.sqrt(det)
    return {"A": [[a11, a12], [a21, a22]], "B": [b1, b2], "K_us": k_us, "yaw_gain": gain,
            "yaw_gain_from_A": (a21 * b1 - a11 * b2) / det, "omega_n": wn, "zeta": -tr / (2 * wn),
            "beta_gain": (b1 * a22 - a12 * b2) / det}


def run_analytic(p, steer: np.ndarray, v: float, relax: bool) -> dict:
    """The linear single-track model's exact response to the steer samples
    at constant speed v. relax adds a first-order lag on each axle's force
    with time constant sigma / v (the relaxation-length tyre model of
    Pacejka 2012; SlipX relaxation.hpp), sigma the twin's relaxation length."""
    lb = linear_bicycle(p, v)
    m, iz, a, b, cf, cr = p.mass, p.izz, p.lf, p.lr, p.c_alpha_f, p.c_alpha_r
    if not relax:
        sys_ = signal.StateSpace(np.array(lb["A"]), np.array([[lb["B"][0]], [lb["B"][1]]]),
                                 np.eye(2), np.zeros((2, 1)))
    else:
        sf, sr = v / p.tyre_front.relax_length, v / p.tyre_rear.relax_length
        # x = (v_y, r, Fy_f, Fy_r); Fy' = (v / sigma) (-C alpha - Fy)
        A = np.array([[0.0, -v, 1 / m, 1 / m],
                      [0.0, 0.0, a / iz, -b / iz],
                      [-sf * cf / v, -sf * cf * a / v, -sf, 0.0],
                      [-sr * cr / v, sr * cr * b / v, 0.0, -sr]])
        B = np.array([[0.0], [0.0], [sf * cf], [0.0]])
        C = np.zeros((2, 4))
        C[0, 0] = C[1, 1] = 1.0
        sys_ = signal.StateSpace(A, B, C, np.zeros((2, 1)))
    t = np.arange(len(steer)) * DT
    _, y, _ = signal.lsim(sys_, steer, t)
    vy, r = y[:, 0], y[:, 1]
    yaw = np.concatenate([[0.0], np.cumsum(0.5 * (r[1:] + r[:-1]) * DT)])
    xd, yd = v * np.cos(yaw) - vy * np.sin(yaw), v * np.sin(yaw) + vy * np.cos(yaw)
    xs = np.concatenate([[0.0], np.cumsum(0.5 * (xd[1:] + xd[:-1]) * DT)])
    ys = np.concatenate([[0.0], np.cumsum(0.5 * (yd[1:] + yd[:-1]) * DT)])
    return {"t": t, "x": xs, "y": ys, "yaw": yaw, "vx": np.full_like(t, v), "vy": vy, "r": r,
            "steer": steer}


def run_kinematic(p, steer: np.ndarray, v: float) -> dict:
    """r = v tan(delta) / L at the CoG, v the body-x speed (Rajamani 2012, sec. 2.2)."""
    big_l = p.lf + p.lr
    r = v * np.tan(steer) / big_l
    vy = r * p.lr
    t = np.arange(len(steer)) * DT
    yaw = np.concatenate([[0.0], np.cumsum(0.5 * (r[1:] + r[:-1]) * DT)])
    xd, yd = v * np.cos(yaw) - vy * np.sin(yaw), v * np.sin(yaw) + vy * np.cos(yaw)
    xs = np.concatenate([[0.0], np.cumsum(0.5 * (xd[1:] + xd[:-1]) * DT)])
    ys = np.concatenate([[0.0], np.cumsum(0.5 * (yd[1:] + yd[:-1]) * DT)])
    return {"t": t, "x": xs, "y": ys, "yaw": yaw, "vx": np.full_like(t, v), "vy": vy, "r": r,
            "steer": steer}


# ----------------------------------------------------------------- metrics

def frame_from(run: dict, k0: int) -> tuple[np.ndarray, np.ndarray]:
    """Positions from sample k0 on, in the frame of the pose at k0."""
    c, s = math.cos(run["yaw"][k0]), math.sin(run["yaw"][k0])
    dx, dy = run["x"][k0:] - run["x"][k0], run["y"][k0:] - run["y"][k0]
    return c * dx + s * dy, -s * dx + c * dy


def path_deviation(a: dict, b: dict, k0: int, n: int) -> dict:
    ax, ay = frame_from(a, k0)
    bx, by = frame_from(b, k0)
    d = np.hypot(ax[: n + 1] - bx[: n + 1], ay[: n + 1] - by[: n + 1])
    travelled = float(np.sum(np.hypot(np.diff(bx[: n + 1]), np.diff(by[: n + 1]))))
    return {"end_m": float(d[-1]), "max_m": float(d.max()), "path_length_m": travelled,
            "end_pct_of_path": 100.0 * float(d[-1]) / travelled if travelled > 0 else None}


def steady(run: dict) -> dict:
    k = int(round(STEADY_AVG / DT))
    r, vx, vy, st = run["r"][-k:], run["vx"][-k:], run["vy"][-k:], run["steer"][-k:]
    rm = float(r.mean())
    spread = float(r.max() - r.min())
    vxm = float(vx.mean())
    ay = vxm * rm
    speed = math.hypot(vxm, float(vy.mean()))
    return {"r": rm, "ay": ay, "vx": vxm, "beta": math.atan2(float(vy.mean()), vxm),
            "steer": float(st.mean()), "radius": speed / rm if rm != 0 else None,
            "steady": bool(spread <= STEADY_TOL * abs(rm) and abs(vxm - run["vx"][0]) < 0.02 * abs(run["vx"][0]))}


def kinematic_steer(p, ss: dict) -> float:
    """The steer angle a kinematic car needs for the same CoG radius."""
    big_l, b = p.lf + p.lr, p.lr
    r_cog = abs(ss["radius"])
    return math.copysign(math.atan(big_l / math.sqrt(max(r_cog * r_cog - b * b, 1e-12))), ss["r"])


def step_metrics(run: dict, k_step: int) -> dict:
    """Yaw-rate step metrics (after ISO 7401): steady value over the last
    0.5 s, 10-90 % rise time, peak time and overshoot from the steer step,
    and the response time from 50 % achieved steer to 90 % yaw rate."""
    t, r, st = run["t"][k_step:] - run["t"][k_step], run["r"][k_step:], run["steer"][k_step:]
    k = int(round(0.5 / DT))
    rss, sss = float(r[-k:].mean()), float(st[-k:].mean())

    def first_cross(sig, level):
        idx = np.nonzero(sig >= level)[0]
        if len(idx) == 0:
            return None
        i = idx[0]
        if i == 0:
            return float(t[0])
        f = (level - sig[i - 1]) / (sig[i] - sig[i - 1])
        return float(t[i - 1] + f * (t[i] - t[i - 1]))

    s = 1.0 if rss >= 0 else -1.0
    rn = s * r / abs(rss)
    t10, t90 = first_cross(rn, 0.1), first_cross(rn, 0.9)
    ipk = int(np.argmax(rn))
    t50s = first_cross(st / sss, 0.5)
    over = 100.0 * (float(rn[ipk]) - 1.0)
    return {"r_ss": rss, "steer_ss": sss, "gain": rss / sss,
            "rise_time_s": (t90 - t10) if t10 is not None and t90 is not None else None,
            "peak_time_s": float(t[ipk]) if over > 0.1 else None,   # none: monotonic
            "overshoot_pct": over,
            "response_time_s": (t90 - t50s) if t90 is not None and t50s is not None else None}


def sine_metrics(run: dict, k0: int, freq: float, periods: int) -> dict:
    """Least-squares fit of the yaw rate and the steer over the last two
    periods: amplitude ratio and the yaw rate's phase lag behind the steer."""
    t = run["t"][k0:] - run["t"][k0]
    sel = t >= (periods - 2) / freq
    w = 2 * math.pi * freq
    basis = np.column_stack([np.sin(w * t[sel]), np.cos(w * t[sel]), np.ones(sel.sum())])

    def fit(y):
        c, *_ = np.linalg.lstsq(basis, y[sel], rcond=None)
        return math.hypot(c[0], c[1]), math.atan2(c[1], c[0])

    ar, pr = fit(run["r"][k0:])
    as_, ps = fit(run["steer"][k0:])
    lag = math.degrees((ps - pr + math.pi) % (2 * math.pi) - math.pi)
    return {"r_amp": ar, "steer_amp": as_, "gain": ar / as_, "phase_lag_deg": lag}


def identify_yaw(run: dict, k0: int, t_fit: float = 0.5) -> dict:
    """Natural frequency and damping identified from a step response. The
    yaw rate alone cannot separate them here (with zeta near 1 a zero nearly
    cancels a pole, and very different (wn, zeta) fit it equally well), so
    the whole linear single-track model is regressed instead: the
    derivatives of (v_y, r), central differences of the 1 ms samples, on
    (v_y, r, delta) over t_fit seconds after the step, by least squares.
    wn and zeta are then those of the identified A, as in linear_bicycle."""
    n = int(round(t_fit / DT))
    sl = slice(k0 + 2, k0 + n)
    vy, r, st = run["vy"], run["r"], run["steer"]
    dvy = (vy[k0 + 3:k0 + n + 1] - vy[k0 + 1:k0 + n - 1]) / (2 * DT)
    dr = (r[k0 + 3:k0 + n + 1] - r[k0 + 1:k0 + n - 1]) / (2 * DT)
    X = np.column_stack([vy[sl], r[sl], st[sl]])
    cy, *_ = np.linalg.lstsq(X, dvy, rcond=None)
    cr, *_ = np.linalg.lstsq(X, dr, rcond=None)
    a = np.array([[cy[0], cy[1]], [cr[0], cr[1]]])
    det, tr = float(np.linalg.det(a)), float(np.trace(a))
    wn = math.sqrt(det)
    res = np.concatenate([dvy - X @ cy, dr - X @ cr])
    return {"A": a.tolist(), "B": [float(cy[2]), float(cr[2])], "omega_n": wn, "zeta": -tr / (2 * wn),
            "gain": float((a[1, 0] * cy[2] - a[0, 0] * cr[2]) / det),
            "rms_residual": float(np.sqrt(np.mean(res ** 2)))}


def rel(a: float, b: float) -> float:
    return 100.0 * (a - b) / b


# ---------------------------------------------------------- the manoeuvres

def ramp_hold(target: float, pre: float = 0.0):
    def f(t):
        if t < pre:
            return 0.0
        return target * min((t - pre) / STEADY_RAMP, 1.0)
    return f


def references_for(p, q, steer, v, which=("analytic", "analytic-relax", "ST", "STD")) -> dict:
    out = {}
    for name in which:
        if name == "analytic":
            out[name] = run_analytic(p, steer, v, relax=False)
        elif name == "analytic-relax":
            out[name] = run_analytic(p, steer, v, relax=True)
        elif name == "kinematic":
            out[name] = run_kinematic(p, steer, v)
        else:
            out[name] = run_cr(name, q, steer, v)
    return out


def steady_sweeps(lib, p, q, log) -> dict:
    res = {"speed_sweep": {"steer": SPEED_SWEEP["steer"], "rows": []},
           "understeer": {"speed": UNDERSTEER["speed"], "rows": []}}
    t_end = STEADY_RAMP + STEADY_HOLD
    peq = twin_params(lib, "L2-eqC")
    for v in SPEED_SWEEP["speeds"]:
        row = {"v": v, "analytic": linear_bicycle(p, v)["yaw_gain"],
               "analytic_eqC": linear_bicycle(peq, v)["yaw_gain"]}
        for cfg in CONFIGS + CONFIGS_EQ:
            row[cfg] = steady(run_twin(lib, cfg, ramp_hold(SPEED_SWEEP["steer"]), v, t_end))
        steer = run_twin(lib, "L1", ramp_hold(SPEED_SWEEP["steer"]), v, t_end)["steer"]
        for name, run in references_for(p, q, steer, v, ("ST", "STD")).items():
            row[name] = steady(run)
        res["speed_sweep"]["rows"].append(row)
        log(f"  speed sweep {v:4.1f} m/s")
    v = UNDERSTEER["speed"]
    for d in UNDERSTEER["steers"]:
        row = {"steer_cmd": d}
        for cfg in CONFIGS + CONFIGS_EQ:
            ss = steady(run_twin(lib, cfg, ramp_hold(d), v, t_end))
            ss["steer_minus_kinematic"] = ss["steer"] - kinematic_steer(p, ss) if ss["radius"] else None
            row[cfg] = ss
        steer = run_twin(lib, "L1", ramp_hold(d), v, t_end)["steer"]
        for name, run in references_for(p, q, steer, v, ("ST", "STD")).items():
            ss = steady(run)
            ss["steer_minus_kinematic"] = ss["steer"] - kinematic_steer(p, ss) if ss["radius"] else None
            row[name] = ss
        for key, pp in (("analytic", p), ("analytic_eqC", peq)):
            lb = linear_bicycle(pp, v)
            row[key] = {"r": lb["yaw_gain"] * d, "ay": v * lb["yaw_gain"] * d,
                        "steer_minus_kinematic": lb["K_us"] * v * lb["yaw_gain"] * d}
        res["understeer"]["rows"].append(row)
        log(f"  understeer {d:.2f} rad")
    return res


def transients(lib, p, q, log) -> dict:
    res = {"step": [], "sine": None, "series": {}}
    peq = twin_params(lib, "L2-eqC")
    v, pre = STEP["speed"], STEP["pre"]
    k0 = int(round(pre / DT))
    n_path = int(round(PATH_N / DT))
    for d in STEP["steers"]:
        block = {"steer": d, "speed": v, "twin": {}, "references": {}, "errors": {}}
        for cfg in CONFIGS + CONFIGS_EQ:
            tw = run_twin(lib, cfg, lambda t, d=d: d if t >= pre else 0.0, v, pre + STEP["after"])
            pr = peq if cfg.endswith("eqC") else p
            refs = references_for(pr, q, tw["steer"], v)
            block["twin"][cfg] = step_metrics(tw, k0)
            block["references"][cfg] = {n: step_metrics(r, k0) for n, r in refs.items()}
            block["errors"][cfg] = {
                n: {"r_ss_pct": rel(block["twin"][cfg]["r_ss"], block["references"][cfg][n]["r_ss"]),
                    "path": path_deviation(tw, r, k0, n_path)} for n, r in refs.items()}
            if d == STEP["steers"][0] and cfg in ("L1", "L1-eqC"):
                # L1 and the references have no servo and no tyre lag, so the
                # yaw response is exactly second order: identify it.
                lb = linear_bicycle(pr, v)
                block.setdefault("identified", {})[cfg] = {
                    "closed_form": {"omega_n": lb["omega_n"], "zeta": lb["zeta"], "gain": lb["yaw_gain"]},
                    "twin": identify_yaw(tw, k0), "ST": identify_yaw(refs["ST"], k0),
                    "STD": identify_yaw(refs["STD"], k0)}
            if d == STEP["steers"][0] and cfg in CONFIGS:
                sl = slice(k0 - 100, k0 + 400, 2)
                res["series"].setdefault("step", {"t": (tw["t"][sl] - pre).tolist(), "steer": d})
                res["series"]["step"][cfg] = (tw["r"][sl] / d).tolist()
                res["series"]["step"][cfg + "_steer"] = (tw["steer"][sl] / d).tolist()
                for n, r in refs.items():
                    res["series"]["step"][f"{n}|{cfg}"] = (r["r"][sl] / d).tolist()
            log(f"  step {d:.2f} rad {cfg}")
        res["step"].append(block)
    v, pre, a, fq, per = SINE["speed"], SINE["pre"], SINE["amp"], SINE["freq"], SINE["periods"]
    k0 = int(round(pre / DT))
    t_end = pre + per / fq
    block = {"speed": v, "amp": a, "freq": fq, "twin": {}, "references": {}, "errors": {}}
    for cfg in CONFIGS + CONFIGS_EQ:
        tw = run_twin(lib, cfg, lambda t: a * math.sin(2 * math.pi * fq * (t - pre)) if t >= pre else 0.0,
                      v, t_end)
        pr = peq if cfg.endswith("eqC") else p
        refs = references_for(pr, q, tw["steer"], v)
        block["twin"][cfg] = sine_metrics(tw, k0, fq, per)
        block["references"][cfg] = {n: sine_metrics(r, k0, fq, per) for n, r in refs.items()}
        nn = len(tw["t"]) - 1 - k0
        block["errors"][cfg] = {
            n: {"gain_pct": rel(block["twin"][cfg]["gain"], block["references"][cfg][n]["gain"]),
                "phase_deg": block["twin"][cfg]["phase_lag_deg"] - block["references"][cfg][n]["phase_lag_deg"],
                "r_rms": float(np.sqrt(np.mean((tw["r"][k0:] - r["r"][k0:]) ** 2))),
                "path": path_deviation(tw, r, k0, nn)} for n, r in refs.items()}
        log(f"  sine {cfg}")
    res["sine"] = block
    return res


def low_speed(lib, p, q, log) -> dict:
    res = {"cases": [], "series": {}}
    d = p.steer_max if LOW["steer"] is None else LOW["steer"]
    pre = LOW["pre"]
    k0 = int(round(pre / DT))
    for v in LOW["speeds"]:
        case = {"speed": v, "steer": d, "kinematic_r": v * math.tan(d) / (p.lf + p.lr), "twin": {},
                "references": {}, "errors": {}}
        for cfg in ("L0", "L1", "L2", "L2-unity"):
            tw = run_twin(lib, cfg, lambda t: d if t >= pre else 0.0, v, pre + LOW["after"])
            refs = references_for(p, q, tw["steer"], v, ("kinematic", "KS", "ST"))
            ss = steady(tw)
            case["twin"][cfg] = ss
            case["references"][cfg] = {n: steady(r) for n, r in refs.items()}
            nn = len(tw["t"]) - 1 - k0
            case["errors"][cfg] = {n: {"r_pct": rel(ss["r"], case["references"][cfg][n]["r"]),
                                       "path": path_deviation(tw, r, k0, min(nn, int(round(PATH_N / DT))))}
                                   for n, r in refs.items()}
            if v == LOW["speeds"][0]:
                bx, by = frame_from(tw, k0)
                res["series"][cfg] = {"x": bx[::20].tolist(), "y": by[::20].tolist()}
                if cfg == "L2-unity":
                    for n, r in refs.items():
                        rx, ry = frame_from(r, k0)
                        res["series"][n] = {"x": rx[::20].tolist(), "y": ry[::20].tolist()}
            log(f"  low speed {v} m/s {cfg}")
        res["cases"].append(case)
    return res


# ------------------------------------------------------------------ summary

def summarise(p, st: dict, tr: dict, lo: dict) -> dict:
    s = {}
    # Steady state, speed sweep: yaw-rate gain against the analytic gain.
    for cfg in CONFIGS + CONFIGS_EQ:
        key = "analytic_eqC" if cfg.endswith("eqC") else "analytic"
        errs = [rel(row[cfg]["r"] / row[cfg]["steer"], row[key]) for row in st["speed_sweep"]["rows"]]
        s.setdefault("speed_sweep_yaw_gain_err_pct_vs_analytic", {})[cfg] = {
            "max_abs": max(abs(e) for e in errs), "per_speed": dict(zip(SPEED_SWEEP["speeds"], errs))}
    for ref in ("ST", "STD"):
        for cfg in CONFIGS + CONFIGS_EQ:
            errs = [rel(row[cfg]["r"], row[ref]["r"]) for row in st["speed_sweep"]["rows"]]
            s.setdefault(f"speed_sweep_yaw_rate_err_pct_vs_{ref}", {})[cfg] = {
                "max_abs": max(abs(e) for e in errs), "per_speed": dict(zip(SPEED_SWEEP["speeds"], errs))}
    # Understeer curve: steady errors in r and a_y, in the linear range and overall.
    for ref in ("analytic", "ST", "STD"):
        for cfg in CONFIGS + CONFIGS_EQ:
            lin, allr = [], []
            rk = "analytic_eqC" if ref == "analytic" and cfg.endswith("eqC") else ref
            for row in st["understeer"]["rows"]:
                a, b = row[cfg], row[rk]
                if not ref.startswith("analytic") and not b["steady"]:
                    continue
                if not a["steady"]:
                    continue
                e = (rel(a["r"], b["r"]), rel(a["ay"], b["ay"]))
                allr.append(e)
                if abs(b["ay"]) < LINEAR_AY:
                    lin.append(e)
            s.setdefault(f"understeer_err_pct_vs_{ref}", {})[cfg] = {
                "linear_max_abs_r": max(abs(e[0]) for e in lin) if lin else None,
                "linear_max_abs_ay": max(abs(e[1]) for e in lin) if lin else None,
                "all_steady_max_abs_r": max(abs(e[0]) for e in allr) if allr else None,
                "all_steady_max_abs_ay": max(abs(e[1]) for e in allr) if allr else None,
                "points": len(allr)}
    # Highest steady lateral acceleration each model reaches.
    s["max_steady_ay"] = {}
    for name in CONFIGS + CONFIGS_EQ + ("ST", "STD"):
        ays = [abs(row[name]["ay"]) for row in st["understeer"]["rows"] if row[name]["steady"]]
        s["max_steady_ay"][name] = max(ays) if ays else None
    return s


# --------------------------------------------------------------------- main

def git_commit() -> str | None:
    try:
        return subprocess.run(["git", "-C", str(ROOT), "rev-parse", "HEAD"], capture_output=True,
                              text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def to_json(o):
    if isinstance(o, dict):
        return {str(k): to_json(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [to_json(v) for v in o]
    if isinstance(o, (np.floating, float)):
        return None if not math.isfinite(float(o)) else round(float(o), 9)
    if isinstance(o, np.integer):
        return int(o)
    if isinstance(o, np.bool_):
        return bool(o)
    return o


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--lib", type=pathlib.Path, default=ROOT / "build" / "bin" / "rc_twin.dll")
    ap.add_argument("--out", type=pathlib.Path, default=ROOT / "results" / "reference" / "cross_check.json")
    ap.add_argument("--no-figures", action="store_true")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args(argv)

    def log(msg):
        if not args.quiet:
            print(msg, file=sys.stderr, flush=True)

    lib = dr.load_lib(args.lib)
    lib.rc_twin_core_version.restype = ctypes.c_char_p
    p = twin_params(lib, "L2")
    q = cr_params(p)
    analytic = {"provisional": {v: linear_bicycle(p, v) for v in (1.0, 2.0, 4.0, 5.0, 8.0, 10.0)},
                "eqC": {v: linear_bicycle(twin_params(lib, "L2-eqC"), v) for v in (1.0, 2.0, 4.0, 5.0, 8.0, 10.0)}}
    k_us = analytic["provisional"][5.0]["K_us"]
    analytic["characteristic_speed_m_s"] = math.sqrt((p.lf + p.lr) / k_us) if k_us > 0 else None

    log("steady state")
    st = steady_sweeps(lib, p, q, log)
    log("transients")
    tr = transients(lib, p, q, log)
    log("low speed")
    lo = low_speed(lib, p, q, log)

    params = {f: getattr(p, f) for f, _ in dr.Params._fields_ if f not in ("tyre_front", "tyre_rear")}
    params["tyre_front"] = {f: getattr(p.tyre_front, f) for f, _ in dr.Tyre._fields_}
    params["tyre_rear"] = {f: getattr(p.tyre_rear, f) for f, _ in dr.Tyre._fields_}
    pu = twin_params(lib, "L2-unity")
    result = {
        "what": "Model-to-model cross-check of the rc_twin Core against analytic linear single-track "
                "theory and the CommonRoad vehicle models. Not a validation against a car: every "
                "parameter is provisional.",
        "meta": {"commit": git_commit(), "date": datetime.date.today().isoformat(),
                 "rc_twin_abi": lib.rc_twin_abi_version(), "slipx_core": lib.rc_twin_core_version().decode(),
                 "python": platform.python_version(),
                 "packages": {k: importlib.metadata.version(k) for k in
                              ("commonroad-vehicle-models", "numpy", "scipy")},
                 "command": "python tools/cross_check.py",
                 "settings": {"dt": DT, "controller_hz": 1.0 / (DT * CTRL_EVERY), "kp": KP, "ki": KI,
                              "speed_sweep": SPEED_SWEEP, "understeer": UNDERSTEER,
                              "steady_ramp_hold_avg_s": [STEADY_RAMP, STEADY_HOLD, STEADY_AVG],
                              "steady_tolerance": STEADY_TOL, "linear_ay": LINEAR_AY, "step": STEP,
                              "sine": SINE, "low_speed": {**LOW, "steer": p.steer_max}, "path_n_s": PATH_N}},
        "twin_params": {"provisional": params,
                        "unity_options": {k: getattr(pu, k) for k in
                                          ("wheel_inertia", "ackermann", "wheel_rate_front", "damping_front",
                                           "suspension_travel", "v_kinematic")},
                        "eqC_c_alpha": twin_params(lib, "L2-eqC").c_alpha_f},
        "commonroad_mapping": cr_mapping(p, q),
        "analytic": analytic,
        "steady": st,
        "transient": tr,
        "low_speed": lo,
        "summary": summarise(p, st, tr, lo),
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(to_json(result), indent=1) + "\n", encoding="utf-8")
    log(f"wrote {args.out}")
    if not args.no_figures:
        figures(result, args.out.parent, p)
    return 0


# ------------------------------------------------------------------ figures

STYLE = {  # colour (validated categorical slots, dataviz reference palette), line style
    "analytic": ("#52514e", "-"), "analytic-relax": ("#52514e", "--"),
    "ST": ("#2a78d6", "-"), "STD": ("#eb6834", "-"), "KS": ("#eda100", "-"),
    "kinematic": ("#52514e", ":"),
    "L1": ("#1baf7a", "--"), "L2": ("#6250d6", "-"), "L2-unity": ("#e34948", "-."),
    "L0": ("#eda100", ":"),
}
LABEL = {"analytic": "linear theory", "analytic-relax": "linear + relaxation", "ST": "CommonRoad ST",
         "STD": "CommonRoad STD", "KS": "CommonRoad KS", "kinematic": "kinematic",
         "L0": "twin L0", "L1": "twin L1", "L2": "twin L2", "L2-unity": "twin L2, Unity car"}


META = {"pdf": {"CreationDate": None, "ModDate": None}, "png": {"Software": None}}   # reproducible files


def figures(res: dict, out_dir: pathlib.Path, p) -> None:
    import logging
    logging.getLogger("fontTools").setLevel(logging.ERROR)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family": "serif", "mathtext.fontset": "cm", "font.size": 7.5,
                         "axes.linewidth": 0.6, "lines.linewidth": 1.1, "legend.fontsize": 6.5,
                         "xtick.major.width": 0.5, "ytick.major.width": 0.5, "axes.grid": True,
                         "grid.linewidth": 0.3, "grid.color": "#d9d8d4", "legend.frameon": False,
                         "pdf.fonttype": 42, "savefig.dpi": 300})
    width = 122 / 25.4   # LNCS text width

    # Figure 1: steady state.
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(width, 2.1))
    v_us = res["steady"]["understeer"]["speed"]
    rows = res["steady"]["understeer"]["rows"]
    ds = np.array([row["steer_cmd"] for row in rows])
    a1.plot(ds, [row["analytic"]["ay"] for row in rows], "-", color=STYLE["analytic"][0],
            label=LABEL["analytic"])
    for name in ("ST", "STD", "L1", "L2", "L2-unity"):
        pts = [(row[name]["steer"], row[name]["ay"]) for row in rows if row[name]["steady"]]
        x, y = zip(*pts)
        c, ls = STYLE[name]
        a1.plot(x, y, ls, color=c, label=LABEL[name], marker="o", ms=2.2, mew=0)
    a1.set_ylim(0, 12.5)
    a1.set_xlim(0, 0.41)
    a1.annotate("ST and linear theory\ncontinue linearly", xy=(0.255, 12.3), xytext=(0.01, 10.0),
                fontsize=5.8, color="#52514e", arrowprops={"arrowstyle": "->", "lw": 0.5, "color": "#52514e"})
    a1.set_xlabel(r"steer angle $\delta$ [rad]")
    a1.set_ylabel(r"steady $a_y$ [m/s$^2$]")
    a1.set_title(f"(a) steady circle at {v_us:g} m/s", fontsize=7.5)
    rows = res["steady"]["speed_sweep"]["rows"]
    vs = np.array([row["v"] for row in rows])
    kin = vs / (p.lf + p.lr)
    vv = np.linspace(vs[0], vs[-1], 50)
    a2.plot(vv, [linear_bicycle(p, v)["yaw_gain"] * (p.lf + p.lr) / v for v in vv], "-",
            color=STYLE["analytic"][0])
    a2.axhline(1.0, color="#8a8984", lw=0.8, ls=":", label=r"neutral steer, $v/L$")
    for name in ("ST", "STD", "L1", "L2", "L2-unity"):
        c, ls = STYLE[name]
        a2.plot(vs, np.array([row[name]["r"] / row[name]["steer"] for row in rows]) / kin, ls, color=c,
                marker="o", ms=2.2, mew=0)
    a2.plot(vs, np.array([row["L2-eqC"]["r"] / row["L2-eqC"]["steer"] for row in rows]) / kin, "none",
            marker="o", ms=3.4, mfc="none", mec=STYLE["L2"][0], mew=0.6, label=r"twin L2, equal $C_\alpha$")
    a2.set_xlabel("speed $v$ [m/s]")
    a2.set_ylabel(r"$(r/\delta)\,/\,(v/L)$")
    a2.set_title(r"(b) yaw-rate gain, $\delta$ = 0.01 rad", fontsize=7.5)
    h1, l1 = a1.get_legend_handles_labels()
    h2, l2 = a2.get_legend_handles_labels()
    fig.legend(h1 + h2, l1 + l2, loc="lower center", ncol=4, bbox_to_anchor=(0.5, -0.03), handlelength=2.2)
    fig.tight_layout(rect=(0, 0.17, 1, 1), w_pad=1.2)
    for ext in ("pdf", "png"):
        fig.savefig(out_dir / f"cross_check_steady.{ext}", bbox_inches="tight", metadata=META[ext])
    plt.close(fig)

    # Figure 2: step steer and the low-speed case.
    fig, (b1, b2) = plt.subplots(1, 2, figsize=(width, 2.45))
    s = res["transient"]["series"]["step"]
    t = 1e3 * np.array(s["t"])
    for key, name, steer in (("analytic|L1", "analytic", "L1"), ("L1", "L1", None),
                             ("analytic-relax|L2", "analytic-relax", "L2"), ("ST|L2", "ST", "L2"),
                             ("STD|L2", "STD", "L2"), ("L2", "L2", None), ("L2-unity", "L2-unity", None)):
        c, ls = STYLE[name]
        b1.plot(t, s[key], ls, color=c, label=LABEL[name])
    g5 = linear_bicycle(p, STEP["speed"])["yaw_gain"]
    b1.plot(t, g5 * np.array(s["L2_steer"]), ls=(0, (4, 1.5, 1, 1.5)), color="#8a8984",
            label=r"L2 achieved steer $\times$ linear gain")
    b1.set_xlim(-20, 420)
    b1.set_xlabel("time after the steer step [ms]")
    b1.set_ylabel(r"$r/\delta_0$ [1/s]")
    b1.set_title(r"(a) step steer, 5 m/s, $\delta_0$ = 0.03 rad", fontsize=7.5)
    ser = res["low_speed"]["series"]
    for name in ("kinematic", "KS", "ST", "L1", "L2", "L2-unity"):
        c, ls = STYLE[name]
        b2.plot(ser[name]["x"], ser[name]["y"], ls, color=c, label=LABEL[name])
    b2.set_aspect("equal", adjustable="box")
    b2.set_xlim(-0.6, 1.0)
    b2.set_ylim(-0.1, 1.6)
    b2.set_xlabel("x [m]")
    b2.set_ylabel("y [m]")
    b2.set_title("(b) full lock at 0.2 m/s, 10 s", fontsize=7.5)
    # One legend for both panels, each series once.
    seen = {}
    for ax in (b1, b2):
        for h, lab in zip(*ax.get_legend_handles_labels()):
            seen.setdefault(lab, h)
    fig.legend(list(seen.values()), list(seen.keys()), loc="lower center", ncol=4,
               bbox_to_anchor=(0.5, -0.02), handlelength=2.2)
    fig.tight_layout(rect=(0, 0.2, 1, 1), w_pad=1.2)
    for ext in ("pdf", "png"):
        fig.savefig(out_dir / f"cross_check_transient.{ext}", bbox_inches="tight", metadata=META[ext])
    plt.close(fig)


if __name__ == "__main__":
    sys.exit(main())
