#!/usr/bin/env python3
"""Drive the twin along a road file, headlessly, and report the run.

The road (roads/road.schema.json) becomes the ground through rc_twin's road
API; the car follows the centreline with a pure-pursuit steering controller
and a proportional speed controller at 100 Hz over 1 ms Core steps. The
report is JSON: time, energy, peak accelerations, how far the car strayed
from the centreline, and, with suspension, peak body pitch and roll. This is
the headless harness of NEXT_STEPS Phase 1 and a first cut of Phase 4's edge
profiler. Deterministic: the same road, car and options give the same report.

Usage:
    python tools/drive_road.py roads/samples/hill_breaker_bend.json --speed 2

Exit status: 0 the car reached the end of the road, 2 it left the road or
timed out, 1 bad input.

Parameters are SlipX's provisional defaults unless set; nothing here is a
measured car.
"""

from __future__ import annotations

import argparse
import ctypes
import json
import math
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
ABI = 8
OK, E_GROUND = 0, -4


class Tyre(ctypes.Structure):
    _fields_ = [(n, ctypes.c_double) for n in
                ("mu_y0", "mu_x0", "k_mu", "relax_length", "shape_c", "curvature_e")]


class Params(ctypes.Structure):
    """native/include/rc_twin.h, rc_twin_params, field for field."""
    _fields_ = (
        [("struct_size", ctypes.c_uint32), ("provenance", ctypes.c_int32)]
        + [(n, ctypes.c_double) for n in
           ("mass", "izz", "ixx", "iyy", "lf", "lr", "track_front", "track_rear",
            "h_cog", "wheel_radius", "c_alpha_f", "c_alpha_r", "mu_clip")]
        + [("tyre_front", Tyre), ("tyre_rear", Tyre), ("c_kappa", ctypes.c_double)]
        + [(n, ctypes.c_double) for n in ("accel_max", "decel_max", "v_max")]
        + [("layout", ctypes.c_int32), ("differential", ctypes.c_int32)]
        + [(n, ctypes.c_double) for n in
           ("lsd_preload", "torque_stall", "omega_free", "torque_per_amp",
            "drive_efficiency", "current_max", "regen_current_max", "pack_nominal_v",
            "pack_v_full", "pack_v_empty", "pack_capacity_ah", "pack_internal_resistance",
            "steer_max", "steer_rate_max", "steer_bandwidth", "steer_damping",
            "drag_coeff", "roll_resist", "v_eps", "wheel_inertia", "ackermann",
            "wheel_rate_front", "wheel_rate_rear", "damping_front", "damping_rear",
            "suspension_travel", "v_kinematic")]
    )


class State(ctypes.Structure):
    """rc_twin_state: 28 doubles."""
    _fields_ = [(n, ctypes.c_double) for n in
                ("x", "y", "z", "yaw", "pitch", "roll", "vx", "vy", "vz",
                 "roll_rate", "pitch_rate", "yaw_rate", "w_fl", "w_fr", "w_rl", "w_rr",
                 "steer", "steer_rate", "soc", "pack_v", "fz_fl", "fz_fr", "fz_rl", "fz_rr",
                 "lag_fl", "lag_fr", "lag_rl", "lag_rr")]


class GroundSample(ctypes.Structure):
    """rc_twin_ground_sample."""
    _fields_ = [(n, ctypes.c_double) for n in ("height", "nx", "ny", "nz", "mu_scale")]


class Contact(ctypes.Structure):
    """rc_twin_contact: wall contact constants and the footprint (ADR 0009)."""
    _fields_ = [(n, ctypes.c_double) for n in
                ("restitution", "friction", "restitution_min_speed",
                 "half_length", "half_width", "centre_offset")]


class WallReport(ctypes.Structure):
    """rc_twin_wall_report: the last rc_twin_step call's wall contacts."""
    _fields_ = [("contacts", ctypes.c_uint32), ("reserved", ctypes.c_uint32),
                ("impulse", ctypes.c_double), ("max_approach", ctypes.c_double)]


class RoadError(ValueError):
    pass


def load_lib(path: pathlib.Path) -> ctypes.CDLL:
    lib = ctypes.CDLL(str(path))
    d, i32, u32, vp = ctypes.c_double, ctypes.c_int32, ctypes.c_uint32, ctypes.c_void_p
    dp, cpp = ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_char_p)
    lib.rc_twin_abi_version.restype = u32
    lib.rc_twin_params_default.argtypes = [ctypes.POINTER(Params)]
    lib.rc_twin_create.restype = vp
    lib.rc_twin_create.argtypes = [i32, i32, ctypes.POINTER(Params), cpp]
    lib.rc_twin_destroy.argtypes = [vp]
    lib.rc_twin_reset.argtypes = [vp, ctypes.POINTER(State)]
    lib.rc_twin_get_state.argtypes = [vp, ctypes.POINTER(State)]
    lib.rc_twin_step.restype = i32
    lib.rc_twin_step.argtypes = [vp, d, d, d, u32]
    lib.rc_twin_set_ground_query.argtypes = [vp, vp, vp]
    lib.rc_twin_road_create.restype = vp
    lib.rc_twin_road_create.argtypes = [dp, dp, u32, d, cpp]
    lib.rc_twin_road_destroy.argtypes = [vp]
    lib.rc_twin_road_length.restype = d
    lib.rc_twin_road_length.argtypes = [vp]
    lib.rc_twin_road_add_surface.restype = i32
    lib.rc_twin_road_add_surface.argtypes = [vp, d, d, d]
    lib.rc_twin_road_add_breaker.restype = i32
    lib.rc_twin_road_add_breaker.argtypes = [vp, d, d, d]
    lib.rc_twin_road_locate.argtypes = [vp, d, d, dp, dp]
    lib.rc_twin_road_point.argtypes = [vp, d, dp, dp, dp, dp]
    lib.rc_twin_road_ground.restype = i32
    lib.rc_twin_road_ground.argtypes = [vp, i32, d, d, ctypes.POINTER(GroundSample)]
    lib.rc_twin_set_params.restype = i32
    lib.rc_twin_set_params.argtypes = [vp, ctypes.POINTER(Params), cpp]
    lib.rc_twin_contact_default.restype = i32
    lib.rc_twin_contact_default.argtypes = [ctypes.POINTER(Params), ctypes.POINTER(Contact)]
    lib.rc_twin_set_contact.restype = i32
    lib.rc_twin_set_contact.argtypes = [vp, ctypes.POINTER(Contact)]
    lib.rc_twin_get_contact.restype = i32
    lib.rc_twin_get_contact.argtypes = [vp, ctypes.POINTER(Contact)]
    lib.rc_twin_add_wall.restype = i32
    lib.rc_twin_add_wall.argtypes = [vp, dp, u32, i32]
    lib.rc_twin_clear_walls.restype = i32
    lib.rc_twin_clear_walls.argtypes = [vp]
    lib.rc_twin_wall_count.restype = u32
    lib.rc_twin_wall_count.argtypes = [vp]
    lib.rc_twin_get_wall_report.restype = i32
    lib.rc_twin_get_wall_report.argtypes = [vp, ctypes.POINTER(WallReport)]
    if lib.rc_twin_abi_version() != ABI:
        raise RuntimeError(f"rc_twin ABI {lib.rc_twin_abi_version()}, this script needs {ABI}")
    return lib


def add_walls(lib, car, walls) -> None:
    """Hand walls to a vehicle: each {"points": [[x, y], ...], "closed": bool}."""
    for w in walls:
        pts = w["points"]
        xy = (ctypes.c_double * (2 * len(pts)))(*[c for p in pts for c in p[:2]])
        rc = lib.rc_twin_add_wall(car, xy, len(pts), 1 if w.get("closed") else 0)
        if rc != OK:
            raise ValueError(f"rc_twin_add_wall {w.get('name', '')!r} failed with {rc}")


def wall_report(lib, car) -> "WallReport":
    r = WallReport()
    lib.rc_twin_get_wall_report(car, ctypes.byref(r))
    return r


def check_road(road: dict) -> None:
    """The schema's rules, without a jsonschema dependency."""
    def need(cond: bool, msg: str) -> None:
        if not cond:
            raise RoadError(msg)
    num = (int, float)
    need(isinstance(road, dict), "road must be a JSON object")
    need(road.get("schema_version") == "0.1.0", "schema_version must be \"0.1.0\"")
    need(isinstance(road.get("id"), str) and road["id"], "id must be a non-empty string")
    need(isinstance(road.get("width"), num) and road["width"] > 0, "width must be a positive number [m]")
    pts = road.get("centreline")
    need(isinstance(pts, list) and len(pts) >= 2, "centreline needs at least two points")
    for k, p in enumerate(pts):
        need(isinstance(p, dict) and all(isinstance(p.get(c), num) for c in "xyz"),
             f"centreline[{k}] needs numeric x, y, z")
        need(set(p) <= {"x", "y", "z", "bank"}, f"centreline[{k}] has unknown fields")
        need(abs(p.get("bank", 0.0)) <= 0.5, f"centreline[{k}].bank must be within +-0.5 rad")
    for k, s in enumerate(road.get("surfaces", [])):
        need(all(isinstance(s.get(c), num) for c in ("from_s", "to_s", "mu_scale")),
             f"surfaces[{k}] needs from_s, to_s, mu_scale")
        need(s["to_s"] > s["from_s"] >= 0 and s["mu_scale"] >= 0, f"surfaces[{k}] is out of range")
    for k, b in enumerate(road.get("speed_breakers", [])):
        need(all(isinstance(b.get(c), num) for c in ("s", "height", "length")),
             f"speed_breakers[{k}] needs s, height, length")
        need(b.get("profile") == "round_hump", f"speed_breakers[{k}].profile must be round_hump")
        need(b["height"] > 0 and b["length"] > 0 and b["s"] >= 0, f"speed_breakers[{k}] is out of range")
    extra = set(road) - {"schema_version", "id", "description", "width", "centreline",
                         "surfaces", "speed_breakers"}
    need(not extra, f"unknown fields: {sorted(extra)}")


def build_road(lib: ctypes.CDLL, road: dict) -> ctypes.c_void_p:
    pts = road["centreline"]
    xyz = (ctypes.c_double * (3 * len(pts)))(*[v for p in pts for v in (p["x"], p["y"], p["z"])])
    bank = (ctypes.c_double * len(pts))(*[p.get("bank", 0.0) for p in pts])
    why = ctypes.c_char_p()
    r = lib.rc_twin_road_create(xyz, bank, len(pts), float(road["width"]), ctypes.byref(why))
    if not r:
        raise RoadError(why.value.decode() if why.value else "rc_twin_road_create failed")
    for s in road.get("surfaces", []):
        lib.rc_twin_road_add_surface(r, s["from_s"], s["to_s"], s["mu_scale"])
    for b in road.get("speed_breakers", []):
        lib.rc_twin_road_add_breaker(r, b["s"], b["height"], b["length"])
    return r


# The reference driver (ADR 0004 asks for it to be recorded with every cache).
CONTROLLER = {
    "steering": "pure pursuit on the centreline from the rear axle, lookahead max(0.4 m, 0.5 s x speed)",
    "speed": "proportional, accel = 3.0 (1/s) x (target - speed), within the car's limits",
    "speed_breakers": "if breaker_speed is set: slow to it before each breaker at comfort_decel, "
                      "hold it until the whole car is over, then resume",
    "rate_hz": 100,
    "core_step_s": 0.001,
}


def car_params(lib: ctypes.CDLL, args: argparse.Namespace) -> Params:
    """SlipX's provisional defaults plus the options and the payload."""
    p = Params()
    lib.rc_twin_params_default(ctypes.byref(p))
    p.wheel_inertia = args.wheel_inertia
    p.ackermann = args.ackermann
    p.wheel_rate_front = p.wheel_rate_rear = args.wheel_rate
    p.damping_front = p.damping_rear = args.damping
    p.suspension_travel = args.travel
    p.v_kinematic = getattr(args, "v_kinematic", 0.0)
    # A payload is a point mass at the CoG: it adds mass and, about the CoG,
    # no inertia.
    p.mass += getattr(args, "payload", 0.0)
    return p


def target_speed(road: dict, p: Params, s: float, args: argparse.Namespace) -> float:
    """The reference driver's target with the car's CoG at arc length s."""
    v = args.speed
    vb = getattr(args, "breaker_speed", None)
    if vb is None or vb >= v:
        return v
    a = getattr(args, "comfort_decel", 2.0)
    front = s + p.lf
    rear = s - p.lr
    for b in road.get("speed_breakers", []):
        start = b["s"] - 0.5 * b["length"] - (v * v - vb * vb) / (2.0 * a) - 0.3
        end = b["s"] + 0.5 * b["length"]
        if front >= start and rear <= end:
            return vb
    return v


def drive(lib: ctypes.CDLL, road: dict, args: argparse.Namespace) -> tuple[int, dict]:
    p = car_params(lib, args)
    why = ctypes.c_char_p()
    car = lib.rc_twin_create(2, 0, ctypes.byref(p), ctypes.byref(why))
    if not car:
        raise RoadError("car: " + (why.value.decode() if why.value else "rc_twin_create failed"))
    r = build_road(lib, road)
    try:
        length = lib.rc_twin_road_length(r)
        x, y, z, h = (ctypes.c_double() for _ in range(4))
        s0 = p.lr + 0.05  # rear wheels just onto the road
        lib.rc_twin_road_point(r, s0, ctypes.byref(x), ctypes.byref(y), ctypes.byref(z), ctypes.byref(h))
        v0 = args.speed if args.start_speed is None else args.start_speed
        w0 = v0 / p.wheel_radius
        st = State(x=x.value, y=y.value, z=z.value, yaw=h.value, vx=v0,
                   soc=getattr(args, "soc", 1.0), pack_v=11.1,
                   w_fl=w0, w_fr=w0, w_rl=w0, w_rr=w0)
        lib.rc_twin_reset(car, ctypes.byref(st))
        ground = ctypes.cast(lib.rc_twin_road_ground, ctypes.c_void_p)
        lib.rc_twin_set_ground_query(car, ground, r)

        dt, sub = 0.001, 10                       # Core step, steps per control tick
        tick = dt * sub
        s_cur, d_cur = ctypes.c_double(), ctypes.c_double()
        t = energy = 0.0
        peak = {"ax": 0.0, "ay": 0.0, "offset": 0.0, "pitch": 0.0, "roll": 0.0}
        status, end_s = "reached_end", length - p.lf
        prev = State()
        lib.rc_twin_get_state(car, ctypes.byref(prev))
        wb = p.lf + p.lr
        while True:
            lib.rc_twin_road_locate(r, prev.x, prev.y, ctypes.byref(s_cur), ctypes.byref(d_cur))
            if s_cur.value >= end_s:
                break
            if t >= args.timeout:
                status = "timeout"
                break
            # Pure pursuit on the centreline ahead, from the rear axle.
            look = max(0.4, 0.5 * prev.vx)
            lib.rc_twin_road_point(r, s_cur.value + look, ctypes.byref(x), ctypes.byref(y),
                                   ctypes.byref(z), ctypes.byref(h))
            rx = prev.x - p.lr * math.cos(prev.yaw)
            ry = prev.y - p.lr * math.sin(prev.yaw)
            alpha = math.atan2(y.value - ry, x.value - rx) - prev.yaw
            ld = math.hypot(x.value - rx, y.value - ry)
            steer = math.atan2(2.0 * wb * math.sin(alpha), ld)
            steer = max(-p.steer_max, min(p.steer_max, steer))
            v_target = target_speed(road, p, s_cur.value, args)
            accel = max(-p.decel_max, min(p.accel_max, 3.0 * (v_target - prev.vx)))
            rc = lib.rc_twin_step(car, steer, accel, dt, sub)
            if rc == E_GROUND:
                status = "left_road"
                break
            if rc != OK:
                raise RuntimeError(f"rc_twin_step failed with {rc}")
            cur = State()
            lib.rc_twin_get_state(car, ctypes.byref(cur))
            t += tick
            # Battery-terminal energy: V I dt, with I from the charge drawn.
            energy += cur.pack_v * (prev.soc - cur.soc) * p.pack_capacity_ah
            ax = (cur.vx - prev.vx) / tick - cur.vy * cur.yaw_rate
            ay = (cur.vy - prev.vy) / tick + cur.vx * cur.yaw_rate
            peak["ax"] = max(peak["ax"], abs(ax))
            peak["ay"] = max(peak["ay"], abs(ay))
            peak["offset"] = max(peak["offset"], abs(d_cur.value))
            peak["pitch"] = max(peak["pitch"], abs(cur.pitch))
            peak["roll"] = max(peak["roll"], abs(cur.roll))
            prev = cur
        travelled = max(0.0, s_cur.value - s0)
        report = {
            "road": road["id"],
            "status": status,
            "time_s": round(t, 6),
            "distance_m": round(travelled, 6),
            "mean_speed_mps": round(travelled / t, 6) if t > 0 else 0.0,
            "energy_wh": round(energy, 6),
            "soc_end": round(prev.soc, 9),
            "peak_ax_mps2": round(peak["ax"], 4),
            "peak_ay_mps2": round(peak["ay"], 4),
            "peak_offset_m": round(peak["offset"], 4),
            "car": {
                "target_speed_mps": args.speed,
                "wheel_inertia": args.wheel_inertia,
                "ackermann": args.ackermann,
                "suspension": args.wheel_rate > 0,
                "payload_kg": getattr(args, "payload", 0.0),
                "soc_start": getattr(args, "soc", 1.0),
                "breaker_speed_mps": getattr(args, "breaker_speed", None),
                "parameters": "PROVISIONAL (SlipX defaults); not a measured car",
            },
        }
        if args.wheel_rate > 0:
            report["peak_pitch_rad"] = round(peak["pitch"], 6)
            report["peak_roll_rad"] = round(peak["roll"], 6)
        return (0 if status == "reached_end" else 2), report
    finally:
        lib.rc_twin_destroy(car)
        lib.rc_twin_road_destroy(r)


def add_car_options(ap: argparse.ArgumentParser) -> None:
    """Car and driver options, shared with tools/edge_profiler.py."""
    ap.add_argument("--timeout", type=float, default=120.0, help="[s] of simulated time")
    ap.add_argument("--wheel-inertia", type=float, default=0.0, help="[kg m^2] (ADR 0006)")
    ap.add_argument("--ackermann", type=float, default=0.0, help="fraction of ideal (ADR 0007)")
    ap.add_argument("--wheel-rate", type=float, default=0.0, help="[N/m] per wheel, both axles (ADR 0008)")
    ap.add_argument("--damping", type=float, default=0.0, help="[N s/m] per wheel")
    ap.add_argument("--travel", type=float, default=0.015, help="[m] to the bump stop")
    ap.add_argument("--v-kinematic", type=float, default=0.0,
                    help="[m/s] kinematic at and below, blending up to v_eps (ADR 0011); 0 off")
    ap.add_argument("--breaker-speed", type=float, default=None,
                    help="[m/s] the driver slows to for speed breakers (default: does not slow)")
    ap.add_argument("--comfort-decel", type=float, default=2.0, help="[m/s^2] the driver slows at")
    ap.add_argument("--lib", type=pathlib.Path, default=ROOT / "build" / "bin" / "rc_twin.dll")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("road", type=pathlib.Path, help="road JSON (roads/road.schema.json)")
    ap.add_argument("--speed", type=float, default=2.0, help="target speed [m/s]")
    ap.add_argument("--start-speed", type=float, default=None, help="[m/s], default the target")
    ap.add_argument("--payload", type=float, default=0.0, help="[kg] at the CoG")
    ap.add_argument("--soc", type=float, default=1.0, help="state of charge at the start, 0..1")
    add_car_options(ap)
    args = ap.parse_args(argv)
    try:
        road = json.loads(args.road.read_text(encoding="utf-8"))
        check_road(road)
        lib = load_lib(args.lib)
        code, report = drive(lib, road, args)
    except (RoadError, OSError, json.JSONDecodeError) as e:
        print(f"drive_road: {e}", file=sys.stderr)
        return 1
    print(json.dumps(report, indent=2))
    return code


if __name__ == "__main__":
    sys.exit(main())
