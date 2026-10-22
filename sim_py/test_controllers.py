"""Tests for sim_py/controllers.py and sim_py/evaluate_policies.py.

    python -I sim_py/test_controllers.py
RC_TWIN_LIB overrides the DLL path (default build/bin/rc_twin.dll).
"""

from __future__ import annotations

import importlib.util
import json
import math
import os
import pathlib
import sys
import unittest

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


campus = _load("campus", HERE / "campus.py")
envmod = campus.envmod
C = _load("controllers", HERE / "controllers.py")
import evaluate_policies as ev  # noqa: E402

LIB = os.environ.get("RC_TWIN_LIB") or str(ROOT / "build" / "bin" / "rc_twin.dll")


def nominal():
    env = campus.CampusEnv(seed=0, ranges=envmod.RANGES_NOMINAL, lib_path=LIB)
    p = env.nominal_params()
    env.close()
    return C.Nominal(p)


class Geometry(unittest.TestCase):
    def test_curvature_of_a_circle_and_a_line(self):
        r = 2.0
        pts = [(r * math.sin(t), r - r * math.cos(t)) for t in (0.2, 0.5, 1.0)]   # a left turn, radius 2
        self.assertAlmostEqual(C.path_curvature(*pts), 0.5, places=9)
        mirrored = [(x, -y) for x, y in pts]
        self.assertAlmostEqual(C.path_curvature(*mirrored), -0.5, places=9)
        self.assertEqual(C.path_curvature((0.5, 0.1), (1.0, 0.1), (2.0, 0.1)), 0.0)

    def test_chord_heading_is_the_heading_error_on_a_straight_road(self):
        # A car yawed psi to the left of a straight road sees the road ahead
        # turned -psi in its own frame.
        for psi in (-0.3, 0.0, 0.2):
            pts = [(a * math.cos(-psi), a * math.sin(-psi)) for a in (0.5, 1.0, 2.0)]
            self.assertAlmostEqual(C.chord_heading(pts), psi, places=12)


class LQRDesign(unittest.TestCase):
    def test_the_closed_loop_is_stable_at_every_scheduled_speed(self):
        car = nominal()
        for v in C.LQR.SPEEDS:
            A, B = C._bicycle(car, v)
            Ad, Bd = C._discretise(A, B, envmod.TICK)
            K = C._dlqr(Ad, Bd, (30.0, 1.0, 0.3, 0.1), 30.0)
            M = C._madd(Ad, C._mm(Bd, [K]), -1.0)
            # Spectral radius below 1: a state decays under repeated steps.
            x = [[1.0], [0.5], [-0.3], [0.2]]
            for _ in range(3000):
                x = C._mm(M, x)
            self.assertLess(max(abs(c[0]) for c in x), 1e-6, f"v={v}")

    def test_discretisation_matches_the_exact_double_integrator(self):
        A = [[0.0, 1.0], [0.0, 0.0]]
        B = [[0.0], [1.0]]
        Ad, Bd = C._discretise(A, B, 0.1)
        self.assertEqual([[round(x, 12) for x in r] for r in Ad], [[1.0, 0.1], [0.0, 1.0]])
        self.assertAlmostEqual(Bd[0][0], 0.005, places=12)
        self.assertAlmostEqual(Bd[1][0], 0.1, places=12)


class Driving(unittest.TestCase):
    def test_tuned_controllers_drive_campus_trips_at_four_metres_per_second(self):
        tuned = json.loads((ROOT / "policies" / "controllers.json").read_text(encoding="utf-8"))
        for kind in ("pure_pursuit", "stanley", "lqr"):
            self.assertEqual(tuned[kind]["target_speed"], 4.0)
            rep = ev.evaluate([f"controller:{kind}"], trips=6, conditions=("train",), lib=LIB, workers=1,
                              target_speed=4.0)
            s = rep["policies"][f"controller:{kind}"]["train"]["summary"]
            self.assertEqual(s["success"][0], 1.0, kind)

    def test_the_unity_car_gains_with_kerb_recovery_drive_stop_trips(self):
        spec = "recover:controller:pure_pursuit:policies/controllers_unity_stop.json"
        rep = ev.evaluate([spec], trips=3, conditions=("train",), lib=LIB, workers=1, target_speed=4.0,
                          task="stop", car="unity")
        eps = rep["policies"][spec]["train"]["episodes"]
        self.assertEqual([e["status"] for e in eps], ["reached_end"] * 3)
        self.assertEqual([e["back_offs"] for e in eps], [0] * 3, "no back-off on a clean trip")

    def test_the_cornering_limit_slows_the_car_before_a_sharp_turn(self):
        car = nominal()
        idx = C._idx(envmod)
        obs = [0.0] * envmod.FRAME_SIZE
        obs[idx["speed_ratio"]] = 1.0
        obs[idx["target_speed_ratio"]] = 1.0
        # A right-angle turn ahead: radius about 0.5 m.
        for k, (x, y) in zip(("0.5", "1.0", "2.0"), ((0.5, 0.0), (0.85, -0.15), (1.0, -1.0))):
            obs[idx[f"ahead_{k}_x"]], obs[idx[f"ahead_{k}_y"]] = x, y
        free = C.build("pure_pursuit", envmod, _Params(car), 0.7, 4.0)
        limited = C.build("pure_pursuit", envmod, _Params(car), 0.7, 4.0, a_lat=5.0)
        self.assertEqual(free(obs)[1], 0.0, "at the target: hold")
        self.assertLess(limited(obs)[1], -0.5, "brake for the corner")


class CornerSpeed(unittest.TestCase):
    """The corner speed plan: slow for the route's corners before they come into view."""

    def test_the_profile_brakes_ahead_of_a_sharp_corner(self):
        # A right angle at 10 m on a 20 m route, a 1.5 m window: curvature
        # (pi/2) / 1.5 within 0.75 m of the corner.
        heading = lambda s: 0.0 if s < 10.0 else -0.5 * math.pi
        a_lat, a_brake, w = 3.0, 1.5, 1.5
        prof = C.speed_profile(heading, 20.0, 4.0, a_lat, a_brake, window=w, step=0.1)
        v_c = math.sqrt(a_lat / (0.5 * math.pi / w))
        at = lambda s: prof[round(s / 0.1)]
        self.assertAlmostEqual(at(10.0), v_c, places=9)
        self.assertAlmostEqual(at(9.3), v_c, places=9, msg="the whole window is the corner")
        # Before the window the cap rises as braking at a_brake allows ...
        self.assertAlmostEqual(at(8.3), math.sqrt(v_c ** 2 + 2.0 * a_brake * 1.0), places=6)
        # ... up to the target speed, far enough away; after the corner the car may speed up at once.
        self.assertEqual(at(0.0), 4.0)
        self.assertEqual(at(11.0), 4.0)
        self.assertEqual(C.speed_profile(lambda s: 0.3, 10.0, 4.0, a_lat, a_brake), [4.0] * 101, "straight")

    def test_after_a_corner_the_plan_rises_as_accelerating_allows(self):
        heading = lambda s: 0.0 if s < 10.0 else -0.5 * math.pi
        a_lat, a_brake, a_accel, w = 3.0, 1.5, 1.0, 1.5
        prof = C.speed_profile(heading, 20.0, 4.0, a_lat, a_brake, window=w, step=0.1, a_accel=a_accel)
        v_c = math.sqrt(a_lat / (0.5 * math.pi / w))
        at = lambda s: prof[round(s / 0.1)]
        self.assertAlmostEqual(at(10.7), v_c, places=9, msg="the corner's last point")
        self.assertAlmostEqual(at(11.7), math.sqrt(v_c ** 2 + 2.0 * a_accel * 1.0), places=6)
        self.assertEqual(at(19.9), 4.0, "the target speed again, far enough on")
        # Before the corner nothing changes, and without a_accel neither does the exit.
        before = C.speed_profile(heading, 20.0, 4.0, a_lat, a_brake, window=w, step=0.1)
        self.assertEqual(prof[:100], before[:100])
        self.assertEqual(before[round(11.7 / 0.1)], 4.0)

    def test_with_a_launch_speed_the_car_pulls_away_from_rest_at_a_accel_too(self):
        heading = lambda s: 0.0 if s < 10.0 else -0.5 * math.pi
        prof = C.speed_profile(heading, 20.0, 4.0, 3.0, 1.5, a_accel=1.0, v_launch=0.5)
        at = lambda s: prof[round(s / 0.1)]
        self.assertEqual(at(0.0), 0.5)
        self.assertAlmostEqual(at(2.0), math.sqrt(0.5 ** 2 + 2.0 * 1.0 * 2.0), places=6)
        free = C.speed_profile(heading, 20.0, 4.0, 3.0, 1.5, a_accel=1.0)
        self.assertEqual(free[0], 4.0, "without it, a trip starts at the target speed's cap")
        self.assertEqual(prof[100:], free[100:], "after the launch, the same plan")

    def test_the_plan_caps_the_drivers_throttle_and_never_raises_it(self):
        env = campus.CampusEnv(seed=0, ranges=envmod.RANGES_NOMINAL, lib_path=LIB, target_speed=4.0, task="stop",
                               car="unity")
        obs, _ = env.reset(seed=0, options={"trip": ("S3", "AB2")})
        idx = C._idx(envmod)
        floor = C.CornerSpeed(lambda o: (0.1, 0.4), env, a_lat=0.01, a_brake=0.01)   # crawls everywhere
        free = C.CornerSpeed(lambda o: (0.1, 0.4), env, a_lat=1e6, a_brake=1e6)      # never slows
        floor.reset()
        free.reset()
        fast = list(obs)
        fast[idx["speed_ratio"]] = 1.0
        self.assertEqual(free(fast), (0.1, 0.4))
        steer, thr = floor(fast)
        self.assertEqual(steer, 0.1)
        self.assertLess(thr, -0.9, "far over the plan: brake")
        env.close()

    def test_the_spec_drives_stop_trips_with_the_tuned_plan(self):
        spec = "recover:corner:tuned:controller:pure_pursuit:policies/controllers_unity_stop.json"
        rep = ev.evaluate([spec], trips=3, conditions=("train",), lib=LIB, workers=1, target_speed=4.0,
                          task="stop", car="unity")
        eps = rep["policies"][spec]["train"]["episodes"]
        self.assertEqual([e["status"] for e in eps], ["reached_end"] * 3)
        tuned = json.loads((ROOT / "policies" / "controllers_unity_stop.json").read_text(encoding="utf-8"))
        self.assertEqual(set(tuned["corner_speed"]["gains"]), {"a_lat", "a_brake", "a_accel", "v_launch"})


class _Params:
    """A dr.Params stand-in from a Nominal (the controllers read these fields)."""

    def __init__(self, n):
        self.mass, self.izz, self.lf, self.lr = n.m, n.iz, n.lf, n.lr
        self.c_alpha_f, self.c_alpha_r, self.steer_max = n.cf, n.cr, n.steer_max


class Statistics(unittest.TestCase):
    def test_wilson_and_mean_intervals(self):
        p, lo, hi = ev.wilson(50, 100)
        self.assertAlmostEqual(p, 0.5)
        self.assertAlmostEqual(lo, 0.4038, places=3)
        self.assertAlmostEqual(hi, 0.5962, places=3)
        self.assertEqual(ev.wilson(10, 10)[2], 1.0)
        m, lo, hi = ev.mean_ci([1.0, 2.0, 3.0, 4.0])
        self.assertAlmostEqual(m, 2.5)
        self.assertAlmostEqual(hi - m, 1.96 * math.sqrt(5 / 3) / 2, places=12)

    def test_the_harness_is_deterministic(self):
        a = ev.evaluate(["controller:lqr"], trips=3, conditions=("perturbed",), lib=LIB, workers=1, target_speed=4.0)
        b = ev.evaluate(["controller:lqr"], trips=3, conditions=("perturbed",), lib=LIB, workers=1, target_speed=4.0)
        self.assertEqual(a["policies"], b["policies"])


if __name__ == "__main__":
    unittest.main()
