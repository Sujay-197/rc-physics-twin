"""Classical path-following controllers, as policies: obs -> (steer, throttle).

Each uses only what a real controller on the car could: the observation
(rc_twin_env.FRAME_FIELDS, current frame) and the nominal car (SlipX's
provisional defaults: geometry, mass, yaw inertia, axle cornering
stiffnesses, steer limit), never the episode's randomised car. They share one
speed loop, so a comparison between them is a comparison of lateral control.

  PurePursuit   steer toward a lookahead point on the centreline
  Stanley       front-axle cross-track and heading error, curvature feedforward
  LQR           discrete LQR on the dynamic-bicycle lateral error model,
                gain-scheduled on speed, with steady-state feedforward
                (Rajamani, Vehicle Dynamics and Control, 2nd ed., sec. 3.2-3.3)

Gains are tuned by tools/tune_controllers.py on the training randomisation
and frozen in policies/controllers.json; see sim_py/evaluate_policies.py for
the comparison. Standard library only.
"""

from __future__ import annotations

import math

_FIELDS = None


def _idx(envmod):
    global _FIELDS
    if _FIELDS is None:
        f = envmod.FRAME_FIELDS
        _FIELDS = {k: f.index(k) for k in f}
    return _FIELDS


def _clip(x: float, lo: float = -1.0, hi: float = 1.0) -> float:
    return lo if x < lo else hi if x > hi else x


def chord_heading(pts) -> float:
    """The car's heading relative to the road just ahead: minus the angle of
    the chord from the 0.5 m to the 1 m look-ahead point, in the car's frame.

    The nearest-point heading error asks a tracking controller to turn the
    full angle of a corner at its vertex; the fleet sim's routes turn 90
    degrees at junction nodes, tighter than the car can turn inside the road,
    so a controller following that error ends up in the outer kerb. The chord
    starts turning before the corner, as a driver does."""
    (x0, y0), (x1, y1) = pts[0], pts[1]
    return -math.atan2(y1 - y0, x1 - x0)


def path_curvature(p0, p1, p2) -> float:
    """Signed curvature of the circle through three points (left positive);
    0 if they are collinear."""
    (x0, y0), (x1, y1), (x2, y2) = p0, p1, p2
    a = math.dist(p0, p1)
    b = math.dist(p1, p2)
    c = math.dist(p0, p2)
    cross = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0)
    if a * b * c < 1e-12:
        return 0.0
    return 2.0 * cross / (a * b * c)


class Nominal:
    """The car a controller is designed for: SlipX's provisional defaults."""

    def __init__(self, p):
        self.m, self.iz = p.mass, p.izz
        self.lf, self.lr = p.lf, p.lr
        self.L = p.lf + p.lr
        self.cf, self.cr = p.c_alpha_f, p.c_alpha_r   # per axle [N/rad]
        self.steer_max = p.steer_max


class _Base:
    """Shared plumbing: the observation's fields and the speed loop."""

    def __init__(self, envmod, nominal: Nominal, half_width: float, k_speed: float = 3.0,
                 a_lat: float | None = None, a_stop: float = 1.2, stop_margin: float = 0.1):
        self.i = _idx(envmod)
        # "stop" task: planned braking [m/s^2], below what the weakest (held-out)
        # cars can do, aimed stop_margin [m] short of the end. Provisional car.
        self.a_stop, self.stop_margin = a_stop, stop_margin
        self.remaining_scale = getattr(envmod, "REMAINING_SCALE_M", None)
        self.i_rem = None      # set by build() for the "stop" task
        self.car = nominal
        self.hw = half_width
        self.k_speed = k_speed
        self.a_lat = a_lat   # cornering limit [m/s^2]: slow to sqrt(a_lat / |kappa|); None: never

    def _state(self, obs):
        i = self.i
        v_ref_ratio = obs[i["target_speed_ratio"]]
        v_ratio = obs[i["speed_ratio"]]
        return {
            "remaining": (obs[self.i_rem] * self.remaining_scale
                          if self.i_rem is not None and obs[self.i_rem] < 1.0 else None),
            "v_ratio": v_ratio,
            "v_ref_ratio": v_ref_ratio,
            "vy": obs[i["lateral_velocity"]],
            "r": obs[i["yaw_rate"]],
            "e": obs[i["offset_ratio"]] * self.hw,    # left of the centreline, m
            "psi": obs[i["heading_error"]],            # car yaw - road yaw, rad
            "pts": [(obs[i[f"ahead_{a}_x"]], obs[i[f"ahead_{a}_y"]]) for a in ("0.5", "1.0", "2.0")],
        }

    def _throttle(self, s) -> float:
        """Hold the target speed, or less where the road ahead curves more
        than the cornering limit allows at that speed."""
        goal = s["v_ref_ratio"]
        if s["remaining"] is not None:
            # Stop at the end: no faster than braking at a_stop allows.
            rem = max(0.0, s["remaining"] - self.stop_margin)
            goal = min(goal, math.sqrt(2.0 * self.a_stop * rem) / self.v_ref * s["v_ref_ratio"])
        if self.a_lat:
            k = abs(path_curvature(*s["pts"]))
            if k > 1e-6:
                goal = min(goal, math.sqrt(self.a_lat / k) / self.v_ref * s["v_ref_ratio"])
        u = _clip(self.k_speed * (goal - s["v_ratio"]))
        if s["remaining"] is not None and u < 0.0 and s["v_ratio"] <= 0.01:
            u = 0.0   # braking at rest would reverse
        return u

    def _steer_out(self, delta: float) -> float:
        return _clip(delta / self.car.steer_max)


class PurePursuit(_Base):
    def __init__(self, envmod, nominal, half_width, lookahead: float = 1.0, **kw):
        super().__init__(envmod, nominal, half_width, **kw)
        self.lookahead = lookahead
        self.name = f"pure_pursuit(ld={lookahead:g})"

    def __call__(self, obs):
        s = self._state(obs)
        # The centreline point at the lookahead distance, interpolated along
        # the three observed points (0.5, 1, 2 m ahead along the road).
        pts, d = s["pts"], self.lookahead
        if d <= 0.5:
            ax, ay = pts[0]
        elif d <= 1.0:
            t = (d - 0.5) / 0.5
            ax, ay = pts[0][0] + t * (pts[1][0] - pts[0][0]), pts[0][1] + t * (pts[1][1] - pts[0][1])
        else:
            t = min(1.0, (d - 1.0) / 1.0)
            ax, ay = pts[1][0] + t * (pts[2][0] - pts[1][0]), pts[1][1] + t * (pts[2][1] - pts[1][1])
        ld2 = ax * ax + ay * ay
        delta = math.atan2(2.0 * self.car.L * ay, ld2)
        return self._steer_out(delta), self._throttle(s)


class Stanley(_Base):
    def __init__(self, envmod, nominal, half_width, k: float = 2.0, k_soft: float = 0.5,
                 k_damp: float = 0.0, reference: str = "chord", **kw):
        super().__init__(envmod, nominal, half_width, **kw)
        self.k, self.k_soft, self.k_damp, self.reference = k, k_soft, k_damp, reference
        self.name = f"stanley(k={k:g},soft={k_soft:g},damp={k_damp:g},{reference})"

    def __call__(self, obs):
        s = self._state(obs)
        v = s["v_ratio"] / max(s["v_ref_ratio"], 1e-9) * self.v_ref
        e, psi, kappa = _errors(s, self.reference)
        e_front = e + self.car.lf * math.sin(psi)
        delta = (math.atan(self.car.L * kappa)          # follow the road's curve
                 - psi                                   # align with it
                 - math.atan2(self.k * e_front, self.k_soft + abs(v))
                 - self.k_damp * (s["r"] - v * kappa))   # damp yaw relative to the road
        return self._steer_out(delta), self._throttle(s)


class LQR(_Base):
    """Discrete LQR on the lateral error state x = [e, e', psi, psi'] (e left
    of the centreline, psi = yaw - road yaw), per axle cornering stiffness,
    zero-order hold at the control period. Gains are solved for a grid of
    speeds and interpolated."""

    SPEEDS = (0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0)

    def __init__(self, envmod, nominal, half_width, dt: float, q=(100.0, 1.0, 10.0, 0.1), r: float = 1.0,
                 reference: str = "chord", **kw):
        super().__init__(envmod, nominal, half_width, **kw)
        self.q, self.r, self.reference = tuple(q), r, reference
        self.dt = dt
        self.gains = [(v, _dlqr(*_discretise(*_bicycle(nominal, v), dt), q, r)) for v in self.SPEEDS]
        self.name = f"lqr(q={'/'.join(f'{x:g}' for x in q)},r={r:g},{reference})"

    def gain(self, v: float):
        g = self.gains
        if v <= g[0][0]:
            return g[0][1]
        for (v0, k0), (v1, k1) in zip(g, g[1:]):
            if v <= v1:
                t = (v - v0) / (v1 - v0)
                return [a + t * (b - a) for a, b in zip(k0, k1)]
        return g[-1][1]

    def __call__(self, obs):
        s = self._state(obs)
        car = self.car
        v = max(0.3, s["v_ratio"] / max(s["v_ref_ratio"], 1e-9) * self.v_ref)
        e, psi, kappa = _errors(s, self.reference)
        e_dot = s["vy"] + v * math.sin(psi)
        psi_dot = s["r"] - v * kappa
        k = self.gain(v)
        fb = -(k[0] * e + k[1] * e_dot + k[2] * psi + k[3] * psi_dot)
        # Steady-state feedforward (Rajamani eq. 3.12, per-axle stiffness).
        kv = car.m * car.lr / (car.cf * car.L) - car.m * car.lf / (car.cr * car.L)
        ff = (car.L * kappa + kv * v * v * kappa
              - k[2] * (car.lr * kappa - car.lf * car.m * v * v * kappa / (car.cr * car.L)))
        return self._steer_out(fb + ff), self._throttle(s)


def _errors(s, reference: str):
    """(e, psi, kappa): the offset from the nearest centreline point, the
    heading error from the chord just ahead ("chord") or from the nearest
    point ("nearest"), the curvature of the circle through the look-ahead
    points."""
    psi = chord_heading(s["pts"]) if reference == "chord" else s["psi"]
    return s["e"], psi, path_curvature(*s["pts"])


# ------------------------------------------------------------- linear algebra
# Small dense matrices as lists of rows; enough for a 4-state model.

def _mm(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(len(b))) for j in range(len(b[0]))] for i in range(len(a))]


def _madd(a, b, s: float = 1.0):
    return [[a[i][j] + s * b[i][j] for j in range(len(a[0]))] for i in range(len(a))]


def _mT(a):
    return [list(r) for r in zip(*a)]


def _eye(n):
    return [[1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]


def _bicycle(car: Nominal, v: float):
    cf, cr, m, iz, lf, lr = car.cf, car.cr, car.m, car.iz, car.lf, car.lr
    A = [[0.0, 1.0, 0.0, 0.0],
         [0.0, -(cf + cr) / (m * v), (cf + cr) / m, (-cf * lf + cr * lr) / (m * v)],
         [0.0, 0.0, 0.0, 1.0],
         [0.0, -(cf * lf - cr * lr) / (iz * v), (cf * lf - cr * lr) / iz, -(cf * lf * lf + cr * lr * lr) / (iz * v)]]
    B = [[0.0], [cf / m], [0.0], [cf * lf / iz]]
    return A, B


def _discretise(A, B, dt: float, terms: int = 12):
    """Zero-order hold: Ad = exp(A dt), Bd = sum A^k dt^(k+1)/(k+1)! B."""
    n = len(A)
    Ad, Sd = _eye(n), [[dt if i == j else 0.0 for j in range(n)] for i in range(n)]
    term = _eye(n)
    for k in range(1, terms):
        term = [[x * dt / k for x in row] for row in _mm(term, A)]       # (A dt)^k / k!
        Ad = _madd(Ad, term)
        Sd = _madd(Sd, [[x * dt / (k + 1) for x in row] for row in term])
    return Ad, _mm(Sd, B)


def _dlqr(Ad, Bd, q, r: float, iters: int = 5000, tol: float = 1e-10):
    """Discrete-time LQR gain for one input, by Riccati iteration."""
    n = len(Ad)
    Q = [[q[i] if i == j else 0.0 for j in range(n)] for i in range(n)]
    P = [row[:] for row in Q]
    AT, BT = _mT(Ad), _mT(Bd)
    for _ in range(iters):
        BtP = _mm(BT, P)
        s = r + _mm(BtP, Bd)[0][0]
        K = [[x / s for x in _mm(BtP, Ad)[0]]]                          # (R + B'PB)^-1 B'PA
        Pn = _madd(Q, _mm(_mm(AT, P), _madd(Ad, _mm(Bd, K), -1.0)))
        if max(abs(Pn[i][j] - P[i][j]) for i in range(n) for j in range(n)) < tol * (1.0 + abs(P[0][0])):
            P = Pn
            break
        P = Pn
    BtP = _mm(BT, P)
    s = r + _mm(BtP, Bd)[0][0]
    return [x / s for x in _mm(BtP, Ad)[0]]


# ------------------------------------------------------------- kerb recovery

class KerbRecovery:
    """Back off a kerb the car is pinned against, then let the driver retry.

    On the Unity twin's car a car that strikes a kerb at a junction nose
    first stays pinned against it at full throttle until the trip times out
    (Docs/policy-study.md, "Closing the gap to the Unity car"). The driver
    cannot tell: the observation has no wall contact, and its lookahead
    point is still round the corner. Pinned, the car is not still: it
    bounces off the kerb and back at a few tenths of a metre a second. So:
    when the driver has asked for throttle above `min_throttle` for
    `stall_time` and the car has moved less than `stall_distance` in that
    time (its speed, integrated), reverse with the steering on the other
    lock, which swings the nose toward the way the driver was steering, for
    `back_distance` or at most `back_time`, then hand back. A negative
    demand held at rest reverses (on the wheel-inertia car, through the
    ESC's reverse latch, ADR 0010).

    Wraps any policy obs -> (steer, throttle); `back_offs` counts them per
    trip. unity/.../Drivers.cs KerbRecovery is this, line for line."""

    def __init__(self, inner, envmod, target_speed: float, stall_distance: float = 0.05, stall_time: float = 0.5,
                 min_throttle: float = 0.3, back_throttle: float = -0.6, back_distance: float = 0.3,
                 back_time: float = 1.5):
        self.inner = inner
        self.tick = envmod.TICK
        self.i_v = envmod.FRAME_FIELDS.index("speed_ratio")
        self.v_ref = target_speed
        self.stall_distance, self.stall_time, self.min_throttle = stall_distance, stall_time, min_throttle
        self.back_throttle, self.back_distance, self.back_time = back_throttle, back_distance, back_time
        self.name = f"{getattr(inner, 'name', 'policy')}+kerb_recovery"
        self.reset()

    def reset(self):
        self.stalled = 0.0       # how long the driver has asked for throttle ... [s]
        self.moved = 0.0         # ... and how far the car has gone in that time [m]
        self.backing = None      # while backing off: [steer, time, distance]
        self.back_offs = 0
        if hasattr(self.inner, "reset"):
            self.inner.reset()

    def __call__(self, obs):
        steer, throttle = self.inner(obs)   # every tick, so a driver with memory keeps it
        v = obs[self.i_v] * self.v_ref
        if self.backing is not None:
            b = self.backing
            b[1] += self.tick
            b[2] += max(0.0, -v) * self.tick
            if b[1] < self.back_time and b[2] < self.back_distance:
                return b[0], self.back_throttle
            self.backing = None
        if throttle > self.min_throttle:
            self.stalled += self.tick
            self.moved += v * self.tick
            if abs(self.moved) > self.stall_distance:
                self.stalled = self.moved = 0.0
        else:
            self.stalled = self.moved = 0.0
        if self.stalled >= self.stall_time:
            self.stalled = self.moved = 0.0
            self.back_offs += 1
            lock = 0.0 if abs(steer) < 0.05 else (-1.0 if steer > 0.0 else 1.0)
            self.backing = [lock, 0.0, 0.0]
            return lock, self.back_throttle
        return steer, throttle


# ---------------------------------------------------------------- factories

KINDS = {"pure_pursuit": PurePursuit, "stanley": Stanley, "lqr": LQR}


def build(kind: str, envmod, params, half_width: float, target_speed: float, task: str = "follow", **gains):
    """A controller of `kind` for the nominal car `params` (dr.Params)."""
    nominal = Nominal(params)
    extra = {"dt": envmod.TICK} if kind == "lqr" else {}
    c = KINDS[kind](envmod, nominal, half_width, **extra, **gains)
    c.v_ref = target_speed
    if task == "stop":
        c.i_rem = envmod.STOP_FIELDS.index("remaining_ratio")
    return c
