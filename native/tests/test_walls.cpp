// Walls through the C ABI (ADR 0009): SlipX's segment contact after every
// Core step. Level ground, the provisional car at L2.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <initializer_list>

#include "check.h"
#include "rc_twin.h"

namespace {

const double kDt = 0.001;

rc_twin_params defaults() {
  rc_twin_params p;
  rc_twin_params_default(&p);
  return p;
}

rc_twin_vehicle* make(const rc_twin_params& p) {
  return rc_twin_create(RC_TWIN_TIER_L2_DOUBLE_TRACK, RC_TWIN_INTEGRATOR_RK4, &p,
                        nullptr);
}

void start(rc_twin_vehicle* v, double x, double y, double yaw, double vx) {
  rc_twin_params p;
  rc_twin_get_params(v, &p);
  rc_twin_state s;
  std::memset(&s, 0, sizeof s);
  s.x = x;
  s.y = y;
  s.yaw = yaw;
  s.vx = vx;
  s.soc = 1.0;
  s.pack_v = p.pack_v_full;
  for (int i = 0; i < 4; ++i) s.omega_w[i] = vx / p.wheel_radius;
  rc_twin_reset(v, &s);
}

rc_twin_state state(const rc_twin_vehicle* v) {
  rc_twin_state s;
  rc_twin_get_state(v, &s);
  return s;
}

rc_twin_wall_report report(const rc_twin_vehicle* v) {
  rc_twin_wall_report r;
  rc_twin_get_wall_report(v, &r);
  return r;
}

// The footprint's corners, world frame.
void corners(const rc_twin_vehicle* v, double out[4][2]) {
  rc_twin_contact c;
  rc_twin_get_contact(v, &c);
  const rc_twin_state s = state(v);
  const double fx = std::cos(s.yaw), fy = std::sin(s.yaw);
  const double cx = s.x + fx * c.centre_offset, cy = s.y + fy * c.centre_offset;
  const double sl[4] = {1, 1, -1, -1}, sw[4] = {1, -1, -1, 1};
  for (int i = 0; i < 4; ++i) {
    out[i][0] = cx + fx * c.half_length * sl[i] - fy * c.half_width * sw[i];
    out[i][1] = cy + fy * c.half_length * sl[i] + fx * c.half_width * sw[i];
  }
}

// A drive with steering and throttle changes, the same for every vehicle.
void drive(rc_twin_vehicle* v, int steps) {
  for (int k = 0; k < steps; ++k) {
    const double steer = 0.3 * std::sin(0.003 * k);
    const double accel = k < steps / 2 ? 3.0 : -2.0;
    CHECK(rc_twin_step(v, steer, accel, kDt, 1) == RC_TWIN_OK);
  }
}

void test_defaults_and_api() {
  const rc_twin_params p = defaults();
  rc_twin_contact c;
  CHECK(rc_twin_contact_default(&p, &c) == RC_TWIN_OK);
  CHECK(c.restitution == 0.3 && c.friction == 0.5 && c.restitution_min_speed == 0.1);
  CHECK(c.half_length == 0.5 * (p.lf + p.lr) + p.wheel_radius);
  CHECK(c.half_width == 0.5 * (p.track_front > p.track_rear ? p.track_front : p.track_rear));
  CHECK(c.centre_offset == 0.5 * (p.lf - p.lr));
  // The provisional car is symmetric (lf = lr, equal tracks), so check the
  // footprint rule on one that is not.
  rc_twin_params q = p;
  q.lf = 0.18;
  q.lr = 0.14;
  q.track_front = 0.22;
  q.track_rear = 0.25;
  rc_twin_contact cq;
  CHECK(rc_twin_contact_default(&q, &cq) == RC_TWIN_OK);
  CHECK(cq.centre_offset == 0.5 * (0.18 - 0.14));
  CHECK(cq.half_width == 0.5 * 0.25);
  CHECK(cq.half_length == 0.5 * (0.18 + 0.14) + q.wheel_radius);

  rc_twin_vehicle* v = make(p);
  CHECK(v != nullptr);
  rc_twin_contact got;
  CHECK(rc_twin_get_contact(v, &got) == RC_TWIN_OK);
  CHECK(std::memcmp(&got, &c, sizeof c) == 0);
  CHECK(rc_twin_wall_count(v) == 0);

  const double line[] = {0, 0, 1, 0, 1, 1};
  CHECK(rc_twin_add_wall(nullptr, line, 3, 0) == RC_TWIN_E_NULL);
  CHECK(rc_twin_add_wall(v, nullptr, 3, 0) == RC_TWIN_E_NULL);
  CHECK(rc_twin_add_wall(v, line, 1, 0) == RC_TWIN_E_INVALID);
  CHECK(rc_twin_add_wall(v, line, 2, 1) == RC_TWIN_E_INVALID);
  const double bad[] = {0, 0, NAN, 1};
  CHECK(rc_twin_add_wall(v, bad, 2, 0) == RC_TWIN_E_INVALID);
  CHECK(rc_twin_wall_count(v) == 0);   // all or nothing
  CHECK(rc_twin_add_wall(v, line, 3, 0) == RC_TWIN_OK);
  CHECK(rc_twin_wall_count(v) == 2);
  CHECK(rc_twin_add_wall(v, line, 3, 1) == RC_TWIN_OK);
  CHECK(rc_twin_wall_count(v) == 5);
  const double dup[] = {2, 2, 2, 2, 3, 2};
  CHECK(rc_twin_add_wall(v, dup, 3, 0) == RC_TWIN_OK);
  CHECK(rc_twin_wall_count(v) == 6);   // the zero-length segment is dropped

  // Walls survive reset and new params, as SlipX keeps them across reset().
  CHECK(rc_twin_reset(v, nullptr) == RC_TWIN_OK);
  CHECK(rc_twin_set_params(v, &p, nullptr) == RC_TWIN_OK);
  CHECK(rc_twin_wall_count(v) == 6);
  CHECK(rc_twin_clear_walls(v) == RC_TWIN_OK);
  CHECK(rc_twin_wall_count(v) == 0);
  CHECK(rc_twin_wall_count(nullptr) == 0);
  CHECK(rc_twin_clear_walls(nullptr) == RC_TWIN_E_NULL);

  rc_twin_contact x = c;
  x.restitution = 1.5;
  CHECK(rc_twin_set_contact(v, &x) == RC_TWIN_E_INVALID);
  x = c;
  x.half_width = 0.0;
  CHECK(rc_twin_set_contact(v, &x) == RC_TWIN_E_INVALID);
  x = c;
  x.friction = INFINITY;
  CHECK(rc_twin_set_contact(v, &x) == RC_TWIN_E_INVALID);
  x = c;
  x.friction = -0.1;
  CHECK(rc_twin_set_contact(v, &x) == RC_TWIN_E_INVALID);
  CHECK(rc_twin_get_contact(v, &got) == RC_TWIN_OK);
  CHECK(std::memcmp(&got, &c, sizeof c) == 0);   // refused, unchanged
  x = c;
  x.half_width += 0.02;
  CHECK(rc_twin_set_contact(v, &x) == RC_TWIN_OK);
  CHECK(rc_twin_set_params(v, &p, nullptr) == RC_TWIN_OK);
  CHECK(rc_twin_get_contact(v, &got) == RC_TWIN_OK);
  CHECK(got.half_width == c.half_width + 0.02);   // set_params keeps it
  CHECK(rc_twin_set_contact(nullptr, &x) == RC_TWIN_E_NULL);
  CHECK(rc_twin_get_wall_report(v, nullptr) == RC_TWIN_E_NULL);
  rc_twin_destroy(v);
}

void test_no_walls_is_bit_identical() {
  const rc_twin_params p = defaults();
  rc_twin_vehicle* plain = make(p);
  rc_twin_vehicle* cleared = make(p);
  rc_twin_vehicle* far = make(p);
  const double wall[] = {-1, 50, 1, 50};
  rc_twin_add_wall(cleared, wall, 2, 0);
  rc_twin_clear_walls(cleared);
  rc_twin_add_wall(far, wall, 2, 0);   // never reached
  for (rc_twin_vehicle* v : {plain, cleared, far}) {
    start(v, 0.0, 0.0, 0.4, 0.5);
    drive(v, 3000);
  }
  const rc_twin_state a = state(plain), b = state(cleared), c = state(far);
  CHECK(std::memcmp(&a, &b, sizeof a) == 0);
  CHECK(std::memcmp(&a, &c, sizeof a) == 0);
  CHECK(report(far).contacts == 0);
  for (rc_twin_vehicle* v : {plain, cleared, far}) rc_twin_destroy(v);
}

void test_head_on_rebounds_at_restitution() {
  rc_twin_vehicle* v = make(defaults());
  rc_twin_contact c;
  rc_twin_get_contact(v, &c);
  const double front = c.centre_offset + c.half_length;
  const double wall_x = front + 0.05;
  const double wall[] = {wall_x, -1.0, wall_x, 1.0};
  rc_twin_add_wall(v, wall, 2, 0);
  start(v, 0.0, 0.0, 0.0, 1.0);

  bool hit = false;
  for (int k = 0; k < 2000; ++k) {
    CHECK(rc_twin_step(v, 0.0, 0.0, kDt, 1) == RC_TWIN_OK);
    const rc_twin_state s = state(v);
    CHECK(s.x + front <= wall_x + 1e-12);   // never through
    const rc_twin_wall_report r = report(v);
    if (r.contacts > 0 && !hit) {
      hit = true;
      CHECK(r.contacts == 1);
      CHECK(r.max_approach > 0.9);
      // Centred and square on: the impulse is along x through the CoG, so
      // the car leaves at -e times its approach speed and does not turn.
      CHECK(std::fabs(s.vx + 0.3 * r.max_approach) < 1e-12);
      CHECK(s.yaw_rate == 0.0);
      CHECK(std::fabs(s.vy) < 1e-12);
      const double m = defaults().mass;
      CHECK(std::fabs(r.impulse - 1.3 * m * r.max_approach) < 1e-9);
    }
  }
  CHECK(hit);
  CHECK(state(v).x < 0.05);   // it bounced back
  rc_twin_destroy(v);
}

void test_glancing_hit_stays_in_the_friction_cone() {
  rc_twin_vehicle* v = make(defaults());
  const double wall_y = 0.4;
  const double wall[] = {-5.0, wall_y, 5.0, wall_y};
  rc_twin_add_wall(v, wall, 2, 0);
  const double yaw = 0.5;   // about 29 degrees into the wall
  start(v, 0.0, 0.0, yaw, 1.5);
  bool hit = false;
  for (int k = 0; k < 3000 && !hit; ++k) {
    const rc_twin_state before = state(v);
    CHECK(rc_twin_step(v, 0.0, 0.0, kDt, 1) == RC_TWIN_OK);
    const rc_twin_wall_report r = report(v);
    if (r.contacts == 0) continue;
    hit = true;
    const rc_twin_state s = state(v);
    // World CoG velocity before (approximately: one Core step earlier) and
    // after; the normal is -y, the tangent x.
    const double vy0 = std::sin(before.yaw) * before.vx + std::cos(before.yaw) * before.vy;
    const double vx0 = std::cos(before.yaw) * before.vx - std::sin(before.yaw) * before.vy;
    const double vy1 = std::sin(s.yaw) * s.vx + std::cos(s.yaw) * s.vy;
    const double vx1 = std::cos(s.yaw) * s.vx - std::sin(s.yaw) * s.vy;
    CHECK(vy0 > 0.0);
    CHECK(vy1 < vy0);                       // pushed back off the wall
    const double dn = vy0 - vy1, dt = std::fabs(vx1 - vx0);
    CHECK(dt <= 0.5 * dn + 1e-3);           // Coulomb, up to one Core step
    CHECK(std::fabs(s.yaw_rate - before.yaw_rate) > 0.1);   // a corner hit spins it
  }
  CHECK(hit);
  for (int k = 0; k < 2000; ++k) {
    CHECK(rc_twin_step(v, 0.0, 1.0, kDt, 1) == RC_TWIN_OK);
    double cs[4][2];
    corners(v, cs);
    for (auto& q : cs) CHECK(q[1] <= wall_y + 1e-9);
  }
  rc_twin_destroy(v);
}

void test_mirror_symmetry_bit_for_bit() {
  rc_twin_vehicle* a = make(defaults());
  rc_twin_vehicle* b = make(defaults());
  const double wa[] = {-5.0, 0.4, 5.0, 0.4};
  const double wb[] = {-5.0, -0.4, 5.0, -0.4};
  rc_twin_add_wall(a, wa, 2, 0);
  rc_twin_add_wall(b, wb, 2, 0);
  start(a, 0.0, 0.0, 0.5, 1.5);
  start(b, 0.0, 0.0, -0.5, 1.5);
  uint32_t hits = 0;
  for (int k = 0; k < 2000; ++k) {
    const double steer = 0.1 * std::sin(0.01 * k);
    CHECK(rc_twin_step(a, steer, 1.0, kDt, 1) == RC_TWIN_OK);
    CHECK(rc_twin_step(b, -steer, 1.0, kDt, 1) == RC_TWIN_OK);
    hits += report(a).contacts;
    CHECK(report(a).contacts == report(b).contacts);
  }
  const rc_twin_state sa = state(a), sb = state(b);
  CHECK(hits > 0);
  CHECK(sa.x == sb.x && sa.y == -sb.y && sa.yaw == -sb.yaw);
  CHECK(sa.vx == sb.vx && sa.vy == -sb.vy && sa.yaw_rate == -sb.yaw_rate);
  rc_twin_destroy(a);
  rc_twin_destroy(b);
}

void test_a_closed_box_holds_the_car() {
  rc_twin_vehicle* v = make(defaults());
  const double h = 1.5;
  const double box[] = {-h, -h, h, -h, h, h, -h, h};
  CHECK(rc_twin_add_wall(v, box, 4, 1) == RC_TWIN_OK);
  start(v, 0.0, 0.0, 0.3, 0.0);
  uint32_t hits = 0;
  double worst = 0.0;
  for (int k = 0; k < 10000; ++k) {
    const double steer = (k / 1500) % 2 == 0 ? 0.15 : -0.35;
    CHECK(rc_twin_step(v, steer, 6.0, kDt, 1) == RC_TWIN_OK);
    hits += report(v).contacts;
    const rc_twin_state s = state(v);
    CHECK(std::fabs(s.x) < h && std::fabs(s.y) < h);
    double cs[4][2];
    corners(v, cs);
    for (auto& q : cs) {
      worst = std::fmax(worst, std::fmax(std::fabs(q[0]), std::fabs(q[1])) - h);
    }
  }
  CHECK(hits > 10);
  // Two walls resolved in turn at a corner can leave a sliver; under 1 mm.
  std::printf("closed box: %u contacts, worst corner past the wall %.2e m\n", hits, worst);
  CHECK(worst < 1e-3);
  rc_twin_destroy(v);
}

void test_report_is_per_call() {
  rc_twin_vehicle* v = make(defaults());
  rc_twin_contact c;
  rc_twin_get_contact(v, &c);
  const double wall_x = c.centre_offset + c.half_length + 0.002;
  const double wall[] = {wall_x, -1.0, wall_x, 1.0};
  rc_twin_add_wall(v, wall, 2, 0);
  start(v, 0.0, 0.0, 0.0, 1.0);
  CHECK(rc_twin_step(v, 0.0, 0.0, kDt, 10) == RC_TWIN_OK);   // 10 steps, 1 cm
  CHECK(report(v).contacts == 1);
  CHECK(rc_twin_step(v, 0.0, 0.0, kDt, 1) == RC_TWIN_OK);    // backing off
  CHECK(report(v).contacts == 0 && report(v).impulse == 0.0);
  rc_twin_destroy(v);
}

// Sliding sideways into a wall: the lateral velocity rebounds at e.
void test_a_sideways_hit_rebounds_too() {
  rc_twin_vehicle* v = make(defaults());
  rc_twin_contact c;
  rc_twin_get_contact(v, &c);
  const double wall_y = c.half_width + 0.03;
  const double wall[] = {-2.0, wall_y, 2.0, wall_y};
  rc_twin_add_wall(v, wall, 2, 0);
  rc_twin_state s;
  std::memset(&s, 0, sizeof s);
  s.vy = 1.0;   // drifting left, square to the wall
  s.soc = 1.0;
  s.pack_v = defaults().pack_v_full;
  rc_twin_reset(v, &s);
  bool hit = false;
  for (int k = 0; k < 300 && !hit; ++k) {
    CHECK(rc_twin_step(v, 0.0, 0.0, kDt, 1) == RC_TWIN_OK);
    const rc_twin_wall_report r = report(v);
    if (r.contacts == 0) continue;
    hit = true;
    const rc_twin_state t = state(v);
    CHECK(std::fabs(t.vy + 0.3 * r.max_approach) < 1e-9);
    CHECK(t.y + c.half_width <= wall_y + 1e-12);
  }
  CHECK(hit);
  rc_twin_destroy(v);
}

// A footprint centred well ahead of the CoG is what hits the wall.
void test_the_footprint_offset_counts() {
  rc_twin_vehicle* v = make(defaults());
  rc_twin_contact c;
  rc_twin_get_contact(v, &c);
  c.centre_offset = 0.2;   // a long nose
  CHECK(rc_twin_set_contact(v, &c) == RC_TWIN_OK);
  const double front = c.centre_offset + c.half_length;
  const double wall[] = {front + 0.05, -1.0, front + 0.05, 1.0};
  rc_twin_add_wall(v, wall, 2, 0);
  start(v, 0.0, 0.0, 0.0, 1.0);
  uint32_t hits = 0;
  for (int k = 0; k < 500; ++k) {
    CHECK(rc_twin_step(v, 0.0, 0.0, kDt, 1) == RC_TWIN_OK);
    hits += report(v).contacts;
    CHECK(state(v).x + front <= front + 0.05 + 1e-12);
  }
  CHECK(hits >= 1);
  rc_twin_destroy(v);
}

// Friction at the wall takes speed along it: the same glancing hit with the
// contact friction at zero leaves the car faster along the wall.
void test_friction_takes_speed_along_the_wall() {
  double along[2];
  for (int f = 0; f < 2; ++f) {
    rc_twin_vehicle* v = make(defaults());
    rc_twin_contact c;
    rc_twin_get_contact(v, &c);
    c.friction = f == 0 ? 0.5 : 0.0;
    rc_twin_set_contact(v, &c);
    const double wall[] = {-5.0, 0.4, 5.0, 0.4};
    rc_twin_add_wall(v, wall, 2, 0);
    start(v, 0.0, 0.0, 0.5, 1.5);
    for (int k = 0; k < 3000; ++k) {
      CHECK(rc_twin_step(v, 0.0, 0.0, kDt, 1) == RC_TWIN_OK);
      if (report(v).contacts > 0) break;
    }
    const rc_twin_state s = state(v);
    along[f] = std::cos(s.yaw) * s.vx - std::sin(s.yaw) * s.vy;
    rc_twin_destroy(v);
  }
  CHECK(along[0] < along[1] - 0.01);
}

// Level ground through the query: the wall pass runs on that path too.
int32_t level(void*, int32_t, double, double, rc_twin_ground_sample*) { return 0; }

void test_walls_act_with_a_ground_query() {
  rc_twin_vehicle* v = make(defaults());
  rc_twin_set_ground_query(v, level, nullptr);
  rc_twin_contact c;
  rc_twin_get_contact(v, &c);
  const double front = c.centre_offset + c.half_length;
  const double wall[] = {front + 0.05, -1.0, front + 0.05, 1.0};
  rc_twin_add_wall(v, wall, 2, 0);
  start(v, 0.0, 0.0, 0.0, 1.0);
  uint32_t hits = 0;
  for (int k = 0; k < 500; ++k) {
    CHECK(rc_twin_step(v, 0.0, 0.0, kDt, 1) == RC_TWIN_OK);
    hits += report(v).contacts;
    CHECK(state(v).x + front <= front + 0.05 + 1e-12);
  }
  CHECK(hits >= 1);
  CHECK(state(v).vx < 0.0);
  rc_twin_destroy(v);
}

}  // namespace

int main() {
  test_defaults_and_api();
  test_no_walls_is_bit_identical();
  test_head_on_rebounds_at_restitution();
  test_glancing_hit_stays_in_the_friction_cone();
  test_mirror_symmetry_bit_for_bit();
  test_a_closed_box_holds_the_car();
  test_report_is_per_call();
  test_walls_act_with_a_ground_query();
  test_a_sideways_hit_rebounds_too();
  test_the_footprint_offset_counts();
  test_friction_takes_speed_along_the_wall();
  TEST_MAIN_END();
}
