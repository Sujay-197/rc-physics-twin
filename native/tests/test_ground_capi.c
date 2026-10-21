/* The ground query through the C ABI (ADR 0005), from plain C. */
#include <math.h>
#include <string.h>

#include "check.h"
#include "rc_twin.h"

typedef struct {
  int calls;
  int fail_at;           /* call index to fail at, or -1 */
  double slope_x;        /* road descends toward +x with this tangent */
  double bank;           /* left (+y) side higher with this tangent */
  double mu;
  double height;
  double nz;             /* override nz when not 0 (to make it invalid) */
  int wheel[16];
  double x[16], y[16];
} ground_t;

static int32_t query(void* user, int32_t wheel, double x, double y,
                     rc_twin_ground_sample* out) {
  ground_t* g = (ground_t*)user;
  if (g->calls < 16) {
    g->wheel[g->calls] = wheel;
    g->x[g->calls] = x;
    g->y[g->calls] = y;
  }
  if (g->calls == g->fail_at) { g->calls++; return 1; }
  g->calls++;
  out->nx = g->slope_x;
  out->ny = -g->bank;
  out->nz = g->nz != 0.0 ? g->nz : 1.0;
  out->mu_scale = g->mu;
  out->height = g->height;
  return 0;
}

static ground_t level(void) {
  ground_t g;
  memset(&g, 0, sizeof g);
  g.fail_at = -1;
  g.mu = 1.0;
  return g;
}

static rc_twin_vehicle* make(int32_t tier) {
  rc_twin_params p;
  rc_twin_params_default(&p);
  return rc_twin_create(tier, RC_TWIN_INTEGRATOR_RK4, &p, NULL);
}

static void moving(rc_twin_vehicle* v, double vx, double yaw) {
  rc_twin_state s;
  memset(&s, 0, sizeof s);
  s.vx = vx;
  s.yaw = yaw;
  s.soc = 1.0;
  s.pack_v = 11.1;
  rc_twin_reset(v, &s);
}

static void test_abi_v2(void) {
  CHECK(rc_twin_abi_version() >= 2u);
  CHECK(rc_twin_set_ground_query(NULL, query, NULL) == RC_TWIN_E_NULL);
}

static void test_contact_points_and_order(void) {
  rc_twin_params p;
  rc_twin_vehicle* v = make(RC_TWIN_TIER_L2_DOUBLE_TRACK);
  ground_t g = level();
  int i;
  rc_twin_params_default(&p);
  rc_twin_set_ground_query(v, query, &g);

  /* At the origin facing +x: FL is ahead and to the left. */
  CHECK(rc_twin_step(v, 0.0, 0.0, 0.001, 1) == RC_TWIN_OK);
  CHECK(g.calls == 4);
  for (i = 0; i < 4; ++i) CHECK(g.wheel[i] == i);
  CHECK(fabs(g.x[0] - p.lf) < 1e-12 && fabs(g.y[0] - 0.5 * p.track_front) < 1e-12);
  CHECK(fabs(g.x[1] - p.lf) < 1e-12 && fabs(g.y[1] + 0.5 * p.track_front) < 1e-12);
  CHECK(fabs(g.x[2] + p.lr) < 1e-12 && fabs(g.y[2] - 0.5 * p.track_rear) < 1e-12);
  CHECK(fabs(g.x[3] + p.lr) < 1e-12 && fabs(g.y[3] + 0.5 * p.track_rear) < 1e-12);

  /* Facing +y (yaw 90 deg): FL is at (-track/2, lf). */
  g = level();
  moving(v, 0.0, 1.5707963267948966);
  rc_twin_step(v, 0.0, 0.0, 0.001, 1);
  CHECK(fabs(g.x[0] + 0.5 * p.track_front) < 1e-12 && fabs(g.y[0] - p.lf) < 1e-12);

  /* n substeps ask 4 n times. */
  g = level();
  rc_twin_step(v, 0.0, 0.0, 0.001, 3);
  CHECK(g.calls == 12);
  rc_twin_destroy(v);
}

static void test_level_query_matches_no_query(void) {
  rc_twin_vehicle* a = make(RC_TWIN_TIER_L2_DOUBLE_TRACK);
  rc_twin_vehicle* b = make(RC_TWIN_TIER_L2_DOUBLE_TRACK);
  ground_t g = level();
  rc_twin_state sa, sb;
  rc_twin_set_ground_query(b, query, &g);
  rc_twin_step(a, 0.2, 3.0, 0.001, 800);
  rc_twin_step(b, 0.2, 3.0, 0.001, 800);
  rc_twin_get_state(a, &sa);
  rc_twin_get_state(b, &sb);
  CHECK(memcmp(&sa, &sb, sizeof sa) == 0);
  rc_twin_destroy(a);
  rc_twin_destroy(b);
}

static void test_grade_grip_and_height_reach_the_core(void) {
  rc_twin_vehicle* flat = make(RC_TWIN_TIER_L2_DOUBLE_TRACK);
  rc_twin_vehicle* hill = make(RC_TWIN_TIER_L2_DOUBLE_TRACK);
  rc_twin_vehicle* ice = make(RC_TWIN_TIER_L2_DOUBLE_TRACK);
  ground_t gh = level(), gi = level();
  rc_twin_state sf, sh, si;
  gh.slope_x = 0.05;  /* downhill ahead */
  gh.height = 2.5;
  gi.mu = 0.1;
  rc_twin_set_ground_query(hill, query, &gh);
  rc_twin_set_ground_query(ice, query, &gi);
  moving(flat, 3.0, 0.0);
  moving(hill, 3.0, 0.0);
  moving(ice, 3.0, 0.0);
  CHECK(rc_twin_step(flat, 0.0, 8.0, 0.001, 300) == RC_TWIN_OK);
  CHECK(rc_twin_step(hill, 0.0, 8.0, 0.001, 300) == RC_TWIN_OK);
  CHECK(rc_twin_step(ice, 0.0, 8.0, 0.001, 300) == RC_TWIN_OK);
  rc_twin_get_state(flat, &sf);
  rc_twin_get_state(hill, &sh);
  rc_twin_get_state(ice, &si);
  CHECK(sh.vx > sf.vx);          /* gravity helps downhill */
  CHECK(sh.z == 2.5);            /* mean contact height */
  CHECK(si.vx < sf.vx - 0.5);    /* little traction on ice */
  rc_twin_destroy(flat);
  rc_twin_destroy(hill);
  rc_twin_destroy(ice);
}

static void test_failures_keep_the_state(void) {
  rc_twin_vehicle* v = make(RC_TWIN_TIER_L2_DOUBLE_TRACK);
  ground_t g = level();
  rc_twin_state before, after;
  moving(v, 2.0, 0.0);
  rc_twin_get_state(v, &before);
  rc_twin_set_ground_query(v, query, &g);

  g.fail_at = 9; /* fails in the third substep */
  CHECK(rc_twin_step(v, 0.0, 1.0, 0.001, 5) == RC_TWIN_E_GROUND);
  rc_twin_get_state(v, &after);
  CHECK(memcmp(&before, &after, sizeof before) == 0);

  g = level();
  g.nz = -1.0; /* normal pointing into the ground */
  CHECK(rc_twin_step(v, 0.0, 1.0, 0.001, 1) == RC_TWIN_E_GROUND);
  g = level();
  g.mu = -0.5;
  CHECK(rc_twin_step(v, 0.0, 1.0, 0.001, 1) == RC_TWIN_E_GROUND);
  rc_twin_get_state(v, &after);
  CHECK(memcmp(&before, &after, sizeof before) == 0);

  /* Removing the query restores level ground. */
  CHECK(rc_twin_set_ground_query(v, NULL, &g) == RC_TWIN_OK);
  CHECK(rc_twin_step(v, 0.0, 1.0, 0.001, 1) == RC_TWIN_OK);
  rc_twin_destroy(v);
}

static void test_l1_accepts_level_and_refuses_grade(void) {
  rc_twin_vehicle* v = make(RC_TWIN_TIER_L1_BICYCLE);
  ground_t g = level();
  rc_twin_set_ground_query(v, query, &g);
  moving(v, 2.0, 0.0);
  CHECK(rc_twin_step(v, 0.0, 1.0, 0.001, 10) == RC_TWIN_OK);
  g.slope_x = 0.05;
  CHECK(rc_twin_step(v, 0.0, 1.0, 0.001, 1) == RC_TWIN_E_GROUND);
  g.slope_x = 0.0;
  g.mu = 0.5;
  CHECK(rc_twin_step(v, 0.0, 1.0, 0.001, 1) == RC_TWIN_E_GROUND);
  rc_twin_destroy(v);
}

/* ADR 0010: reverse engages only with the car stopped, wheels included. A car
 * standing on driven wheels that spin on ice is braked: the motor brake takes
 * them from 20 rad/s to 2.5 in one step and stops them at zero in the next,
 * where driving backwards would carry them past zero. */
static void test_reverse_waits_for_the_wheels_to_stop(void) {
  rc_twin_params p;
  rc_twin_vehicle* v;
  rc_twin_state s;
  ground_t g = level();
  int i;
  rc_twin_params_default(&p);
  p.wheel_inertia = 1e-5;
  v = rc_twin_create(RC_TWIN_TIER_L2_DOUBLE_TRACK, RC_TWIN_INTEGRATOR_RK4, &p, NULL);
  g.mu = 0.0;
  rc_twin_set_ground_query(v, query, &g);
  memset(&s, 0, sizeof s);
  s.soc = 1.0;
  s.pack_v = 11.1;
  for (i = 0; i < 4; ++i) s.omega_w[i] = 20.0;
  rc_twin_reset(v, &s);
  CHECK(rc_twin_step(v, 0.0, -2.0, 0.001, 2) == RC_TWIN_OK);
  rc_twin_get_state(v, &s);
  CHECK(s.omega_w[2] == 0.0 && s.omega_w[3] == 0.0);
  rc_twin_destroy(v);
}

int main(void) {
  test_abi_v2();
  test_contact_points_and_order();
  test_level_query_matches_no_query();
  test_grade_grip_and_height_reach_the_core();
  test_failures_keep_the_state();
  test_l1_accepts_level_and_refuses_grade();
  test_reverse_waits_for_the_wheels_to_stop();
  TEST_MAIN_END();
}
