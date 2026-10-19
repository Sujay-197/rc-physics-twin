/* Roads (rc_twin_road_*): geometry checked against hand-worked values, then
 * a car driven on roads through the ground query. Plain C, DLL only. */
#include <math.h>
#include <string.h>

#include "check.h"
#include "rc_twin.h"

#define NEAR(a, b, tol) (fabs((a) - (b)) <= (tol))

static rc_twin_road* straight(double z1, double bank) {
  const double xyz[6] = {0.0, 0.0, 0.0, 10.0, 0.0, z1};
  const double b[2] = {bank, bank};
  return rc_twin_road_create(xyz, b, 2, 1.0, NULL);
}

static void test_create_rejects_bad_input(void) {
  const double one[3] = {0, 0, 0};
  const double same[6] = {1, 1, 0, 1, 1, 5};
  const double ok[6] = {0, 0, 0, 1, 0, 0};
  const double steep[2] = {0.6, 0.0};
  const char* why = NULL;
  CHECK(rc_twin_road_create(NULL, NULL, 2, 1.0, &why) == NULL && why != NULL);
  CHECK(rc_twin_road_create(one, NULL, 1, 1.0, &why) == NULL && why != NULL);
  CHECK(rc_twin_road_create(same, NULL, 2, 1.0, &why) == NULL && why != NULL);
  CHECK(rc_twin_road_create(ok, NULL, 2, 0.0, &why) == NULL && why != NULL);
  CHECK(rc_twin_road_create(ok, steep, 2, 1.0, &why) == NULL && why != NULL);
  {
    rc_twin_road* r = rc_twin_road_create(ok, NULL, 2, 1.0, &why);
    CHECK(r != NULL && why == NULL);
    CHECK(rc_twin_road_add_surface(r, 2.0, 1.0, 0.5) == RC_TWIN_E_INVALID);
    CHECK(rc_twin_road_add_surface(r, 0.0, 1.0, -0.1) == RC_TWIN_E_INVALID);
    CHECK(rc_twin_road_add_breaker(r, 0.5, 0.0, 0.3) == RC_TWIN_E_INVALID);
    CHECK(rc_twin_road_add_breaker(NULL, 0.5, 0.01, 0.3) == RC_TWIN_E_NULL);
    rc_twin_road_destroy(r);
  }
  rc_twin_road_destroy(NULL);
}

static void test_length_locate_and_point(void) {
  const double xyz[9] = {0, 0, 0, 3, 4, 0, 3, 10, 1};  /* 5 m then 6 m */
  rc_twin_road* r = rc_twin_road_create(xyz, NULL, 3, 2.0, NULL);
  double s, d, x, y, z, h;
  CHECK(NEAR(rc_twin_road_length(r), 11.0, 1e-12));
  /* A point 0.5 m left of the second segment at s = 5 + 2. */
  rc_twin_road_locate(r, 2.5, 6.0, &s, &d);
  CHECK(NEAR(s, 7.0, 1e-12) && NEAR(d, 0.5, 1e-12));
  rc_twin_road_locate(r, 3.5, 6.0, &s, &d);
  CHECK(NEAR(d, -0.5, 1e-12));                     /* right is negative */
  rc_twin_road_point(r, 8.0, &x, &y, &z, &h);
  CHECK(NEAR(x, 3.0, 1e-12) && NEAR(y, 7.0, 1e-12) && NEAR(z, 0.5, 1e-12));
  CHECK(NEAR(h, 1.5707963267948966, 1e-12));
  rc_twin_road_destroy(r);
}

static void test_grade_bank_breaker_and_grip(void) {
  rc_twin_ground_sample g;
  /* 10 m rising 0.5 m: 5 % grade. Normal tilts back by atan(0.05). */
  rc_twin_road* r = straight(0.5, 0.0);
  CHECK(rc_twin_road_ground(r, 0, 4.0, 0.0, &g) == 0);
  CHECK(NEAR(g.height, 0.2, 1e-12));
  CHECK(NEAR(g.nx / g.nz, -0.05, 1e-12) && NEAR(g.ny, 0.0, 1e-15));
  CHECK(g.mu_scale == 1.0);
  rc_twin_road_destroy(r);

  /* Banked 0.1 rad rising to the left: 0.3 m left is tan(0.1) * 0.3 higher,
   * and the normal tilts to the right (toward -y). */
  r = straight(0.0, 0.1);
  CHECK(rc_twin_road_ground(r, 0, 4.0, 0.3, &g) == 0);
  CHECK(NEAR(g.height, 0.3 * tan(0.1), 1e-12));
  CHECK(NEAR(g.ny / g.nz, -tan(0.1), 1e-12));
  /* Off the 1 m width: not modelled. */
  CHECK(rc_twin_road_ground(r, 0, 4.0, 0.6, &g) != 0);
  rc_twin_road_destroy(r);

  /* The same bank on a road running along +y: its left is -x, so the left
   * edge is at x = -0.3 and the normal tilts toward +x. */
  {
    const double xyz[6] = {0.0, 0.0, 0.0, 0.0, 10.0, 0.0};
    const double b[2] = {0.1, 0.1};
    r = rc_twin_road_create(xyz, b, 2, 1.0, NULL);
    CHECK(rc_twin_road_ground(r, 0, -0.3, 4.0, &g) == 0);
    CHECK(NEAR(g.height, 0.3 * tan(0.1), 1e-12));
    CHECK(NEAR(g.nx / g.nz, tan(0.1), 1e-12) && NEAR(g.ny, 0.0, 1e-15));
    rc_twin_road_destroy(r);
  }

  /* Behind the start of a graded road the end segment extends: the height
   * carries on down the grade rather than flattening at the first point. */
  r = straight(0.5, 0.0);
  CHECK(rc_twin_road_ground(r, 0, -0.2, 0.0, &g) == 0);
  CHECK(NEAR(g.height, -0.01, 1e-12));
  rc_twin_road_destroy(r);

  /* A 2 cm round hump 0.4 m long centred at s = 5: full height at its
   * centre, half height a quarter in with the steepest slope, h pi / L. */
  r = straight(0.0, 0.0);
  rc_twin_road_add_breaker(r, 5.0, 0.02, 0.4);
  rc_twin_road_add_surface(r, 6.0, 8.0, 0.4);
  rc_twin_road_add_surface(r, 7.0, 7.5, 0.2);   /* later wins */
  CHECK(rc_twin_road_ground(r, 0, 5.0, 0.0, &g) == 0);
  CHECK(NEAR(g.height, 0.02, 1e-12) && NEAR(g.nx, 0.0, 1e-12));
  CHECK(rc_twin_road_ground(r, 0, 4.9, 0.0, &g) == 0);
  CHECK(NEAR(g.height, 0.01, 1e-12));
  CHECK(NEAR(-g.nx / g.nz, 0.02 * 3.14159265358979323846 / 0.4, 1e-12));
  CHECK(rc_twin_road_ground(r, 0, 4.7, 0.0, &g) == 0 && g.height == 0.0);
  CHECK(rc_twin_road_ground(r, 0, 6.5, 0.0, &g) == 0 && g.mu_scale == 0.4);
  CHECK(rc_twin_road_ground(r, 0, 7.2, 0.0, &g) == 0 && g.mu_scale == 0.2);
  CHECK(rc_twin_road_ground(r, 0, 8.0, 0.0, &g) == 0 && g.mu_scale == 1.0);
  /* The ends extend: a rear wheel just behind s = 0 is still on the road. */
  CHECK(rc_twin_road_ground(r, 0, -0.2, 0.0, &g) == 0);
  rc_twin_road_destroy(r);
}

static rc_twin_vehicle* car_on(rc_twin_road* r, double x0, double v0) {
  rc_twin_params p;
  rc_twin_state s;
  rc_twin_vehicle* v;
  rc_twin_params_default(&p);
  v = rc_twin_create(RC_TWIN_TIER_L2_DOUBLE_TRACK, 0, &p, NULL);
  memset(&s, 0, sizeof s);
  s.x = x0;
  s.vx = v0;
  s.soc = 1.0;
  s.pack_v = 11.1;
  rc_twin_reset(v, &s);
  rc_twin_set_ground_query(v, rc_twin_road_ground, r);
  return v;
}

static void test_a_car_on_roads(void) {
  /* Coasting down a 5 % road gains on a level one; ice costs traction;
   * leaving the road stops the step and keeps the state. */
  rc_twin_road* level = straight(0.0, 0.0);
  rc_twin_road* down = straight(-0.5, 0.0);
  rc_twin_vehicle* a = car_on(level, 1.0, 2.0);
  rc_twin_vehicle* b = car_on(down, 1.0, 2.0);
  rc_twin_state sa, sb, before, after;
  CHECK(rc_twin_step(a, 0.0, 0.0, 0.001, 1000) == RC_TWIN_OK);
  CHECK(rc_twin_step(b, 0.0, 0.0, 0.001, 1000) == RC_TWIN_OK);
  rc_twin_get_state(a, &sa);
  rc_twin_get_state(b, &sb);
  CHECK(sb.vx > sa.vx + 0.4);           /* about g sin(2.9 deg) * 1 s = 0.49 */
  CHECK(sb.z < -0.05);                  /* and it went down */

  /* Steering hard off the 1 m wide road fails the step at the edge. */
  rc_twin_reset(a, &sa);
  {
    int k, status = RC_TWIN_OK;
    for (k = 0; k < 3000 && status == RC_TWIN_OK; ++k) {
      rc_twin_get_state(a, &before);
      status = rc_twin_step(a, 0.4, 1.0, 0.001, 1);
    }
    rc_twin_get_state(a, &after);
    CHECK(status == RC_TWIN_E_GROUND);
    CHECK(memcmp(&before, &after, sizeof before) == 0);
    CHECK(fabs(after.y) > 0.3);          /* it was near the edge */
  }
  rc_twin_destroy(a);
  rc_twin_destroy(b);
  rc_twin_road_destroy(level);
  rc_twin_road_destroy(down);
}

int main(void) {
  test_create_rejects_bad_input();
  test_length_locate_and_point();
  test_grade_bank_breaker_and_grip();
  test_a_car_on_roads();
  TEST_MAIN_END();
}
