/* The C ABI, exercised from plain C through the shared library only. */
#include <math.h>
#include <string.h>

#include "check.h"
#include "rc_twin.h"

static rc_twin_vehicle* make(int32_t tier) {
  rc_twin_params p;
  rc_twin_params_default(&p);
  return rc_twin_create(tier, RC_TWIN_INTEGRATOR_RK4, &p, NULL);
}

static int same_state(const rc_twin_state* a, const rc_twin_state* b) {
  return memcmp(a, b, sizeof *a) == 0;
}

static void test_versions(void) {
  const char* v = rc_twin_core_version();
  CHECK(rc_twin_abi_version() == RC_TWIN_ABI_VERSION);
  CHECK(RC_TWIN_ABI_VERSION == 7u);
  CHECK(v != NULL && strcmp(v, "0.2.0") == 0);
}

static void test_params_default(void) {
  rc_twin_params p;
  memset(&p, 0, sizeof p);
  CHECK(rc_twin_params_default(NULL) == RC_TWIN_E_NULL);
  CHECK(rc_twin_params_default(&p) == RC_TWIN_OK);
  CHECK(p.struct_size == sizeof(rc_twin_params));
  CHECK(p.provenance == RC_TWIN_PROVISIONAL);
  CHECK(p.mass > 0.0 && p.lf > 0.0 && p.lr > 0.0);
}

static void test_create_rejects_bad_input(void) {
  rc_twin_params p;
  const char* reason = NULL;

  CHECK(rc_twin_create(RC_TWIN_TIER_L2_DOUBLE_TRACK, 0, NULL, &reason) == NULL);
  CHECK(reason != NULL);

  rc_twin_params_default(&p);
  p.struct_size = 8;
  reason = NULL;
  CHECK(rc_twin_create(RC_TWIN_TIER_L2_DOUBLE_TRACK, 0, &p, &reason) == NULL);
  CHECK(reason != NULL);

  rc_twin_params_default(&p);
  p.mass = -1.0;
  reason = NULL;
  CHECK(rc_twin_create(RC_TWIN_TIER_L2_DOUBLE_TRACK, 0, &p, &reason) == NULL);
  CHECK(reason != NULL);

  rc_twin_params_default(&p);
  reason = NULL;
  CHECK(rc_twin_create(RC_TWIN_TIER_L3_EXTENDED, 0, &p, &reason) == NULL);
  CHECK(reason != NULL);
  reason = NULL;
  CHECK(rc_twin_create(7, 0, &p, &reason) == NULL);
  CHECK(reason != NULL);
  reason = NULL;
  CHECK(rc_twin_create(RC_TWIN_TIER_L2_DOUBLE_TRACK, 9, &p, &reason) == NULL);
  CHECK(reason != NULL);
  p.layout = 5;
  reason = NULL;
  CHECK(rc_twin_create(RC_TWIN_TIER_L2_DOUBLE_TRACK, 0, &p, &reason) == NULL);
  CHECK(reason != NULL);

  /* reason is optional */
  CHECK(rc_twin_create(RC_TWIN_TIER_L3_EXTENDED, 0, &p, NULL) == NULL);
}

static void test_starts_at_rest(void) {
  rc_twin_vehicle* v = make(RC_TWIN_TIER_L2_DOUBLE_TRACK);
  rc_twin_state s;
  CHECK(v != NULL);
  CHECK(rc_twin_get_state(v, &s) == RC_TWIN_OK);
  CHECK(s.x == 0.0 && s.y == 0.0 && s.yaw == 0.0 && s.vx == 0.0);
  CHECK(s.soc == 1.0);
  rc_twin_destroy(v);
}

static void test_null_and_invalid_arguments(void) {
  rc_twin_vehicle* v = make(RC_TWIN_TIER_L2_DOUBLE_TRACK);
  rc_twin_state before, after;
  rc_twin_params p;

  CHECK(rc_twin_step(NULL, 0.0, 1.0, 0.001, 1) == RC_TWIN_E_NULL);
  CHECK(rc_twin_get_state(NULL, &before) == RC_TWIN_E_NULL);
  CHECK(rc_twin_get_state(v, NULL) == RC_TWIN_E_NULL);
  CHECK(rc_twin_get_params(v, NULL) == RC_TWIN_E_NULL);
  CHECK(rc_twin_reset(NULL, NULL) == RC_TWIN_E_NULL);
  CHECK(rc_twin_set_params(v, NULL, NULL) == RC_TWIN_E_NULL);
  CHECK(rc_twin_get_params(NULL, &p) == RC_TWIN_E_NULL);
  rc_twin_destroy(NULL); /* must be a no-op */

  rc_twin_get_state(v, &before);
  CHECK(rc_twin_step(v, 0.0, 1.0, 0.0, 1) == RC_TWIN_E_INVALID);
  CHECK(rc_twin_step(v, 0.0, 1.0, -0.001, 1) == RC_TWIN_E_INVALID);
  CHECK(rc_twin_step(v, 0.0, 1.0, NAN, 1) == RC_TWIN_E_INVALID);
  CHECK(rc_twin_step(v, 0.0, 1.0, INFINITY, 1) == RC_TWIN_E_INVALID);
  CHECK(rc_twin_step(v, 0.0, 1.0, 0.001, 0) == RC_TWIN_E_INVALID);
  CHECK(rc_twin_step(v, NAN, 1.0, 0.001, 1) == RC_TWIN_E_INVALID);
  CHECK(rc_twin_step(v, 0.0, NAN, 0.001, 1) == RC_TWIN_E_INVALID);
  rc_twin_get_state(v, &after);
  CHECK(same_state(&before, &after));
  rc_twin_destroy(v);
}

static void test_straight_line_on_flat_ground(void) {
  rc_twin_vehicle* v = make(RC_TWIN_TIER_L2_DOUBLE_TRACK);
  rc_twin_state s;
  CHECK(rc_twin_step(v, 0.0, 2.0, 0.001, 1000) == RC_TWIN_OK);
  rc_twin_get_state(v, &s);
  /* 2 m/s^2 for 1 s, less resistance and ESC lag: clearly moving forward. */
  CHECK(s.vx > 1.0 && s.vx < 2.0);
  CHECK(s.x > 0.4);
  /* A symmetric car with an open diff and no steer goes straight. */
  CHECK(fabs(s.y) < 1e-9);
  CHECK(fabs(s.yaw) < 1e-9);
  CHECK(s.soc < 1.0); /* driving drew charge */
  rc_twin_destroy(v);
}

static void test_left_steer_turns_left(void) {
  int32_t tiers[3] = {RC_TWIN_TIER_L0_KINEMATIC, RC_TWIN_TIER_L1_BICYCLE,
                      RC_TWIN_TIER_L2_DOUBLE_TRACK};
  int i;
  for (i = 0; i < 3; ++i) {
    rc_twin_vehicle* v = make(tiers[i]);
    rc_twin_state s;
    rc_twin_step(v, 0.0, 2.0, 0.001, 1000);
    rc_twin_step(v, 0.2, 0.0, 0.001, 1000);
    rc_twin_get_state(v, &s);
    CHECK(s.yaw > 0.1);
    CHECK(s.y > 0.0);
    rc_twin_destroy(v);
  }
}

static void test_n_steps_equal_n_single_steps(void) {
  rc_twin_vehicle* a = make(RC_TWIN_TIER_L2_DOUBLE_TRACK);
  rc_twin_vehicle* b = make(RC_TWIN_TIER_L2_DOUBLE_TRACK);
  rc_twin_state sa, sb;
  int i;
  rc_twin_step(a, 0.15, 3.0, 0.001, 700);
  for (i = 0; i < 700; ++i) rc_twin_step(b, 0.15, 3.0, 0.001, 1);
  rc_twin_get_state(a, &sa);
  rc_twin_get_state(b, &sb);
  CHECK(same_state(&sa, &sb));
  rc_twin_destroy(a);
  rc_twin_destroy(b);
}

static void test_reset(void) {
  rc_twin_vehicle* v = make(RC_TWIN_TIER_L2_DOUBLE_TRACK);
  rc_twin_vehicle* fresh = make(RC_TWIN_TIER_L2_DOUBLE_TRACK);
  rc_twin_state moved, s, rest;

  rc_twin_step(v, 0.1, 2.0, 0.001, 500);
  rc_twin_get_state(v, &moved);
  CHECK(rc_twin_reset(v, NULL) == RC_TWIN_OK);
  rc_twin_get_state(v, &s);
  rc_twin_get_state(fresh, &rest);
  CHECK(same_state(&s, &rest));

  /* Restoring a snapshot and replaying gives the same trajectory. */
  CHECK(rc_twin_reset(v, &moved) == RC_TWIN_OK);
  rc_twin_get_state(v, &s);
  CHECK(same_state(&s, &moved));
  rc_twin_destroy(v);
  rc_twin_destroy(fresh);
}

static void test_params_roundtrip_and_set(void) {
  rc_twin_vehicle* v = make(RC_TWIN_TIER_L2_DOUBLE_TRACK);
  rc_twin_params def, got, heavier, bad;
  rc_twin_state before, after;
  const char* reason = NULL;

  rc_twin_params_default(&def);
  CHECK(rc_twin_get_params(v, &got) == RC_TWIN_OK);
  CHECK(memcmp(&def, &got, sizeof def) == 0);

  rc_twin_step(v, 0.0, 2.0, 0.001, 300);
  rc_twin_get_state(v, &before);
  heavier = def;
  heavier.mass = def.mass * 1.5;
  heavier.wheel_inertia = 2e-4;
  heavier.ackermann = 0.7;
  heavier.wheel_rate_front = 310.0;
  heavier.wheel_rate_rear = 290.0;
  heavier.damping_front = 9.0;
  heavier.damping_rear = 8.0;
  heavier.suspension_travel = 0.015;
  CHECK(rc_twin_set_params(v, &heavier, &reason) == RC_TWIN_OK);
  rc_twin_get_params(v, &got);
  CHECK(got.mass == heavier.mass);
  CHECK(got.wheel_inertia == 2e-4);
  CHECK(got.ackermann == 0.7);
  CHECK(got.wheel_rate_front == 310.0 && got.wheel_rate_rear == 290.0);
  CHECK(got.damping_front == 9.0 && got.damping_rear == 8.0);
  CHECK(got.suspension_travel == 0.015);
  CHECK(memcmp(&got, &heavier, sizeof got) == 0);
  rc_twin_get_state(v, &after);
  CHECK(same_state(&before, &after)); /* state kept across set_params */

  bad = def;
  bad.wheel_radius = 0.0;
  reason = NULL;
  CHECK(rc_twin_set_params(v, &bad, &reason) == RC_TWIN_E_INVALID);
  CHECK(reason != NULL);
  rc_twin_get_params(v, &got);
  CHECK(got.mass == heavier.mass); /* old params stay in force */
  rc_twin_destroy(v);
}

int main(void) {
  test_versions();
  test_params_default();
  test_create_rejects_bad_input();
  test_starts_at_rest();
  test_null_and_invalid_arguments();
  test_straight_line_on_flat_ground();
  test_left_steer_turns_left();
  test_n_steps_equal_n_single_steps();
  test_reset();
  test_params_roundtrip_and_set();
  TEST_MAIN_END();
}
