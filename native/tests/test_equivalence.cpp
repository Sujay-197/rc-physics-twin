// The C ABI must be a transparent window onto the Core: the same params and
// inputs give a bit-identical trajectory whether the Core is driven directly
// or through rc_twin. These tests catch flattening mistakes (a swapped or
// dropped field) that a behavioural test could miss.
#include <cstring>
#include <memory>

#include "check.h"
#include "rc_twin.h"
#include "slipx/ground.hpp"
#include "slipx/params.hpp"
#include "slipx/state.hpp"
#include "slipx/vehicle_model.hpp"

namespace {

bool bits_equal(double a, double b) { return std::memcmp(&a, &b, sizeof a) == 0; }

bool same(const slipx::VehicleState& s, const rc_twin_state& c) {
  bool ok = bits_equal(s.pos.x, c.x) && bits_equal(s.pos.y, c.y) &&
            bits_equal(s.pos.z, c.z) && bits_equal(s.yaw, c.yaw) &&
            bits_equal(s.pitch, c.pitch) && bits_equal(s.roll, c.roll) &&
            bits_equal(s.vel_body.x, c.vx) && bits_equal(s.vel_body.y, c.vy) &&
            bits_equal(s.vel_body.z, c.vz) && bits_equal(s.rates.x, c.roll_rate) &&
            bits_equal(s.rates.y, c.pitch_rate) &&
            bits_equal(s.rates.z, c.yaw_rate) && bits_equal(s.steer, c.steer) &&
            bits_equal(s.steer_rate, c.steer_rate) && bits_equal(s.soc, c.soc) &&
            bits_equal(s.pack_v, c.pack_v);
  for (int i = 0; i < 4; ++i) {
    ok = ok && bits_equal(s.omega_w[i], c.omega_w[i]) &&
         bits_equal(s.Fz[i], c.fz[i]) && bits_equal(s.alpha_lag[i], c.alpha_lag[i]);
  }
  return ok;
}

void tyre_defaults_match(const slipx::TyreCoefficients& t, const rc_twin_tyre& c) {
  CHECK(t.mu_y0 == c.mu_y0);
  CHECK(t.mu_x0 == c.mu_x0);
  CHECK(t.k_mu == c.k_mu);
  CHECK(t.relax_length == c.relax_length);
  CHECK(t.shape_c == c.shape_c);
  CHECK(t.curvature_e == c.curvature_e);
}

void test_defaults_match_core() {
  const slipx::VehicleParams p{};
  rc_twin_params c;
  rc_twin_params_default(&c);
  CHECK(c.provenance == RC_TWIN_PROVISIONAL);
  CHECK(p.mass == c.mass);
  CHECK(p.izz == c.izz);
  CHECK(p.ixx == c.ixx);
  CHECK(p.iyy == c.iyy);
  CHECK(p.lf == c.lf);
  CHECK(p.lr == c.lr);
  CHECK(p.track_front == c.track_front);
  CHECK(p.track_rear == c.track_rear);
  CHECK(p.h_cog == c.h_cog);
  CHECK(p.wheel_radius == c.wheel_radius);
  CHECK(p.c_alpha_f == c.c_alpha_f);
  CHECK(p.c_alpha_r == c.c_alpha_r);
  CHECK(p.mu_clip == c.mu_clip);
  tyre_defaults_match(p.tyre_front, c.tyre_front);
  tyre_defaults_match(p.tyre_rear, c.tyre_rear);
  CHECK(p.c_kappa == c.c_kappa);
  CHECK(p.accel_max == c.accel_max);
  CHECK(p.decel_max == c.decel_max);
  CHECK(p.v_max == c.v_max);
  CHECK(c.layout == RC_TWIN_RWD);
  CHECK(c.differential == RC_TWIN_DIFF_OPEN);
  CHECK(p.lsd_preload == c.lsd_preload);
  CHECK(p.torque_stall == c.torque_stall);
  CHECK(p.omega_free == c.omega_free);
  CHECK(p.torque_per_amp == c.torque_per_amp);
  CHECK(p.drive_efficiency == c.drive_efficiency);
  CHECK(p.current_max == c.current_max);
  CHECK(p.regen_current_max == c.regen_current_max);
  CHECK(p.pack_nominal_v == c.pack_nominal_v);
  CHECK(p.pack_v_full == c.pack_v_full);
  CHECK(p.pack_v_empty == c.pack_v_empty);
  CHECK(p.pack_capacity_ah == c.pack_capacity_ah);
  CHECK(p.pack_internal_resistance == c.pack_internal_resistance);
  CHECK(p.steer_max == c.steer_max);
  CHECK(p.steer_rate_max == c.steer_rate_max);
  CHECK(p.steer_bandwidth == c.steer_bandwidth);
  CHECK(p.steer_damping == c.steer_damping);
  CHECK(p.drag_coeff == c.drag_coeff);
  CHECK(p.roll_resist == c.roll_resist);
  CHECK(p.v_eps == c.v_eps);
}

// Every field moved off its default by a distinct factor, so a swap of any
// two fields changes the trajectory. Applied identically to both sides.
template <class T>
void nudge_tyre(T& t, double k) {
  t.mu_y0 *= 1.0 + 1 * k;
  t.mu_x0 *= 1.0 + 2 * k;
  t.k_mu *= 1.0 + 3 * k;
  t.relax_length *= 1.0 + 4 * k;
  t.shape_c *= 1.0 + 5 * k;
  t.curvature_e *= 1.0 - 6 * k;
}

template <class P>
void nudge(P& p) {
  const double k = 0.003;
  p.mass *= 1.0 + 1 * k;
  p.izz *= 1.0 + 2 * k;
  p.ixx *= 1.0 + 3 * k;
  p.iyy *= 1.0 + 4 * k;
  p.lf *= 1.0 + 5 * k;
  p.lr *= 1.0 - 6 * k;
  p.track_front *= 1.0 + 7 * k;
  p.track_rear *= 1.0 - 8 * k;
  p.h_cog *= 1.0 + 9 * k;
  p.wheel_radius *= 1.0 + 10 * k;
  p.c_alpha_f *= 1.0 + 11 * k;
  p.c_alpha_r *= 1.0 - 12 * k;
  p.mu_clip *= 1.0 + 13 * k;
  nudge_tyre(p.tyre_front, k);
  nudge_tyre(p.tyre_rear, 2 * k);
  p.c_kappa *= 1.0 + 14 * k;
  p.accel_max *= 1.0 + 15 * k;
  p.decel_max *= 1.0 + 16 * k;
  p.v_max *= 1.0 + 17 * k;
  p.lsd_preload = 0.05;
  p.torque_stall *= 1.0 + 18 * k;
  p.omega_free *= 1.0 + 19 * k;
  p.torque_per_amp *= 1.0 + 20 * k;
  p.drive_efficiency *= 1.0 + 21 * k;
  p.current_max *= 1.0 - 22 * k;
  p.regen_current_max *= 1.0 + 23 * k;
  p.pack_nominal_v *= 1.0 + 0.1 * k;
  p.pack_v_full *= 1.0 + 24 * k;
  p.pack_v_empty *= 1.0 - 25 * k;
  p.pack_capacity_ah *= 1.0 + 26 * k;
  p.pack_internal_resistance *= 1.0 + 27 * k;
  p.steer_max *= 1.0 + 28 * k;
  p.steer_rate_max *= 1.0 + 29 * k;
  p.steer_bandwidth *= 1.0 + 30 * k;
  p.steer_damping *= 1.0 + 31 * k;
  p.drag_coeff *= 1.0 + 32 * k;
  p.roll_resist *= 1.0 + 33 * k;
  p.v_eps *= 1.0 + 34 * k;
}

// A drive that exercises steering, drive, coast and regen.
void command_at(int i, double& steer, double& accel) {
  if (i < 800) { steer = 0.0; accel = 4.0; }
  else if (i < 1600) { steer = 0.25; accel = 1.0; }
  else if (i < 2200) { steer = -0.3; accel = 0.0; }
  else { steer = 0.05; accel = -6.0; }
}

void run_equivalence(int32_t tier, int32_t integrator, bool nudged,
                     int32_t layout, int32_t diff) {
  slipx::VehicleParams p{};
  rc_twin_params c;
  rc_twin_params_default(&c);
  if (nudged) {
    nudge(p);
    nudge(c);
    p.layout = static_cast<slipx::DriveLayout>(layout);
    p.differential = static_cast<slipx::Differential>(diff);
    c.layout = layout;
    c.differential = diff;
  }

  const char* reason = nullptr;
  auto model = slipx::VehicleModel::try_create(
      static_cast<slipx::Tier>(tier), p, static_cast<slipx::Integrator>(integrator),
      &reason);
  rc_twin_vehicle* v = rc_twin_create(tier, integrator, &c, &reason);
  CHECK(model != nullptr);
  CHECK(v != nullptr);
  if (!model || !v) return;

  slipx::VehicleState s{};
  rc_twin_state cs;
  bool ok = true;
  for (int i = 0; i < 3000; ++i) {
    double steer, accel;
    command_at(i, steer, accel);
    model->step(s, slipx::DriveInput{steer, accel}, 0.001);
    rc_twin_step(v, steer, accel, 0.001, 1);
    rc_twin_get_state(v, &cs);
    if (!same(s, cs)) { ok = false; break; }
  }
  CHECK(ok);
  CHECK(s.pos.x != 0.0);  // the run actually went somewhere
  rc_twin_destroy(v);
}

void test_state_roundtrip_is_exact() {
  // A state with every field distinct survives reset -> get_state unchanged.
  rc_twin_params c;
  rc_twin_params_default(&c);
  rc_twin_vehicle* v = rc_twin_create(RC_TWIN_TIER_L2_DOUBLE_TRACK, 0, &c, nullptr);
  rc_twin_state in;
  double* f = reinterpret_cast<double*>(&in);
  for (std::size_t i = 0; i < sizeof in / sizeof(double); ++i) f[i] = 0.1 + 0.37 * double(i);
  rc_twin_state out;
  rc_twin_reset(v, &in);
  rc_twin_get_state(v, &out);
  CHECK(std::memcmp(&in, &out, sizeof in) == 0);
  rc_twin_destroy(v);
}

// Ground that varies across the world, so a wrong contact point, wheel order
// or field mapping changes the samples the Core sees.
void field_at(double x, double y, rc_twin_ground_sample* out) {
  out->height = 0.1 * x - 0.05 * y;
  out->nx = 0.04 + 0.01 * x;
  out->ny = -0.03 + 0.02 * std::sin(y);
  out->nz = 1.0;
  out->mu_scale = 0.6 + 0.3 * std::cos(0.7 * x + y);
}

int32_t field_query(void*, int32_t, double x, double y, rc_twin_ground_sample* out) {
  field_at(x, y, out);
  return 0;
}

void test_ground_equivalence() {
  slipx::VehicleParams p{};
  rc_twin_params c;
  rc_twin_params_default(&c);
  auto model = slipx::VehicleModel::create(slipx::Tier::L2_DoubleTrack, p);
  rc_twin_vehicle* v = rc_twin_create(RC_TWIN_TIER_L2_DOUBLE_TRACK, 0, &c, nullptr);
  rc_twin_set_ground_query(v, field_query, nullptr);

  slipx::VehicleState s{};
  s.vel_body.x = 1.0;
  rc_twin_state cs;
  rc_twin_get_state(v, &cs);
  cs.vx = 1.0;
  rc_twin_reset(v, &cs);

  bool ok = true;
  for (int i = 0; i < 3000 && ok; ++i) {
    double steer, accel;
    command_at(i, steer, accel);
    // Contact points worked out here, independently of rc_twin.
    slipx::GroundContact g;
    const double bx[4] = {p.lf, p.lf, -p.lr, -p.lr};
    const double by[4] = {p.track_front / 2, -p.track_front / 2,
                          p.track_rear / 2, -p.track_rear / 2};
    for (int w = 0; w < 4; ++w) {
      const double x = s.pos.x + std::cos(s.yaw) * bx[w] - std::sin(s.yaw) * by[w];
      const double y = s.pos.y + std::sin(s.yaw) * bx[w] + std::cos(s.yaw) * by[w];
      rc_twin_ground_sample o;
      field_at(x, y, &o);
      g.wheel[w].height = o.height;
      g.wheel[w].normal = {o.nx, o.ny, o.nz};
      g.wheel[w].mu_scale = o.mu_scale;
    }
    ok = model->step_on(s, slipx::DriveInput{steer, accel}, g, 0.001) &&
         rc_twin_step(v, steer, accel, 0.001, 1) == RC_TWIN_OK;
    rc_twin_get_state(v, &cs);
    ok = ok && same(s, cs);
  }
  CHECK(ok);
  CHECK(s.pos.z != 0.0);  // the field was felt
  rc_twin_destroy(v);
}

}  // namespace

int main() {
  static_assert(sizeof(rc_twin_state) == 28 * sizeof(double), "state layout");
  test_defaults_match_core();
  test_state_roundtrip_is_exact();
  for (int32_t tier = 0; tier <= 2; ++tier) {
    for (int32_t integ = 0; integ <= 1; ++integ) {
      run_equivalence(tier, integ, false, RC_TWIN_RWD, RC_TWIN_DIFF_OPEN);
    }
  }
  run_equivalence(RC_TWIN_TIER_L2_DOUBLE_TRACK, 0, true, RC_TWIN_AWD, RC_TWIN_DIFF_LSD);
  run_equivalence(RC_TWIN_TIER_L2_DOUBLE_TRACK, 0, true, RC_TWIN_FWD, RC_TWIN_DIFF_SPOOL);
  run_equivalence(RC_TWIN_TIER_L1_BICYCLE, 1, true, RC_TWIN_RWD, RC_TWIN_DIFF_OPEN);
  test_ground_equivalence();
  TEST_MAIN_END();
}
