// rc_twin C ABI over slipx_core. See include/rc_twin.h for the contract.
//
// Nothing here changes the physics: params and state are copied field by
// field between the flat C structs and the Core's own types, and step calls
// VehicleModel::step unchanged. tests/test_equivalence.cpp holds that to
// bit-identity.
#include "rc_twin.h"

#include <cmath>
#include <cstring>
#include <memory>
#include <new>

#include "slipx/ground.hpp"
#include "slipx/params.hpp"
#include "slipx/state.hpp"
#include "slipx/vehicle_model.hpp"
#include "slipx/version.hpp"

static_assert(sizeof(rc_twin_state) == 28 * sizeof(double),
              "rc_twin_state must be 28 packed doubles");
static_assert(slipx::kWheelCount == 4, "per-wheel arrays are fixed at 4");

struct rc_twin_vehicle {
  slipx::Tier tier;
  slipx::Integrator integrator;
  std::unique_ptr<slipx::VehicleModel> model;
  slipx::VehicleState state;
  rc_twin_ground_fn ground_fn = nullptr;  // NULL: level ground at z = 0
  void* ground_user = nullptr;
};

namespace {

void set_reason(const char** reason, const char* why) {
  if (reason != nullptr) *reason = why;
}

void to_core(const rc_twin_tyre& c, slipx::TyreCoefficients& t) {
  t.mu_y0 = c.mu_y0;
  t.mu_x0 = c.mu_x0;
  t.k_mu = c.k_mu;
  t.relax_length = c.relax_length;
  t.shape_c = c.shape_c;
  t.curvature_e = c.curvature_e;
}

void from_core(const slipx::TyreCoefficients& t, rc_twin_tyre& c) {
  c.mu_y0 = t.mu_y0;
  c.mu_x0 = t.mu_x0;
  c.k_mu = t.k_mu;
  c.relax_length = t.relax_length;
  c.shape_c = t.shape_c;
  c.curvature_e = t.curvature_e;
}

// Returns nullptr on success, or a static reason. Enum values are range
// checked here because casting an out-of-range int to a C++ enum and handing
// it to the Core would be undefined behaviour waiting to happen.
const char* to_core(const rc_twin_params& c, slipx::VehicleParams& p) {
  if (c.struct_size != sizeof(rc_twin_params)) {
    return "rc_twin_params.struct_size mismatch: fill with "
           "rc_twin_params_default() from the same rc_twin.h";
  }
  if (c.provenance < RC_TWIN_PROVISIONAL || c.provenance > RC_TWIN_MEASURED) {
    return "rc_twin_params.provenance out of range";
  }
  if (c.layout < RC_TWIN_RWD || c.layout > RC_TWIN_AWD) {
    return "rc_twin_params.layout out of range";
  }
  if (c.differential < RC_TWIN_DIFF_OPEN || c.differential > RC_TWIN_DIFF_LSD) {
    return "rc_twin_params.differential out of range";
  }

  p.provenance = static_cast<slipx::Provenance>(c.provenance);
  p.mass = c.mass;
  p.izz = c.izz;
  p.ixx = c.ixx;
  p.iyy = c.iyy;
  p.lf = c.lf;
  p.lr = c.lr;
  p.track_front = c.track_front;
  p.track_rear = c.track_rear;
  p.h_cog = c.h_cog;
  p.wheel_radius = c.wheel_radius;
  p.c_alpha_f = c.c_alpha_f;
  p.c_alpha_r = c.c_alpha_r;
  p.mu_clip = c.mu_clip;
  to_core(c.tyre_front, p.tyre_front);
  to_core(c.tyre_rear, p.tyre_rear);
  p.c_kappa = c.c_kappa;
  p.accel_max = c.accel_max;
  p.decel_max = c.decel_max;
  p.v_max = c.v_max;
  p.layout = static_cast<slipx::DriveLayout>(c.layout);
  p.differential = static_cast<slipx::Differential>(c.differential);
  p.lsd_preload = c.lsd_preload;
  p.torque_stall = c.torque_stall;
  p.omega_free = c.omega_free;
  p.torque_per_amp = c.torque_per_amp;
  p.drive_efficiency = c.drive_efficiency;
  p.current_max = c.current_max;
  p.regen_current_max = c.regen_current_max;
  p.pack_nominal_v = c.pack_nominal_v;
  p.pack_v_full = c.pack_v_full;
  p.pack_v_empty = c.pack_v_empty;
  p.pack_capacity_ah = c.pack_capacity_ah;
  p.pack_internal_resistance = c.pack_internal_resistance;
  p.steer_max = c.steer_max;
  p.steer_rate_max = c.steer_rate_max;
  p.steer_bandwidth = c.steer_bandwidth;
  p.steer_damping = c.steer_damping;
  p.drag_coeff = c.drag_coeff;
  p.roll_resist = c.roll_resist;
  p.v_eps = c.v_eps;
  p.wheel_inertia = c.wheel_inertia;
  return nullptr;
}

void from_core(const slipx::VehicleParams& p, rc_twin_params& c) {
  std::memset(&c, 0, sizeof c);  // padding included, so memcmp is meaningful
  c.struct_size = sizeof(rc_twin_params);
  c.provenance = static_cast<int32_t>(p.provenance);
  c.mass = p.mass;
  c.izz = p.izz;
  c.ixx = p.ixx;
  c.iyy = p.iyy;
  c.lf = p.lf;
  c.lr = p.lr;
  c.track_front = p.track_front;
  c.track_rear = p.track_rear;
  c.h_cog = p.h_cog;
  c.wheel_radius = p.wheel_radius;
  c.c_alpha_f = p.c_alpha_f;
  c.c_alpha_r = p.c_alpha_r;
  c.mu_clip = p.mu_clip;
  from_core(p.tyre_front, c.tyre_front);
  from_core(p.tyre_rear, c.tyre_rear);
  c.c_kappa = p.c_kappa;
  c.accel_max = p.accel_max;
  c.decel_max = p.decel_max;
  c.v_max = p.v_max;
  c.layout = static_cast<int32_t>(p.layout);
  c.differential = static_cast<int32_t>(p.differential);
  c.lsd_preload = p.lsd_preload;
  c.torque_stall = p.torque_stall;
  c.omega_free = p.omega_free;
  c.torque_per_amp = p.torque_per_amp;
  c.drive_efficiency = p.drive_efficiency;
  c.current_max = p.current_max;
  c.regen_current_max = p.regen_current_max;
  c.pack_nominal_v = p.pack_nominal_v;
  c.pack_v_full = p.pack_v_full;
  c.pack_v_empty = p.pack_v_empty;
  c.pack_capacity_ah = p.pack_capacity_ah;
  c.pack_internal_resistance = p.pack_internal_resistance;
  c.steer_max = p.steer_max;
  c.steer_rate_max = p.steer_rate_max;
  c.steer_bandwidth = p.steer_bandwidth;
  c.steer_damping = p.steer_damping;
  c.drag_coeff = p.drag_coeff;
  c.roll_resist = p.roll_resist;
  c.v_eps = p.v_eps;
  c.wheel_inertia = p.wheel_inertia;
}

void to_core(const rc_twin_state& c, slipx::VehicleState& s) {
  s.pos = {c.x, c.y, c.z};
  s.yaw = c.yaw;
  s.pitch = c.pitch;
  s.roll = c.roll;
  s.vel_body = {c.vx, c.vy, c.vz};
  s.rates = {c.roll_rate, c.pitch_rate, c.yaw_rate};
  for (std::size_t i = 0; i < 4; ++i) {
    s.omega_w[i] = c.omega_w[i];
    s.Fz[i] = c.fz[i];
    s.alpha_lag[i] = c.alpha_lag[i];
  }
  s.steer = c.steer;
  s.steer_rate = c.steer_rate;
  s.soc = c.soc;
  s.pack_v = c.pack_v;
}

void from_core(const slipx::VehicleState& s, rc_twin_state& c) {
  c.x = s.pos.x;
  c.y = s.pos.y;
  c.z = s.pos.z;
  c.yaw = s.yaw;
  c.pitch = s.pitch;
  c.roll = s.roll;
  c.vx = s.vel_body.x;
  c.vy = s.vel_body.y;
  c.vz = s.vel_body.z;
  c.roll_rate = s.rates.x;
  c.pitch_rate = s.rates.y;
  c.yaw_rate = s.rates.z;
  for (std::size_t i = 0; i < 4; ++i) {
    c.omega_w[i] = s.omega_w[i];
    c.fz[i] = s.Fz[i];
    c.alpha_lag[i] = s.alpha_lag[i];
  }
  c.steer = s.steer;
  c.steer_rate = s.steer_rate;
  c.soc = s.soc;
  c.pack_v = s.pack_v;
}

// Asks the host for the ground under each wheel of state s: the wheel centre
// in the body frame (x forward, y left; FL, FR, RL, RR), rotated by yaw and
// moved to the CoG. Returns false if the host reports a failure.
bool sample_ground(const rc_twin_vehicle& v, const slipx::VehicleState& s,
                   slipx::GroundContact* g) {
  const slipx::VehicleParams& p = v.model->params();
  const double bx[4] = {p.lf, p.lf, -p.lr, -p.lr};
  const double by[4] = {0.5 * p.track_front, -0.5 * p.track_front,
                        0.5 * p.track_rear, -0.5 * p.track_rear};
  const double cy = std::cos(s.yaw);
  const double sy = std::sin(s.yaw);
  for (int32_t i = 0; i < 4; ++i) {
    const double x = s.pos.x + cy * bx[i] - sy * by[i];
    const double y = s.pos.y + sy * bx[i] + cy * by[i];
    rc_twin_ground_sample out{0.0, 0.0, 0.0, 1.0, 1.0};
    if (v.ground_fn(v.ground_user, i, x, y, &out) != 0) return false;
    g->wheel[static_cast<std::size_t>(i)].height = out.height;
    g->wheel[static_cast<std::size_t>(i)].normal = {out.nx, out.ny, out.nz};
    g->wheel[static_cast<std::size_t>(i)].mu_scale = out.mu_scale;
  }
  return true;
}

// Builds a model, or returns nullptr with *why set. Never throws.
std::unique_ptr<slipx::VehicleModel> build(slipx::Tier tier,
                                           slipx::Integrator integrator,
                                           const rc_twin_params* params,
                                           const char** why) {
  if (params == nullptr) {
    *why = "params is NULL";
    return nullptr;
  }
  slipx::VehicleParams p;
  if (const char* bad = to_core(*params, p)) {
    *why = bad;
    return nullptr;
  }
  try {
    return slipx::VehicleModel::try_create(tier, p, integrator, why);
  } catch (...) {  // try_create is noexcept; this guards allocation failure paths
    *why = "rc_twin: the Core failed to construct the model";
    return nullptr;
  }
}

}  // namespace

extern "C" {

RC_TWIN_API uint32_t rc_twin_abi_version(void) { return RC_TWIN_ABI_VERSION; }

RC_TWIN_API const char* rc_twin_core_version(void) { return slipx::kVersion; }

RC_TWIN_API rc_twin_status rc_twin_params_default(rc_twin_params* out) {
  if (out == nullptr) return RC_TWIN_E_NULL;
  from_core(slipx::VehicleParams{}, *out);
  return RC_TWIN_OK;
}

RC_TWIN_API rc_twin_vehicle* rc_twin_create(int32_t tier, int32_t integrator,
                                            const rc_twin_params* params,
                                            const char** reason) {
  set_reason(reason, nullptr);
  if (tier < RC_TWIN_TIER_L0_KINEMATIC || tier > RC_TWIN_TIER_L3_EXTENDED) {
    set_reason(reason, "tier out of range");
    return nullptr;
  }
  if (integrator < RC_TWIN_INTEGRATOR_RK4 ||
      integrator > RC_TWIN_INTEGRATOR_SEMI_IMPLICIT_EULER) {
    set_reason(reason, "integrator out of range");
    return nullptr;
  }
  const auto t = static_cast<slipx::Tier>(tier);
  const auto i = static_cast<slipx::Integrator>(integrator);
  const char* why = nullptr;
  auto model = build(t, i, params, &why);
  if (!model) {
    set_reason(reason, why != nullptr ? why : "rc_twin: cannot create model");
    return nullptr;
  }
  auto* v = new (std::nothrow) rc_twin_vehicle{t, i, std::move(model), {}};
  if (v == nullptr) set_reason(reason, "rc_twin: out of memory");
  return v;
}

RC_TWIN_API void rc_twin_destroy(rc_twin_vehicle* v) { delete v; }

RC_TWIN_API rc_twin_status rc_twin_set_params(rc_twin_vehicle* v,
                                              const rc_twin_params* params,
                                              const char** reason) {
  set_reason(reason, nullptr);
  if (v == nullptr || params == nullptr) {
    set_reason(reason, "NULL argument");
    return RC_TWIN_E_NULL;
  }
  const char* why = nullptr;
  auto model = build(v->tier, v->integrator, params, &why);
  if (!model) {
    set_reason(reason, why != nullptr ? why : "rc_twin: cannot create model");
    return RC_TWIN_E_INVALID;
  }
  v->model = std::move(model);
  return RC_TWIN_OK;
}

RC_TWIN_API rc_twin_status rc_twin_get_params(const rc_twin_vehicle* v,
                                              rc_twin_params* out) {
  if (v == nullptr || out == nullptr) return RC_TWIN_E_NULL;
  from_core(v->model->params(), *out);
  return RC_TWIN_OK;
}

RC_TWIN_API rc_twin_status rc_twin_reset(rc_twin_vehicle* v,
                                         const rc_twin_state* initial) {
  if (v == nullptr) return RC_TWIN_E_NULL;
  v->state = slipx::VehicleState{};
  if (initial != nullptr) to_core(*initial, v->state);
  return RC_TWIN_OK;
}

RC_TWIN_API rc_twin_status rc_twin_get_state(const rc_twin_vehicle* v,
                                             rc_twin_state* out) {
  if (v == nullptr || out == nullptr) return RC_TWIN_E_NULL;
  from_core(v->state, *out);
  return RC_TWIN_OK;
}

RC_TWIN_API rc_twin_status rc_twin_step(rc_twin_vehicle* v, double steer_cmd,
                                        double accel_cmd, double dt,
                                        uint32_t n) {
  if (v == nullptr) return RC_TWIN_E_NULL;
  if (!(std::isfinite(dt) && dt > 0.0) || n == 0 || !std::isfinite(steer_cmd) ||
      !std::isfinite(accel_cmd)) {
    return RC_TWIN_E_INVALID;
  }
  const slipx::DriveInput input{steer_cmd, accel_cmd};
  // Step a copy so a failure leaves the published state untouched.
  slipx::VehicleState s = v->state;
  try {
    if (v->ground_fn == nullptr) {
      for (uint32_t k = 0; k < n; ++k) v->model->step(s, input, dt);
    } else {
      for (uint32_t k = 0; k < n; ++k) {
        slipx::GroundContact g;
        if (!sample_ground(*v, s, &g)) return RC_TWIN_E_GROUND;
        if (!v->model->step_on(s, input, g, dt)) return RC_TWIN_E_GROUND;
      }
    }
  } catch (...) {
    return RC_TWIN_E_INTERNAL;
  }
  v->state = s;
  return RC_TWIN_OK;
}

RC_TWIN_API rc_twin_status rc_twin_set_ground_query(rc_twin_vehicle* v,
                                                    rc_twin_ground_fn fn,
                                                    void* user) {
  if (v == nullptr) return RC_TWIN_E_NULL;
  v->ground_fn = fn;
  v->ground_user = (fn != nullptr) ? user : nullptr;
  return RC_TWIN_OK;
}

}  // extern "C"
