// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
// Modified 2026-10-19 by the rc-physics-twin project: see THIRD_PARTY.md.
//
// L2: double-track with load transfer, MF-lite tyres, relaxation, and the
// drivetrain: differential, ESC, battery and steering servo (ADR-0031).
//
// Thirteen states: vx, vy, r, four lagged slip angles, steer_rate, soc, x, y,
// yaw, steer. This is the first tier at which CoG height, track width, tyre
// compound, differential type and battery state do anything, and it is the
// tier the system identification in P2 fits against.
//
// The pieces it assembles were each built and tested on their own, and each is
// a pure function this file calls rather than reimplements: load_transfer.hpp
// (ADR-0022), tyre.hpp (ADR-0023) and relaxation.hpp (ADR-0026). The
// drivetrain and actuators are this file's own, and ADR-0031 is their record.
//
// ============================================================================
// What is present that L1 does not have
// ============================================================================
//
//   Four contact patches, each with its own vertical load, slip angle and
//   friction budget. A left turn now loads the right-hand wheels, and the
//   inside wheels can lift.
//
//   A real saturation shape. MF-lite has a peak and a falling branch beyond
//   it, so the car can spin rather than sliding and recovering the moment the
//   slip angle comes back, which is what L1's clip does.
//
//   Combined slip. Longitudinal and lateral force compete for one friction
//   budget per tyre, so braking into a corner costs cornering force.
//
//   A tyre transient. Force follows a change in slip angle over a relaxation
//   length rather than instantly.
//
//   A drivetrain. The drive layout selects the driven axle (or both), the
//   differential splits torque within it (open, spool or preloaded LSD), and
//   braking goes through the driven axle only, because a 1/10-scale car
//   brakes through its motor and has no friction brakes.
//
//   An ESC and a battery. Available torque follows a torque-speed curve
//   scaled by actual pack voltage, capped by the current limit; braking
//   torque is capped by the regen limit, which is usually the operative
//   brake limit and is weak. Pack voltage sags under load through the
//   internal resistance, and state of charge decays with the current drawn.
//
//   A steering servo. The road wheel angle follows the command through a
//   slew-limited second-order lag, so commanded and achieved steer differ
//   and both are visible (DriveInput::steer_cmd against VehicleState::steer).
//
// ============================================================================
// What is absent, and is absent deliberately rather than forgotten
// ============================================================================
//
//   No wheel rotational state (ADR-0027) unless params.wheel_inertia > 0
//   (rc-physics-twin ADR 0006, below). Wheel speeds and slip ratios are
//   reported quasi-statically from the delivered forces, so a locked or a
//   spinning wheel is not a dynamic event this tier can represent. When the
//   friction ellipse caps one wheel of a spool axle, the reported wheel
//   speeds break the shared-speed constraint in favour of force consistency;
//   the force is the trajectory-authoritative quantity.
//
//   No low-voltage cutoff. soc clamps at zero and the curve keeps evaluating
//   at pack_v_empty. A real ESC cuts drive earlier; the cutoff is firmware
//   configuration, not something identifiable from driving, so it is absent
//   rather than guessed.
//
//   Parallel steer, unless params.ackermann is set (rc-physics-twin ADR 0007).
//   Both front wheels take the same road wheel angle. Real steering geometry
//   turns the inner wheel further (Ackermann), and the difference matters at
//   large angles and low speed. There is no Ackermann parameter in the schema
//   to represent it with, and inventing one would be the guess ADR-0009
//   refuses; ADR 0007 adds one that is measured statically on the car.
//
//   No suspension, unless params.has_suspension() (rc-physics-twin ADR 0008,
//   below). Load transfer is quasi-static, so it arrives in the same instant
//   as the acceleration causing it (ADR-0022). Roll and pitch stay zero.
//
// ============================================================================
// The algebraic loops, and how each is closed
// ============================================================================
//
// Vertical load depends on acceleration, acceleration depends on tyre force,
// and tyre force depends on vertical load. That loop is closed by a FIXED two
// pass evaluation: static loads, forces, resulting accelerations, then loads
// again and the final forces. Fixed rather than iterated to a tolerance,
// because a convergence loop is a place for nondeterminism to hide and the
// iteration count would become part of the trajectory (ADR-0027).
//
// The longitudinal loop is closed by not having a wheel rotational state at
// all: the force demand is met up to the friction budget, and the slip ratio
// consistent with the force that was actually delivered is reported.
//
// The electrical loop (torque draws current, current sags the pack, the
// sagged pack rescales the torque curve) is closed by a fixed two passes of
// its own, once per step at the entry state: pass one takes the open-circuit
// voltage and computes the electrical power, pass two solves the terminal
// voltage in closed form and re-evaluates the curve there (ADR-0031). The
// torque budget is therefore a per-step constant, like the clipped command,
// and the derivative stays a pure function.
//
// Validity: forward motion, as at L1, with params.v_eps flooring the speed in
// every slip denominator. Below roughly v_eps, use L0.
//
// ============================================================================
// Ground (rc-physics-twin ADR 0005)
// ============================================================================
//
// step_on takes one ground sample per wheel, held for the step. This tier
// uses them as follows, and uses nothing else about them:
//
//   mu_scale multiplies that tyre's friction budget, both ellipse axes. With
//   MF-lite's B fixed at construction, that scales the tyre's cornering
//   stiffness in proportion too. Real tyres lose less stiffness than peak
//   friction on a slippery surface, so this overstates the stiffness loss;
//   it is the simplest reading that keeps one surface parameter.
//
//   The four normals are averaged into one road plane. Gravity's component
//   along the plane is added to the body-frame accelerations (forward and
//   left axes projected onto the plane), and the component into the plane,
//   g n_z, replaces g in the loads and the rolling resistance. Gravity along
//   the road transfers load through the tyre force that resists it, which is
//   already in the load pass (load_transfer.hpp).
//
//   pos.z is set to the mean contact height. Per-wheel height differences and
//   normals beyond their mean need a suspension, which is L3's.
//
// ============================================================================
// Wheel rotational dynamics (rc-physics-twin ADR 0006)
// ============================================================================
//
// With params.wheel_inertia > 0, omega_w is integrated state. The step is
// split: the chassis integrates as usual with each wheel's
// entry slip velocity (omega R - v_long), every tyre's longitudinal force
// following from its slip ratio (c_kappa at load times kappa, through the
// friction ellipse); then each wheel
// speed is advanced by backward Euler on the linear tyre law, in closed form,
// at the end-of-step loads. That answers ADR-0027's step-size objection: the
// wheel mode's time constant is far below 1 ms, and the implicit update is
// stable for any inertia. A saturated tyre's force barely depends on the wheel
// speed, so there the update is explicit. The ESC torque reaches the wheels
// through the differential as torque; motor braking can stop a wheel but not
// reverse it, which is lock-up. With wheel_inertia == 0 none of this runs.
//
// ============================================================================
// Suspension (rc-physics-twin ADR 0008)
// ============================================================================
//
// With both wheel rates positive the body heaves, pitches and rolls on a
// spring and damper at each corner, absolute to the world: pos.z is the
// height of the point h_cog below the CoG, pitch is nose up, roll is right
// side down, their rates in vel_body.z, rates.y and rates.x. A corner sits at
// Z + x pitch + y roll; its spring carries k (s0 + ground - z) - c dz/dt,
// never negative, with a bump stop ten times stiffer past suspension_travel.
// Those spring forces (times the road normal's z on a grade) are the tyre
// loads, held for the chassis step in place of the two load passes; then the
// six body states advance by RK4 with the tyre forces held, moments about the
// point below the CoG:
//
//   m Z''      = sum F - m g
//   I_yy p''   = sum x F + m h a_x + m g h p
//   I_xx r''   = sum y F + m h a_y + m g h r
//
// a_x, a_y being the CoG's kinematic accelerations. Small angles; pitch and
// roll centres at the ground; the damper sees the body's velocity only.
//
// Still planar: x, y and yaw are the horizontal projection, and the speed is
// integrated into them as if horizontal, which overstates horizontal travel
// by 1 / cos(grade): 0.5 % at a 10 % grade. Level, unscaled ground runs
// exactly the upstream arithmetic, so the published hashes do not move.

#include <array>
#include <cmath>

#include "models_internal.hpp"
#include "slipx/integrator.hpp"
#include "slipx/load_transfer.hpp"
#include "slipx/math.hpp"
#include "slipx/relaxation.hpp"
#include "slipx/tyre.hpp"

namespace slipx {
namespace internal {
namespace {

// Velocity-like indices first (see integrator.hpp). The lagged slip angles
// are grouped with the velocities rather than with the positions: they are
// forces in waiting, and semi-implicit Euler should have them updated before
// the position integration that consumes their effect. steer_rate is
// velocity-like and steer position-like, so the servo pair integrates in the
// same order as vx and x; soc's derivative is a per-step constant and sits
// with the velocities so the position block stays contiguous.
enum : std::size_t {
  kVx = 0, kVy = 1, kR = 2,
  kLag0 = 3,
  kSteerRate = 7, kSoc = 8,
  kX = 9, kY = 10, kYaw = 11, kSteer = 12,
  kN = 13, kNVel = 9
};

// A tyre's friction budget at one vertical load: the two ellipse axes and the
// slip stiffness. This is where the powers of L2 live, so it is also where
// the tier's cost lives.
struct TyreBudget {
  double fx_max = 0.0;                                            //     [N]
  double fy_max = 0.0;                                            //     [N]
  double c_kappa = 0.0;                                           // [N per -]
};

// Everything the derivative needs that does not change within a step.
struct StepConstants {
  double delta_cmd = 0.0;  // travel-clipped steering command         [rad]
  double fx_demand = 0.0;  // longitudinal force demand after the ESC
                           // and battery limits (ADR-0031)             [N]
  double front_share = 0.0;  // fraction of fx_demand to each axle,
  double rear_share = 0.0;   // set by the drive layout                 [-]
  double soc_rate = 0.0;   // d(soc)/dt, a per-step constant          [1/s]

  MfLite tyre_front;       // one FRONT tyre, B derived at its static load
  MfLite tyre_rear;        // one REAR tyre

  // The tyre-only half of the peak force law (tyre.hpp), which depends on the
  // nominal load and never on the load being asked about.
  NominalPeak peak_front;
  NominalPeak peak_rear;

  // Longitudinal slip stiffness per tyre at that tyre's static load, used to
  // report the slip ratio consistent with the delivered force and to close
  // the spool's constrained-speed form. Positive. The two are equal today and
  // are kept separate so an axle split, if one is ever identifiable, does not
  // have to reshape this struct.
  double c_kappa_front = 0.0;                                  //  [N per -]
  double c_kappa_rear = 0.0;                                   //  [N per -]

  double fz_nom_front = 0.0;  // static load on one front tyre           [N]
  double fz_nom_rear = 0.0;   // static load on one rear tyre            [N]

  // Wheel positions in the body frame. Geometry, so per-model rather than
  // per-step, but they are read four times in every force evaluation and this
  // is where the evaluation looks for constants.
  std::array<double, kWheelCount> wheel_x{};                    //      [m]
  std::array<double, kWheelCount> wheel_y{};                    //      [m]

  // The first load pass always runs at the static loads, which do not change
  // within a step and, for a given car, not between steps either. Both the
  // loads and the budgets they imply are therefore per-step constants, and
  // hoisting them here is half of what stopped this tier calling pow two
  // hundred and eighty times a step.
  WheelLoads statics;
  std::array<TyreBudget, kWheelCount> static_budget{};

  // The ground (see the note at the top). The defaults are level, unscaled
  // ground, under which every use below is an exact identity.
  // Wheel dynamics (ADR 0006): on, and each wheel's slip velocity
  // (omega R - v_long) at the entry state, held for the chassis step.
  bool wheels = false;
  std::array<double, kWheelCount> slip_v{};                  //      [m/s]
  // The ESC's total wheel torque as the wheel update sees it, linear in the
  // driven wheels' mean speed: esc_a - esc_b * omega_mean. On the
  // torque-speed curve esc_b is the curve's slope (the motor's back-EMF
  // damping); anywhere else the torque is the step's constant budget.
  double esc_a = 0.0;                                          //    [N m]
  double esc_b = 0.0;                                          // [N m s]

  // Suspension (ADR 0008): on, per-wheel ground heights, static sags, and
  // the loads and budgets the springs give at the step's start.
  bool susp = false;
  std::array<double, kWheelCount> ground_h{};                  //      [m]
  std::array<double, kWheelCount> sag{};                       //      [m]
  WheelLoads susp_loads;
  std::array<TyreBudget, kWheelCount> susp_budget{};

  bool graded = false;          // the road plane is tilted
  Vec3 normal{0.0, 0.0, 1.0};   // mean unit road normal, world frame    [-]
  double g_normal = kGravity;   // gravity into the road            [m/s^2]
  std::array<double, kWheelCount> mu{1.0, 1.0, 1.0, 1.0};          //  [-]
};

// Gravity's component along the road plane, on the body's forward and left
// axes projected onto it. Downhill ahead tilts the normal forward, so the
// forward component is positive; a left side higher than the right tilts it
// right, so the left component is negative and the car is pushed right.
inline void grade_accel(const StepConstants& c, double yaw, double* ax,
                        double* ay) {
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);
  const double fn = c.normal.x * cy + c.normal.y * sy;    // forward . n
  const double ln = -c.normal.x * sy + c.normal.y * cy;   // left . n
  *ax = kGravity * fn * c.normal.z / std::sqrt(1.0 - fn * fn);
  *ay = kGravity * ln * c.normal.z / std::sqrt(1.0 - ln * ln);
}

// Everything the diagnostics block wants and the integrator does not.
struct Forces {
  std::array<double, kWheelCount> alpha{};    // geometric slip angle  [rad]
  std::array<double, kWheelCount> kappa{};    // reported slip ratio     [-]
  std::array<double, kWheelCount> fx{};       // delivered, wheel frame  [N]
  std::array<double, kWheelCount> fy{};       //                         [N]
  std::array<double, kWheelCount> fz{};       //                         [N]
  std::array<double, kWheelCount> vx_wheel{}; // wheel centre, body x  [m/s]
  std::array<bool, kWheelCount> saturated{};
  // What the wheel update needs (ADR 0006); filled only with wheel dynamics.
  std::array<double, kWheelCount> v_long{};   // along the wheel heading[m/s]
  std::array<double, kWheelCount> v_abs{};    // |v_long| floored at v_eps
  std::array<double, kWheelCount> c_kappa{};  // slip stiffness at load
  std::array<double, kWheelCount> fy_pure{};  // lateral demand          [N]
  std::array<double, kWheelCount> fx_max{};   // ellipse axes            [N]
  std::array<double, kWheelCount> fy_max{};
  double ax = 0.0;                            //                    [m/s^2]
  double ay = 0.0;                            //                    [m/s^2]
  double mz = 0.0;                            // yaw moment          [N m]
  double transfer_long = 0.0;                 //                        [N]
  double transfer_lat = 0.0;                  //                        [N]
};

// What one force pass knows about a wheel before the differential splits the
// longitudinal demand.
struct WheelPre {
  double fy_pure = 0.0;   // lateral demand from the lagged slip     [N]
  double fx_max = 0.0;    // friction budget axes at this load       [N]
  double fy_max = 0.0;    //                                         [N]
  double c_kappa = 0.0;   // slip stiffness at this load       [N per -]
  double vx_safe = 0.0;   // floored wheel-centre speed            [m/s]
};

// What a derivative evaluation knows about a wheel before it knows a vertical
// load, which is everything expensive about it.
//
// The two load passes of ADR-0027 see identical values here: a slip angle is
// an arctangent of the wheel's own velocity, the Magic Formula shape term is
// two more arctangents and a sine of the LAGGED slip angle, and none of the
// four depends on how hard the tyre is pressed into the road. Evaluating them
// once per derivative instead of once per pass halves the transcendental
// count of the tier, and changes no arithmetic: the same function of the same
// arguments is the same number.
struct WheelKinematics {
  double alpha = 0.0;     // geometric slip angle                  [rad]
  double shape = 0.0;     // MF shape term at the lagged slip        [-]
  double vxw = 0.0;       // wheel centre velocity, body x         [m/s]
  double vx_safe = 0.0;   // the same, floored away from zero      [m/s]
  double v_long = 0.0;    // along the wheel heading (ADR 0006)    [m/s]
  double v_abs = 0.0;     // |v_long| floored at v_eps             [m/s]
};

struct Kinematics {
  std::array<WheelKinematics, kWheelCount> wheel{};
  double cos_delta = 1.0;
  double sin_delta = 0.0;
  // Per-wheel road wheel angle and its trigonometry (ADR 0007). The rear
  // entries are zero; with ackermann == 0 the front entries are delta and
  // the two values above, so the arithmetic is the parallel-steer one.
  std::array<double, kWheelCount> steer_w{};
  std::array<double, kWheelCount> cos_w{1.0, 1.0, 1.0, 1.0};
  std::array<double, kWheelCount> sin_w{};
  double resistance = 0.0;  // drag plus rolling resistance          [N]
};

class DoubleTrackL2 final : public VehicleModel {
 public:
  DoubleTrackL2(const VehicleParams& p, Integrator integ)
      : params_(p), integrator_(integ) {}

  Tier tier() const override { return Tier::L2_DoubleTrack; }
  const VehicleParams& params() const override { return params_; }
  Integrator integrator() const override { return integrator_; }
  std::size_t state_dimension() const override { return kN; }

  void step(VehicleState& s, const DriveInput& u, double dt,
            StepDiagnostics* out) const override {
    run(s, u, nullptr, dt, out);
  }

  bool step_on(VehicleState& s, const DriveInput& u, const GroundContact& g,
               double dt, StepDiagnostics* out) const override {
    if (validate(g) != nullptr) return false;
    run(s, u, &g, dt, out);
    return true;
  }

 private:
  void run(VehicleState& s, const DriveInput& u, const GroundContact* ground,
           double dt, StepDiagnostics* out) const {
    // The body's state at the step's start (ADR 0008), before s is written.
    const std::array<double, 6> body0 = {s.vel_body.z, s.rates.y, s.rates.x,
                                         s.pos.z,      s.pitch,   s.roll};
    const double delta_cmd = clamp(u.steer_cmd, -params_.steer_max,
                                   params_.steer_max);
    const bool steer_sat = (delta_cmd != u.steer_cmd);

    double accel = clamp(u.accel_cmd, -params_.decel_max, params_.accel_max);
    const bool accel_sat = (accel != u.accel_cmd);

    bool speed_sat = false;
    if (s.vel_body.x >= params_.v_max && accel > 0.0) {
      accel = 0.0;
      speed_sat = true;
    }

    StepConstants c;
    c.delta_cmd = delta_cmd;
    c.wheels = (params_.wheel_inertia > 0.0);
    if (ground != nullptr) {
      // One road plane from the four normals, each normalised first so a
      // sample's length carries no weight. Axle pairs first: mirror symmetry.
      Vec3 n[kWheelCount];
      for (unsigned i = 0; i < kWheelCount; ++i) {
        const Vec3& w = ground->wheel[i].normal;
        const double len = std::sqrt(w.x * w.x + w.y * w.y + w.z * w.z);
        n[i] = Vec3{w.x / len, w.y / len, w.z / len};
        c.mu[i] = ground->wheel[i].mu_scale;
      }
      Vec3 sum{(n[kFrontLeft].x + n[kFrontRight].x) +
                   (n[kRearLeft].x + n[kRearRight].x),
               (n[kFrontLeft].y + n[kFrontRight].y) +
                   (n[kRearLeft].y + n[kRearRight].y),
               (n[kFrontLeft].z + n[kFrontRight].z) +
                   (n[kRearLeft].z + n[kRearRight].z)};
      const double len =
          std::sqrt(sum.x * sum.x + sum.y * sum.y + sum.z * sum.z);
      c.normal = Vec3{sum.x / len, sum.y / len, sum.z / len};
      c.graded = (c.normal.x != 0.0 || c.normal.y != 0.0);
      c.g_normal = kGravity * c.normal.z;
    }
    for (unsigned i = 0; i < kWheelCount; ++i) {
      wheel_offset(i, &c.wheel_x[i], &c.wheel_y[i]);
    }
    if (c.wheels) {
      // Holding the slip velocity rather than the wheel speed over the
      // chassis step: a rolling wheel keeps its slip while the car speeds up
      // or slows within the step, so the chassis does not see a wheel that
      // lags the road by a whole step's change in speed (ADR 0006).
      std::array<double, kWheelCount> st{};
      std::array<double, kWheelCount> cw{};
      std::array<double, kWheelCount> sw{};
      front_angles(s.steer, std::cos(s.steer), std::sin(s.steer), &st, &cw, &sw);
      for (unsigned i = 0; i < kWheelCount; ++i) {
        const bool front = (i == kFrontLeft || i == kFrontRight);
        const double vxw = s.vel_body.x - s.rates.z * c.wheel_y[i];
        const double vyw = s.vel_body.y + s.rates.z * c.wheel_x[i];
        const double v_long = front ? vxw * cw[i] + vyw * sw[i] : vxw;
        c.slip_v[i] = s.omega_w[i] * params_.wheel_radius - v_long;
      }
    }
    switch (params_.layout) {
      case DriveLayout::kFrontWheelDrive:
        c.front_share = 1.0;
        c.rear_share = 0.0;
        break;
      case DriveLayout::kRearWheelDrive:
        c.front_share = 0.0;
        c.rear_share = 1.0;
        break;
      case DriveLayout::kAllWheelDrive:
        // Locked centre, 50/50: a typical 1/10-scale 4WD is a belt or shaft
        // with no centre differential, and the schema deliberately has no
        // field for one (ADR-0031).
        c.front_share = 0.5;
        c.rear_share = 0.5;
        break;
    }

    // ------------------------------------------------------ ESC and battery
    // Evaluated once per step at the entry state, quasi-static over one step
    // (ADR-0031): the electrical time constants are far below the 1 kHz step
    // and pack state moves on the scale of seconds. The result is a per-step
    // torque budget, which keeps the derivative pure.
    //
    // The curve speed is the mean of the driven wheels' entry speeds. A state
    // constructed by hand at speed with omega_w still zero gets one step of
    // stall-region torque before the report catches up; that is noted in
    // ADR-0031 and is not a bug.
    double omega_mean = 0.0;
    switch (params_.layout) {
      case DriveLayout::kFrontWheelDrive:
        omega_mean = 0.5 * (s.omega_w[kFrontLeft] + s.omega_w[kFrontRight]);
        break;
      case DriveLayout::kRearWheelDrive:
        omega_mean = 0.5 * (s.omega_w[kRearLeft] + s.omega_w[kRearRight]);
        break;
      case DriveLayout::kAllWheelDrive:
        omega_mean =
            0.5 * (0.5 * (s.omega_w[kFrontLeft] + s.omega_w[kFrontRight]) +
                   0.5 * (s.omega_w[kRearLeft] + s.omega_w[kRearRight]));
        break;
    }

    const double t_cmd = params_.mass * accel * params_.wheel_radius;
    const double soc_entry = clamp(s.soc, 0.0, 1.0);
    const double ocv =
        params_.pack_v_empty +
        soc_entry * (params_.pack_v_full - params_.pack_v_empty);

    // The torque the ESC will pass at a given pack voltage: the voltage-scaled
    // curve for drive, the amp caps both ways.
    const auto esc_budget = [&](double v_pack) {
      const double scale = v_pack / params_.pack_nominal_v;
      double curve = params_.torque_stall * scale *
                     (1.0 - omega_mean / (params_.omega_free * scale));
      if (curve < 0.0) curve = 0.0;
      const double drive_cap =
          std::fmin(curve, params_.torque_per_amp * params_.current_max);
      const double regen_cap =
          params_.torque_per_amp * params_.regen_current_max;
      return clamp(t_cmd, -regen_cap, drive_cap);
    };
    // Battery-terminal electrical power for a wheel torque. Efficiency loses
    // power in both directions: it divides on discharge and multiplies on
    // regen (ADR-0031).
    const auto electrical_power = [&](double torque) {
      const double p_mech = torque * omega_mean;
      return (p_mech >= 0.0) ? p_mech / params_.drive_efficiency
                             : p_mech * params_.drive_efficiency;
    };

    // Pass one at open-circuit voltage; pass two at the sagged terminal
    // voltage, solved in closed form from V^2 - ocv V + R P = 0 (upper root).
    // A negative discriminant means the demand exceeds the pack's maximum
    // deliverable power, and V clamps to the peak-power point ocv/2.
    const double p1 = electrical_power(esc_budget(ocv));
    double v_pack = ocv;
    const double rp = params_.pack_internal_resistance * p1;
    if (rp != 0.0) {
      const double disc = ocv * ocv - 4.0 * rp;
      v_pack = (disc >= 0.0) ? 0.5 * (ocv + std::sqrt(disc)) : 0.5 * ocv;
    }
    const double t_budget = esc_budget(v_pack);
    const double p2 = electrical_power(t_budget);
    const double pack_current = p2 / v_pack;
    const bool esc_sat = (t_budget != t_cmd);

    c.fx_demand = t_budget / params_.wheel_radius;
    c.esc_a = t_budget;
    if (c.wheels) {
      // Is the torque the curve's? The same expression as esc_budget, so the
      // comparison is exact.
      const double scale = v_pack / params_.pack_nominal_v;
      const double curve = params_.torque_stall * scale *
                           (1.0 - omega_mean / (params_.omega_free * scale));
      const double cap = params_.torque_per_amp * params_.current_max;
      if (t_budget > 0.0 && t_budget == curve && curve < cap) {
        c.esc_a = params_.torque_stall * scale;
        c.esc_b = params_.torque_stall / params_.omega_free;
      }
    }
    c.soc_rate = -pack_current / (3600.0 * params_.pack_capacity_ah);

    // The tyres are built once per step rather than once per derivative
    // evaluation: B is a function of the parameters and the static load only,
    // so it does not change within a step (CORE-03).
    // The tyre's nominal load is a property of this car on level ground and
    // stays there on a grade; the first load pass uses the loads the grade
    // actually leaves on the road. Level ground: the same numbers.
    const WheelLoads level = static_loads(params_);
    c.statics = static_loads(params_, c.g_normal);
    c.fz_nom_front = level.fz[kFrontLeft];
    c.fz_nom_rear = level.fz[kRearLeft];

    // VehicleParams carries cornering stiffness per AXLE, because that is what
    // a single-track tier needs; MF-lite is per tyre, so each axle value is
    // halved here. Doing it in one place keeps the two conventions from
    // meeting anywhere else.
    c.tyre_front = make_mf_lite(params_.tyre_front, 0.5 * params_.c_alpha_f,
                                c.fz_nom_front);
    c.tyre_rear = make_mf_lite(params_.tyre_rear, 0.5 * params_.c_alpha_r,
                               c.fz_nom_rear);
    c.peak_front = nominal_peak(c.tyre_front);
    c.peak_rear = nominal_peak(c.tyre_rear);
    // One slip stiffness for all four tyres: the run that identifies it cannot
    // separate the axles. See the field's note in params.hpp.
    c.c_kappa_front = params_.c_kappa;
    c.c_kappa_rear = params_.c_kappa;

    // The budget the first load pass will use, every pass, every derivative.
    budget_at(c, c.statics, &c.static_budget);

    c.susp = params_.has_suspension();
    if (c.susp) {
      for (unsigned i = 0; i < kWheelCount; ++i) {
        const bool front = (i == kFrontLeft || i == kFrontRight);
        c.ground_h[i] = (ground != nullptr) ? ground->wheel[i].height : 0.0;
        // The sag that holds the car level at rest on level ground.
        c.sag[i] = level.fz[i] /
                   (front ? params_.wheel_rate_front : params_.wheel_rate_rear);
      }
      std::array<double, kWheelCount> spring{};
      spring_forces(c, body0, &spring);
      WheelLoads& L = c.susp_loads;
      for (unsigned i = 0; i < kWheelCount; ++i) {
        L.fz[i] = spring[i] * c.normal.z;
        if (spring[i] == 0.0) L.wheel_lifted = true;
      }
      L.fz_front = L.fz[kFrontLeft] + L.fz[kFrontRight];
      L.fz_rear = L.fz[kRearLeft] + L.fz[kRearRight];
      L.transfer_long = c.statics.fz_front - L.fz_front;
      L.transfer_lat = 0.5 * ((L.fz[kFrontRight] - L.fz[kFrontLeft]) +
                              (L.fz[kRearRight] - L.fz[kRearLeft]));
      budget_at(c, L, &c.susp_budget);
    }

    StateVec<kN> y{};
    y[kVx] = s.vel_body.x;
    y[kVy] = s.vel_body.y;
    y[kR] = s.rates.z;
    for (unsigned i = 0; i < kWheelCount; ++i) y[kLag0 + i] = s.alpha_lag[i];
    y[kSteerRate] = s.steer_rate;
    y[kSoc] = soc_entry;
    y[kX] = s.pos.x;
    y[kY] = s.pos.y;
    y[kYaw] = s.yaw;
    y[kSteer] = s.steer;

    const auto deriv = [&](const StateVec<kN>& q) {
      Forces f;
      return derivative(q, c, &f);
    };

    integrate<kN>(integrator_, y, dt, kNVel, deriv);

    // The mechanical end stop, inelastic: travel clamps and the rate dies at
    // the stop (ADR-0031). The command is already inside the travel, so this
    // only engages on overshoot.
    if (y[kSteer] > params_.steer_max) {
      y[kSteer] = params_.steer_max;
      y[kSteerRate] = 0.0;
    } else if (y[kSteer] < -params_.steer_max) {
      y[kSteer] = -params_.steer_max;
      y[kSteerRate] = 0.0;
    }
    y[kSoc] = clamp(y[kSoc], 0.0, 1.0);

    s.vel_body.x = y[kVx];
    s.vel_body.y = y[kVy];
    s.vel_body.z = 0.0;
    s.rates = Vec3{0.0, 0.0, y[kR]};
    for (unsigned i = 0; i < kWheelCount; ++i) s.alpha_lag[i] = y[kLag0 + i];
    s.pos.x = y[kX];
    s.pos.y = y[kY];
    s.yaw = wrap_to_pi(y[kYaw]);
    s.steer = y[kSteer];
    s.steer_rate = y[kSteerRate];
    s.soc = y[kSoc];
    // Algebraic, like the per-wheel Fz: a reported consequence of this step's
    // load, not a degree of freedom (ADR-0031).
    s.pack_v = v_pack;
    if (ground != nullptr && !c.susp) s.pos.z = mean_height(*ground);

    // Reported at the end of the step, so the numbers describe the state the
    // caller now holds. Same convention as L0 and L1.
    Forces f;
    const StateVec<kN> d = derivative(y, c, &f);

    if (c.wheels) {
      // Integrated wheel speeds (ADR 0006), at the end-of-step loads.
      update_wheels(c, f, t_budget, dt, &s.omega_w);
    } else {
      // Wheel speeds follow from the reported slip ratio rather than from a
      // rotational state (ADR-0027). kappa = (omega R - v) / v, so
      // omega = v (1 + kappa) / R. These are consistent with the delivered
      // force by construction and are NOT an independent degree of freedom.
      for (unsigned i = 0; i < kWheelCount; ++i) {
        s.omega_w[i] =
            f.vx_wheel[i] * (1.0 + f.kappa[i]) / params_.wheel_radius;
      }
    }
    for (unsigned i = 0; i < kWheelCount; ++i) s.Fz[i] = f.fz[i];

    if (c.susp) {
      // The body, with the tyre forces held from the end-of-step evaluation:
      // the CoG's kinematic accelerations include gravity along the road.
      double gx = 0.0;
      double gy = 0.0;
      if (c.graded) grade_accel(c, y[kYaw], &gx, &gy);
      std::array<double, 6> b = body0;
      integrate_body(c, f.ax + gx, f.ay + gy, dt, &b);
      s.vel_body.z = b[0];
      s.rates.y = b[1];
      s.rates.x = b[2];
      s.pos.z = b[3];
      s.pitch = b[4];
      s.roll = b[5];
    }

    if (out != nullptr) {
      reset_diagnostics(*out, Tier::L2_DoubleTrack);
      for (unsigned i = 0; i < kWheelCount; ++i) {
        out->alpha[i] = f.alpha[i];
        out->kappa[i] = f.kappa[i];
        out->fx[i] = f.fx[i];
        out->fy[i] = f.fy[i];
        out->fz[i] = f.fz[i];
        out->tyre_saturated[i] = f.saturated[i];
      }

      // Axle-resolved values, so a controller or a plot written against L1
      // keeps working. Slip angle is the mean over the axle and force is the
      // sum, which are the two that reduce to L1's single tyre correctly.
      out->alpha_front = 0.5 * (f.alpha[kFrontLeft] + f.alpha[kFrontRight]);
      out->alpha_rear = 0.5 * (f.alpha[kRearLeft] + f.alpha[kRearRight]);
      out->fy_front = f.fy[kFrontLeft] + f.fy[kFrontRight];
      out->fy_rear = f.fy[kRearLeft] + f.fy[kRearRight];
      out->fz_front = f.fz[kFrontLeft] + f.fz[kFrontRight];
      out->fz_rear = f.fz[kRearLeft] + f.fz[kRearRight];

      out->ax = d[kVx] - y[kVy] * y[kR];
      out->ay = d[kVy] + y[kVx] * y[kR];
      if (c.graded) {
        // Specific force: what an accelerometer reads, which excludes the
        // gravity the derivative carries on a grade.
        double gx = 0.0;
        double gy = 0.0;
        grade_accel(c, y[kYaw], &gx, &gy);
        out->ax -= gx;
        out->ay -= gy;
      }
      out->load_transfer_long = f.transfer_long;
      out->load_transfer_lat = f.transfer_lat;

      out->drive_torque = t_budget;
      out->pack_current = pack_current;

      out->steer_saturated = steer_sat;
      out->accel_saturated = accel_sat;
      out->speed_saturated = speed_sat;
      out->esc_saturated = esc_sat;
    }
  }

  // ----------------------------------------------------- body (ADR 0008)

  // The spring and damper force at each corner for body state
  // b = {Z', pitch', roll', Z, pitch, roll}. Never negative: a wheel whose
  // spring would pull has left the ground.
  void spring_forces(const StepConstants& c, const std::array<double, 6>& b,
                     std::array<double, kWheelCount>* out) const {
    for (unsigned i = 0; i < kWheelCount; ++i) {
      const bool front = (i == kFrontLeft || i == kFrontRight);
      const double k = front ? params_.wheel_rate_front : params_.wheel_rate_rear;
      const double d = front ? params_.damping_front : params_.damping_rear;
      const double z = (b[3] + c.wheel_x[i] * b[4]) + c.wheel_y[i] * b[5];
      const double zd = (b[0] + c.wheel_x[i] * b[1]) + c.wheel_y[i] * b[2];
      const double comp = (c.sag[i] + c.ground_h[i]) - z;
      double force = 0.0;
      if (comp > 0.0) {
        force = k * comp - d * zd;
        const double over = comp - (c.sag[i] + params_.suspension_travel);
        if (over > 0.0) force += 10.0 * k * over;  // the bump stop
        if (force < 0.0) force = 0.0;
      }
      (*out)[i] = force;
    }
  }

  // Advance the body's six states by dt with the CoG's kinematic
  // accelerations (ax, ay) held. Velocity-like states first, as integrate()
  // expects. Sums are paired by axle, so a mirrored state mirrors exactly.
  void integrate_body(const StepConstants& c, double ax, double ay, double dt,
                      std::array<double, 6>* b) const {
    const double m = params_.mass;
    const double h = params_.h_cog;
    const auto deriv = [&](const StateVec<6>& q) {
      std::array<double, kWheelCount> f{};
      spring_forces(c, q, &f);
      const double sum = (f[kFrontLeft] + f[kFrontRight]) +
                         (f[kRearLeft] + f[kRearRight]);
      const double sum_x =
          (c.wheel_x[kFrontLeft] * f[kFrontLeft] +
           c.wheel_x[kFrontRight] * f[kFrontRight]) +
          (c.wheel_x[kRearLeft] * f[kRearLeft] +
           c.wheel_x[kRearRight] * f[kRearRight]);
      const double sum_y =
          (c.wheel_y[kFrontLeft] * f[kFrontLeft] +
           c.wheel_y[kFrontRight] * f[kFrontRight]) +
          (c.wheel_y[kRearLeft] * f[kRearLeft] +
           c.wheel_y[kRearRight] * f[kRearRight]);
      StateVec<6> d{};
      d[0] = sum / m - kGravity;
      d[1] = (sum_x + m * h * ax + m * kGravity * h * q[4]) / params_.iyy;
      d[2] = (sum_y + m * h * ay + m * kGravity * h * q[5]) / params_.ixx;
      d[3] = q[0];
      d[4] = q[1];
      d[5] = q[2];
      return d;
    };
    StateVec<6> y = *b;
    integrate<6>(integrator_, y, dt, 3, deriv);
    *b = y;
  }

  // ---------------------------------------------------- steering geometry

  // Each front wheel's road wheel angle for reference angle delta (ADR 0007):
  // the ideal Ackermann angle points the wheel at the turn centre of a
  // virtual centre wheel at delta, on the rear-axle line, and ackermann
  // blends from parallel (0) toward it. Odd in delta with left and right
  // swapped exactly, so a right turn mirrors a left one bit for bit. With
  // ackermann == 0 both front wheels get delta and the given cos and sin.
  void front_angles(double delta, double cos_delta, double sin_delta,
                    std::array<double, kWheelCount>* steer,
                    std::array<double, kWheelCount>* cos_w,
                    std::array<double, kWheelCount>* sin_w) const {
    (*steer)[kRearLeft] = 0.0;
    (*steer)[kRearRight] = 0.0;
    (*cos_w)[kRearLeft] = 1.0;
    (*cos_w)[kRearRight] = 1.0;
    (*sin_w)[kRearLeft] = 0.0;
    (*sin_w)[kRearRight] = 0.0;
    if (params_.ackermann == 0.0) {
      (*steer)[kFrontLeft] = delta;
      (*steer)[kFrontRight] = delta;
      (*cos_w)[kFrontLeft] = cos_delta;
      (*cos_w)[kFrontRight] = cos_delta;
      (*sin_w)[kFrontLeft] = sin_delta;
      (*sin_w)[kFrontRight] = sin_delta;
      return;
    }
    const double wb = params_.wheelbase();
    const double ht = 0.5 * params_.track_front;
    const double tn = std::tan(delta);
    const double ideal_l = std::atan(wb * tn / (wb - ht * tn));
    const double ideal_r = std::atan(wb * tn / (wb + ht * tn));
    const double dl = delta + params_.ackermann * (ideal_l - delta);
    const double dr = delta + params_.ackermann * (ideal_r - delta);
    (*steer)[kFrontLeft] = dl;
    (*steer)[kFrontRight] = dr;
    (*cos_w)[kFrontLeft] = std::cos(dl);
    (*cos_w)[kFrontRight] = std::cos(dr);
    (*sin_w)[kFrontLeft] = std::sin(dl);
    (*sin_w)[kFrontRight] = std::sin(dr);
  }

  // ----------------------------------------------- wheel update (ADR 0006)

  // One tyre's force at wheel speed w, with everything else as evaluated.
  CombinedForce tyre_at(const Forces& f, unsigned i, double w) const {
    const double kappa = (w * params_.wheel_radius - f.v_long[i]) / f.v_abs[i];
    return friction_ellipse(f.c_kappa[i] * kappa, f.fy_pure[i], f.fx_max[i],
                            f.fy_max[i]);
  }

  // The wheel speed at which tyre i's longitudinal force reaches its peak,
  // for the lateral demand it carries, on the side of the force's sign. Past
  // it the force no longer grows with the wheel speed; short of it the tyre
  // is linear. Returns false for a tyre with no slip stiffness (unloaded).
  bool knee(const Forces& f, unsigned i, double force, double* w) const {
    if (!(f.c_kappa[i] > 0.0) || force == 0.0) return false;
    const double ny = f.fy_max[i] > 0.0 ? f.fy_pure[i] / f.fy_max[i] : 1.0;
    const double room = 1.0 - ny * ny;
    const double fx_peak = room > 0.0 ? f.fx_max[i] * std::sqrt(room) : 0.0;
    const double slip = fx_peak * f.v_abs[i] / f.c_kappa[i];
    *w = (f.v_long[i] + (force > 0.0 ? slip : -slip)) / params_.wheel_radius;
    return true;
  }

  // A saturated update uses the peak force and is explicit. If that carries
  // the wheel back past the peak, the implicit answer is the peak itself: the
  // force there is continuous and the wheel settles on it. Without this a
  // light wheel overshoots to the far side of the peak every step and its
  // force flips sign: chatter.
  double settle_at_knee(const Forces& f, unsigned i, double force,
                        double w) const {
    double wk = 0.0;
    if (!knee(f, i, force, &wk)) return w;
    return force > 0.0 ? std::fmax(w, wk) : std::fmin(w, wk);
  }

  // One wheel with its own torque: backward Euler on I dw/dt = T - R Fx with
  // Fx linear in w, solved in closed form; explicit if the tyre saturates.
  // Used for undriven wheels (no torque) and the LSD decision.
  double wheel_solve(const Forces& f, unsigned i, double torque, double w0,
                     double a) const {
    const double r = params_.wheel_radius;
    const double k = f.c_kappa[i] / f.v_abs[i];
    double w = (a * w0 + torque + r * k * f.v_long[i]) / (a + r * r * k);
    const CombinedForce at = tyre_at(f, i, w);
    if (at.saturated) {
      w = settle_at_knee(f, i, at.fx, w0 + (torque - r * at.fx) / a);
    }
    return w;
  }

  // A locked axle: one speed, both tyres, inertia 2 I. Pair sums only, so the
  // solve is exactly mirror symmetric.
  double spool_solve(const Forces& f, unsigned l, unsigned rr, double torque,
                     double w0, double a) const {
    const double r = params_.wheel_radius;
    const double kl = f.c_kappa[l] / f.v_abs[l];
    const double kr = f.c_kappa[rr] / f.v_abs[rr];
    double w = (2.0 * a * w0 + torque +
                r * (kl * f.v_long[l] + kr * f.v_long[rr])) /
               (2.0 * a + r * r * (kl + kr));
    const CombinedForce fl = tyre_at(f, l, w);
    const CombinedForce fr = tyre_at(f, rr, w);
    if (fl.saturated || fr.saturated) {
      w = w0 + (torque - r * (fl.fx + fr.fx)) / (2.0 * a);
    }
    return w;
  }

  // One degree of freedom of the driven wheels: a single wheel, or a locked
  // pair turning at one speed.
  struct Dof {
    unsigned wheel[2] = {0, 0};
    unsigned n = 1;          // wheels in it
    double share = 0.0;      // fraction of the ESC torque it receives,
                             // signed: under braking, against the rotation
    double extra = 0.0;      // constant torque on top (LSD bias)     [N m]
    double w0 = 0.0;         // entry speed                           [rad/s]
  };

  // The driven wheels, solved together. Each DOF j obeys, by backward Euler,
  //
  //   n_j a (w_j' - w_j) = share_j (A - B w_mean') + extra_j - R F_j(w_j')
  //
  // with w_mean' the mean of the driven wheel speeds, the ESC's coupling.
  // F_j is linear in w_j' for a tyre inside its ellipse and a constant for a
  // saturated one, so the system is a diagonal plus one rank-one term and
  // Sherman-Morrison solves it in closed form. Two passes, always: linear
  // tyres first, then saturated ones frozen at the force the first pass
  // found. Sums are formed per axle and then across axles, so the solve is
  // exactly mirror symmetric.
  void solve_driven(const StepConstants& c, const Forces& f, double a,
                    const Dof* dofs, const unsigned* axle_of, unsigned count,
                    unsigned driven_wheels,
                    std::array<double, kWheelCount>* w) const {
    const double r = params_.wheel_radius;
    std::array<bool, kWheelCount> sat{};
    std::array<double, kWheelCount> fsat{};
    std::array<double, 4> out{};
    const double inv_n = 1.0 / static_cast<double>(driven_wheels);

    for (int pass = 0; pass < 2; ++pass) {
      std::array<double, 4> y{};
      std::array<double, 4> z{};
      // Per axle partial sums of v.y and v.z, then front + rear.
      double num_axle[2] = {0.0, 0.0};
      double den_axle[2] = {0.0, 0.0};
      for (unsigned j = 0; j < count; ++j) {
        const Dof& d = dofs[j];
        double diag = static_cast<double>(d.n) * a;
        double rhs = diag * d.w0 + d.share * c.esc_a + d.extra;
        double tyre_diag[2] = {0.0, 0.0};
        double tyre_rhs[2] = {0.0, 0.0};
        for (unsigned m = 0; m < d.n; ++m) {
          const unsigned i = d.wheel[m];
          if (sat[i]) {
            tyre_rhs[m] = -r * fsat[i];
          } else {
            const double k = f.c_kappa[i] / f.v_abs[i];
            tyre_diag[m] = r * r * k;
            tyre_rhs[m] = r * k * f.v_long[i];
          }
        }
        diag += (d.n == 2) ? (tyre_diag[0] + tyre_diag[1]) : tyre_diag[0];
        rhs += (d.n == 2) ? (tyre_rhs[0] + tyre_rhs[1]) : tyre_rhs[0];
        y[j] = rhs / diag;
        z[j] = d.share * c.esc_b / diag;
        const double v = static_cast<double>(d.n) * inv_n;
        num_axle[axle_of[j]] += v * y[j];
        den_axle[axle_of[j]] += v * z[j];
      }
      const double num = num_axle[0] + num_axle[1];
      const double den = 1.0 + (den_axle[0] + den_axle[1]);
      const double w_mean = num / den;
      for (unsigned j = 0; j < count; ++j) out[j] = y[j] - z[j] * w_mean;

      if (pass == 0) {
        for (unsigned j = 0; j < count; ++j) {
          for (unsigned m = 0; m < dofs[j].n; ++m) {
            const unsigned i = dofs[j].wheel[m];
            const CombinedForce at = tyre_at(f, i, out[j]);
            sat[i] = at.saturated;
            fsat[i] = at.fx;
          }
        }
      }
    }

    for (unsigned j = 0; j < count; ++j) {
      for (unsigned m = 0; m < dofs[j].n; ++m) {
        const unsigned i = dofs[j].wheel[m];
        if (sat[i]) out[j] = settle_at_knee(f, i, fsat[i], out[j]);
      }
      for (unsigned m = 0; m < dofs[j].n; ++m) (*w)[dofs[j].wheel[m]] = out[j];
    }
  }

  // An LSD axle locks while the torque its clutch must carry to hold both
  // wheels at one speed is within the preload (decided with the axle's share
  // of the entry torque); past it, it runs open with the preload biased to
  // the slower wheel, SlipX's force rule in torque.
  bool lsd_locked(const Forces& f, unsigned l, unsigned rr, double torque,
                  double wl0, double wr0, double a) const {
    const double r = params_.wheel_radius;
    const double half = 0.5 * torque;
    const double ws = spool_solve(f, l, rr, torque, 0.5 * (wl0 + wr0), a);
    const double need_l = a * (ws - wl0) - half + r * tyre_at(f, l, ws).fx;
    const double need_r = a * (ws - wr0) - half + r * tyre_at(f, rr, ws).fx;
    return 2.0 * std::fabs(0.5 * (need_l - need_r)) <= params_.lsd_preload;
  }

  void update_wheels(const StepConstants& c, const Forces& f, double t_budget,
                     double dt, std::array<double, kWheelCount>* w) const {
    const double a = params_.wheel_inertia / dt;
    Dof dofs[4];
    unsigned axle_of[4] = {0, 0, 0, 0};
    unsigned count = 0;
    unsigned driven_wheels = 0;

    const auto add_axle = [&](unsigned axle, unsigned l, unsigned rr,
                              double share) {
      const double wl0 = (*w)[l];
      const double wr0 = (*w)[rr];
      driven_wheels += 2;
      bool locked = params_.differential == Differential::kSpool;
      if (params_.differential == Differential::kLsd) {
        locked = lsd_locked(f, l, rr, share * t_budget, wl0, wr0, a);
      }
      if (locked) {
        Dof d;
        d.wheel[0] = l;
        d.wheel[1] = rr;
        d.n = 2;
        d.share = share;
        d.w0 = 0.5 * (wl0 + wr0);
        axle_of[count] = axle;
        dofs[count++] = d;
        return;
      }
      double bias_l = 0.0;
      double bias_r = 0.0;
      if (params_.differential == Differential::kLsd) {
        const double bias = 0.5 * params_.lsd_preload;
        if (wl0 < wr0) {
          bias_l = bias;
          bias_r = -bias;
        } else if (wr0 < wl0) {
          bias_l = -bias;
          bias_r = bias;
        }
      }
      Dof dl;
      dl.wheel[0] = l;
      dl.share = 0.5 * share;
      dl.extra = bias_l;
      dl.w0 = wl0;
      Dof dr;
      dr.wheel[0] = rr;
      dr.share = 0.5 * share;
      dr.extra = bias_r;
      dr.w0 = wr0;
      axle_of[count] = axle;
      dofs[count++] = dl;
      axle_of[count] = axle;
      dofs[count++] = dr;
    };

    if (c.front_share != 0.0) add_axle(0, kFrontLeft, kFrontRight, c.front_share);
    if (c.rear_share != 0.0) add_axle(1, kRearLeft, kRearRight, c.rear_share);

    // Motor braking (a negative budget off the curve) acts against the
    // wheel's rotation whichever way it turns, and stops it rather than
    // reversing it. The direction is the one the tyre alone would turn it
    // this step; a wheel the tyre would not turn at all stays where it is.
    const bool braking = (c.esc_b == 0.0 && c.esc_a < 0.0);
    std::array<double, kWheelCount> free = *w;
    if (braking) {
      StepConstants no_torque = c;
      no_torque.esc_a = 0.0;
      solve_driven(no_torque, f, a, dofs, axle_of, count, driven_wheels, &free);
      for (unsigned j = 0; j < count; ++j) {
        const double wf = free[dofs[j].wheel[0]];
        const double dir = (wf > 0.0) ? 1.0 : (wf < 0.0) ? -1.0 : 0.0;
        dofs[j].share *= dir;
      }
    }
    solve_driven(c, f, a, dofs, axle_of, count, driven_wheels, w);
    if (braking) {
      for (unsigned j = 0; j < count; ++j) {
        const double wf = free[dofs[j].wheel[0]];
        for (unsigned m = 0; m < dofs[j].n; ++m) {
          double& wi = (*w)[dofs[j].wheel[m]];
          if ((wf > 0.0 && wi < 0.0) || (wf < 0.0 && wi > 0.0) || wf == 0.0) {
            wi = 0.0;
          }
        }
      }
    }

    // Undriven wheels roll on their own with no torque.
    if (c.front_share == 0.0) {
      (*w)[kFrontLeft] = wheel_solve(f, kFrontLeft, 0.0, (*w)[kFrontLeft], a);
      (*w)[kFrontRight] = wheel_solve(f, kFrontRight, 0.0, (*w)[kFrontRight], a);
    }
    if (c.rear_share == 0.0) {
      (*w)[kRearLeft] = wheel_solve(f, kRearLeft, 0.0, (*w)[kRearLeft], a);
      (*w)[kRearRight] = wheel_solve(f, kRearRight, 0.0, (*w)[kRearRight], a);
    }
  }

  // Wheel position in the body frame. x forward, y to the LEFT, so the left
  // wheels carry a positive y (conventions.hpp).
  void wheel_offset(unsigned i, double* x, double* y) const {
    const bool front = (i == kFrontLeft || i == kFrontRight);
    const bool left = (i == kFrontLeft || i == kRearLeft);
    *x = front ? params_.lf : -params_.lr;
    const double track = front ? params_.track_front : params_.track_rear;
    *y = left ? 0.5 * track : -0.5 * track;
  }

  // The differential: one axle's longitudinal demand becomes two delivered
  // force pairs (ADR-0031). Closed forms only; the fixed evaluation counts
  // are part of the trajectory, exactly as the load passes are.
  //
  // Mirror symmetry is load-bearing here: every rule below commutes with
  // swapping left and right, including its tie cases, and the invariant test
  // holds the whole tier to it bit for bit.
  void split_axle(unsigned l, unsigned r, double f_axle,
                  const std::array<WheelPre, kWheelCount>& pre,
                  std::array<CombinedForce, kWheelCount>* delivered) const {
    const auto ellipse = [&](unsigned i, double fx) {
      return friction_ellipse(fx, pre[i].fy_pure, pre[i].fx_max, pre[i].fy_max);
    };

    // The spool's constrained-speed solution, shared by kSpool and kLsd.
    // Both wheels turn at one speed, so with the linear slip stiffness c_i
    // the wheel force at a common contact speed omega R is
    // F_i = (c_i / v_i) omega R - c_i, and force balance closes it:
    // omega R = (F_axle + c_l + c_r) / (c_l / v_l + c_r / v_r). The slower
    // (inner) wheel carries the larger slip ratio and drives harder, which
    // is the spool's understeer push arriving from the constraint alone.
    const auto spool_forces = [&](double* fl, double* fr) {
      const double al = pre[l].c_kappa / pre[l].vx_safe;
      const double ar = pre[r].c_kappa / pre[r].vx_safe;
      const double denom = al + ar;
      if (denom <= 0.0) {
        // Both wheels unloaded: nothing to react the torque against.
        *fl = 0.0;
        *fr = 0.0;
        return;
      }
      // The pair sum is bracketed first: a three-term sum associated left to
      // right computes (f_axle + c_l) + c_r, whose rounding differs from its
      // left/right mirror, and the mirror-symmetry invariant catches exactly
      // that (ADR-0004: fixed reduction order in a numerical path).
      const double omega_r =
          (f_axle + (pre[l].c_kappa + pre[r].c_kappa)) / denom;
      *fl = al * omega_r - pre[l].c_kappa;
      *fr = ar * omega_r - pre[r].c_kappa;
    };

    switch (params_.differential) {
      case Differential::kOpen: {
        // Equal torque both sides, and the weaker side caps both: an open
        // differential cannot support a torque difference across the axle,
        // so the side that delivers less sets what the other receives. The
        // exact-tie case keeps both wheels' own ellipse results; re-running
        // one side on a tie would break mirror symmetry.
        const double half = 0.5 * f_axle;
        CombinedForce dl = ellipse(l, half);
        CombinedForce dr = ellipse(r, half);
        const double mag_l = std::fabs(dl.fx);
        const double mag_r = std::fabs(dr.fx);
        if (mag_l < mag_r) {
          dr = ellipse(r, dl.fx);
        } else if (mag_r < mag_l) {
          dl = ellipse(l, dr.fx);
        }
        (*delivered)[l] = dl;
        (*delivered)[r] = dr;
        return;
      }
      case Differential::kSpool: {
        double fl = 0.0;
        double fr = 0.0;
        spool_forces(&fl, &fr);
        (*delivered)[l] = ellipse(l, fl);
        (*delivered)[r] = ellipse(r, fr);
        return;
      }
      case Differential::kLsd: {
        // Locked while the implied torque difference is within the preload;
        // past it the clutch slips and the split falls back to an open one
        // biased by the preload toward the slower wheel, which is the wheel
        // the clutch is feeding torque to. With no speed difference there is
        // no slip direction to bias along and the halves stay equal.
        double fl = 0.0;
        double fr = 0.0;
        spool_forces(&fl, &fr);
        if (std::fabs(fl - fr) * params_.wheel_radius <= params_.lsd_preload) {
          (*delivered)[l] = ellipse(l, fl);
          (*delivered)[r] = ellipse(r, fr);
          return;
        }
        const double half = 0.5 * f_axle;
        const double bias = 0.5 * params_.lsd_preload / params_.wheel_radius;
        if (pre[l].vx_safe < pre[r].vx_safe) {
          fl = half + bias;
          fr = half - bias;
        } else if (pre[r].vx_safe < pre[l].vx_safe) {
          fl = half - bias;
          fr = half + bias;
        } else {
          fl = half;
          fr = half;
        }
        (*delivered)[l] = ellipse(l, fl);
        (*delivered)[r] = ellipse(r, fr);
        return;
      }
    }
  }

  // Everything one derivative evaluation can settle before it knows a vertical
  // load. See the note on WheelKinematics for why this is a separate pass.
  Kinematics kinematics(const StateVec<kN>& q, const StepConstants& c) const {
    const double vx = q[kVx];
    const double vy = q[kVy];
    const double r = q[kR];

    Kinematics kin;

    // The ACHIEVED road wheel angle is state now (ADR-0031), so the steer
    // trigonometry is per evaluation rather than per step.
    const double delta = q[kSteer];
    kin.cos_delta = std::cos(delta);
    kin.sin_delta = std::sin(delta);
    front_angles(delta, kin.cos_delta, kin.sin_delta, &kin.steer_w, &kin.cos_w,
                 &kin.sin_w);

    for (unsigned i = 0; i < kWheelCount; ++i) {
      const double xw = c.wheel_x[i];
      const double yw = c.wheel_y[i];
      const bool front = (i == kFrontLeft || i == kFrontRight);

      // Velocity of the wheel centre in the body frame. The yaw rate makes the
      // outside wheels travel faster than the inside ones, which is the whole
      // reason a double-track model differs from a bicycle.
      const double vxw = vx - r * yw;
      const double vyw = vy + r * xw;

      const double vx_safe = (vxw >= 0.0) ? std::fmax(vxw, params_.v_eps)
                                          : std::fmin(vxw, -params_.v_eps);

      // ISO 8855, and the same construction as L1: the slip angle is measured
      // between the wheel's travel and its plane, and the steered wheels have
      // their plane rotated by delta.
      const double steer_i = kin.steer_w[i];

      const MfLite& tyre = front ? c.tyre_front : c.tyre_rear;

      kin.wheel[i].alpha = std::atan2(vyw, vx_safe) - steer_i;
      // The shape term uses the LAGGED slip angle, which is the state, while
      // the load below is the instantaneous one. That asymmetry is the point
      // of ADR-0026: the force is inside the current friction budget however
      // much history the slip angle carries.
      kin.wheel[i].shape = mf_shape(tyre.b, tyre.c, tyre.e, q[kLag0 + i]);
      kin.wheel[i].vxw = vxw;
      kin.wheel[i].vx_safe = vx_safe;
      if (c.wheels) {
        // The slip ratio's speed is along the wheel's own heading.
        const double v_long =
            front ? vxw * kin.cos_w[i] + vyw * kin.sin_w[i] : vxw;
        kin.wheel[i].v_long = v_long;
        kin.wheel[i].v_abs = std::fmax(std::fabs(v_long), params_.v_eps);
      }
    }

    // Aerodynamic drag and rolling resistance, both opposing travel, and both
    // acting at the CoG rather than per wheel. Same smoothing as L1: a
    // discontinuity at zero speed makes RK4's four evaluations disagree about
    // which side of it they are on.
    const double f_drag = params_.drag_coeff * vx * std::fabs(vx);
    const double f_roll = params_.roll_resist * params_.mass * c.g_normal *
                          std::tanh(vx / params_.v_eps);
    kin.resistance = f_drag + f_roll;

    return kin;
  }

  // The friction budget of all four tyres at one set of vertical loads. The
  // only part of a force evaluation that has to be redone when the loads
  // change, and the only part that costs a power.
  void budget_at(const StepConstants& c, const WheelLoads& loads,
                 std::array<TyreBudget, kWheelCount>* out) const {
    for (unsigned i = 0; i < kWheelCount; ++i) {
      const bool front = (i == kFrontLeft || i == kFrontRight);
      const MfLite& tyre = front ? c.tyre_front : c.tyre_rear;
      const NominalPeak& peak = front ? c.peak_front : c.peak_rear;
      const double fz = loads.fz[i];

      // One power for both ellipse axes: the load-only half of the peak force
      // law is shared and the tyre-only half is a per-step constant. See the
      // split in tyre.hpp, which exists for this call site.
      const double load = load_factor(tyre, fz);
      (*out)[i].fx_max = peak.x * load * c.mu[i];
      (*out)[i].fy_max = peak.y * load * c.mu[i];

      // Slip stiffness at this load, entering the same way it does for
      // cornering stiffness: the stiffness scales as the peak force does.
      const double c_kappa_nom = front ? c.c_kappa_front : c.c_kappa_rear;
      const double fz_nom = front ? c.fz_nom_front : c.fz_nom_rear;
      (*out)[i].c_kappa = c_kappa_nom * std::pow(fz / fz_nom, 1.0 - tyre.k_mu);
    }
  }

  // One evaluation of the tyre forces at a given set of vertical loads. Called
  // twice per derivative: see the note on the algebraic loop above.
  void evaluate_forces(const StepConstants& c, const Kinematics& kin,
                       const WheelLoads& loads,
                       const std::array<TyreBudget, kWheelCount>& budget,
                       Forces* f) const {

    // Phase one: everything about a wheel that does not depend on how the
    // differential splits the longitudinal demand.
    std::array<WheelPre, kWheelCount> pre{};
    for (unsigned i = 0; i < kWheelCount; ++i) {
      pre[i].fy_pure = -budget[i].fy_max * kin.wheel[i].shape;
      pre[i].fx_max = budget[i].fx_max;
      pre[i].fy_max = budget[i].fy_max;
      pre[i].c_kappa = budget[i].c_kappa;
      pre[i].vx_safe = kin.wheel[i].vx_safe;

      f->alpha[i] = kin.wheel[i].alpha;
      f->fz[i] = loads.fz[i];
      f->vx_wheel[i] = kin.wheel[i].vxw;
    }

    // Phase two: the drivetrain. Each driven axle splits its share of the
    // demand through the differential; an undriven axle freewheels, which is
    // a zero longitudinal demand through the same ellipse.
    //
    // With wheel dynamics (ADR 0006) the wheels are state: each tyre's
    // longitudinal force follows from its own slip ratio at the held wheel
    // speed, and the differential acts on torque in the wheel update instead.
    std::array<CombinedForce, kWheelCount> delivered{};
    if (c.wheels) {
      for (unsigned i = 0; i < kWheelCount; ++i) {
        const double kappa = c.slip_v[i] / kin.wheel[i].v_abs;
        delivered[i] = friction_ellipse(budget[i].c_kappa * kappa,
                                        pre[i].fy_pure, pre[i].fx_max,
                                        pre[i].fy_max);
        f->kappa[i] = kappa;
        f->v_long[i] = kin.wheel[i].v_long;
        f->v_abs[i] = kin.wheel[i].v_abs;
        f->c_kappa[i] = budget[i].c_kappa;
        f->fy_pure[i] = pre[i].fy_pure;
        f->fx_max[i] = pre[i].fx_max;
        f->fy_max[i] = pre[i].fy_max;
      }
    } else if (c.front_share != 0.0) {
      split_axle(kFrontLeft, kFrontRight, c.front_share * c.fx_demand, pre,
                 &delivered);
    } else {
      delivered[kFrontLeft] = friction_ellipse(
          0.0, pre[kFrontLeft].fy_pure, pre[kFrontLeft].fx_max,
          pre[kFrontLeft].fy_max);
      delivered[kFrontRight] = friction_ellipse(
          0.0, pre[kFrontRight].fy_pure, pre[kFrontRight].fx_max,
          pre[kFrontRight].fy_max);
    }
    if (c.wheels) {
      // Done above, all four wheels.
    } else if (c.rear_share != 0.0) {
      split_axle(kRearLeft, kRearRight, c.rear_share * c.fx_demand, pre,
                 &delivered);
    } else {
      delivered[kRearLeft] = friction_ellipse(
          0.0, pre[kRearLeft].fy_pure, pre[kRearLeft].fx_max,
          pre[kRearLeft].fy_max);
      delivered[kRearRight] = friction_ellipse(
          0.0, pre[kRearRight].fy_pure, pre[kRearRight].fx_max,
          pre[kRearRight].fy_max);
    }

    // Phase three: body frame, moments and the slip-ratio report.
    //
    // Per-wheel contributions, reduced below in a fixed, mirror-symmetric
    // order. Accumulating them one at a time in wheel order looks equivalent
    // and is not: floating-point addition is commutative but not associative,
    // so ((FL + FR) + RL) + RR and its left/right mirror differ in the last
    // bits, and the mirror-symmetry invariant fails by about 1e-5 relative.
    // Pairing each axle first makes the reduction exactly symmetric, because
    // each pair sum is a two-term sum and those do commute exactly (CORE-04,
    // ADR-0004: fixed reduction order in a numerical path).
    std::array<double, kWheelCount> fx_b{};
    std::array<double, kWheelCount> fy_b{};
    std::array<double, kWheelCount> mz_w{};

    for (unsigned i = 0; i < kWheelCount; ++i) {
      const double xw = c.wheel_x[i];
      const double yw = c.wheel_y[i];
      const bool front = (i == kFrontLeft || i == kFrontRight);

      // Wheel frame to body frame. Only the front wheels are rotated.
      const double cos_i = front ? kin.cos_w[i] : 1.0;
      const double sin_i = front ? kin.sin_w[i] : 0.0;
      fx_b[i] = delivered[i].fx * cos_i - delivered[i].fy * sin_i;
      fy_b[i] = delivered[i].fx * sin_i + delivered[i].fy * cos_i;

      // Yaw moment about the CoG. The second term is why a double-track model
      // yaws under asymmetric braking and a bicycle model cannot.
      mz_w[i] = xw * fy_b[i] - yw * fx_b[i];

      f->fx[i] = delivered[i].fx;
      f->fy[i] = delivered[i].fy;
      f->saturated[i] = delivered[i].saturated;

      // The slip ratio consistent with the force that was actually delivered,
      // through the linear slip stiffness at this load. Reported, not
      // integrated (ADR-0027). An undriven wheel delivers no longitudinal
      // force and freewheels at exactly zero slip. With wheel dynamics the
      // slip ratio is the wheel's own, set above.
      if (!c.wheels) {
        f->kappa[i] = (pre[i].c_kappa > 0.0)
                          ? (delivered[i].fx / pre[i].c_kappa)
                          : 0.0;
      }
    }

    // Axle first, then the two axles. See the note above the arrays.
    double fx_body = (fx_b[kFrontLeft] + fx_b[kFrontRight]) +
                     (fx_b[kRearLeft] + fx_b[kRearRight]);
    const double fy_body = (fy_b[kFrontLeft] + fy_b[kFrontRight]) +
                           (fy_b[kRearLeft] + fy_b[kRearRight]);
    const double mz = (mz_w[kFrontLeft] + mz_w[kFrontRight]) +
                      (mz_w[kRearLeft] + mz_w[kRearRight]);

    // Drag and rolling resistance, computed once per derivative rather than
    // once per load pass: neither depends on a vertical load.
    fx_body -= kin.resistance;

    f->ax = fx_body / params_.mass;
    f->ay = fy_body / params_.mass;
    f->mz = mz;
    f->transfer_long = loads.transfer_long;
    f->transfer_lat = loads.transfer_lat;
  }

  StateVec<kN> derivative(const StateVec<kN>& q, const StepConstants& c,
                          Forces* f) const {
    // What both passes agree about, worked out once.
    const Kinematics kin = kinematics(q, c);

    if (c.susp) {
      // The springs' loads, held for the step (ADR 0008): one pass.
      evaluate_forces(c, kin, c.susp_loads, c.susp_budget, f);
    } else {
      // Pass one: static loads, to get an acceleration to transfer load with.
      // The budget at those loads is a per-step constant, so it arrives ready.
      Forces first;
      evaluate_forces(c, kin, c.statics, c.static_budget, &first);

      // Pass two: the loads those accelerations imply, and the forces they
      // give. Two passes, always, whatever the residual (ADR-0027).
      const WheelLoads loads =
          quasi_static_loads(params_, first.ax, first.ay, c.g_normal);
      std::array<TyreBudget, kWheelCount> budget;
      budget_at(c, loads, &budget);
      evaluate_forces(c, kin, loads, budget, f);
    }

    StateVec<kN> d{};
    d[kVx] = f->ax + q[kVy] * q[kR];
    d[kVy] = f->ay - q[kVx] * q[kR];
    d[kR] = f->mz / params_.izz;
    if (c.graded) {
      double gx = 0.0;
      double gy = 0.0;
      grade_accel(c, q[kYaw], &gx, &gy);
      d[kVx] += gx;
      d[kVy] += gy;
    }

    for (unsigned i = 0; i < kWheelCount; ++i) {
      const bool front = (i == kFrontLeft || i == kFrontRight);
      const double sigma = front ? params_.tyre_front.relax_length
                                 : params_.tyre_rear.relax_length;
      d[kLag0 + i] = relaxation_rate(f->alpha[i], q[kLag0 + i],
                                     f->vx_wheel[i], sigma);
    }

    // The servo: a slew-limited second-order lag on the achieved road wheel
    // angle (ADR-0031). The rate state can wind past the slew limit; the
    // angle only ever moves at the clamped rate.
    const double wn = params_.steer_bandwidth;
    d[kSteer] = clamp(q[kSteerRate], -params_.steer_rate_max,
                      params_.steer_rate_max);
    d[kSteerRate] = wn * wn * (c.delta_cmd - q[kSteer]) -
                    2.0 * params_.steer_damping * wn * q[kSteerRate];

    // Per-step constant: the electrical loop closed once at the entry state.
    d[kSoc] = c.soc_rate;

    const double cos_yaw = std::cos(q[kYaw]);
    const double sin_yaw = std::sin(q[kYaw]);
    d[kX] = q[kVx] * cos_yaw - q[kVy] * sin_yaw;
    d[kY] = q[kVx] * sin_yaw + q[kVy] * cos_yaw;
    d[kYaw] = q[kR];

    return d;
  }

  VehicleParams params_;
  Integrator integrator_;
};

}  // namespace

std::unique_ptr<VehicleModel> make_l2(const VehicleParams& p, Integrator i) {
  return std::unique_ptr<VehicleModel>(new DoubleTrackL2(p, i));
}

}  // namespace internal
}  // namespace slipx
