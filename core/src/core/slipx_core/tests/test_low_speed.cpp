// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
// Added 2026-10-25 by the rc-physics-twin project: see THIRD_PARTY.md.
//
// The kinematic blend at low speed (params.v_kinematic, rc-physics-twin
// ADR 0011). Below v_eps the slip denominators are floored, so without the
// blend the car turns at full lock as if it were doing v_eps: its yaw rate
// is v_eps tan(delta) / L whatever its speed. With the blend it follows the
// kinematic single-track turn, r = vx tan(delta) / L, and stops turning when
// it stops.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstring>

#include "slipx/ground.hpp"
#include "slipx/math.hpp"
#include "slipx/vehicle_model.hpp"
#include "test_support.hpp"

namespace {

using slipx::DriveInput;
using slipx::Integrator;
using slipx::Tier;
using slipx::VehicleModel;
using slipx::VehicleParams;
using slipx::VehicleState;

constexpr double kDt = 1.0e-3;

// The reference car, optionally as the Unity twin drives it: wheel inertia
// (ADR 0006), ideal Ackermann (ADR 0007) and a suspension (ADR 0008) at the
// twin's 3 Hz ride frequency and 0.3 damping ratio.
VehicleParams car(double v_kinematic, bool twin) {
  VehicleParams p = slipx_test::reference_params();
  p.v_kinematic = v_kinematic;
  if (twin) {
    p.wheel_inertia = 1e-5;
    p.ackermann = 1.0;
    const double w = 2.0 * slipx::kPi * 3.0;
    const double k = p.mass * w * w / 4.0;
    p.wheel_rate_front = p.wheel_rate_rear = k;
    p.damping_front = p.damping_rear =
        2.0 * 0.3 * std::sqrt(4.0 * k * p.mass) / 4.0;
    p.suspension_travel = 0.015;
  }
  return p;
}

struct Circle {
  double vx = 0.0;      // mean over the second half              [m/s]
  double r = 0.0;       // mean yaw rate                          [rad/s]
  double r_sd = 0.0;    // its standard deviation                 [rad/s]
  double vy = 0.0;      // mean lateral velocity                  [m/s]
  double delta = 0.0;   // achieved reference steer at the end    [rad]
};

// Full lock at a held speed, forwards or (target < 0) in reverse, from rest.
Circle full_lock(const VehicleParams& p, double target, double seconds = 6.0) {
  auto m = VehicleModel::create(Tier::L2_DoubleTrack, p, Integrator::kRK4);
  VehicleState s = slipx_test::at_rest();
  const int n = static_cast<int>(std::lround(seconds / kDt));
  double sum_vx = 0.0, sum_r = 0.0, sum_r2 = 0.0, sum_vy = 0.0;
  int k_n = 0;
  for (int k = 0; k < n; ++k) {
    DriveInput u{p.steer_max, 3.0 * (target - s.vel_body.x)};
    u.reverse = target < 0.0;
    m->step(s, u, kDt);
    if (k >= n / 2) {
      sum_vx += s.vel_body.x;
      sum_r += s.rates.z;
      sum_r2 += s.rates.z * s.rates.z;
      sum_vy += s.vel_body.y;
      ++k_n;
    }
  }
  Circle c;
  c.vx = sum_vx / k_n;
  c.r = sum_r / k_n;
  c.r_sd = std::sqrt(std::fmax(sum_r2 / k_n - c.r * c.r, 0.0));
  c.vy = sum_vy / k_n;
  c.delta = s.steer;
  return c;
}

TEST(LowSpeed, TheKinematicSpeedMustBeBelowVEps) {
  VehicleParams p = slipx_test::reference_params();
  for (double bad : {-0.1, p.v_eps, 2.0 * p.v_eps, std::nan(""),
                     std::numeric_limits<double>::infinity()}) {
    p.v_kinematic = bad;
    EXPECT_NE(slipx::validate(p), nullptr) << bad;
  }
  for (double ok : {0.0, 0.1, 0.6 * p.v_eps}) {
    p.v_kinematic = ok;
    EXPECT_EQ(slipx::validate(p), nullptr) << ok;
  }
}

TEST(LowSpeed, WithoutTheBlendTheFloorSetsTheYawRate) {
  // The defect, as SlipX's L2 has it: at 0.1 and 0.2 m/s the car turns at
  // the same rate, the one it would have at v_eps, several times the
  // kinematic one.
  const VehicleParams p = car(0.0, false);
  const Circle slow = full_lock(p, 0.1);
  const Circle less_slow = full_lock(p, 0.2);
  const double r_at_veps = p.v_eps * std::tan(slow.delta) / p.wheelbase();
  EXPECT_NEAR(less_slow.r, r_at_veps, 0.1 * r_at_veps);
  EXPECT_GT(less_slow.r, 2.0 * less_slow.vx * std::tan(less_slow.delta) /
                             p.wheelbase());
}

TEST(LowSpeed, CreepingAtFullLockFollowsTheKinematicTurn) {
  // The reference car, the twin, and one with its CoG off the middle, where
  // the lateral velocity tells lr from lf.
  for (int variant : {0, 1, 2}) {
    const bool twin = variant == 1;
    VehicleParams p = car(0.3, twin);
    if (variant == 2) {
      p.lf = 0.12;
      p.lr = 0.20;
    }
    for (double v : {0.1, 0.2, -0.1, -0.2}) {
      const Circle c = full_lock(p, v);
      const double r_kin = c.vx * std::tan(c.delta) / p.wheelbase();
      EXPECT_NEAR(c.r, r_kin, 1e-3 * std::fabs(r_kin)) << variant << " " << v;
      // The CoG of a kinematic single-track car moves sideways at lr r.
      EXPECT_NEAR(c.vy, p.lr * c.r, 1e-3 * std::fabs(p.lr * c.r))
          << variant << " " << v;
      EXPECT_LT(c.r_sd, 1e-3 * std::fabs(c.r)) << variant << " " << v;
    }
  }
}

TEST(LowSpeed, SteeringWhileCreepingKeepsToTheKinematicTurn) {
  // The servo sweeps the wheels from lock to lock at 0.2 m/s: the yaw rate
  // follows vx tan(delta) / L as delta moves, not a step behind it, and the
  // CoG's lateral velocity lr times that (a CoG off the middle tells lr from
  // lf).
  for (bool twin : {false, true}) {
    VehicleParams p = car(0.3, twin);
    p.lf = 0.12;
    p.lr = 0.20;
    auto m = VehicleModel::create(Tier::L2_DoubleTrack, p, Integrator::kRK4);
    VehicleState s = slipx_test::at_rest();
    for (int k = 0; k < 3000; ++k) {
      m->step(s, DriveInput{-p.steer_max, 3.0 * (0.2 - s.vel_body.x)}, kDt);
    }
    double worst = 0.0;
    double worst_vy = 0.0;
    for (int k = 0; k < 1500; ++k) {
      m->step(s, DriveInput{p.steer_max, 3.0 * (0.2 - s.vel_body.x)}, kDt);
      const double r_kin = s.vel_body.x * std::tan(s.steer) / p.wheelbase();
      worst = std::fmax(worst, std::fabs(s.rates.z - r_kin));
      worst_vy = std::fmax(worst_vy, std::fabs(s.vel_body.y - p.lr * r_kin));
    }
    EXPECT_GT(s.steer, 0.9 * p.steer_max) << twin;
    EXPECT_LT(worst, 2e-3) << twin;
    EXPECT_LT(worst_vy, 2e-3 * p.lr) << twin;
  }
}

// The rates of vx, vy and r at a state, from one very short step.
std::array<double, 3> rates_at(const VehicleParams& p, const VehicleState& s0) {
  auto m = VehicleModel::create(Tier::L2_DoubleTrack, p, Integrator::kRK4);
  VehicleState s = s0;
  const double dt = 1e-7;
  m->step(s, DriveInput{s0.steer, 1.0}, dt);
  return {(s.vel_body.x - s0.vel_body.x) / dt,
          (s.vel_body.y - s0.vel_body.y) / dt, (s.rates.z - s0.rates.z) / dt};
}

TEST(LowSpeed, InTheWindowTheDerivativesBlendLinearly) {
  // At 0.45 m/s with v_kinematic 0.3 the dynamic model's weight is 0.75.
  // v_kinematic 0 gives the dynamic rates at the same state, and 0.49 the
  // kinematic ones (0.45 is below it). The state is off the kinematic turn,
  // so the two differ.
  for (bool twin : {false, true}) {
    VehicleState s;
    s.vel_body.x = 0.45;
    s.vel_body.y = 0.02;
    s.rates.z = 0.3;
    s.steer = 0.3;
    const double R = car(0.0, twin).wheel_radius;
    for (auto& w : s.omega_w) w = 0.45 / R;
    const auto dyn = rates_at(car(0.0, twin), s);
    const auto kin = rates_at(car(0.49, twin), s);
    const auto mix = rates_at(car(0.3, twin), s);
    for (int j = 0; j < 3; ++j) {
      const double want = 0.75 * dyn[j] + 0.25 * kin[j];
      EXPECT_NEAR(mix[j], want, 1e-5 * (std::fabs(dyn[j]) + std::fabs(kin[j])))
          << twin << " " << j;
    }
    EXPECT_GT(std::fabs(dyn[2] - kin[2]), 0.1) << twin;
  }
}

TEST(LowSpeed, AKinematicCarsSteeredWheelsDriveWithTheirWholeGrip) {
  // An all-wheel-drive twin creeping at full lock, then pulling away hard.
  // Once it has rolled a few relaxation lengths, the floored slip angles ask
  // the front tyres for a lateral force a creeping car does not need; left in
  // their friction ellipses it would leave them too little grip to drive
  // with, and the front wheels would spin up.
  VehicleParams p = car(0.3, true);
  p.layout = slipx::DriveLayout::kAllWheelDrive;
  auto m = VehicleModel::create(Tier::L2_DoubleTrack, p, Integrator::kRK4);
  VehicleState s = slipx_test::at_rest();
  for (int k = 0; k < 4000; ++k) {
    m->step(s, DriveInput{p.steer_max, 3.0 * (0.1 - s.vel_body.x)}, kDt);
  }
  double worst = 0.0;
  for (int k = 0; k < 30; ++k) {
    m->step(s, DriveInput{p.steer_max, 6.0}, kDt);
    const double vxw = s.vel_body.x - s.rates.z * 0.5 * p.track_front;
    worst = std::fmax(worst, std::fabs(s.omega_w[slipx::kFrontLeft] *
                                           p.wheel_radius - vxw));
  }
  EXPECT_LT(s.vel_body.x, p.v_kinematic);
  EXPECT_LT(worst, 0.1);
}

TEST(LowSpeed, AKinematicCarRollsDownhill) {
  // On a 5 degree grade with no demand, a car let go from rest rolls away
  // at about g sin(theta) less its rolling resistance.
  const double theta = 5.0 * slipx::kPi / 180.0;
  slipx::GroundContact g;
  for (auto& w : g.wheel) {
    w.normal = slipx::Vec3{std::sin(theta), 0.0, std::cos(theta)};
  }
  const VehicleParams p = car(0.3, false);
  auto m = VehicleModel::create(Tier::L2_DoubleTrack, p, Integrator::kRK4);
  VehicleState s = slipx_test::at_rest();
  for (int k = 0; k < 250; ++k) ASSERT_TRUE(m->step_on(s, DriveInput{}, g, kDt));
  const double expect = slipx::kGravity * std::sin(theta) * 0.25;
  EXPECT_LT(s.vel_body.x, p.v_kinematic);
  EXPECT_NEAR(s.vel_body.x, expect, 0.1 * expect);
}

TEST(LowSpeed, ReversingThroughTheBlendIsSteady) {
  // Reversing slowly at full lock was erratic: the yaw rate wandered by
  // more than its own mean. In the blend window it must hold steady.
  const VehicleParams p = car(0.3, true);
  for (double v : {-0.3, -0.4}) {
    const Circle c = full_lock(p, v);
    EXPECT_LT(c.vx, 0.0) << v;
    EXPECT_LT(c.r_sd, 0.02 * std::fabs(c.r)) << v;
  }
}

TEST(LowSpeed, AStoppedCarStopsTurning) {
  // Brake to rest at full lock from above v_eps, holding zero speed (without
  // wheel dynamics a held negative demand would reverse): once stopped, the
  // car neither creeps round nor keeps a yaw rate.
  for (bool twin : {false, true}) {
    const VehicleParams p = car(0.3, twin);
    auto m = VehicleModel::create(Tier::L2_DoubleTrack, p, Integrator::kRK4);
    VehicleState s = slipx_test::travelling(1.0);
    for (auto& w : s.omega_w) w = 1.0 / p.wheel_radius;
    for (int k = 0; k < 1500; ++k) {
      m->step(s, DriveInput{p.steer_max, 3.0 * (1.0 - s.vel_body.x)}, kDt);
    }
    const auto hold_zero = [&] {
      m->step(s, DriveInput{p.steer_max, -4.0 * s.vel_body.x}, kDt);
    };
    for (int k = 0; k < 3000; ++k) hold_zero();
    const double yaw = s.yaw;
    for (int k = 0; k < 1000; ++k) hold_zero();
    EXPECT_LT(std::fabs(s.vel_body.x), 1e-3) << twin;
    EXPECT_LT(std::fabs(s.rates.z), 1e-3) << twin;
    EXPECT_LT(std::fabs(s.yaw - yaw), 1e-3) << twin;
  }
}

TEST(LowSpeed, AboveVEpsTheBlendChangesNothing) {
  // From v_eps up the dynamic model is untouched, bit for bit, forwards and
  // backwards.
  for (bool twin : {false, true}) {
    auto off = VehicleModel::create(Tier::L2_DoubleTrack, car(0.0, twin));
    auto on = VehicleModel::create(Tier::L2_DoubleTrack, car(0.3, twin));
    VehicleState a = slipx_test::travelling(2.0);
    for (auto& w : a.omega_w) w = 2.0 / car(0.0, twin).wheel_radius;
    VehicleState b = a;
    for (int k = 0; k < 2000; ++k) {
      const DriveInput u{k < 1000 ? 0.3 : -0.25, 3.0 * (2.0 - a.vel_body.x)};
      off->step(a, u, kDt);
      on->step(b, u, kDt);
    }
    EXPECT_GT(a.vel_body.x, car(0.0, twin).v_eps);
    EXPECT_EQ(std::memcmp(&a, &b, sizeof a), 0) << twin;
  }
  auto off = VehicleModel::create(Tier::L2_DoubleTrack, car(0.0, false));
  auto on = VehicleModel::create(Tier::L2_DoubleTrack, car(0.3, false));
  VehicleState a = slipx_test::travelling(-1.0);
  VehicleState b = a;
  for (int k = 0; k < 2000; ++k) {
    const DriveInput u{0.3, 3.0 * (-1.0 - a.vel_body.x)};
    off->step(a, u, kDt);
    on->step(b, u, kDt);
  }
  EXPECT_LT(a.vel_body.x, -car(0.0, false).v_eps);
  EXPECT_EQ(std::memcmp(&a, &b, sizeof a), 0);
}

TEST(LowSpeed, ALeftCreepMirrorsARightCreepBitForBit) {
  for (bool twin : {false, true}) {
    const VehicleParams p = car(0.3, twin);
    auto m = VehicleModel::create(Tier::L2_DoubleTrack, p, Integrator::kRK4);
    VehicleState a = slipx_test::at_rest();
    VehicleState b = a;
    for (int k = 0; k < 4000; ++k) {
      const double steer = k < 2000 ? p.steer_max : -0.2;
      const double accel = k < 3000 ? 0.4 : -1.0;
      m->step(a, DriveInput{steer, accel}, kDt);
      m->step(b, DriveInput{-steer, accel}, kDt);
    }
    EXPECT_EQ(a.vel_body.x, b.vel_body.x) << twin;
    EXPECT_EQ(a.vel_body.y, -b.vel_body.y) << twin;
    EXPECT_EQ(a.rates.z, -b.rates.z) << twin;
    EXPECT_EQ(a.pos.y, -b.pos.y) << twin;
    EXPECT_EQ(a.yaw, -b.yaw) << twin;
  }
}

TEST(LowSpeed, NoEffectBelowL2) {
  for (Tier t : {Tier::L0_Kinematic, Tier::L1_Bicycle}) {
    auto off = VehicleModel::create(t, car(0.0, false));
    auto on = VehicleModel::create(t, car(0.3, false));
    VehicleState a = slipx_test::travelling(0.2);
    VehicleState b = a;
    for (int k = 0; k < 1500; ++k) {
      const DriveInput u{0.38, k < 700 ? 0.3 : -0.2};
      off->step(a, u, kDt);
      on->step(b, u, kDt);
    }
    EXPECT_EQ(std::memcmp(&a, &b, sizeof a), 0) << slipx::to_string(t);
  }
}

}  // namespace
