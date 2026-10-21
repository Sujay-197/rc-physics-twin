// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
// Added 2026-10-19 by the rc-physics-twin project: see THIRD_PARTY.md.
//
// Ackermann steering at L2 (params.ackermann, rc-physics-twin ADR 0007).
// The geometry is checked where it is exact: a car in pure rotation about a
// turn centre on the rear-axle line, with the reference angle pointing at
// that centre. Ideal Ackermann then leaves both front tyres at zero slip;
// parallel steer leaves them opposed, which is scrub.

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>

#include "slipx/vehicle_model.hpp"
#include "test_support.hpp"

namespace {

using slipx::DriveInput;
using slipx::Integrator;
using slipx::StepDiagnostics;
using slipx::Tier;
using slipx::VehicleModel;
using slipx::VehicleParams;
using slipx::VehicleState;
using slipx::kFrontLeft;
using slipx::kFrontRight;
using slipx::kRearLeft;
using slipx::kRearRight;

VehicleParams with_ackermann(double a) {
  VehicleParams p = slipx_test::reference_params();
  p.ackermann = a;
  return p;
}

// Rotating at yaw rate r about (-lr, R) in the body frame: every point moves
// as r z x (p - c). The reference angle is the centre wheel's: tan = L / R.
VehicleState pure_rotation(const VehicleParams& p, double radius, double r) {
  VehicleState s;
  s.vel_body.x = r * radius;
  s.vel_body.y = r * p.lr;
  s.rates.z = r;
  s.steer = std::atan(p.wheelbase() / radius);
  return s;
}

// The geometric slip angles of that state: one very short step, so the
// state is effectively unchanged, with the command holding the steer.
StepDiagnostics slips(const VehicleParams& p, double radius) {
  auto m = VehicleModel::create(Tier::L2_DoubleTrack, p);
  // A right turn (negative radius) turns the other way: yaw rate negated too.
  VehicleState s = pure_rotation(p, radius, radius > 0.0 ? 2.0 : -2.0);
  StepDiagnostics d;
  m->step(s, DriveInput{s.steer, 0.0}, 1e-7, &d);
  return d;
}

TEST(Ackermann, TheFractionMustBeBetweenAntiAndIdeal) {
  VehicleParams p = slipx_test::reference_params();
  for (double bad : {-1.5, 1.01, std::nan("")}) {
    p.ackermann = bad;
    EXPECT_NE(slipx::validate(p), nullptr) << bad;
  }
  for (double ok : {-1.0, 0.0, 0.5, 1.0}) {
    p.ackermann = ok;
    EXPECT_EQ(slipx::validate(p), nullptr) << ok;
  }
}

TEST(AckermannGeometry, IdealAckermannPointsBothFrontWheelsAtTheTurnCentre) {
  const StepDiagnostics d = slips(with_ackermann(1.0), 0.8);
  EXPECT_NEAR(d.alpha[kFrontLeft], 0.0, 1e-6);
  EXPECT_NEAR(d.alpha[kFrontRight], 0.0, 1e-6);
  EXPECT_NEAR(d.alpha[kRearLeft], 0.0, 1e-6);
  EXPECT_NEAR(d.alpha[kRearRight], 0.0, 1e-6);
}

TEST(AckermannGeometry, ParallelSteerScrubsTheFrontTyresAgainstEachOther) {
  const StepDiagnostics d = slips(with_ackermann(0.0), 0.8);
  // The inner (left) wheel is under-steered for its path, the outer
  // over-steered: slip angles of opposite sign.
  EXPECT_GT(d.alpha[kFrontLeft], 1e-3);
  EXPECT_LT(d.alpha[kFrontRight], -1e-3);
}

TEST(AckermannGeometry, TheFractionBlendsTheAngleLinearly) {
  const StepDiagnostics zero = slips(with_ackermann(0.0), 0.8);
  const StepDiagnostics half = slips(with_ackermann(0.5), 0.8);
  const StepDiagnostics anti = slips(with_ackermann(-1.0), 0.8);
  for (unsigned i : {kFrontLeft, kFrontRight}) {
    EXPECT_NEAR(half.alpha[i], 0.5 * zero.alpha[i], 1e-6) << i;
    EXPECT_NEAR(anti.alpha[i], 2.0 * zero.alpha[i], 1e-6) << i;
  }
}

TEST(AckermannGeometry, ARightTurnIsTheMirrorOfALeftTurn) {
  const VehicleParams p = with_ackermann(1.0);
  StepDiagnostics left = slips(p, 0.8);
  StepDiagnostics right = slips(p, -0.8);
  EXPECT_EQ(left.alpha[kFrontLeft], -right.alpha[kFrontRight]);
  EXPECT_EQ(left.alpha[kFrontRight], -right.alpha[kFrontLeft]);
}

// Each front wheel's angle, worked out here from the ADR 0007 formula.
double wheel_angle(const VehicleParams& p, double delta, bool left) {
  const double wb = p.wheelbase();
  const double ht = 0.5 * p.track_front;
  const double tn = std::tan(delta);
  const double ideal = std::atan(wb * tn / (left ? wb - ht * tn : wb + ht * tn));
  return delta + p.ackermann * (ideal - delta);
}

// In a turn at speed, a well-steered car. Ends with one step's diagnostics.
VehicleState cornering(const VehicleParams& p, StepDiagnostics* d) {
  auto m = VehicleModel::create(Tier::L2_DoubleTrack, p);
  VehicleState s = slipx_test::travelling(2.0);
  for (auto& w : s.omega_w) w = 2.0 / p.wheel_radius;
  for (int k = 0; k < 1500; ++k) m->step(s, DriveInput{0.35, 1.0}, 0.001, d);
  return s;
}

TEST(AckermannGeometry, EachFrontTyresForceIsTurnedByItsOwnAngle) {
  // The lateral specific force is the tyre forces summed in the body frame:
  // each front tyre's (fx, fy) turned by that wheel's own angle.
  const VehicleParams p = with_ackermann(1.0);
  StepDiagnostics d;
  const VehicleState s = cornering(p, &d);
  const double dl = wheel_angle(p, s.steer, true);
  const double dr = wheel_angle(p, s.steer, false);
  const double fy_body =
      (d.fx[kFrontLeft] * std::sin(dl) + d.fy[kFrontLeft] * std::cos(dl)) +
      (d.fx[kFrontRight] * std::sin(dr) + d.fy[kFrontRight] * std::cos(dr)) +
      (d.fy[kRearLeft] + d.fy[kRearRight]);
  EXPECT_NEAR(d.ay, fy_body / p.mass, 1e-9);
  EXPECT_GT(std::fabs(dl - dr), 0.01);  // the angles really differ here
}

TEST(AckermannGeometry, AFreeFrontWheelRollsAlongItsOwnHeading) {
  // With wheel dynamics, an undriven front wheel rolls at its centre's speed
  // along its own heading, not along the reference wheel's.
  VehicleParams p = with_ackermann(1.0);
  p.wheel_inertia = 1e-5;
  StepDiagnostics d;
  const VehicleState s = cornering(p, &d);
  for (unsigned i : {kFrontLeft, kFrontRight}) {
    const bool left = (i == kFrontLeft);
    const double yw = left ? 0.5 * p.track_front : -0.5 * p.track_front;
    const double vxw = s.vel_body.x - s.rates.z * yw;
    const double vyw = s.vel_body.y + s.rates.z * p.lf;
    const double d_i = wheel_angle(p, s.steer, left);
    const double v_long = vxw * std::cos(d_i) + vyw * std::sin(d_i);
    EXPECT_NEAR(s.omega_w[i] * p.wheel_radius, v_long, 1e-3 * v_long) << i;
  }
}

TEST(Ackermann, NoEffectBelowL2) {
  for (Tier t : {Tier::L0_Kinematic, Tier::L1_Bicycle}) {
    auto parallel = VehicleModel::create(t, with_ackermann(0.0));
    auto ideal = VehicleModel::create(t, with_ackermann(1.0));
    VehicleState a = slipx_test::travelling(1.0);
    VehicleState b = a;
    for (int k = 0; k < 1500; ++k) {
      const DriveInput u{k < 700 ? 0.3 : -0.2, 2.0};
      parallel->step(a, u, 0.001);
      ideal->step(b, u, 0.001);
    }
    EXPECT_EQ(std::memcmp(&a, &b, sizeof a), 0) << slipx::to_string(t);
  }
}

TEST(AckermannDynamics, ParallelSteerDragsInASlowTightTurn) {
  // Scrub costs speed: the same throttle in the same tight turn leaves the
  // parallel-steer car slower than the ideal one.
  double speed[2] = {0.0, 0.0};
  int n = 0;
  for (double a : {0.0, 1.0}) {
    auto m = VehicleModel::create(Tier::L2_DoubleTrack, with_ackermann(a));
    VehicleState s = slipx_test::travelling(1.0);
    for (int k = 0; k < 2000; ++k) m->step(s, DriveInput{0.38, 0.6}, 0.001);
    speed[n++] = s.vel_body.x;
  }
  EXPECT_LT(speed[0], speed[1]);
}

TEST(AckermannInvariant, ALeftDriveMirrorsARightDriveBitForBit) {
  for (double wheel_inertia : {0.0, 1e-4}) {
    VehicleParams p = with_ackermann(1.0);
    p.wheel_inertia = wheel_inertia;
    auto m = VehicleModel::create(Tier::L2_DoubleTrack, p, Integrator::kRK4);
    VehicleState a = slipx_test::travelling(1.0);
    for (auto& w : a.omega_w) w = 1.0 / p.wheel_radius;
    VehicleState b = a;
    for (int k = 0; k < 2500; ++k) {
      const double steer = k < 1200 ? 0.35 : -0.2;
      const double accel = k < 2000 ? 2.0 : -4.0;
      m->step(a, DriveInput{steer, accel}, 0.001);
      m->step(b, DriveInput{-steer, accel}, 0.001);
    }
    EXPECT_EQ(a.pos.y, -b.pos.y) << wheel_inertia;
    EXPECT_EQ(a.yaw, -b.yaw) << wheel_inertia;
    EXPECT_EQ(a.vel_body.x, b.vel_body.x) << wheel_inertia;
    EXPECT_EQ(a.omega_w[kFrontLeft], b.omega_w[kFrontRight]) << wheel_inertia;
  }
}

}  // namespace
