// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
// Added 2026-10-19 by the rc-physics-twin project: see THIRD_PARTY.md.
//
// Wheel rotational dynamics at L2 (params.wheel_inertia > 0, rc-physics-twin
// ADR 0006): stable at 1 kHz for any inertia, the closed-form coast-down
// with rotating wheels, the quasi-static limit, wheelspin, lock-up, the three
// differentials, mirror symmetry and the ESC seeing the real wheel speed.

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <limits>

#include "slipx/ground.hpp"
#include "slipx/vehicle_model.hpp"
#include "test_support.hpp"

namespace {

using slipx::Differential;
using slipx::DriveInput;
using slipx::GroundContact;
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

constexpr double kDt = 0.001;
constexpr double kInertia = 1e-4;  // stiff at 1 kHz: tau ~ 0.2 to 0.3 ms

VehicleParams with_wheels(double inertia = kInertia,
                          Differential diff = Differential::kOpen) {
  VehicleParams p = slipx_test::reference_params();
  p.wheel_inertia = inertia;
  p.differential = diff;
  return p;
}

std::unique_ptr<VehicleModel> l2(const VehicleParams& p,
                                 Integrator i = Integrator::kRK4) {
  return VehicleModel::create(Tier::L2_DoubleTrack, p, i);
}

// Travelling at v with every wheel rolling freely.
VehicleState rolling(const VehicleParams& p, double v) {
  VehicleState s = slipx_test::travelling(v);
  for (auto& w : s.omega_w) w = v / p.wheel_radius;
  return s;
}

GroundContact grip(double fl, double fr, double rl, double rr) {
  GroundContact g;
  g.wheel[kFrontLeft].mu_scale = fl;
  g.wheel[kFrontRight].mu_scale = fr;
  g.wheel[kRearLeft].mu_scale = rl;
  g.wheel[kRearRight].mu_scale = rr;
  return g;
}

GroundContact grip(double mu) { return grip(mu, mu, mu, mu); }

StepDiagnostics run(const VehicleModel& m, VehicleState& s, DriveInput u,
                    int n, const GroundContact& g = GroundContact{}) {
  StepDiagnostics d;
  for (int k = 0; k < n; ++k) {
    if (!m.step_on(s, u, g, kDt, &d)) {
      ADD_FAILURE() << "refused at step " << k;
      break;
    }
  }
  return d;
}

double kappa(const VehicleParams& p, const VehicleState& s, unsigned i) {
  // Rear wheels and a straight car: the wheel heading is body x.
  const double v = s.vel_body.x;
  return (s.omega_w[i] * p.wheel_radius - v) / std::fmax(std::fabs(v), p.v_eps);
}

DriveInput scripted(int k) {
  if (k < 800) return DriveInput{0.0, 4.0};
  if (k < 1600) return DriveInput{0.25, 1.0};
  if (k < 2200) return DriveInput{-0.3, 0.0};
  return DriveInput{0.05, -6.0};
}

// ------------------------------------------------------------- parameter

TEST(Wheels, InertiaMustBeFiniteAndNotNegative) {
  VehicleParams p = slipx_test::reference_params();
  p.wheel_inertia = -1e-4;
  EXPECT_NE(slipx::validate(p), nullptr);
  p.wheel_inertia = std::numeric_limits<double>::infinity();
  EXPECT_NE(slipx::validate(p), nullptr);
  p.wheel_inertia = 0.0;
  EXPECT_EQ(slipx::validate(p), nullptr);
}

// ------------------------------------------------------------- numerics

TEST(Wheels, StableAtOneKilohertzForAnyInertia) {
  for (double inertia : {1e-8, 1e-6, 1e-5, 1e-4, 1e-3, 1e-2}) {
    for (Integrator i : {Integrator::kRK4, Integrator::kSemiImplicitEuler}) {
      const VehicleParams p = with_wheels(inertia);
      auto m = l2(p, i);
      VehicleState s = rolling(p, 0.5);
      for (int k = 0; k < 3000; ++k) {
        ASSERT_TRUE(m->step_on(s, scripted(k), grip(k < 1500 ? 1.0 : 0.3), kDt));
      }
      for (double w : s.omega_w) {
        EXPECT_TRUE(std::isfinite(w)) << inertia;
        EXPECT_LT(std::fabs(w), 2000.0) << inertia;  // ~100 m/s at the rim
      }
      EXPECT_LT(s.vel_body.x, p.v_max) << inertia;
    }
  }
}

TEST(Wheels, LockedWheelsOnAMovingCarSpinUpToRoadSpeed) {
  const VehicleParams p = with_wheels();
  auto m = l2(p);
  VehicleState s = slipx_test::travelling(3.0);  // omega_w all zero
  run(*m, s, DriveInput{0.0, 0.0}, 50);
  for (unsigned i = 0; i < 4; ++i) {
    EXPECT_NEAR(s.omega_w[i] * p.wheel_radius, s.vel_body.x, 0.01 * s.vel_body.x)
        << "wheel " << i;
  }
}

TEST(Wheels, TinyInertiaApproachesTheQuasiStaticTier) {
  // Well inside the friction ellipse the two closures must agree: a torque
  // balance on a massless wheel delivers the force the quasi-static tier
  // demands. Near the limit they legitimately differ (a fixed force demand
  // and a torque balance share combined slip differently), so this stays
  // gentle: moderate throttle, then a mild turn.
  const VehicleParams quasi = slipx_test::reference_params();
  const VehicleParams tiny = with_wheels(1e-8);
  auto mq = l2(quasi);
  auto mt = l2(tiny);
  VehicleState a = rolling(quasi, 1.0);
  VehicleState b = a;
  for (int k = 0; k < 2200; ++k) {
    const DriveInput u = k < 1000 ? DriveInput{0.0, 1.5} : DriveInput{0.08, 0.5};
    mq->step(a, u, kDt);
    ASSERT_TRUE(mt->step_on(b, u, GroundContact{}, kDt));
  }
  EXPECT_NEAR(a.pos.x, b.pos.x, 0.01);
  EXPECT_NEAR(a.pos.y, b.pos.y, 0.01);
  EXPECT_NEAR(a.vel_body.x, b.vel_body.x, 0.01);
  EXPECT_NEAR(a.yaw, b.yaw, 0.005);
}

// ----------------------------------------------------------- analytical

// Coasting with free wheels, resistance decelerates the car AND the four
// spinning wheels: m dv/dt + 4 I/R^2 dv/dt = -F_res, so the deceleration
// shrinks by m / (m + 4 I / R^2). A large inertia makes the effect plain.
TEST(WheelsAnalytical, RotatingWheelsAddEffectiveMassOnACoastDown) {
  for (Differential diff :
       {Differential::kOpen, Differential::kSpool, Differential::kLsd}) {
  SCOPED_TRACE(static_cast<int>(diff));
  const double inertia = 1e-3;
  const VehicleParams quasi = slipx_test::reference_params();
  VehicleParams heavy = with_wheels(inertia, diff);
  heavy.lsd_preload = 0.05;
  auto mq = l2(quasi);
  auto mh = l2(heavy);
  VehicleState a = rolling(quasi, 3.0);
  VehicleState b = rolling(heavy, 3.0);
  run(*mq, a, DriveInput{0.0, 0.0}, 50);  // settle the slip transient
  run(*mh, b, DriveInput{0.0, 0.0}, 50);
  const double va0 = a.vel_body.x;
  const double vb0 = b.vel_body.x;
  run(*mq, a, DriveInput{0.0, 0.0}, 500);
  run(*mh, b, DriveInput{0.0, 0.0}, 500);
  const double ratio = (vb0 - b.vel_body.x) / (va0 - a.vel_body.x);
  const double r = quasi.wheel_radius;
  const double expected = quasi.mass / (quasi.mass + 4.0 * inertia / (r * r));
  EXPECT_NEAR(ratio, expected, 0.02 * expected);
  }
}

// A locked wheel on a moving car is spun up by its tyre at no more than the
// friction limit allows: R mu Fz / I. On grip 0.1 that is slow enough to
// measure.
TEST(WheelsAnalytical, ALockedWheelSpinsUpAtTheFrictionLimit) {
  const VehicleParams p = with_wheels();
  auto m = l2(p);
  VehicleState s = slipx_test::travelling(3.0);  // all wheels locked
  run(*m, s, DriveInput{0.0, 0.0}, 10, grip(0.1));
  const double fz = s.Fz[kFrontLeft];
  const double limit = p.wheel_radius * 0.1 * p.tyre_front.mu_x0 * fz / p.wheel_inertia;
  // Load sensitivity lifts mu a little at this light load, hence the margin.
  EXPECT_NEAR(s.omega_w[kFrontLeft], limit * 0.010, 0.15 * limit * 0.010);
  EXPECT_LT(s.omega_w[kFrontLeft] * p.wheel_radius, 0.5 * s.vel_body.x);
}

// ------------------------------------------------------ spin and lock-up

TEST(Wheels, LowGripAndFullThrottleSpinTheDrivenWheels) {
  const VehicleParams p = with_wheels();
  auto m = l2(p);
  VehicleState s = rolling(p, 1.0);
  run(*m, s, DriveInput{0.0, 8.0}, 300, grip(0.2));
  EXPECT_GT(kappa(p, s, kRearLeft), 0.5);   // spinning
  EXPECT_GT(kappa(p, s, kRearRight), 0.5);
  EXPECT_LT(std::fabs(kappa(p, s, kFrontLeft)), 0.02);  // undriven, rolling

  // The quasi-static tier cannot do this: its slip stays at what the
  // delivered force implies.
  const VehicleParams q = slipx_test::reference_params();
  auto mq = l2(q);
  VehicleState sq = rolling(q, 1.0);
  run(*mq, sq, DriveInput{0.0, 8.0}, 300, grip(0.2));
  EXPECT_LT(kappa(q, sq, kRearLeft), 0.1);
}

TEST(Wheels, HardBrakingOnLowGripLocksTheDrivenWheels) {
  const VehicleParams p = with_wheels();
  auto m = l2(p);
  VehicleState s = rolling(p, 4.0);
  run(*m, s, DriveInput{0.0, -12.0}, 200, grip(0.1));
  EXPECT_EQ(s.omega_w[kRearLeft], 0.0);   // locked, exactly
  EXPECT_EQ(s.omega_w[kRearRight], 0.0);
  EXPECT_GT(s.vel_body.x, 2.0);           // still sliding
  EXPECT_NEAR(s.omega_w[kFrontLeft] * p.wheel_radius, s.vel_body.x,
              0.02 * s.vel_body.x);       // fronts still roll

  // Lift the brake and the road spins the locked wheels back up. On grip
  // 0.1 the tyre can give about R mu Fz / I = 500 rad/s^2, so reaching road
  // speed (some 75 rad/s) takes about 0.15 s.
  run(*m, s, DriveInput{0.0, 0.0}, 300, grip(0.1));
  EXPECT_NEAR(s.omega_w[kRearLeft] * p.wheel_radius, s.vel_body.x,
              0.05 * s.vel_body.x);
}

TEST(Wheels, TheSameBrakeOnHighGripDoesNotLock) {
  const VehicleParams p = with_wheels();
  auto m = l2(p);
  VehicleState s = rolling(p, 4.0);
  run(*m, s, DriveInput{0.0, -12.0}, 200);
  EXPECT_GT(s.omega_w[kRearLeft], 0.0);
  EXPECT_GT(kappa(p, s, kRearLeft), -0.2);
}

TEST(Wheels, BrakingASpunCarRollingBackwardsDoesNotRunAway) {
  // A spin on low grip can leave the car rolling backwards; motor braking
  // must then oppose the backwards rotation, not drive it.
  const VehicleParams p = with_wheels(1e-6);
  auto m = l2(p);
  VehicleState s = rolling(p, -2.0);
  for (auto& w : s.omega_w) w = -2.0 / p.wheel_radius;
  run(*m, s, DriveInput{0.0, -12.0}, 1000, grip(0.3));
  for (double w : s.omega_w) {
    EXPECT_LE(w, 0.0);
    EXPECT_GT(w, -2.5 / p.wheel_radius);
  }
}

TEST(Wheels, AWheelNeverTurnsBackwardsUnderBraking) {
  const VehicleParams p = with_wheels(1e-5);
  auto m = l2(p);
  VehicleState s = rolling(p, 2.0);
  for (int k = 0; k < 1500; ++k) {
    ASSERT_TRUE(m->step_on(s, DriveInput{0.0, -12.0}, grip(0.05), kDt));
    for (double w : s.omega_w) ASSERT_GE(w, 0.0) << "step " << k;
  }
}

TEST(Wheels, TheEscSeesTheSpinningWheel) {
  // Wheelspin runs the motor up its torque-speed curve, so the delivered
  // torque falls well below what the same throttle gets on grip.
  const VehicleParams p = with_wheels();
  auto m = l2(p);
  VehicleState ice = rolling(p, 1.0);
  VehicleState tarmac = ice;
  const StepDiagnostics di = run(*m, ice, DriveInput{0.0, 8.0}, 400, grip(0.1));
  const StepDiagnostics dt = run(*m, tarmac, DriveInput{0.0, 8.0}, 400);
  EXPECT_LT(di.drive_torque, 0.8 * dt.drive_torque);
}

TEST(Wheels, ASpinningLightWheelSettlesOnTheMotorCurve) {
  // With a very light wheel the motor's torque-speed curve is what sets the
  // spin speed. Held constant over a step, it would let the wheel overshoot
  // to free speed and back every step; solved with the wheel it settles.
  const VehicleParams p = with_wheels(1e-6);
  auto m = l2(p);
  VehicleState s = rolling(p, 1.0);
  run(*m, s, DriveInput{0.0, 8.0}, 400, grip(0.1));
  double lo = 1e9;
  double hi = -1e9;
  for (int k = 0; k < 50; ++k) {
    run(*m, s, DriveInput{0.0, 8.0}, 1, grip(0.1));
    lo = std::fmin(lo, s.omega_w[kRearLeft]);
    hi = std::fmax(hi, s.omega_w[kRearLeft]);
  }
  EXPECT_GT(lo, 2.0 * s.vel_body.x / p.wheel_radius);  // really spinning
  EXPECT_LT(hi - lo, 0.02 * hi);                       // and steady
}

// Full throttle into a full-lock corner, as from a keyboard. The open diff
// spins the unloaded inner wheel, which is real; the loaded outer wheel must
// keep driving, not chatter between drive and brake. A saturated update that
// jumped past the tyre's peak every step did exactly that and made the car
// undrivable; it must now follow the quasi-static car closely.
TEST(Wheels, FullThrottleIntoAFullLockCornerStaysDrivable) {
  const VehicleParams quasi = slipx_test::reference_params();
  const VehicleParams light = with_wheels(1e-5);
  auto mq = l2(quasi);
  auto ml = l2(light);
  VehicleState a;
  VehicleState b;
  StepDiagnostics d;
  double min_outer_fx = 1e9;
  for (int k = 0; k < 3000; ++k) {
    const DriveInput u{k < 1000 ? 0.0 : 0.4, 8.0};
    mq->step(a, u, kDt);
    ASSERT_TRUE(ml->step_on(b, u, GroundContact{}, kDt, &d));
    if (k > 1000) min_outer_fx = std::fmin(min_outer_fx, d.fx[kRearRight]);
  }
  EXPECT_GT(min_outer_fx, 0.0);  // the outer rear never brakes
  EXPECT_NEAR(b.vel_body.x, a.vel_body.x, 0.05 * a.vel_body.x);
  EXPECT_NEAR(b.yaw, a.yaw, 0.05 * std::fabs(a.yaw));
}

// ---------------------------------------------------------- differentials

TEST(WheelsDifferential, AnOpenDiffSpinsTheSlipperyWheel) {
  const VehicleParams p = with_wheels(kInertia, Differential::kOpen);
  auto m = l2(p);
  VehicleState s = rolling(p, 1.0);
  run(*m, s, DriveInput{0.0, 8.0}, 300, grip(1.0, 1.0, 0.1, 1.0));
  EXPECT_GT(s.omega_w[kRearLeft], 2.0 * s.omega_w[kRearRight]);
}

TEST(WheelsDifferential, ASpoolTurnsBothWheelsAtOneSpeed) {
  const VehicleParams p = with_wheels(kInertia, Differential::kSpool);
  auto m = l2(p);
  VehicleState s = rolling(p, 1.0);
  run(*m, s, DriveInput{0.25, 8.0}, 300, grip(1.0, 1.0, 0.1, 1.0));
  EXPECT_EQ(s.omega_w[kRearLeft], s.omega_w[kRearRight]);
}

TEST(WheelsDifferential, MorePreloadFeedsTheGrippingWheelMore) {
  // Split grip, full throttle. Past its preload the LSD runs open with the
  // preload biased to the slower, gripping wheel: more preload, a smaller
  // speed gap across the axle, until it locks. (Measured: 882, 827 and 618
  // rad/s at 0, 0.1 and 0.3 N m.) The car's speed is not asserted: over this
  // window the spinning wheel is still winding up, and its inertia takes the
  // same torque whatever the preload.
  double last_gap = 1e9;
  for (double preload : {0.0, 0.1, 0.3}) {
    VehicleParams p = with_wheels(kInertia, Differential::kLsd);
    p.lsd_preload = preload;
    auto m = l2(p);
    VehicleState s = rolling(p, 1.0);
    run(*m, s, DriveInput{0.0, 8.0}, 300, grip(1.0, 1.0, 0.1, 1.0));
    const double gap = s.omega_w[kRearLeft] - s.omega_w[kRearRight];
    EXPECT_GT(gap, 0.0) << preload;           // still slipping
    EXPECT_LT(gap, 0.95 * last_gap) << preload;
    last_gap = gap;
  }
}

TEST(WheelsDifferential, AnLsdLocksWithinItsPreloadAndSlipsPastIt) {
  VehicleParams locked = with_wheels(kInertia, Differential::kLsd);
  locked.lsd_preload = 10.0;  // far beyond anything this car can produce
  VehicleParams free = with_wheels(kInertia, Differential::kLsd);
  free.lsd_preload = 0.0;
  auto ml = l2(locked);
  auto mf = l2(free);
  VehicleState a = rolling(locked, 1.0);
  VehicleState b = a;
  const GroundContact split = grip(1.0, 1.0, 0.1, 1.0);
  run(*ml, a, DriveInput{0.0, 8.0}, 300, split);
  run(*mf, b, DriveInput{0.0, 8.0}, 300, split);
  EXPECT_EQ(a.omega_w[kRearLeft], a.omega_w[kRearRight]);
  EXPECT_GT(b.omega_w[kRearLeft], 2.0 * b.omega_w[kRearRight]);
}

// -------------------------------------------------------------- invariant

TEST(WheelsInvariant, ARightTurnMirrorsALeftTurnBitForBit) {
  for (Differential diff :
       {Differential::kOpen, Differential::kSpool, Differential::kLsd}) {
    VehicleParams p = with_wheels(kInertia, diff);
    p.lsd_preload = 0.05;
    auto m = l2(p);
    VehicleState a = rolling(p, 1.0);
    VehicleState b = a;
    for (int k = 0; k < 2500; ++k) {
      const DriveInput u = scripted(k);
      const double mu = k < 1200 ? 1.0 : 0.3;
      ASSERT_TRUE(m->step_on(a, u, grip(mu, 1.0, mu, 1.0), kDt));
      ASSERT_TRUE(m->step_on(b, DriveInput{-u.steer_cmd, u.accel_cmd},
                             grip(1.0, mu, 1.0, mu), kDt));
    }
    EXPECT_EQ(a.pos.y, -b.pos.y);
    EXPECT_EQ(a.yaw, -b.yaw);
    EXPECT_EQ(a.vel_body.x, b.vel_body.x);
    EXPECT_EQ(a.omega_w[kFrontLeft], b.omega_w[kFrontRight]);
    EXPECT_EQ(a.omega_w[kRearLeft], b.omega_w[kRearRight]);
    EXPECT_EQ(a.omega_w[kRearRight], b.omega_w[kRearLeft]);
  }
}

TEST(WheelsInvariant, TheSameRunTwiceIsTheSameRun) {
  const VehicleParams p = with_wheels(kInertia, Differential::kLsd);
  auto m = l2(p);
  VehicleState a = rolling(p, 1.0);
  VehicleState b = a;
  for (int k = 0; k < 2500; ++k) {
    ASSERT_TRUE(m->step_on(a, scripted(k), grip(0.4), kDt));
    ASSERT_TRUE(m->step_on(b, scripted(k), grip(0.4), kDt));
  }
  EXPECT_EQ(std::memcmp(&a, &b, sizeof a), 0);
}

}  // namespace
