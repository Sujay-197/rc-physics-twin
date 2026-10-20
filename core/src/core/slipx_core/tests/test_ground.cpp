// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
// Added 2026-10-19 by the rc-physics-twin project: see THIRD_PARTY.md.
//
// VehicleModel::step_on and the L2 ground model (ground.hpp, rc-physics-twin
// ADR 0005). Level ground must be step exactly; a grade must match the
// closed form; a bank must mirror bit for bit; grip must cap force; and a
// tier without a ground model must refuse rather than ignore.

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <limits>

#include "slipx/ground.hpp"
#include "slipx/load_transfer.hpp"
#include "slipx/tyre.hpp"
#include "slipx/vehicle_model.hpp"
#include "test_support.hpp"

namespace {

using slipx::DriveInput;
using slipx::GroundContact;
using slipx::Integrator;
using slipx::StepDiagnostics;
using slipx::Tier;
using slipx::Vec3;
using slipx::VehicleModel;
using slipx::VehicleParams;
using slipx::VehicleState;

constexpr double kG = slipx::kGravity;
constexpr double kDt = 0.001;

GroundContact uniform(Vec3 n, double mu = 1.0, double h = 0.0) {
  GroundContact g;
  for (auto& w : g.wheel) {
    w.normal = n;
    w.mu_scale = mu;
    w.height = h;
  }
  return g;
}

// Road descending toward world +x at angle theta: z = -x tan(theta).
GroundContact downhill_x(double theta) {
  return uniform(Vec3{std::sin(theta), 0.0, std::cos(theta)});
}

// Road whose left (world +y) side is higher by angle phi; negative phi
// raises the right side.
GroundContact bank_left_high(double phi) {
  return uniform(Vec3{0.0, -std::sin(phi), std::cos(phi)});
}

bool same_bits(const VehicleState& a, const VehicleState& b) {
  return std::memcmp(&a, &b, sizeof a) == 0;
}

std::unique_ptr<VehicleModel> make(Tier t, Integrator i = Integrator::kRK4,
                                   VehicleParams p = slipx_test::reference_params()) {
  return VehicleModel::create(t, p, i);
}

DriveInput scripted(int k) {
  if (k < 600) return DriveInput{0.0, 4.0};
  if (k < 1200) return DriveInput{0.25, 1.0};
  if (k < 1700) return DriveInput{-0.3, 0.0};
  return DriveInput{0.05, -6.0};
}

// Runs n steps on the given ground with a constant input; returns the last
// diagnostics. Fails the test if any step is refused.
StepDiagnostics drive(const VehicleModel& m, VehicleState& s,
                      const GroundContact& g, DriveInput u, int n) {
  StepDiagnostics d;
  for (int k = 0; k < n; ++k) {
    if (!m.step_on(s, u, g, kDt, &d)) {
      ADD_FAILURE() << "step_on refused at step " << k;
      break;
    }
  }
  return d;
}

// ------------------------------------------------------------------ level

TEST(Ground, LevelGroundIsExactlyStepOnEveryTier) {
  const GroundContact level{};
  for (Tier t : {Tier::L0_Kinematic, Tier::L1_Bicycle, Tier::L2_DoubleTrack}) {
    for (Integrator i : {Integrator::kRK4, Integrator::kSemiImplicitEuler}) {
      auto m = make(t, i);
      VehicleState a = slipx_test::travelling(0.5);
      VehicleState b = a;
      for (int k = 0; k < 2200; ++k) {
        m->step(a, scripted(k), kDt);
        ASSERT_TRUE(m->step_on(b, scripted(k), level, kDt));
        ASSERT_TRUE(same_bits(a, b)) << slipx::to_string(t) << " step " << k;
      }
    }
  }
}

TEST(Ground, LevelGroundAtAHeightOnlySetsPosZ) {
  for (Tier t : {Tier::L0_Kinematic, Tier::L1_Bicycle, Tier::L2_DoubleTrack}) {
    auto m = make(t);
    VehicleState a = slipx_test::travelling(1.0);
    VehicleState b = a;
    const GroundContact raised = uniform(Vec3{0.0, 0.0, 2.0}, 1.0, 1.25);
    for (int k = 0; k < 500; ++k) {
      m->step(a, scripted(k), kDt);
      ASSERT_TRUE(m->step_on(b, scripted(k), raised, kDt));
    }
    EXPECT_EQ(b.pos.z, 1.25);
    b.pos.z = a.pos.z;
    EXPECT_TRUE(same_bits(a, b)) << slipx::to_string(t);
  }
}

// ---------------------------------------------------------------- refusal

TEST(Ground, TiersWithoutAGroundModelRefuseGradeAndGrip) {
  for (Tier t : {Tier::L0_Kinematic, Tier::L1_Bicycle}) {
    auto m = make(t);
    VehicleState s = slipx_test::travelling(2.0);
    const VehicleState before = s;
    EXPECT_FALSE(m->step_on(s, DriveInput{0.0, 1.0}, downhill_x(0.05), kDt));
    EXPECT_FALSE(m->step_on(s, DriveInput{0.0, 1.0},
                            uniform(Vec3{0.0, 0.0, 1.0}, 0.5), kDt));
    EXPECT_TRUE(same_bits(s, before)) << slipx::to_string(t);
  }
}

TEST(Ground, InvalidSamplesAreRefusedAndLeaveTheStateAlone) {
  auto m = make(Tier::L2_DoubleTrack);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  GroundContact bad[5] = {GroundContact{}, GroundContact{}, GroundContact{},
                          GroundContact{}, GroundContact{}};
  bad[0].wheel[0].height = nan;
  bad[1].wheel[1].normal = Vec3{0.0, 0.0, 0.0};
  bad[2].wheel[2].normal = Vec3{0.3, 0.0, -1.0};
  bad[3].wheel[3].mu_scale = -0.1;
  bad[4].wheel[0].mu_scale = nan;
  for (const auto& g : bad) {
    VehicleState s = slipx_test::travelling(2.0);
    const VehicleState before = s;
    EXPECT_NE(slipx::validate(g), nullptr);
    EXPECT_FALSE(m->step_on(s, DriveInput{0.0, 1.0}, g, kDt));
    EXPECT_TRUE(same_bits(s, before));
  }
}

// ------------------------------------------------------------- analytical

// Coasting with no drive demand, the difference between a grade and level
// ground is gravity along the road, g sin(theta), less the small change in
// rolling resistance (which scales with cos(theta)). The difference is taken
// so drag and rolling resistance cancel to first order.
double coast_delta_v(double theta, double seconds) {
  auto m = make(Tier::L2_DoubleTrack);
  VehicleState graded = slipx_test::travelling(3.0);
  VehicleState level = graded;
  const int n = static_cast<int>(std::lround(seconds / kDt));
  drive(*m, graded, downhill_x(theta), DriveInput{0.0, 0.0}, n);
  drive(*m, level, GroundContact{}, DriveInput{0.0, 0.0}, n);
  return graded.vel_body.x - level.vel_body.x;
}

TEST(GroundAnalytical, CoastingDownAGradeGainsGSinTheta) {
  const double theta = std::atan(0.05);  // a 5 % grade
  const double t = 0.5;
  const double expected = kG * std::sin(theta) * t;
  EXPECT_NEAR(coast_delta_v(theta, t), expected, 0.02 * expected);
}

TEST(GroundAnalytical, CoastingUpAGradeLosesGSinTheta) {
  const double theta = std::atan(0.05);
  const double t = 0.5;
  const double expected = -kG * std::sin(theta) * t;
  EXPECT_NEAR(coast_delta_v(-theta, t), expected, 0.02 * std::fabs(expected));
}

TEST(GroundAnalytical, ForwardMeansTheBodyAxisNotWorldX) {
  // Facing world +y on a road that falls toward +y is downhill too.
  auto m = make(Tier::L2_DoubleTrack);
  const double theta = std::atan(0.05);
  VehicleState s = slipx_test::travelling(3.0);
  s.yaw = 0.5 * slipx::kPi;
  VehicleState level = s;
  drive(*m, s, uniform(Vec3{0.0, std::sin(theta), std::cos(theta)}),
        DriveInput{0.0, 0.0}, 500);
  drive(*m, level, GroundContact{}, DriveInput{0.0, 0.0}, 500);
  const double expected = kG * std::sin(theta) * 0.5;
  EXPECT_NEAR(s.vel_body.x - level.vel_body.x, expected, 0.02 * expected);
}

TEST(GroundAnalytical, TyreLoadsCarryOnlyTheNormalComponentOfWeight) {
  auto m = make(Tier::L2_DoubleTrack);
  const double theta = std::atan(0.10);
  VehicleState s = slipx_test::travelling(2.0);
  drive(*m, s, downhill_x(theta), DriveInput{0.0, 0.0}, 50);
  const double sum = (s.Fz[0] + s.Fz[1]) + (s.Fz[2] + s.Fz[3]);
  const double weight_normal = m->params().mass * kG * std::cos(theta);
  EXPECT_NEAR(sum, weight_normal, 1e-9 * weight_normal);
}

TEST(GroundAnalytical, ClimbingAtSteadySpeedLoadsTheRearAxle) {
  // Holding speed uphill takes tyre force at the ground against gravity at
  // the CoG: that couple moves load to the rear, as on a parked car.
  auto m = make(Tier::L2_DoubleTrack);
  const double theta = std::atan(0.10);
  VehicleState up = slipx_test::travelling(2.0);
  VehicleState level = up;
  for (int k = 0; k < 1000; ++k) {
    ASSERT_TRUE(m->step_on(up, DriveInput{0.0, slipx_test::hold_speed(up, 2.0) +
                                                    kG * std::sin(theta)},
                           downhill_x(-theta), kDt));
    ASSERT_TRUE(m->step_on(level, DriveInput{0.0, slipx_test::hold_speed(level, 2.0)},
                           GroundContact{}, kDt));
  }
  const double rear_share_up = (up.Fz[2] + up.Fz[3]) /
                               ((up.Fz[0] + up.Fz[1]) + (up.Fz[2] + up.Fz[3]));
  const double rear_share_level =
      (level.Fz[2] + level.Fz[3]) /
      ((level.Fz[0] + level.Fz[1]) + (level.Fz[2] + level.Fz[3]));
  EXPECT_GT(rear_share_up, rear_share_level + 0.01);
}

TEST(GroundAnalytical, ABankPushesTheCarTowardItsLowSide) {
  auto m = make(Tier::L2_DoubleTrack);
  VehicleState s = slipx_test::travelling(3.0);
  drive(*m, s, bank_left_high(std::atan(0.10)), DriveInput{0.0, 0.0}, 500);
  EXPECT_LT(s.pos.y, -1e-3);  // drifted right, toward the low side
}

TEST(GroundDiagnostics, TheAccelerometerReadsResistanceAndNotGravity) {
  // Coasting straight down a steep grade with no drive, the tyres carry no
  // force, so the specific force is drag plus rolling resistance, and the
  // rolling resistance presses on g cos(theta), not g.
  auto m = make(Tier::L2_DoubleTrack);
  const VehicleParams& p = m->params();
  const double theta = std::atan(0.30);
  VehicleState s = slipx_test::travelling(3.0);
  const VehicleState start = s;
  const StepDiagnostics d =
      drive(*m, s, downhill_x(theta), DriveInput{0.0, 0.0}, 1);
  const double v = s.vel_body.x;
  const double expected =
      -(p.drag_coeff * v * std::fabs(v) +
        p.roll_resist * p.mass * kG * std::cos(theta) * std::tanh(v / p.v_eps)) /
      p.mass;
  EXPECT_NEAR(d.ax, expected, 1e-9);
  EXPECT_GT(s.vel_body.x, start.vel_body.x);  // yet gravity sped it up
}

TEST(GroundGrip, LowGripCapsCorneringAtTheFrictionLimit) {
  // A hard step steer at speed asks for far more lateral force than mu 0.3
  // can give; the lateral specific force must stay inside it throughout.
  auto m = make(Tier::L2_DoubleTrack);
  const VehicleParams& p = m->params();
  const double mu = 0.3;
  VehicleState s = slipx_test::travelling(4.0);
  double peak = 0.0;
  StepDiagnostics d;
  for (int k = 0; k < 800; ++k) {
    ASSERT_TRUE(m->step_on(s, DriveInput{0.35, 0.0},
                           uniform(Vec3{0.0, 0.0, 1.0}, mu), kDt, &d));
    peak = std::fmax(peak, std::fabs(d.ay));
  }
  // Load sensitivity lets a lightly loaded tyre exceed its nominal mu a
  // little, hence the margin; the unscaled car would reach several times this.
  EXPECT_LE(peak, 1.15 * mu * p.tyre_front.mu_y0 * kG);
  EXPECT_GT(peak, 0.5 * mu * p.tyre_front.mu_y0 * kG);
}

TEST(GroundAnalytical, BothLoadPassesStartFromTheGradedWeight) {
  // Friction-limited straight-line drive on a steep grade, worked by hand
  // through the two load passes from the public pieces: graded static loads,
  // the rear budget at those loads, the acceleration it gives, and the loads
  // that acceleration transfers. The reported loads must be exactly those.
  auto m = make(Tier::L2_DoubleTrack);
  const VehicleParams& p = m->params();
  const double theta = std::atan(0.30);
  const double mu = 0.2;
  VehicleState s = slipx_test::travelling(2.0);
  // Uphill on low grip, so the rear tyres are at their limit in both passes.
  drive(*m, s, uniform(Vec3{-std::sin(theta), 0.0, std::cos(theta)}, mu),
        DriveInput{0.0, 8.0}, 1);

  const double gn = kG * std::cos(theta);
  const slipx::WheelLoads statics = slipx::static_loads(p, gn);
  const slipx::WheelLoads level = slipx::static_loads(p);
  const slipx::MfLite rear = slipx::make_mf_lite(
      p.tyre_rear, 0.5 * p.c_alpha_r, level.fz[slipx::kRearLeft]);
  const slipx::NominalPeak peak = slipx::nominal_peak(rear);
  const double fx_wheel =
      peak.x * slipx::load_factor(rear, statics.fz[slipx::kRearLeft]) * mu;
  const double v = s.vel_body.x;
  const double resistance =
      p.drag_coeff * v * std::fabs(v) +
      p.roll_resist * p.mass * gn * std::tanh(v / p.v_eps);
  const double ax1 = (2.0 * fx_wheel - resistance) / p.mass;
  const slipx::WheelLoads expected = slipx::quasi_static_loads(p, ax1, 0.0, gn);
  for (unsigned i = 0; i < slipx::kWheelCount; ++i) {
    EXPECT_NEAR(s.Fz[i], expected.fz[i], 1e-12 * expected.fz[i]) << "wheel " << i;
  }
}

TEST(GroundAnalytical, ASampleNormalsLengthCarriesNoWeight) {
  // One wheel on a slope, three on the level: scaling that wheel's normal by
  // four (exactly representable) must change nothing at all.
  auto m = make(Tier::L2_DoubleTrack);
  const double theta = std::atan(0.2);
  GroundContact unit = uniform(Vec3{0.0, 0.0, 1.0});
  unit.wheel[slipx::kFrontLeft].normal = Vec3{std::sin(theta), 0.0, std::cos(theta)};
  GroundContact longer = unit;
  longer.wheel[slipx::kFrontLeft].normal =
      Vec3{4.0 * std::sin(theta), 0.0, 4.0 * std::cos(theta)};
  VehicleState a = slipx_test::travelling(2.0);
  VehicleState b = a;
  drive(*m, a, unit, DriveInput{0.1, 2.0}, 300);
  drive(*m, b, longer, DriveInput{0.1, 2.0}, 300);
  EXPECT_TRUE(same_bits(a, b));
  VehicleState level = slipx_test::travelling(2.0);
  drive(*m, level, GroundContact{}, DriveInput{0.1, 2.0}, 300);
  EXPECT_FALSE(same_bits(a, level));  // the one sloped wheel did count
}

// -------------------------------------------------------------- invariant

TEST(GroundInvariant, ALeftBankMirrorsARightBankBitForBit) {
  for (Integrator i : {Integrator::kRK4, Integrator::kSemiImplicitEuler}) {
    auto m = make(Tier::L2_DoubleTrack, i);
    VehicleState a = slipx_test::travelling(2.5);
    VehicleState b = a;
    const double phi = std::atan(0.08);
    for (int k = 0; k < 1500; ++k) {
      const DriveInput u = scripted(k);
      ASSERT_TRUE(m->step_on(a, u, bank_left_high(phi), kDt));
      ASSERT_TRUE(m->step_on(b, DriveInput{-u.steer_cmd, u.accel_cmd},
                             bank_left_high(-phi), kDt));
    }
    EXPECT_EQ(a.pos.x, b.pos.x);
    EXPECT_EQ(a.pos.y, -b.pos.y);
    EXPECT_EQ(a.yaw, -b.yaw);
    EXPECT_EQ(a.vel_body.x, b.vel_body.x);
    EXPECT_EQ(a.vel_body.y, -b.vel_body.y);
    EXPECT_EQ(a.rates.z, -b.rates.z);
    EXPECT_EQ(a.Fz[0], b.Fz[1]);
    EXPECT_EQ(a.Fz[2], b.Fz[3]);
  }
}

// ------------------------------------------------------------------- grip

double accel_over(double mu, double seconds) {
  auto m = make(Tier::L2_DoubleTrack);
  VehicleState s = slipx_test::travelling(1.0);
  const int n = static_cast<int>(std::lround(seconds / kDt));
  drive(*m, s, uniform(Vec3{0.0, 0.0, 1.0}, mu), DriveInput{0.0, 8.0}, n);
  return (s.vel_body.x - 1.0) / seconds;
}

TEST(GroundGrip, LowGripCapsTractionAtTheFrictionLimit) {
  const double high = accel_over(1.0, 0.3);
  const double low = accel_over(0.2, 0.3);
  const VehicleParams p = slipx_test::reference_params();
  // No car can out-accelerate mu g on all four wheels; this one drives two.
  EXPECT_LE(low, 0.2 * p.tyre_rear.mu_x0 * kG);
  EXPECT_GT(low, 0.0);
  EXPECT_LT(low, 0.5 * high);
}

TEST(GroundGrip, NoGripMeansNoTraction) {
  auto m = make(Tier::L2_DoubleTrack);
  VehicleState s = slipx_test::travelling(2.0);
  const StepDiagnostics d =
      drive(*m, s, uniform(Vec3{0.0, 0.0, 1.0}, 0.0), DriveInput{0.0, 8.0}, 300);
  EXPECT_LT(s.vel_body.x, 2.0);  // only resistance acts
  EXPECT_TRUE(d.tyre_saturated[slipx::kRearLeft]);
  EXPECT_TRUE(d.tyre_saturated[slipx::kRearRight]);
  EXPECT_EQ(d.fx[slipx::kRearLeft], 0.0);
}

TEST(GroundGrip, SplitGripOnASpoolYawsTowardTheSlipperySide) {
  // The grippy right rear wheel pushes harder than the left, so the car
  // turns left: positive yaw. Mirrored grip gives the mirrored yaw exactly.
  VehicleParams p = slipx_test::reference_params();
  p.differential = slipx::Differential::kSpool;
  auto m = make(Tier::L2_DoubleTrack, Integrator::kRK4, p);
  GroundContact left_slippery = uniform(Vec3{0.0, 0.0, 1.0});
  left_slippery.wheel[slipx::kFrontLeft].mu_scale = 0.2;
  left_slippery.wheel[slipx::kRearLeft].mu_scale = 0.2;
  GroundContact right_slippery = uniform(Vec3{0.0, 0.0, 1.0});
  right_slippery.wheel[slipx::kFrontRight].mu_scale = 0.2;
  right_slippery.wheel[slipx::kRearRight].mu_scale = 0.2;

  VehicleState a = slipx_test::travelling(1.0);
  VehicleState b = a;
  drive(*m, a, left_slippery, DriveInput{0.0, 8.0}, 300);
  drive(*m, b, right_slippery, DriveInput{0.0, 8.0}, 300);
  EXPECT_GT(a.yaw, 1e-3);
  EXPECT_EQ(a.yaw, -b.yaw);
}

}  // namespace
