// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
// Added 2026-10-19 by the rc-physics-twin project: see THIRD_PARTY.md.
//
// Suspension at L2 (rc-physics-twin ADR 0008): the body's three modes at
// their closed-form frequencies, the damping ratio, steady load transfer
// matching the quasi-static formulae, grade, a speed breaker, the bump stop,
// wheel lift and mirror symmetry.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "slipx/ground.hpp"
#include "slipx/load_transfer.hpp"
#include "slipx/vehicle_model.hpp"
#include "test_support.hpp"

namespace {

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

constexpr double kG = slipx::kGravity;
constexpr double kDt = 0.001;
constexpr double kPi = slipx::kPi;
constexpr double kRate = 300.0;  // N/m per wheel: about 3 Hz in heave

VehicleParams sprung(double damping = 0.0, double travel = 0.02) {
  VehicleParams p = slipx_test::reference_params();
  p.wheel_rate_front = kRate;
  p.wheel_rate_rear = kRate;
  p.damping_front = damping;
  p.damping_rear = damping;
  p.suspension_travel = travel;
  return p;
}

std::unique_ptr<VehicleModel> l2(const VehicleParams& p) {
  return VehicleModel::create(Tier::L2_DoubleTrack, p);
}

// Frequency from upward zero crossings of a signal sampled every kDt.
double frequency(const std::vector<double>& x) {
  std::vector<double> t;
  for (std::size_t i = 1; i < x.size(); ++i) {
    if (x[i - 1] < 0.0 && x[i] >= 0.0) {
      t.push_back((static_cast<double>(i) - x[i] / (x[i] - x[i - 1])) * kDt);
    }
  }
  if (t.size() < 2) return 0.0;
  return static_cast<double>(t.size() - 1) / (t.back() - t.front());
}

// Free vibration at rest: one body coordinate displaced, no damping.
std::vector<double> ring(const VehicleParams& p, int which, double amount) {
  auto m = l2(p);
  VehicleState s;
  if (which == 0) s.pos.z = amount;
  if (which == 1) s.pitch = amount;
  if (which == 2) s.roll = amount;
  std::vector<double> x;
  for (int k = 0; k < 2000; ++k) {
    m->step(s, DriveInput{0.0, 0.0}, kDt);
    x.push_back(which == 0 ? s.pos.z : which == 1 ? s.pitch : s.roll);
  }
  return x;
}

// ------------------------------------------------------------- parameters

TEST(Suspension, ParametersAreChecked) {
  VehicleParams p = sprung();
  EXPECT_EQ(slipx::validate(p), nullptr);
  p.wheel_rate_rear = 0.0;
  EXPECT_NE(slipx::validate(p), nullptr);  // both or neither
  p = sprung();
  p.suspension_travel = 0.0;
  EXPECT_NE(slipx::validate(p), nullptr);
  p = sprung();
  p.damping_front = -1.0;
  EXPECT_NE(slipx::validate(p), nullptr);
  EXPECT_FALSE(slipx_test::reference_params().has_suspension());
}

TEST(Suspension, ACarAtRestOnLevelGroundIsInEquilibrium) {
  for (double front_rate : {kRate, 450.0}) {  // and with unequal axles
  VehicleParams p = sprung(5.0);
  p.wheel_rate_front = front_rate;
  SCOPED_TRACE(front_rate);
  auto m = l2(p);
  VehicleState s;
  for (int k = 0; k < 1000; ++k) m->step(s, DriveInput{0.0, 0.0}, kDt);
  EXPECT_NEAR(s.pos.z, 0.0, 1e-12);
  EXPECT_NEAR(s.pitch, 0.0, 1e-12);
  EXPECT_NEAR(s.roll, 0.0, 1e-12);
  const slipx::WheelLoads st = slipx::static_loads(p);
  for (unsigned i = 0; i < 4; ++i) EXPECT_NEAR(s.Fz[i], st.fz[i], 1e-9);
  }
}

TEST(Suspension, AReboundDamperNeverPullsTheTyreDown) {
  // A body rising fast against a stiff damper: k (sag) - c z' is negative,
  // but a tyre cannot pull the road, so its load is zero, not less.
  VehicleParams p = sprung(20.0);
  auto m = l2(p);
  VehicleState s;
  s.vel_body.z = 2.0;
  m->step(s, DriveInput{0.0, 0.0}, kDt);
  m->step(s, DriveInput{0.0, 0.0}, kDt);
  for (unsigned i = 0; i < 4; ++i) EXPECT_GE(s.Fz[i], 0.0) << i;
  EXPECT_EQ(s.Fz[kFrontLeft], 0.0);
}

// ----------------------------------------------------- modal frequencies

TEST(SuspensionModes, HeaveRingsAtSqrtOfTotalRateOverMass) {
  const VehicleParams p = sprung();
  const double expected = std::sqrt(4.0 * kRate / p.mass) / (2.0 * kPi);
  EXPECT_NEAR(frequency(ring(p, 0, -0.004)), expected, 0.01 * expected);
}

TEST(SuspensionModes, PitchRingsWithTheGravityTermIncluded) {
  // About the point below the CoG the weight is a destabilising moment
  // m g h p, so the stiffness is sum k x^2 - m g h.
  const VehicleParams p = sprung();
  const double kx = 2.0 * kRate * (p.lf * p.lf + p.lr * p.lr);
  const double expected =
      std::sqrt((kx - p.mass * kG * p.h_cog) / p.iyy) / (2.0 * kPi);
  EXPECT_NEAR(frequency(ring(p, 1, 0.01)), expected, 0.01 * expected);
}

TEST(SuspensionModes, RollRingsWithTheGravityTermIncluded) {
  const VehicleParams p = sprung();
  const double ky = kRate * 0.5 * (p.track_front * p.track_front +
                                   p.track_rear * p.track_rear);
  const double expected =
      std::sqrt((ky - p.mass * kG * p.h_cog) / p.ixx) / (2.0 * kPi);
  EXPECT_NEAR(frequency(ring(p, 2, 0.01)), expected, 0.01 * expected);
}

TEST(SuspensionModes, HeaveDecaysAtTheDampingRatio) {
  const double c = 3.0;
  const VehicleParams p = sprung(c);
  const auto x = ring(p, 0, -0.004);
  std::vector<double> peaks;
  for (std::size_t i = 1; i + 1 < x.size(); ++i) {
    if (x[i] > x[i - 1] && x[i] >= x[i + 1] && x[i] > 0.0) peaks.push_back(x[i]);
  }
  ASSERT_GE(peaks.size(), 2u);
  const double zeta = 4.0 * c / (2.0 * std::sqrt(4.0 * kRate * p.mass));
  const double expected = std::exp(-2.0 * kPi * zeta / std::sqrt(1.0 - zeta * zeta));
  EXPECT_NEAR(peaks[1] / peaks[0], expected, 0.02 * expected);
}

// ------------------------------------------------------- steady transfer

TEST(SuspensionSteady, AccelerationTransfersMAHOverL) {
  // Once the pitch has settled, the springs carry SlipX's quasi-static
  // longitudinal transfer, m a h / L, with a the CoG's actual acceleration,
  // plus what the pitch itself adds by moving the CoG back: m g h pitch / L.
  // (Without that term the transfer comes out 7 % high, which is the body's
  // own contribution, not an error.)
  const VehicleParams p = sprung(4.0);
  auto m = l2(p);
  VehicleState s = slipx_test::travelling(1.0);
  for (int k = 0; k < 1500; ++k) m->step(s, DriveInput{0.0, 2.0}, kDt);
  const double v0 = s.vel_body.x;
  m->step(s, DriveInput{0.0, 2.0}, kDt);
  const double a = (s.vel_body.x - v0) / kDt;
  const slipx::WheelLoads st = slipx::static_loads(p);
  const double rear_gain = (s.Fz[kRearLeft] + s.Fz[kRearRight]) - st.fz_rear;
  const double expected = p.mass * p.h_cog * (a + kG * s.pitch) / p.wheelbase();
  EXPECT_GT(s.pitch, 0.0);  // squat: nose up
  EXPECT_NEAR(rear_gain, expected, 0.01 * expected);
}

TEST(SuspensionSteady, CorneringTransfersMAyHOverT) {
  const VehicleParams p = sprung(4.0);
  auto m = l2(p);
  VehicleState s = slipx_test::travelling(2.0);
  StepDiagnostics d;
  for (int k = 0; k < 3000; ++k) {
    m->step(s, DriveInput{0.15, slipx_test::hold_speed(s, 2.0)}, kDt, &d);
  }
  const double ay = s.vel_body.x * s.rates.z;  // steady: centripetal
  const double right_minus_left = (s.Fz[kFrontRight] - s.Fz[kFrontLeft]) +
                                  (s.Fz[kRearRight] - s.Fz[kRearLeft]);
  // As for pitch: the roll moves the CoG outward and adds m g h roll.
  const double expected =
      2.0 * p.mass * p.h_cog * (ay + kG * s.roll) / p.track_front;
  EXPECT_GT(s.roll, 0.0);  // a left turn rolls the body right side down
  EXPECT_NEAR(right_minus_left, expected, 0.01 * expected);
}

// ------------------------------------------------------------------ grade

// Ground falling toward world +x with tangent g, sampled under each wheel.
GroundContact slope_under(const VehicleParams& p, const VehicleState& s,
                          double tangent) {
  GroundContact g;
  const double bx[4] = {p.lf, p.lf, -p.lr, -p.lr};
  const double by[4] = {p.track_front / 2, -p.track_front / 2,
                        p.track_rear / 2, -p.track_rear / 2};
  const double n = std::sqrt(1.0 + tangent * tangent);
  for (unsigned i = 0; i < 4; ++i) {
    const double x = s.pos.x + std::cos(s.yaw) * bx[i] - std::sin(s.yaw) * by[i];
    g.wheel[i].height = -tangent * x;
    g.wheel[i].normal = {tangent / n, 0.0, 1.0 / n};
  }
  return g;
}

TEST(SuspensionGrade, CoastingFreelyDownAGradeCarriesNoTransfer) {
  // Gravity along the road and the inertia of the CoG it accelerates cancel:
  // a freely rolling car on an incline is loaded as at rest, front to rear.
  // Averaged over the last half second, so the residual heave ringing does
  // not decide the result.
  const VehicleParams p = sprung(8.0);
  auto m = l2(p);
  VehicleState s = slipx_test::travelling(2.0);
  s.pitch = -std::atan(0.1);  // already lying on the grade,
  s.vel_body.z = -2.0 * 0.1;  // and sinking with it
  double front = 0.0;
  double rear = 0.0;
  const double v_start_avg = [&] {
    for (int k = 0; k < 2500; ++k) {
      EXPECT_TRUE(m->step_on(s, DriveInput{0.0, 0.0}, slope_under(p, s, 0.1), kDt));
    }
    return s.vel_body.x;
  }();
  const int n = 500;
  for (int k = 0; k < n; ++k) {
    ASSERT_TRUE(m->step_on(s, DriveInput{0.0, 0.0}, slope_under(p, s, 0.1), kDt));
    front += s.Fz[kFrontLeft] + s.Fz[kFrontRight];
    rear += s.Fz[kRearLeft] + s.Fz[kRearRight];
  }
  front /= n;
  rear /= n;
  const double a = (s.vel_body.x - v_start_avg) / (n * kDt);
  EXPECT_NEAR(front / (front + rear), p.lr / p.wheelbase(), 0.005);
  EXPECT_NEAR(s.pitch, -std::atan(0.1), 3e-3);  // the body follows the road
  // A real car on a straight grade has no acceleration normal to the road,
  // so its tyres carry m g cos(theta). These springs are vertical (ADR 0008),
  // so the vertical part of the car's acceleration down the grade, a
  // tan(theta), comes off their load: about 1.5 % here, inside the stated
  // small-angle limit. The front/rear split above is unaffected.
  const double normal = p.mass * kG * std::cos(std::atan(0.1));
  EXPECT_NEAR(front + rear, normal, 0.02 * normal);
  EXPECT_LT(front + rear, normal);  // and the error has the predicted sign
  (void)a;
}

// --------------------------------------------------------- road features

GroundContact bump_under_front(double height) {
  GroundContact g;
  g.wheel[kFrontLeft].height = height;
  g.wheel[kFrontRight].height = height;
  return g;
}

TEST(SuspensionRoad, ASpeedBreakerKicksTheFrontAndSettles) {
  // Against the same car on a smooth road. Damped at about 0.2 of critical
  // in pitch; lightly damped (2 N s/m, 0.08) it is still ringing at 2.5 s.
  const VehicleParams p = sprung(6.0, 0.03);
  auto m = l2(p);
  VehicleState s = slipx_test::travelling(2.0);
  VehicleState smooth = s;
  double peak_front = 0.0;
  double peak_pitch = 0.0;
  double base_front = 0.0;
  double base_pitch = 0.0;
  for (int k = 0; k < 2500; ++k) {
    const double h = (k >= 100 && k < 150) ? 0.015 : 0.0;  // 1.5 cm for 50 ms
    ASSERT_TRUE(m->step_on(s, DriveInput{0.0, slipx_test::hold_speed(s, 2.0)},
                           bump_under_front(h), kDt));
    ASSERT_TRUE(m->step_on(smooth, DriveInput{0.0, slipx_test::hold_speed(smooth, 2.0)},
                           GroundContact{}, kDt));
    if (k < 300) {
      peak_front = std::fmax(peak_front, s.Fz[kFrontLeft] - smooth.Fz[kFrontLeft]);
      peak_pitch = std::fmax(peak_pitch, s.pitch - smooth.pitch);
    }
    base_front = smooth.Fz[kFrontLeft];
    base_pitch = smooth.pitch;
  }
  EXPECT_GT(peak_front, 0.5 * base_front);  // the hit
  EXPECT_GT(peak_pitch, 0.005);             // nose thrown up
  EXPECT_NEAR(s.pitch, base_pitch, 1e-4);   // and settled again
  EXPECT_NEAR(s.Fz[kFrontLeft], base_front, 0.01 * base_front);
}

TEST(SuspensionRoad, PastItsTravelTheBumpStopTakesOver) {
  // A step taller than the travel: the spring alone would carry k (sag + h);
  // the bump stop adds ten times the rate past the travel.
  const double travel = 0.01;
  const double h = 0.02;
  const VehicleParams p = sprung(0.0, travel);
  auto m = l2(p);
  VehicleState s;
  StepDiagnostics d;
  m->step_on(s, DriveInput{0.0, 0.0}, bump_under_front(h), 1e-7, &d);
  const double sag = slipx::static_loads(p).fz[kFrontLeft] / kRate;
  m->step_on(s, DriveInput{0.0, 0.0}, bump_under_front(h), 1e-7, &d);
  const double expected = kRate * (sag + h) + 10.0 * kRate * (h - travel);
  EXPECT_NEAR(s.Fz[kFrontLeft], expected, 1e-3 * expected);
}

TEST(SuspensionRoad, AWheelOverAHoleLeavesTheGround) {
  const VehicleParams p = sprung(2.0);
  auto m = l2(p);
  VehicleState s;
  GroundContact hole;
  hole.wheel[kRearRight].height = -0.1;  // far deeper than the sag
  for (int k = 0; k < 5; ++k) ASSERT_TRUE(m->step_on(s, DriveInput{0.0, 0.0}, hole, kDt));
  EXPECT_EQ(s.Fz[kRearRight], 0.0);
}

TEST(SuspensionRoad, StableOnARoughRoadAtOneKilohertz) {
  // Rough but drivable: 8 mm bumps, different under every wheel, against
  // 15 mm of travel. (Several centimetres under each wheel at once throws
  // this soft car into the air and over, which is past what a small-angle
  // body can describe; ADR 0008 states it.)
  const VehicleParams p = sprung(6.0, 0.015);
  auto m = l2(p);
  VehicleState s = slipx_test::travelling(3.0);
  for (int k = 0; k < 4000; ++k) {
    GroundContact g;
    for (unsigned i = 0; i < 4; ++i) {
      g.wheel[i].height = 0.008 * std::sin(0.013 * k + 1.7 * i) * ((k / 200) % 2);
    }
    ASSERT_TRUE(m->step_on(s, DriveInput{0.1 * std::sin(0.002 * k), 3.0}, g, kDt));
  }
  EXPECT_TRUE(std::isfinite(s.pos.z));
  EXPECT_LT(std::fabs(s.pitch), 0.3);
  EXPECT_LT(std::fabs(s.roll), 0.3);
}

// -------------------------------------------------------------- invariant

TEST(SuspensionInvariant, ARightTurnMirrorsALeftTurnBitForBit) {
  VehicleParams p = sprung(3.0, 0.015);
  p.wheel_inertia = 1e-5;
  p.ackermann = 1.0;
  auto m = l2(p);
  VehicleState a = slipx_test::travelling(1.0);
  for (auto& w : a.omega_w) w = 1.0 / p.wheel_radius;
  VehicleState b = a;
  for (int k = 0; k < 2500; ++k) {
    const double steer = k < 1200 ? 0.3 : -0.2;
    GroundContact ga;
    GroundContact gb;
    if (k > 600 && k < 640) {  // a bump under the left wheels, mirrored
      ga.wheel[kFrontLeft].height = 0.01;
      gb.wheel[kFrontRight].height = 0.01;
    }
    ASSERT_TRUE(m->step_on(a, DriveInput{steer, 2.0}, ga, kDt));
    ASSERT_TRUE(m->step_on(b, DriveInput{-steer, 2.0}, gb, kDt));
  }
  EXPECT_EQ(a.pos.y, -b.pos.y);
  EXPECT_EQ(a.yaw, -b.yaw);
  EXPECT_EQ(a.roll, -b.roll);
  EXPECT_EQ(a.pitch, b.pitch);
  EXPECT_EQ(a.pos.z, b.pos.z);
  EXPECT_EQ(a.Fz[kFrontLeft], b.Fz[kFrontRight]);
}

}  // namespace
