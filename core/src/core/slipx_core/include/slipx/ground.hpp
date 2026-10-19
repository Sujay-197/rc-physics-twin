// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
// Added 2026-10-19 by the rc-physics-twin project: see THIRD_PARTY.md.
//
// The ground under each contact patch, as the host sees it at the start of a
// step (rc-physics-twin ADR 0005).
//
// The core does not ask for the ground: it is handed four samples, one per
// wheel, alongside the drive input. That keeps step a pure function of its
// arguments (no callback, no hidden state, nothing the core can call that
// might differ between runs), and the host stays free to answer from a
// height map, a road graph or a raycast. Samples are taken at the wheel
// centres projected onto the ground at the entry state and are held for the
// step, the same quasi-static treatment the ESC budget gets: a wheel moves at
// most a couple of centimetres in a 1 ms step.
//
// What a tier does with the samples is its own business and is documented
// there. A tier that cannot represent a sample refuses the step rather than
// ignoring it (VehicleModel::step_on).

#ifndef SLIPX_GROUND_HPP
#define SLIPX_GROUND_HPP

#include <array>
#include <cmath>

#include "slipx/conventions.hpp"
#include "slipx/math.hpp"

namespace slipx {

struct GroundSample {
  // Ground height under the wheel, world z.                             [m]
  double height = 0.0;
  // Outward surface normal, world frame. Need not be unit length; it is
  // normalised by the tier. Must point up (z > 0).                       [-]
  Vec3 normal{0.0, 0.0, 1.0};
  // Multiplies the tyre's peak friction on this surface: 1 is the surface
  // the tyre coefficients describe, 0.3 is roughly wet paint or dust.
  // Not negative.                                                        [-]
  double mu_scale = 1.0;
};

struct GroundContact {
  std::array<GroundSample, kWheelCount> wheel{};
};

// True if every sample is level and unscaled: normal exactly (0, 0, z) with
// z > 0, and mu_scale exactly 1. Heights may differ; level ground at any
// height is a pure translation every tier can represent.
inline bool is_level(const GroundContact& g) {
  for (const GroundSample& s : g.wheel) {
    if (s.normal.x != 0.0 || s.normal.y != 0.0 || !(s.normal.z > 0.0)) {
      return false;
    }
    if (s.mu_scale != 1.0) return false;
  }
  return true;
}

// Returns nullptr if the samples are usable, or a static message.
inline const char* validate(const GroundContact& g) {
  for (const GroundSample& s : g.wheel) {
    if (!std::isfinite(s.height)) return "ground height must be finite [m]";
    if (!std::isfinite(s.normal.x) || !std::isfinite(s.normal.y) ||
        !std::isfinite(s.normal.z)) {
      return "ground normal must be finite";
    }
    if (!(s.normal.z > 0.0)) return "ground normal must point up (z > 0)";
    if (!std::isfinite(s.mu_scale) || !(s.mu_scale >= 0.0)) {
      return "ground mu_scale must be finite and not negative [-]";
    }
  }
  return nullptr;
}

// Mean contact height, axle pairs first so the sum is mirror symmetric.
inline double mean_height(const GroundContact& g) {
  return 0.25 * ((g.wheel[kFrontLeft].height + g.wheel[kFrontRight].height) +
                 (g.wheel[kRearLeft].height + g.wheel[kRearRight].height));
}

}  // namespace slipx

#endif  // SLIPX_GROUND_HPP
