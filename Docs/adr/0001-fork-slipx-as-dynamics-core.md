# ADR 0001: Fork SlipX as the dynamics Core

**Status:** Proposed  **Date:** 2026-10-19

## Context

We need slow, accurate, deterministic 1/10-scale vehicle dynamics with an electric powertrain. Writing a double-track model, tire model, drivetrain and battery from scratch costs weeks and is easy to get subtly wrong.

## Decision

Fork **SlipX** (Apache-2.0, C++17) and extend it.

Read from its PyPI page (not yet checked against source): its double-track tier has four contact patches, per-corner vertical loads, MF-lite tires with a falling branch, a combined-slip ellipse, tire relaxation, a drivetrain with ESC torque-speed curve, current and regen limits, battery sag and state of charge, and a rate-limited servo. The core depends only on the C++ standard library and is built for bit-identical replay on one build.

## Gaps we will fill

- Ackermann steering (upstream uses parallel steer)
- Wheel rotational state (upstream has none)
- Suspension tier (upstream L3 raises "not implemented")
- Road-aware ground queries: grade, speed breakers, surface friction (upstream scene layer is planned, not built)

## Alternatives considered

| Option | Why not |
|---|---|
| Gym-Khana as the core | Planar 2D, no slope or bumps, Python speed |
| Unity WheelCollider / TLabVehiclePhysics | Engine-coupled, tire model quality unverified, no energy model found |
| MuJoCo + custom tires | More work for the same result; weaker determinism story |
| From scratch | Weeks of risk for no upside |

## Consequences

- Positive: faster start, determinism and provenance discipline inherited.
- Negative: we own an alpha-stage codebase with no empirical validation; upstream API may break before 1.0.
- Obligations: Apache-2.0 notices, mark modified files (see `THIRD_PARTY.md`).
- Mitigation: pin an upstream commit, keep extensions in separable modules, upstream PRs where sensible.
