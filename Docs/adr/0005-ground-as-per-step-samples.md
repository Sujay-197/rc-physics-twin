# ADR 0005: Ground reaches the Core as per-step samples, not a callback

**Status:** Accepted  **Date:** 2026-10-19

## Context

Phase 2 needs the Core to feel the road: grade, banking and surface grip per contact patch. The spec (section 5) describes a ground query called per contact patch per step. SlipX's `VehicleModel::step` is `const` and reads nothing outside its arguments (SlipX ADR-0004); that purity is what makes replay bit-identical, and a callback from inside the integrator into host code would break it.

## Decision

- The host samples the ground under each wheel at the start of a step and passes the four samples in: `VehicleModel::step_on(state, input, GroundContact, dt, diag)` (`core/.../slipx/ground.hpp`). Each sample is height, outward normal and `mu_scale`. Samples are held for the step, the same quasi-static treatment SlipX gives the ESC budget; a wheel moves at most a couple of centimetres in 1 ms.
- The **C ABI** keeps the spec's callback shape for hosts (`rc_twin_set_ground_query`): `rc_twin` computes the four contact points from the entry state, calls the host once per wheel, and hands the samples to `step_on`. The callback runs outside the integrator, so the Core stays pure.
- **L2** uses the samples as follows: `mu_scale` multiplies that tyre's friction budget; the four normals are averaged into one road plane; gravity along the plane is added to the body-frame accelerations, and `g n_z` replaces `g` in the loads and rolling resistance; `pos.z` is the mean contact height. Tyre nominal loads stay at their level-ground values.
- **L0 and L1** accept level, unscaled ground only, and refuse anything else (`step_on` returns false, state untouched). They never silently ignore a grade, matching SlipX ADR-0005's "no fallback" rule.
- Level, unscaled ground runs exactly the upstream arithmetic: the existing 216 core tests and the six conformance hashes are unchanged.

## Consequences

- Positive: grade, banking and grip work at L2 with no change to determinism, and headless hosts (CI, the edge profiler) supply samples from road data without Unity.
- Limits, stated in `l2_double_track.cpp`: still planar, so horizontal travel is overstated by 1/cos(grade) (0.5 % at 10 %); per-wheel height differences and normals beyond their mean need suspension (L3); `mu_scale` also scales cornering stiffness, which overstates the stiffness loss on slippery surfaces.
- This is a change to vendored SlipX code. It is listed in `THIRD_PARTY.md` and is a candidate to offer upstream once it has settled.
