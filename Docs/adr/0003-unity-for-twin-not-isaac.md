# ADR 0003: Build the twin in Unity (not Isaac Sim)

**Status:** Accepted (signed off by the project owner, 2026-10-19)  **Date:** 2026-10-19

## Context

The `rc_transit` design spec (2026-08-16) plans an Isaac Sim digital twin in Phase 3 and treats the physical RC car and the Unity fleet sim as separate systems. This project needs a physics-accurate twin sooner and tied to the existing fleet sim and road tiles.

## Decision

Build the twin around Unity and the existing fleet sim. Isaac Sim stays an option for camera/LiDAR realism and large-scale RL later.

## Rationale

- The fleet sim and road graph already live in Unity.
- The Core is engine-agnostic (ADR 0002), so switching rendering/RL host later costs a plugin rewrite, not a physics rewrite.
- Isaac gives better sensors and GPU RL, which are explicitly non-goals here.

## Consequences

- This deviates from the `rc_transit` spec's stated Phase 3. Update that spec or note the deviation there.
- The `IVehicleNavigator` seam, which the `rc_transit` spec deliberately leaves unexercised on hardware, will be exercised in simulation by a `PhysicsNavigator`.
- Risk: Unity 6 native-plugin and ML tooling compatibility must be proven in Phase 1.
