# ADR 0004: Edge profiler decouples the fleet sim from the physics

**Status:** Proposed  **Date:** 2026-10-19

## Context

The fleet sim is deterministic, headless, and runs a full simulated day in well under a second by deriving leg time from route length divided by cruise speed. Running 1 kHz physics inside it would make sweeps impractical, yet the cruise-speed shortcut ignores slope, speed breakers and energy.

## Decision

Run the Core offline, once per road edge and operating condition, and cache **traversal time and energy** as edge costs. The fleet sim reads the cache and stays fast and deterministic.

Cache key: `(edge_id, vehicle_class, payload, soc_bucket)`. Invalidate on any change to car parameters or road features.

## Consequences

- Fleet roadmap items C (time-varying edge costs) and D (heterogeneous fleet) get physics-grounded inputs.
- Energy-aware dispatch and charging constraints become possible.
- Approximation: the cache assumes the reference controller's driving style. Document the controller and record it in each cache's manifest.
- The fleet sim never links the Core.
