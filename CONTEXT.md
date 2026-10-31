# Domain glossary

Pinned vocabulary so docs, code and issues stay consistent.

| Term | Meaning |
|---|---|
| **Core** | The deterministic vehicle-dynamics library (forked SlipX, C++17). Knows nothing about Unity or roads beyond a ground-query callback. |
| **Ground query** | Callback `f(x, y) -> (height, normal, friction_scale, surface_id)` supplied by the host (Unity or a test harness). |
| **Tier** | Fidelity level of the vehicle model (L0 kinematic, L1 bicycle, L2 double-track, L3 adds suspension/thermal). |
| **Contact patch** | One of four tire-road interfaces; receives load, slip and friction. |
| **Road feature** | Annotated property of a road edge: grade, speed breaker, surface, curvature, banking. |
| **Edge profile** | Cached result of simulating the car over one road edge: traversal time, energy, peak load, speed trace summary. |
| **Edge profiler** | Batch job that produces edge profiles by running the Core headlessly. |
| **SoC** | Battery state of charge, 0 to 1. |
| **Provenance label** | `measured`, `identified` or `provisional` tag on every parameter set. |
| **Oracle** | Independent model (gymkhana single-track drift) to cross-check the Core in planar scenarios. Planned, not yet in the repo. |
| **Fleet sim** | The separate demand/dispatch simulation (`campus_fleet_man_unity`). Consumes edge profiles, never runs the Core in its hot loop. |
| **Manual control** | Keyboard/gamepad writing into the same action vector the RL policy uses. |
