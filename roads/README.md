# Roads

Road-feature schema and sample data (Phase 3). See `Docs/specs/2026-10-19-physics-twin-design.md` section 5.

Planned format: one JSON/YAML file per edge or per graph, with per-polyline-point elevation and banking, a list of speed breakers (`s`, `height`, `length`, `profile`), and a `surface_id` per segment.

Elevation pattern reference: ApexSim track CSV (`x, y, elevation, banking`).
