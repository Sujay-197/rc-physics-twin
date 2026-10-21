# Roads

One JSON file per road edge, described by [`road.schema.json`](road.schema.json) (schema 0.1.0). See `Docs/specs/2026-10-19-physics-twin-design.md` section 5.

An edge is:

- **a centreline**, a horizontal polyline in travel order, with an elevation `z` and an optional `bank` at each point (radians, positive when the road rises to the left of travel; linear between points);
- **a width**, centred on the centreline;
- **surfaces**: stretches `[from_s, to_s)` with a `mu_scale` grip multiplier (ADR 0005); later stretches win; elsewhere the grip is 1;
- **speed breakers**: round humps (`height` times sin² over `length`), centred at arc length `s`.

World frame ISO 8855, SI units. Arc length `s` is horizontal, from the first point; the first and last segments extend straight beyond the ends. **Off the road's width is not modelled**: the ground query fails there, and the car stops rather than driving on a guess.

## Using a road

- **Headless:** `tools/drive_road.py` drives the twin along a road and prints a JSON report (time, energy, peak accelerations, offset from the centreline, body pitch and roll with suspension):

  ```bash
  python tools/drive_road.py roads/samples/hill_breaker_bend.json --speed 2 --wheel-rate 311 --damping 9.9
  ```

- **From code:** `rc_twin_road_create` and friends in `native/include/rc_twin.h` build the same road from arrays; `rc_twin_road_ground` is the ground query to pass to `rc_twin_set_ground_query`.

## Samples

| File | What it has |
|---|---|
| [`samples/straight_flat.json`](samples/straight_flat.json) | 20 m level straight: a baseline |
| [`samples/hill_breaker_bend.json`](samples/hill_breaker_bend.json) | Level run-in, a 1.5 cm speed breaker, a 5 % climb, a banked left bend, a 5 % descent and a damp stretch (grip 0.6) |

The sample roads are made up to exercise the features; they are not surveyed roads. Elevation pattern reference for real tracks: the ApexSim track CSV (`x, y, elevation, banking`).
