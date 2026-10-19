# Cars

Each car is a versioned directory (layout borrowed from SlipX's car format):

```
cars/<name>/
  car.yaml          manifest
  dynamics.yaml     mass, wheelbase, track, CoG, inertia, suspension
  powertrain.yaml   motor curve, current limit, battery (V_oc(SoC), R_int, Q), efficiency map
  limits.yaml       steering range and rate, regen cap
  tyres/            (compound, surface) pairs
  provenance.yaml   label: measured | identified | provisional, with sources
```

Rules:

- Every parameter set carries a provenance label. Default is `provisional`.
- Never present a `provisional` set as validated.
- Numbers must come from a datasheet, a measurement, or a cited source. Do not invent them.

First car (to be created in Phase 1): `reference_1_10_electric`, parameters sourced from the Gym-Khana F1TENTH defaults and a named motor/battery datasheet, all labeled `provisional`.
