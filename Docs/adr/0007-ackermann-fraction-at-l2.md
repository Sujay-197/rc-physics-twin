# ADR 0007: Ackermann steering at L2 as a fraction of ideal

**Status:** Accepted  **Date:** 2026-10-19
**Related:** SlipX ADR-0009, `core/src/core/slipx_core/src/l2_double_track.cpp` header ("Parallel steer"), spec 4.4

## Context

The spec (4.4) wants Ackermann steering. SlipX L2 steers both front wheels to the same angle and says why: there is no Ackermann parameter in its schema, and inventing one would be the guess its ADR-0009 refuses (every parameter must be identifiable on the real car). The effect is real at large angles and low speed: with parallel steer the two front tyres point at different turn centres and scrub against each other, which shows up as front slip angles of opposite sign and a drag on the car.

Ackermann geometry is usually stated as a percentage of the ideal, where both front wheels point at one centre on the rear-axle line. That percentage is measurable on the car itself, statically: steer to a known angle, read both front wheel angles with a protractor or a phone level. That is a direct measurement, better than an identified one, so it meets ADR-0009's bar.

## Decision

1. **One parameter, `VehicleParams::ackermann`, the fraction of ideal Ackermann:** 0 is parallel steer (SlipX's L2 exactly, the default), 1 is ideal, negative is anti-Ackermann (some racing set-ups). `validate()` requires it finite and in [-1, 1].
2. **The steering state stays the reference angle** `delta`: the angle of a virtual centre wheel, `tan(delta) = L / R`. The servo model, the travel limit and the slip angle at L0/L1 are unchanged. At L2 each front wheel takes

       delta_ideal_left  = atan(L tan(delta) / (L - (t/2) tan(delta)))
       delta_ideal_right = atan(L tan(delta) / (L + (t/2) tan(delta)))
       delta_wheel       = delta + ackermann (delta_ideal - delta)

   with `L` the wheelbase and `t` the front track. In a left turn the left wheel is the inner one and turns further. The formulas are odd in `delta` and swap left and right exactly, so mirror symmetry holds bit for bit.
3. **Each front wheel uses its own angle everywhere the steer enters:** its slip angle, the rotation of its forces into the body frame, and (with wheel dynamics, ADR 0006) its rolling speed.
4. **Below L2 it has no effect:** L0 and L1 are single-track, and their one front wheel is the reference wheel already.
5. **`ackermann == 0` runs the old arithmetic exactly:** every existing test and hash is unchanged.

## Consequences

- At `ackermann = 1` a slow, tight turn has the two front tyres agreeing on a centre: their slip angles match where parallel steer has them opposed. Tests assert this, along with the single turn centre and mirror symmetry.
- **The Unity twin uses 1** (ideal), labelled provisional: it is a geometric reference, not a measurement of the car. Replace it with the measured fraction.
- **Limits.** A fixed fraction across the steering range; real linkages vary with angle. Steering compliance and caster effects are not modelled. The fraction is measured statically, and dynamic toe change is ignored.
