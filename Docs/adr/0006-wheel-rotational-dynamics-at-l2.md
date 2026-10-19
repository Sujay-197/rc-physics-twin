# ADR 0006: Wheel rotational dynamics at L2, opt-in, with an implicit wheel update

**Status:** Accepted  **Date:** 2026-10-19
**Related:** SlipX ADR-0027 (`core/docs/adr/0027-l2-closes-its-algebraic-loops-without-iterating.md`), SlipX ADR-0031, our ADR 0005

## Context

The spec (4.5) wants per-wheel spin so braking lock-up and wheelspin are representable:

    I_w dw/dt = T_drive - T_brake - R F_x

SlipX L2 has no wheel state on purpose. Its ADR-0027 closes the longitudinal loop quasi-statically and names the price: "L2 cannot represent a locked or a spinning wheel". It also sets the bar for reversing that: an answer to the step-size problem, not a restatement. The problem is real. SlipX's ADR-0027 puts a 1/10-scale wheel's inertia at order 1e-5 kg m². With that, SlipX's default slip stiffness of 120 N per unit slip and R = 0.05 m, the wheel-speed mode has a time constant of `I V / (R^2 C_kappa)`: 0.033 ms at 1 m/s, thirty times shorter than the 1 ms step. Even ten times the inertia (a heavy wheel plus reflected motor inertia) gives 0.33 ms. An explicit integrator, RK4 included, diverges.

Options:

- **Sub-step the wheels inside the step.** Works but the sub-step count becomes a stability-driven tuning knob tied to the parameters, and still explicit.
- **Add a longitudinal relaxation length and keep the force lagged.** Softens the stiffness but still needs wheel inertia for lock-up, and adds an unidentified parameter.
- **Split the step: chassis by RK4 with wheel speeds held, then wheels by a linearly implicit update.** Chosen.

## Decision

1. **Opt-in through one parameter.** `VehicleParams::wheel_inertia` [kg m²], per wheel, including the drivetrain inertia reflected to it. `0` (the default) is SlipX's quasi-static L2 exactly, so every existing test and hash is unchanged. `> 0` makes `omega_w` integrated state. L0/L1 ignore it (they have no wheels); `validate()` requires it finite and not negative.
2. **Chassis.** RK4 (or semi-implicit Euler) over the step as before, with each wheel's **slip velocity** `omega R - v_long` held at its entry value. Per derivative evaluation, each tyre's longitudinal force is `c_kappa(Fz) * kappa`, `kappa = slip / max(|v_long|, v_eps)`, through the same friction ellipse as the lateral force. `v_long` is the wheel-centre velocity along the wheel's heading. Holding the slip rather than the wheel speed matters: a rolling wheel keeps its slip while the car speeds up within the step, whereas a held wheel speed lags the road by a whole step's change in speed. With a stiff tyre that lag cost about 5 % of the drive force and put a phantom drag on free-rolling wheels; holding the slip removed both (a very light wheel now matches SlipX's quasi-static tier to 0.04 % in speed over 2 s). The differential no longer splits a force demand; it splits torque (below).
3. **Wheels, after the chassis step,** at the end-of-step loads and lateral forces, by backward Euler. For one undriven wheel, in closed form:

       a = I / dt
       omega' = (a omega + T + R (C/V) v_long) / (a + R^2 C / V)

   The driven wheels are solved together, because the motor couples them: on its torque-speed curve the ESC's torque falls linearly with the driven wheels' mean speed (the motor's back-EMF damping), `T = A - B omega_mean`, and with a light wheel that is the only thing setting the spin speed. Each driven degree of freedom (a wheel, or a locked pair) then obeys `n a (omega' - omega) = share (A - B omega_mean') + extra - R F(omega')`: a diagonal system plus one rank-one coupling, solved in closed form by Sherman-Morrison. Off the curve `B = 0` and the wheels decouple. Saturation is handled in a fixed two passes: linear tyres first, then any tyre that the first pass found saturated is frozen at that force (where it barely depends on the wheel speed) and the system solved again. If a saturated wheel's explicit update would carry it back past the tyre's peak, it is set at the peak instead: the force is continuous there and that is the implicit answer. (Without this, a light wheel near the peak overshot to the far side every step and its force flipped between drive and brake; full throttle into a corner made the car undrivable in the twin. A test now drives exactly that.) No iteration to a tolerance, so the operation count does not depend on the data (SlipX ADR-0027's determinism argument holds). Sums run per axle and then across axles, so left/right mirror symmetry is exact.
4. **Torque, not force, through the differential.** The ESC's per-step wheel torque goes to the driven axle(s) by the drive layout. Open: equal torque each side, each wheel solved on its own, so the unloaded wheel spins. Spool: one shared speed, the two wheels solved together with inertia `2 I`. LSD: the spool solution if the torque it needs across the clutch is within the preload, otherwise open with the preload biased to the slower wheel, the same rule SlipX uses for forces. Undriven wheels get zero torque and roll freely.
5. **Lock-up.** Braking is motor braking (a negative ESC torque off the curve). It acts against the wheel's rotation whichever way the wheel turns, the direction being the one the tyre alone would turn it this step, and it stops a wheel rather than reversing it: a braked wheel that would cross zero stops at zero and stays locked while the brake exceeds what the tyre returns. A locked wheel slides at `kappa = -1`, saturated. (Applying the brake torque with a fixed sign was the first version; a car spun round onto its back wheels then had its brake drive them backwards without limit. A test now holds that case.)

## Consequences

- **Lock-up and wheelspin are dynamic events** when `wheel_inertia > 0`: the slip ratio can run to -1 or well above 1, and `omega_w` is integrated, not derived. The ESC sees the real wheel speed, so a spinning wheel runs up the torque-speed curve and loses torque.
- **First-order coupling.** Holding the slip over the chassis step and updating the wheels after is a Lie splitting, first order in dt. Against SlipX's quasi-static tier in its own regime (a very light wheel, inside the friction ellipse) it agrees to 0.04 %; it has not been measured against a converged reference.
- **Verified by:** `core/.../tests/test_wheels.cpp`, 19 tests: stable at 1 kHz for inertias from 1e-8 to 1e-2 with both integrators; the closed-form coast-down `m / (m + 4 I / R^2)` for open, spool and LSD; a locked wheel spinning up at the friction limit; wheelspin; lock-up and release; braking a car rolling backwards; a light spinning wheel settling on the motor curve; the three differentials; mirror symmetry bit for bit; determinism. 14 planted mutations, all caught; three escaped the first pass and became tests.
- **The default car has no wheel dynamics** (inertia 0) because SlipX's defaults must stay SlipX's. Hosts that want spin set a value; the Unity twin uses 1e-5 kg m², SlipX ADR-0027's order of magnitude, labelled provisional where it is set. It is identifiable from a lifted-wheel spin-down or a step-torque wheelspin test.
- **Limits.** One inertia for all four wheels, though driven wheels also carry reflected motor inertia. The longitudinal force law is linear up to the friction ellipse: no falling branch past peak slip, so a locked tyre slides at its peak rather than a lower sliding friction. Forward motion only, as for the rest of L2.
- This changes vendored SlipX code, recorded in `THIRD_PARTY.md`. It answers the question ADR-0027 leaves open and could be offered upstream once it has settled.
