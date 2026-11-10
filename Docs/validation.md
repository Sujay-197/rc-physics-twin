# Cross-check of the Core against independent vehicle models

**Question.** Is the physics Core only self-consistent, or does it agree with vehicle dynamics that someone else wrote down?

**Answer, in one line.** Within the linear range the twin matches closed-form single-track theory to 0.05 % at L1 and 0.6 % at L2 in steady state. The L2 transient matches the same theory with a tyre relaxation length added to within 1 ms of rise time and 0.5 points of overshoot. Driven with the parameters CommonRoad can represent, the twin agrees with the independently written CommonRoad models to the same level. At the limit, L2 and CommonRoad's Pacejka model agree within 2 %. Every disagreement larger than that has a stated physical or mapping cause.

**What this is not.** It is model against model. There is no car: every parameter is `provisional` (SlipX's generic 1/10 car), and agreement here says the equations are implemented as the references implement them, not that they describe a real car. That needs the coast-down, skidpad and step-steer tests on hardware in [`NEXT_STEPS.md`](NEXT_STEPS.md).

Reproduce (about 95 s on one core; the venv is described in [`sim_py/requirements-validation.txt`](../sim_py/requirements-validation.txt)):

```bash
.venv-validation/Scripts/python tools/cross_check.py
```

Outputs: [`results/reference/cross_check.json`](../results/reference/cross_check.json) (every number below, the parameters, the mapping, the settings, the commit and the package versions) and two figures, `results/reference/cross_check_steady.{pdf,png}` and `results/reference/cross_check_transient.{pdf,png}`, sized for a single-column LNCS page (122 mm). Two runs of the same build give byte-identical files. The numbers below are from a run at commit `59457e0` (Core: SlipX 0.2.0 through `rc_twin` ABI 8; commonroad-vehicle-models 3.0.2, numpy 2.5.4, scipy 1.18.1, Python 3.14.6, Windows 11).

## Key numbers

| Check | Twin | Reference | Agreement |
|---|---|---|---|
| Steady yaw-rate gain, 1 to 10 m/s, δ = 0.01 rad | L1 / L2 / Unity car | linear theory | max error 0.05 % / 0.52 % / 0.59 % |
| Same, equal axle stiffness | L1 / L2 | CommonRoad ST / STD | max error 0.06 % / 0.06 % (L1 against STD and L2 against ST: 0.4 %) |
| Same, provisional car | L2 | CommonRoad ST | up to −9.6 % at 10 m/s: a mapping limitation (below) |
| Steady circle at 4 m/s, a_y < 4 m/s², yaw rate and a_y | L1 / L2 / Unity car | linear theory | max error 0.04 % / 0.05 % / 0.20 % |
| Steady circle at 4 m/s, every steady point to the limit | L2, equal stiffness / L2 | CommonRoad STD | max error 1.9 % / 1.7 % in a_y |
| Peak steady a_y at 4 m/s | L2: 9.92 m/s² | STD: 10.09 m/s² | −1.7 %, explained by load transfer |
| Yaw natural frequency and damping at 5 m/s, identified from the step | L1: 67.4 rad/s, 1.035 | closed form: 67.7 rad/s, 1.031 | −0.4 %, +0.4 % |
| Step steer, 5 m/s, 0.03 rad: 10 to 90 % rise time | L1: 24.9 ms; L2: 41.7 ms | linear: 24.7 ms; linear + relaxation: 40.8 ms | 0.2 ms; 0.9 ms |
| Same: yaw-rate overshoot | L2: 9.2 % | linear + relaxation: 9.7 % | 0.5 points |
| Same: path deviation 3 s after the step (15 m driven) | L1 / L2 | linear / linear + relaxation | 1.6 cm / 2.1 cm (0.1 % of the path) |
| Sine steer, 5 m/s, 0.03 rad at 1 Hz: yaw gain and phase | L1 / L2 | linear / linear + relaxation | −0.04 %, −0.18° / −0.06 %, +0.05° |
| Full lock at 0.2 m/s: steady yaw rate | Unity car (blend on) | kinematic and CommonRoad KS | 0.000 %; path 6 mm apart after 3 s |
| Same | L1, L2 (blend off) | kinematic | +150 %: the `v_eps` floor (ADR 0011) |

## What is compared

### The twin

All through `rc_twin` (ABI 8) with SlipX's provisional defaults (`rc_twin_params_default`), RK4, 1 ms steps, level ground, no walls:

| Name | Tier and options |
|---|---|
| L0 | kinematic bicycle |
| L1 | dynamic bicycle, linear tyres clipped at `mu_clip`, no steering servo |
| L2 | double track: MF-lite tyres with load sensitivity, quasi-static load transfer, relaxation length, RWD with an open differential, ESC and battery, steering servo; parallel steer, quasi-static wheels, no suspension, no kinematic blend |
| L2, Unity car | L2 with `TwinCar`'s options (`sim_py/rc_twin_env.py`, `UNITY_CAR`): wheel inertia 1e-5 kg m², ideal Ackermann, suspension at 3 Hz with damping ratio 0.3 and 15 mm travel, kinematic blend at and below 0.3 m/s |
| ... equal stiffness | the same with both axles at the mean cornering stiffness, 437.5 N/rad, the only split CommonRoad can represent (below) |

Nothing in `core/` was changed. The Core's defaults are untouched; the equal-stiffness variant is set through `rc_twin_params` like any other car.

### The references

1. **Linear single-track theory**, closed form (Rajamani 2012, ch. 2 and 3; Pacejka 2012, ch. 1). Two states (v_y, r), constant speed, small angles, linear tyres F_y = −C_α α. It gives the steady yaw-rate gain r/δ = v / (L + K_us v²) with the understeer gradient K_us = (m / L)(l_r / C_αf − l_f / C_αr), and the natural frequency and damping of the yaw response from the system matrix. Transients are its exact response (`scipy.signal.lsim`) to the twin's achieved steer. **Linear + relaxation** adds a first-order lag on each axle's force with time constant σ / v, σ the twin's relaxation length (0.08 m). That is the transient tyre model L2 uses (SlipX `relaxation.hpp`; Pacejka 2012).
2. **CommonRoad vehicle models** (Althoff and Würsching 2020; Althoff et al. 2017), Python package [`commonroad-vehicle-models`](https://pypi.org/project/commonroad-vehicle-models/) 3.0.2, BSD-3-Clause (the licence file in the installed package, Technical University of Munich). It is installed, not vendored. The package's own dynamics functions are called unchanged; `tools/cross_check.py` only maps the parameters and integrates them (RK4, 1 ms; finer for STD's stiff wheel states, below).
   - **KS**, kinematic single track, reference point the rear axle (converted to the CoG for comparison).
   - **ST**, single track with linear tyres and longitudinal load transfer, linearised in the slip and steer angles. It switches to a kinematic model below 0.1 m/s.
   - **STD**, single-track drift: Pacejka (PAC2002-form) tyres with combined slip, front and rear wheel-speed states, and a blend into the kinematic model below about 0.2 m/s.
   - **MB**, the multi-body model, is **not** run. It needs about 40 further parameters (sprung and unsprung masses, roll-axis heights, suspension, anti-roll and tyre compliances). The twin does not have them, the package's parameter sets are full-size cars, and no cited source gives them at 1/10 scale. Taking them from a full-size car would be inventing them.
3. **The kinematic relation** r = v_x tan δ / L at the CoG (Rajamani 2012, sec. 2.2), for the low-speed case.

## Parameter mapping

The twin's provisional parameters, as `rc_twin_params_default` returns them, and where each one goes. The JSON's `twin_params` and `commonroad_mapping` blocks hold the full lists.

| Twin (SlipX) | Value | Linear theory | CommonRoad | Notes |
|---|---|---|---|---|
| `mass` | 3.5 kg | m | `m` | |
| `izz` | 0.05 kg m² | I_z | `I_z` | |
| `lf`, `lr` | 0.16, 0.16 m | l_f, l_r | `a`, `b` | |
| `h_cog` | 0.06 m | not used | `h_s` (and `h_cg`) | ST and STD use it for longitudinal load transfer only |
| `c_alpha_f`, `c_alpha_r` | 420, 455 N/rad per axle | C_αf, C_αr | `p_ky1 = −C_S p_dy1` | **Not mappable one-to-one**: see below |
| `tyre_*.mu_y0` | 1.10 | not used | `p_dy1` | STD's peak lateral friction. ST uses only the product C_S μ |
| `tyre_*.shape_c`, `curvature_e` | 1.68, 0.42 | not used | `p_cy1`, `p_ey1` | MF-lite's C and E map onto PAC2002's directly. MF-lite derives B from C_α, as PAC2002 derives B_y from K_y |
| `tyre_*.k_mu` | 0.15 | not used | none | PAC2002's pure-lateral D_y in the package is linear in load. The twin's load sensitivity has no counterpart: see the limit results |
| `tyre_*.relax_length` | 0.08 m | σ (linear + relaxation only) | none | CommonRoad's tyres are instantaneous |
| `c_kappa` | 120 N per unit slip, per tyre | not used | `p_kx1 = 2 c_κ / F_z,axle` = 13.98 | STD only. `p_dx1` = `mu_x0` = 1.2; `p_cx1`, `p_ex1` take MF-lite's C and E |
| `wheel_radius` | 0.05 m | not used | `R_w` | STD only |
| `wheel_inertia` | 0 (L2), 1e-5 kg m² (Unity car) | not used | `I_y_w` = 2e-5 kg m² per axle | STD needs a positive wheel inertia, so it takes the Unity car's value for both comparisons |
| `layout` RWD; brakes on the driven axle | | not used | `T_se` = 0, `T_sb` = 0 | STD only |
| `steer_max` | 0.40 rad | | limits opened | The input is the twin's achieved steer, already within travel |
| shifts (none in MF-lite) | 0 | | `p_hy1`, `p_vy1`, `p_hx1`, `p_vx1`, `r_hx1`, `r_hy1`, `r_vy1`, `r_vy3` = 0 | |
| combined-slip shape (MF-lite uses a friction ellipse instead) | | | `r_bx*`, `r_cx1`, `r_ex1`, `r_by*`, `r_cy1`, `r_ey1`: CommonRoad's defaults | No counterpart in the twin. Here they only meet the small drive slip needed to hold speed |
| g | 9.80665 m/s² | | 9.81 (hard-coded in ST and STD) | C_S is mapped with 9.81 so the axle stiffness is exact; the 0.03 % difference in weight is left |

**The one mapping that does not go through.** CommonRoad 3.0.2's ST and STD derive one normalised cornering stiffness for both axles, `C_S = −p_ky1 / p_dy1` (`vehicle_dynamics_st.py`). An axle's stiffness is then C_S μ F_z,axle. The twin's car has equal axle loads (l_f = l_r) and unequal stiffnesses, 420 and 455 N/rad, so it understeers slightly: K_us = 3.21e-4 rad/(m/s²), characteristic speed 31.6 m/s. No single C_S gives both. The mapping keeps the total, C_S μ m g = C_αf + C_αr, so each CommonRoad axle has 437.5 N/rad and the car is exactly neutral steer. Rather than tune anything, the twin is also run with that same equal split (the "equal stiffness" rows). That separates the mapping's effect, which linear theory predicts exactly, from any disagreement between the models themselves.

## Method

Every model starts at the origin, heading along x, at the manoeuvre's speed, with no yaw or lateral velocity. The twin's L2 also starts with its wheels rolling, with soc 0.8. Each model holds its own body-x speed with the same PI controller (k_p = 4 /s, k_i = 4 /s², 100 Hz). The twin's L1 and L2 must push against drag, rolling resistance and the tyres' induced drag; CommonRoad's ST takes acceleration as its input and needs none. Steering is the **twin's achieved road-wheel angle**, recorded every 1 ms and fed to every reference: to CommonRoad through its steering-rate input, to linear theory as its input signal. L2's servo (45 rad/s, damping 0.7, 10 rad/s slew) therefore shapes all the curves it is compared with, and is not counted as a chassis disagreement. L1 and L0 have no servo. With the Unity car's Ackermann on, the achieved angle is that of the virtual centre wheel.

| Manoeuvre | Settings | Metrics |
|---|---|---|
| Steady circle, speed sweep | δ = 0.01 rad (ramped over 1 s, held 3 s), 1 to 10 m/s | yaw-rate gain r/δ averaged over the last second |
| Steady circle, understeer curve (ISO 4138 constant speed, variable steer) | 4 m/s, δ from 0.01 to 0.40 rad, 17 points | steady r and a_y = v_x r. A point is steady if r varies by under 1 % over the last second |
| Step steer (after ISO 7401) | 5 m/s, steps of 0.03 rad (a_y about 2.3 m/s², linear) and 0.10 rad (about 7.5 m/s², nonlinear), 1.5 s of straight running first | steady yaw rate; 10 to 90 % rise time; overshoot; response time from 50 % steer to 90 % yaw rate; path deviation 3 s after the step |
| Sine steer | 5 m/s, 0.03 rad at 1 Hz, three periods | yaw-rate gain and phase lag fitted over the last two periods; RMS yaw-rate difference; path deviation at the end |
| Low speed, full lock | 0.2 and 0.4 m/s, δ = 0.40 rad, 10 s | steady yaw rate and radius; path deviation after 3 s |

Path deviation is the distance between the two CoG positions after N seconds, each measured in the frame of its own pose at the start of the manoeuvre, so speed lost during the straight run does not count.

## Results

### Steady state

**Linear range.** The twin reproduces closed-form linear single-track theory. Over the speed sweep the yaw-rate gain error is at most 0.05 % at L1. The residual is what L1 keeps and the theory linearises away (`atan` slip angles, the `cos δ` force projection, induced drag), plus the speed controller's small error. It is 0.52 % at L2 and 0.59 % for the Unity car, both at 10 m/s (a_y = 3.1 m/s²), and 0.06 % or less up to 6 m/s. The L2 residual grows with lateral acceleration, as it should: MF-lite's curve falls below its tangent as slip grows, and load transfer lowers both axles' stiffness through load sensitivity. On the understeer curve at 4 m/s and below a_y = 4 m/s², yaw rate and lateral acceleration agree with theory within 0.04 % (L1), 0.05 % (L2) and 0.20 % (Unity car).

**Against CommonRoad, provisional car.** The gain differs from ST and STD by −0.16 % at 1 m/s, growing to −2.5 % at 5 m/s and −9.6 % at 10 m/s (L2). This is the mapping, not the dynamics. Linear theory predicts the ratio of the twin's gain to CommonRoad's neutral-steer gain as 1 / (1 + v² / v_ch²), with v_ch = 31.6 m/s: −2.4 % at 5 m/s and −9.1 % at 10 m/s. Figure 1b shows it: the twin and linear theory fall away from the neutral-steer line together.

**Against CommonRoad, equal stiffness.** With the same split on both sides the gain error is at most 0.06 % between the models with the same kind of tyre: L1 against ST (linear) and L2 against STD (Magic Formula). Across the pairs, L1 against STD and L2 against ST, it reaches 0.4 % at 10 m/s, where the nonlinear tyre starts to soften. In the understeer curve's linear range it is at most 0.07 % against ST and 0.24 % against STD.

**At the limit** (Figure 1a). Linear theory and ST have no saturation and keep rising (ST: 20.1 m/s² at 0.40 rad). The twin's L1 clips: it reaches 10.53 m/s² at 0.22 rad, where the front axle meets `mu_clip` F_z. Further steer only tilts the clipped force, so a_y falls slowly and there is no falling branch, as SlipX's L1 documents. L2 and STD both have a Magic Formula with a falling branch, and they agree through the peak and past it. Over all 17 steady points the a_y error is at most 1.7 % (L2) and 1.9 % (L2, equal stiffness). The peaks are 9.92 m/s² for L2 against 10.09 m/s² for STD, both at δ = 0.24 rad, then both fall to about 8.4 m/s² at 0.40 rad. The 1.7 % peak deficit is close to the 1.8 % that the twin's lateral load transfer and load sensitivity predict, and STD, a single-track model, has neither:
  - At 10 m/s² an axle carrying 1.75 kg transfers m_axle a_y h / t = 4.4 N from the inner to the outer tyre (static 8.58 N each).
  - MF-lite's peak force goes as F_z^(1 − k_mu) with k_mu = 0.15, so the axle's peak becomes (12.96^0.85 + 4.21^0.85) / (2 × 8.58^0.85) = 0.982 of its static value, a 1.8 % loss.
  - The Unity car's Ackermann and suspension move its curve by up to 3 % from the bare L2's at the last points past the peak (3.1 % max against STD).

### Transients

**Natural frequency and damping.** Closed-form theory gives ω_n = 67.7 rad/s (10.8 Hz) and ζ = 1.031 at 5 m/s. The yaw mode of this car is about critically damped at every speed in range: ζ 1.043 at 1 m/s, 0.995 at 10 m/s (the `analytic` block of the JSON). The yaw rate alone cannot separate ω_n from ζ here, because with ζ near 1 a zero nearly cancels a pole. The script therefore regresses the full two-state model on the twin's step response, and the twin's L1 gives ω_n = 67.4 rad/s and ζ = 1.035 (−0.4 %, +0.4 %). The same regression on ST gives 67.0 rad/s and 1.043 against the equal-stiffness closed form of 66.9 rad/s and 1.043. It is a check of the method, not a separate claim.

**Step steer, 0.03 rad** (Figure 2a). L1 has instantaneous steering and tyres, and is the linear model: rise time 24.9 ms against 24.7 ms, no overshoot, steady yaw rate −0.01 %, 1.6 cm apart after 3 s and 15 m. L2 is slower and overshoots, for two reasons, both of which the references reproduce when given them:
  - **The servo.** Its own step overshoots, and fed L2's achieved steer, linear theory, ST and STD all show 3.7 to 3.8 % overshoot and a 53 to 54 ms rise.
  - **The relaxation length.** The tyre's force lags its slip angle by σ / v = 16 ms at 5 m/s, comparable with the yaw mode's own time scale (1 / ω_n = 15 ms). Adding exactly that lag to linear theory gives a 40.8 ms rise and 9.7 % overshoot, against L2's 41.7 ms and 9.2 %, and paths 2.1 cm apart after 3 s.
  
  CommonRoad's tyres have no relaxation, so ST and STD rise 12 ms later than L2 and overshoot 5.5 points less. That is a modelling difference, not an error on either side.

**Step steer, 0.10 rad** (a_y about 7.5 m/s², 0.77 g). L2's steady yaw rate is 1.2 % below linear theory and 2.0 % below STD; with equal stiffness, 0.8 % below linear theory and 0.9 % above STD. The tyres are now well into their nonlinear range. STD's rise slows to 61 ms (its PAC2002 tyre softens with slip) and L2 settles between STD and the linear model. With equal stiffness, L2's steady yaw rate settles between STD's and the linear model's. Paths after 3 s: L2 and STD 15 cm apart with equal stiffness (1.0 % of 15 m), 38 cm with the provisional split.

**Sine steer, 1 Hz.** At 1 Hz the yaw mode is far below its 10.8 Hz natural frequency, so all models track closely. L1 against linear theory: gain −0.04 %, phase −0.18°, 1.6 cm after three periods. L2 against linear + relaxation: gain −0.06 %, phase +0.05°. Against ST and STD the provisional car's gain is 1.8 to 2.5 % lower, the neutral-steer mapping again; with equal stiffness, within 0.05 % (L1) and 0.75 % (L2, which has the relaxation they lack).

### Low speed: the kinematic blend (ADR 0011)

At full lock (0.40 rad) and 0.2 m/s, the kinematic yaw rate is v tan δ / L = 0.264 rad/s, a CoG radius of 0.774 m (Figure 2b).

| Model | Steady yaw rate | Radius | Against kinematic |
|---|---|---|---|
| Twin L0 | 0.2642 rad/s | 0.774 m | 0.000 % |
| Twin L2, Unity car (blend on, kinematic at and below 0.3 m/s) | 0.2642 rad/s | 0.774 m | 0.000 %; 6 mm from CommonRoad KS after 3 s |
| CommonRoad KS | 0.2642 rad/s | 0.774 m | 0.000 % |
| CommonRoad ST | 0.2551 rad/s | 0.800 m | −3.4 %: ST is linearised in δ, and δ = 0.40 is not small (tan 0.40 / 0.40 = 1.057), partly offset by ST's speed state being the CoG speed rather than v_x |
| Twin L1 and L2, blend off | 0.660 rad/s | 0.342 m | +150 %: the slip-angle floor sets the turn to v_eps tan δ / L = 0.661 rad/s whatever the speed |

So the twin's L0 and its blended L2 reproduce an independently written kinematic model exactly at creeping speed, and the floor defect that ADR 0011 describes is the twin's own (L1 and the bare L2), not physics: CommonRoad's dynamic ST, which has no floor, stays within 3.4 % of kinematic there.

At 0.4 m/s the Unity car is inside its blend window (λ = 0.5). Its yaw rate is 0.701 rad/s, 33 % above kinematic and 6 % above the floored bare L2 (0.660). ADR 0011 states this limitation: inside the window the dynamic model still dominates. A wider window would hand more of it to the kinematic model.

## For the paper

> We cross-checked the Core against closed-form linear single-track theory and the independently implemented CommonRoad vehicle models (KS, ST and STD) on steady-state circular driving, step and sine steer, and full-lock creeping, with identical parameters, initial state and steering input. In the linear range the steady yaw-rate gain agrees with theory to 0.05 % (L1) and 0.6 % (L2) from 1 to 10 m/s. L2's step response matches linear theory with the same tyre relaxation length to within 1 ms of rise time, giving trajectories 2 cm apart after 15 m. At the limit, L2 and CommonRoad's Pacejka model agree to within 2 % in lateral acceleration, a gap explained by lateral load transfer, which the single-track reference lacks.

The honest limitation, in one sentence: this is model-to-model agreement with provisional parameters, so it shows the equations are implemented consistently with independent references, not that they predict a real car.

## Limitations

- **Model against model, not model against car.** No parameter is measured. A shared assumption (the Magic Formula's form, a quasi-static load transfer, a first-order relaxation) would agree with itself here and still be wrong on hardware.
- **CommonRoad cannot represent unequal normalised axle stiffness** (3.0.2), so the provisional car is compared through an equal-split variant, and the provisional comparison carries the predicted neutral-steer offset. The equal-split twin is a different car, though only by a cornering stiffness split.
- **STD borrows** the Unity car's wheel inertia and CommonRoad's default combined-slip coefficients, which the twin has no counterpart for. Here they only act on the drive slip needed to hold speed. That influence was not measured separately.
- **Not exercised:** braking or accelerating in a turn (combined slip), grades and banks, the suspension's own dynamics (the Unity car's 3 Hz body modes are inside its curves but not checked separately), reversing, and the battery and ESC. MB was not run (above).
- **Single operating points.** The transients are at 5 m/s and the understeer curve at 4 m/s. The speed sweep covers 1 to 10 m/s at one small steer.
- **No independent second-order tyre-lag reference.** "Linear + relaxation" uses the same first-order lag law as SlipX (`relaxation.hpp`), written independently in this script. It confirms the implementation, not the law.

## References

- Althoff, M., Würsching, G.: CommonRoad: Vehicle Models (version 2020a). Technical report, Technical University of Munich (2020). https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models (the citation the package's source and parameter files give).
- Althoff, M., Koschi, M., Manzinger, S.: CommonRoad: Composable benchmarks for motion planning on roads. In: IEEE Intelligent Vehicles Symposium (IV), pp. 719–726 (2017).
- commonroad-vehicle-models 3.0.2, Python package, BSD-3-Clause. https://pypi.org/project/commonroad-vehicle-models/
- Pacejka, H.B.: Tire and Vehicle Dynamics, 3rd edn. Butterworth-Heinemann, Oxford (2012). Ch. 1: single-track handling, understeer gradient, characteristic speed.
- Rajamani, R.: Vehicle Dynamics and Control, 2nd edn. Springer, New York (2012). Ch. 2: kinematic and dynamic bicycle models; ch. 3: steady-state cornering.
- ISO 4138:2021, Passenger cars: Steady-state circular driving behaviour, open-loop test methods.
- ISO 7401:2011, Road vehicles: Lateral transient response test methods, open-loop test methods.
- Kabzan, J. et al.: AMZ Driverless: The full autonomous racing system. Journal of Field Robotics 37(7), 1267–1294 (2020). The kinematic blend of ADR 0011.
- SlipX (the Core), Apache-2.0: `core/src/core/slipx_core/src/l1_bicycle.cpp`, `l2_double_track.cpp`, `include/slipx/tyre.hpp`, `relaxation.hpp`, `load_transfer.hpp`.
