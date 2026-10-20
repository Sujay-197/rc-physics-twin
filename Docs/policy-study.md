# Policy study: what should drive the twin around the campus?

**Question.** Does a learned policy drive the 1/10 car round the campus better than a well-tuned classical controller? If so, which kind of learning works best?

**Short answer.** On this campus, with this car and this task:

- **Success.** Learning from scratch with a short history of past frames finishes more trips with cars it never trained on than the tuned pure pursuit. SAC + history finished 96 % with held-out cars against pure pursuit's 89 %. PPO + history, over 3 seeds, finished 94.7 %.
- **Significance.** The gain is modest. Only one learned run clears p < 0.05 on its own (sign test, same trips): PPO seed 0, with held-out cars.
- **Driving quality.** The learned policies drive worse: three to four times further off the centreline, 30 to 60 % more energy per metre, and up to 30 times rougher steering.
- **Residual RL.** Learning a correction on top of pure pursuit, the approach the literature favours for small racing cars, stayed within a few trips of pure pursuit itself.

On the stop-to-stop task the twin's autopilot now drives (see [What changed](#what-changed-because-of-this)), the controller does better still. Trained for that task, every learned policy finished fewer trips than the tuned pure pursuit with held-out cars, or the same: residual PPO. The autopilot therefore drives with the tuned pure pursuit, and a learned policy can be switched on beside it. See [What the twin uses](#what-the-twin-uses).

All numbers are in simulation, on SlipX's provisional car parameters, not a measured car. They rank methods; they do not predict the real car.

## Setup

| | |
|---|---|
| Car | SlipX L2 double-track, provisional default parameters (`core/`, `native/`), 1 ms steps, 100 Hz control |
| Place | the campus fleet sim's road network at 1/10 (`campus/`), kerbs and buildings as walls (ADR 0009) |
| Task | `sim_py/rc_twin_env.py`, task `"follow"`: drive a stop-to-stop route at 4 m/s, from 4 m/s, until past its end; trips drawn by the stops' demand weights |
| Observation | the current frame (23 values: speed, slip, wheel speeds, steer, offset and heading error, centreline 0.5, 1 and 2 m ahead, ...) and the 4 previous frames with their actions |
| Reward | progress capped at the target speed, minus offset, steering-change, energy and wall terms and a constant time cost; the shape of each term is explained in `sim_py/README.md` |
| Domain randomisation | grip, payload, servo bandwidth, motor torque, tyre stiffness, battery resistance |

**Conditions**, 100 trips each, the same trips and cars for every policy (evaluation seeds 20000 to 20099):

- **train**: cars from the training ranges;
- **held-out**: cars outside them: less grip (0.5 to 0.8), heavier (+1 to 2 kg), slower servo, weaker motor, softer tyres;
- **perturbed**: training cars that, 4 m into the trip, lose 40 % of their grip, gain 1 kg and get a 40 % slower servo.

Seeds are kept apart: controller tuning uses 30000 onward, validation during training 40000 onward, and evaluation 20000 onward.

## Methods

| Method | Why it is here | Seeds | Budget |
|---|---|---|---|
| Pure pursuit, tuned | the controller small racing cars most often use, for its robustness and simplicity ([F1TENTH survey, arXiv 2402.18558](https://arxiv.org/html/2402.18558v2)) | - | grid search, `tools/tune_controllers.py` |
| Stanley, tuned | front-axle cross-track control (Thrun et al., *J. Field Robotics* 2006) | - | grid search |
| LQR, tuned | speed-scheduled LQR on the dynamic bicycle error model (Rajamani, *Vehicle Dynamics and Control*, sec. 3.2-3.3) | - | grid search |
| CEM, linear | does a tiny policy with no deep-learning dependencies get close? | 3 | 40 iterations |
| PPO + history | the whole task learnt from scratch; the history lets it infer the car, as RMA does (Kumar et al., RSS 2021) | 3 | 30M steps |
| SAC + history | a sample-efficient off-policy learner | 1 | 2.5M steps |
| Residual PPO on pure pursuit | learn a correction to a good controller ([Johannink et al., ICRA 2019](https://arxiv.org/pdf/1812.03201)); residual SAC on pure pursuit is the strongest recent small-racing-car result ([arXiv 2505.07321](https://arxiv.org/html/2505.07321v1)) | 3 | 20M steps |
| Residual SAC on pure pursuit | the same with SAC | 1 | 2M steps |

**Not run** (`tools/run_study.py` says why), to finish the study in an evening:

- **Residual TD3.** TD3 and SAC are used interchangeably for residual control, and residual SAC is in the study.
- **RecurrentPPO (an LSTM in place of the history).** Recurrent model-free RL is competitive only with careful architecture and hyperparameter choices ([Ni et al., ICML 2022](https://proceedings.mlr.press/v162/ni22a.html)), so one untuned run would not settle the question.

**Seeds.** Each cheap method had three seeds and each slow one a single seed. Results from few runs are statistically fragile ([Agarwal et al., NeurIPS 2021](https://proceedings.neurips.cc/paper_files/paper/2021/file/f514cec81cb148559cf475e7426eed5e-Paper.pdf)), so every rate carries a Wilson interval. Every learned method is also compared with pure pursuit pair by pair: same trip, same car.

**Model selection.** Each run is scored 20 times on 30 validation trips, and the best checkpoint is the one evaluated (`sim_py/train_rl.py`). PPO-family runs also decay their learning rate and cap each update's KL divergence. Without these, an earlier residual PPO run finished fewer held-out trips after 20M steps (81 %) than pure pursuit (89 %), having beaten pure pursuit on validation at 3M steps. The selection caught the same drift here: every residual PPO seed's best checkpoint was at 1M steps.

## Results

The full tables, with every interval, are in `results/policy_study.md`; every episode is in `results/policy_study.json`. Success, pooled over a method's seeds, with the 95 % interval:

| Method | Train | Held-out | Perturbed |
|---|---|---|---|
| pure pursuit (tuned) | 100 (96-100) | 89 (81-94) | 94 (88-97) |
| Stanley (tuned) | 100 (96-100) | 85 (77-91) | 79 (70-86) |
| LQR (tuned) | 100 (96-100) | 70 (60-78) | 80 (71-87) |
| CEM, linear, 3 seeds | 76 (71-80) | 72 (67-77) | 71 (65-76) |
| PPO + history, 3 seeds | 100 (99-100) | **94.7 (92-97)** | 91.7 (88-94) |
| SAC + history, 1 seed | 100 (96-100) | **96 (90-98)** | **99 (95-100)** |
| residual PPO, 3 seeds | 100 (99-100) | 88.3 (84-91) | 94.3 (91-96) |
| residual SAC, 1 seed | 100 (96-100) | 92 (85-96) | 95 (89-98) |

**Against pure pursuit, trip by trip.** The table counts trips each run finished that pure pursuit did not (+) and trips it failed that pure pursuit finished (−), with a two-sided sign test on those discordant trips:

| Run | Held-out | Perturbed |
|---|---|---|
| PPO s0 | +10 / −2, p = 0.039 | +4 / −3 |
| PPO s1 | +9 / −6, p = 0.61 | +4 / −10, p = 0.18 |
| PPO s2 | +9 / −3, p = 0.15 | +4 / −6 |
| SAC s0 | +10 / −3, p = 0.09 | +6 / −1, p = 0.13 |
| residual PPO s0, s1, s2 | +2/−4, +2/−2, +3/−3 | +3/−2, +3/−3, +1/−1 |
| residual SAC s0 | +8 / −5, p = 0.58 | +4 / −3 |
| Stanley | +7 / −11 | +2 / −17, p = 0.001 |
| LQR | +4 / −23, p < 0.001 | +1 / −15, p = 0.001 |

**Driving quality,** held-out cars, over the trips each finished:

| Method | Mean offset | 95th pct offset | Speed error | Steer roughness | Energy |
|---|---|---|---|---|---|
| pure pursuit | 3.5 cm | 18.3 cm | 4.2 % | 0.0023 | 0.68 mWh/m |
| residual PPO | 3.9 cm | 18.0 cm | 4.2 % | 0.0023 | 0.66 mWh/m |
| residual SAC | 7.8 cm | 23.6 cm | 4.9 % | 0.050 | 0.81 mWh/m |
| PPO + history | 11.5 cm | 29.7 cm | 9.5 % | 0.0069 | 0.83 mWh/m |
| SAC + history | 12.1 cm | 25.1 cm | 7.5 % | 0.051 | 0.95 mWh/m |

**Training cost** on 16 threads and an RTX 5060 Ti:

| Method | Time per run |
|---|---|
| CEM | 1.5 to 2.5 minutes |
| PPO | 19 to 26 minutes for 31M steps |
| residual PPO | 18 to 21 minutes |
| SAC | 55 minutes for 2.5M steps |
| residual SAC | 67 minutes for 2M steps |

**What the numbers say.**

1. **The tuned pure pursuit is the classical controller to beat.** Stanley and LQR are worse with held-out cars, and much worse after the mid-trip change. For LQR, both gaps are significant.
2. **Learning from scratch with history buys robustness and costs quality.** PPO and SAC + history finish more held-out trips than pure pursuit, though one seed at a time the gain is mostly within noise. They drive 3 to 4 times further off the centreline. SAC also steers 20 to 30 times more roughly, and both use more energy. The reward does not pay for a tidy line or smooth steering: its offset term is small next to progress and the time cost.
3. **Residual RL did not pay off here.** Validation kept residual PPO's earliest checkpoint every time, a policy that is pure pursuit plus a small correction, and it scored like pure pursuit. Residual SAC moved further from pure pursuit and gained a few trips, within noise. The literature's gains come from lap time on race tracks, where there is speed to gain. Here the speed is set by the task, and the failures come from cars that cannot make a corner.
4. **CEM's linear policy is too small.** Two of its three seeds fail a quarter to half of the trips even with training cars.

### Most held-out failures are about the start, not the driving

The task starts each episode at 4 m/s. On 22 of the campus's 90 stop pairs (S1, S5 and AB2 departures), the first junction, a turn of 51 to 90 degrees, comes 1.2 to 1.6 m after the start. A car arriving there at full speed has about 0.3 s to brake for it.

- **Training cars.** In a recording of all 90 routes, every failure of the six policies recorded was on these 22 routes.
- **Held-out cars, grip down to 0.5.** The failure rate on these routes was 48 %, against 21 % on the rest.
- **This study's evaluation trips.** With held-out cars, pure pursuit finished 50 % of turn-at-start trips and 92 % of the others. Residual PPO finished 46 % and 92 %, residual SAC 62 % and 95 %.

So most of the gap between pure pursuit and a perfect score is this setup. Part of the learned policies' held-out advantage is having learned to brake hard at once (PPO + history: 96 % on turn-at-start trips). `tools/study_report.py` reports success both ways.

## What changed because of this

A bus never leaves a stop at full speed, and it stops when it arrives. The `"follow"` task did neither: it could even end with the car running straight through its destination. So the environment gained a `"stop"` task (`sim_py/rc_twin_env.py`):

- the car starts at rest;
- the episode finishes only when the car is stopped within 0.3 m of the destination at under 0.1 m/s;
- running past the destination fails;
- the observation gains the distance left.

The tuned controllers plan their stop from that observation. Planned at 2 m/s², the stop overshot every held-out car, which brake at about 1.7 m/s² at most. Planned at 1.2 m/s², aimed 0.1 m short, the tuned pure pursuit finishes 100 % of stop-task trips with training cars, 92 % with held-out cars and 94 % after the mid-trip change. `"follow"` is unchanged, so every model here still loads and scores as reported.

## What the twin uses

The release task is `"stop"`, so the contenders were trained again on it. The tuned controllers and every learned run were evaluated on 100 trips per condition at 4 m/s, as above.

| Driver on the stop task | Seeds | Train | Held-out | Perturbed |
|---|---|---|---|---|
| **pure pursuit (tuned)** | - | **100** | **92** | **94** |
| Stanley (tuned) | - | 100 | 82 | 79 |
| LQR (tuned) | - | 100 | 69 | 81 |
| residual PPO on pure pursuit | 3 | 100, 100, 100 | 86, 93, 92 | 89, 91, 89 |
| PPO + history | 1 | 97 | 81 | 82 |
| SAC + history | 1 | 81 | 69 | 52 |

**Starting from rest, learning from scratch falls behind.** Each learned driver had the same budget as before: 30M steps for PPO, 2.5M for SAC, 20M for residual PPO. PPO and SAC + history must now also learn to pull away from rest and to brake to a stop, and with that budget they finish fewer trips than the controller, under every condition. Residual PPO again keeps the checkpoint closest to pure pursuit and scores like it.

So:

- **The twin's autopilot drives with the tuned pure pursuit**, with its stop plan. Its line and steering are the cleanest of anything tested, and on the stop task it is as robust as any learned policy and more robust than most.
- **A learned policy can be switched on beside it.** The release is `policies/release/autopilot.json`: residual PPO seed 1, the best held-out score among the learned stop-task policies. It runs in C# through `PolicyDriver`, and its recorded actions are replayed in Unity's tests. In the campus player it drives S3 to AB2, AB1 and S6 at 4 m/s exactly as pure pursuit does, with no kerb contact.
- **A learned policy that beats the controller is still open.** Candidates:
  - a longer budget on the stop task;
  - a reward that pays for a tidy line;
  - tuned hyperparameters;
  - an environment car that matches Unity's (wheel inertia, Ackermann steering and suspension are on in Unity, off in training).

  `Docs/NEXT_STEPS.md` tracks them.

## Limits

- **The car is SlipX's provisional defaults, not a measured car.** The randomisation ranges are training choices. A policy that is robust across them is robust in simulation, and no more than that.
- **One target speed (4 m/s), one campus, one task per table.** A ranking can change with any of them.
- **Few seeds for the slow methods.** SAC and residual SAC had one seed each. Their intervals are wide, and their seed-to-seed spread is unknown.
- **No hyperparameter search for the learners.** The controllers were tuned; the learners used standard settings plus the stability changes above. A tuned learner could do better, and RecurrentPPO above all needs tuning to be judged fairly (Ni et al.).
- **The reward sets the trade-off.** A heavier offset or steering term would trade some of the learned policies' robustness for a tidier line. That is a design choice, not a property of the method.

## Reproducing

```bash
.venv/Scripts/python tools/tune_controllers.py
```

```bash
.venv/Scripts/python tools/run_study.py
```

```bash
.venv/Scripts/python tools/study_report.py --trips 100 --target-speed 4
```

`tools/record_drives.py --pairs` records every route for the Unity replay (`sim_py/README.md`). Tuning, training and evaluation are deterministic given their seeds; training on GPU is not bit-exact.

## References

- Agarwal, Schwarzer, Castro, Courville, Bellemare. *Deep Reinforcement Learning at the Edge of the Statistical Precipice.* NeurIPS 2021.
- Johannink et al. *Residual Reinforcement Learning for Robot Control.* ICRA 2019. arXiv 1812.03201.
- Kumar, Fu, Pathak, Malik. *RMA: Rapid Motor Adaptation for Legged Robots.* RSS 2021.
- Ni, Eysenbach, Salakhutdinov. *Recurrent Model-Free RL Can Be a Strong Baseline for Many POMDPs.* ICML 2022.
- Rajamani. *Vehicle Dynamics and Control*, 2nd ed., Springer 2012.
- Thrun et al. *Stanley: The Robot that Won the DARPA Grand Challenge.* Journal of Field Robotics 2006.
- *Unifying F1TENTH Autonomous Racing: Survey, Methods and Benchmarks.* arXiv 2402.18558.
- *Drive Fast, Learn Faster: On-Board RL for High Performance Autonomous Racing.* arXiv 2505.07321.
