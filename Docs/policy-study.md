# Policy study: what should drive the twin around the campus?

**Question.** Does a learned policy drive the 1/10 car round the campus better than a well-tuned classical controller? If so, which kind of learning works best?

**Short answer.** On this campus, with this car and this task:

- **Success.** Learning from scratch with a short history of past frames finishes more trips with cars it never trained on than the tuned pure pursuit. SAC + history finished 96 % with held-out cars against pure pursuit's 89 %. PPO + history, over 3 seeds, finished 94.7 %.
- **Significance.** The gain is modest. Only one learned run clears p < 0.05 on its own (sign test, same trips): PPO seed 0, with held-out cars.
- **Driving quality.** The learned policies drive worse: three to four times further off the centreline, 30 to 60 % more energy per metre, and up to 30 times rougher steering.
- **Residual RL.** Learning a correction on top of pure pursuit, the approach the literature favours for small racing cars, stayed within a few trips of pure pursuit itself.

On the stop-to-stop task the twin's autopilot now drives (see [What changed](#what-changed-because-of-this)), the controller does better still. Trained for that task, every learned policy finished fewer trips than the tuned pure pursuit with held-out cars, or the same: residual PPO. The autopilot therefore drives with the tuned pure pursuit, and a learned policy can be switched on beside it. See [What the twin uses](#what-the-twin-uses).

All of that was on the Core's simplest car, not the car the Unity twin drives. On the Unity twin's car (wheel inertia, Ackermann steering, suspension), the tuned pure pursuit finishes only 59 % of stop-task trips with held-out cars, and the released policy 50 %. Training on that car, with a reward that pays for a tidy line, still gave no learned policy that beats pure pursuit. See [Closing the gap to the Unity car](#closing-the-gap-to-the-unity-car).

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
- **A learned policy that beats the controller is still open.** The next section tries two of the candidates first listed here: an environment car that matches Unity's, and a reward that pays for a tidy line. A longer budget and tuned hyperparameters remain. `Docs/NEXT_STEPS.md` tracks them.

## Closing the gap to the Unity car

Everything above trained and scored on the Core's simplest car: quasi-static wheels, parallel steering and a rigid body. The Unity twin drives `TwinCar` as the campus scene sets it up. Its wheels have inertia (1e-5 kg m²), its steering is ideal Ackermann, and it rides the DEMO suspension: 3 Hz heave, damping ratio 0.3, 15 mm travel. This round matched the training car to the twin's and tried again for a learned stop-task driver (branch `rl/twin-car`).

**What changed.**

- **The environment's car** (`car=` in `sim_py/rc_twin_env.py`):
  - `"bare"`, the default, is the study's car;
  - `"unity"` is the twin's, with springs set for the unladen car as `TwinCar` sets them;
  - `"both"` draws one of the two per episode.

  `train_rl.py --car` records it in the model's config, and `load_policy` refuses a policy on a car it did not train on. A `"both"` policy drives either car, and older models count as `"bare"`. `evaluate_policies.py --car` picks the car; `--any-car` lets a policy drive a car it did not train on, to measure the gap.
- **A stop-task reward that pays for a tidy line.** The offset and steering-change terms are charged linearly, as |offset| / half width and |change in steer action|, rather than squared. Squares cost almost nothing near the centreline. Two weightings were tried: v1 at 2 for both terms, v2 at 1 for both (`REWARD_STOP`, current). Each model's JSON now records its weights. `"follow"` keeps its reward, so every result above still reproduces.
- **Pure pursuit tuned for the Unity car.** `tools/tune_controllers.py --task stop --car unity` gives lookahead 1.5 m, speed gain 2 and a cornering limit of 12 m/s² (`policies/controllers_unity_stop.json`). The study's gains were tuned on `"follow"` with the bare car. `policies/controllers.json` is unchanged: the residual policies are built on it.

**The car gap is large.** Stop task, 4 m/s, 100 trips per condition, success in % (train / held-out / perturbed):

| Driver | Bare car | Unity car |
|---|---|---|
| pure pursuit, study gains | 100 / 92 / 94 | 91 / 59 / 69 |
| pure pursuit, tuned for the Unity car | - | 99 / 71 / 66 |
| residual PPO s1, released (trained on the bare car) | 100 / 93 / 91 | 94 / 50 / 65 |

Most failures on the Unity car are timeouts, not crashes. At a junction, a held-out car hits the kerb and stays pinned against it, nose first, at full throttle: about 20 000 wall contacts in 120 s. On the bare car the same car on the same trip glances off the kerb and drives on. No driver here backs off a kerb, so one strike ends the trip. The twin's autopilot is therefore much less robust in Unity than the bare-car numbers said.

**Learned drivers trained on the Unity car.** Same protocol, Unity car. Quality is with held-out cars, over the trips each finished. Trip time is the median ratio to pure pursuit (study gains) over the trips both finished.

| Driver | Reward | Train | Held-out | Perturbed | Mean offset | Steer roughness | Kerb contact | Trip time |
|---|---|---|---|---|---|---|---|---|
| pure pursuit, study gains | - | 91 | 59 | 69 | 2.5 cm | 0.0018 | 46 % | 1.00 |
| pure pursuit, tuned for the Unity car | - | 99 | 71 | 66 | 3.1 cm | 0.0023 | 43 % | 1.00 |
| residual PPO s0 | v1 | 96 | 43 | 67 | 6.6 cm | 0.0055 | 62 % | 1.03 |
| residual PPO s1 | v1 | 100 | 54 | 66 | 12.3 cm | 0.0066 | 50 % | 1.00 |
| residual PPO s2 | v1 | 96 | 51 | 67 | 23.5 cm | 0.0086 | 94 % | 1.10 |
| PPO + history s0 | v1 | 88 | 82 | 67 | 2.5 cm | 0.0063 | 12 % | 2.30 |
| PPO + history s0 | v2 | 97 | 68 | 51 | 10.8 cm | 0.0098 | 48 % | 1.34 |
| residual PPO s0 | v2 | 98 | 36 | 63 | 9.6 cm | 0.0060 | 71 % | 0.99 |
| residual PPO s0, trained on both cars | v2 | 98 | 63 | 37 | 6.8 cm | 0.0100 | 37 % | 0.97 |

Budgets as before: 20M steps for residual PPO (16 minutes a run), 30M for PPO + history (29 minutes). Every best checkpoint came late, between 15M and 30M steps. With the old reward on the bare car, every residual run had kept its 1M-step checkpoint.

**What the numbers say.**

1. **Training on the Unity car buys training-car success, not robustness.** Residual PPO lifts pure pursuit's 91 % with training cars to 96 to 100 %. But every run finishes fewer held-out trips than the controller it corrects: 36 to 54 % against 59 %. Validation uses training cars, so model selection rewards exactly this. Training on both cars did not change the picture: 63 % with held-out cars, but 37 % after the mid-trip change, and the roughest steering of all. On the bare car the same policy finished 100 / 90 / 79 %.
2. **The tidy-line reward works, at a price in speed.** On the stop task with the old reward (bare car), PPO + history drove 27.5 cm off the centreline with held-out cars, along the kerb. With v1 (Unity car) it drove 2.5 cm off, like pure pursuit, but took 2.3 times as long per trip. Leaving the road is charged the offset weight for every tick it saves, which keeps crashing from paying. At weight 2, a failure therefore cost 2.5 times as much against progress as before, and the policy learned to crawl. At weight 1 (v2) it drove faster again (1.34 times pure pursuit's trip time) and lost the line (10.8 cm).
3. **Here, a tidy line costs robustness.** As an ablation, residual PPO with reward v1 trained on the bare car (seed 0). It drove a tidier line than pure pursuit, 1.4 against 2.0 cm with training cars, and finished every training trip. With held-out cars it finished 70 % against pure pursuit's 92 %: it failed 22 trips pure pursuit finished, and finished none that pure pursuit failed. The campus centreline turns 90 degrees at junctions. A policy paid to stay on it turns in late and hard, which a low-grip car cannot do. Pure pursuit cuts the corner by its lookahead, and that is what saves those cars.
4. **The held-out gains left came from driving slowly.** PPO + history with v1 finished 82 % of held-out trips: 24 that pure pursuit failed, against 1 the other way. It did so at well under half the target speed. That is not the 4 m/s task.
5. **Kerb strikes are too cheap.** The wall term charges the speed a strike takes. Residual seed 2 glanced along a kerb on 94 % of held-out trips, at small cost each time.

**Decision.** No learned policy beats the tuned pure pursuit on held-out success without worse driving quality. The release, `policies/release/autopilot.json`, is unchanged, and the campus player was not rebuilt.

**What this means for the twin.**

- Both of the twin's drivers were tuned or trained on the bare car. On the Unity car they finish 50 to 59 % of trips with held-out cars.
- Pure pursuit tuned for the Unity car finishes 99 / 71 / 66 %. Switching the autopilot to those gains is a small, separate change (done: [The autopilot on the Unity car](#the-autopilot-on-the-unity-car)).
- Most failures on the Unity car are kerb pins. A driver that backs off a kerb and tries again would recover most of them (done: 100 / 78 / 76 %, and 98 / 82 / 86 % once the car can reverse).
- Before more learning:
  - an offset term measured from a drivable path that cuts the junction corners, not from the junction polyline;
  - kerb contact charged for every tick it lasts, not only by impulse;
  - validation on cars outside the training ranges, so selection stops rewarding training-car gains alone;
  - tuned hyperparameters, and a longer budget for PPO + history, which was still improving at 30M steps under both rewards.

## The autopilot on the Unity car

The two cheap wins above, in the twin:

- **The Unity-car gains.** The autopilot's pure pursuit uses `policies/controllers_unity_stop.json` (lookahead 1.5 m, speed gain 2, cornering limit 12 m/s²): `PurePursuitDriver.TunedForTwin`, checked against that file in Edit Mode. `policies/controllers.json` and `PurePursuitDriver.Tuned`'s defaults stay the study's gains, which the residual policies are built on.
- **Backing off a pinned kerb.** `KerbRecovery` (`sim_py/controllers.py`, `Drivers.cs`, line for line, checked on a scripted trace) wraps the autopilot's driver, pure pursuit or learned. A pinned car is not still: it bounces off the kerb and back at a few tenths of a metre a second. So the test is distance, not speed. If the driver has asked for more than 0.3 throttle for 0.5 s and the car has moved less than 5 cm in that time (its speed, integrated), the recovery reverses at -0.6 on the other lock. That swings the nose toward where the driver was steering. It reverses for 0.3 m or at most 1.5 s, then hands back. The parameters were set by hand, not tuned on these seeds. The observation has no wall contact, so this is the only signal a driver has.

Stop task, 4 m/s, Unity car, 100 trips per condition, success in % (train / held-out / perturbed). `recover:` wraps a policy spec in `evaluate_policies.py`:

| Driver | Core without reverse | Core with ESC reverse (ADR 0010, the Core now) |
|---|---|---|
| pure pursuit, study gains | 91 / 59 / 69 | 90 / 61 / 67 |
| pure pursuit, Unity-car gains | 99 / 71 / 66 | 98 / 66 / 68 |
| pure pursuit, Unity-car gains, kerb recovery | **100 / 78 / 76** | **98 / 82 / 86** |
| pure pursuit, study gains, kerb recovery | - | 98 / 84 / 82 |

**Reverse.** With wheel inertia, as on the Unity car, a negative demand used to be the motor brake (the first column): it stops the wheels and never turns them backwards, so the car cannot reverse. The recovery still helps there. Lifting off and steering the other way frees many pinned cars. With held-out cars, timeouts fall from 22 to 4, but leaving the road rises from 4 to 14: a freed car often leaves the road later. Trip by trip, the recovery wins 8 and loses 1 against the controller alone (held-out), and wins 10 and loses none (perturbed). The second column is the Core with ADR 0010's ESC reverse latch, merged with the driven turn-round: a negative demand held at rest engages reverse. The autopilot runs on it. With it, no trip times out. On held-out cars the recovery finishes 17 trips the controller alone lost and loses 1 it finished. ADR 0010 also changes the Core slightly beyond reversing (the slip angle when a wheel rolls backwards), and a brake demand at rest now reverses, so the rows without the recovery differ a little between the columns.

The remaining failures are leaving the road (15 and 11 % held-out and perturbed, with reverse) and running past the stop (3 %). These are low-grip cars that spin at junctions, which backing off does not fix.

## Limits

- **The car is SlipX's provisional defaults, not a measured car.** The randomisation ranges are training choices. A policy that is robust across them is robust in simulation, and no more than that.
- **One target speed (4 m/s), one campus, one task per table.** A ranking can change with any of them.
- **Few seeds for the slow methods.** SAC and residual SAC had one seed each. Their intervals are wide, and their seed-to-seed spread is unknown.
- **No hyperparameter search for the learners.** The controllers were tuned; the learners used standard settings plus the stability changes above. A tuned learner could do better, and RecurrentPPO above all needs tuning to be judged fairly (Ni et al.).
- **The Unity car's suspension is a DEMO target, not a measurement.** The car-gap numbers say the twin's car is harder than the bare one. How much harder the real car is depends on rates nobody has measured yet.
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

The Unity-car round (`rl/twin-car`). Its baselines also use the earlier stop-task models, kept locally in `policies/v2/`:

```bash
.venv/Scripts/python tools/tune_controllers.py --kinds pure_pursuit --task stop --car unity --out policies/controllers_unity_stop.json
```

```bash
.venv/Scripts/python sim_py/train_rl.py --algo ppo --residual --seed 0 --timesteps 20000000 --task stop --car unity --out policies/twin-car/ppo-res-stop-unity-r2-s0
```

```bash
.venv/Scripts/python sim_py/evaluate_policies.py controller:pure_pursuit sb3:policies/twin-car/ppo-res-stop-unity-r2-s0 --task stop --car unity --trips 100 --target-speed 4
```

The autopilot on the Unity car, on the current Core (ESC reverse, the second column):

```bash
.venv/Scripts/python sim_py/evaluate_policies.py controller:pure_pursuit controller:pure_pursuit:policies/controllers_unity_stop.json recover:controller:pure_pursuit:policies/controllers_unity_stop.json --task stop --car unity --trips 100 --target-speed 4
```

The v1-reward runs trained at commit `fe415b6`, where `REWARD_STOP` weighs offset and steering at 2.

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
