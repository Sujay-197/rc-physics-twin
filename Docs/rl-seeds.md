# More training seeds for the learned-vs-classical comparison

The [policy study](policy-study.md) trained SAC + history and residual SAC once each, and PPO + history and residual PPO three times. Results from so few runs are fragile (Agarwal et al., NeurIPS 2021). This adds seeds and changes nothing else: the same budget, reward, car, checkpoint selection and hyperparameters, on the same code and Core the original runs trained on.

**Short answer.**

- **Bare car, follow task: SAC + history holds up.** Over 5 seeds it finishes 96.6 % of trips with held-out cars, against pure pursuit's 89 %. Every seed beats pure pursuit, and the gain is significant with the trip as the unit: +7.6 points (95 % CI +2.0 to +13.8, p = 0.016), and +10.8 points (p = 0.001) on a fresh set of trips. PPO + history also beats pure pursuit with every seed (+5.6 points), but the interval reaches 0 (p = 0.085). After the mid-trip change, PPO + history loses (3 of 5 seeds are worse).
- **The residual learners tie pure pursuit.** Residual PPO: +0.0 points over 5 seeds. Residual SAC's single-seed edge (92 %) was luck: over 5 seeds it scores 89.2 %, and after the mid-trip change it loses (−3.6 points).
- **Unity car, second round: residual PPO still ties the autopilot.** With kerb recovery, over 5 seeds: 85.0 % held-out against 82 %, +3.0 points (−1.2 to +7.4, p = 0.22), and −2.6 points on fresh trips. PPO + history's single run (85 %) was the best of five. The others finish 23 to 67 % of held-out trips, and pooled it loses badly: 50.6 %, −31.4 points (p < 0.001).
- **On today's Core, the follow-task gain is gone.** The ESC reverse (ADR 0010) came the day after the study, and it frees kerb-pinned cars. Pure pursuit then finishes 96 % of held-out trips, and SAC + history ties it (+0.0 points). On the Unity car, the shipped corner-plan autopilot finishes every trip in every condition.

Everything here is in simulation, on one physics model and SlipX's provisional car parameters, not a measured car.

## Each group on its own code

The Core changed after these runs:

- **ADR 0010, the ESC reverse** (21 Oct), came the day after the policy study (`e552c96`, 20 Oct). It lets a car pinned against a kerb back off. On today's Core, pure pursuit finishes 96 % of the study's held-out trips, not 89 %: 7 of its 9 held-out timeouts were kerb pins.
- **ADR 0011, the low-speed blend** for the Unity car (25 Oct), came after the second round (`47175b9`).

So each group trains and is evaluated in a checkout of its own commit, with that commit's Core (`git worktree add --detach`). Both checkouts reproduce what the study recorded:

- **Follow task, `e552c96`.** Pure pursuit and all nine of the study's SAC, residual SAC, PPO and residual PPO models give every one of their 300 trips the recorded outcome and length (`results/policy_study.json`). The study's models are the seed 0 to 2 runs here; seeds 1 to 4 for SAC and residual SAC and seeds 3 and 4 for PPO and residual PPO are new.
- **Unity car, `47175b9`.** The autopilot gives 98 / 82 / 86 % and pure pursuit without the recovery 98 / 66 / 68 %, as recorded. The second round's models were never kept, only their configs, so all five seeds trained again. Seeds 0 to 2 give exactly the recorded success: residual PPO 100 / 86 / 85, 100 / 83 / 85 and 100 / 82 / 86 with the recovery, PPO + history 91 / 85 / 80. Training reproduced them.

Both groups were also evaluated on today's Core as a check.

## Protocol

As in the study: 4 m/s, 100 trips per condition (seeds 20000 to 20099), the same trips and cars for every policy, in the train, held-out and perturbed conditions. Each run's checkpoint is the best of 20 on 30 validation trips (seeds 40000 onward). Follow task: bare car, against the tuned pure pursuit. Unity car: stop task, reward v3, the residual on the Unity-car gains, against the autopilot (pure pursuit on the Unity-car gains with the kerb recovery), learned drivers also with the kerb recovery.

**Statistics.** Success rates are pooled over a method's seeds, with a Wilson 95 % interval, and each seed's rate gives the spread. Against the baseline, trip by trip, in two ways:

- **Pooled over seeds** (as the study did): trips each run finished that the baseline did not (+), and the reverse (−), summed over the seeds, with a two-sided exact sign test. This counts the same trip once per seed, as if the seeds drove different trips, so with five seeds it overstates significance.
- **With the trip as the unit** (the headline here): per trip, the share of seeds that finished it minus whether the baseline did. The difference in success rate gets a bootstrap 95 % interval (trips resampled), and a paired sign-flip permutation test. Each seed is also counted as better or worse than the baseline.

## Bare car, follow task (Core `e552c96`)

Success in %, pooled over seeds (95 % interval), and each seed's:

| Method | Seeds | Train | Held-out | Perturbed | Held-out per seed | Perturbed per seed |
|---|---|---|---|---|---|---|
| pure pursuit | - | 100 (96-100) | 89 (81-94) | 94 (88-97) | | |
| SAC + history | 5 | 100 (99-100) | **96.6 (95-98)** | **98.2 (97-99)** | 96, 98, 93, 98, 98 | 99, 100, 97, 99, 96 |
| PPO + history | 5 | 100 (99-100) | **94.6 (92-96)** | 92.4 (90-94) | 97, 92, 95, 93, 96 | 95, 88, 92, 93, 94 |
| residual SAC | 5 | 99.2 (98-100) | 89.2 (86-92) | 90.4 (88-93) | 92, 79, 95, 90, 90 | 95, 81, 95, 84, 97 |
| residual PPO | 5 | 100 (99-100) | 89.0 (86-91) | 93.8 (91-96) | 87, 89, 89, 90, 90 | 95, 94, 94, 93, 93 |

Seed 0 is the study's run (and seeds 1 and 2 for PPO and residual PPO).

**Against pure pursuit.** The difference in success rate with the trip as the unit (95 % interval, permutation p), the seeds better / worse, and the study's pooled count:

| Method | Held-out: difference | seeds | pooled | Perturbed: difference | seeds | pooled |
|---|---|---|---|---|---|---|
| SAC + history | **+7.6 (+2.0 to +13.8), p = 0.016** | 5 / 0 | +50 / −12 | +4.2 (+0.0 to +9.0), p = 0.094 | 5 / 0 | +28 / −7 |
| PPO + history | +5.6 (+0.0 to +11.8), p = 0.085 | 5 / 0 | +48 / −20 | −1.6 (−5.8 to +3.0), p = 0.55 | 1 / 3 | +22 / −30 |
| residual SAC | +0.2 (−5.4 to +6.0), p = 1.00 | 4 / 1 | +37 / −36 | −3.6 (−8.0 to +1.0), p = 0.15 | 3 / 2 | +18 / −36 |
| residual PPO | +0.0 (−4.0 to +4.0), p = 1.00 | 2 / 1 | +15 / −15 | −0.2 (−3.6 to +3.0), p = 1.00 | 1 / 2 | +11 / −12 |

Per seed, held-out, against pure pursuit (two-sided sign test where p < 0.1):

| Method | s0 | s1 | s2 | s3 | s4 |
|---|---|---|---|---|---|
| SAC + history | +10 / −3 (0.09) | +11 / −2 (0.022) | +9 / −5 | +10 / −1 (0.012) | +10 / −1 (0.012) |
| PPO + history | +10 / −2 (0.039) | +9 / −6 | +9 / −3 | +11 / −7 | +9 / −2 (0.065) |
| residual SAC | +8 / −5 | +4 / −14 (0.031) | +9 / −3 | +7 / −6 | +9 / −8 |
| residual PPO | +2 / −4 | +2 / −2 | +3 / −3 | +4 / −3 | +4 / −3 |

**Driving quality,** held-out cars, over the trips each finished, pooled over seeds:

| Method | Mean offset | 95th pct offset | Speed error | Steer roughness | Energy | Wall contact | Trip time |
|---|---|---|---|---|---|---|---|
| pure pursuit | 3.5 cm | 18.3 cm | 4.2 % | 0.0023 | 0.68 mWh/m | 33 % | 1.00 |
| SAC + history | 8.5 cm | 23.2 cm | 8.4 % | 0.055 | 0.83 mWh/m | 25 % | 1.01 |
| PPO + history | 10.3 cm | 27.9 cm | 8.5 % | 0.0059 | 0.74 mWh/m | 37 % | 0.99 |
| residual SAC | 6.9 cm | 23.9 cm | 4.4 % | 0.035 | 0.77 mWh/m | 41 % | 1.00 |
| residual PPO | 3.8 cm | 18.1 cm | 4.0 % | 0.0023 | 0.66 mWh/m | 31 % | 1.01 |

Trip time is the median ratio to pure pursuit over trips both finished.

**What changes.**

1. **SAC + history's gain is real on this Core.** One seed gave 96 %; five give 96.6 %, all five beat pure pursuit with held-out cars, four of them with p < 0.1 on their own, and the trip-level interval excludes zero. The cost is as before, a little smaller: 2.4 times pure pursuit's offset (single seed: 3.5 times), 24 times rougher steering, 22 % more energy.
2. **PPO + history beats pure pursuit with held-out cars on every seed, but not after the mid-trip change.** There it finishes 92.4 % against 94 %, and loses on 3 of 5 seeds.
3. **Residual SAC's edge was a lucky seed.** Its seed 1 finishes 79 % of held-out trips and 81 % after the change. Seeds 1 and 3 kept their checkpoint at 0.1M steps, seeds 0, 2 and 4 at 1.6 to 2M. Pooled, it ties pure pursuit with held-out cars and loses after the change.
4. **Residual PPO is pure pursuit.** All five seeds kept their 1M-step checkpoint, as the study's three did, and none differs from pure pursuit by more than 3 trips.

## Unity car, second round (Core `47175b9`)

Stop task, reward v3, residual on the Unity-car gains. Success in %, pooled over seeds (95 % interval):

| Driver | Seeds | Train | Held-out | Perturbed | Held-out per seed | Perturbed per seed |
|---|---|---|---|---|---|---|
| **autopilot**: pure pursuit, Unity-car gains, kerb recovery | - | 98 (93-99) | 82 (73-88) | 86 (78-91) | | |
| pure pursuit, Unity-car gains, no recovery | - | 98 (93-99) | 66 (56-75) | 68 (58-76) | | |
| residual PPO, kerb recovery | 5 | 100 (99-100) | **85.0 (82-88)** | 86.2 (83-89) | 86, 83, 82, 89, 85 | 85, 85, 86, 88, 87 |
| residual PPO, no recovery | 5 | 99.2 (98-100) | 69.2 (65-73) | 71.2 (67-75) | 70, 71, 65, 67, 73 | 72, 70, 73, 69, 72 |
| PPO + history, 45M steps | 5 | 94.2 (92-96) | 50.6 (46-55) | 47.0 (43-51) | 85, 23, 25, 67, 53 | 80, 21, 22, 63, 49 |

PPO + history scores the same with and without the kerb recovery: the recovery never engages for it.

**Against the autopilot,** with the trip as the unit:

| Driver | Held-out: difference | seeds | pooled | Perturbed: difference | seeds | pooled |
|---|---|---|---|---|---|---|
| residual PPO, kerb recovery | +3.0 (−1.2 to +7.4), p = 0.22 | 4 / 0 | +33 / −18 | +0.2 (−5.6 to +6.2), p = 1.00 | 2 / 2 | +38 / −37 |
| residual PPO, no recovery | −12.8 (−18.8 to −7.0), p < 0.001 | 0 / 5 | +8 / −72 | −14.8 (−22.0 to −7.8), p < 0.001 | 0 / 5 | +13 / −87 |
| PPO + history | −31.4 (−38.8 to −23.6), p < 0.001 | 1 / 4 | +28 / −185 | −39.0 (−47.0 to −31.0), p < 0.001 | 0 / 5 | +25 / −220 |

Residual PPO with the recovery, per seed, held-out: +6 / −2, +8 / −7, +4 / −4, +10 / −3 (p = 0.09), +5 / −2. Seeds 0 to 2 are the study's +18 / −13 (p = 0.47).

**Driving quality,** held-out cars, over the trips each finished, pooled over seeds:

| Driver | Mean offset | Steer roughness | Kerb contact | Trip time |
|---|---|---|---|---|
| autopilot | 6.8 cm | 0.0048 | 42 % | 1.00 |
| residual PPO, kerb recovery | 8.6 cm | 0.0060 | 43 % | 0.99 |
| PPO + history | 12.7 cm | 0.0092 | 51 % | 1.17 |

**What changes.**

1. **Residual PPO still ties the autopilot.** Five seeds give the same answer as three: a few more held-out trips (+3 points), within noise, at the cost of a slightly wider line. The pooled sign test would now say p = 0.049, but only because it counts each trip five times; with the trip as the unit, p = 0.22.
2. **PPO + history's second-round result was its best seed.** Seed 0 is the round's 91 / 85 / 80 %. Seeds 1 to 4 finish 23, 25, 67 and 53 % of held-out trips, mostly by timing out or overshooting the stop. All five scored 90 to 97 % on their validation trips (selected at 27.5 to 45.9M steps), so validation could not tell them apart: it drives training cars, which every seed handles.

## On today's Core (check)

The same models and trips on the current `main` Core (ESC reverse, low-speed blend). Held-out success; full tables in `results/reference/rl_seeds/*_today_20000.md`.

| Group | Driver | Train | Held-out | Perturbed | Held-out vs baseline (trip as unit) |
|---|---|---|---|---|---|
| follow | pure pursuit | 100 | 96 | 99 | - |
| | SAC + history (5) | 100 | 96.0 | 98.0 | +0.0 (−3.8 to +4.4), p = 1.00 |
| | PPO + history (5) | 100 | 93.0 | 88.4 | −3.0 (−7.4 to +2.0), p = 0.24 |
| | residual SAC (5) | 99.8 | 95.6 | 94.8 | −0.4 (−4.4 to +4.0), p = 0.93 |
| | residual PPO (5) | 100 | 95.4 | 99.4 | −0.6 (−2.4 to +1.2), p = 0.68 |
| Unity car | autopilot without the corner plan | 100 | 88 | 88 | - |
| | residual PPO, kerb recovery (5) | 100 | 84.8 | 89.0 | −3.2 (−8.8 to +2.4), p = 0.31 |
| | PPO + history (5) | 73.8 | 36.8 | 36.4 | −51.2 (−58.2 to −43.8), p < 0.001 |
| | shipped autopilot (corner plan) | 100 | 100 | 100 | +12.0 (+6.0 to +19.0), p < 0.001 |

With the ESC reverse, pure pursuit no longer gets stuck on kerbs, and that was most of what the learners had over it on the follow task. After the mid-trip change, PPO + history and residual SAC now lose significantly. The learned models were trained on the older Cores; the shipped autopilot's corner speed plan came after both rounds.

## Fresh trips (seeds 50000 onward)

A second set of 100 trips per condition, with seeds the study never used for learned policies, on each group's own Core. Full tables in `results/reference/rl_seeds/*_50000.md`.

| Group | Driver | Train | Held-out | Perturbed | Held-out vs baseline (trip as unit) | Seeds better / worse |
|---|---|---|---|---|---|---|
| follow | pure pursuit | 100 | 86 | 93 | - | |
| | SAC + history (5) | 100 | 96.8 | 97.4 | **+10.8 (+4.6 to +17.4), p = 0.001** | 5 / 0 |
| | PPO + history (5) | 100 | 91.6 | 90.8 | +5.6 (−0.2 to +11.8), p = 0.081 | 5 / 0 |
| | residual SAC (5) | 99.2 | 89.4 | 90.8 | +3.4 (−1.4 to +8.4), p = 0.21 | 3 / 1 |
| | residual PPO (5) | 100 | 88.6 | 93.8 | +2.6 (−0.8 to +6.2), p = 0.20 | 5 / 0 |
| Unity car | autopilot | 100 | 91 | 87 | - | |
| | residual PPO, kerb recovery (5) | 99.4 | 88.4 | 84.6 | −2.6 (−6.2 to +1.0), p = 0.20 | 0 / 4 |
| | PPO + history (5) | 91.4 | 51.2 | 46.0 | −39.8 (−45.8 to −33.0), p < 0.001 | 0 / 5 |

The same picture. SAC + history beats pure pursuit with held-out cars, on every seed, and PPO + history beats it on every seed but within noise; after the mid-trip change, PPO + history loses (−2.2, 3 of 5 seeds worse). The residuals tie their baselines. On the Unity car, residual PPO's small edge over the autopilot on seeds 20000 onward does not carry over (−2.6 here), and PPO + history's seeds rank as before (79, 29, 21, 70, 57 % held-out).

## What this means for the policy study

- The study's first table holds for SAC + history, on the Core it ran on, now with 5 seeds instead of 1. It does not hold for residual SAC. PPO + history beats pure pursuit with held-out cars on every seed, but the trip-level test does not reach p < 0.05, and after the mid-trip change it loses.
- The second round's residual PPO is a tie with 5 seeds as with 3. Its PPO + history row is one seed's result and not typical of the method.
- Both tables are on Cores that are no longer current. On today's Core, pure pursuit closes the follow-task gap, and the shipped autopilot beats every learned driver on the Unity car.

## Not done

- **The bare-car stop task.** Its models (`policies/v2/`) were not kept, and that table is not part of this comparison's headline.
- **Rounds 1 and 3 on the Unity car.** Round 2 is the setup the twin keeps (round 3's `--val-edge` stays off).
- **CEM, TD3, RecurrentPPO, and any hyperparameter search.** Only seeds were added.

## Notes

- Residual PPO seed 3 on the Unity car shows 11185 s of training time: the PC slept for about 2 h 50 min during its 20th chunk. Training resumed where it stopped; its numbers are unaffected.
- All training was on the GPU (`"device": "cuda"` in every config). Training reproduced the second round's seeds 0 to 2 here, though the study notes that GPU training is not bit-exact in general.
- `results/reference/rl_seeds/<name>.outcomes.json` keeps every trip's outcome (seed, status, ticks, wall contact) for each policy, so the tests can be recomputed without re-running the evaluation.

## Reproducing

The study-era checkouts, each with its own build (README, "Build and test the native library"):

```bash
git worktree add --detach ../rc-physics-twin-seeds-e552c96 e552c96
```

```bash
git worktree add --detach ../rc-physics-twin-seeds-47175b9 47175b9
```

In each, and in this checkout:

```bash
.toolchain/venv/Scripts/cmake -S . -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE="$PWD/.toolchain/zig-toolchain.cmake" -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

```bash
.toolchain/venv/Scripts/cmake --build build --target rc_twin
```

The study's own models (`policies/study/*-s0`, and PPO and residual PPO `-s1`, `-s2`) are local files, as before. Train the new seeds; the queue skips finished runs and can be restarted, and writes every command it runs to `results/rl_seeds/commands.txt`:

```bash
.venv/Scripts/python tools/run_seeds.py --dry-run
```

```bash
.venv/Scripts/python tools/run_seeds.py
```

Each run is `sim_py/train_rl.py` of the group's checkout, for example:

```bash
.venv/Scripts/python -I ../rc-physics-twin-seeds-e552c96/sim_py/train_rl.py --algo sac --timesteps 2500000 --batch 4096 --utd 0.0625 --learning-starts 20000 --seed 1 --target-speed 4 --out policies/study/sac-s1
```

```bash
.venv/Scripts/python -I ../rc-physics-twin-seeds-47175b9/sim_py/train_rl.py --algo ppo --residual --residual-gains policies/controllers_unity_stop.json --timesteps 20000000 --task stop --car unity --seed 3 --target-speed 4 --out policies/twin-car/seeds/ppo-res-stop-unity-r3-s3
```

(run from the checkout, with `--out` an absolute path into this one). Evaluate, each comparison on its checkout, and summarise:

```bash
.venv/Scripts/python tools/rl_seeds_eval.py
```

```bash
.venv/Scripts/python tools/rl_seeds_report.py results/rl_seeds/eval/follow_e552c96_20000.json --baseline controller:pure_pursuit --out results/reference/rl_seeds/follow_e552c96_20000
```

```bash
.venv/Scripts/python tools/rl_seeds_report.py results/rl_seeds/eval/unity_47175b9_20000.json --baseline recover:controller:pure_pursuit:policies/controllers_unity_stop.json --out results/reference/rl_seeds/unity_47175b9_20000
```

and the same for `follow_today_20000`, `unity_today_20000`, `follow_e552c96_50000` and `unity_47175b9_50000`.

Training took about 11 hours of compute on 16 threads and an RTX 5060 Ti (SAC 52 to 57 minutes a run, residual SAC 38 to 40, PPO 17, residual PPO 13, the Unity car's residual PPO 15 and PPO + history 29 to 32); the six evaluations about an hour.
