Task follow, car bare, 4 m/s, 100 trips per condition per seed (seeds 20000 to 20099). Baseline `controller:pure_pursuit`.

| method | seeds | train: pooled (95 % CI) | held_out: pooled (95 % CI) | perturbed: pooled (95 % CI) | train: per seed | held_out: per seed | perturbed: per seed |
|---|---|---|---|---|---|---|---|
| controller:pure_pursuit | - | 100.0 (96-100) | 96.0 (90-98) | 99.0 (95-100) | 100 | 96 | 99 |
| sac | 5 | 100.0 (99-100) | 96.0 (94-97) | 98.0 (96-99) | 100, 100, 100, 100, 100 | 96, 98, 92, 100, 94 | 99, 100, 97, 98, 96 |
| sac-res | 5 | 99.8 (99-100) | 95.6 (93-97) | 94.8 (92-96) | 100, 99, 100, 100, 100 | 93, 94, 98, 97, 96 | 98, 91, 97, 88, 100 |
| ppo | 5 | 100.0 (99-100) | 93.0 (90-95) | 88.4 (85-91) | 100, 100, 100, 100, 100 | 97, 92, 90, 90, 96 | 93, 88, 83, 84, 94 |
| ppo-res | 5 | 100.0 (99-100) | 95.4 (93-97) | 99.4 (98-100) | 100, 100, 100, 100, 100 | 95, 94, 96, 95, 97 | 98, 100, 100, 99, 100 |

Against the baseline, trip by trip: + finished where the baseline failed, − the reverse; two-sided exact sign test.

| method | train: pooled | held_out: pooled | perturbed: pooled | train: per seed | held_out: per seed | perturbed: per seed |
|---|---|---|---|---|---|---|
| sac | +0 / −0, p = 1.00 | +18 / −18, p = 1.00 | +5 / −10, p = 0.30 | +0/−0; +0/−0; +0/−0; +0/−0; +0/−0 | +4/−4; +4/−2; +4/−8; +4/−0; +2/−4 | +1/−1; +1/−0; +1/−3; +1/−2; +1/−4 |
| sac-res | +0 / −1, p = 1.00 | +18 / −20, p = 0.87 | +3 / −24, p < 0.001 | +0/−0; +0/−1; +0/−0; +0/−0; +0/−0 | +4/−7; +3/−5; +4/−2; +3/−2; +4/−4 | +1/−2; +0/−8 (p = 0.008); +1/−3; +0/−11 (p < 0.001); +1/−0 |
| ppo | +0 / −0, p = 1.00 | +19 / −34, p = 0.053 | +5 / −58, p < 0.001 | +0/−0; +0/−0; +0/−0; +0/−0; +0/−0 | +4/−3; +4/−8; +3/−9; +4/−10; +4/−4 | +1/−7 (p = 0.070); +1/−12 (p = 0.003); +1/−17 (p < 0.001); +1/−16 (p < 0.001); +1/−6 |
| ppo-res | +0 / −0, p = 1.00 | +4 / −7, p = 0.55 | +4 / −2, p = 0.69 | +0/−0; +0/−0; +0/−0; +0/−0; +0/−0 | +1/−2; +0/−2; +1/−1; +0/−1; +2/−1 | +1/−2; +1/−0; +1/−0; +0/−0; +1/−0 |

The same with the trip as the unit (a trip counts once, however many seeds drove it): the difference in success rate with a bootstrap 95 % interval over trips; trips where the share of seeds that finished beats the baseline (+) or loses to it (−), with a sign test; a paired sign-flip permutation test of the per-trip differences; and seeds with more + than − trips, and fewer.

| method | train: difference [pp] (95 % CI), permutation p | held_out: difference [pp] (95 % CI), permutation p | perturbed: difference [pp] (95 % CI), permutation p | train: trips +/− | held_out: trips +/− | perturbed: trips +/− | train: seeds better/worse | held_out: seeds better/worse | perturbed: seeds better/worse |
|---|---|---|---|---|---|---|---|---|---|
| sac | +0.0 (+0.0 to +0.0), p = 1.00 | +0.0 (-3.8 to +4.4), p = 1.00 | -1.0 (-3.0 to +1.6), p = 0.61 | +0 / −0, p = 1.00 | +4 / −15, p = 0.019 | +1 / −9, p = 0.021 | 0 / 0 of 5 | 2 / 2 of 5 | 1 / 3 of 5 |
| sac-res | -0.2 (-0.6 to +0.0), p = 1.00 | -0.4 (-4.4 to +4.0), p = 0.93 | -4.2 (-7.0 to -1.6), p = 0.003 | +0 / −1, p = 1.00 | +4 / −15, p = 0.019 | +1 / −17, p < 0.001 | 0 / 1 of 5 | 2 / 2 of 5 | 1 / 4 of 5 |
| ppo | +0.0 (+0.0 to +0.0), p = 1.00 | -3.0 (-7.4 to +2.0), p = 0.24 | -10.6 (-14.6 to -6.6), p < 0.001 | +0 / −0, p = 1.00 | +4 / −27, p < 0.001 | +1 / −37, p < 0.001 | 0 / 0 of 5 | 1 / 3 of 5 | 0 / 5 of 5 |
| ppo-res | +0.0 (+0.0 to +0.0), p = 1.00 | -0.6 (-2.4 to +1.2), p = 0.68 | +0.4 (-0.8 to +2.2), p = 1.00 | +0 / −0, p = 1.00 | +3 / −4, p = 1.00 | +1 / −2, p = 1.00 | 0 / 0 of 5 | 1 / 3 of 5 | 3 / 1 of 5 |

Driving quality, held_out cars, over the trips each finished (pooled over seeds); trip time is the median ratio to the baseline over trips both finished.

| method | mean offset [cm] | p95 offset [cm] | speed error [%] | steer roughness | energy [mWh/m] | wall contact [% trips] | left road | timed out | overshot | trip time |
|---|---|---|---|---|---|---|---|---|---|---|
| controller:pure_pursuit | 6.1 | 20.8 | 9.3 | 0.0023 | 0.77 | 33 | 4 | 0 | 0 | 1.00 |
| sac | 8.3 | 23.0 | 7.4 | 0.0540 | 0.82 | 25 | 3 | 17 | 0 | 1.01 |
| sac-res | 9.0 | 25.6 | 8.9 | 0.0335 | 0.92 | 41 | 18 | 4 | 0 | 1.00 |
| ppo | 9.6 | 27.3 | 6.4 | 0.0058 | 0.73 | 37 | 11 | 24 | 0 | 0.99 |
| ppo-res | 6.2 | 20.3 | 9.2 | 0.0022 | 0.76 | 31 | 21 | 2 | 0 | 1.01 |
