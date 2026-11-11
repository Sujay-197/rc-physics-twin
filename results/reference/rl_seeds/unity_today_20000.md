Task stop, car unity, 4 m/s, 100 trips per condition per seed (seeds 20000 to 20099). Baseline `recover:controller:pure_pursuit:policies/controllers_unity_stop.json`.

| method | seeds | train: pooled (95 % CI) | held_out: pooled (95 % CI) | perturbed: pooled (95 % CI) | train: per seed | held_out: per seed | perturbed: per seed |
|---|---|---|---|---|---|---|---|
| recover:controller:pure_pursuit:policies/controllers_unity_stop.json | - | 100.0 (96-100) | 88.0 (80-93) | 88.0 (80-93) | 100 | 88 | 88 |
| controller:pure_pursuit:policies/controllers_unity_stop.json | - | 99.0 (95-100) | 65.0 (55-74) | 64.0 (54-73) | 99 | 65 | 64 |
| recover:ppo-res-stop-unity-r3 | 5 | 100.0 (99-100) | 84.8 (81-88) | 89.0 (86-91) | 100, 100, 100, 100, 100 | 84, 82, 83, 87, 88 | 88, 85, 89, 94, 89 |
| ppo-res-stop-unity-r3 | 5 | 98.6 (97-99) | 63.4 (59-68) | 66.0 (62-70) | 98, 98, 99, 100, 98 | 67, 63, 60, 61, 66 | 64, 65, 66, 70, 65 |
| ppo-stop-unity-r3 | 5 | 73.8 (70-77) | 36.8 (33-41) | 36.4 (32-41) | 81, 95, 67, 39, 87 | 76, 19, 14, 31, 44 | 71, 14, 26, 26, 45 |
| recover:ppo-stop-unity-r3 | 5 | 73.8 (70-77) | 36.8 (33-41) | 36.4 (32-41) | 81, 95, 67, 39, 87 | 76, 19, 14, 31, 44 | 71, 14, 26, 26, 45 |
| recover:corner:3.4,1.2,1.2,0.5:controller:pure_pursuit:policies/controllers_unity_stop.json | - | 100.0 (96-100) | 100.0 (96-100) | 100.0 (96-100) | 100 | 100 | 100 |

Against the baseline, trip by trip: + finished where the baseline failed, − the reverse; two-sided exact sign test.

| method | train: pooled | held_out: pooled | perturbed: pooled | train: per seed | held_out: per seed | perturbed: per seed |
|---|---|---|---|---|---|---|
| controller:pure_pursuit:policies/controllers_unity_stop.json | +0 / −1, p = 1.00 | +0 / −23, p < 0.001 | +0 / −24, p < 0.001 | +0/−1 | +0/−23 (p < 0.001) | +0/−24 (p < 0.001) |
| recover:ppo-res-stop-unity-r3 | +0 / −0, p = 1.00 | +26 / −42, p = 0.068 | +33 / −28, p = 0.61 | +0/−0; +0/−0; +0/−0; +0/−0; +0/−0 | +3/−7; +4/−10; +6/−11; +7/−8; +6/−6 | +5/−5; +5/−8; +7/−6; +9/−3; +7/−6 |
| ppo-res-stop-unity-r3 | +0 / −7, p = 0.016 | +1 / −124, p < 0.001 | +9 / −119, p < 0.001 | +0/−2; +0/−2; +0/−1; +0/−0; +0/−2 | +0/−21 (p < 0.001); +0/−25 (p < 0.001); +0/−28 (p < 0.001); +0/−27 (p < 0.001); +1/−23 (p < 0.001) | +1/−25 (p < 0.001); +0/−23 (p < 0.001); +2/−24 (p < 0.001); +4/−22 (p < 0.001); +2/−25 (p < 0.001) |
| ppo-stop-unity-r3 | +0 / −131, p < 0.001 | +17 / −273, p < 0.001 | +15 / −273, p < 0.001 | +0/−19 (p < 0.001); +0/−5 (p = 0.062); +0/−33 (p < 0.001); +0/−61 (p < 0.001); +0/−13 (p < 0.001) | +8/−20 (p = 0.036); +1/−70 (p < 0.001); +1/−75 (p < 0.001); +3/−60 (p < 0.001); +4/−48 (p < 0.001) | +7/−24 (p = 0.003); +2/−76 (p < 0.001); +1/−63 (p < 0.001); +4/−66 (p < 0.001); +1/−44 (p < 0.001) |
| recover:ppo-stop-unity-r3 | +0 / −131, p < 0.001 | +17 / −273, p < 0.001 | +15 / −273, p < 0.001 | +0/−19 (p < 0.001); +0/−5 (p = 0.062); +0/−33 (p < 0.001); +0/−61 (p < 0.001); +0/−13 (p < 0.001) | +8/−20 (p = 0.036); +1/−70 (p < 0.001); +1/−75 (p < 0.001); +3/−60 (p < 0.001); +4/−48 (p < 0.001) | +7/−24 (p = 0.003); +2/−76 (p < 0.001); +1/−63 (p < 0.001); +4/−66 (p < 0.001); +1/−44 (p < 0.001) |
| recover:corner:3.4,1.2,1.2,0.5:controller:pure_pursuit:policies/controllers_unity_stop.json | +0 / −0, p = 1.00 | +12 / −0, p < 0.001 | +12 / −0, p < 0.001 | +0/−0 | +12/−0 (p < 0.001) | +12/−0 (p < 0.001) |

The same with the trip as the unit (a trip counts once, however many seeds drove it): the difference in success rate with a bootstrap 95 % interval over trips; trips where the share of seeds that finished beats the baseline (+) or loses to it (−), with a sign test; a paired sign-flip permutation test of the per-trip differences; and seeds with more + than − trips, and fewer.

| method | train: difference [pp] (95 % CI), permutation p | held_out: difference [pp] (95 % CI), permutation p | perturbed: difference [pp] (95 % CI), permutation p | train: trips +/− | held_out: trips +/− | perturbed: trips +/− | train: seeds better/worse | held_out: seeds better/worse | perturbed: seeds better/worse |
|---|---|---|---|---|---|---|---|---|---|
| controller:pure_pursuit:policies/controllers_unity_stop.json | -1.0 (-3.0 to +0.0), p = 1.00 | -23.0 (-31.0 to -15.0), p < 0.001 | -24.0 (-33.0 to -16.0), p < 0.001 | +0 / −1, p = 1.00 | +0 / −23, p < 0.001 | +0 / −24, p < 0.001 | 0 / 1 of 1 | 0 / 1 of 1 | 0 / 1 of 1 |
| recover:ppo-res-stop-unity-r3 | +0.0 (+0.0 to +0.0), p = 1.00 | -3.2 (-8.8 to +2.4), p = 0.31 | +1.0 (-4.0 to +6.2), p = 0.76 | +0 / −0, p = 1.00 | +10 / −20, p = 0.099 | +11 / −18, p = 0.26 | 0 / 0 of 5 | 0 / 4 of 5 | 3 / 1 of 5 |
| ppo-res-stop-unity-r3 | -1.4 (-3.4 to +0.0), p = 0.25 | -24.6 (-32.6 to -17.0), p < 0.001 | -22.0 (-30.8 to -13.6), p < 0.001 | +0 / −3, p = 0.25 | +1 / −32, p < 0.001 | +4 / −26, p < 0.001 | 0 / 4 of 5 | 0 / 5 of 5 | 0 / 5 of 5 |
| ppo-stop-unity-r3 | -26.2 (-29.6 to -23.0), p < 0.001 | -51.2 (-58.2 to -43.8), p < 0.001 | -51.6 (-59.2 to -43.8), p < 0.001 | +0 / −92, p < 0.001 | +10 / −87, p < 0.001 | +8 / −87, p < 0.001 | 0 / 5 of 5 | 0 / 5 of 5 | 0 / 5 of 5 |
| recover:ppo-stop-unity-r3 | -26.2 (-29.6 to -23.0), p < 0.001 | -51.2 (-58.2 to -43.8), p < 0.001 | -51.6 (-59.2 to -43.8), p < 0.001 | +0 / −92, p < 0.001 | +10 / −87, p < 0.001 | +8 / −87, p < 0.001 | 0 / 5 of 5 | 0 / 5 of 5 | 0 / 5 of 5 |
| recover:corner:3.4,1.2,1.2,0.5:controller:pure_pursuit:policies/controllers_unity_stop.json | +0.0 (+0.0 to +0.0), p = 1.00 | +12.0 (+6.0 to +19.0), p < 0.001 | +12.0 (+6.0 to +19.0), p < 0.001 | +0 / −0, p = 1.00 | +12 / −0, p < 0.001 | +12 / −0, p < 0.001 | 0 / 0 of 1 | 1 / 0 of 1 | 1 / 0 of 1 |

Driving quality, held_out cars, over the trips each finished (pooled over seeds); trip time is the median ratio to the baseline over trips both finished.

| method | mean offset [cm] | p95 offset [cm] | speed error [%] | steer roughness | energy [mWh/m] | wall contact [% trips] | left road | timed out | overshot | trip time |
|---|---|---|---|---|---|---|---|---|---|---|
| recover:controller:pure_pursuit:policies/controllers_unity_stop.json | 7.3 | 21.3 | 42.8 | 0.0061 | 22.47 | 42 | 9 | 0 | 3 | 1.00 |
| controller:pure_pursuit:policies/controllers_unity_stop.json | 1.8 | 11.1 | 32.2 | 0.0011 | 1.68 | 42 | 4 | 28 | 3 | 1.00 |
| recover:ppo-res-stop-unity-r3 | 7.9 | 22.1 | 42.3 | 0.0064 | 20.39 | 43 | 58 | 2 | 16 | 0.99 |
| ppo-res-stop-unity-r3 | 2.9 | 12.9 | 31.9 | 0.0017 | 1.96 | 43 | 23 | 144 | 16 | 0.99 |
| ppo-stop-unity-r3 | 11.3 | 30.8 | 43.6 | 0.0092 | 1.19 | 53 | 41 | 163 | 112 | 1.15 |
| recover:ppo-stop-unity-r3 | 11.3 | 30.8 | 43.6 | 0.0092 | 1.19 | 53 | 41 | 163 | 112 | 1.15 |
| recover:corner:3.4,1.2,1.2,0.5:controller:pure_pursuit:policies/controllers_unity_stop.json | 2.0 | 9.8 | 42.3 | 0.0010 | 0.51 | 0 | 0 | 0 | 0 | 1.22 |
