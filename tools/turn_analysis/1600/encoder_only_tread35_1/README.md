# Encoder-only 1600 mm/s turn lengths

- effective tread: 35.1 mm
- odometry inputs: right/left encoder pulses only
- `ideal.rad_velo` is used only to locate the commanded turn interval
- `ego.velo`, gyro omega, beta, and acceleration are not integrated

| motion | side | turns | encoder angle [deg] | body end (lat, fwd) [mm] | Lstart [mm] | Lend [mm] | feasible |
| --- | :---: | ---: | ---: | ---: | ---: | ---: | :---: |
| long90 | R | 1 | 89.117 | (72.39, 61.74) | 27.99 | 17.62 | yes |
| long90 | L | 1 | 87.326 | (71.46, 62.19) | 26.94 | 18.56 | yes |
| long180 | R | 2 | 178.682 | (96.18, -5.74) | -262.65 | -268.46 | no |
| long180 | L | 4 | 177.898 | (95.54, -6.15) | -155.26 | -161.52 | no |
| in45 | R | 2 | 42.991 | (27.94, 54.39) | 17.28 | 25.05 | yes |
| in45 | L | 2 | 43.215 | (27.00, 55.16) | 15.67 | 26.30 | yes |
| out45 | R | 2 | 43.439 | (60.93, 21.23) | 33.40 | 5.45 | yes |
| out45 | L | 2 | 43.439 | (60.88, 20.78) | 34.03 | 5.06 | yes |
| in135 | R | 2 | 134.123 | (83.72, 28.77) | 22.32 | 8.75 | yes |
| in135 | L | 2 | 132.780 | (83.67, 28.95) | 21.91 | 8.63 | yes |
| out135 | R | 2 | 134.571 | (75.86, -39.42) | 19.79 | 19.57 | yes |
| out135 | L | 2 | 133.451 | (75.85, -39.06) | 19.26 | 19.57 | yes |
| v90 | R | 8 | 89.117 | (73.48, -6.56) | 16.21 | 7.05 | yes |
| v90 | L | 8 | 88.277 | (73.64, -6.96) | 16.29 | 6.65 | yes |

## R/L mean adopted values

| motion | Lstart [mm] | Lend [mm] |
| --- | ---: | ---: |
| long90 | 27.47 | 18.09 |
| long180 | infeasible | infeasible |
| in45 | 16.48 | 25.67 |
| out45 | 33.72 | 5.26 |
| in135 | 22.11 | 8.69 |
| out135 | 19.52 | 19.57 |
| v90 | 16.25 | 6.85 |

long180 is infeasible with non-negative straight lengths: the encoder-only turn body already ends at about 96 mm lateral, beyond the 90 mm target, while its exit heading is slightly below 180 degrees.
