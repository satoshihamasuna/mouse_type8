# search_turn right 350 mm/s

## Turn-section metrics

| Cohort | Turns | Absolute error RMS [mm/s] | Ripple RMS [mm/s] | SP feedback RMS [V] |
|---|---:|---:|---:|---:|
| baseline | 40 | 24.279 | 23.202 | 0.06950 |
| old_trial | 14 | 31.021 | 29.893 | 0.12018 |
| current | 4 | 21.466 | 16.873 | 0.08953 |

## Response-based FF recommendation

- k_alpha: `-0.00043932`
- k_omega2: `+0.00036570`
- predicted turn-induced speed RMS: `23.433 mm/s`
- predicted FF range: `-0.166` to `+0.350 V`
- learning rate: `50%`
- controller FF limit: `+/-0.35 V`

## Plots and numeric data

- [Time response](time_response.png)
- [Voltage model](voltage_model.png)
- [Phase-average CSV](phase_average.csv)
- [Full JSON](summary.json)
