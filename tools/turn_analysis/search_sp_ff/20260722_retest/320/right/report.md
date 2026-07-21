# search_turn right 320 mm/s

## Turn-section metrics

| Cohort | Turns | Absolute error RMS [mm/s] | Ripple RMS [mm/s] | SP feedback RMS [V] |
|---|---:|---:|---:|---:|
| baseline | 56 | 21.403 | 18.508 | 0.05675 |
| old_trial | 8 | 26.193 | 21.675 | 0.07743 |
| current | 4 | 15.656 | 13.136 | 0.06302 |

## Response-based FF recommendation

- k_alpha: `-0.00039273`
- k_omega2: `+0.00035521`
- predicted turn-induced speed RMS: `15.361 mm/s`
- predicted FF range: `-0.121` to `+0.350 V`
- learning rate: `50%`
- controller FF limit: `+/-0.35 V`

## Plots and numeric data

- [Time response](time_response.png)
- [Voltage model](voltage_model.png)
- [Phase-average CSV](phase_average.csv)
- [Full JSON](summary.json)
