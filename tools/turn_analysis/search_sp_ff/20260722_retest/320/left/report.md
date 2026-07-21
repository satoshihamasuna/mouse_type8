# search_turn left 320 mm/s

## Turn-section metrics

| Cohort | Turns | Absolute error RMS [mm/s] | Ripple RMS [mm/s] | SP feedback RMS [V] |
|---|---:|---:|---:|---:|
| baseline | 44 | 14.887 | 12.607 | 0.03940 |
| old_trial | 8 | 20.358 | 17.556 | 0.05966 |
| current | 4 | 15.186 | 12.282 | 0.05681 |

## Response-based FF recommendation

- k_alpha: `-0.00018364`
- k_omega2: `+0.00020642`
- predicted turn-induced speed RMS: `9.398 mm/s`
- predicted FF range: `-0.053` to `+0.350 V`
- learning rate: `50%`
- controller FF limit: `+/-0.35 V`

## Plots and numeric data

- [Time response](time_response.png)
- [Voltage model](voltage_model.png)
- [Phase-average CSV](phase_average.csv)
- [Full JSON](summary.json)
