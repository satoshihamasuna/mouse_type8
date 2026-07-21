# search_turn left 400 mm/s

## Turn-section metrics

| Cohort | Turns | Absolute error RMS [mm/s] | Ripple RMS [mm/s] | SP feedback RMS [V] |
|---|---:|---:|---:|---:|
| baseline | 48 | 26.218 | 22.656 | 0.07188 |
| current | 4 | 19.318 | 16.799 | 0.07689 |

## Response-based FF recommendation

- k_alpha: `-0.00021316`
- k_omega2: `+0.00017144`
- predicted turn-induced speed RMS: `22.774 mm/s`
- predicted FF range: `-0.100` to `+0.350 V`
- learning rate: `50%`
- controller FF limit: `+/-0.35 V`

## Plots and numeric data

- [Time response](time_response.png)
- [Voltage model](voltage_model.png)
- [Phase-average CSV](phase_average.csv)
- [Full JSON](summary.json)
