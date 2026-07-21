# search_turn right 300 mm/s

## Turn-section metrics

| Cohort | Turns | Absolute error RMS [mm/s] | Ripple RMS [mm/s] | SP feedback RMS [V] |
|---|---:|---:|---:|---:|
| baseline | 12 | 16.920 | 16.785 | 0.04924 |
| old_trial | 6 | 21.276 | 21.237 | 0.05858 |
| current | 3 | 12.208 | 10.018 | 0.05622 |

## Response-based FF recommendation

- k_alpha: `-0.00020727`
- k_omega2: `+0.00032866`
- predicted turn-induced speed RMS: `14.669 mm/s`
- predicted FF range: `-0.056` to `+0.070 V`
- learning rate: `50%`
- controller FF limit: `+/-0.35 V`

## Plots and numeric data

- [Time response](time_response.png)
- [Voltage model](voltage_model.png)
- [Phase-average CSV](phase_average.csv)
- [Full JSON](summary.json)
