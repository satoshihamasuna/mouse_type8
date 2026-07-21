# search_turn left 300 mm/s

## Turn-section metrics

| Cohort | Turns | Absolute error RMS [mm/s] | Ripple RMS [mm/s] | SP feedback RMS [V] |
|---|---:|---:|---:|---:|
| baseline | 12 | 11.916 | 11.591 | 0.03691 |
| old_trial | 6 | 15.670 | 15.574 | 0.04260 |
| current | 3 | 12.429 | 11.365 | 0.04651 |

## Response-based FF recommendation

- k_alpha: `-0.00005311`
- k_omega2: `+0.00009601`
- predicted turn-induced speed RMS: `10.678 mm/s`
- predicted FF range: `-0.014` to `+0.019 V`
- learning rate: `50%`
- controller FF limit: `+/-0.35 V`

## Plots and numeric data

- [Time response](time_response.png)
- [Voltage model](voltage_model.png)
- [Phase-average CSV](phase_average.csv)
- [Full JSON](summary.json)
