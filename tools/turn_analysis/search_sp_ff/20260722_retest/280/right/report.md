# search_turn right 280 mm/s

## Turn-section metrics

| Cohort | Turns | Absolute error RMS [mm/s] | Ripple RMS [mm/s] | SP feedback RMS [V] |
|---|---:|---:|---:|---:|
| baseline | 12 | 11.897 | 11.694 | 0.03468 |
| old_trial | 9 | 14.622 | 14.574 | 0.03931 |
| current | 3 | 10.283 | 8.624 | 0.04481 |

## Response-based FF recommendation

- k_alpha: `-0.00011754`
- k_omega2: `+0.00022613`
- predicted turn-induced speed RMS: `10.042 mm/s`
- predicted FF range: `-0.027` to `+0.036 V`
- learning rate: `50%`
- controller FF limit: `+/-0.35 V`

## Plots and numeric data

- [Time response](time_response.png)
- [Voltage model](voltage_model.png)
- [Phase-average CSV](phase_average.csv)
- [Full JSON](summary.json)
