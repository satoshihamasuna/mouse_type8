# search_turn left 280 mm/s

## Turn-section metrics

| Cohort | Turns | Absolute error RMS [mm/s] | Ripple RMS [mm/s] | SP feedback RMS [V] |
|---|---:|---:|---:|---:|
| baseline | 12 | 10.061 | 9.717 | 0.03030 |
| old_trial | 6 | 11.580 | 11.531 | 0.03223 |
| current | 3 | 9.682 | 8.777 | 0.03926 |

## Response-based FF recommendation

- k_alpha: `-0.00001704`
- k_omega2: `+0.00007938`
- predicted turn-induced speed RMS: `7.890 mm/s`
- predicted FF range: `-0.004` to `+0.010 V`
- learning rate: `50%`
- controller FF limit: `+/-0.35 V`

## Plots and numeric data

- [Time response](time_response.png)
- [Voltage model](voltage_model.png)
- [Phase-average CSV](phase_average.csv)
- [Full JSON](summary.json)
