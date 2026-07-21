# search_turn left 370 mm/s

## Turn-section metrics

| Cohort | Turns | Absolute error RMS [mm/s] | Ripple RMS [mm/s] | SP feedback RMS [V] |
|---|---:|---:|---:|---:|
| baseline | 40 | 22.456 | 21.710 | 0.06338 |
| old_trial | 8 | 31.079 | 30.715 | 0.09146 |
| current | 4 | 21.757 | 18.869 | 0.08469 |

## Response-based FF recommendation

- k_alpha: `-0.00019615`
- k_omega2: `+0.00019698`
- predicted turn-induced speed RMS: `23.298 mm/s`
- predicted FF range: `-0.083` to `+0.095 V`
- learning rate: `50%`
- controller FF limit: `+/-0.35 V`

## Plots and numeric data

- [Time response](time_response.png)
- [Voltage model](voltage_model.png)
- [Phase-average CSV](phase_average.csv)
- [Full JSON](summary.json)
