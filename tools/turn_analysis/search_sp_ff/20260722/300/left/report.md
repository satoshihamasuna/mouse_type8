# search_turn left 300 mm/s

## Turn-section metrics

| Cohort | Turns | Speed error RMS [mm/s] | SP feedback RMS [V] |
|---|---:|---:|---:|
| baseline | 6 | 12.427 | 0.04594 |
| trial | 6 | 15.670 | 0.04260 |

## Response-based FF recommendation

- k_alpha: `-0.00006533`
- k_omega2: `+0.00005630`
- predicted turn-induced speed RMS: `11.499 mm/s`
- predicted FF range: `-0.018` to `+0.020 V`
- learning rate: `50%`
- controller FF limit: `+/-0.35 V`

## Plots and numeric data

- [Time response](time_response.png)
- [Voltage model](voltage_model.png)
- [Phase-average CSV](phase_average.csv)
- [Full JSON](summary.json)
