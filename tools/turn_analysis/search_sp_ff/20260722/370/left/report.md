# search_turn left 370 mm/s

## Turn-section metrics

| Cohort | Turns | Speed error RMS [mm/s] | SP feedback RMS [V] |
|---|---:|---:|---:|
| baseline | 8 | 24.815 | 0.08933 |
| trial | 8 | 31.079 | 0.09146 |

## Response-based FF recommendation

- k_alpha: `-0.00016902`
- k_omega2: `+0.00012027`
- predicted turn-induced speed RMS: `26.477 mm/s`
- predicted FF range: `-0.073` to `+0.079 V`
- learning rate: `50%`
- controller FF limit: `+/-0.35 V`

## Plots and numeric data

- [Time response](time_response.png)
- [Voltage model](voltage_model.png)
- [Phase-average CSV](phase_average.csv)
- [Full JSON](summary.json)
