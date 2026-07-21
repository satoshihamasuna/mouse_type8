# search_turn left 350 mm/s

## Turn-section metrics

| Cohort | Turns | Speed error RMS [mm/s] | SP feedback RMS [V] |
|---|---:|---:|---:|
| baseline | 8 | 21.352 | 0.07722 |
| trial | 8 | 26.087 | 0.07806 |

## Response-based FF recommendation

- k_alpha: `-0.00018576`
- k_omega2: `+0.00013722`
- predicted turn-induced speed RMS: `21.405 mm/s`
- predicted FF range: `-0.071` to `+0.350 V`
- learning rate: `50%`
- controller FF limit: `+/-0.35 V`

## Plots and numeric data

- [Time response](time_response.png)
- [Voltage model](voltage_model.png)
- [Phase-average CSV](phase_average.csv)
- [Full JSON](summary.json)
