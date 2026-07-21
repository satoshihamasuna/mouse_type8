# search_turn left 320 mm/s

## Turn-section metrics

| Cohort | Turns | Speed error RMS [mm/s] | SP feedback RMS [V] |
|---|---:|---:|---:|
| baseline | 8 | 15.549 | 0.05775 |
| trial | 8 | 20.358 | 0.05966 |

## Response-based FF recommendation

- k_alpha: `-0.00016557`
- k_omega2: `+0.00009479`
- predicted turn-induced speed RMS: `11.667 mm/s`
- predicted FF range: `-0.049` to `+0.350 V`
- learning rate: `50%`
- controller FF limit: `+/-0.35 V`

## Plots and numeric data

- [Time response](time_response.png)
- [Voltage model](voltage_model.png)
- [Phase-average CSV](phase_average.csv)
- [Full JSON](summary.json)
