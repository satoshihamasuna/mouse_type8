# search_turn right 300 mm/s

## Turn-section metrics

| Cohort | Turns | Speed error RMS [mm/s] | SP feedback RMS [V] |
|---|---:|---:|---:|
| baseline | 6 | 17.208 | 0.05869 |
| trial | 6 | 21.276 | 0.05858 |

## Response-based FF recommendation

- k_alpha: `-0.00022961`
- k_omega2: `+0.00017918`
- predicted turn-induced speed RMS: `14.474 mm/s`
- predicted FF range: `-0.064` to `+0.072 V`
- learning rate: `50%`
- controller FF limit: `+/-0.35 V`

## Plots and numeric data

- [Time response](time_response.png)
- [Voltage model](voltage_model.png)
- [Phase-average CSV](phase_average.csv)
- [Full JSON](summary.json)
