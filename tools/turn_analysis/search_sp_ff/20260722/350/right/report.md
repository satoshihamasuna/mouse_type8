# search_turn right 350 mm/s

## Turn-section metrics

| Cohort | Turns | Speed error RMS [mm/s] | SP feedback RMS [V] |
|---|---:|---:|---:|
| baseline | 8 | 25.438 | 0.09223 |
| trial | 14 | 31.021 | 0.12018 |

## Response-based FF recommendation

- k_alpha: `-0.00045500`
- k_omega2: `+0.00021685`
- predicted turn-induced speed RMS: `21.303 mm/s`
- predicted FF range: `-0.175` to `+0.350 V`
- learning rate: `50%`
- controller FF limit: `+/-0.35 V`

## Plots and numeric data

- [Time response](time_response.png)
- [Voltage model](voltage_model.png)
- [Phase-average CSV](phase_average.csv)
- [Full JSON](summary.json)
