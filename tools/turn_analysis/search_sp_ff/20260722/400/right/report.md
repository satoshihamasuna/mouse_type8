# search_turn right 400 mm/s

## Turn-section metrics

| Cohort | Turns | Speed error RMS [mm/s] | SP feedback RMS [V] |
|---|---:|---:|---:|
| baseline | 8 | 32.227 | 0.12314 |

## Response-based FF recommendation

- k_alpha: `-0.00056554`
- k_omega2: `+0.00007881`
- predicted turn-induced speed RMS: `32.353 mm/s`
- predicted FF range: `-0.292` to `+0.297 V`
- learning rate: `50%`
- controller FF limit: `+/-0.35 V`

## Plots and numeric data

- [Time response](time_response.png)
- [Voltage model](voltage_model.png)
- [Phase-average CSV](phase_average.csv)
- [Full JSON](summary.json)
