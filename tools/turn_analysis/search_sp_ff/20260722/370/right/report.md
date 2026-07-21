# search_turn right 370 mm/s

## Turn-section metrics

| Cohort | Turns | Speed error RMS [mm/s] | SP feedback RMS [V] |
|---|---:|---:|---:|
| baseline | 8 | 30.388 | 0.10725 |
| trial | 8 | 37.474 | 0.12310 |

## Response-based FF recommendation

- k_alpha: `-0.00053817`
- k_omega2: `+0.00020249`
- predicted turn-induced speed RMS: `26.105 mm/s`
- predicted FF range: `-0.235` to `+0.246 V`
- learning rate: `50%`
- controller FF limit: `+/-0.35 V`

## Plots and numeric data

- [Time response](time_response.png)
- [Voltage model](voltage_model.png)
- [Phase-average CSV](phase_average.csv)
- [Full JSON](summary.json)
