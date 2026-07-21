# search_turn right 320 mm/s

## Turn-section metrics

| Cohort | Turns | Speed error RMS [mm/s] | SP feedback RMS [V] |
|---|---:|---:|---:|
| baseline | 8 | 21.388 | 0.07140 |
| trial | 8 | 26.193 | 0.07743 |

## Response-based FF recommendation

- k_alpha: `-0.00036311`
- k_omega2: `+0.00021241`
- predicted turn-induced speed RMS: `12.986 mm/s`
- predicted FF range: `-0.107` to `+0.350 V`
- learning rate: `50%`
- controller FF limit: `+/-0.35 V`

## Plots and numeric data

- [Time response](time_response.png)
- [Voltage model](voltage_model.png)
- [Phase-average CSV](phase_average.csv)
- [Full JSON](summary.json)
