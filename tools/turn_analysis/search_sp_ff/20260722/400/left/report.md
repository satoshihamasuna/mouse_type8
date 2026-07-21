# search_turn left 400 mm/s

## Turn-section metrics

| Cohort | Turns | Speed error RMS [mm/s] | SP feedback RMS [V] |
|---|---:|---:|---:|
| baseline | 8 | 25.445 | 0.09231 |

## Response-based FF recommendation

- k_alpha: `-0.00012914`
- k_omega2: `+0.00005493`
- predicted turn-induced speed RMS: `30.808 mm/s`
- predicted FF range: `-0.066` to `+0.069 V`
- learning rate: `50%`
- controller FF limit: `+/-0.35 V`

## Plots and numeric data

- [Time response](time_response.png)
- [Voltage model](voltage_model.png)
- [Phase-average CSV](phase_average.csv)
- [Full JSON](summary.json)
