# search_turn left 280 mm/s

## Turn-section metrics

| Cohort | Turns | Speed error RMS [mm/s] | SP feedback RMS [V] |
|---|---:|---:|---:|
| baseline | 6 | 10.688 | 0.03768 |
| trial | 6 | 11.580 | 0.03223 |

## Response-based FF recommendation

- k_alpha: `-0.00004510`
- k_omega2: `+0.00003266`
- predicted turn-induced speed RMS: `9.275 mm/s`
- predicted FF range: `-0.011` to `+0.012 V`
- learning rate: `50%`
- controller FF limit: `+/-0.35 V`

## Plots and numeric data

- [Time response](time_response.png)
- [Voltage model](voltage_model.png)
- [Phase-average CSV](phase_average.csv)
- [Full JSON](summary.json)
