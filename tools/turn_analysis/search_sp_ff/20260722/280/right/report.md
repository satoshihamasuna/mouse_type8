# search_turn right 280 mm/s

## Turn-section metrics

| Cohort | Turns | Speed error RMS [mm/s] | SP feedback RMS [V] |
|---|---:|---:|---:|
| baseline | 6 | 11.824 | 0.04088 |
| trial | 9 | 14.622 | 0.03931 |

## Response-based FF recommendation

- k_alpha: `-0.00016090`
- k_omega2: `+0.00012841`
- predicted turn-induced speed RMS: `9.793 mm/s`
- predicted FF range: `-0.039` to `+0.044 V`
- learning rate: `50%`
- controller FF limit: `+/-0.35 V`

## Plots and numeric data

- [Time response](time_response.png)
- [Voltage model](voltage_model.png)
- [Phase-average CSV](phase_average.csv)
- [Full JSON](summary.json)
