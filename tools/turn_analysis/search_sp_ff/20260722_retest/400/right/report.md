# search_turn right 400 mm/s

## Turn-section metrics

| Cohort | Turns | Absolute error RMS [mm/s] | Ripple RMS [mm/s] | SP feedback RMS [V] |
|---|---:|---:|---:|---:|
| baseline | 48 | 30.769 | 26.471 | 0.09051 |
| current | 4 | 25.930 | 23.367 | 0.07417 |

## Response-based FF recommendation

- k_alpha: `-0.00049441`
- k_omega2: `+0.00027685`
- predicted turn-induced speed RMS: `28.314 mm/s`
- predicted FF range: `-0.233` to `+0.350 V`
- learning rate: `50%`
- controller FF limit: `+/-0.35 V`

## Plots and numeric data

- [Time response](time_response.png)
- [Voltage model](voltage_model.png)
- [Phase-average CSV](phase_average.csv)
- [Full JSON](summary.json)
