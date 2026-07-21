# search_turn left 350 mm/s

## Turn-section metrics

| Cohort | Turns | Absolute error RMS [mm/s] | Ripple RMS [mm/s] | SP feedback RMS [V] |
|---|---:|---:|---:|---:|
| baseline | 40 | 19.089 | 18.356 | 0.05334 |
| old_trial | 8 | 26.087 | 25.756 | 0.07806 |
| current | 4 | 18.496 | 15.846 | 0.07537 |

## Response-based FF recommendation

- k_alpha: `-0.00016139`
- k_omega2: `+0.00019436`
- predicted turn-induced speed RMS: `18.370 mm/s`
- predicted FF range: `-0.060` to `+0.350 V`
- learning rate: `50%`
- controller FF limit: `+/-0.35 V`

## Plots and numeric data

- [Time response](time_response.png)
- [Voltage model](voltage_model.png)
- [Phase-average CSV](phase_average.csv)
- [Full JSON](summary.json)
