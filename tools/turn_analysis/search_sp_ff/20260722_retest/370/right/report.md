# search_turn right 370 mm/s

## Turn-section metrics

| Cohort | Turns | Absolute error RMS [mm/s] | Ripple RMS [mm/s] | SP feedback RMS [V] |
|---|---:|---:|---:|---:|
| baseline | 60 | 29.770 | 27.644 | 0.09417 |
| old_trial | 8 | 37.474 | 36.871 | 0.12310 |
| current | 4 | 21.967 | 17.218 | 0.07852 |

## Response-based FF recommendation

- k_alpha: `-0.00042835`
- k_omega2: `+0.00044768`
- predicted turn-induced speed RMS: `25.331 mm/s`
- predicted FF range: `-0.181` to `+0.208 V`
- learning rate: `50%`
- controller FF limit: `+/-0.35 V`

## Plots and numeric data

- [Time response](time_response.png)
- [Voltage model](voltage_model.png)
- [Phase-average CSV](phase_average.csv)
- [Full JSON](summary.json)
