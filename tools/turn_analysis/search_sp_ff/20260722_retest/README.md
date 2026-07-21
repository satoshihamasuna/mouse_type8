# search_turn translational-speed FF retest (2026-07-22)

## Conclusion

The current `k_alpha < 0`, `k_omega2 > 0` implementation reduced within-turn speed ripple in every measured speed/direction condition. The equal-weight mean fell from `18.41 mm/s` to `14.43 mm/s` (`-21.6%`). Absolute target-speed error RMS fell from `19.97 mm/s` to `17.03 mm/s` (`-14.7%`).

The current coefficient table is retained unchanged. Each new condition has only 3--4 turns, and a model identified only from the current-vs-baseline difference drove the closed-loop pole to its fit bound. That makes another coefficient update less reliable than the measured improvement already obtained.

## Baseline vs current

| Speed [mm/s] | Direction | Baseline ripple RMS [mm/s] | Current ripple RMS [mm/s] | Change |
|---:|:---:|---:|---:|---:|
| 280 | right | 11.69 | 8.62 | -26.2% |
| 280 | left  | 9.72 | 8.78 | -9.7% |
| 300 | right | 16.79 | 10.02 | -40.3% |
| 300 | left  | 11.59 | 11.37 | -1.9% |
| 320 | right | 18.51 | 13.14 | -29.0% |
| 320 | left  | 12.61 | 12.28 | -2.6% |
| 350 | right | 23.20 | 16.87 | -27.3% |
| 350 | left  | 18.36 | 15.85 | -13.7% |
| 370 | right | 27.64 | 17.22 | -37.7% |
| 370 | left  | 21.71 | 18.87 | -13.1% |
| 400 | right | 26.47 | 23.37 | -11.7% |
| 400 | left  | 22.66 | 16.80 | -25.9% |

`Ripple RMS` is calculated after removing each turn's mean speed error, so it measures fluctuation during the turn rather than a steady offset.

## Voltage-response check

The deliberately sign-wrong old trial was retained as the identification excitation and the new current run was held out as validation. The old-trial model has a positive voltage gain in all fits:

| Fit | Closed-loop pole [1/s] | Voltage gain [(m/s2)/V] | Delay [ms] | R2 |
|:---|---:|---:|---:|---:|
| all | -7.498 | +2.956 | 20 | 0.470 |
| right | -11.930 | +2.936 | 20 | 0.393 |
| left | -8.909 | +3.578 | 20 | 0.596 |

The held-out current-only identification is not used for coefficient learning because its pole reaches the imposed `-100 1/s` boundary (left/all), indicating that run-to-run differences and the small current sample cannot uniquely identify a further update.

## Files

- [Coefficient and metric summary](coefficient_summary.csv)
- [Voltage-response model](voltage_response_model.json)
- [Voltage-response validation](voltage_response_validation.png)
- Per-condition reports: `280/right/report.md` through `400/left/report.md`

