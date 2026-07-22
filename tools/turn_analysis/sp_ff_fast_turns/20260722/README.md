# Fast-turn translational-speed FF analysis

## Scope

The latest internally consistent Settings group was selected for every available 1600, 1800, and 2000 mm/s motion. Analysis is restricted to the requested turn profile and contains 22 speed/motion conditions and 174 turns.

| Speed [mm/s] | Conditions | Turns | Mean absolute error RMS [mm/s] | Mean ripple RMS [mm/s] | Mean SP PID delta RMS [V] |
|---:|---:|---:|---:|---:|---:|
| 1600 | 8 | 61 | 46.63 | 39.63 | 0.098 |
| 1800 | 10 | 65 | 78.77 | 67.94 | 0.243 |
| 2000 | 4 | 48 | 124.13 | 115.49 | 0.401 |

Ripple RMS removes each turn's mean speed offset. The increase with speed is therefore a real within-turn fluctuation, not only a steady calibration error.

## Largest fluctuation

| Speed [mm/s] | Motion | Turns | Ripple RMS [mm/s] | Absolute error RMS [mm/s] | Saturation |
|---:|:---|---:|---:|---:|---:|
| 2000 | long_l180 | 12 | 160.0 | 165.3 | 0.0% |
| 2000 | long_r180 | 12 | 157.5 | 162.6 | 0.0% |
| 1800 | in_l135 | 2 | 99.4 | 131.5 | 0.0% |
| 1800 | long_r180 | 6 | 97.9 | 99.6 | 0.0% |
| 1800 | long_l180 | 12 | 94.1 | 96.2 | 0.0% |
| 1800 | in_r135 | 2 | 93.6 | 124.4 | 0.0% |

The 1800 mm/s in45 and V90 logs contain 2.7--6.5% motor-voltage saturation. Their coefficient estimates must not be transferred to other shapes without a lower-voltage trial.

## FF signs and voltage response

The direct `configured turn FF + SP PID residual` fit consistently selects `k_alpha <= 0`, but usually drives `k_omega2` to zero. This does not disprove a positive centrifugal-load term: the existing logs have zero turn-specific SP FF, and the speed integral response makes `alpha_mag` and `omega2` strongly correlated with the residual controller voltage.

The independent voltage/response regression gives the following pooled results:

| Speed [mm/s] | Voltage gain [(m/s2)/V] | Closed-loop speed term [1/s] | Delay [ms] | R2 | Assessment |
|---:|---:|---:|---:|---:|:---|
| 1600 | -0.936 | -23.49 | 20 | 0.305 | invalid sign |
| 1800 | +0.315 | -17.93 | 20 | 0.428 | weak / saturation present |
| 2000 | +5.642 | -27.56 | 8 | 0.745 | includes the first clamped-FF r90 trial |

The selected delay reaches the 20 ms search boundary at 1600 and 1800 mm/s. The 2000 mm/s pooled fit now includes the first r90 FF trial, while the other 2000 mm/s shapes remain baseline runs, so it is still observational rather than a clean matched A/B identification.

## Applied trial parameters

These are the conservative values selected from the baseline-only fit. They remain trial parameters, not validated final values.

| Speed [mm/s] | k_alpha | k_omega2 | SP-turn FF clamp |
|---:|---:|---:|---:|
| 1600 | 0 | 0 | +/-1.0 V |
| 1800 | -0.00010034 | +0.00062859 | +/-1.0 V |
| 2000 | -0.00009606 | +0.00039806 | +/-1.0 V |

The first 2000 mm/s r90 trial requested `-0.259` to `+0.609 V`, but the old +/-0.35 V limit clipped 69.1% of turn samples. The common SP-turn FF limit is now +/-1.0 V for every speed and turn. The current 2000 mm/s coefficients therefore do not reach the clamp; the larger ceiling leaves room for the next coefficient adjustment.

Applied scope:

- 1600: long90, long180, in45, and V90 explicitly remain `0, 0`.
- 1800: long90, long180, in45, in135, and V90 use `-0.00010034, +0.00062859`.
- 2000: long90 and long180 use `-0.00009606, +0.00039806`.
- Unmeasured out45/out135 and unmeasured 2000 short turns retain zero turn-specific SP FF.

## Files

- [All condition metrics and fits](coefficient_summary.csv)
- [Speed-level response models and candidates](speed_summary.csv)
- [Full machine-readable results](summary.json)
- Per-condition reports and plots are under `<speed>/<motion>/report.md` and `voltage_response.png`.

Re-run:

```powershell
python tools\analysis\turn_sp_voltage_report.py --speeds 1600 1800 2000 --output tools\turn_analysis\sp_ff_fast_turns\20260722
```
