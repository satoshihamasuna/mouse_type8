# Search-turn translational voltage/response analysis (2026-07-22)

## Scope

- Motion: `search_turn`, turn section only (`abs(ideal.rad_velo) >= 0.05 rad/s`)
- Speeds: 280, 300, 320, 350, 370 and 400 mm/s
- Directions: right and left
- Baseline logs: `20260721_22*search_turn*.csv`
- First-trial logs: `20260721_23*search_turn*.csv` and
  `20260722_00*search_turn*.csv`
- Every CSV is paired with its same-stem `.settings.json`.  The settings values
  are used to reconstruct ordinary SP FF and the configured turn-only FF.

The first-trial logs are available through 370 mm/s.  The 400 mm/s result is an
extrapolation of the direction-specific voltage-response model and therefore
uses the same conservative 50% learning update as the measured speeds.

## Main finding

The turn-acceleration term must use angular-speed magnitude acceleration,
not absolute angular acceleration:

```text
alpha_mag = sign(target omega) * target alpha
turn_sp_ff = k_alpha * alpha_mag + k_omega2 * target omega^2
```

`alpha_mag` is positive at turn entry and negative at exit for both right and
left turns.  This matches the measured response: the vehicle is too fast in the
first half and too slow in the second half.  Therefore `k_alpha < 0` removes
voltage on entry and adds voltage on exit.  The response-based fit gives
`k_omega2 > 0` at every speed and direction.

The previous trial used `abs(alpha)` and a negative omega-squared coefficient.
It increased speed-error RMS at all ten measured speed/direction combinations;
for example right 370 mm/s changed from 30.39 to 37.47 mm/s.  It is retained in
the plots as the `trial` cohort but is not used as the new model structure.

## Voltage-response identification

The added FF voltage in the first trial is treated as an excitation signal.
Trial-minus-baseline velocity is fitted with a first-order closed-loop model:

```text
d(delta_v)/dt = a_closed_loop * delta_v + b_voltage * delta_turn_ff(t - delay)
```

The direction-specific models used for coefficient selection are:

| Direction | a [1/s] | b [(m/s^2)/V] | Delay | Response R2 |
|---|---:|---:|---:|---:|
| Right | -11.5717 | 3.7909 | 5 ms | 0.411 |
| Left | -13.9251 | 5.3925 | 20 ms | 0.661 |

The combined model has R2 = 0.509.  Because the response model is useful but
not precise enough for a full one-shot update, the code values use 50% of the
bounded optimum.  Turn-only SP FF is limited to +/-0.35 V in the controller.

## Coefficients applied to code

| Speed [mm/s] | Direction | k_alpha | k_omega2 |
|---:|:---:|---:|---:|
| 280 | R | -0.00016090 | +0.00012841 |
| 280 | L | -0.00004510 | +0.00003266 |
| 300 | R | -0.00022961 | +0.00017918 |
| 300 | L | -0.00006533 | +0.00005630 |
| 320 | R | -0.00036311 | +0.00021241 |
| 320 | L | -0.00016557 | +0.00009479 |
| 350 | R | -0.00045500 | +0.00021685 |
| 350 | L | -0.00018576 | +0.00013722 |
| 370 | R | -0.00053817 | +0.00020249 |
| 370 | L | -0.00016902 | +0.00012027 |
| 400 | R | -0.00056554 | +0.00007881 |
| 400 | L | -0.00012914 | +0.00005493 |

## Files

- `coefficient_summary.csv`: one row per speed, direction and cohort
- `summary.json`: complete metrics, unconstrained voltage fit and response fit
- `voltage_response_model.json`: identified closed-loop response parameters
- `voltage_response_validation.png`: excitation voltage versus measured response
- `<speed>/<direction>/phase_average.csv`: phase-normalized numeric traces
- `<speed>/<direction>/time_response.png`: voltage and response on one turn axis
- `<speed>/<direction>/voltage_model.png`: required-voltage model diagnostic
- `<speed>/<direction>/summary.json`: settings sources, metrics and fits

The direct required-voltage regression is retained as a diagnostic.  It prefers
a negative omega-squared coefficient because feedback voltage is phase-lagged
and the vehicle overspeeds near the turn centre.  It is not used for the final
coefficient sign.  The applied coefficients come from the independently
measured voltage-to-speed response and the full turn error waveform.
