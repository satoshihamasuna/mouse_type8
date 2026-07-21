# 1600/2000 mm/s longturn180 R/L SP-FF analysis

2026-07-22の最新longturn180ログだけを使い、方向ごとに次のSPターンFFを推定した。

```text
V_sp_turn = k_alpha * sign(omega) * alpha + k_omega2 * omega^2
```

| speed | direction | k_alpha | k_omega2 | voltage response | response R2 |
|---:|:---:|---:|---:|---:|---:|
| 1600 | R | -0.00019277 | +0.00003695 | +3.101 (m/s2)/V | 0.494 |
| 1600 | L | -0.00018248 | +0.00005183 | +2.375 (m/s2)/V | 0.598 |
| 2000 | R | -0.00033918 | +0.00018206 | +7.078 (m/s2)/V | 0.730 |
| 2000 | L | -0.00028747 | +0.00013429 | +7.224 (m/s2)/V | 0.550 |

係数はPID補正値だけではなく、ターン前のオフセットを除いたモータ共通電圧、速度変化、実加速度の電圧応答を対応させて求めた。符号制約は`k_alpha <= 0`、`k_omega2 >= 0`とし、ログごとの定常電圧差は固定効果として分離した。4条件とも電圧から加速度へのゲインは正であり、期待した係数符号と一致した。

longturn180上のFF電圧は1600で約`-0.23～+0.24 V`、2000で約`-0.56～+0.65 V`となり、共通`±1.0 V`クランプには当たらない。係数は同じ速度・方向の全ターン形状へ振り分けた。

既存の各形状ログに新係数を投影すると、1600は測定済み全形状でクランプなし。2000 L90も最大`+0.958 V`で範囲内。一方、2000 R90は約16.4%が`±1.0 V`へ到達する予測なので、次の実走ログではクランプ率と速度リップルを再確認する必要がある。未計測の2000 short-turn形状は方向モデルを初期値として適用している。

## Files

- [R/L coefficient summary](direction_coefficients.csv)
- [Projection to measured turn shapes](measured_shape_projection.csv)
- [1600 R voltage/response plot](1600/long_r180/voltage_response.png)
- [1600 L voltage/response plot](1600/long_l180/voltage_response.png)
- [2000 R voltage/response plot](2000/long_r180/voltage_response.png)
- [2000 L voltage/response plot](2000/long_l180/voltage_response.png)

各方向のサブディレクトリには、使用ログ名を含む`summary.json`、位相平均値、ターンごとの指標、電圧応答図を保存している。
