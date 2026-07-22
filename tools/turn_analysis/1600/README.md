# 1600 mm/s ターン・接地スリップモデル

## 目的

1600 mm/sで取得した左右の実測ログを使い、各ターンの軌跡から共通の
`Lstart` と `Lend` を決める。STMの運動制御モデルは変更せず、Python側で
軌跡を再生して `Params/Param_A/turn_1600.h` の距離パラメータを求める。

## 速度モデル

ログの `ego.velo` は、地面に対する速度ベクトルの大きさとして扱う。
機体姿勢から速度ベクトルが横滑り角 `beta` だけずれるため、まず接地面での
並進スリップ倍率を

```text
s(beta) = max(0, 1 - C_ground * beta^2)
```

とし、機体座標系の速度を次式で計算する。

```text
v_forward = ego.velo * s(beta) * cos(beta)
v_lateral = ego.velo * s(beta) * sin(beta)
```

前後・横成分へ同じ `s(beta)` を掛けることで、速度ベクトルの方向は
機体姿勢から `beta` ずれたままになる。今回使用する係数は次のとおり。

```text
C_ground = 3.125327524776992
```

この係数は、2026-07-22のlong180動画で確認した90 mmの横移動に対し、
同時期のログを再生して求めた。個別の同定値は右3.091784、左3.158871で、
左右差をターンパラメータへ持ち込まないよう平均値を採用した。

## 使用する実測量

- `ego.velo`: 合成速度の大きさ
- `ego.rad_velo`: ヨーレート。積分して旋回角を再現する
- `ego.turn_slip_theta`: 横滑り角 `beta`

加速度はログ取得と横滑り状態の確認に使用するが、この軌跡計算では
`ego.horizon_accel` を再積分して横速度を作っていない。横速度は上式から
直接求めるため、加速度オフセットの積分ドリフトを持ち込まない。

### 旋回終了時のbeta

in45、out45、V90のログでは、旋回終了サンプルで
`ego.turn_slip_theta` が約-0.09 radから0へ強制的にリセットされる。この0を
物理的な横滑り消失として使うと、速度方向が約5度飛び、軌跡終端に不自然な
折れが発生する。

そこで、強制ゼロのサンプルだけを次式による1 ms後の値へ置き換える。

```text
beta_dot = -k * beta / ego.velo - omega
beta_next = beta + beta_dot * 0.001
```

その後のLend区間も同じ式でbetaを連続的に減衰させる。Lendがシミュレータの
減衰区間より短い場合は、指定距離で軌跡を補間して打ち切る。

## 現在turn_1600.hへ反映したLstart/Lend

元MATLAB式を1600 mm/s、`Kp=4.0`、`Ki=0.01`、`alpha=1.0`、
`k_R=239.34375`、`k_L=260.464286`で計算し、右・左の平均を共通値にした。
実測ログと接地スリップモデルによる値ではない。単位はmm。

| ターン | r_min | Lstart | Lend |
| --- | ---: | ---: | ---: |
| long90 | 52.0 | 14.69 | 34.06 |
| long180 | 48.0 | 10.79 | 32.40 |
| in45 | 55.0 | 25.41 | 23.05 |
| out45 | 60.0 | 21.00 | 20.61 |
| in135 | 42.0 | 11.60 | 25.47 |
| out135 | 39.3 | 12.47 | 40.99 |
| V90 | 40.0 | 4.51 | 23.78 |

## 再計算

MATLAB移植式は `tools/turn_simulator.py` の `MATLAB_DYNAMICS` を指定し、
各方向のkで計算する。設定後の整合性確認はリポジトリのルートで次を実行する。

```powershell
python -m pytest -q
```

主なファイルは以下のとおり。

- `tools/turn_simulator.py`: MATLAB移植を基にしたターン積分器
- `tools/analysis/turn1600_variable_speed_lengths.py`: 最新ログの抽出、モデル再生、左右平均距離の算出
- `resultant_speed_ground_slip_lengths/report.md`: 方向別の数値結果
- `resultant_speed_ground_slip_lengths/trajectories/`: 理想軌跡と予測軌跡
- `resultant_speed_ground_slip_lengths/velocity_profiles/`: 合成・前後・横速度
- `video_20260722_175611/`: long180動画との照合結果

## 適用上の注意

`C_ground` はlong180から同定したため、他形状への適用は外挿を含む。特に
in135、out135、V90は `beta` が大きく、速度低下量が大きくなる。今回の値を
初期値として実走し、壁との距離と終点誤差を確認してから最終固定する。

`C_ground` とLstart/Lendを同じログへ同時に過度適合させないため、新しい
long180動画で係数を確認し、その他のターンで距離だけを再調整するのが望ましい。
