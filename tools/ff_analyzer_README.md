# Feedforward analyzer

`ff_analyzer.py`は、myshellで取得したCSVログから並進・旋回のFF係数を推定します。

推定モデルは次のとおりです。

```text
sp_required = sign(velocity) * FF_SP_BIAS_COEF
            + FF_SP_VELO_COEF * ideal.velo
            + FF_SP_ACCEL_COEF * ideal.accel

om_required = FF_OM_VELO_COEF * ideal.rad_velo
            + FF_OM_ACCEL_COEF * ideal.rad_accel
```

必要電圧は`feedforward + feedback`で近似します。出力飽和付近を除外し、MADによる外れ値除去を行います。ログに`ideal.rad_accel`がなければ、`ideal.rad_velo`とログ周期から計算します。

## 実行例

最新ログだけを確認します。

```powershell
python tools\ff_analyzer.py --latest
```

入力を省略して`python tools\ff_analyzer.py`だけで実行した場合も、古い設定のログを混在させないため最新ログ1本を使用します。

複数ログをまとめて同定します。速度一定、並進加減速、角速度一定、角加減速が含まれるログを組み合わせてください。

```powershell
python tools\ff_analyzer.py tools\logs\straight_*.csv tools\logs\turn_*.csv
```

グラフを表示する場合：

```powershell
python tools\ff_analyzer.py tools\logs\straight_*.csv --plot
```

JSONにも保存できます。

```powershell
python tools\ff_analyzer.py tools\logs\straight_*.csv --json tools\logs\ff_result.json
```

ログ周期が2 ms以外の場合：

```powershell
python tools\ff_analyzer.py tools\logs\straight_*.csv --period-ms 1.0
```

## 出力の見方

- `RMSE`: モデルで説明できなかった電圧のRMS。小さいほど良好です。
- `R2`: 必要電圧をモデルが説明できた割合。1に近いほど良好です。
- `cond`: 速度項と加速度項の分離しやすさ。30を大きく超える場合は実験条件を増やします。
- `rejected`: 外れ値として除外されたサンプル数です。
- `Residual feedback by phase`: 現在のFFでPIDが補っている残差です。
- `unmodelled bias`: 速度・加速度に比例しない一定電圧成分です。警告が出た場合、速度係数だけで合わせず、`DEAD_V`または符号付きクーロン摩擦項を確認します。

`WARNING: ... excitation is sparse`が出た係数は、そのログだけでは十分に決められません。例えばターンログだけで`FF_SP_ACCEL_COEF`を決定せず、直進加減速ログも追加します。

直進ログだけなど、片方の運動成分が含まれない場合は、そのチャンネルを`unavailable`と表示して、同定できた側だけを出力します。空のCSVやデータ行が1行しかないCSVは`SKIPPED`として表示します。

## 注意

これは閉ループログからの近似同定です。壁制御、タイヤスリップ、電圧飽和が強い区間では推定が偏ります。算出値をそのまま確定値にせず、変更後に以下を再確認してください。

異なるFF設定、異なるモーション種別、古いモータドライバのログを無条件に一括投入しないでください。直進用とターン用を別々に選び、同じ条件のログ同士で解析します。

`--with-intercept`で表示される切片は`FF_SP_BIAS_COEF`の候補です。速度が正なら加算、負なら減算し、発進時は加速度の符号を使用します。`DEAD_V`と役割が重なるため、変更後は低速域と停止時の出力も確認します。

- `sp_feedback`、`om_feedback`の平均とRMSが減るか
- `ideal`と`ego`の追従誤差が減るか
- 加速・減速、右・左ターンで符号対称性があるか
- `V_r`、`V_l`がバッテリー電圧に張り付いていないか
