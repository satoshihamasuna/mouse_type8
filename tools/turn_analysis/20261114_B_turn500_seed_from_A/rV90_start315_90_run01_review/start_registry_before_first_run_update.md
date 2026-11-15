# 開始配置の記録と再利用

開始位置を案内する前に、この記録と該当する試行の計画・静止画像を読む。成功した配置と、提案だけ/却下/未確認の配置を区別する。次の機体やターンへ無条件に適用しない。

## 共通の座標と確認方法

- 現在の格子盤は45 mm/マス、360×360 mm。盤面の格子の左上を(0,0)、画面右をX正・下をY正とする。機体の位置は前後マーカー中点で表し、車軸中心との一致は未検証。向きは右0°/下90°。Bは前黄・後赤、Aは前赤・後黄。
- 過去の`world`/`pose`座標は軸反転や隅IDが異なる。数値・角度だけをコピーせず、保存した基準の`board_to_pose_transform`と盤面画像で変換する。Aの古い135°は、現在の画面で右下45°に相当する例がある。
- まず使用済み配置を再検討する。ターン変更時も、盤面に収まるという理由だけで新しい格子点へ移さない。穴・段差に対して、タイヤ/前後の接地点・機体幅・旋回の膨らみ・停止余裕を確認する。中心軌道やマーカー軌道が盤内という検査は、穴との非干渉の検証ではない。
- 位置が合わない試写が続いたら、機体/HY0020をOFFにし、カメラだけの静止画像で配置を合わせてからONを依頼する。移動先を示す写真は、ユーザー向け本文にクリック可能な画像リンクを示す。ツール内部の画像表示だけで伝わったと扱わない。

## 記録（2026-11-15時点）

| 機体・用途 | マーカー中点・向き（上記の盤面座標） | 状態・注意 |
| --- | --- | --- |
| B 右45°出口 | 計画(90,45) mm、黄色前が右下45°。実際の静止画像は概ね右89/下44 mm | 走行実績あり。同じ32.8/18 mm設定の有効3本を保存。穴について全接地点を機械的に検証した記録ではない |
| B 右45°入口・右135°入口 | 計画約(225,45) mm、黄色前が下90° | 走行実績あり。違う出口ターンへそのまま流用しない |
| A 右135°出口・穴回避後 | 概ね(180,135) mm、赤前が右下45°。保存OFF像は盤面(178.82,132.27) mm/約41.98°、充電後ON像は(180.14,135.95) mm/約44.13° | Aで同一候補3本・操作者の穴/壁接触なし確認あり。Bへは車体・接地点・現在の盤面/撮影の再確認が必要 |
| B 右135°出口への提案(135,45) mm・右下45° | 2026-11-15に提案したが撤回 | 斜め前走が黒い穴のある(180,90) mm付近を通る。ユーザーから穴への干渉を指摘。盤内に収まるだけでは採用不可。この提案/移動図を再利用しない |
| B 右135°出口・現在の開始候補 | 基準(180,45) mm、前黄が右下45°。2026-11-15のOFF像は約(179,42) mm/約46° | Bで同じ候補の独立3本を同じ2倍撮影/検出v8で評価し、全フレーム/静止照合を通過、操作者が全3本で穴/壁への接触なしを確認。右135出口の単体相対候補を保存済み。Aの穴回避位置から上へ約90 mmで穴に対する格子位相は同じ。条件変更・再開時にはBの全接地点・車幅・旋回/停止余裕を再確認し、別候補の穴非干渉まで保証したとは扱わない |

| A 右V90・実際に採用した穴回避配置 | 盤面の概ね(315,90) mm、前赤が左下135°。最終ON像は(317.93,88.07) mm/約134.76° | 同一候補3本、穴/壁接触なし確認あり。旧pose座標(42.07,88.07)/45.24°を現在の盤面軸へ変換した値。初期提案(180,35) mm・右下向きは実際の採用位置ではない |
| B 右V90・初期提案（今回不使用） | 前回B右135出口と同じ(180,45) mm、前黄が右下45° | Bの以前の既使用点を基にした案。今回の実配置は下の約(315,90) mm。Aの実使用配置を90°回転した穴格子位相に相当する案。V90でのB全接地点・全幅・停止余裕は今回の操作者確認と実走が必要。中心線の格子点最短距離約28.5 mmだけで穴非干渉を保証しない |

| B 右V90・今回の静止確認位置（未走行） | 中点は概ね(315,90) mm、前黄が左下135°。実測は約(316,88) mm/134.5° | Bの新規ON静止像で確認。Aの実使用点に近いため、この実配置を維持する。操作者が位置/向き維持と両方OFFを確認（OFF像を新規撮影した意味ではない）。元180/45案との相違で走行0回停止した結果を保持。新経路の穴・全接地点・車幅・停止余裕の再確認と新規静止照合後に実行する |

### 根拠

- B右45出口の[3本比較](../../../../tools/turn_analysis/20261114_B_turn500_seed_from_A/outR45_lengths_three_review/README.md)、[配置計画](../../../../tools/turn_analysis/20261114_B_turn500_seed_from_A/B_outR45_Aseed_plan.json)。
- B右135入口の[準備資料](../../../../tools/turn_analysis/20261114_B_turn500_seed_from_A/B_in_R135_preparation.md)。
- A右135出口の[穴回避配置計画](../../../../tools/logs/pixel_camera/cli_out_r135_lstart40_7_aftercharge_ready_20261112_082139/out_r135_hole_avoidance_layout_plan.json)、[当時の配置と経路の画像](../../../../tools/logs/pixel_camera/cli_out_r135_lstart40_7_aftercharge_ready_20261112_082139/current_setup_and_route_guide.png)、[3本候補](../../../../tools/logs/pixel_camera/cli_out_r135_lstart40_7_aftercharge_ready_20261112_082139/out_r135_candidate.json)。
- Aの穴接触なしの明示確認は[操作者の走行後記録](../../../../tools/logs/pixel_camera/cli_out_r135_lstart40_7_aftercharge_ready_20261112_082139/operator_stop_out_r135_lstart40_7_cli_004.json)。配置計画内の`hole_clearance_verified:false`は画像モデルだけでは検証していない意味であり、操作者確認と区別する。
- Aで以前却下した[穴リスク記録](../../../../tools/logs/pixel_camera/cli_out_r135_off_layout_ready_20261112_072112/placement_hole_risk_review.json)。当時の中心軌道が穴付近を通った。却下例を成功位置として再利用しない。

- B右135出口の現在位置の[OFF静止画像](../../../../tools/turn_analysis/20261114_B_turn500_seed_from_A/outR135_start_placement_revision/off_position_frame.png)と[180/45 mm候補のOFF検査](../../../../tools/turn_analysis/20261114_B_turn500_seed_from_A/outR135_start_placement_revision/off_position_180_45_check.json)。旧135/45 mmへの移動図は撤回済み。

- B右135出口の[最初の有効試行](../../../../tools/turn_analysis/20261114_B_turn500_seed_from_A/outR135_start180_45_run01_review/final_validity.json)と[操作者の接触なし・OFF確認](../../../../tools/logs/pixel_camera/tuning_cli_B_outR135_start180_45_20261115_prepared/power_off_confirmation.json)。

## 更新する項目

配置を新しく決めたら、対象機体/ターン、盤面・穴位置、座標系、測定点、座標・向き、前後色、実際の静止画像/試行、車幅/接地点/停止余裕、操作者の確認と実走結果、未確認事項を計画へ保存する。この表には再開時に必要な要点と根拠リンクを反映し、却下位置も残す。過去の記録を今回のON・手離し・実行の許可として再使用しない。

- B右135出口の[同一候補3本の比較](../../../../tools/turn_analysis/20261114_B_turn500_seed_from_A/outR135_corr1p5_Lstart33p7_three_v8_review/review.json)。前黄が黄緑に映る欠測と、基板の黄色を前マーカーと誤認する例があるため、再開時は[検出v8の検証資料](../../../../tools/turn_analysis/20261114_B_turn500_seed_from_A/marker_hue_revision_v8/README.md)と今回の機体JSONを照合する。色範囲を別の機体/照明へ無条件に流用しない。

- 右V90のAの[実使用配置計画](../../../../tools/logs/pixel_camera/cli_r_v90_10_6_3_6_c05_repeat03_ready_20261111_182538/r_v90_hole_avoidance_layout_plan.json)、[最終静止検査](../../../../tools/logs/pixel_camera/cli_r_v90_10_6_3_6_c05_repeat03_ready_20261111_182538/powered_static_dense_recognition.json)、[接触なし確認](../../../../tools/logs/pixel_camera/cli_r_v90_10_6_3_6_c05_repeat03_ready_20261111_182538/operator_stop_r_v90_10_6_3_6_c05_cli_003.json)。Bの[今回の配置図](../../../../tools/turn_analysis/20261114_B_turn500_seed_from_A/B_rV90_start180_45_route_guide.png)と[初回計画](../../../../tools/turn_analysis/20261114_B_turn500_seed_from_A/B_rV90_Aseed_plan.json)は未実行の案であり、新しい静止確認と操作者の穴/全幅確認に成功してから1回だけ走行する。

- B右V90の[今回位置からの経路図](../../../../tools/turn_analysis/20261114_B_turn500_seed_from_A/B_rV90_start315_90_route_guide.png)、[静止像と位置維持/OFF確認の記録](../../../../tools/turn_analysis/20261114_B_turn500_seed_from_A/rV90_start315_90_position_confirmation.json)、[現在位置の実行前計画](../../../../tools/turn_analysis/20261114_B_turn500_seed_from_A/B_rV90_start315_90_plan.json)。元の180/45案から移し直さず、今回実測した315/90付近を使う。
