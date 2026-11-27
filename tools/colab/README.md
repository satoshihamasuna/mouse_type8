# Google Colab によるマウス動画認識率の検証

`mouse_type8` の保存済み走行動画を使い、Google Colab（GPU 環境）上でマウス認識率の再現検証・新モデル比較を行うための手順と設計です。実機接続・BLE・カメラ操作は行いません。

## 1. 現状の認識率ボトルネック（保存済み検証の整理）

2026-11-22 の保存済み検証（`tools/logs/pixel_camera/ml_pose_review_20261122_v1/README.md`・`recognition_review_20261122_015121_v1/README.md`）から、以下の課題が確認されています。

| 対象 | 従来色ペア (`legacy`) | 外観＋小型CNN (`learned_assist`) | 判明している要因・課題 |
| --- | --- | --- | --- |
| 機体 A・暗い静止 24 枚 | 0/24 (0%) | 24/24 (100%) | 緩和色ペア＋外観評価で回復（CNN 精密化 5 枚、中点 RMS 約 0.56 mm） |
| 機体 A・暗い旋回 60 枚 | 0/60 (0%) | 2/60 (3.3%) | 露光中央値約 4.0 ms（1.6 m/s で約 6.5 mm ブレ）・ISO 約 2166 で色候補が消失。既存小型 CNN は直前位置近傍 64×64 切り出し前提のため、一度ロストすると再捕捉できない |
| 機体 B・静止 / 旋回 | 24/24・57/57 (AR 成立中) | 24/24・57/57 | 後色を黄→赤に修正後、AR 成立フレームは従来法で 100%。残り 3 枚は AR 欠測。学習補助の追加効果は未確認 |

加えて、既存 `eval_A_cnn` では CPU 推論中央値が約 1.65 秒/フレーム（候補一括スコア化前）と重く、試行錯誤に時間がかかる点が課題でした。

## 2. Colab で何を検証するか（検証設計）

同じ **実復号 ArUco 盤面登録条件** と **独立動画分割（SHA-256 検証）** の下で、以下の 4 系統を横並び比較します。

1. **`legacy`（基準）**: 従来の前後色ペア検出のみ。
2. **`learned_assist`（既存互換＋GPU 再学習）**: 既存 `BodyKeypointCNN` を Colab GPU で強拡張・多ステップ学習し、外観モデルと組み合わせて評価。重みは CPU 互換形式で保存されるため、そのままローカル PC へ持ち帰り可能。
3. **`board_cnn_global`（新方式・盤面グローバル検出）**: [colab_board_detector.py](../analysis/colab_board_detector.py) の `BoardKeypointDetector`。色ペア候補・前フレーム位置に依存せず、AR 正対補正した盤面全体（既定 256×256）から前後キーポイントを直接推定し、暗い旋回・モーションブラー時の再捕捉能力を検証。
4. **`hybrid_board_assist`（ハイブリッド）**: 通常の色ペアが観測できるコマはそのまま実観測（`observed=True`）とし、色ペアが欠測したコマだけを `board_cnn_global`（`estimated=True`）で補う方式。

### 主な評価指標

`evaluate_body_detectors.py` が各ケース・各方式ごとに以下を算出します。

- `aruco_calibration_rate`: ArUco 登録が成立したフレーム割合（盤面登録自体の欠測を切り分け）
- `observed_given_calibration`: AR 成立フレームのうち、前後色ペアを実観測できた割合
- `coverage_given_calibration`: AR 成立フレームのうち、実観測＋学習推定で位置が得られた割合
- `max_consecutive_missing_frames`: AR 成立中の最大連続欠測フレーム数（旋回中のロスト長）
- `implausible_position_steps` / `implausible_heading_steps`: 物理的に不自然な位置・角度の飛び件数
- `midpoint_jitter_rms_mm` / `heading_jitter_rms_deg`: 静止区間の散らばり
- `processing_median_ms` / `processing_p95_ms`: 1 フレームあたりの処理時間

## 3. ローカル PC での準備（Colab 用バンドルの出力）

Windows 絶対パス依存を避けるため、[export_colab_recognition_bundle.py](../analysis/export_colab_recognition_bundle.py) で動画・設定 JSON・任意モデル・学習パッチを相対パス構成へまとめます。元ファイルは変更しません。

```powershell
# 機体 A の既存レビュー構成を Colab 用 ZIP へ書き出す例
python tools/analysis/export_colab_recognition_bundle.py `
  --manifest tools/logs/pixel_camera/ml_pose_review_20261122_v1/evaluation_cnn_A.json `
  --appearance-model tools/logs/pixel_camera/ml_pose_review_20261122_v1/model_A/appearance.npz `
  --cnn-model tools/logs/pixel_camera/ml_pose_review_20261122_v1/model_A/keypoints.pt `
  --patches tools/logs/pixel_camera/ml_pose_review_20261122_v1/model_A `
  --output tools/logs/pixel_camera/colab_bundles/bundle_A `
  --zip
```

出力される `bundle_A.zip`（または `bundle_A/` フォルダー）を Google Drive へアップロードしてください。機体 B を検証する場合は別バンドル（`bundle_B`）として出力し、機体 A と混ぜないでください。

解析コード用の `mouse_type8_tools.zip` は、リポジトリのルートで次のように新規作成します。動画・取得ログ・学習済みモデルは含めず、上記のデータバンドルと別々に配置します。既存の ZIP は上書きしません。

```powershell
@'
from pathlib import Path
import zipfile

output = Path('tools/logs/pixel_camera/colab_bundles/mouse_type8_tools.zip')
output.parent.mkdir(parents=True, exist_ok=True)
with zipfile.ZipFile(output, 'x', compression=zipfile.ZIP_DEFLATED) as archive:
    for pattern in ('tools/*.py', 'tools/analysis/*.py', 'tools/requirements*.txt'):
        for path in sorted(Path('.').glob(pattern)):
            archive.write(path, arcname=path.as_posix())
'@ | py -3.12 -
```

## 4. VS Code またはブラウザ（Google Colab）での実行手順

[mouse_recognition_colab.ipynb](mouse_recognition_colab.ipynb) は **VS Code のローカル Python カーネル** と **VS Code の Google Colab 拡張機能（リモート GPU カーネル）**・**ブラウザ版 Google Colab** のすべてを自動判別します。

### A. VS Code の Google Colab 拡張機能（GPU カーネル）で実行する場合
1. Google Drive の `マイドライブ/mouse_type8_colab_bundles/` へ `mouse_type8_tools.zip` と `bundle_A.zip`（または `bundle_B.zip`）を配置する（リポジトリ全体のアップロードは不要）。
2. VS Code で [mouse_recognition_colab.ipynb](mouse_recognition_colab.ipynb) を開き、右上のカーネル選択から **Colab（T4 GPU 等）** を選ぶ。
3. セル 2 の `TARGET_MOUSE = 'A'`（または `'B'`）を確認し、「すべて実行」を押す（自動で Google Drive をマウントして展開・学習・評価し、`マイドライブ/mouse_type8_colab_results/` へ保存します）。

### B. VS Code のローカル Python カーネルでそのまま実行する場合
1. VS Code で [mouse_recognition_colab.ipynb](mouse_recognition_colab.ipynb) を開き、右上のカーネル選択からローカルの **Python 3.12**（`OpenCV aruco`・`ipykernel` 導入済み）を選ぶ。
2. セル 2 の `TARGET_MOUSE = 'A'`（または `'B'`）を確認し、「すべて実行」を押す。
   - `.tools/vision_python312`（CPU 版 PyTorch）と `tools/logs/pixel_camera/colab_bundles/bundle_A` を自動検出し、CPU 向けの軽量プリセット（`BOARD_CNN_PX=160, STEPS=200` 等）で動作します。
   - 結果は `tools/logs/pixel_camera/colab_results/recognition_<機体>_<日時>/` へ新規保存されます。

### C. ブラウザ版 Google Colab で実行する場合
1. [mouse_recognition_colab.ipynb](mouse_recognition_colab.ipynb) を Colab で開き、ランタイムを **T4 GPU** に設定する。
2. 左のファイル欄（`/content/`）へ `mouse_type8_tools.zip` と `bundle_A.zip` を直接アップロードするか、上記 A と同様に Google Drive へ配置して「すべてのセルを実行」する。

## 5. ローカル採用時の注意

- Colab 上の被覆率向上だけでは採用せず、`recovered_gallery_*.png` と `comparison_montage.jpg` で誤認がないか目視確認してください。
- `keypoints_gpu.pt` は CPU テンソルで保存されるためローカル PC の `evaluate_body_detectors.py` でそのまま読み込めます。採用前にローカル CPU 環境での処理時間・再現性を再確認してください。
- 色ペア擬似正解からの学習だけでは絶対サブピクセル精度を保証できません。実運用へ採用する前に、人手で指定した独立正解フレームでも誤差を確認してください。
- 検証結果だけで `Params/` や通常 GUI の既定モードを自動変更しないでください。

## 6. ホストテスト

Windows では OpenCV ArUco・NumPy・pandas を導入した Python 3.12 を指定して実行します。小型 CNN と盤面 CNN には PyTorch も必要です。ローカルの `.tools/vision_python312` に配置した PyTorch は自動検出します。

```powershell
py -3.12 -m unittest discover -s tools/tests -p 'test_body_*.py' -v
py -3.12 -m unittest discover -s tools/tests -p test_colab_recognition.py -v
```

CPU での学習・保存読込、相対パスでのバンドル評価、学習動画の混入拒否、実観測と学習推定の区別、ノートブックの Python セル構文を確認します。依存不足のスキップは実行成功と区別してください。Colab 上の CUDA 実行と実動画での認識精度は別途確認が必要です。
