"""カメラ側READMEの露出補正説明を、既存内容をバックアップして更新する。"""
from pathlib import Path
import difflib
import hashlib
import json

camera = Path(r'C:/Users/sato1/Documents/git/Debug_camera')
evidence = Path(__file__).resolve().parents[1] / 'tools/logs/pixel_camera/20261109_brightness_aruco'
backup = evidence / 'source_backup'
changes = {
    'README.md': [(
        '高速撮影セッションはAE・AF・AWBが自動制御に上書きされます。ISO、露光時間、手動フォーカス、WB、EVの変更UIはPhase 1では提供していません。通常モードの対応範囲を高速撮影でも利用可能とは扱いません。AE等の実測値はsensor.csvに残します。',
        '高速撮影セッションはAE・AF・AWBが自動制御に上書きされます。端末内のPhase 1試験UIではISO、露光時間、手動フォーカス、WB、EVの変更を提供していません。2026-11-09版のPC遠隔操作では自動露出の補正値（EV）を変更できます。Pixel 4a (5G)で−2/−1/＋1 EVの要求と撮影結果メタデータの一致を確認しました。手動ISO・露光時間・ピント固定とは別です。通常モードの対応範囲を高速撮影でも利用可能とは扱いません。AE等の実測値はsensor.csvに残します。設定と確認方法は[USB遠隔録画](docs/USB_REMOTE.md#露出補正ev)を参照してください。')],
    'docs/USB_REMOTE.md': [
        ('- AE/AF/AWB: 自動。手動露光等は無効', '- AE/AF/AWB: 自動。PCから自動露出の補正値（EV）を指定可能。手動ISO・露光時間・ピント固定は未対応'),
        ('- configure(): 現時点では1280×720・240 fpsのみ受け付ける', '- configure(): 1280×720・240 fpsとexposure_compensation_stepsを受け付ける'),
        ('## 接続方式', '''## 露出補正（EV）

2026-11-09版のアプリでは、録画開始前に自動露出の目標をPCから補正できます。
既存のpixel_camera.guiにはEVの入力欄がないため、Python APIまたはTYPE8のStart_Pixel_Framing.cmdを使います。

```python
camera.connect()
camera.configure(exposure_compensation_steps=-6)  # Pixel 4a (5G): -1 EV
```

値は整数の段階数です。範囲とEV刻みはget_capabilities()の対象カメラのev_steps_range、ev_stepを確認します。
今回のPixelは[-24, 24]、1/6 EVで、-6が-1 EV、6が+1 EVです。範囲外・非整数・非数値を拒否します。
configureでこのキーを省略すると選択値を維持し、PC接続サービスの再起動時には0へ戻ります。
録画中・転送中には設定できません。設定受付だけで実際の撮影結果を保証しません。

撮影JSONのev_steps_requested、ev_step、ev_steps_result_valuesと、sensor.csv末尾のev_steps_result列で確認します。
従来のsensor.csvの8列は順序を維持しています。露光時間・ISO・AE状態と実画像も合わせて確認してください。
カメラ機能申告のev_effect_verifiedは機種横断で未検証のためfalseのままです。
このPixelでは-2/-1/+1 EVの要求と結果値が一致し、-1と-2では暗くなることも実画像で確認しました。
物理的な240露光/秒や、全補正範囲での画質は未検証です。

## 接続方式''')],
}
for name, replacements in changes.items():
    target = camera / name
    original = target.read_bytes()
    saved = backup / name
    if saved.exists():
        raise RuntimeError('Backup already exists: ' + str(saved))
    text = original.decode('utf-8')
    for old, new in replacements:
        if text.count(old) != 1:
            raise RuntimeError('Expected exactly one occurrence: ' + name)
        text = text.replace(old, new)
    saved.parent.mkdir(parents=True, exist_ok=True)
    saved.write_bytes(original)
    newline = '\r\n' if b'\r\n' in original else '\n'
    target.write_bytes(text.replace('\r\n', '\n').replace('\n', newline).encode('utf-8'))

added = [
    'app/src/main/java/com/example/myapplication/probe/ExposureCompensation.kt',
    'app/src/test/java/com/example/myapplication/probe/ExposureCompensationTest.kt',
]
patch = []
for original in sorted(backup.rglob('*')):
    if original.is_file() and (camera / original.relative_to(backup)).is_file():
        name = original.relative_to(backup).as_posix()
        patch.extend(difflib.unified_diff(original.read_text(encoding='utf-8').splitlines(True),
                     (camera / name).read_text(encoding='utf-8').splitlines(True), 'a/' + name, 'b/' + name))
for name in added:
    patch.extend(difflib.unified_diff([], (camera / name).read_text(encoding='utf-8').splitlines(True), '/dev/null', 'b/' + name))
(evidence / 'camera_source_changes.patch').write_text(''.join(patch), encoding='utf-8')
apk = camera / 'app/build/outputs/apk/debug/app-debug.apk'
(evidence / 'installed_apk.json').write_text(json.dumps(dict(apk=str(apk), sha256=hashlib.sha256(apk.read_bytes()).hexdigest(),
    installed_device='Pixel 4a (5G)', build_checks=['assembleDebug', 'testDebugUnitTest', 'lintDebug'],
    device_ev_requested_and_result_steps=[-12, -6, 6], robot_motion_commands_sent=False), indent=2), encoding='utf-8')
print('Updated camera documentation; backed up originals and regenerated source patch.')
