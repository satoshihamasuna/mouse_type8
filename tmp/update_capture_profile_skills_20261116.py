"""用途別撮影設定を既存スキルへ追記する。既定は差分の準備のみ。"""
import argparse
import difflib
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
STAGE = ROOT / 'tmp/capture_profiles_skill_20261116'
parser = argparse.ArgumentParser()
parser.add_argument('--apply', action='store_true')
args = parser.parse_args()


def replace_once(text, before, after):
    if text.count(before) != 1:
        raise RuntimeError('Expected exactly one matching instruction: ' + before[:90])
    return text.replace(before, after, 1)


if not args.apply:
    proposals = {}
    turn = '.agents/skills/mouse-turn-tuning/SKILL.md'
    text = (ROOT / turn).read_text(encoding='utf-8')
    text = replace_once(text,
        '以前作ったターン調整ツールの確認や、そのGUIの操作・改善依頼にも使う。',
        '以前作ったターン調整ツールの確認や、そのGUIの操作・改善、調整用と9×9迷路用の録画設定切替にも使う。')
    text = replace_once(text,
        '| 探索・最短経路を通した評価 |',
        '| 単体ターンを録画し、BLEログと比較 | `tools/pixel_camera_tuning.py`（`tools/Start_Pixel_Tuning.cmd`） |\n'
        '| 9×9迷路の探索・最短を録画 | `tools/ble/mouse_ble_explore_camera_gui.py`（`tools/Start_BLE_Explore_Camera.cmd`） |\n'
        '| 探索・最短経路を通した評価 |')
    text = replace_once(text,
        '操作・接続・ログ取得では[既存GUIの手順](references/gui-workflow.md)を読む。',
        '録画付きGUIの起動、調整盤と9×9迷路の切替、画角の変更では[用途別の録画設定とArUco参照](references/capture-profiles.md)を読む。\n'
        '**調整用と9×9用の撮影設定は分離し、位置を評価する各フレームで登録済みArUcoを必ず参照する。**\n\n'
        '操作・接続・ログ取得では[既存GUIの手順](references/gui-workflow.md)を読む。')
    proposals[turn] = text

    gui = '.agents/skills/mouse-turn-tuning/references/gui-workflow.md'
    text = (ROOT / gui).read_text(encoding='utf-8')
    text = replace_once(text, '## 読む実装・説明',
        '録画付きで使う場合は先に[用途別の録画設定とArUco参照](capture-profiles.md)で専用GUIを選ぶ。以下の素のBLE中継の既定ポートはPixelカメラと競合するため、録画用の接続設定と混ぜない。\n\n'
        '## 読む実装・説明')
    proposals[gui] = text

    analysis = '.agents/skills/mouse-log-analysis/SKILL.md'
    text = (ROOT / analysis).read_text(encoding='utf-8')
    text = replace_once(text, '## 座標基準と判定の範囲を確認する',
        '調整盤・9×9迷路の動画では[用途別の録画設定とArUco参照](../mouse-turn-tuning/references/capture-profiles.md)も確認し、用途に合った盤面基準・尺度と、各評価フレームのArUco登録証拠を照合する。\n\n'
        '## 座標基準と判定の範囲を確認する')
    proposals[analysis] = text
    proposals['.agents/skills/mouse-turn-tuning/references/capture-profiles.md'] = (STAGE / 'capture-profiles.md').read_text(encoding='utf-8')

    manifest = []
    for index, (name, proposed) in enumerate(proposals.items()):
        target = ROOT / name
        original = target.read_bytes() if target.exists() else None
        newline = '\r\n' if original and b'\r\n' in original else '\n'
        payload = proposed.replace('\n', newline).encode('utf-8')
        staged = STAGE / f'proposal_{index}.bin'
        staged.write_bytes(payload)
        manifest.append(dict(target=name, staged=staged.name,
            original_sha256=hashlib.sha256(original).hexdigest() if original is not None else None,
            proposed_sha256=hashlib.sha256(payload).hexdigest()))
        old_text = original.decode('utf-8').replace('\r\n', '\n') if original is not None else ''
        print(''.join(difflib.unified_diff(old_text.splitlines(True), proposed.splitlines(True),
            fromfile=name, tofile=name)), end='')
    (STAGE / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
else:
    manifest = json.loads((STAGE / 'manifest.json').read_text(encoding='utf-8'))
    skill_root = (ROOT / '.agents/skills').resolve()
    for item in manifest:
        target = (ROOT / item['target']).resolve()
        if not target.is_relative_to(skill_root):
            raise RuntimeError('Target outside project skills')
        actual = hashlib.sha256(target.read_bytes()).hexdigest() if target.exists() else None
        payload = (STAGE / item['staged']).read_bytes()
        if actual != item['original_sha256'] or hashlib.sha256(payload).hexdigest() != item['proposed_sha256']:
            raise RuntimeError('Skill or proposal changed after preparation: ' + str(target))
    for item in manifest:
        target = ROOT / item['target']
        target.write_bytes((STAGE / item['staged']).read_bytes())
        print('Updated:', item['target'])
