"""B1600右135度出口の今日用準備記録をまとめる。実機操作なし。"""
from pathlib import Path
from datetime import datetime, timezone
import ast
import hashlib
import json
import shutil

root=Path(__file__).resolve().parents[1]
old=root/'tools/logs/pixel_camera/turn1600_B_outr135_20261120_v1'
base=root/'tools/logs/pixel_camera/turn1600_B_outr135_20261121_resume_v1'
read=lambda path:json.loads(path.read_text(encoding='utf-8-sig'))
def save(path,value):
    with path.open('x',encoding='utf-8') as stream:
        json.dump(value,stream,ensure_ascii=False,indent=2,allow_nan=False)
        stream.write('\n')
source_run=old/'angle_c1p0_Ls6_Le27p9_01/local_execution/run_B_OUTR135_1600_c1p0_Ls6_Le27p9_green_20261120_001'
actual=read(source_run/'saved_1ms.settings.json')
snapshot=read(base/'resource_snapshot.json')
stages=[];hashes={}
for number in (1,2,3):
    stage=base/f'geometry_c1p0_Ls5p3_Le31_{number:02d}'
    settings=read(stage/'candidate.settings.json');config=read(stage/'config.json')
    phase=read(stage/'phase.json');plan=read(stage/'placement_plan.json')
    assert config['mouse']=='B' and config['address']=='E1:7F:86:21:84:73'
    assert settings['mouse']=='B' and settings['motion_name']=='out_r135' and settings['preset_speed_mm_s']==1600
    assert settings['parameters']==dict(velo=1.6,r_min=-41.,lstart=5.3,lend=31.,degree=-135.,correction=1.,pre_accel=6.5)
    for key in ('pid','feedforward','suction','turn_count'):
        assert settings[key]==actual[key]
    assert settings['turn_count']==1 and not list((stage/'local_execution/jobs').glob('*.json'))
    source=(stage/'run_recorded_once.py').read_text(encoding='utf-8-sig')
    ast.parse(source)
    expected_id=f'B_OUTR135_1600_c1p0_Ls5p3_Le31_green_20261121_{number:03d}'
    assert expected_id in source and phase['request_id']==expected_id
    assert 'camera_before_ble_' in source and 'coarse_placement_check.py' in source
    for file in ('candidate.settings.json','config.json','run_recorded_once.py','coarse_placement_check.py','offline_validation.json'):
        path=stage/file;assert path.is_file()
        hashes[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
    for key in ('reference','geometry'):
        path=Path(config[key]);assert path.is_file()
        hashes[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
    stages.append(dict(stage=str(stage),request_id=expected_id,config=config,parameters=settings['parameters'],turn_count=1))
guide=base/'start_placement_guide.jpg'
assert not guide.exists()
shutil.copyfile(old/'start_placement_guide.jpg',guide)
stage1=Path(stages[0]['stage']);plan=read(stage1/'placement_plan.json')
result=dict(prepared_at=datetime.now(timezone.utc).isoformat(),date_jst='2026-11-21',
    mouse='B',motion='out_r135',preset_mm_s=1600,stages=stages,
    source_trial=str(source_run/'trial.json'),basis=str(old/'c1p0_001_review_next_lengths5p3_31.json'),
    ble_advertisement_name='HY0020-MOUSE-B',ble_address='E1:7F:86:21:84:73',
    placement=plan['placement'],front_colour='green',rear_colour='red',guide=str(guide),
    pre_run_mm=plan['pre_run_mm'],post_run_mm=plan['post_run_mm'],post_straight_accel_m_s2=20.,
    resource_snapshot=str(base/'resource_snapshot.json'),
    resource_use_observed=bool(snapshot['active_processes']),
    current_robot_power='NOT_CONFIRMED_THIS_RESUME',current_placement_confirmed=False,
    current_clearance_confirmed=False,current_hands_clear_confirmed=False,
    input_validation_passed=True,hardware_connections_attempted=0,robot_commands_sent=0,
    firmware_write_performed=False,source_params_changed=False,source_hashes=hashes,
    next_steps=['実行直前に他の実機取得プロセスと8766番ポートが解放されていることを再確認する。競合相手を停止しない。',
        '同じ盤/カメラ高さ1500mm/前緑・後赤を現在の配置と照合する。開始中点180/45mm、緑前右下45度。撤回済み135/45mmへ移さない。',
        '今回の穴/全接地点/車幅/旋回/停止余裕と両方ON・手離しを操作者が確認する。過去のON確認を再利用しない。',
        '新規静止撮影で概略配置と実ArUco登録を確認してから、B用BLEで電圧・実機モード・PID/FF・候補・後走加速度20m/s²を照合し、新IDを1回実行する。',
        '終了/失敗後は実行を再送せず保存・切断し、両方OFFを案内する。解析は電源OFFで行う。'],
    limitations=['今日の実機配置/通信/電圧/カメラ状態は未確認。準備だけで走行の許可や品質確認とはしない。',
        '形状評価は実測入口基準の高さ補正マーカー中点。盤面固定/車軸絶対位置・BのEKF係数・連続接続は未確認。'])
save(base/'preparation.json',result)
save(base/'resume_state.json',dict(state='PREPARED_OFFLINE_WAITING_SHARED_RESOURCES_AND_FRESH_READINESS',
    prepared_at=result['prepared_at'],mouse='B',motion='out_r135',preset_mm_s=1600,
    next_stage=str(stage1),next_request_id=stages[0]['request_id'],next_request_sent=False,
    candidate=str(stage1/'candidate.settings.json'),config=str(stage1/'config.json'),
    ble_address=result['ble_address'],ble_advertisement_name=result['ble_advertisement_name'],
    power_state='NOT_CONFIRMED_THIS_RESUME',operator_ready=False,execution_pending=False,
    current_placement_confirmed=False,current_hands_clear_confirmed=False,guide=str(guide),
    preparation=str(base/'preparation.json'),resource_snapshot=str(base/'resource_snapshot.json'),
    next_action='実機取得/8766競合の解消を新規照会し、今回の開始配置/干渉なし/ON/手離し確認後に今日の新IDで1回比較。'))
old_state=read(old/'resume_state.json')
save(old/'resume_state_before_20261121_preparation.json',old_state)
old_state.update(state='SUPERSEDED_BY_20261121_RESUME_PREPARATION',next_request_sent=False,
    operator_ready=False,execution_pending=False,power_state='NOT_CONFIRMED_THIS_RESUME',
    current_placement_confirmed=False,current_hands_clear_confirmed=False,
    next_stage=str(stage1),next_request_id=stages[0]['request_id'],
    candidate=str(stage1/'candidate.settings.json'),config=str(stage1/'config.json'),
    next_action='今日用の独立準備へ移行。turn1600_B_outr135_20261121_resume_v1/resume_state.jsonを確認。旧IDで実行しない。',
    superseded_by=str(base/'resume_state.json'))
(old/'resume_state.json').write_text(json.dumps(old_state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
text='''# 機体B・1600 右135度出口の再開準備（2026-11-21）

次の未実行候補は半径−41 mm、Lstart 5.3 mm、Lend 31.0 mm、角度補正＋1.0°。BのPID/FF・吸引ON duty650を取得済み設定から保持した。今日用の独立3試行分をオフライン検証済み。物理ターン回数は各1回。

BLEは **HY0020-MOUSE-B / E1:7F:86:21:84:73**。A用C1:B5:60:87:91:73へ接続しない。取得済みログと設定のアドレスは当時の記録として保持。

開始中点は **(180,45) mm・緑前が右下45°・後赤**。同じ360 mm/45 mm格子盤、カメラ高さ1500 mm、EV−1.5・ズーム1.8・ライトOFF・AF固定ON。開始位置の案内は[start_placement_guide.jpg](start_placement_guide.jpg)。保存写真への案内であり、今回の位置測定ではない。撤回済み(135,45) mmを使わない。

斜め前走127.279 mm、直進後走90 mm、後走直進加速度20 m/s²。新規静止撮影で概略配置を照合してからBLEを開始する。走行動画は実測入口基準で評価し、必要区間観測率95%以上と各評価フレームの実ArUco登録2個以上を維持する。

準備時に別のB rV90試行と8766番のB用中継を観測した。この準備では実機に接続せず、他のプロセスも停止していない。再開直前に取得プロセス/ポートの解放と現在のカメラ状態を確認し、今回の配置・干渉なし・両方ON・手離しの返答後に新IDで1回だけ実行する。昔のON確認は再利用しない。

前回の補正＋1.0°・長さ6.0/27.9 mmは入口相対出口角134.749°、Lend位置誤差3.564 mm。この実測から長さ解5.321/30.953 mmを求めて5.3/31.0 mmへ丸めた候補。今回の候補は未走行、採用・書込み未実施。BのEKF係数と連続接続は未確認。

詳細は[preparation.json](preparation.json)と[resume_state.json](resume_state.json)。実機接続・設定送信・走行・書込みは0件。今日の実機通信・電圧・配置の品質は未確認。
'''
(base/'README.md').write_text(text,encoding='utf-8')
print(json.dumps(dict(state='PREPARED_OFFLINE',stages=len(stages),next_request_id=stages[0]['request_id'],
    B_address=result['ble_address'],resource_use_observed=result['resource_use_observed'],hardware_connections_attempted=0),ensure_ascii=False))
