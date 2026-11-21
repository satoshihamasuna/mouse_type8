"""実測済み候補からR180暫定値を決めて保存する。接続・走行・Params変更なし。"""
from pathlib import Path
from datetime import datetime, timezone, timedelta
import hashlib
import json
import math
import shutil

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1800_A_r180_20261120_v1'
OUT=BASE/'selection_20261121_v1'
POINTER=BASE/'right180_selected_candidate.json'
assert not OUT.exists() and not POINTER.exists(), '既存の決定資料は上書きしない'
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
def save(p,v):
    with p.open('x',encoding='utf-8') as f:json.dump(v,f,ensure_ascii=False,indent=2,allow_nan=False)
paths=[
    BASE/'radius47p6_c1p5_s15_01/rear_visibility_review_v1/both15_previous_charged_midpoint/report.json',
    BASE/'radius47p6_c1p5_s15_01/rear_visibility_review_v1/both15_current_midpoint/report.json',
    BASE/'resume_20261121_Le40p5_retry01/entry_clock_midpoint_review_v1/report.json',
    BASE/'lend31p5_r47p6_c1p5_charged_01/entry_clock_midpoint_review_v1/report.json',
    BASE/'lend36p0_r47p6_c1p5_01/entry_clock_midpoint_review_v1/report.json',
]
rows=[]
for p in paths:
    r=read(p)
    assert r['mouse']=='A' and r['motion']=='long_r180'
    assert r['relative_geometry_eligible'] and all(r['checks'].values())
    errors=dict(width=(r['return_spacing_at_forward45_mm']-90)/2,
        angle=r['path_angle_deg']-180,right=(r['lend_right_forward_mm'][0]-90)/2,
        forward=(r['lend_right_forward_mm'][1]-90)/3)
    rows.append(dict(source_review=str(p),source_trial=r['source_trial'],parameters=r['parameters'],
        width_mm=r['return_spacing_at_forward45_mm'],angle_deg=r['path_angle_deg'],
        lend_right_forward_mm=r['lend_right_forward_mm'],normalized_errors=errors,
        worst_normalized_error=max(map(abs,errors.values())),squared_normalized_error=sum(v*v for v in errors.values()),
        strict_all_targets_met=all(abs(v)<=1 for v in errors.values()),observation_quality_passed=True))
selected=min(rows,key=lambda r:(r['worst_normalized_error'],r['squared_normalized_error']))
assert selected['source_review']==str(paths[0])
run=Path(selected['source_trial']).parent
settings=read(run/'saved_1ms.settings.json')
trial=read(run/'trial.json')
assert settings['mouse']=='A' and settings['preset_speed_mm_s']==1800 and settings['turn_count']==1
assert settings['parameters']==selected['parameters']
assert settings['parameters']==dict(velo=1.8,r_min=-48.2,lstart=7.52,lend=31.5,degree=-180.,correction=2.,pre_accel=6.5)
assert trial['saved_log']['status']=='saved' and trial['saved_log']['drop_count']==0
candidate={k:settings[k] for k in ('schema_version','mouse','mouse_identity_source','motion_name','preset_speed_mm_s','parameters','pid','feedforward','suction','turn_count')}
assert candidate['suction']==dict(enabled=True,duty=650)
tracking=read(run/'tracking_1ms_review_v1/verification.json')
audit=BASE/'lend36p0_r47p6_c1p5_01/speed_variation_audit_v1/report.json'
prep_path=BASE/'preparation.json'
repeat=BASE/'repeat_r47p6_c1p5_Le36_charged_01'
repeat_phase=read(repeat/'phase.json')
assert not (repeat/'local_execution/jobs/A_R180_1800_Le36_chargecheck_20261121_001.json').exists()
OUT.mkdir()
save(OUT/'selected.settings.json',candidate)
save(OUT/'comparison.json',dict(method='Minimize largest working-target-normalized residual across five directly comparable red15/yellow15 relative reviews; engineering selection, not validation or a statistical confidence claim',
    targets=dict(width_mm=90.,angle_deg=180.,lend_right_forward_mm=[90.,90.]),
    scales=dict(width_mm=2.,angle_deg=1.,lend_right_mm=2.,lend_forward_mm=3.),
    scope='Same per-frame registered ArUco2/3 and relative marker-midpoint method. This is a subset of historical trials; original detector results and other earlier trials are retained.',trials=rows))
save(OUT/'preparation_before_selection.json',read(prep_path))
save(OUT/'repeat_phase_before_selection.json',repeat_phase)
decision=dict(state='PROVISIONAL_SELECTED_EXISTING_MEASURED_CONFIGURATION',mouse='A',motion='long_r180',preset_mm_s=1800,
    decided_at_jst=datetime.now(timezone(timedelta(hours=9))).isoformat(),
    operator_request='難しです．そろそろパラメータ決定できない？',
    selected_settings=str(OUT/'selected.settings.json'),parameters=candidate['parameters'],pid=candidate['pid'],
    feedforward=candidate['feedforward'],suction=candidate['suction'],selected_measurement=selected,
    selected_battery_min_v=tracking['battery_min_v'],saved_1ms_frames=trial['saved_log']['frame_count'],
    selected_configuration_physically_tested=True,new_unmeasured_parameter_combination=False,
    strict_all_targets_met=False,provisional_selection_only=True,additional_tuning_runs_planned=0,
    battery_exchange_required_for_selection=False,operator_ready=False,execution_pending=False,
    last_confirmed_power_state='Both_OFF',ble_connected_this_turn=False,execution_sent_this_turn=False,
    fixed_board_accepted=False,continuous_route_verified=False,reproducibility_across_battery_states_verified=False,
    parameter_adoption_allowed=False,source_params_changed=False,firmware_flashed=False,
    ff_changed=False,ekf_identified_or_changed=False,
    uncertainty='Measured width about93.1mm exceeds working90+/-2mm target, and Lend-right exceeds90+/-2mm. Speed and entry variation remain unresolved. Selection closes single-turn tuning as a provisional practical decision, not a full geometry pass.',
    source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in
        [*paths,run/'saved_1ms.settings.json',run/'saved_1ms.csv',run/'trial.json',audit,Path(__file__)]})
save(OUT/'decision.json',decision)
shutil.copyfile(Path(__file__),OUT/'selection_source.py')
text=f'''# 機体A・1800 R180 暫定決定（2026-11-21）

追加試走を行わず、既存の実測済みの組から次の値で今回の単体調整を区切る。
新しい組合せを未測定のまま採用したものではない。全目標の合格や、電池状態をまたぐ再現性の保証でもない。

| 項目 | 暫定決定値 |
| --- | ---: |
| 速度プリセット | 1800 mm/s |
| 右旋回の名目半径 | 48.2 mm（r_min = −48.2 mm） |
| Lstart | 7.52 mm |
| Lend | 31.5 mm |
| 論理角度 | −180° |
| 角度補正 | ＋2.0° |
| 吸引 | ON・duty650 |
| 前走加速度設定 | 6.5 m/s²（単体試走の90 mm前走では既存実装が18 m/s²を使用） |

PID・FFは選んだ実機応答の値をそのまま保存し、変更しない。試走条件の後直線加速度は20 m/s²。
`selected.settings.json` にPID/FFを含む一式を保存した。起動・preset読込みで消えるRAM設定と、コンパイル済みのターン表は別物である。
今回、Params本体の書換え・ビルド・実機への設定送信・ファームウェア書込みは行っていない。

## 選定の根拠

赤/黄の最小彩度15・毎フレームの実ArUco2/3登録・独立した入口時計による相対中点評価がそろう5候補を比較した。
幅90±2 mm、角度180±1°、Lend右90±2 mm・前90±3 mmの尺度で誤差を割り、最大の誤差が最も小さい組を選んだ。
この尺度は候補選定用の工学的な優先度であり、統計的な信頼区間ではない。過去の別検出条件の結果や不合格結果も削除していない。

選定した充電済み条件の実測は、幅 **{selected['width_mm']:.2f} mm**、返り角 **{selected['angle_deg']:.2f}°**、
Lend中点 **右{selected['lend_right_forward_mm'][0]:.2f} mm・前{selected['lend_right_forward_mm'][1]:.2f} mm**。
1 msログ{trial['saved_log']['frame_count']}点を欠落0で取得、最低電圧{tracking['battery_min_v']:.2f} V。
測定窓の観測率は100%。元の検出条件での評価も幅93.22 mm・角度179.57°であり、大きく矛盾しない。
ここでの半径は設定値で、既存の進入補正が加わる。物理的な曲率半径の保証ではない。

幅は名目より約3.1 mm広く、Lend右も約3.0 mm外側に残るため、厳密な全幾何目標の合格とは扱わない。
直近の速度変動（動画中点の約1.65→1.55 m/s）は保存済みで、単一の幾何値だけで全試行を説明できていない。
電池だけが原因とは断定しない。EKF係数・速度推定・FFの新しい採用は行わない。
入口相対の選定であり、盤面固定の接続、絶対車軸位置・body yaw、連続走行の合格は未確認。

## 作業の終了状態

最後の試走後、接触なし・停止・機体とHY0020の両方OFFを確認済み。
追加の充電済み電池交換・試走要求は、この暫定決定のためには不要とした。
準備だけ済ませたLe36.0の充電後再試行は未実行のまま保存し、今回の決定により後回しとする。

- [決定値一式](selected.settings.json)
- [比較表と誤差](comparison.json)
- [入力ハッシュ・決定状態](decision.json)
- [選定元の実機設定]({(run/'saved_1ms.settings.json').as_posix()})
- [選定元の相対中点評価]({paths[0].as_posix()})
- [直近の速度変動確認]({(audit.parent/'README.md').as_posix()})
'''
with (OUT/'README.md').open('x',encoding='utf-8') as f:f.write(text)
save(POINTER,dict(state=decision['state'],settings=str(OUT/'selected.settings.json'),decision=str(OUT/'decision.json'),readme=str(OUT/'README.md'),parameters=candidate['parameters'],provisional=True,source_params_changed=False,firmware_flashed=False))
prep=read(prep_path)
prep.update(state='PROVISIONAL_PARAMETERS_SELECTED',phase='SELECTION_SAVED_NO_MORE_TUNING_RUNS_PLANNED',
    selected_candidate=str(POINTER),candidate=str(OUT/'selected.settings.json'),
    next_candidate=None,next_stage=None,next_request_id=None,next_candidate_prepared=False,
    operator_ready=False,execution_pending=False,next_operator_ready=False,next_execution_pending=False,
    battery_replacement_required=False,battery_replacement_reason='No additional battery exchange required for provisional selection',
    additional_tuning_runs_planned=0,power_state='Both_OFF_confirmed',both_powered_off_confirmed=True,
    source_params_changed=False,parameter_adoption_allowed=False,provisional_selection_only=True)
prep_path.write_text(json.dumps(prep,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
repeat_phase.update(state='UNEXECUTED_DEFERRED_AFTER_PROVISIONAL_SELECTION',operator_ready=False,
    execution_pending=False,selection=str(POINTER),execution_write_attempted=False,request_consumed=False)
(repeat/'phase.json').write_text(json.dumps(repeat_phase,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(selected=candidate['parameters'],width_mm=selected['width_mm'],angle_deg=selected['angle_deg'],
    worst_normalized_error=selected['worst_normalized_error'],output=str(OUT),no_hardware_action=True),ensure_ascii=False))
