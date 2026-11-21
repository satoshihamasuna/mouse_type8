"""停止中の通信試験を記録し、標準手順の未実行3本目へ引き継ぐ。"""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
root=Path(__file__).resolve().parents[1]
base=root/'tools/logs/pixel_camera/turn1600_B_outr135_20261121_resume_v1'
comm=base/'communication_only_v1'
read=lambda path:json.loads(path.read_text(encoding='utf-8-sig'))
def save(path,value):
    with path.open('x',encoding='utf-8') as stream:
        json.dump(value,stream,ensure_ascii=False,indent=2,allow_nan=False);stream.write('\n')
summaries=list((comm/'results').glob('*/summary.json'));assert len(summaries)==1
path=summaries[0];summary=read(path)
assert read(comm/'completion.json')['returncode']==0
assert summary['passed'] and summary['address']=='E1:7F:86:21:84:73'
assert 'mouse:B' in summary['idle_response'] and 'state:idle' in summary['idle_response']
assert summary['periods_ms']==[20,10,5] and len(summary['segments'])==3
rows=[]
for segment in summary['segments']:
    sample=segment['sample']
    assert segment['passed'] and sample['lost_frames']==sample['crc_errors']==0
    assert sample['cadence_within_5_percent']
    rows.append(dict(period_ms=segment['period_ms'],duration_seconds=segment['requested_duration_s'],
        samples=sample['rows'],effective_hz=sample['effective_hz'],mean_interval_ms=sample['interval_ms']['mean'],
        lost_frames=sample['lost_frames'],crc_errors=sample['crc_errors'],
        bridge_delta=segment.get('bridge_delta'),diagnostics_delta=segment.get('diagnostics_delta'),
        link_delta=segment.get('link_delta'),bridge_final_status_read=segment.get('bridge_final_status_read'),
        diagnostics_final_read=segment.get('diagnostics_final_read'),link_final_read=segment.get('link_final_read')))
next_stage=base/'standard_flow_c1p0_Ls5p3_Le31_01'
next_id='B_OUTR135_1600_c1p0_Ls5p3_Le31_standard_green_20261121_001'
assert not list((next_stage/'local_execution/jobs').glob('*.json'))
config=read(next_stage/'config.json');assert config['address']==summary['address'] and config.get('stream_period_ms',5)==5
result=dict(recorded_at=datetime.now(timezone.utc).isoformat(),request_id='B_BLE_COMM_ONLY_20261121_001',
    state='PASS_IDLE_COMMUNICATION_ONLY',source_summary=str(path),source_summary_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
    mouse='B',ble_address=summary['address'],device_idle_response=summary['idle_response'],segments=rows,
    total_samples=sum(row['samples'] for row in rows),requested_measurement_seconds=60,
    negotiated_link_source=summary.get('link_metrics_source'),connection_interval_ms=summary.get('connection_interval_ms'),
    mtu_size=summary.get('mtu_size'),phy=summary.get('phy'),data_length=summary.get('data_length'),
    last_link_observation=summary.get('link_last_observed'),
    motion_commands_sent=0,parameter_write_commands_sent=0,camera_operations=0,
    operator_reply='両方OFF・出口の配置そのまま',power_state='BOTH_OFF_USER_CONFIRMED',
    root_cause_identified=False,interpretation='今回の停止中20秒×3条件では5msを含め切断/欠落/CRCなし。配信周期だけが原因と断定できない。',
    limitations=['一接続の短時間・停止中試験。長い設定送信との同時負荷、録画付き設定手順、走行中、長期/反復の安定性は別確認。',
        '先の終了処理修正と、標準MCPへ揃えた将来の取得手順の効果は、この通信ベンチだけでは実証していない。'],
    next_stage=str(next_stage),next_request_id=next_id,next_request_sent=False,
    successful_same_candidate_turn_runs=2)
save(comm/'review.json',result)
text='# 機体B・停止中のBLE通信確認\n\n'
text+='B用E1:7F:86:21:84:73へ一接続し、実機応答でmouse:B state:idleを確認した。走行/吸引/走行パラメータ変更なし。終了後は両方OFF・出口配置保持を操作者が確認。\n\n|配信周期 [ms]|要求時間 [s]|サンプル数|実効 [Hz]|平均周期 [ms]|欠落/CRC|\n|---|---:|---:|---:|---:|---:|\n'
for row in rows:text+=f'|{row["period_ms"]}|20|{row["samples"]}|{row["effective_hz"]:.3f}|{row["mean_interval_ms"]:.4f}|0/0|\n'
text+='\n計7,013点、全段合格。リンク診断で接続間隔15 ms、MTU247、data length251 byteを記録。PHYはデバイス報告の生値tx=1/rx=1。元の全カウンターと最終読取り成否はresults配下に保存。\n\nこれは短時間・停止中の一接続であり、設定送信/録画との同時処理や走行・反復の安定性の保証ではない。5msを維持したまま独自performを除いた標準MCPで次のout135を確認する準備を保持。診断から自動で走行へ移らず、今回のON/手離し確認と競合再確認後に未消費のstandard_green_20261121_001を使用する。\n'
(comm/'README.md').write_text(text,encoding='utf-8')
state_path=base/'resume_state.json';state=read(state_path)
state.update(state='COMMUNICATION_ALL_PERIODS_PASS_BOTH_OFF_STANDARD_THIRD_PREPARED',
    power_state='BOTH_OFF_USER_CONFIRMED',operator_power_off_confirmation='両方OFF・出口の配置そのまま',
    operator_ready=False,current_placement_confirmed=True,current_hands_clear_confirmed=False,
    execution_pending=False,ble_connected=False,next_request_sent=False,
    last_communication_test=str(comm/'review.json'),last_communication_result='PASS_20_10_5_MS',
    last_request_id=result['request_id'],last_request_result='PASS_IDLE_COMMUNICATION_ONLY',last_error=None,
    last_execution_write_attempted=False,execution_commands_sent=0,successful_same_candidate_runs=2,
    next_action_kind='turn_capture_one_standard_flow',next_stage=str(next_stage),next_request_id=next_id,
    candidate=str(next_stage/'candidate.settings.json'),config=str(next_stage/'config.json'),
    next_action='出口配置維持。今回の両方ON/干渉なし/手離し確認後、競合を再確認し、標準MCP・B用BLE・5msで同一候補3本目を新IDで1回取得。')
state_path.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
with (base/'README.md').open('a',encoding='utf-8') as stream:
    stream.write('\n通信のみの比較は20/10/5ms各20秒を一接続で全合格。1002/2008/4003点、計7013点で欠落/CRC0、実機B idle確認済み。終了後に操作者が両方OFF・出口配置保持を確認。配信周期だけでは従来の切断を説明できず、長い設定送信等との同時負荷は未検証。次は5msのまま標準MCPへ揃えたstandard_flow stage01でout135同候補3本目を確認する準備。今回の新しいON/手離し確認前は接続しない。詳細communication_only_v1/README.md。\n')
print(json.dumps(dict(state=result['state'],total_samples=result['total_samples'],next_request_id=next_id),ensure_ascii=False))
