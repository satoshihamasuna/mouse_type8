"""B右V90の旋回FF比較候補を準備する。実機操作なし。"""
from pathlib import Path
import copy
import hashlib
import json
import sys

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/'tools/logs/pixel_camera/turn1600_B_rv90_20261120_v1'
OLD = BASE/'baseline01'
RUN = OLD/'local_execution/run_B_RV90_1600_baseline_green_20261120_002'
NEW = BASE/'ff_accdec_0p0006_01'
RID = 'B_RV90_1600_ff0p0006_green_20261120_001'
read = lambda path: json.loads(path.read_text(encoding='utf-8-sig'))


def save(path, value):
    with path.open('x', encoding='utf-8') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')


assert not NEW.exists()
actual = read(RUN/'saved_1ms.settings.json')
candidate = copy.deepcopy(read(OLD/'candidate.settings.json'))
assert actual['feedforward'] == candidate['feedforward']
assert actual['feedforward']['ff_om_accel'] == actual['feedforward']['ff_om_decel'] == .000378
candidate['feedforward'].update(ff_om_accel=.000600, ff_om_decel=.000600)
candidate['source'] = 'B右V90・1600・吸引650の制御追従比較候補。形状/PID/その他FFは初回と同じ。旋回加減速FFを0.000378から0.000600へ段階変更。単発近似同定値を採用値と扱わない。'
NEW.mkdir()
save(NEW/'candidate.settings.json', candidate)
save(NEW/'baseline.settings.json', actual)
save(NEW/'config.json', read(OLD/'config.json'))
wrapper = (OLD/'run_recorded_once.py').read_text(encoding='utf-8')
wrapper = wrapper.replace('B_RV90_1600_baseline_green_20261120_001', RID)
before = "            differences = [key for key, value in expected['feedforward'].items()\n                if abs(actual['feedforward'][key]-value) > 0.000002]\n            if differences:\n                raise RuntimeError('B右V90の実機FFが準備値と異なる: '+','.join(differences))"
after = """            baseline = tuning.read_json(OUT/'baseline.settings.json')
            allowed = (baseline['feedforward'], expected['feedforward'])
            if not any(all(abs(actual['feedforward'][key]-value) <= 0.000002
                           for key, value in values.items()) for values in allowed):
                raise RuntimeError('B右V90の実機FFが基準値/今回候補のいずれとも一致しない')"""
assert wrapper.count(before) == 1
wrapper = wrapper.replace(before, after)
compile(wrapper, str(NEW/'run_recorded_once.py'), 'exec')
(NEW/'run_recorded_once.py').write_text(wrapper, encoding='utf-8', newline='\n')
fit = read(RUN/'ff_turn_diagnostic_v1/fit.json')
plan = dict(mouse='B', motion='r_v90', preset_mm_s=1600, request_id=RID,
    changes={'ff_om_accel':[.000378,.000600], 'ff_om_decel':[.000378,.000600]},
    parameters=candidate['parameters'], pid=candidate['pid'], suction=candidate['suction'],
    baseline_trial=str(RUN/'trial.json'), baseline_tracking=str(RUN/'tracking_1ms_review_v3/verification.json'),
    ff_diagnostic=str(RUN/'ff_turn_diagnostic_v1/fit.json'),
    reasoning='初回は立上り約13rad/sの不足・ピーク約6rad/s超過・減速後の振れ。閉ループ旋回診断は加速0.001107/減速0.001001を示すが励起点数が少ないため中間値0.000600で比較する。速度FF・PID・形状は固定。',
    acceptance='角速度の前後半/全体RMS・ピーク超過・Lendまでの振れと積分・電圧飽和を初回と比較。動画の相対軌道/停止と全フレームの品質も再確認。悪化時は候補を採用しない。',
    placement=read(OLD/'placement_plan.json')['placement'],
    post_diagonal_accel_m_s2=25., guide=str(BASE/'start_placement_guide.jpg'),
    operator_ready=False, execution_commands_sent=0, power_state='BOTH_OFF_USER_CONFIRMED_AFTER_BASELINE',
    hardware_operations=0, source_params_changed=False, firmware_write_performed=False,
    baseline_source_sha256=hashlib.sha256((RUN/'saved_1ms.settings.json').read_bytes()).hexdigest(),
    identification_limitations=['単発閉ループの近似回帰。速度/加速/減速の励起点数不足を保持。',
        '並進FFは定速だけで分離不能なので変更しない。',
        'B右V90の比較専用候補。Bの他ターンやAの設定へ反映しない。'])
save(NEW/'comparison_plan.json', plan)
sys.path.insert(0, str(ROOT/'tools'))
from pixel_camera_tuning_mcp import McpTuningService
payload = McpTuningService(NEW/'offline_validation', {}).prepare(str(NEW/'config.json'), str(NEW/'candidate.settings.json'), plan['placement'], 25., None)
save(NEW/'offline_validation.json', dict(state='VALIDATED_NOT_EXECUTED', payload=payload, hardware_operations=0))
print(json.dumps(dict(state='VALIDATED_NOT_EXECUTED', folder=str(NEW), changes=plan['changes'], hardware_operations=0),ensure_ascii=False))
