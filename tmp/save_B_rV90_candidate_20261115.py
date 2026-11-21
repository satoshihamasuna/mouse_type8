from pathlib import Path
import json,hashlib,shutil,re
BASE=Path('tools/turn_analysis/20261114_B_turn500_seed_from_A').resolve()
OUT=BASE/'rV90_Ls9_Le11p6_c1_three_v8_review'
def load(p):return json.loads(Path(p).read_text(encoding='utf-8'))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,d):
    with Path(p).open('x',encoding='utf-8',newline='\n') as f:
        json.dump(d,f,ensure_ascii=False,indent=2);f.write('\n')
review=load(OUT/'review.json')
assert review['valid_trials']==3 and review['repeatability_guidance_passed']
plan=load(BASE/'B_rV90_Ls9_Le11p6_c1_repeat03_plan.json')
source=Path(plan['source_B']['path']);before=source.read_bytes()
assert sha(source)==plan['source_B']['sha256']
(OUT/'source_snapshots').mkdir()
shutil.copy2(source,OUT/'source_snapshots/turn_500_before_rV90_candidate.h')
text=before.decode('utf-8')
needle='const static t_turn_param_table slalom_RV90_500_table = {0.50f,-41.0f,6.36,12.60,-90.0f,Turn_R};'
assert text.count(needle)==1
lines=[
'// 2026-11-15 機体B r_V90: 同じ2倍撮影・検出v8の単体3本を確認した候補。実テーブル・共有ゲインは既存値を維持。',
'// 半径-41 mm、Lstart9.0/Lend11.6 mm、論理-90度/補正+1.0度。SP PID1.50/0.02/0、OM PID0.10/0.002/0。',
'// 相対終点平均は右127.90/前190.75 mm（目標127.28/190.92 mm）、各軸最大誤差0.76/0.64 mm、出口方向差最大0.29度。',
'// 実測入口方向に対する相対マーカー中点の形状評価。盤面固定の停止点には上約5.4〜10.3 mmの差が残る。',
'// 左V90・盤面固定目標・絶対車軸位置・連続接続は未検証。Bの全FFは B_rV90_single_turn_candidate.settings.json を参照。',
'// 根拠: tools/turn_analysis/20261114_B_turn500_seed_from_A/rV90_Ls9_Le11p6_c1_three_v8_review/review.json',
'// const static t_pid_gain om_gain_RV90_500_candidate = {0.10f,0.002f,0.0f};',
'// const static t_turn_param_table slalom_RV90_500_candidate_table = {0.50f,-41.0f,9.00f,11.60f,-90.0f,Turn_R,1.0f};',
'// const static t_param param_RV90_500_candidate = {&slalom_RV90_500_candidate_table,&sp_gain_turnV90_500,&om_gain_RV90_500_candidate};']
text=text.replace(needle,'\r\n'.join(lines)+'\r\n'+needle)
def tokens(s):return re.findall(r'\S+',re.sub(r'/\*.*?\*/|//[^\r\n]*','',s,flags=re.S))
assert tokens(text)==tokens(before.decode('utf-8'))
after=text.encode('utf-8');assert after.count(b'\r\n')==after.count(b'\n')
source.write_bytes(after)
save(OUT/'comment_only_revision.json',dict(state='COMMENT_ONLY_ACTIVE_CPP_TOKENS_UNCHANGED',source=str(source.resolve()),
    before_sha256=hashlib.sha256(before).hexdigest(),after_sha256=sha(source),active_cpp_tokens_unchanged=True,
    CRLF_preserved=True,firmware_build_required=False,firmware_flashing_performed=False))
session=Path('tools/logs/pixel_camera/tuning_cli_B_rV90_Ls9_Le11p6_c1_repeat03_20261115_prepared')
actual=session/'run_B_rV90_Ls9_Le11p6_c1_repeat03/motion.settings.json'
target=BASE/'B_rV90_single_turn_candidate.settings.json';assert not target.exists();shutil.copy2(actual,target)
save(BASE/'B_rV90_single_turn_candidate.meta.json',dict(mouse='B',motion='r_v90',
    state='THREE_VALID_RELATIVE_GUIDANCE_PASSED_CANDIDATE_COMMENT_ONLY',settings=str(target),settings_sha256=sha(target),
    review=str(OUT/'review.json'),valid_trials=3,relative_guidance_passed=True,
    evaluation_frame='relative_origin_and_measured_entry_direction',camera_zoom_ratio=2,
    detector_geometry=str(BASE/'body_geometry_B_zoom2_v8.json'),firmware_source_candidate_commented=True,
    firmware_active_parameters_changed=False,firmware_flashing_performed=False,active_shared_om_kp=.3,candidate_right_om_kp=.1,
    left_v90_verified=False,continuous_connection_verified=False,absolute_axle_position_verified=False,body_yaw_verified=False,
    board_fixed_goal_verified=False,board_fixed_observed_stop_error_y_mm_range=[-10.292282104492188,-5.44598388671875],
    power='BOTH_OFF_USER_CONFIRMED',service='CLOSED_CONFIRMED',next_target='NOT_SELECTED',source_revision=str(OUT/'comment_only_revision.json')))
mp=BASE/'next_session_B_rV90_Ls9_Le11p6_c1_repeat03_v1.json';meta=load(mp)
meta.update(state='THREE_VALID_RELATIVE_CANDIDATE_SAVED_POWER_OFF_CLOSED',same_settings_valid_trials=3,
    candidate_meta=str(BASE/'B_rV90_single_turn_candidate.meta.json'),next_target='NOT_SELECTED')
mp.write_text(json.dumps(meta,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
(OUT/'README.md').write_text('''機体B・右V90・500プリセット：単体相対形状の候補（コメント保存のみ）

同じ半径−41 mm、Lstart9.0 mm、Lend11.6 mm、論理−90°/補正＋1.0°、SP PID1.50/0.02/0、OM PID0.10/0.002/0、Bの全FFで独立3本。前走127.2792 mm・後走63.6396 mmは斜め、速度0.5 m/s、回数1、吸引OFF。平均相対誤差は横＋0.618 mm・前後−0.165 mm、各軸最大誤差0.758/0.633 mm、出口方向差最大0.287°。終点ばらつき幅は横0.377 mm・前後1.011 mm。必要走行区間の観測率99.59%/100%/99.19%、固定288フレーム静止照合は全3本100%。全本で操作者が穴/壁接触なし・両方OFFを確認し、接続終了。

Aの3本候補の形状を初回に使用し、BのPID/全FFを保持した。初回の横不足7.509 mmに対し、入口長・出口長・角度をまとめた次候補で改善し、そのまま3本を確認した。個別パラメータの効果は分離していない。初期180/45案と実配置315/90との差で静止確認のみ0回停止した資料も保持。Aの実使用位置に近いB登録開始点(315,90) mm・前黄左下135°を使用し、開始位置のskills記録へ保存した。

判定は実測入口方向へ合わせた相対マーカー中点。盤面固定診断では入口が予定135°より約＋0.80〜2.44°、出口が予定225°より約＋1.08〜2.35°、停止点は横−1.58〜＋0.04 mm/縦−10.29〜−5.45 mm。盤面上では上へ約5〜10 mmの差が残る。設置・進入差を相対ターン値へ埋め込まず別評価する。盤面固定目標・高さ補正済み機軸・絶対車軸・左V90・連続接続は未検証。

Params/Param_B/turn_500.hへ未採用の候補コメントを追記し、コメント除外後のC++トークン一致とCRLF保持を確認した。実テーブル変更・ビルド・書込みなし。準備ヘルパーのB PID/FF保持、誤配置・古い準備確認・失敗時実行防止など11件が成功。撮影は高さ約1500 mm・2倍・露出−1 EV・AF固定・B前黄/後赤検出v8。
''',encoding='utf-8')
print('B_RV90_CANDIDATE_SETTINGS_META_COMMENT_ONLY_SOURCE_SAVED')
