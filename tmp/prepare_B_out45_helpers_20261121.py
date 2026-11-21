"""既存のB単発取得/解析をout45のenumと予め定めた目標・出口窓へ適用する。"""
from pathlib import Path
import json
import re
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_outr45_20261121_v1'
old_base='turn1600_B_inr45_20261120_v1';new_base=BASE.name
pairs={
 'review_B_inr45_tracking_20261120.py':'review_B_out45_tracking_20261121.py',
 'review_B_inr45_midpoint_20261120.py':'review_B_out45_midpoint_20261121.py',
 'postprocess_B_inr45_20261120.py':'postprocess_B_out45_20261121.py',
 'record_B_inr45_next_operator_ready_20261121.py':'record_B_out45_next_operator_ready_20261121.py',
 'checkpoint_B_inr45_next_after_run_20261121.py':'checkpoint_B_out45_next_after_run_20261121.py'}
for old_name,new_name in pairs.items():
 source=(ROOT/'tmp'/old_name).read_text(encoding='utf-8-sig').replace(old_base,new_base).replace("'in_r45'","'out_r45'")
 for old,new in pairs.items():source=source.replace(old,new)
 if old_name.startswith('review_'):
  source=re.sub(r'^RUN=BASE/\(sys\.argv\[1\].*$',"RUN=BASE/sys.argv[1]\nassert RUN.resolve().is_relative_to(BASE.resolve())",source,flags=re.M)
 if 'tracking' in old_name:source=source.replace('B1600右135°入口','B1600右45°出口')
 if 'midpoint' in old_name:
  replacements={"raw['motion.pattern'].to_numpy()==7":"raw['motion.pattern'].to_numpy()==9",
   'exit_window=[lend_time+.015,pts(stop)-.025]':'exit_window=[lend_time+.010,pts(stop)-.010]',
   'target=np.array([45.,180.])':'target=np.array([45./np.sqrt(2),315./np.sqrt(2)])\nstop_target=np.array([135./np.sqrt(2),405./np.sqrt(2)])',
   '# 動画原点は前走90mmの開始位置。in45単体の前90mmを加えた目標。':'# 動画原点は斜め前走127.279mmの開始位置。out45の格子目標を入口座標へ変換。',
   "midpoint_Lend_residual_mm=(measured-target).tolist(),length_only_candidate_at_current_angle=lengths(lend_time),":"midpoint_Lend_residual_mm=(measured-target).tolist(),length_only_candidate_at_current_angle=lengths(lend_time),\n    target_midpoint_stop_right_forward_mm=stop_target.tolist(),midpoint_stop_residual_mm=(np.median(local[steady],axis=0)-stop_target).tolist(),\n    evaluation_policy=read(BASE/'analysis_policy.json'),",
   "RUN/'marker_reference_used.json',RUN/'body_geometry_used.json',Path(__file__)":"RUN/'marker_reference_used.json',RUN/'body_geometry_used.json',BASE/'analysis_policy.json',Path(__file__)",
   '機体B1600 右45度入口':'機体B1600 右45度出口',
   '前走を含む目標中点45/180mm':'前走を含む目標中点31.82/222.74mm'}
  for old,new in replacements.items():
   assert old in source,(old_name,old);source=source.replace(old,new)
 if 'checkpoint' in old_name:
  old="state['next_request_id']==identifier and state['request_id']!=identifier"
  assert old in source;source=source.replace(old,"state['next_request_id']==identifier and not state['next_request_sent']")
 destination=ROOT/'tmp'/new_name;compile(source,str(destination),'exec')
 with destination.open('x',encoding='utf-8') as f:f.write(source)
state_path=BASE/'resume_state.json';state=json.loads(state_path.read_text(encoding='utf-8'))
state.update(next_stage=state['stage'],next_request_id=state['request_id'],next_request_sent=False,next_operator_ready=False,
 next_settings=state['selected_settings'],readiness_question_pending=None)
state['ble_binding']['hardware_connections_started']=0
state_path.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print('Out45 enum9, fixed 10/10ms exit window and targets configured; no hardware command')
