"""右V90の配置準備手順を、左の未調整500プリセットに合わせる。機器操作なし。"""
from pathlib import Path
import py_compile

root=Path(__file__).resolve().parents[1]
source=root/'tmp/prepare_r_v90_off_layout_20261111.py'
code=source.read_text(encoding='utf-8')
code=code.replace('r_v90','l_v90').replace('R V90','L V90').replace('右V90','左V90')

def once(old,new):
    global code
    assert code.count(old)==1,(old,code.count(old))
    code=code.replace(old,new)

once('right_mm=90/math.sqrt(2)+diagonal','right_mm=-(90/math.sqrt(2)+diagonal)')
once('exit_right_positive_deg=90','exit_right_positive_deg=-90')
once('r_min_mm=-41.','r_min_mm=41.')
once('degree_left_positive_deg=-90.','degree_left_positive_deg=90.')
once('slalom_RV90_500_table','slalom_LV90_500_table')
once('nominal_proposed_start_pose_xy_mm=[180,35]','nominal_proposed_start_pose_xy_mm=[315,90]')
once('image_view_start_from_left_mm=180,image_view_start_from_top_mm=35',
     'image_view_start_from_left_mm=45,image_view_start_from_top_mm=90')
once('前走2区画・後走1区画','前走127.2792 mm（2×DIAG_SECTION）・後走63.6396 mm（DIAG_SECTION）')
once('"r_min\']==-41 and parameters[\'degree\']==-90 and parameters[\'correction\']==0"',
     '"r_min\']==41 and parameters[\'degree\']==90 and parameters[\'correction\']==0"')
once('"exit_forward=right"','"exit_forward=-right"')
once('"assert target[\'right_mm\']>0 and target[\'exit_right_positive_deg\']==90"',
     '"assert target[\'right_mm\']<0 and target[\'exit_right_positive_deg\']==-90"')
once('左V90なので出口方向は開始時のright軸。','左V90なので出口方向は開始時の-right軸。')
once('start=np.array([180.,35.])','start=np.array([45.,90.])')
once('post=end-right*(diagonal+12.60)','post=end+right*(diagonal+12.60)')
once('np.column_stack([forward,right])','np.column_stack([forward,-right])')
once("xytext=(218,66)","xytext=(100,108)")
once("xytext=(70,12)","xytext=(8,40)")
once('機体中心：左右中央・上端から約35 mm','機体中心：左端から約45 mm・上端から約90 mm')
once('xytext=(12,96)','xytext=(90,25)')
once('OUT_L45_CANDIDATE_SAVED_OFF_RV90_PASSIVE_LAYOUT_PREPARED',
     'RV90_CANDIDATE_SAVED_OFF_LV90_PASSIVE_LAYOUT_PREPARED')
destination=root/'tmp/prepare_l_v90_off_layout_20261111.py'
assert not destination.exists()
destination.write_text(code,encoding='utf-8')
py_compile.compile(str(destination),doraise=True)
print(destination)
