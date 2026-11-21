"""既存カメラサーバーの状態を読むだけ。接続開始・撮影・設定変更なし。"""
from pathlib import Path
from datetime import datetime, timezone
import json
import sys
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'tools'))
from pixel_camera_capture import camera_workspace
sys.path.insert(0,str(camera_workspace()/'python'))
from pixel_camera import PixelCameraClient
status=PixelCameraClient(connection='wifi',port=8765,auto_start=False).get_status()
out=root/'tools/logs/pixel_camera/turn1600_B_outr135_20261121_resume_v1/camera_status.json'
result=dict(recorded_at=datetime.now(timezone.utc).isoformat(),status=status,
    camera_commands_sent=['get_status'],recording_started=False,robot_commands_sent=0)
with out.open('x',encoding='utf-8') as stream:json.dump(result,stream,ensure_ascii=False,indent=2)
print(json.dumps(dict(connected=status.get('connected'),phase=status.get('device',{}).get('phase'),recording_started=False),ensure_ascii=False))
