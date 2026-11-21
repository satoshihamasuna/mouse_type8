"""次の独立stageだけ静止撮影/転送→BLEの順にする。通常起動は検証のみ。"""
from pathlib import Path
import ast
import hashlib
import json
import shutil

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tools/logs/pixel_camera/turn1600_B_inr135_20261120_v1'
STAGE=BASE/'geometry_c0p4_Ls10p5_Le17p2_02'
target=STAGE/'run_recorded_once.py'
source=target.read_text(encoding='utf-8-sig')
backup=STAGE/'run_recorded_once_before_camera_first.py'
assert not backup.exists();shutil.copyfile(target,backup)
needle='class RecordedService(tuning.McpTuningService):\n'
assert source.count(needle)==1
method='''    def perform(self, action, payload, identifier, progress):
        # Wi-Fi静止動画の転送が終わるまでBLE中継を起動しない。
        from pixel_camera_capture import camera_workspace
        sys.path.insert(0, str(camera_workspace()/'python'))
        from pixel_camera import PixelCameraClient
        camera = PixelCameraClient(connection='wifi', port=8765, auto_start=False)
        status = camera.get_status()
        if not status.get('connected') or status.get('device', {}).get('phase') != 'IDLE':
            raise RuntimeError('静止撮影前のカメラ待機状態を確認できません')
        before = self.folder/('camera_before_ble_'+identifier)
        before.mkdir()
        progress('static_check_before_ble', {})
        checked = dense_static_check(camera, self.capture_service(payload['config']), before, payload['placement'])
        original_check = tuning.static_check
        def checked_before_ble(camera, device, folder, placement):
            if placement != payload['placement']:
                raise RuntimeError('静止撮影後に配置条件が変わりました')
            result = dict(checked, camera_before_ble=True, source_static_check=str(before/'static_check.json'))
            tuning.save_json(folder/'static_check.json', result)
            return result
        tuning.static_check = checked_before_ble
        try:
            result = super().perform(action, payload, identifier, progress)
            result['camera_before_ble_static_folder'] = str(before)
            return result
        finally:
            tuning.static_check = original_check

'''
source=source.replace(needle,needle+method)
compile(source,str(target),'exec');ast.parse(source)
target.write_text(source,encoding='utf-8',newline='\n')
with (STAGE/'camera_before_ble_sequence.json').open('x',encoding='utf-8') as file:
    json.dump(dict(state='VALIDATED_NOT_EXECUTED',scope='only_next_independent_stage',
        previous_wrapper=str(backup),previous_sha256=hashlib.sha256(backup.read_bytes()).hexdigest(),
        next_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),
        sequence=['new_camera_static_capture','transfer_and_decode_validation','coarse_placement_check','BLE_connect',
            'device_settings_readback','motion_recording_ready','execute_once','stop_and_save','BLE_close'],
        settings_and_placement_unchanged=True,execution_resend_allowed=False,physical_disconnect_cause_proven=False),
        file,ensure_ascii=False,indent=2)
print('Prepared camera-before-BLE wrapper; no connection or robot command')
