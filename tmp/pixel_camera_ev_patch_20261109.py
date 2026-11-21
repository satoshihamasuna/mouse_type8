"""Debug_cameraの変更を差分・原本バックアップ付きで適用する。"""
from pathlib import Path
import difflib
import hashlib
import json

root = Path('C:/Users/sato1/Documents/git/Debug_camera')
backup = Path(__file__).resolve().parents[1] / 'tools/logs/pixel_camera/20261109_brightness_aruco/source_backup'
changes = {}

def modify(relative, replacements):
    path = root / relative
    original = path.read_bytes()
    text = original.decode('utf-8').replace('\r\n', '\n')
    updated = text
    for old, new in replacements:
        if updated.count(old) != 1:
            raise RuntimeError(f'Expected one match in {relative}: {old[:80]!r}')
        updated = updated.replace(old, new, 1)
    ending = '\r\n' if b'\r\n' in original else '\n'
    changes[path] = (original, updated.replace('\n', ending).encode('utf-8'))

modify('app/src/main/java/com/example/myapplication/probe/CameraController.kt', [
    ('val iso: Int?, val ae: Int?, val af: Int?, val awb: Int?',
     'val iso: Int?, val ae: Int?, val af: Int?, val awb: Int?, val evSteps: Int? = null'),
    ('recordingId: String? = null, controllerId: String? = null, autoTransfer: Boolean = false,',
     'recordingId: String? = null, controllerId: String? = null, autoTransfer: Boolean = false,\n        exposureCompensationSteps: Int = 0,'),
    ('.put("requested_duration_seconds", seconds).put("audio", false).put("ev_steps_requested", 0)',
     '.put("requested_duration_seconds", seconds).put("audio", false).put("ev_steps_requested", exposureCompensationSteps)'),
    ('val characteristics = manager.getCameraCharacteristics(camera.id)',
     '''val characteristics = manager.getCameraCharacteristics(camera.id)
        val evRange = characteristics[CameraCharacteristics.CONTROL_AE_COMPENSATION_RANGE]
        ExposureCompensation.validate(exposureCompensationSteps.toDouble(),
            (evRange?.lower ?: 0)..(evRange?.upper ?: 0))
        metadata.put("ev_step", characteristics[CameraCharacteristics.CONTROL_AE_COMPENSATION_STEP]?.toString())'''),
    ('set(CaptureRequest.CONTROL_AE_MODE, CameraMetadata.CONTROL_AE_MODE_ON)',
     'set(CaptureRequest.CONTROL_AE_MODE, CameraMetadata.CONTROL_AE_MODE_ON)\n                set(CaptureRequest.CONTROL_AE_EXPOSURE_COMPENSATION, exposureCompensationSteps)'),
    ('result[CaptureResult.CONTROL_AF_STATE], result[CaptureResult.CONTROL_AWB_STATE]))',
     'result[CaptureResult.CONTROL_AF_STATE], result[CaptureResult.CONTROL_AWB_STATE],\n                        result[CaptureResult.CONTROL_AE_EXPOSURE_COMPENSATION]))'),
    ('frame_number,sensor_timestamp_ns,exposure_ns,frame_duration_ns,iso,ae_state,af_state,awb_state',
     'frame_number,sensor_timestamp_ns,exposure_ns,frame_duration_ns,iso,ae_state,af_state,awb_state,ev_steps_result'),
    ('s.frame, s.timestampNs, s.exposureNs, s.frameDurationNs, s.iso, s.ae, s.af, s.awb)',
     's.frame, s.timestampNs, s.exposureNs, s.frameDurationNs, s.iso, s.ae, s.af, s.awb, s.evSteps)'),
    ('val sensorCsv = File(storage.directory, "$base.sensor.csv")',
     '''metadata.put("ev_steps_result_values", org.json.JSONArray(sampleCopy.mapNotNull { it.evSteps }.distinct()))
        val sensorCsv = File(storage.directory, "$base.sensor.csv")'''),
])
modify('app/src/main/java/com/example/myapplication/remote/RecordingService.kt', [
    ('import android.content.Intent', 'import android.hardware.camera2.CameraManager\nimport android.hardware.camera2.CameraCharacteristics\nimport android.content.Intent'),
    ('private var lastTransferAttempt = 0L', '@Volatile private var exposureSteps = 0\n    private var lastTransferAttempt = 0L'),
    ('.put("transfer_percent", value.transferPercent)',
     '.put("transfer_percent", value.transferPercent).put("exposure_compensation_steps", exposureSteps)'),
    ('setOf("width", "height", "fps")', 'setOf("width", "height", "fps", "exposure_compensation_steps")'),
    ('JSONObject().put("width", 1280).put("height", 720).put("fps", 240)',
     '''if (payload.has("exposure_compensation_steps")) {
                        val camera = checkNotNull(report).cameras.firstOrNull { it.targetSupported } ?: error("NO_720P240_CAMERA")
                        val characteristics = getSystemService(CameraManager::class.java).getCameraCharacteristics(camera.id)
                        val range = characteristics[CameraCharacteristics.CONTROL_AE_COMPENSATION_RANGE]
                        val requested = payload.get("exposure_compensation_steps")
                        require(requested is Number) { "EXPOSURE_COMPENSATION_MUST_BE_INTEGER" }
                        exposureSteps = ExposureCompensation.validate(requested.toDouble(),
                            (range?.lower ?: 0)..(range?.upper ?: 0))
                    }
                    JSONObject().put("width", 1280).put("height", 720).put("fps", 240)
                        .put("exposure_compensation_steps", exposureSteps)'''),
    ('controller.prepare()\n        change', 'controller.prepare()\n        val selectedExposureSteps = exposureSteps\n        change'),
    ('controller.record(capabilities, camera, encoder, seconds, recordingId = id, controllerId = owner, autoTransfer = true)',
     'controller.record(capabilities, camera, encoder, seconds, recordingId = id, controllerId = owner, autoTransfer = true,\n                    exposureCompensationSteps = selectedExposureSteps)'),
])

new_files = {
    'app/src/main/java/com/example/myapplication/probe/ExposureCompensation.kt': '''package com.example.myapplication.probe

/** Request steps are integral and constrained by this device's camera characteristics. */
object ExposureCompensation {
    fun validate(value: Double, range: IntRange): Int {
        require(value.isFinite() && value == kotlin.math.floor(value)) { "EXPOSURE_COMPENSATION_MUST_BE_INTEGER" }
        require(value >= range.first && value <= range.last) { "EXPOSURE_COMPENSATION_OUT_OF_RANGE" }
        return value.toInt()
    }
}
''',
    'app/src/test/java/com/example/myapplication/probe/ExposureCompensationTest.kt': '''package com.example.myapplication.probe

import org.junit.Assert.assertEquals
import org.junit.Assert.assertThrows
import org.junit.Test

class ExposureCompensationTest {
    @Test fun acceptsDeviceRangeAndBothDirections() {
        for (value in listOf(-24, -6, 0, 6, 24)) {
            assertEquals(value, ExposureCompensation.validate(value.toDouble(), -24..24))
        }
    }
    @Test fun rejectsUnsupportedAndNonintegralValues() {
        for (value in listOf(-25.0, 25.0, 0.5, Double.NaN, Double.POSITIVE_INFINITY)) {
            assertThrows(IllegalArgumentException::class.java) { ExposureCompensation.validate(value, -24..24) }
        }
        assertThrows(IllegalArgumentException::class.java) { ExposureCompensation.validate(1.0, 0..0) }
    }
}
''',
}
for relative, content in new_files.items():
    path = root / relative
    if path.exists():
        raise FileExistsError(path)
    changes[path] = (None, content.encode('utf-8'))

backup.mkdir(parents=True, exist_ok=False)
diffs = []
hashes = {}
for path, (before, after) in changes.items():
    relative = path.relative_to(root)
    if before is not None:
        target = backup / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(before)
        hashes[str(relative)] = hashlib.sha256(before).hexdigest()
    diffs.extend(difflib.unified_diff((before or b'').decode('utf-8').splitlines(True),
        after.decode('utf-8').splitlines(True), fromfile=str(path), tofile=str(path)))
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(after)
(backup.parent / 'camera_source_changes.patch').write_text(''.join(diffs), encoding='utf-8')
(backup / 'original_sha256.json').write_text(json.dumps(hashes, indent=2)+'\n', encoding='utf-8')
print('Updated', len(changes), 'files; originals and patch:', backup)
