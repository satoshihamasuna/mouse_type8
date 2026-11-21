package com.example.myapplication.probe

import android.content.Context
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraManager
import android.media.MediaCodecInfo
import android.media.MediaCodecList
import android.media.MediaFormat
import android.os.Build
import android.util.Range
import android.util.Size
import org.json.JSONArray
import org.json.JSONObject
import java.time.Instant

const val TARGET_WIDTH = 1280
const val TARGET_HEIGHT = 720
const val TARGET_FPS = 240
const val TARGET_BITRATE = 48_000_000

data class CameraOption(val id: String, val label: String, val targetSupported: Boolean)
data class EncoderOption(
    val name: String, val hardware: Boolean, val surfaceInput: Boolean,
    val sizeAndRateSupported: Boolean, val formatSupported: Boolean, val bitrate: Int
)
data class CapabilityReport(
    val json: JSONObject, val cameras: List<CameraOption>, val encoders: List<EncoderOption>
)

fun jsonArray(values: Iterable<*>): JSONArray = JSONArray().apply { values.forEach { put(it) } }

class DeviceCapabilityManager(context: Context) {
    private val manager = context.getSystemService(CameraManager::class.java)

    fun inspect(): CapabilityReport {
        val cameras = mutableListOf<CameraOption>()
        val cameraJson = JSONArray()
        for (id in manager.cameraIdList) {
            val entry = JSONObject().put("id", id)
            try {
                val c = manager.getCameraCharacteristics(id)
                val back = c[CameraCharacteristics.LENS_FACING] == CameraCharacteristics.LENS_FACING_BACK
                val capabilities = c[CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES] ?: intArrayOf()
                val highSpeed = capabilities.contains(CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES_CONSTRAINED_HIGH_SPEED_VIDEO)
                val map = c[CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP]
                val modes = JSONArray()
                var target = false
                if (highSpeed && map != null) {
                    for (size in map.highSpeedVideoSizes) {
                        val ranges = map.getHighSpeedVideoFpsRangesFor(size)
                        modes.put(JSONObject().put("width", size.width).put("height", size.height)
                            .put("fps_ranges", jsonArray(ranges.map { listOf(it.lower, it.upper).let(::jsonArray) })))
                        if (size == Size(TARGET_WIDTH, TARGET_HEIGHT) && ranges.contains(Range(TARGET_FPS, TARGET_FPS))) target = true
                    }
                }
                val level = when (c[CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL]) {
                    CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_LEGACY -> "LEGACY"
                    CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_LIMITED -> "LIMITED"
                    CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_FULL -> "FULL"
                    CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_3 -> "LEVEL_3"
                    CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_EXTERNAL -> "EXTERNAL"
                    else -> "UNKNOWN"
                }
                entry.put("back_facing", back).put("hardware_level", level)
                    .put("capabilities", jsonArray(capabilities.toList())).put("high_speed", highSpeed)
                    .put("high_speed_modes", modes).put("target_fixed_240_supported", target)
                    .put("flash_available", c[CameraCharacteristics.FLASH_INFO_AVAILABLE] == true)
                    .put("sensor_orientation", c[CameraCharacteristics.SENSOR_ORIENTATION])
                    .put("timestamp_source", c[CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE])
                    .put("focal_lengths_mm", jsonArray(c[CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS]?.toList().orEmpty()))
                    .put("normal_ae_fps_ranges", jsonArray(c[CameraCharacteristics.CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES]?.map { it.toString() }.orEmpty()))
                    .put("normal_iso_range", c[CameraCharacteristics.SENSOR_INFO_SENSITIVITY_RANGE]?.toString())
                    .put("normal_exposure_time_ns_range", c[CameraCharacteristics.SENSOR_INFO_EXPOSURE_TIME_RANGE]?.toString())
                    .put("ev_steps_range", c[CameraCharacteristics.CONTROL_AE_COMPENSATION_RANGE]?.toString())
                    .put("ev_step", c[CameraCharacteristics.CONTROL_AE_COMPENSATION_STEP]?.toString())
                    .put("normal_af_modes", jsonArray(c[CameraCharacteristics.CONTROL_AF_AVAILABLE_MODES]?.toList().orEmpty()))
                    .put("normal_awb_modes", jsonArray(c[CameraCharacteristics.CONTROL_AWB_AVAILABLE_MODES]?.toList().orEmpty()))
                    .put("minimum_focus_distance_diopters", c[CameraCharacteristics.LENS_INFO_MINIMUM_FOCUS_DISTANCE])
                    .put("request_keys", jsonArray(c.availableCaptureRequestKeys.orEmpty().map { it.name }))
                if (back) cameras.add(CameraOption(id, "背面 ID $id / $level / 720p240固定: ${if (target) "申告あり" else "申告なし"}", target))
            } catch (e: Exception) {
                entry.put("query_error", e.toString())
            }
            cameraJson.put(entry)
        }
        val encoders = mutableListOf<EncoderOption>()
        val codecJson = JSONArray()
        for (codec in MediaCodecList(MediaCodecList.ALL_CODECS).codecInfos) {
            if (!codec.isEncoder || codec.isAlias || !codec.supportedTypes.any { it.equals(MediaFormat.MIMETYPE_VIDEO_AVC, true) }) continue
            val entry = JSONObject().put("name", codec.name)
            try {
                val caps = codec.getCapabilitiesForType(MediaFormat.MIMETYPE_VIDEO_AVC)
                val video = checkNotNull(caps.videoCapabilities) { "Video capabilities unavailable" }
                val surface = caps.colorFormats.contains(MediaCodecInfo.CodecCapabilities.COLOR_FormatSurface)
                val bitrate = video.bitrateRange.clamp(TARGET_BITRATE)
                val rate = video.areSizeAndRateSupported(TARGET_WIDTH, TARGET_HEIGHT, TARGET_FPS.toDouble())
                val format = caps.isFormatSupported(encoderFormat(bitrate))
                encoders.add(EncoderOption(codec.name, codec.isHardwareAccelerated, surface, rate, format, bitrate))
                entry.put("hardware", codec.isHardwareAccelerated).put("software_only", codec.isSoftwareOnly)
                    .put("surface_input", surface).put("size_and_rate_supported", rate).put("format_supported", format)
                    .put("test_bitrate", bitrate).put("bitrate_range", video.bitrateRange.toString())
                    .put("supported_fps_for_720p", runCatching { video.getSupportedFrameRatesFor(TARGET_WIDTH, TARGET_HEIGHT).toString() }.getOrNull())
                    .put("achievable_fps_for_720p_estimate_only", runCatching { video.getAchievableFrameRatesFor(TARGET_WIDTH, TARGET_HEIGHT)?.toString() }.getOrNull())
                    .put("performance_points", jsonArray(video.supportedPerformancePoints?.map { it.toString() }.orEmpty()))
                    .put("profile_levels", jsonArray(caps.profileLevels.map { JSONObject().put("profile", it.profile).put("level", it.level) }))
            } catch (e: Exception) {
                entry.put("query_error", e.toString())
            }
            codecJson.put(entry)
        }
        val report = JSONObject()
            .put("schema_version", 1).put("created_at", Instant.now().toString())
            .put("device", JSONObject().put("manufacturer", Build.MANUFACTURER).put("model", Build.MODEL)
                .put("device", Build.DEVICE).put("android_release", Build.VERSION.RELEASE).put("sdk", Build.VERSION.SDK_INT)
                .put("build_fingerprint", Build.FINGERPRINT).put("security_patch", Build.VERSION.SECURITY_PATCH))
            .put("target", JSONObject().put("width", TARGET_WIDTH).put("height", TARGET_HEIGHT).put("fps", TARGET_FPS)
                .put("mime", MediaFormat.MIMETYPE_VIDEO_AVC))
            .put("cameras", cameraJson).put("avc_encoders", codecJson)
            .put("high_speed_controls", JSONObject().put("ae", "AUTO").put("af", "CONTINUOUS_VIDEO")
                .put("awb", "AUTO").put("manual_iso_enabled", false).put("manual_shutter_enabled", false)
                .put("ev_effect_verified", false))
            .put("recording_verified", false)
            .put("note", "API declarations are not proof of sustained or physically distinct 240 fps. Test recording and a moving timed target are required.")
        return CapabilityReport(report, cameras, encoders.sortedWith(compareByDescending<EncoderOption> { it.hardware }
            .thenByDescending { it.sizeAndRateSupported && it.formatSupported }))
    }
}

fun encoderFormat(bitrate: Int): MediaFormat =
    MediaFormat.createVideoFormat(MediaFormat.MIMETYPE_VIDEO_AVC, TARGET_WIDTH, TARGET_HEIGHT).apply {
        setInteger(MediaFormat.KEY_COLOR_FORMAT, MediaCodecInfo.CodecCapabilities.COLOR_FormatSurface)
        setInteger(MediaFormat.KEY_BIT_RATE, bitrate)
        setInteger(MediaFormat.KEY_FRAME_RATE, TARGET_FPS)
        setInteger(MediaFormat.KEY_I_FRAME_INTERVAL, 1)
        setInteger(MediaFormat.KEY_MAX_B_FRAMES, 0)
        // Preserve the camera's timestamps. Do not set CAPTURE_RATE or stretch PTS to 30 fps.
    }
