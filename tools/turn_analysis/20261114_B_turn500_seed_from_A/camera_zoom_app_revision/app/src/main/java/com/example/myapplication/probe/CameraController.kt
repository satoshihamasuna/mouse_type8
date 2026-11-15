package com.example.myapplication.probe

import android.Manifest
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.os.BatteryManager
import android.content.pm.PackageManager
import android.hardware.camera2.*
import android.hardware.camera2.params.OutputConfiguration
import android.hardware.camera2.params.SessionConfiguration
import android.os.Handler
import android.os.HandlerThread
import android.os.PowerManager
import android.os.SystemClock
import android.util.Range
import androidx.core.content.ContextCompat
import org.json.JSONObject
import java.io.File
import java.time.Instant
import java.util.Collections
import java.util.concurrent.CompletableFuture
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicInteger
import java.util.concurrent.atomic.AtomicReference

data class CaptureSample(
    val frame: Long, val timestampNs: Long?, val exposureNs: Long?, val frameDurationNs: Long?,
    val iso: Int?, val ae: Int?, val af: Int?, val awb: Int?, val evSteps: Int? = null, val flashMode: Int? = null,
    val afMode: Int? = null, val afTrigger: Int? = null, val focusDistance: Float? = null,
    val zoomRatio: Float? = null
)
data class ProbeOutcome(val summary: String, val metadata: File)

/** Blocking orchestration runs only on the ViewModel's worker. Camera/codec have separate threads. */
class CameraController(private val context: Context, private val storage: ProbeStorage) {
    private val stopRequested = AtomicBoolean(false)

    private fun batteryTemperatureC(): Double? {
        val intent = context.registerReceiver(null, IntentFilter(Intent.ACTION_BATTERY_CHANGED)) ?: return null
        if (!intent.hasExtra(BatteryManager.EXTRA_TEMPERATURE)) return null
        return intent.getIntExtra(BatteryManager.EXTRA_TEMPERATURE, 0) / 10.0
    }

    fun requestStop() { stopRequested.set(true) }
    fun prepare() { stopRequested.set(false) }

    fun record(
        report: CapabilityReport, camera: CameraOption, encoderOption: EncoderOption, seconds: Int,
        recordingId: String? = null, controllerId: String? = null, autoTransfer: Boolean = false,
        exposureCompensationSteps: Int = 0, torch: Boolean = false, focusLock: Boolean = false, zoomRatio: Float = 1f,
        onProgress: (String, Double) -> Unit
    ): ProbeOutcome {
        require(seconds in 1..300)
        check(camera.targetSupported) { "Camera2に720p [240,240]の申告がありません" }
        check(encoderOption.surfaceInput) { "Surface入力に対応していません" }
        check(ContextCompat.checkSelfPermission(context, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED) {
            "カメラ権限がありません"
        }
        storage.requireSpace(seconds, encoderOption.bitrate)
        val base = recordingId ?: storage.newBase()
        require(base.matches(Regex("[A-Za-z0-9_-]{1,96}")))
        val partial = File(storage.directory, "$base.partial.mp4")
        val metadata = JSONObject().put("schema_version", 2).put("started_at", Instant.now().toString())
            .put("camera_id", camera.id).put("encoder", encoderOption.name)
            .put("encoder_declares_target", encoderOption.sizeAndRateSupported && encoderOption.formatSupported)
            .put("capabilities_snapshot", report.json).put("configured_fps", TARGET_FPS)
            .put("width", TARGET_WIDTH).put("height", TARGET_HEIGHT).put("bitrate", encoderOption.bitrate)
            .put("requested_duration_seconds", seconds).put("audio", false).put("ev_steps_requested", exposureCompensationSteps)
            .put("torch_requested", torch).put("zoom_ratio_requested", zoomRatio)
            .put("physical_240fps_verified", false).put("transfer_status", "NOT_IMPLEMENTED_RETAINED_ON_DEVICE")
            .put("verification_note", "High-speed Camera2 callbacks may be batched/interpolated by Android. Matching timestamps do not prove independent physical frames.")
        if (autoTransfer) {
            metadata.put("recording_id", base).put("remote_controller_id", controllerId)
                .put("transfer_protocol", 1).put("transfer_status", "PENDING")
        }
        // A durable attempt record also identifies files left by process death.
        storage.json("$base.json", metadata.put("state", "PREPARING").put("video_file", partial.name))
        val manager = context.getSystemService(CameraManager::class.java)
        val power = context.getSystemService(PowerManager::class.java)
        metadata.put("thermal_status_before", power.currentThermalStatus)
            .put("battery_temperature_c_before", batteryTemperatureC() ?: JSONObject.NULL)
            .put("available_bytes_before", storage.availableBytes())
        val characteristics = manager.getCameraCharacteristics(camera.id)
        check(!torch || characteristics[CameraCharacteristics.FLASH_INFO_AVAILABLE] == true) { "TORCH_UNAVAILABLE" }
        val zoomRange = characteristics[CameraCharacteristics.CONTROL_ZOOM_RATIO_RANGE]
        val zoomSupported = characteristics.availableCaptureRequestKeys.contains(CaptureRequest.CONTROL_ZOOM_RATIO)
        DigitalZoom.validate(zoomRatio.toDouble(), zoomRange?.lower ?: 1f, zoomRange?.upper ?: 1f, zoomSupported)
        val zoom = DigitalZoom(zoomRatio)
        val evRange = characteristics[CameraCharacteristics.CONTROL_AE_COMPENSATION_RANGE]
        ExposureCompensation.validate(exposureCompensationSteps.toDouble(),
            (evRange?.lower ?: 0)..(evRange?.upper ?: 0))
        metadata.put("ev_step", characteristics[CameraCharacteristics.CONTROL_AE_COMPENSATION_STEP]?.toString())
        val cameraThread = HandlerThread("camera2-probe").apply { start() }
        val cameraHandler = Handler(cameraThread.looper)
        // Open/session callbacks must remain deliverable even after this attempt times out.
        val callbackExecutor = ContextCompat.getMainExecutor(context)
        val opened = CompletableFuture<CameraDevice>()
        val closed = CompletableFuture<Unit>()
        val configured = CompletableFuture<CameraCaptureSession>()
        val readyToStop = AtomicReference<CompletableFuture<Unit>?>(null)
        val acceptingCallbacks = AtomicBoolean(true)
        val cameraError = AtomicReference<String?>(null)
        val focus = ContinuousFocusLock(focusLock)
        val focusTriggerFrame = AtomicReference<Long?>(null)
        val failures = AtomicInteger()
        val bufferLosses = AtomicInteger()
        val samples = Collections.synchronizedList(mutableListOf<CaptureSample>())
        val starts = Collections.synchronizedList(mutableListOf<Pair<Long, Long>>())
        val issues = mutableListOf<String>()
        var device: CameraDevice? = null
        var session: CameraCaptureSession? = null
        var encoder: AvcEncoder? = null
        var encoded: EncoderResult? = null
        var elapsed = 0.0
        var recordingStartedNs: Long? = null
        var sessionCreated = false
        try {
            check(!stopRequested.get()) { "開始前に停止が要求されました" }
            onProgress("エンコーダを準備中", 0.0)
            encoder = AvcEncoder(encoderOption, partial,
                characteristics[CameraCharacteristics.SENSOR_ORIENTATION] ?: 0, seconds * 1_000_000L)
            val stateCallback = object : CameraDevice.StateCallback() {
                override fun onOpened(cameraDevice: CameraDevice) {
                    opened.complete(cameraDevice)
                    if (!acceptingCallbacks.get()) cameraDevice.close()
                }
                override fun onDisconnected(cameraDevice: CameraDevice) {
                    cameraError.set("CAMERA_DISCONNECTED")
                    opened.completeExceptionally(IllegalStateException("CAMERA_DISCONNECTED"))
                    cameraDevice.close()
                }
                override fun onError(cameraDevice: CameraDevice, error: Int) {
                    cameraError.set("CAMERA_ERROR_$error")
                    opened.completeExceptionally(IllegalStateException("CAMERA_ERROR_$error"))
                    cameraDevice.close()
                }
                override fun onClosed(cameraDevice: CameraDevice) { closed.complete(Unit) }
            }
            // Permissions can be revoked after the explicit check; SecurityException is caught below.
            manager.openCamera(camera.id, callbackExecutor, stateCallback)
            onProgress("カメラを開いています", 0.0)
            device = opened.get(10, TimeUnit.SECONDS)
            check(!stopRequested.get()) { "セッション作成前に停止が要求されました" }
            val sessionCallback = object : CameraCaptureSession.StateCallback() {
                override fun onConfigured(captureSession: CameraCaptureSession) {
                    configured.complete(captureSession)
                    if (!acceptingCallbacks.get()) captureSession.close()
                }
                override fun onConfigureFailed(captureSession: CameraCaptureSession) {
                    captureSession.close()
                    configured.completeExceptionally(IllegalStateException("HIGH_SPEED_SESSION_CONFIGURATION_FAILED"))
                }
                override fun onReady(captureSession: CameraCaptureSession) { readyToStop.get()?.complete(Unit) }
            }
            onProgress("高速撮影セッションを作成中", 0.0)
            device.createCaptureSession(SessionConfiguration(
                SessionConfiguration.SESSION_HIGH_SPEED,
                listOf(OutputConfiguration(encoder.surface)), callbackExecutor, sessionCallback
            ))
            session = configured.get(10, TimeUnit.SECONDS)
            val highSpeed = session as? CameraConstrainedHighSpeedCaptureSession
                ?: error("作成されたセッションが高速撮影セッションではありません")
            sessionCreated = true
            val requestBuilder = device.createCaptureRequest(CameraDevice.TEMPLATE_RECORD).apply {
                addTarget(encoder.surface)
                set(CaptureRequest.CONTROL_MODE, CameraMetadata.CONTROL_MODE_AUTO)
                set(CaptureRequest.CONTROL_AE_MODE, CameraMetadata.CONTROL_AE_MODE_ON)
                set(CaptureRequest.CONTROL_AE_EXPOSURE_COMPENSATION, exposureCompensationSteps)
                if (zoomSupported) set(CaptureRequest.CONTROL_ZOOM_RATIO, zoomRatio)
                set(CaptureRequest.FLASH_MODE, if (torch) CameraMetadata.FLASH_MODE_TORCH else CameraMetadata.FLASH_MODE_OFF)
                set(CaptureRequest.CONTROL_AWB_MODE, CameraMetadata.CONTROL_AWB_MODE_AUTO)
                set(CaptureRequest.CONTROL_AF_MODE, CameraMetadata.CONTROL_AF_MODE_CONTINUOUS_VIDEO)
                set(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, Range(TARGET_FPS, TARGET_FPS))
                set(CaptureRequest.CONTROL_AF_TRIGGER, CameraMetadata.CONTROL_AF_TRIGGER_IDLE)
            }
            val request = requestBuilder.build()
            val callback = object : CameraCaptureSession.CaptureCallback() {
                override fun onCaptureStarted(s: CameraCaptureSession, r: CaptureRequest, timestamp: Long, frameNumber: Long) {
                    starts.add(frameNumber to timestamp)
                }
                override fun onCaptureCompleted(s: CameraCaptureSession, r: CaptureRequest, result: TotalCaptureResult) {
                    samples.add(CaptureSample(result.frameNumber, result[CaptureResult.SENSOR_TIMESTAMP],
                        result[CaptureResult.SENSOR_EXPOSURE_TIME], result[CaptureResult.SENSOR_FRAME_DURATION],
                        result[CaptureResult.SENSOR_SENSITIVITY], result[CaptureResult.CONTROL_AE_STATE],
                        result[CaptureResult.CONTROL_AF_STATE], result[CaptureResult.CONTROL_AWB_STATE],
                        result[CaptureResult.CONTROL_AE_EXPOSURE_COMPENSATION], result[CaptureResult.FLASH_MODE],
                        result[CaptureResult.CONTROL_AF_MODE], r[CaptureRequest.CONTROL_AF_TRIGGER],
                        result[CaptureResult.LENS_FOCUS_DISTANCE], result[CaptureResult.CONTROL_ZOOM_RATIO]))
                    zoom.observe(result[CaptureResult.CONTROL_ZOOM_RATIO])
                    zoom.error?.let { cameraError.compareAndSet(null, it) }
                    when (focus.observe(result[CaptureResult.CONTROL_AF_STATE], SystemClock.elapsedRealtimeNanos())) {
                        ContinuousFocusLock.Action.TRIGGER -> {
                            try {
                                // High-speed sessions require an unmodified createHighSpeedRequestList.
                                // START is sent once as a nonrepeating burst; the IDLE burst resumes.
                                requestBuilder.set(CaptureRequest.CONTROL_AF_TRIGGER, CameraMetadata.CONTROL_AF_TRIGGER_START)
                                val triggerBurst = highSpeed.createHighSpeedRequestList(requestBuilder.build())
                                highSpeed.captureBurst(triggerBurst, this, cameraHandler)
                                focusTriggerFrame.set(result.frameNumber)
                            } catch (error: Exception) {
                                cameraError.compareAndSet(null, "FOCUS_LOCK_TRIGGER_FAILED: $error")
                            } finally {
                                requestBuilder.set(CaptureRequest.CONTROL_AF_TRIGGER, CameraMetadata.CONTROL_AF_TRIGGER_IDLE)
                            }
                        }
                        ContinuousFocusLock.Action.FAIL -> cameraError.compareAndSet(null, focus.error)
                        ContinuousFocusLock.Action.NONE -> Unit
                    }
                }
                override fun onCaptureFailed(s: CameraCaptureSession, r: CaptureRequest, failure: CaptureFailure) {
                    failures.incrementAndGet()
                }
                override fun onCaptureBufferLost(s: CameraCaptureSession, r: CaptureRequest, target: android.view.Surface, frameNumber: Long) {
                    bufferLosses.incrementAndGet()
                }
            }
            val burst = highSpeed.createHighSpeedRequestList(request)
            metadata.put("high_speed_burst_size", burst.size)
            check(!stopRequested.get()) { "録画開始前に停止が要求されました" }
            highSpeed.setRepeatingBurst(burst, callback, cameraHandler)
            val startNs = SystemClock.elapsedRealtimeNanos()
            recordingStartedNs = startNs
            var lastUpdate = 0L
            while (!stopRequested.get()) {
                val now = SystemClock.elapsedRealtimeNanos()
                elapsed = (now - startNs) / 1e9
                val progress = encoder.progress
                if (progress.limitReached) break
                cameraError.get()?.let { error(it) }
                check(!encoder.completed.isDone) { "録画中にエンコーダが終了しました" }
                check(zoom.confirmed || elapsed < 5.0) { "ZOOM_RATIO_NOT_CONFIRMED" }
                check(elapsed < seconds + 15.0) { "録画時間の上限に到達する前にタイムアウトしました" }
                check(progress.firstFrameAtNs != null || elapsed < 10.0) { "最初の映像フレームが届きません" }
                progress.lastOutputAtNs?.let {
                    check(now - it < 5_000_000_000L) { "映像出力が5秒間停止しました" }
                }
                if (now - lastUpdate >= 250_000_000) {
                    check(storage.availableBytes() > 64L * 1024 * 1024) { "ストレージ残量が64 MiBを下回りました" }
                    onProgress(if (progress.firstFrameAtNs == null) "最初の映像フレームを待機中"
                        else if (!zoom.confirmed) "ズームを確認中"
                        else if (focusLock && !focus.locked) "ピントを固定中" else "録画中",
                        progress.elapsedSeconds)
                    lastUpdate = now
                }
                Thread.sleep(25)
            }
            metadata.put("stop_reason", if (stopRequested.get()) "USER_OR_ACTIVITY_STOP" else "DURATION")
        } catch (e: Exception) {
            issues.add(e.cause?.toString() ?: e.toString())
        } finally {
            recordingStartedNs?.let { elapsed = (SystemClock.elapsedRealtimeNanos() - it) / 1e9 }
            onProgress("停止処理・MP4確定中", encoder?.progress?.elapsedSeconds ?: 0.0)
            if (sessionCreated && session != null) {
                try {
                    val ready = CompletableFuture<Unit>()
                    readyToStop.set(ready)
                    session.stopRepeating()
                    ready.get(4, TimeUnit.SECONDS)
                } catch (e: Exception) {
                    issues.add("Camera drain: ${e.cause ?: e}")
                }
            }
            // Prevent an open/configure callback that arrives after a timeout from leaking a camera.
            acceptingCallbacks.set(false)
            // A successful future may have completed just after a timed get failed.
            if (device == null && opened.isDone && !opened.isCompletedExceptionally) device = opened.getNow(null)
            if (session == null && configured.isDone && !configured.isCompletedExceptionally) session = configured.getNow(null)
            runCatching { session?.close() }.onFailure { issues.add(it.toString()) }
            if (device != null) {
                runCatching { device.close(); closed.get(5, TimeUnit.SECONDS) }
                    .onFailure { issues.add("Camera close: $it") }
            }
            if (encoder != null) {
                runCatching { encoded = encoder.finish() }.onFailure { issues.add("Encoder finish: $it") }
                // Camera is closed before releasing its input Surface.
                runCatching { encoder.surface.release() }
            }
            cameraError.get()?.let { if (it !in issues) issues.add(it) }
            if (!zoom.confirmed && "ZOOM_RATIO_NOT_CONFIRMED" !in issues) issues.add("ZOOM_RATIO_NOT_CONFIRMED")
            cameraThread.quitSafely()
            cameraThread.join(2000)
            if (cameraThread.isAlive) issues.add("CAMERA_CALLBACK_THREAD_DRAIN_TIMEOUT")
        }
        if (focusLock && !focus.locked) issues.add(focus.error ?: "FOCUS_LOCK_NOT_CONFIRMED")
        metadata.put("focus_lock", JSONObject().put("requested", focusLock).put("state", focus.state.name)
            .put("trigger_frame", focusTriggerFrame.get() ?: JSONObject.NULL)
            .put("error", focus.error ?: JSONObject.NULL)
            .put("trigger_submission", "one_nonrepeating_high_speed_burst_then_idle")
            .put("individual_sensor_frames_verified", false))
        val sampleCopy = synchronized(samples) { samples.toList() }
        val startCopy = synchronized(starts) { starts.toList() }
        val sensorStats = TimingAnalysis.analyze(sampleCopy.mapNotNull { it.timestampNs })
        val encodedStats = TimingAnalysis.analyze(encoded?.ptsNs.orEmpty())
        val stampedSamples = sampleCopy.filter { it.timestampNs != null }
        val sampling = TimingAnalysis.sensorSampling(stampedSamples.map { it.frame },
            stampedSamples.map { checkNotNull(it.timestampNs) }, metadata.optInt("high_speed_burst_size", 1))
        val sensorJson = sensorStats.toJson().put("interpretation", "callback_cadence_not_capture_fps")
        if (sampling.mode != "EVERY_FRAME") {
            sensorJson.remove("estimated_missing_frames")
            sensorJson.remove("gaps_over_1_5_frames")
        }
        val startedJson = TimingAnalysis.analyze(startCopy.map { it.second }).toJson()
            .put("interpretation", "callback_cadence_not_capture_fps")
        if (sampling.mode != "EVERY_FRAME") {
            startedJson.remove("estimated_missing_frames")
            startedJson.remove("gaps_over_1_5_frames")
        }
        val samplingJson = JSONObject().put("mode", sampling.mode).put("frame_number_step", sampling.frameStep)
            .put("fps_from_frame_number_span_estimate", sampling.fpsFromFrameNumbers)
            .put("estimated_covered_frames", sampling.estimatedCoveredFrames)
            .put("individual_sensor_frames_verified", false)
        encoded?.error?.let { issues.add(it) }
        var video = partial
        var mp4: JSONObject? = null
        if (encoded?.eos == true && encoded.muxerFinalized && encoded.ptsNs.isNotEmpty()) {
            runCatching { storage.inspectMp4(partial) }.onSuccess {
                mp4 = it
                if (it.getInt("sample_count") != encoded.ptsNs.size) issues.add("MP4_SAMPLE_COUNT_MISMATCH")
                if (it.getInt("width") != TARGET_WIDTH || it.getInt("height") != TARGET_HEIGHT) issues.add("MP4_DIMENSIONS_MISMATCH")
            }.onFailure { issues.add("MP4 inspection: $it") }
            if (issues.isEmpty()) {
                val finalized = File(storage.directory, "$base.mp4")
                if (partial.renameTo(finalized)) video = finalized else issues.add("MP4_RENAME_FAILED")
            }
        } else {
            issues.add("MP4_NOT_CLEANLY_FINALIZED")
        }
        metadata.put("zoom_verification", JSONObject().put("confirmed", zoom.confirmed).put("error", zoom.error ?: JSONObject.NULL)
            .put("result_values", org.json.JSONArray(sampleCopy.mapNotNull { it.zoomRatio }.distinct()))
            .put("individual_sensor_frames_verified", false))
        metadata.put("ev_steps_result_values", org.json.JSONArray(sampleCopy.mapNotNull { it.evSteps }.distinct()))
            .put("flash_mode_result_values", org.json.JSONArray(sampleCopy.mapNotNull { it.flashMode }.distinct()))
        val sensorCsv = File(storage.directory, "$base.sensor.csv")
        sensorCsv.bufferedWriter().use { writer ->
            writer.appendLine("frame_number,sensor_timestamp_ns,exposure_ns,frame_duration_ns,iso,ae_state,af_state,awb_state,ev_steps_result,flash_mode_result,af_mode_result,af_trigger_requested,focus_distance_diopters,zoom_ratio_result")
            sampleCopy.forEach { s -> writer.appendLine(listOf(s.frame, s.timestampNs, s.exposureNs, s.frameDurationNs, s.iso, s.ae, s.af, s.awb, s.evSteps, s.flashMode, s.afMode, s.afTrigger, s.focusDistance, s.zoomRatio).joinToString(",") { it?.toString() ?: "" }) }
        }
        File(storage.directory, "$base.started.csv").bufferedWriter().use { writer ->
            writer.appendLine("frame_number,started_timestamp_ns")
            startCopy.forEach { writer.appendLine("${it.first},${it.second}") }
        }
        File(storage.directory, "$base.pts.csv").bufferedWriter().use { writer ->
            writer.appendLine("encoded_sample_index,presentation_time_ns")
            encoded?.ptsNs?.forEachIndexed { index, pts -> writer.appendLine("$index,$pts") }
        }
        val assessment = if (issues.isEmpty()) TimingAnalysis.assessment(sensorStats, encodedStats,
            failures.get(), bufferLosses.get(), elapsed, sampling, encoded?.window?.discardedTailFrames ?: 0) else "RECORDING_ERROR"
        val window = encoded?.window
        val windowJson = JSONObject().put("duration_limit_us", seconds * 1_000_000L)
            .put("origin", "first_encoded_frame_pts").put("timestamps_rescaled", false)
            .put("first_source_pts_us", window?.firstSourcePtsUs ?: JSONObject.NULL)
            .put("last_output_relative_pts_us", window?.lastRelativePtsUs ?: JSONObject.NULL)
            .put("written_frames", window?.writtenFrames ?: 0)
            .put("discarded_tail_frames", window?.discardedTailFrames ?: 0)
            .put("limit_reached", window?.limitReached == true)
            .put("note", "Tail frames at/after the requested duration are intentionally excluded; all output is drained to EOS.")
        recordingStartedNs?.let { start ->
            encoder?.progress?.firstFrameAtNs?.let { first ->
                windowJson.put("first_frame_arrival_latency_seconds", (first - start) / 1e9)
            }
        }
        metadata.put("state", if (issues.isEmpty()) "FINALIZED" else "ERROR")
            .put("finished_at", Instant.now().toString()).put("video_file", video.name)
            .put("video_size_bytes", video.length()).put("sha256", if (video.exists()) storage.sha256(video) else JSONObject.NULL)
            .put("mp4", mp4 ?: JSONObject.NULL).put("sensor_timing", sensorJson).put("sensor_sampling", samplingJson)
            .put("capture_started_timing", startedJson)
            .put("encoded_timing", encodedStats.toJson()).put("capture_failures", failures.get())
            .put("buffer_losses", bufferLosses.get()).put("encoder_output_format", encoded?.outputFormat)
            .put("eos_received", encoded?.eos == true).put("muxer_finalized", encoded?.muxerFinalized == true)
            .put("control_elapsed_seconds", elapsed).put("recording_window", windowJson)
            .put("thermal_status_after", power.currentThermalStatus)
            .put("battery_temperature_c_after", batteryTemperatureC() ?: JSONObject.NULL)
            .put("available_bytes_after_recording", storage.availableBytes())
            .put("sensor_csv", sensorCsv.name).put("started_csv", "$base.started.csv").put("pts_csv", "$base.pts.csv")
            .put("assessment", assessment).put("errors", jsonArray(issues))
        val file = storage.json("$base.json", metadata)
        return ProbeOutcome(
            (if (assessment == "INCONCLUSIVE_BATCHED_SENSOR_METADATA")
                "動画の時刻は240 fps相当。センサー通知が間引かれているため、個々の撮影間隔は判定保留です。\n"
                else "$assessment\n") +
                "センサー通知: ${sensorStats.fps?.let { "%.2f".format(it) } ?: "不明"} 回/秒 / ${sensorStats.count}件\n" +
                "通知間隔: ${sampling.frameStep ?: "不明"} フレーム / フレーム番号からの推定: ${sampling.fpsFromFrameNumbers?.let { "%.2f".format(it) } ?: "不明"} fps\n" +
                "動画: ${encodedStats.fps?.let { "%.2f".format(it) } ?: "不明"} fps / ${encodedStats.count}フレーム\n" +
                "動画: ${video.name}\nレポート: ${file.name}\n" + issues.joinToString("\n"), file
        )
    }
}
