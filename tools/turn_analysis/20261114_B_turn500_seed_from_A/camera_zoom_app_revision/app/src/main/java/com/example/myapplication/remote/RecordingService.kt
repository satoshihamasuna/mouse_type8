package com.example.myapplication.remote

import android.Manifest
import android.app.*
import android.hardware.camera2.CameraManager
import android.hardware.camera2.CameraCharacteristics
import android.content.Intent
import android.content.pm.PackageManager
import android.content.pm.ServiceInfo
import android.os.IBinder
import androidx.core.content.ContextCompat
import com.example.myapplication.MainActivity
import com.example.myapplication.probe.*
import org.json.JSONObject
import java.net.URI
import java.security.MessageDigest
import java.util.UUID
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit

/** Single owner of remote capture and transfer; started only by a visible Activity. */
class RecordingService : Service() {
    private val control = Executors.newSingleThreadScheduledExecutor()
    private val recorderWorker = Executors.newSingleThreadExecutor()
    private val transferWorker = Executors.newSingleThreadExecutor()
    private lateinit var storage: ProbeStorage
    private lateinit var files: StorageManager
    private lateinit var ledger: CommandLedger
    private lateinit var controller: CameraController
    private lateinit var network: NetworkClient
    private lateinit var transfer: FileTransferManager
    private var report: CapabilityReport? = null
    @Volatile private var snapshot = RemoteSnapshot()
    @Volatile private var shuttingDown = false
    private var initialized = false
    @Volatile private var destroyed = false
    @Volatile private var exposureSteps = 0
    @Volatile private var torchEnabled = false
    @Volatile private var focusLockEnabled = false
    @Volatile private var zoomRatio = 1f
    private var lastTransferAttempt = 0L

    override fun onBind(intent: Intent?): IBinder? = null

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        if (intent?.action == ACTION_STOP) {
            post {
                shuttingDown = true
                if (::controller.isInitialized) controller.requestStop()
                if (::transfer.isInitialized) transfer.cancel()
                if (snapshot.phase in setOf("IDLE", "ERROR")) stopSelf()
                else change { it.copy(message = "停止処理中です。未転送動画は保持します") }
            }
            return START_NOT_STICKY
        }
        if (initialized) return START_NOT_STICKY
        initialized = true
        try {
            check(ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED)
            val base = intent?.getStringExtra("base_url") ?: "http://127.0.0.1:8765"
            val token = intent?.getStringExtra("token") ?: ""
            val uri = URI(base)
            require(uri.scheme in setOf("http", "https") && uri.host != null && uri.rawUserInfo == null && uri.rawQuery == null)
            require(token.matches(Regex("[0-9a-f]{32}"))) { "PCの接続コードを入力してください" }
            val manager = getSystemService(NotificationManager::class.java)
            manager.createNotificationChannel(NotificationChannel("remote", "PCカメラ接続", NotificationManager.IMPORTANCE_LOW))
            val open = PendingIntent.getActivity(this, 0, Intent(this, MainActivity::class.java),
                PendingIntent.FLAG_IMMUTABLE or PendingIntent.FLAG_UPDATE_CURRENT)
            val stop = PendingIntent.getService(this, 1, Intent(this, RecordingService::class.java).setAction(ACTION_STOP),
                PendingIntent.FLAG_IMMUTABLE or PendingIntent.FLAG_UPDATE_CURRENT)
            val notification = Notification.Builder(this, "remote").setSmallIcon(android.R.drawable.presence_video_online)
                .setContentTitle("Pixel Camera: PC遠隔操作").setContentText("USB接続を待機しています")
                .setContentIntent(open).addAction(Notification.Action.Builder(null, "接続を停止", stop).build())
                .setOngoing(true).build()
            startForeground(1001, notification, ServiceInfo.FOREGROUND_SERVICE_TYPE_CAMERA)
            val preferences = getSharedPreferences("remote", MODE_PRIVATE)
            preferences.edit().putString("base_url", base).putString("token", token).apply()
            val deviceId = preferences.getString("device_id", null) ?: UUID.randomUUID().toString().also {
                preferences.edit().putString("device_id", it).apply()
            }
            change { it.copy(enabled = true, phase = "INITIALIZING", message = "PC接続を準備しています") }
            post {
                runCatching {
                    storage = ProbeStorage(this)
                    files = StorageManager(storage)
                    controller = CameraController(this, storage)
                    ledger = CommandLedger(storage)
                    val recovery = files.recoverInterrupted()
                    report = DeviceCapabilityManager(this).inspect()
                    network = NetworkClient(base, token, { connected, message ->
                        post {
                            change { it.copy(connected = connected, message = message) }
                            if (connected) files.deletedReceipts().forEach { sendDeleted(it) }
                        }
                    }, { request -> post { command(request) } }, {
                        JSONObject().put("type", "hello").put("protocol_version", 1).put("device_id", deviceId)
                            .put("status", statusJson())
                    })
                    transfer = FileTransferManager(network, base, token, files)
                    change { it.copy(phase = "IDLE", message = recovery ?: "PC接続を待っています", freeBytes = storage.availableBytes()) }
                    control.scheduleWithFixedDelay({ runCatching { tick() }.onFailure {
                        change { value -> value.copy(message = "接続処理: ${it.message}") }
                    } }, 0, 2, TimeUnit.SECONDS)
                }.onFailure { error ->
                    change { it.copy(phase = "ERROR", lastError = error.toString(), message = "準備失敗: $error") }
                    stopSelf()
                }
            }
        } catch (error: Exception) {
            change { it.copy(phase = "ERROR", lastError = error.toString(), message = "接続開始エラー: $error") }
            stopSelf()
        }
        return START_NOT_STICKY
    }

    private fun post(block: () -> Unit) { if (!control.isShutdown) runCatching { control.execute(block) } }

    @Synchronized private fun change(block: (RemoteSnapshot) -> RemoteSnapshot) {
        if (destroyed) return
        snapshot = block(snapshot)
        RemoteState.publish(snapshot)
        if (::network.isInitialized) network.send(JSONObject().put("type", "status").put("status", statusJson()))
    }

    private fun statusJson(): JSONObject {
        val value = snapshot
        return JSONObject().put("phase", value.phase).put("elapsed_seconds", value.elapsed)
            .put("available_bytes", value.freeBytes).put("pending_count", value.pending)
            .put("recording_id", value.recordingId).put("owner", value.owner)
            .put("message", value.message).put("last_error", value.lastError.takeIf { it.isNotEmpty() })
            .put("transfer_percent", value.transferPercent).put("exposure_compensation_steps", exposureSteps).put("torch", torchEnabled)
            .put("focus_lock", focusLockEnabled).put("zoom_ratio", zoomRatio).put("max_recording_seconds", 300)
    }

    private fun command(request: JSONObject) {
        if (request.optString("type") != "command") return
        val id = request.optString("command_id")
        val owner = request.optString("client_id")
        val operation = request.optString("command")
        val payload = request.optJSONObject("payload") ?: JSONObject()
        val fingerprint = MessageDigest.getInstance("SHA-256")
            .digest("$owner|$operation|$payload".toByteArray()).joinToString("") { "%02x".format(it.toInt() and 0xff) }
        try {
            require(id.matches(Regex("[A-Za-z0-9_-]{1,96}")) && owner.matches(Regex("[A-Za-z0-9_-]{1,96}")))
            check(request.optInt("protocol_version") == 1) { "UNSUPPORTED_PROTOCOL" }
            ledger.lookup(id, fingerprint)?.let { network.send(it); return }
            check(!shuttingDown) { "DISCONNECTING" }
            var startTask: (() -> Unit)? = null
            val result = when (operation) {
                "get_status" -> statusJson()
                "get_capabilities" -> checkNotNull(report).json
                "get_pending" -> JSONObject().put("files", files.summary())
                "configure" -> {
                    check(snapshot.phase in setOf("IDLE", "ERROR")) { "BUSY" }
                    require(payload.keys().asSequence().all { it in setOf("width", "height", "fps", "exposure_compensation_steps", "torch", "focus_lock", "zoom_ratio") }) { "UNSUPPORTED_SETTING" }
                    require(payload.optInt("width", 1280) == 1280 && payload.optInt("height", 720) == 720 && payload.optInt("fps", 240) == 240) { "UNSUPPORTED_MODE" }
                    val requestedZoom = if (payload.has("zoom_ratio")) {
                        val value = payload.get("zoom_ratio")
                        require(value is Number) { "ZOOM_RATIO_MUST_BE_NUMBER" }
                        val camera = checkNotNull(report).cameras.firstOrNull { it.targetSupported } ?: error("NO_720P240_CAMERA")
                        val characteristics = getSystemService(CameraManager::class.java).getCameraCharacteristics(camera.id)
                        val range = characteristics[CameraCharacteristics.CONTROL_ZOOM_RATIO_RANGE]
                        DigitalZoom.validate(value.toDouble(), range?.lower ?: 1f, range?.upper ?: 1f,
                            characteristics.availableCaptureRequestKeys.contains(android.hardware.camera2.CaptureRequest.CONTROL_ZOOM_RATIO))
                    } else zoomRatio
                    val requestedFocusLock = if (payload.has("focus_lock")) {
                        val value = payload.get("focus_lock")
                        require(value is Boolean) { "FOCUS_LOCK_MUST_BE_BOOLEAN" }
                        value
                    } else focusLockEnabled
                    val requestedTorch = if (payload.has("torch")) {
                        val value = payload.get("torch")
                        require(value is Boolean) { "TORCH_MUST_BE_BOOLEAN" }
                        if (value) {
                            val camera = checkNotNull(report).cameras.firstOrNull { it.targetSupported } ?: error("NO_720P240_CAMERA")
                            val characteristics = getSystemService(CameraManager::class.java).getCameraCharacteristics(camera.id)
                            check(characteristics[CameraCharacteristics.FLASH_INFO_AVAILABLE] == true) { "TORCH_UNAVAILABLE" }
                        }
                        value
                    } else torchEnabled
                    if (payload.has("exposure_compensation_steps")) {
                        val camera = checkNotNull(report).cameras.firstOrNull { it.targetSupported } ?: error("NO_720P240_CAMERA")
                        val characteristics = getSystemService(CameraManager::class.java).getCameraCharacteristics(camera.id)
                        val range = characteristics[CameraCharacteristics.CONTROL_AE_COMPENSATION_RANGE]
                        val requested = payload.get("exposure_compensation_steps")
                        require(requested is Number) { "EXPOSURE_COMPENSATION_MUST_BE_INTEGER" }
                        exposureSteps = ExposureCompensation.validate(requested.toDouble(),
                            (range?.lower ?: 0)..(range?.upper ?: 0))
                    }
                    torchEnabled = requestedTorch
                    focusLockEnabled = requestedFocusLock
                    zoomRatio = requestedZoom
                    JSONObject().put("width", 1280).put("height", 720).put("fps", 240).put("torch", torchEnabled)
                        .put("exposure_compensation_steps", exposureSteps).put("focus_lock", focusLockEnabled).put("zoom_ratio", zoomRatio)
                }
                "start_recording" -> {
                    check(snapshot.phase in setOf("IDLE", "ERROR")) { "BUSY" }
                    val seconds = payload.optInt("duration_seconds", 30)
                    require(seconds in 1..300) { "DURATION_MUST_BE_1_TO_300" }
                    val capabilities = checkNotNull(report)
                    val camera = capabilities.cameras.firstOrNull { it.targetSupported } ?: error("NO_720P240_CAMERA")
                    val encoder = capabilities.encoders.firstOrNull { it.hardware && it.surfaceInput && it.sizeAndRateSupported && it.formatSupported }
                        ?: error("NO_DECLARED_720P240_HARDWARE_ENCODER")
                    storage.requireSpace(seconds, encoder.bitrate)
                    val recordingId = storage.newBase()
                    startTask = { beginRecord(capabilities, camera, encoder, seconds, recordingId, owner) }
                    JSONObject().put("accepted", true).put("recording_id", recordingId)
                }
                "stop_recording" -> {
                    if (snapshot.phase in setOf("PREPARING", "RECORDING", "STOPPING", "FINALIZING")) {
                        check(snapshot.owner == owner) { "NOT_RECORDING_OWNER" }
                        startTask = { controller.requestStop(); change { it.copy(phase = "STOPPING", message = "停止要求を受け付けました") } }
                    }
                    JSONObject().put("accepted", true).put("file_finalized", false).put("recording_id", snapshot.recordingId)
                }
                else -> error("UNKNOWN_COMMAND")
            }
            val response = JSONObject().put("type", "response").put("command_id", id).put("ok", true).put("result", result)
            // Write acceptance before side effects. Replay never records a second movie.
            ledger.remember(id, fingerprint, response)
            startTask?.invoke()
            network.send(response)
        } catch (error: Exception) {
            network.send(JSONObject().put("type", "response").put("command_id", id).put("ok", false)
                .put("error", JSONObject().put("code", error.message ?: "COMMAND_FAILED").put("message", error.toString())))
        }
    }

    private fun beginRecord(capabilities: CapabilityReport, camera: CameraOption, encoder: EncoderOption, seconds: Int, id: String, owner: String) {
        check(CameraOwnership.gate.acquire("record:$id")) { "CAMERA_BUSY" }
        controller.prepare()
        val selectedExposureSteps = exposureSteps
        val selectedTorch = torchEnabled
        val selectedFocusLock = focusLockEnabled
        val selectedZoom = zoomRatio
        change { it.copy(phase = "PREPARING", owner = owner, recordingId = id, elapsed = 0.0, lastError = "", message = "録画を準備しています") }
        recorderWorker.execute {
            val outcome = runCatching {
                controller.record(capabilities, camera, encoder, seconds, recordingId = id, controllerId = owner, autoTransfer = true,
                    exposureCompensationSteps = selectedExposureSteps, torch = selectedTorch, focusLock = selectedFocusLock, zoomRatio = selectedZoom) { message, elapsed ->
                    change {
                        val phase = if (message.startsWith("停止")) "FINALIZING" else if (message == "録画中" && it.phase != "STOPPING") "RECORDING" else it.phase
                        it.copy(phase = phase, elapsed = elapsed, message = message)
                    }
                }.also { check(JSONObject(it.metadata.readText()).getString("state") == "FINALIZED") { it.summary } }
            }
            CameraOwnership.gate.release("record:$id")
            post {
                outcome.onSuccess {
                    network.send(JSONObject().put("type", "recording_finalized").put("recording_id", id))
                    change { value -> value.copy(phase = "IDLE", owner = "", message = "MP4を確定しました。転送を開始します", freeBytes = storage.availableBytes()) }
                }.onFailure { error ->
                    change { it.copy(phase = "ERROR", owner = "", lastError = error.toString(), message = "録画失敗・元ファイルを保持: $error") }
                }
                if (shuttingDown) stopSelf() else tick()
            }
        }
    }

    private fun sendDeleted(identity: TransferIdentity) {
        network.send(JSONObject().put("type", "source_deleted").put("recording_id", identity.id).put("sha256", identity.sha256))
    }

    private fun tick() {
        if (shuttingDown || !::network.isInitialized) return
        network.connect()
        if (snapshot.phase !in setOf("IDLE", "ERROR")) return
        val pending = files.pending()
        change { it.copy(pending = pending.size, freeBytes = storage.availableBytes()) }
        if (!network.connected || pending.isEmpty() || System.nanoTime() - lastTransferAttempt < 5_000_000_000L) return
        val entry = pending.first()
        if (!CameraOwnership.gate.acquire("transfer")) return
        lastTransferAttempt = System.nanoTime()
        change { it.copy(phase = "TRANSFERRING", message = "PCへ動画を転送しています", transferPercent = 0) }
        transferWorker.execute {
            val result = runCatching {
                check(transfer.transfer(entry) { sent, total ->
                    change { it.copy(transferPercent = (sent * 100 / total).toInt()) }
                }) { "DELETE_FAILED_SOURCE_RETAINED" }
            }
            CameraOwnership.gate.release("transfer")
            post {
                result.onSuccess {
                    sendDeleted(entry.identity)
                    change { it.copy(phase = "IDLE", message = "PC保存・照合完了。端末の元動画を削除しました",
                        transferPercent = 100, pending = files.pending().size, freeBytes = storage.availableBytes()) }
                }.onFailure { error ->
                    change { it.copy(phase = "IDLE", message = "転送を再試行します。元動画は保持: ${error.message}") }
                }
                if (shuttingDown) stopSelf()
            }
        }
    }

    override fun onDestroy() {
        destroyed = true
        shuttingDown = true
        if (::controller.isInitialized) controller.requestStop()
        if (::transfer.isInitialized) transfer.cancel()
        if (::network.isInitialized) network.close()
        control.shutdownNow()
        recorderWorker.shutdown()
        transferWorker.shutdown()
        RemoteState.publish(snapshot.copy(enabled = false, connected = false,
            message = if (snapshot.lastError.isNotEmpty()) snapshot.message else "PC接続は停止中"))
        super.onDestroy()
    }

    companion object { const val ACTION_STOP = "com.example.myapplication.STOP_REMOTE" }
}
