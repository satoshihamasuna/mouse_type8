package com.example.myapplication.probe

import android.app.Application
import android.os.Handler
import android.os.Looper
import com.example.myapplication.remote.CameraOwnership
import com.example.myapplication.remote.RemoteState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import androidx.lifecycle.AndroidViewModel
import java.util.concurrent.Executors

data class ProbeUiState(
    val busy: Boolean = false,
    val recordingTask: Boolean = false,
    val status: String = "カメラ権限を許可して「機能を調査」を押してください",
    val elapsedSeconds: Double = 0.0,
    val freeBytes: Long = 0,
    val report: CapabilityReport? = null,
    val capabilityPath: String = "",
    val result: String = "",
    val metadataPath: String = ""
)

class ProbeViewModel(application: Application) : AndroidViewModel(application) {
    var state by mutableStateOf(ProbeUiState())
        private set
    private val main = Handler(Looper.getMainLooper())
    private val worker = Executors.newSingleThreadExecutor()
    private val storage = ProbeStorage(application)
    private val controller = CameraController(application, storage)

    fun inspect() {
        if (state.busy || RemoteState.state.enabled) return
        state = state.copy(busy = true, status = "対応機能を調査中")
        worker.execute {
            runCatching {
                val report = DeviceCapabilityManager(getApplication()).inspect()
                val path = storage.json("capabilities.json", report.json).absolutePath
                val free = storage.availableBytes()
                main.post {
                    state = state.copy(busy = false, status = "調査完了・録画は未検証", report = report,
                        capabilityPath = path, freeBytes = free)
                }
            }.onFailure { e ->
                main.post { state = state.copy(busy = false, status = "調査失敗: $e") }
            }
        }
    }

    fun start(cameraId: String?, encoderName: String?, seconds: Int, allowUndeclared: Boolean) {
        if (state.busy || RemoteState.state.enabled) return
        val report = state.report ?: return
        val camera = report.cameras.firstOrNull { it.id == cameraId } ?: return
        val encoder = report.encoders.firstOrNull { it.name == encoderName } ?: return
        if (!camera.targetSupported || !encoder.surfaceInput) return
        if (!allowUndeclared && (!encoder.sizeAndRateSupported || !encoder.formatSupported)) return
        if (!CameraOwnership.gate.acquire("local")) return
        controller.prepare()
        state = state.copy(busy = true, recordingTask = true, status = "試験を開始します",
            elapsedSeconds = 0.0, result = "", metadataPath = "")
        worker.execute {
            runCatching {
                controller.record(report, camera, encoder, seconds) { status, elapsed ->
                    main.post { state = state.copy(status = status, elapsedSeconds = elapsed) }
                }
            }.onSuccess { outcome ->
                val free = storage.availableBytes()
                main.post {
                    state = state.copy(busy = false, recordingTask = false, status = "試験終了",
                        result = outcome.summary, metadataPath = outcome.metadata.absolutePath, freeBytes = free)
                }
            }.onFailure { e ->
                main.post { state = state.copy(busy = false, recordingTask = false, status = "試験エラー",
                    result = "$e\n生成済みファイルは保持しています。probeフォルダを確認してください。") }
            }
            CameraOwnership.gate.release("local")
        }
    }

    fun stop() {
        controller.requestStop()
    }

    override fun onCleared() {
        controller.requestStop()
        worker.shutdown()
        super.onCleared()
    }
}
