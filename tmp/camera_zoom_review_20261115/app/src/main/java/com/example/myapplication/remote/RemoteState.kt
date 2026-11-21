package com.example.myapplication.remote

import android.os.Handler
import android.os.Looper
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue

data class RemoteSnapshot(
    val enabled: Boolean = false, val connected: Boolean = false,
    val phase: String = "IDLE", val message: String = "PC接続は停止中",
    val elapsed: Double = 0.0, val freeBytes: Long = 0, val pending: Int = 0,
    val recordingId: String = "", val owner: String = "", val lastError: String = "",
    val transferPercent: Int = 0
)

object RemoteState {
    var state by mutableStateOf(RemoteSnapshot())
        private set
    private val main = Handler(Looper.getMainLooper())
    fun publish(value: RemoteSnapshot) { main.post { state = value } }
}

class OperationGate {
    private var owner: String? = null
    @Synchronized fun acquire(candidate: String): Boolean {
        if (owner != null) return false
        owner = candidate
        return true
    }
    @Synchronized fun release(candidate: String) {
        if (owner == candidate) owner = null
    }
}

object CameraOwnership { val gate = OperationGate() }

data class TransferIdentity(val id: String, val size: Long, val sha256: String)
data class TransferReceipt(val id: String, val size: Long, val sha256: String, val committed: Boolean)

object ReceiptPolicy {
    fun matches(expected: TransferIdentity, receipt: TransferReceipt): Boolean =
        receipt.committed && expected.size > 0 && expected.id == receipt.id && expected.size == receipt.size &&
            expected.sha256.matches(Regex("[0-9a-f]{64}")) && expected.sha256 == receipt.sha256
}
