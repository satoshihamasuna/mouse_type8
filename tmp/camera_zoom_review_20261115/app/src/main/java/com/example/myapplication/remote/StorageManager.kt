package com.example.myapplication.remote

import com.example.myapplication.probe.ProbeStorage
import org.json.JSONArray
import org.json.JSONObject
import java.io.File

data class PendingVideo(val identity: TransferIdentity, val video: File, val metadata: JSONObject)

/** Legacy Phase 1 files are excluded. Only recordings explicitly tagged transfer_protocol=1 are moved. */
class StorageManager(private val storage: ProbeStorage) {
    private val validId = Regex("[A-Za-z0-9_-]{1,96}")

    private fun read(file: File): JSONObject = JSONObject(file.readText())
    fun pending(): List<PendingVideo> = storage.directory.listFiles().orEmpty()
        .filter { it.extension == "json" }
        .mapNotNull { file ->
            runCatching {
                val value = read(file)
                if (value.optInt("transfer_protocol") != 1 || value.optString("state") != "FINALIZED") return@runCatching null
                val id = value.getString("recording_id")
                require(validId.matches(id) && file.name == "$id.json")
                require(value.getString("video_file") == "$id.mp4")
                val video = File(storage.directory, "$id.mp4")
                if (!video.exists()) return@runCatching null
                val identity = TransferIdentity(id, value.getLong("video_size_bytes"), value.getString("sha256"))
                require(identity.size == video.length() && identity.sha256.matches(Regex("[0-9a-f]{64}")))
                PendingVideo(identity, video, value)
            }.getOrNull()
        }.sortedBy { it.identity.id }

    fun summary(): JSONArray = JSONArray().apply {
        pending().forEach { put(JSONObject().put("recording_id", it.identity.id).put("size", it.identity.size)) }
    }

    fun envelope(entry: PendingVideo): JSONObject {
        val sidecars = JSONObject()
        for ((key, suffix) in listOf("sensor_csv" to ".sensor.csv", "started_csv" to ".started.csv", "pts_csv" to ".pts.csv")) {
            val name = entry.metadata.getString(key)
            require(name == entry.identity.id + suffix)
            val file = File(storage.directory, name)
            require(file.length() <= 2L * 1024 * 1024 && file.isFile)
            sidecars.put(name, file.readText())
        }
        return JSONObject().put("metadata", entry.metadata).put("sidecars", sidecars)
    }

    fun acknowledge(entry: PendingVideo, receiptJson: JSONObject): Boolean {
        val receipt = TransferReceipt(receiptJson.getString("recording_id"), receiptJson.getLong("size"),
            receiptJson.getString("sha256"), receiptJson.opt("committed") == true)
        require(ReceiptPolicy.matches(entry.identity, receipt)) { "ACK_ID_SIZE_OR_HASH_MISMATCH" }
        val metadataFile = File(storage.directory, entry.identity.id + ".json")
        val latest = read(metadataFile)
        require(latest.getString("recording_id") == entry.identity.id && latest.getString("video_file") == entry.video.name)
        require(latest.getLong("video_size_bytes") == entry.identity.size && latest.getString("sha256") == entry.identity.sha256)
        require(entry.video.length() == entry.identity.size && storage.sha256(entry.video) == entry.identity.sha256) {
            "SOURCE_CHANGED_RETAINED"
        }
        // Persist the verified receipt before deletion. A crash here leaves a retryable source file.
        latest.put("transfer_status", "ACK_VERIFIED").put("pc_receipt", receiptJson)
        storage.json(metadataFile.name, latest)
        if (!entry.video.delete()) {
            latest.put("transfer_status", "DELETE_PENDING")
            storage.json(metadataFile.name, latest)
            return false
        }
        storage.json(metadataFile.name, latest.put("transfer_status", "SOURCE_DELETED"))
        return true
    }

    /** Reannounce durable deletion tombstones when an event/connection was lost. */
    fun deletedReceipts(): List<TransferIdentity> = storage.directory.listFiles().orEmpty()
        .filter { it.extension == "json" }.mapNotNull { file ->
            runCatching {
                val value = read(file)
                if (value.optInt("transfer_protocol") != 1 || value.optString("state") != "FINALIZED") return@runCatching null
                val id = value.getString("recording_id")
                require(validId.matches(id) && file.name == "$id.json" && value.getString("video_file") == "$id.mp4")
                if (File(storage.directory, "$id.mp4").exists()) return@runCatching null
                val identity = TransferIdentity(id, value.getLong("video_size_bytes"), value.getString("sha256"))
                val ack = value.optJSONObject("pc_receipt") ?: return@runCatching null
                require(ReceiptPolicy.matches(identity, TransferReceipt(ack.getString("recording_id"),
                    ack.getLong("size"), ack.getString("sha256"), ack.opt("committed") == true)))
                if (value.optString("transfer_status") != "SOURCE_DELETED") {
                    storage.json(file.name, value.put("transfer_status", "SOURCE_DELETED"))
                }
                identity
            }.getOrNull()
        }

    fun recoverInterrupted(): String? {
        var message: String? = null
        storage.directory.listFiles().orEmpty().filter { it.extension == "json" }.forEach { file ->
            runCatching {
                val value = read(file)
                if (value.optInt("transfer_protocol") == 1 && value.optString("state") == "PREPARING") {
                    value.put("state", "ERROR").put("recovery_note", "PROCESS_INTERRUPTED_FILES_RETAINED")
                    storage.json(file.name, value)
                    message = "中断した録画があります。元ファイルを保持しています: ${file.name}"
                }
            }
        }
        return message
    }
}

class CommandLedger(private val storage: ProbeStorage) {
    private val file = File(storage.directory, "remote_commands.json")
    private val entries = if (file.exists()) JSONObject(file.readText()) else JSONObject()

    fun lookup(id: String, fingerprint: String): JSONObject? {
        val value = entries.optJSONObject(id) ?: return null
        require(value.getString("fingerprint") == fingerprint) { "COMMAND_ID_CONFLICT" }
        return value.getJSONObject("response")
    }

    fun remember(id: String, fingerprint: String, response: JSONObject) {
        while (entries.length() >= 256) entries.remove(entries.keys().next())
        entries.put(id, JSONObject().put("fingerprint", fingerprint).put("response", response))
        storage.json(file.name, entries)
    }
}
