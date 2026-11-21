package com.example.myapplication.probe

import android.content.Context
import android.media.MediaExtractor
import android.media.MediaFormat
import android.os.StatFs
import android.util.AtomicFile
import org.json.JSONObject
import java.io.File
import java.io.FileInputStream
import java.security.MessageDigest
import java.time.Instant
import java.time.ZoneId
import java.time.format.DateTimeFormatter
import java.util.UUID

class ProbeStorage(context: Context) {
    // Internal app storage. Remote recordings are deleted only after a verified PC receipt.
    val directory = File(context.filesDir, "probe").apply { check(isDirectory || mkdirs()) }
    fun availableBytes(): Long = StatFs(directory.absolutePath).availableBytes
    fun requireSpace(seconds: Int, bitrate: Int) {
        val estimate = bitrate.toLong() / 8 * (seconds + 5) * 12 / 10 + 128L * 1024 * 1024
        check(availableBytes() >= estimate) { "ストレージ不足: 必要な空き容量は約 ${estimate / 1024 / 1024} MiB" }
    }
    fun newBase(): String = DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss").withZone(ZoneId.systemDefault())
        .format(Instant.now()) + "_240fps_" + UUID.randomUUID().toString().take(8)

    fun json(name: String, value: JSONObject): File = File(directory, name).also { file ->
        val atomic = AtomicFile(file)
        val stream = atomic.startWrite()
        try {
            stream.write(value.toString(2).toByteArray(Charsets.UTF_8))
            atomic.finishWrite(stream)
        } catch (e: Exception) {
            atomic.failWrite(stream)
            throw e
        }
    }

    fun sha256(file: File): String {
        val digest = MessageDigest.getInstance("SHA-256")
        FileInputStream(file).use { input ->
            val buffer = ByteArray(1024 * 1024)
            while (true) {
                val count = input.read(buffer)
                if (count < 0) break
                digest.update(buffer, 0, count)
            }
        }
        return digest.digest().joinToString("") { "%02x".format(it.toInt() and 0xff) }
    }

    fun inspectMp4(file: File): JSONObject {
        val extractor = MediaExtractor()
        try {
            extractor.setDataSource(file.absolutePath)
            val index = (0 until extractor.trackCount).firstOrNull {
                extractor.getTrackFormat(it).getString(MediaFormat.KEY_MIME)?.startsWith("video/") == true
            } ?: error("MP4に映像トラックがありません")
            val format = extractor.getTrackFormat(index)
            extractor.selectTrack(index)
            val pts = mutableListOf<Long>()
            while (extractor.sampleTime >= 0) {
                pts.add(extractor.sampleTime * 1000)
                if (!extractor.advance()) break
            }
            return JSONObject().put("format", format.toString())
                .put("width", format.getInteger(MediaFormat.KEY_WIDTH))
                .put("height", format.getInteger(MediaFormat.KEY_HEIGHT))
                .put("sample_timing", TimingAnalysis.analyze(pts).toJson())
                .put("sample_count", pts.size)
                .put("duration_us", if (format.containsKey(MediaFormat.KEY_DURATION)) format.getLong(MediaFormat.KEY_DURATION) else JSONObject.NULL)
                .put("note", "MediaExtractor counts encoded samples. Independent decoding and a timed moving scene remain necessary.")
        } finally {
            extractor.release()
        }
    }
}

fun TimingStats.toJson(): JSONObject = JSONObject().put("count", count).put("span_seconds", spanSeconds)
    .put("fps_from_timestamps", fps ?: JSONObject.NULL).put("median_interval_ms", medianIntervalMs ?: JSONObject.NULL)
    .put("min_interval_ms", minIntervalMs ?: JSONObject.NULL).put("max_interval_ms", maxIntervalMs ?: JSONObject.NULL)
    .put("gaps_over_1_5_frames", longGaps).put("estimated_missing_frames", estimatedMissingFrames)
    .put("non_increasing_timestamps", nonIncreasing)
