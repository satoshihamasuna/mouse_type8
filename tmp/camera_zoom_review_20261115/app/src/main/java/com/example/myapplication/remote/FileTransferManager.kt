package com.example.myapplication.remote

import okhttp3.MediaType.Companion.toMediaType
import okhttp3.Request
import okhttp3.RequestBody
import okhttp3.RequestBody.Companion.toRequestBody
import okio.BufferedSink
import org.json.JSONObject
import java.util.concurrent.TimeUnit

class FileTransferManager(
    private val network: NetworkClient, private val base: String, private val token: String,
    private val storage: StorageManager
) {
    private val client = network.http.newBuilder().readTimeout(120, TimeUnit.SECONDS)
        .callTimeout(180, TimeUnit.SECONDS).build()

    private fun execute(request: Request): JSONObject = client.newCall(request).execute().use { response ->
        val text = response.body.string()
        check(response.isSuccessful) { "HTTP_${response.code}: $text" }
        JSONObject(text)
    }

    fun transfer(entry: PendingVideo, onProgress: (Long, Long) -> Unit): Boolean {
        val id = entry.identity.id
        val manifest = Request.Builder().url("$base/device/manifests/$id")
            .header("Authorization", "Bearer $token")
            .post(storage.envelope(entry).toString().toRequestBody("application/json; charset=utf-8".toMediaType())).build()
        var receipt = execute(manifest)
        if (!receipt.optBoolean("committed")) {
            val body = object : RequestBody() {
                override fun contentType() = "video/mp4".toMediaType()
                override fun contentLength() = entry.identity.size
                override fun writeTo(sink: BufferedSink) {
                    var sent = 0L
                    var last = 0L
                    entry.video.inputStream().use { input ->
                        val buffer = ByteArray(256 * 1024)
                        while (true) {
                            val count = input.read(buffer)
                            if (count < 0) break
                            sink.write(buffer, 0, count)
                            sent += count
                            val now = System.nanoTime()
                            if (now - last > 250_000_000 || sent == entry.identity.size) {
                                onProgress(sent, entry.identity.size); last = now
                            }
                        }
                    }
                }
            }
            receipt = execute(Request.Builder().url("$base/device/files/$id")
                .header("Authorization", "Bearer $token").put(body).build())
        }
        return storage.acknowledge(entry, receipt)
    }

    fun cancel() { client.dispatcher.cancelAll() }
}
