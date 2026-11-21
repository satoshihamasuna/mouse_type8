package com.example.myapplication.remote

import okhttp3.*
import org.json.JSONObject
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean

class NetworkClient(
    private val base: String, private val token: String,
    private val onConnection: (Boolean, String) -> Unit,
    private val onCommand: (JSONObject) -> Unit,
    private val hello: () -> JSONObject
) {
    val http = OkHttpClient.Builder().connectTimeout(5, TimeUnit.SECONDS)
        .readTimeout(15, TimeUnit.SECONDS).writeTimeout(60, TimeUnit.SECONDS)
        .pingInterval(10, TimeUnit.SECONDS).build()
    private val connecting = AtomicBoolean(false)
    @Volatile private var stopped = false
    @Volatile private var socket: WebSocket? = null
    @Volatile var connected = false
        private set

    fun connect() {
        if (stopped || connected || !connecting.compareAndSet(false, true)) return
        val request = Request.Builder().url(base.replaceFirst("http", "ws") + "/ws/device")
            .header("Authorization", "Bearer $token").build()
        socket = http.newWebSocket(request, object : WebSocketListener() {
            override fun onOpen(webSocket: WebSocket, response: Response) {
                connecting.set(false)
                if (stopped) { webSocket.close(1000, "stopped"); return }
                connected = true
                webSocket.send(hello().toString())
                onConnection(true, "PCに接続しました")
            }
            override fun onMessage(webSocket: WebSocket, text: String) {
                runCatching { onCommand(JSONObject(text)) }
                    .onFailure { onConnection(connected, "受信エラー: $it") }
            }
            override fun onFailure(webSocket: WebSocket, t: Throwable, response: Response?) {
                connecting.set(false); connected = false
                if (!stopped) onConnection(false, "PC接続待ち・再接続します: ${t.message}")
            }
            override fun onClosed(webSocket: WebSocket, code: Int, reason: String) {
                connecting.set(false); connected = false
                if (!stopped) onConnection(false, "PC接続が切れました。再接続します")
            }
            override fun onClosing(webSocket: WebSocket, code: Int, reason: String) { webSocket.close(code, reason) }
        })
    }

    fun send(value: JSONObject) { if (connected) socket?.send(value.toString()) }
    fun close() {
        stopped = true
        connected = false
        socket?.close(1000, "user disconnected")
        http.dispatcher.cancelAll()
        http.connectionPool.evictAll()
        http.dispatcher.executorService.shutdown()
    }
}
