package com.example.myapplication.probe

import android.media.MediaCodec
import android.media.MediaFormat
import android.media.MediaMuxer
import android.os.SystemClock
import android.view.Surface
import java.io.File
import java.util.concurrent.CompletableFuture
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicLong

data class EncoderResult(
    val ptsNs: List<Long>, val outputFormat: String?, val eos: Boolean, val muxerFinalized: Boolean, val error: String?,
    val window: RecordingWindowSnapshot
)

/** Drain on a dedicated thread so camera callbacks never block on MP4 IO. */
data class EncoderProgress(val firstFrameAtNs: Long? = null, val lastOutputAtNs: Long? = null,
    val elapsedSeconds: Double = 0.0, val limitReached: Boolean = false)

class AvcEncoder(option: EncoderOption, file: File, orientation: Int, durationUs: Long) {
    private val window = RecordingWindow(durationUs)
    @Volatile var progress = EncoderProgress()
        private set
    private val codec = MediaCodec.createByCodecName(option.name)
    private var muxer: MediaMuxer? = null
    val surface: Surface
    val completed = CompletableFuture<EncoderResult>()
    private val endDeadlineNs = AtomicLong(Long.MAX_VALUE)

    init {
        var createdSurface: Surface? = null
        try {
            codec.configure(encoderFormat(option.bitrate), null, null, MediaCodec.CONFIGURE_FLAG_ENCODE)
            surface = codec.createInputSurface().also { createdSurface = it }
            muxer = MediaMuxer(file.absolutePath, MediaMuxer.OutputFormat.MUXER_OUTPUT_MPEG_4).apply {
                setOrientationHint(orientation)
            }
            codec.start()
        } catch (e: Exception) {
            runCatching { codec.release() }
            runCatching { muxer?.release() }
            runCatching { createdSurface?.release() }
            throw e
        }
        Thread({ drain() }, "avc-drain").start()
    }

    /** Call only after the camera has released its output surface. */
    fun finish(): EncoderResult {
        if (!completed.isDone) {
            endDeadlineNs.set(System.nanoTime() + TimeUnit.SECONDS.toNanos(8))
            runCatching { codec.signalEndOfInputStream() }
                .onFailure { endDeadlineNs.set(System.nanoTime()) }
        }
        return completed.get(12, TimeUnit.SECONDS)
    }

    private fun drain() {
        val info = MediaCodec.BufferInfo()
        val timestamps = mutableListOf<Long>()
        var formatText: String? = null
        var track = -1
        var muxerStarted = false
        var finalized = false
        var eos = false
        var error: String? = null
        try {
            while (!eos) {
                check(System.nanoTime() < endDeadlineNs.get()) { "エンコーダEOS待機がタイムアウトしました" }
                when (val index = codec.dequeueOutputBuffer(info, 10_000)) {
                    MediaCodec.INFO_OUTPUT_FORMAT_CHANGED -> {
                        check(!muxerStarted) { "エンコーダ出力フォーマットが複数回変更されました" }
                        val format: MediaFormat = codec.outputFormat
                        formatText = format.toString()
                        track = checkNotNull(muxer).addTrack(format)
                        checkNotNull(muxer).start()
                        muxerStarted = true
                    }
                    MediaCodec.INFO_TRY_AGAIN_LATER -> Unit
                    else -> if (index >= 0) {
                        try {
                            if (info.flags and MediaCodec.BUFFER_FLAG_CODEC_CONFIG != 0) info.size = 0
                            if (info.size > 0) {
                                check(muxerStarted) { "映像出力がフォーマット通知より前に到着しました" }
                                val data = checkNotNull(codec.getOutputBuffer(index))
                                data.position(info.offset)
                                data.limit(info.offset + info.size)
                                val keep = window.accept(info.presentationTimeUs)
                                val selection = window.snapshot()
                                val relativePts = checkNotNull(selection.lastRelativePtsUs)
                                if (keep) {
                                    // Only subtract the origin; preserve actual inter-frame intervals.
                                    info.presentationTimeUs = relativePts
                                    checkNotNull(muxer).writeSampleData(track, data, info)
                                    timestamps.add(relativePts * 1000)
                                }
                                // Continue draining discarded tail frames until EOS so camera teardown is safe.
                                val now = SystemClock.elapsedRealtimeNanos()
                                progress = EncoderProgress(progress.firstFrameAtNs ?: now, now,
                                    minOf(relativePts, selection.limitUs) / 1e6, selection.limitReached)
                            }
                            eos = info.flags and MediaCodec.BUFFER_FLAG_END_OF_STREAM != 0
                        } finally {
                            codec.releaseOutputBuffer(index, false)
                        }
                    }
                }
            }
        } catch (e: Exception) {
            error = e.toString()
        } finally {
            runCatching { codec.stop() }.onFailure { if (error == null) error = it.toString() }
            runCatching { codec.release() }.onFailure { if (error == null) error = it.toString() }
            if (muxerStarted) {
                runCatching { muxer?.stop(); finalized = true }.onFailure { if (error == null) error = it.toString() }
            }
            runCatching { muxer?.release() }.onFailure { if (error == null) error = it.toString() }
            completed.complete(EncoderResult(timestamps.toList(), formatText, eos, finalized, error, window.snapshot()))
        }
    }
}
