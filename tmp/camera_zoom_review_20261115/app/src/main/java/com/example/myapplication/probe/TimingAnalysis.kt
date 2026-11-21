package com.example.myapplication.probe

import kotlin.math.abs
import kotlin.math.roundToLong

/** Uses actual timestamps, never the nominal FPS stored in the MP4 header. */
data class TimingStats(
    val count: Int,
    val spanSeconds: Double,
    val fps: Double?,
    val medianIntervalMs: Double?,
    val minIntervalMs: Double?,
    val maxIntervalMs: Double?,
    val longGaps: Int,
    val estimatedMissingFrames: Long,
    val nonIncreasing: Int
)

data class SensorSampling(
    val mode: String,
    val frameStep: Long?,
    val fpsFromFrameNumbers: Double?,
    val estimatedCoveredFrames: Long?
)

object TimingAnalysis {
    /** Classify observed callback sampling; never invent missing sensor timestamps. */
    fun sensorSampling(frameNumbers: List<Long>, timestampsNs: List<Long>, burstSize: Int): SensorSampling {
        if (frameNumbers.size < 2 || frameNumbers.size != timestampsNs.size) {
            return SensorSampling("INSUFFICIENT", null, null, null)
        }
        val steps = frameNumbers.zipWithNext { a, b -> b - a }
        val step = steps.first().takeIf { it > 0 && steps.all { value -> value == it } }
        val spanNs = timestampsNs.last() - timestampsNs.first()
        val fps = if (spanNs > 0) (frameNumbers.last() - frameNumbers.first()) * 1e9 / spanNs else null
        val mode = when {
            step == 1L -> "EVERY_FRAME"
            burstSize > 1 && step == burstSize.toLong() -> "ONE_CALLBACK_PER_BATCH"
            else -> "IRREGULAR"
        }
        return SensorSampling(mode, step, fps,
            if (step != null) frameNumbers.last() - frameNumbers.first() + step else null)
    }
    fun analyze(timestampsNs: List<Long>, targetFps: Int = 240): TimingStats {
        require(targetFps > 0)
        val deltas = timestampsNs.zipWithNext { a, b -> b - a }
        val positive = deltas.filter { it > 0 }.sorted()
        val expected = 1_000_000_000.0 / targetFps
        val span = if (timestampsNs.size > 1) (timestampsNs.last() - timestampsNs.first()) / 1e9 else 0.0
        val median = if (positive.isEmpty()) null else {
            val mid = positive.size / 2
            if (positive.size % 2 == 0) (positive[mid - 1] / 2.0 + positive[mid] / 2.0) else positive[mid].toDouble()
        }
        return TimingStats(
            timestampsNs.size, span,
            if (span > 0) (timestampsNs.size - 1) / span else null,
            median?.div(1e6), positive.firstOrNull()?.div(1e6), positive.lastOrNull()?.div(1e6),
            positive.count { it > expected * 1.5 },
            positive.sumOf { (it / expected).roundToLong().minus(1).coerceAtLeast(0) },
            deltas.count { it <= 0 }
        )
    }

    fun assessment(sensor: TimingStats, encoded: TimingStats, captureFailures: Int, bufferLosses: Int, wallElapsedSeconds: Double, sampling: SensorSampling? = null, discardedTailFrames: Int = 0): String {
        if (sensor.count < 2 || encoded.count < 2) return "INCONCLUSIVE_MISSING_TIMESTAMPS"
        if (captureFailures > 0 || bufferLosses > 0 || sensor.nonIncreasing > 0 || encoded.nonIncreasing > 0 ||
            encoded.longGaps > 0) return "TIMING_GAPS_DETECTED"
        if (sampling != null && sampling.mode !in listOf("EVERY_FRAME", "ONE_CALLBACK_PER_BATCH")) {
            return "INCONCLUSIVE_METADATA_SAMPLING"
        }
        val batched = sampling?.mode == "ONE_CALLBACK_PER_BATCH"
        if (!batched && sensor.longGaps > 0) return "TIMING_GAPS_DETECTED"
        val sensorRate = if (batched) sampling?.fpsFromFrameNumbers else sensor.fps
        if (abs((sensorRate ?: 0.0) - 240.0) > 4.8 || abs((encoded.fps ?: 0.0) - 240.0) > 4.8) {
            return "TARGET_RATE_NOT_OBSERVED"
        }
        val ratio = encoded.spanSeconds / sensor.spanSeconds
        val expectedFrames = (if (batched) sampling?.estimatedCoveredFrames ?: 0 else sensor.count.toLong()) - discardedTailFrames
        val countTolerance = if (batched) sampling?.frameStep ?: 3 else 3L
        if (ratio !in 0.95..1.05 || abs(expectedFrames - encoded.count) > countTolerance) return "INCONCLUSIVE_TIMELINES_DIFFER"
        if (wallElapsedSeconds < 2.0) return "INCONCLUSIVE_SHORT_RECORDING"
        val tolerance = maxOf(0.5, wallElapsedSeconds * 0.1)
        if (abs(sensor.spanSeconds - wallElapsedSeconds) > tolerance || abs(encoded.spanSeconds - wallElapsedSeconds) > tolerance) {
            return "WALL_CLOCK_DURATION_MISMATCH"
        }
        // Sparse metadata cannot validate the timing of each individual sensor frame.
        if (batched) return "INCONCLUSIVE_BATCHED_SENSOR_METADATA"
        return "TIMING_CONSISTENT_REQUIRES_VISUAL_VALIDATION"
    }
}
