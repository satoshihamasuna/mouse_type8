package com.example.myapplication

import com.example.myapplication.probe.TimingAnalysis
import org.junit.Assert.*
import org.junit.Test

class TimingAnalysisTest {
    private fun times(fps: Int, count: Int): List<Long> = List(count) { it * 1_000_000_000L / fps }

    @Test fun actual240TimelineIsRecognizedWithoutReadingContainerFps() {
        val stats = TimingAnalysis.analyze(times(240, 2400))
        assertEquals(240.0, stats.fps!!, 0.001)
        assertEquals(0, stats.longGaps)
        assertEquals(0L, stats.estimatedMissingFrames)
        assertEquals("TIMING_CONSISTENT_REQUIRES_VISUAL_VALIDATION", TimingAnalysis.assessment(stats, stats, 0, 0, 10.0))
    }

    @Test fun thirtyFpsSlowMotionPtsCannotPassAsRealTime240() {
        val sensor = TimingAnalysis.analyze(times(240, 2400))
        val encoded = TimingAnalysis.analyze(times(30, 2400))
        assertNotEquals("TIMING_CONSISTENT_REQUIRES_VISUAL_VALIDATION", TimingAnalysis.assessment(sensor, encoded, 0, 0, 10.0))
        assertEquals(30.0, encoded.fps!!, 0.001)
    }

    @Test fun missingFramesAppearAsTimestampGaps() {
        val timestamps = times(240, 100).filterIndexed { index, _ -> index != 50 }
        val stats = TimingAnalysis.analyze(timestamps)
        assertEquals(1, stats.longGaps)
        assertEquals(1L, stats.estimatedMissingFrames)
    }

    @Test fun repeatedAndReversedTimestampsAreNotSilentlySortedAway() {
        val stats = TimingAnalysis.analyze(listOf(0L, 4_166_667L, 4_166_667L, 1L))
        assertEquals(2, stats.nonIncreasing)
    }

    @Test fun missingCallbacksAreInconclusive() {
        val absent = TimingAnalysis.analyze(emptyList())
        assertNull(absent.fps)
        assertEquals("INCONCLUSIVE_MISSING_TIMESTAMPS",
            TimingAnalysis.assessment(absent, TimingAnalysis.analyze(times(240, 2400)), 0, 0, 10.0))
    }

    @Test fun matchingFpsWithTooFewFramesForElapsedTimeCannotPass() {
        val stats = TimingAnalysis.analyze(times(240, 1200))
        assertEquals("WALL_CLOCK_DURATION_MISMATCH", TimingAnalysis.assessment(stats, stats, 0, 0, 10.0))
    }

    @Test fun oneCallbackPerEightFrameBatchIsInconclusiveRatherThanFrameLoss() {
        val frameIds = List(292) { 7L + it * 8 }
        val callbacks = frameIds.map { it * 1_000_000_000L / 240 }
        val sampling = TimingAnalysis.sensorSampling(frameIds, callbacks, 8)
        assertEquals("ONE_CALLBACK_PER_BATCH", sampling.mode)
        assertEquals(240.0, sampling.fpsFromFrameNumbers!!, 0.001)
        assertEquals("INCONCLUSIVE_BATCHED_SENSOR_METADATA",
            TimingAnalysis.assessment(TimingAnalysis.analyze(callbacks),
                TimingAnalysis.analyze(times(240, 2336)), 0, 0, 10.0, sampling))
    }

    @Test fun batchedMetadataDoesNotHideAnEncodedFrameGap() {
        val frameIds = List(300) { 7L + it * 8 }
        val callbacks = frameIds.map { it * 1_000_000_000L / 240 }
        val encoded = times(240, 2400).filterIndexed { index, _ -> index != 1000 }
        assertEquals("TIMING_GAPS_DETECTED",
            TimingAnalysis.assessment(TimingAnalysis.analyze(callbacks), TimingAnalysis.analyze(encoded),
                0, 0, 10.0, TimingAnalysis.sensorSampling(frameIds, callbacks, 8)))
    }

    @Test fun irregularMetadataIsNotReconstructedAsIndependentFrames() {
        val sampling = TimingAnalysis.sensorSampling(listOf(7L, 15L, 31L),
            listOf(0L, 33_333_333L, 100_000_000L), 8)
        assertEquals("IRREGULAR", sampling.mode)
    }

    @Test fun declaredTailSelectionIsExcludedFromSensorCountComparison() {
        val frameIds = List(307) { 7L + it * 8 }
        val callbacks = frameIds.map { it * 1_000_000_000L / 240 }
        val sampling = TimingAnalysis.sensorSampling(frameIds, callbacks, 8)
        assertEquals("INCONCLUSIVE_TIMELINES_DIFFER",
            TimingAnalysis.assessment(TimingAnalysis.analyze(callbacks),
                TimingAnalysis.analyze(times(240, 2400)), 0, 0, 10.3, sampling))
        assertEquals("INCONCLUSIVE_BATCHED_SENSOR_METADATA",
            TimingAnalysis.assessment(TimingAnalysis.analyze(callbacks),
                TimingAnalysis.analyze(times(240, 2400)), 0, 0, 10.3, sampling, 56))
    }

    @Test fun intentionalTailExclusionCannotHideAGapInsideSavedVideo() {
        val frameIds = List(307) { 7L + it * 8 }
        val callbacks = frameIds.map { it * 1_000_000_000L / 240 }
        val encoded = times(240, 2400).filterIndexed { index, _ -> index != 1000 }
        assertEquals("TIMING_GAPS_DETECTED",
            TimingAnalysis.assessment(TimingAnalysis.analyze(callbacks), TimingAnalysis.analyze(encoded),
                0, 0, 10.3, TimingAnalysis.sensorSampling(frameIds, callbacks, 8), 56))
    }

    @Test fun truncatedEncoderTimelineDoesNotPassEvenAt240Fps() {
        assertEquals("INCONCLUSIVE_TIMELINES_DIFFER",
            TimingAnalysis.assessment(TimingAnalysis.analyze(times(240, 2400)),
                TimingAnalysis.analyze(times(240, 1200)), 0, 0, 10.0))
    }
}
