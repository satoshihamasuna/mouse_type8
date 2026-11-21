package com.example.myapplication

import com.example.myapplication.probe.RecordingWindow
import org.junit.Assert.*
import org.junit.Test

class RecordingWindowTest {
    @Test fun firstFrameDefinesOriginRegardlessOfCameraStartupDelay() {
        val window = RecordingWindow(10_000_000)
        val origin = 987_654_321_000L
        val kept = (0..2420).map { origin + it * 1_000_000L / 240 }.filter { window.accept(it) }
        assertEquals(2400, kept.size)
        assertEquals(origin, kept.first())
        assertEquals(21, window.snapshot().discardedTailFrames)
        assertTrue(window.snapshot().limitReached)
    }

    @Test fun realCadenceIsNotForcedToNominalFrameCount() {
        val window = RecordingWindow(10_000_000)
        val kept = (0..2410).map { it * 1_000_000_000L / 239_770 }.filter { window.accept(it) }
        assertEquals(2398, kept.size)
        assertTrue(kept.last() < 10_000_000)
        assertEquals(13, window.snapshot().discardedTailFrames)
    }

    @Test fun earlyManualStopPreservesShorterPrefixAndActualGap() {
        val window = RecordingWindow(10_000_000)
        assertTrue(window.accept(500))
        assertTrue(window.accept(8_833))
        assertEquals(8_333L, window.snapshot().lastRelativePtsUs)
        assertFalse(window.snapshot().limitReached)
        assertEquals(0, window.snapshot().discardedTailFrames)
    }

    @Test(expected = IllegalStateException::class)
    fun duplicateTimestampInDiscardedTailStillFails() {
        val window = RecordingWindow(10)
        window.accept(100)
        window.accept(110)
        window.accept(110)
    }
}
