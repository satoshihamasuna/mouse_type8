package com.example.myapplication.probe

import org.junit.Assert.*
import org.junit.Test

class DigitalZoomTest {
    @Test fun unsupportedDefaultRemainsCompatible() {
        assertEquals(1f, DigitalZoom.validate(1.0, 1f, 1f, false), 0f)
        assertTrue(DigitalZoom(1f).confirmed)
    }
    @Test fun rejectsInvalidOrUnsupportedZoom() {
        for (value in listOf(Double.NaN, Double.POSITIVE_INFINITY, .5, 8.0)) {
            assertThrows(IllegalArgumentException::class.java) { DigitalZoom.validate(value, 1f, 7f, true) }
        }
        assertThrows(IllegalArgumentException::class.java) { DigitalZoom.validate(2.0, 1f, 7f, false) }
    }
    @Test fun ignoredRequestNeverConfirms() {
        val zoom = DigitalZoom(2f)
        repeat(20) { zoom.observe(1f) }
        zoom.observe(null)
        assertFalse(zoom.confirmed)
    }
    @Test fun needsThreeConsecutiveMatchingResults() {
        val zoom = DigitalZoom(2f)
        zoom.observe(2f); zoom.observe(2f)
        assertFalse(zoom.confirmed)
        zoom.observe(null)
        zoom.observe(2f); zoom.observe(2f)
        assertFalse(zoom.confirmed)
        zoom.observe(2f)
        assertTrue(zoom.confirmed)
    }
    @Test fun zoomLossCannotRecoverSilently() {
        val zoom = DigitalZoom(2f)
        repeat(3) { zoom.observe(2f) }
        zoom.observe(1f)
        assertFalse(zoom.confirmed)
        assertEquals("ZOOM_RATIO_LOST", zoom.error)
        repeat(5) { zoom.observe(2f) }
        assertFalse(zoom.confirmed)
    }
}
