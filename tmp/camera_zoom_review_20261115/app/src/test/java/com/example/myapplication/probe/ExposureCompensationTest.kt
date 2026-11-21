package com.example.myapplication.probe

import org.junit.Assert.assertEquals
import org.junit.Assert.assertThrows
import org.junit.Test

class ExposureCompensationTest {
    @Test fun acceptsDeviceRangeAndBothDirections() {
        for (value in listOf(-24, -6, 0, 6, 24)) {
            assertEquals(value, ExposureCompensation.validate(value.toDouble(), -24..24))
        }
    }
    @Test fun rejectsUnsupportedAndNonintegralValues() {
        for (value in listOf(-25.0, 25.0, 0.5, Double.NaN, Double.POSITIVE_INFINITY)) {
            assertThrows(IllegalArgumentException::class.java) { ExposureCompensation.validate(value, -24..24) }
        }
        assertThrows(IllegalArgumentException::class.java) { ExposureCompensation.validate(1.0, 0..0) }
    }
}
