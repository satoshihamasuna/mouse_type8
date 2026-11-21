package com.example.myapplication.probe

/** Request steps are integral and constrained by this device's camera characteristics. */
object ExposureCompensation {
    fun validate(value: Double, range: IntRange): Int {
        require(value.isFinite() && value == kotlin.math.floor(value)) { "EXPOSURE_COMPENSATION_MUST_BE_INTEGER" }
        require(value >= range.first && value <= range.last) { "EXPOSURE_COMPENSATION_OUT_OF_RANGE" }
        return value.toInt()
    }
}
