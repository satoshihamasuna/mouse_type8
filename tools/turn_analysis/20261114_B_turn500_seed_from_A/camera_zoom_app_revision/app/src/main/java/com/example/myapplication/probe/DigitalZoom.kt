package com.example.myapplication.probe

import kotlin.math.abs

/** Verify Camera2's returned zoom; an accepted request alone is not evidence of zoom. */
class DigitalZoom(private val requested: Float) {
    companion object {
        fun validate(value: Double, minimum: Float, maximum: Float, supported: Boolean): Float {
            require(value.isFinite() && value >= 1.0) { "INVALID_ZOOM_RATIO" }
            require(value == 1.0 || (supported && value >= minimum && value <= maximum)) {
                "ZOOM_RATIO_UNSUPPORTED"
            }
            return value.toFloat()
        }
    }

    @Volatile var confirmed = requested == 1f
        private set
    @Volatile var error: String? = null
        private set
    private var matching = 0

    @Synchronized fun observe(actual: Float?) {
        if (requested == 1f || error != null) return
        if (actual != null && actual.isFinite() && abs(actual-requested) <= .01f) {
            matching++
            if (matching >= 3) confirmed = true
        } else {
            matching = 0
            if (confirmed) {
                confirmed = false
                error = "ZOOM_RATIO_LOST"
            }
        }
    }
}
