package com.example.myapplication.probe

/** Camera2 continuous-video AF lock. A high-speed trigger is submitted as ONE burst,
 * then the existing IDLE repeating burst resumes. Never retry a trigger automatically. */
class ContinuousFocusLock(private val enabled: Boolean) {
    enum class State { DISABLED, SETTLING, TRIGGER_SENT, LOCKED, FAILED }
    enum class Action { NONE, TRIGGER, FAIL }
    @Volatile var state = if (enabled) State.SETTLING else State.DISABLED
        private set
    @Volatile var error: String? = null
        private set
    private var firstSampleNs: Long? = null
    private var focusedSinceNs: Long? = null
    private var previousSampleNs: Long? = null
    val locked: Boolean get() = state == State.LOCKED

    @Synchronized fun observe(af: Int?, nowNs: Long): Action {
        if (!enabled || state == State.FAILED) return Action.NONE
        if (state == State.LOCKED) {
            return if (af == 4) Action.NONE else fail("FOCUS_LOCK_LOST")
        }
        val first = firstSampleNs ?: nowNs.also { firstSampleNs = it }
        if (nowNs - first >= 5_000_000_000L) return fail("FOCUS_LOCK_TIMEOUT")
        if (state == State.TRIGGER_SENT) {
            if (af == 4) state = State.LOCKED
            if (af == 5) return fail("FOCUS_LOCK_NOT_FOCUSED")
            return Action.NONE
        }
        // Camera2 AF states: 2 PASSIVE_FOCUSED, 4 FOCUSED_LOCKED, 5 NOT_FOCUSED_LOCKED.
        // A callback gap cannot count as evidence of continuous stable focus.
        val previous = previousSampleNs
        previousSampleNs = nowNs
        if (af != 2) {
            focusedSinceNs = null
            return Action.NONE
        }
        if (previous == null || nowNs - previous > 150_000_000L) focusedSinceNs = nowNs
        val stable = focusedSinceNs ?: nowNs.also { focusedSinceNs = it }
        if (nowNs - stable < 300_000_000L) return Action.NONE
        state = State.TRIGGER_SENT
        return Action.TRIGGER
    }

    private fun fail(message: String): Action {
        state = State.FAILED
        error = message
        return Action.FAIL
    }
}
