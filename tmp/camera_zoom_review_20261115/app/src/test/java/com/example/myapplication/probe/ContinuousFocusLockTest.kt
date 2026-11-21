package com.example.myapplication.probe

import org.junit.Assert.*
import org.junit.Test

class ContinuousFocusLockTest {
    private fun ms(value: Long) = value * 1_000_000L
    @Test fun locksOnceAfterStablePassiveFocusAndDetectsLoss() {
        val lock = ContinuousFocusLock(true)
        assertEquals(ContinuousFocusLock.Action.NONE, lock.observe(1, ms(0)))
        for (t in listOf(100L, 200L, 300L)) assertEquals(ContinuousFocusLock.Action.NONE, lock.observe(2, ms(t)))
        assertEquals(ContinuousFocusLock.Action.TRIGGER, lock.observe(2, ms(400)))
        assertFalse(lock.locked)
        assertEquals(ContinuousFocusLock.Action.NONE, lock.observe(2, ms(433)))
        assertEquals(ContinuousFocusLock.Action.NONE, lock.observe(4, ms(466)))
        assertTrue(lock.locked)
        assertEquals(ContinuousFocusLock.Action.NONE, lock.observe(4, ms(8000)))
        assertEquals(ContinuousFocusLock.Action.FAIL, lock.observe(1, ms(8033)))
        assertEquals("FOCUS_LOCK_LOST", lock.error)
        assertFalse(lock.locked)
        assertEquals(ContinuousFocusLock.Action.NONE, lock.observe(2, ms(8100)))
    }
    @Test fun rescansAndMissingCallbacksResetStability() {
        val lock = ContinuousFocusLock(true)
        lock.observe(2, ms(0)); lock.observe(2, ms(100)); lock.observe(2, ms(200))
        assertEquals(ContinuousFocusLock.Action.NONE, lock.observe(1, ms(250)))
        lock.observe(2, ms(300)); lock.observe(2, ms(400)); lock.observe(2, ms(500))
        assertEquals(ContinuousFocusLock.Action.NONE, lock.observe(2, ms(1000)))
        lock.observe(2, ms(1100)); lock.observe(2, ms(1200))
        assertEquals(ContinuousFocusLock.Action.TRIGGER, lock.observe(2, ms(1300)))
    }
    @Test fun refusesFailedOrUnconfirmedLockWithoutRetrigger() {
        for (failedState in listOf(5, null)) {
            val lock = ContinuousFocusLock(true)
            for (t in listOf(0L, 100L, 200L, 300L)) lock.observe(2, ms(t))
            if (failedState == 5) {
                assertEquals(ContinuousFocusLock.Action.FAIL, lock.observe(5, ms(400)))
                assertEquals("FOCUS_LOCK_NOT_FOCUSED", lock.error)
            } else {
                assertEquals(ContinuousFocusLock.Action.FAIL, lock.observe(null, ms(5000)))
                assertEquals("FOCUS_LOCK_TIMEOUT", lock.error)
            }
            assertEquals(ContinuousFocusLock.Action.NONE, lock.observe(2, ms(5100)))
            assertFalse(lock.locked)
        }
    }
    @Test fun disabledPreservesExistingRecording() {
        val lock = ContinuousFocusLock(false)
        assertEquals(ContinuousFocusLock.Action.NONE, lock.observe(null, ms(9000)))
        assertEquals(ContinuousFocusLock.State.DISABLED, lock.state)
        assertNull(lock.error)
    }
}
