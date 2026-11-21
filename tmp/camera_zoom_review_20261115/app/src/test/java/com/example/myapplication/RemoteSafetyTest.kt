package com.example.myapplication

import com.example.myapplication.remote.OperationGate
import com.example.myapplication.remote.ReceiptPolicy
import com.example.myapplication.remote.TransferIdentity
import com.example.myapplication.remote.TransferReceipt
import org.junit.Assert.*
import org.junit.Test

class RemoteSafetyTest {
    private val expected = TransferIdentity("record_1", 100, "a".repeat(64))
    private val receipt = TransferReceipt("record_1", 100, "a".repeat(64), true)

    @Test fun successfulMatchingReceiptAllowsDeletion() {
        assertTrue(ReceiptPolicy.matches(expected, receipt))
    }

    @Test fun missingCommitNeverAllowsDeletion() {
        assertFalse(ReceiptPolicy.matches(expected, receipt.copy(committed = false)))
    }

    @Test fun wrongIdSizeOrHashNeverAllowsDeletion() {
        assertFalse(ReceiptPolicy.matches(expected, receipt.copy(id = "other")))
        assertFalse(ReceiptPolicy.matches(expected, receipt.copy(size = 99)))
        assertFalse(ReceiptPolicy.matches(expected, receipt.copy(sha256 = "b".repeat(64))))
        assertFalse(ReceiptPolicy.matches(expected.copy(sha256 = ""), receipt))
    }

    @Test fun secondClientCannotAcquireAnActiveCameraOrReleaseAnotherOwnersLease() {
        val gate = OperationGate()
        assertTrue(gate.acquire("gui"))
        assertFalse(gate.acquire("script"))
        gate.release("script")
        assertFalse(gate.acquire("script"))
        gate.release("gui")
        assertTrue(gate.acquire("script"))
    }
}
