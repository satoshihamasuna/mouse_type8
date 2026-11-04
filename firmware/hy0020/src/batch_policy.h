#ifndef MOUSE_UART_BLE_BATCH_POLICY_H
#define MOUSE_UART_BLE_BATCH_POLICY_H

#include <stdbool.h>
#include <stdint.h>

/* The consumer owns this state. Calls and the associated ring read must use
 * the same session snapshot. Zero initialization starts with no pending wait.
 */
struct uart_ble_batch_policy {
    uint32_t session;
    int64_t deadline_ms;
    bool waiting;
};

/* True permits immediate consumption of up to max_length bytes. Otherwise
 * wait_ms is the remaining bounded partial wait, or 20 ms for an empty ring.
 * A full or timed-out batch ends the episode; its suffix can start a new one.
 */
static inline bool uart_ble_batch_ready(struct uart_ble_batch_policy *policy,
                                        uint32_t session, uint32_t available,
                                        uint32_t max_length, int64_t now_ms,
                                        int64_t flush_ms, int64_t *wait_ms)
{
    *wait_ms = 20;
    if (policy->session != session) {
        policy->session = session;
        policy->waiting = false;
    }
    if (available == 0 || max_length == 0) {
        policy->waiting = false;
        return false;
    }
    if (available >= max_length) {
        policy->waiting = false;
        return true;
    }
    if (!policy->waiting) {
        policy->deadline_ms = now_ms + flush_ms;
        policy->waiting = true;
    }
    if (now_ms >= policy->deadline_ms) {
        policy->waiting = false;
        return true;
    }
    *wait_ms = policy->deadline_ms - now_ms;
    return false;
}

#endif /* MOUSE_UART_BLE_BATCH_POLICY_H */
