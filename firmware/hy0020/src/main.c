#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/ring_buffer.h>
#include <hal/nrf_uarte.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include "batch_policy.h"

#define NUS_UUID(n) BT_UUID_128_ENCODE(n, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca9e)
#define MAX_PAYLOAD 244
#define UART_TO_BLE_CAPACITY 16384
#define BATCH_FLUSH_MS 7
/* nRF52832 UARTE has an 8-bit MAXCNT register (at most 255 bytes).
 * Passing 256 to the HAL writes an effective length of zero.
 */
#define RX_BUFFER_SIZE 252
#define RX_BUFFER_COUNT 4
#define RX_TIMEOUT_US 1000
BUILD_ASSERT(RX_BUFFER_SIZE > 0 &&
             RX_BUFFER_SIZE <= BIT_MASK(UARTE0_EASYDMA_MAXCNT_SIZE),
             "UART RX buffer exceeds the hardware EasyDMA MAXCNT limit");
BUILD_ASSERT(IS_ENABLED(CONFIG_UARTE_NRFX_UARTE_COUNT_BYTES_WITH_TIMER_LEGACY),
             "UART RX requires hardware byte counting to avoid timeout STOPRX");

struct uart_message {
    uint32_t session;
    uint16_t length;
    uint8_t bytes[MAX_PAYLOAD];
};

static const struct device *const bridge_uart = DEVICE_DT_GET(DT_NODELABEL(uart0));
static struct bt_conn *connection;
static struct k_spinlock connection_lock;
static struct k_spinlock stream_lock;
static atomic_t linked, subscribed, session_id, fault_pending;
/* Readable over the status characteristic; never mixed into the UART stream. */
static atomic_t dropped_uart_bytes, uart_errors, queue_busy, ble_errors;
/* Lifetime diagnostics include errors outside a subscribed BLE session. */
static atomic_t last_stop_reason, stop_reason_or;
static atomic_t overrun_count, parity_count, framing_count, break_count;
static atomic_t collision_count, noise_count, other_reason_count;
static atomic_t rx_disabled_count, rx_restart_attempts, rx_restart_failures;
static atomic_t rx_buffer_alloc_failures, rx_buffer_rsp_failures, rx_buf_request_events;
/* Lifetime transmit counters are not reset on disconnect or CCC changes. */
static atomic_t ring_highwater, notify_calls, notify_bytes, notify_failures;
static atomic_t notify_call_max_ms, notify_retry_events;
RING_BUF_DECLARE(uart_to_ble, UART_TO_BLE_CAPACITY);
K_MSGQ_DEFINE(ble_to_uart, sizeof(struct uart_message), 8, 4);
K_MEM_SLAB_DEFINE(rx_buffers, RX_BUFFER_SIZE, RX_BUFFER_COUNT, 4);
K_SEM_DEFINE(rx_available, 0, 1);

static struct bt_conn *connection_ref(void)
{
    k_spinlock_key_t key = k_spin_lock(&connection_lock);
    struct bt_conn *result = connection ? bt_conn_ref(connection) : NULL;
    k_spin_unlock(&connection_lock, key);
    return result;
}

static void reset_stream(void)
{
    k_spinlock_key_t key = k_spin_lock(&stream_lock);
    atomic_inc(&session_id);
    ring_buf_reset(&uart_to_ble);
    k_spin_unlock(&stream_lock, key);
    k_msgq_purge(&ble_to_uart);
    k_sem_give(&rx_available);
}

static void update_peak(atomic_t *peak, uint32_t value)
{
    atomic_val_t previous = atomic_get(peak);
    while (value > (uint32_t)previous &&
           !atomic_cas(peak, previous, (atomic_val_t)value)) {
        previous = atomic_get(peak);
    }
}

/* Keep bytes in the shared ring while coalescing so a session reset discards
 * partial batches too. New arrivals do not extend a partial waiting episode.
 */
static uint32_t uart_batch_get(struct uart_ble_batch_policy *policy,
                                uint8_t *pending, uint16_t max_length,
                                uint32_t *session, int64_t *wait_ms)
{
    k_spinlock_key_t key = k_spin_lock(&stream_lock);
    *session = (uint32_t)atomic_get(&session_id);
    uint32_t available = atomic_get(&subscribed) ? ring_buf_size_get(&uart_to_ble) : 0;
    uint32_t length = 0;
    if (uart_ble_batch_ready(policy, *session, available, max_length,
                            k_uptime_get(), BATCH_FLUSH_MS, wait_ms)) {
        length = ring_buf_get(&uart_to_ble, pending, max_length);
    }
    k_spin_unlock(&stream_lock, key);
    return length;
}

static void ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    atomic_clear(&subscribed);
    reset_stream();
    atomic_set(&subscribed, value == BT_GATT_CCC_NOTIFY);
}

static ssize_t receive_bytes(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                             const void *buf, uint16_t len, uint16_t offset,
                             uint8_t flags)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    if (offset != 0 || (flags & BT_GATT_WRITE_FLAG_PREPARE)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
    if (!atomic_get(&linked) || !atomic_get(&subscribed)) {
        return BT_GATT_ERR(BT_ATT_ERR_CCC_IMPROPER_CONF);
    }
    if (len > MAX_PAYLOAD) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    if (len == 0) {
        return 0;
    }
    struct uart_message message = {
        .session = (uint32_t)atomic_get(&session_id), .length = len,
    };
    memcpy(message.bytes, buf, len);
    if (k_msgq_put(&ble_to_uart, &message, K_NO_WAIT) != 0) {
        atomic_inc(&queue_busy);
        /* With Write Requests, the PC can retry this rejected chunk safely. */
        return BT_GATT_ERR(BT_ATT_ERR_INSUFFICIENT_RESOURCES);
    }
    return len;
}

static ssize_t read_status(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                           void *buf, uint16_t len, uint16_t offset)
{
    uint8_t status[16];
    sys_put_le32((uint32_t)atomic_get(&dropped_uart_bytes), &status[0]);
    sys_put_le32((uint32_t)atomic_get(&uart_errors), &status[4]);
    sys_put_le32((uint32_t)atomic_get(&queue_busy), &status[8]);
    sys_put_le32((uint32_t)atomic_get(&ble_errors), &status[12]);
    return bt_gatt_attr_read(conn, attr, buf, len, offset, status, sizeof(status));
}

static ssize_t read_diagnostics(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                void *buf, uint16_t len, uint16_t offset)
{
    /* One connection, serialized ATT reads. Keep Read Blob fragments on the
     * same sample; individual counters are atomic, not a simultaneous snapshot.
     */
    static uint32_t snapshot[20];
    if (offset == 0) {
        uint32_t values[] = {
            1,
            (uint32_t)atomic_get(&last_stop_reason),
            (uint32_t)atomic_get(&stop_reason_or),
            (uint32_t)atomic_get(&overrun_count),
            (uint32_t)atomic_get(&parity_count),
            (uint32_t)atomic_get(&framing_count),
            (uint32_t)atomic_get(&break_count),
            (uint32_t)atomic_get(&collision_count),
            (uint32_t)atomic_get(&noise_count),
            (uint32_t)atomic_get(&other_reason_count),
            (uint32_t)atomic_get(&rx_disabled_count),
            (uint32_t)atomic_get(&rx_restart_attempts),
            (uint32_t)atomic_get(&rx_restart_failures),
            (uint32_t)atomic_get(&rx_buffer_alloc_failures),
            (uint32_t)atomic_get(&rx_buffer_rsp_failures),
            (uint32_t)atomic_get(&rx_buf_request_events),
            RX_BUFFER_SIZE, RX_BUFFER_COUNT, RX_TIMEOUT_US, BIT(0),
        };
        BUILD_ASSERT(sizeof(values) == sizeof(snapshot));
        for (size_t i = 0; i < ARRAY_SIZE(values); ++i) {
            snapshot[i] = sys_cpu_to_le32(values[i]);
        }
    }
    return bt_gatt_attr_read(conn, attr, buf, len, offset, snapshot, sizeof(snapshot));
}

static ssize_t read_link_diagnostics(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                     void *buf, uint16_t len, uint16_t offset)
{
    /* Fix Read Blob fragments at offset zero, as for UUID 0005. */
    static uint32_t snapshot[24];
    if (offset == 0) {
        uint32_t values[24] = {1};
        values[1] = bt_gatt_get_mtu(conn);
        struct bt_conn_info info;
        if (bt_conn_get_info(conn, &info) == 0 && info.type == BT_CONN_TYPE_LE &&
            info.state == BT_CONN_STATE_CONNECTED) {
            values[2] = info.le.interval_us;
            values[3] = info.le.latency;
            values[4] = (uint32_t)info.le.timeout * 10U;
            if (info.le.phy != NULL) {
                values[5] = info.le.phy->tx_phy;
                values[6] = info.le.phy->rx_phy;
            }
            if (info.le.data_len != NULL) {
                values[7] = info.le.data_len->tx_max_len;
                values[8] = info.le.data_len->tx_max_time;
                values[9] = info.le.data_len->rx_max_len;
                values[10] = info.le.data_len->rx_max_time;
            }
        }
        k_spinlock_key_t key = k_spin_lock(&stream_lock);
        values[11] = ring_buf_size_get(&uart_to_ble);
        k_spin_unlock(&stream_lock, key);
        values[12] = (uint32_t)atomic_get(&ring_highwater);
        values[13] = (uint32_t)atomic_get(&notify_calls);
        values[14] = (uint32_t)atomic_get(&notify_bytes);
        values[15] = (uint32_t)atomic_get(&notify_failures);
        values[16] = (uint32_t)atomic_get(&notify_call_max_ms);
        values[17] = (uint32_t)atomic_get(&notify_retry_events);
        values[18] = BATCH_FLUSH_MS;
        values[19] = CONFIG_BT_CTLR_SDC_TX_PACKET_COUNT;
        values[20] = CONFIG_BT_BUF_ACL_TX_COUNT;
        values[21] = CONFIG_BT_BUF_ACL_TX_SIZE;
        values[22] = UART_TO_BLE_CAPACITY;
        values[23] = BIT(0);
        BUILD_ASSERT(sizeof(values) == sizeof(snapshot));
        for (size_t i = 0; i < ARRAY_SIZE(values); ++i) {
            snapshot[i] = sys_cpu_to_le32(values[i]);
        }
    }
    return bt_gatt_attr_read(conn, attr, buf, len, offset, snapshot, sizeof(snapshot));
}

/* Standard NUS UUIDs. RX uses acknowledged writes to expose backpressure. */
BT_GATT_SERVICE_DEFINE(bridge_service,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(NUS_UUID(0x6e400001))),
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(NUS_UUID(0x6e400003)),
        BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(NUS_UUID(0x6e400002)),
        BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, receive_bytes, NULL),
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(NUS_UUID(0x6e400004)),
        BT_GATT_CHRC_READ, BT_GATT_PERM_READ, read_status, NULL, NULL),
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(NUS_UUID(0x6e400005)),
        BT_GATT_CHRC_READ, BT_GATT_PERM_READ, read_diagnostics, NULL, NULL),
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(NUS_UUID(0x6e400006)),
        BT_GATT_CHRC_READ, BT_GATT_PERM_READ, read_link_diagnostics, NULL, NULL)
);

static void restart_rx(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(rx_restart_work, restart_rx);

static void restart_rx(struct k_work *work)
{
    ARG_UNUSED(work);
    void *buffer;
    if (k_mem_slab_alloc(&rx_buffers, &buffer, K_NO_WAIT) != 0) {
        atomic_inc(&rx_buffer_alloc_failures);
        k_work_reschedule(&rx_restart_work, K_MSEC(10));
        return;
    }
    atomic_inc(&rx_restart_attempts);
    int err = uart_rx_enable(bridge_uart, buffer, RX_BUFFER_SIZE, RX_TIMEOUT_US);
    if (err) {
        atomic_inc(&rx_restart_failures);
        k_mem_slab_free(&rx_buffers, buffer);
        if (err != -EBUSY) {
            k_work_reschedule(&rx_restart_work, K_MSEC(10));
        }
    }
}

static void record_rx_stop(uint32_t reason)
{
    const uint32_t known = UART_ERROR_OVERRUN | UART_ERROR_PARITY | UART_ERROR_FRAMING |
                           UART_BREAK | UART_ERROR_COLLISION | UART_ERROR_NOISE;
    atomic_set(&last_stop_reason, (atomic_val_t)reason);
    atomic_or(&stop_reason_or, (atomic_val_t)reason);
    if (reason & UART_ERROR_OVERRUN) { atomic_inc(&overrun_count); }
    if (reason & UART_ERROR_PARITY) { atomic_inc(&parity_count); }
    if (reason & UART_ERROR_FRAMING) { atomic_inc(&framing_count); }
    if (reason & UART_BREAK) { atomic_inc(&break_count); }
    if (reason & UART_ERROR_COLLISION) { atomic_inc(&collision_count); }
    if (reason & UART_ERROR_NOISE) { atomic_inc(&noise_count); }
    if (reason == 0 || (reason & ~known)) { atomic_inc(&other_reason_count); }
}

static void uart_event(const struct device *dev, struct uart_event *event, void *data)
{
    ARG_UNUSED(data);
    switch (event->type) {
    case UART_RX_BUF_REQUEST: {
        atomic_inc(&rx_buf_request_events);
        void *buffer;
        if (k_mem_slab_alloc(&rx_buffers, &buffer, K_NO_WAIT) == 0) {
            if (uart_rx_buf_rsp(dev, buffer, RX_BUFFER_SIZE) != 0) {
                atomic_inc(&rx_buffer_rsp_failures);
                k_mem_slab_free(&rx_buffers, buffer);
            }
        } else {
            atomic_inc(&rx_buffer_alloc_failures);
        }
        break;
    }
    case UART_RX_BUF_RELEASED:
        k_mem_slab_free(&rx_buffers, event->data.rx_buf.buf);
        break;
    case UART_RX_RDY:
    {
        /* Check the session under the same lock used by reset_stream(). */
        k_spinlock_key_t key = k_spin_lock(&stream_lock);
        if (atomic_get(&linked) && atomic_get(&subscribed)) {
            size_t length = event->data.rx.len;
            uint32_t stored = ring_buf_put(&uart_to_ble,
                event->data.rx.buf + event->data.rx.offset, length);
            update_peak(&ring_highwater, ring_buf_size_get(&uart_to_ble));
            k_spin_unlock(&stream_lock, key);
            if (stored != length) {
                atomic_add(&dropped_uart_bytes, length - stored);
                atomic_set(&fault_pending, 1);
            }
            k_sem_give(&rx_available);
        } else {
            k_spin_unlock(&stream_lock, key);
        }
        break;
    }
    case UART_RX_STOPPED:
        record_rx_stop((uint32_t)event->data.rx_stop.reason);
        if (atomic_get(&linked) && atomic_get(&subscribed)) {
            atomic_inc(&uart_errors);
            atomic_set(&fault_pending, 1);
        }
        break;
    case UART_RX_DISABLED:
        atomic_inc(&rx_disabled_count);
        k_work_reschedule(&rx_restart_work, K_NO_WAIT);
        break;
    default:
        break;
    }
}

static void uart_tx_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    struct uart_message message;
    for (;;) {
        k_msgq_get(&ble_to_uart, &message, K_FOREVER);
        for (uint16_t i = 0; i < message.length; ++i) {
            if (!atomic_get(&linked) || !atomic_get(&subscribed) ||
                message.session != (uint32_t)atomic_get(&session_id)) {
                break;
            }
            uart_poll_out(bridge_uart, message.bytes[i]);
            if (CONFIG_MOUSE_UART_BYTE_GAP_US > 0) {
                k_sleep(K_USEC(CONFIG_MOUSE_UART_BYTE_GAP_US));
            }
        }
    }
}
K_THREAD_DEFINE(uart_tx_id, 2048, uart_tx_thread, NULL, NULL, NULL, 7, 0, 0);

static void advertise(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(advertise_work, advertise);
static const struct bt_data advertisement[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, NUS_UUID(0x6e400001)),
};
static const struct bt_data scan_response[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static void advertise(struct k_work *work)
{
    ARG_UNUSED(work);
    if (atomic_get(&linked)) { return; }
    int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2,
        advertisement, ARRAY_SIZE(advertisement), scan_response, ARRAY_SIZE(scan_response));
    if (err && err != -EALREADY) {
        k_work_reschedule(&advertise_work, K_SECONDS(1));
    }
}

static void on_connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        k_work_reschedule(&advertise_work, K_MSEC(250));
        return;
    }
    reset_stream();
    atomic_clear(&fault_pending);
    k_spinlock_key_t key = k_spin_lock(&connection_lock);
    connection = bt_conn_ref(conn);
    k_spin_unlock(&connection_lock, key);
    atomic_set(&linked, 1);
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
    ARG_UNUSED(reason);
    atomic_clear(&linked);
    atomic_clear(&subscribed);
    reset_stream();
    k_spinlock_key_t key = k_spin_lock(&connection_lock);
    struct bt_conn *previous = connection;
    if (previous == conn) { connection = NULL; }
    k_spin_unlock(&connection_lock, key);
    if (previous == conn) { bt_conn_unref(previous); }
    k_work_reschedule(&advertise_work, K_MSEC(250));
}
BT_CONN_CB_DEFINE(connection_callbacks) = {
    .connected = on_connected, .disconnected = on_disconnected,
};

int main(void)
{
    if (!device_is_ready(bridge_uart) || uart_callback_set(bridge_uart, uart_event, NULL)) {
        return 0;
    }
    k_work_reschedule(&rx_restart_work, K_NO_WAIT);
    if (bt_enable(NULL)) { return 0; }
    k_work_reschedule(&advertise_work, K_NO_WAIT);
    uint8_t pending[MAX_PAYLOAD];
    struct uart_ble_batch_policy batch = {0};
    for (;;) {
        struct bt_conn *conn = connection_ref();
        if (!conn) { k_sleep(K_MSEC(20)); continue; }
        if (atomic_cas(&fault_pending, 1, 0)) {
            bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
            bt_conn_unref(conn);
            continue;
        }
        uint16_t mtu = bt_gatt_get_mtu(conn);
        if (mtu <= 3) {
            /* ATT may already be gone while the disconnect callback is pending. */
            bt_conn_unref(conn);
            k_sleep(K_MSEC(20));
            continue;
        }
        uint16_t max_length = MIN(MAX_PAYLOAD, mtu - 3);
        uint32_t session;
        int64_t wait_ms;
        uint32_t length = uart_batch_get(&batch, pending, max_length, &session, &wait_ms);
        if (length == 0) {
            bt_conn_unref(conn);
            k_sem_take(&rx_available, K_MSEC(wait_ms));
            continue;
        }
        int64_t deadline = k_uptime_get() + 2000;
        while (atomic_get(&linked) && atomic_get(&subscribed) &&
               session == (uint32_t)atomic_get(&session_id)) {
            int64_t started_ms = k_uptime_get();
            atomic_inc(&notify_calls);
            int err = bt_gatt_notify(conn, &bridge_service.attrs[2], pending, length);
            uint32_t duration_ms = (uint32_t)MIN(k_uptime_get() - started_ms, UINT32_MAX);
            update_peak(&notify_call_max_ms, duration_ms);
            if (err == 0) {
                /* Accepted by the host stack, not proof of over-the-air delivery. */
                atomic_add(&notify_bytes, length);
                break;
            }
            if (err == -ENOTCONN) { break; }
            if (err == -ENOMEM || err == -EAGAIN) {
                if (k_uptime_get() < deadline) {
                    atomic_inc(&notify_retry_events);
                    k_sleep(K_MSEC(2));
                    continue;
                }
            }
            atomic_inc(&notify_failures);
            atomic_inc(&ble_errors);
            atomic_set(&fault_pending, 1);
            break;
        }
        bt_conn_unref(conn);
    }
}
