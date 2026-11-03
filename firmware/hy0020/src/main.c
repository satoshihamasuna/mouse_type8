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

#define NUS_UUID(n) BT_UUID_128_ENCODE(n, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca9e)
#define MAX_PAYLOAD 244
/* nRF52832 UARTE has an 8-bit MAXCNT register (at most 255 bytes).
 * Passing 256 to the HAL writes an effective length of zero.
 */
#define RX_BUFFER_SIZE 128
BUILD_ASSERT(RX_BUFFER_SIZE > 0 &&
             RX_BUFFER_SIZE <= BIT_MASK(UARTE0_EASYDMA_MAXCNT_SIZE),
             "UART RX buffer exceeds the hardware EasyDMA MAXCNT limit");

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
RING_BUF_DECLARE(uart_to_ble, 16384);
K_MSGQ_DEFINE(ble_to_uart, sizeof(struct uart_message), 8, 4);
K_MEM_SLAB_DEFINE(rx_buffers, RX_BUFFER_SIZE, 3, 4);
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
    atomic_inc(&session_id);
    k_spinlock_key_t key = k_spin_lock(&stream_lock);
    ring_buf_reset(&uart_to_ble);
    k_spin_unlock(&stream_lock, key);
    k_msgq_purge(&ble_to_uart);
    k_sem_give(&rx_available);
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

/* Standard NUS UUIDs. RX uses acknowledged writes to expose backpressure. */
BT_GATT_SERVICE_DEFINE(bridge_service,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(NUS_UUID(0x6e400001))),
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(NUS_UUID(0x6e400003)),
        BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(NUS_UUID(0x6e400002)),
        BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, receive_bytes, NULL),
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(NUS_UUID(0x6e400004)),
        BT_GATT_CHRC_READ, BT_GATT_PERM_READ, read_status, NULL, NULL)
);

static void restart_rx(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(rx_restart_work, restart_rx);

static void restart_rx(struct k_work *work)
{
    ARG_UNUSED(work);
    void *buffer;
    if (k_mem_slab_alloc(&rx_buffers, &buffer, K_NO_WAIT) != 0) {
        k_work_reschedule(&rx_restart_work, K_MSEC(10));
        return;
    }
    int err = uart_rx_enable(bridge_uart, buffer, RX_BUFFER_SIZE, 1000);
    if (err) {
        k_mem_slab_free(&rx_buffers, buffer);
        if (err != -EBUSY) {
            k_work_reschedule(&rx_restart_work, K_MSEC(10));
        }
    }
}

static void uart_event(const struct device *dev, struct uart_event *event, void *data)
{
    ARG_UNUSED(data);
    switch (event->type) {
    case UART_RX_BUF_REQUEST: {
        void *buffer;
        if (k_mem_slab_alloc(&rx_buffers, &buffer, K_NO_WAIT) == 0) {
            if (uart_rx_buf_rsp(dev, buffer, RX_BUFFER_SIZE) != 0) {
                k_mem_slab_free(&rx_buffers, buffer);
            }
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
        if (atomic_get(&linked) && atomic_get(&subscribed)) {
            atomic_inc(&uart_errors);
            atomic_set(&fault_pending, 1);
        }
        break;
    case UART_RX_DISABLED:
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
    for (;;) {
        struct bt_conn *conn = connection_ref();
        if (!conn) { k_sleep(K_MSEC(20)); continue; }
        if (atomic_cas(&fault_pending, 1, 0)) {
            bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
            bt_conn_unref(conn);
            continue;
        }
        uint32_t session = (uint32_t)atomic_get(&session_id);
        uint16_t mtu = bt_gatt_get_mtu(conn);
        if (mtu <= 3) {
            /* ATT may already be gone while the disconnect callback is pending. */
            bt_conn_unref(conn);
            k_sleep(K_MSEC(20));
            continue;
        }
        uint16_t max_length = MIN(MAX_PAYLOAD, mtu - 3);
        k_spinlock_key_t key = k_spin_lock(&stream_lock);
        uint32_t length = atomic_get(&subscribed) ?
            ring_buf_get(&uart_to_ble, pending, max_length) : 0;
        k_spin_unlock(&stream_lock, key);
        if (length == 0) {
            bt_conn_unref(conn);
            k_sem_take(&rx_available, K_MSEC(20));
            continue;
        }
        int64_t deadline = k_uptime_get() + 2000;
        while (atomic_get(&linked) && atomic_get(&subscribed) &&
               session == (uint32_t)atomic_get(&session_id)) {
            int err = bt_gatt_notify(conn, &bridge_service.attrs[2], pending, length);
            if (err == 0 || err == -ENOTCONN) { break; }
            if ((err == -ENOMEM || err == -EAGAIN) && k_uptime_get() < deadline) {
                k_sleep(K_MSEC(2));
                continue;
            }
            atomic_inc(&ble_errors);
            atomic_set(&fault_pending, 1);
            break;
        }
        bt_conn_unref(conn);
    }
}
