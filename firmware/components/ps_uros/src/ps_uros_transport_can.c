/* micro-ROS custom transport over the XRCE CAN plane (docs/network-map.md §3.4, §7).
 *
 * The client runs with framing = true, so what crosses this boundary is an HDLC
 * byte stream, not datagrams: writes are sliced into 8-byte CAN payloads and reads
 * are drained from a stream buffer fed by the CAN receive callback. CAN guarantees
 * in-order delivery for a fixed identifier, and HDLC supplies its own sync and
 * CRC, so no sequence numbering is needed on this plane.
 *
 * Node 8 reassembles each source's stream onto a PTY, where a stock
 * `micro-ros-agent multiserial` terminates it. */
#include "ps_uros_internal.h"

#if PS_UROS_ENABLED

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"

#include "powersuit_proto/can_id.h"

#include <string.h>

static const char *TAG = "ps_uros";

static struct {
    ps_can_handle_t can;
    uint8_t node_id;
    StreamBufferHandle_t rx;
    uint32_t rx_overruns;
} t;

static void xrce_rx(const ps_can_frame_t *frame, void *arg)
{
    (void)arg;
    if (frame->dlc == 0) {
        return;
    }
    size_t sent = xStreamBufferSend(t.rx, frame->data, frame->dlc, 0);
    if (sent != frame->dlc) {
        /* The agent will retransmit: XRCE reliability sits above this layer. */
        t.rx_overruns++;
    }
}

esp_err_t ps_uros_transport_init(ps_can_handle_t can, uint8_t node_id, size_t ring_bytes)
{
    t.can = can;
    t.node_id = node_id;
    t.rx = xStreamBufferCreate(ring_bytes, 1);
    if (t.rx == NULL) {
        return ESP_ERR_NO_MEM;
    }
    return ps_can_register_class_cb(can, PS_CLS_XRCE, xrce_rx, NULL);
}

bool ps_uros_open(struct uxrCustomTransport *transport)
{
    (void)transport;
    xStreamBufferReset(t.rx);
    return true;
}

bool ps_uros_close(struct uxrCustomTransport *transport)
{
    (void)transport;
    return true;
}

size_t ps_uros_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len,
                     uint8_t *err)
{
    (void)transport;
    size_t written = 0;
    uint32_t id = ps_can_id_pack(PS_CLS_XRCE, t.node_id, PS_NODE_ORCH, PS_T_XRCE_STREAM, 0);

    while (written < len) {
        ps_can_frame_t f;
        uint8_t chunk = (uint8_t)((len - written) > 8u ? 8u : (len - written));
        f.id = id;
        f.dlc = chunk;
        memcpy(f.data, buf + written, chunk);
        if (chunk < 8) {
            memset(f.data + chunk, 0, 8u - chunk);
        }
        /* A short timeout keeps a congested bus from stalling the session task;
         * a partial write is a normal, recoverable outcome for the uxr layer. */
        if (ps_can_send(t.can, &f, pdMS_TO_TICKS(10)) != ESP_OK) {
            break;
        }
        written += chunk;
    }

    if (err != NULL) {
        *err = (written == len) ? 0 : 1;
    }
    return written;
}

size_t ps_uros_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout,
                    uint8_t *err)
{
    (void)transport;
    TickType_t ticks = (timeout <= 0) ? 0 : pdMS_TO_TICKS((uint32_t)timeout);
    size_t got = xStreamBufferReceive(t.rx, buf, len, ticks);
    if (err != NULL) {
        *err = (got > 0) ? 0 : 1;
    }
    return got;
}

void ps_uros_transport_log_stats(void)
{
    if (t.rx_overruns) {
        ESP_LOGW(TAG, "xrce rx overruns: %u", (unsigned)t.rx_overruns);
    }
}

#endif /* PS_UROS_ENABLED */
