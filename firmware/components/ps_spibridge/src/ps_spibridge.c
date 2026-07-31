/* ps_spibridge — ESP32-P4 SPI slave bridge to Node 8 (docs/network-map.md §6).
 *
 * SPI2 is the only slave-capable GPSPI on the P4 (SPI0/1 are flash-only). The
 * Pi 5 is master: fixed 512-byte full-duplex DMA transactions at 1 kHz, plus
 * extra transactions whenever DATA_READY is high. Two transaction slots are
 * kept in flight ping-pong style so the peripheral always has an armed buffer;
 * the pump task rebuilds and re-queues each slot as it completes. DMA buffers
 * are WORD_ALIGNED_ATTR and every transaction length is a multiple of 4 bytes,
 * per the IDF v5.5.5 esp32p4 spi_slave contract. */
#include "ps_spibridge.h"

#include <string.h>

#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "ps_spib";

#define SPIB_HOST            SPI2_HOST
#define SPIB_TASK_PRIO       17
#define SPIB_TASK_CORE       0
#define SPIB_TASK_STACK      4096
#define SPIB_DEF_UPLINK_LEN  256
#define SPIB_DEF_DOWNLINK_LEN 128

typedef struct {
    spi_slave_transaction_t trans;
    uint8_t *tx;
    uint8_t *rx;
} spib_slot_t;

/* DMA transaction buffers: internal SRAM, 4-byte aligned (spi_slave DMA rule). */
WORD_ALIGNED_ATTR static uint8_t s_txbuf[2][PS_SPI_XFER_SIZE];
WORD_ALIGNED_ATTR static uint8_t s_rxbuf[2][PS_SPI_XFER_SIZE];

static spib_slot_t      s_slot[2];
static QueueHandle_t    s_uplink_q;
static QueueHandle_t    s_downlink_q;
static int              s_data_ready_gpio = -1;
static bool             s_started;

static portMUX_TYPE     s_lock = portMUX_INITIALIZER_UNLOCKED;
static ps_spib_stats_t  s_stats;
static uint8_t          s_sticky_flags;      /* ESTOP_LATCHED / HUB_FAULT */
static bool             s_overflow_pending;  /* one-shot: reported in next header */
static uint8_t          s_tx_seq;
static uint8_t          s_rx_seq;
static bool             s_have_rx_seq;
static uint32_t         s_downlink_drops;    /* internal (stats struct is frozen) */

static void spib_update_data_ready(void)
{
    gpio_set_level((gpio_num_t)s_data_ready_gpio,
                   uxQueueMessagesWaiting(s_uplink_q) > 0 ? 1 : 0);
}

/* An idle master (Pi clocking zeros to poll the uplink) is not a protocol
 * error: a zero header has no magic, no records, nothing to lose. */
static bool spib_rx_is_idle(const uint8_t *rx, size_t len)
{
    if (len == 0) {
        return true;
    }
    if (len < PS_SPI_HDR_SIZE) {
        return false;
    }
    for (size_t i = 0; i < PS_SPI_HDR_SIZE; i++) {
        if (rx[i] != 0) {
            return false;
        }
    }
    return true;
}

static void spib_handle_rx(const uint8_t *rx, size_t len)
{
    ps_spi_view_t view;
    int rc = (len >= PS_SPI_HDR_SIZE) ? ps_spi_frame_parse(rx, len, &view)
                                      : PS_SPI_ESHORT;
    if (rc != PS_SPI_OK) {
        if (!spib_rx_is_idle(rx, len)) {
            portENTER_CRITICAL(&s_lock);
            s_stats.crc_errors++; /* all downlink integrity failures */
            portEXIT_CRITICAL(&s_lock);
        }
        return;
    }

    portENTER_CRITICAL(&s_lock);
    if (s_have_rx_seq && view.seq != (uint8_t)(s_rx_seq + 1u)) {
        s_stats.seq_gaps++;
    }
    s_rx_seq = view.seq;
    s_have_rx_seq = true;
    portEXIT_CRITICAL(&s_lock);

    for (size_t i = 0; i < view.count; i++) {
        ps_can_record_t rec;
        if (ps_spi_view_record(&view, i, &rec) != PS_SPI_OK) {
            portENTER_CRITICAL(&s_lock);
            s_stats.crc_errors++;
            portEXIT_CRITICAL(&s_lock);
            continue;
        }
        if (xQueueSend(s_downlink_q, &rec, 0) != pdTRUE) {
            /* Drop-oldest: a stale command is worth less than a fresh one. */
            ps_can_record_t scrap;
            (void)xQueueReceive(s_downlink_q, &scrap, 0);
            (void)xQueueSend(s_downlink_q, &rec, 0);
            s_downlink_drops++;
            ESP_LOGD(TAG, "downlink drop-oldest (%u)", (unsigned)s_downlink_drops);
        }
        portENTER_CRITICAL(&s_lock);
        s_stats.downlink_records++;
        portEXIT_CRITICAL(&s_lock);
    }
}

static void spib_build_tx(uint8_t *buf)
{
    ps_can_record_t recs[PS_SPI_MAX_RECORDS];
    size_t n = 0;
    while (n < PS_SPI_MAX_RECORDS &&
           xQueueReceive(s_uplink_q, &recs[n], 0) == pdTRUE) {
        n++;
    }

    uint8_t flags;
    portENTER_CRITICAL(&s_lock);
    flags = s_sticky_flags;
    if (s_overflow_pending) {
        flags |= PS_SPIF_OVERFLOW;
        s_overflow_pending = false; /* reported once, re-armed by the next drop */
    }
    portEXIT_CRITICAL(&s_lock);
    if (uxQueueMessagesWaiting(s_uplink_q) > 0) {
        flags |= PS_SPIF_MORE_PENDING;
    }

    portENTER_CRITICAL(&s_lock);
    uint8_t seq = s_tx_seq++;
    portEXIT_CRITICAL(&s_lock);
    (void)ps_spi_frame_build(buf, flags, seq, recs, n); /* n <= 31 by loop bound */
}

static void spib_requeue(spib_slot_t *slot)
{
    slot->trans.length = PS_SPI_XFER_SIZE * 8; /* bits; 512 B = multiple of 4 B */
    slot->trans.trans_len = 0;
    ESP_ERROR_CHECK(spi_slave_queue_trans(SPIB_HOST, &slot->trans, portMAX_DELAY));
}

static void spib_pump_task(void *arg)
{
    (void)arg;
    for (;;) {
        spi_slave_transaction_t *done = NULL;
        if (spi_slave_get_trans_result(SPIB_HOST, &done, portMAX_DELAY) != ESP_OK ||
            done == NULL) {
            continue;
        }
        spib_slot_t *slot = (spib_slot_t *)done->user;

        portENTER_CRITICAL(&s_lock);
        s_stats.transactions++;
        portEXIT_CRITICAL(&s_lock);

        /* trans_len = bits the master actually clocked this transaction. */
        spib_handle_rx(slot->rx, done->trans_len / 8);
        spib_build_tx(slot->tx);
        /* The sibling slot stayed armed the whole time (ping-pong invariant). */
        spib_requeue(slot);
        spib_update_data_ready();
    }
}

esp_err_t ps_spib_start(const ps_spib_config_t *cfg)
{
    ESP_RETURN_ON_FALSE(cfg != NULL, ESP_ERR_INVALID_ARG, TAG, "cfg NULL");
    ESP_RETURN_ON_FALSE(!s_started, ESP_ERR_INVALID_STATE, TAG, "already started");

    size_t up_len = cfg->uplink_queue_len ? cfg->uplink_queue_len : SPIB_DEF_UPLINK_LEN;
    size_t dn_len = cfg->downlink_queue_len ? cfg->downlink_queue_len : SPIB_DEF_DOWNLINK_LEN;
    s_uplink_q = xQueueCreate(up_len, sizeof(ps_can_record_t));
    s_downlink_q = xQueueCreate(dn_len, sizeof(ps_can_record_t));
    ESP_RETURN_ON_FALSE(s_uplink_q && s_downlink_q, ESP_ERR_NO_MEM, TAG, "queues");

    s_data_ready_gpio = cfg->data_ready_gpio;
    gpio_config_t drdy = {
        .pin_bit_mask = 1ULL << cfg->data_ready_gpio,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&drdy), TAG, "data_ready gpio");
    gpio_set_level((gpio_num_t)cfg->data_ready_gpio, 0);

    spi_bus_config_t buscfg = {
        .mosi_io_num = cfg->mosi_gpio,
        .miso_io_num = cfg->miso_gpio,
        .sclk_io_num = cfg->sclk_gpio,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = PS_SPI_XFER_SIZE,
    };
    spi_slave_interface_config_t slvcfg = {
        .spics_io_num = cfg->cs_gpio,
        .flags = 0,
        .queue_size = 2,          /* exactly the two ping-pong slots */
        .mode = 0,                /* CPOL=0 CPHA=0, Pi 5 spidev default */
        .post_setup_cb = NULL,
        .post_trans_cb = NULL,
    };
    ESP_RETURN_ON_ERROR(spi_slave_initialize(SPIB_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO),
                        TAG, "spi_slave_initialize");

    for (int i = 0; i < 2; i++) {
        s_slot[i].tx = s_txbuf[i];
        s_slot[i].rx = s_rxbuf[i];
        memset(&s_slot[i].trans, 0, sizeof(s_slot[i].trans));
        s_slot[i].trans.tx_buffer = s_slot[i].tx;
        s_slot[i].trans.rx_buffer = s_slot[i].rx;
        s_slot[i].trans.user = &s_slot[i];
        spib_build_tx(s_slot[i].tx); /* empty frame: valid header, count 0 */
        spib_requeue(&s_slot[i]);
    }

    s_started = true;
    BaseType_t ok = xTaskCreatePinnedToCore(spib_pump_task, "ps_spib", SPIB_TASK_STACK,
                                            NULL, SPIB_TASK_PRIO, NULL, SPIB_TASK_CORE);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_ERR_NO_MEM, TAG, "pump task");
    ESP_LOGI(TAG, "SPI2 slave up: mosi=%d miso=%d sclk=%d cs=%d drdy=%d",
             cfg->mosi_gpio, cfg->miso_gpio, cfg->sclk_gpio, cfg->cs_gpio,
             cfg->data_ready_gpio);
    return ESP_OK;
}

bool ps_spib_uplink_push(const ps_can_record_t *rec)
{
    if (!s_started || rec == NULL) {
        return false;
    }
    if (xQueueSend(s_uplink_q, rec, 0) != pdTRUE) {
        portENTER_CRITICAL(&s_lock);
        s_overflow_pending = true;
        s_stats.uplink_overflows++;
        portEXIT_CRITICAL(&s_lock);
        return false;
    }
    gpio_set_level((gpio_num_t)s_data_ready_gpio, 1);
    return true;
}

bool ps_spib_downlink_pop(ps_can_record_t *rec, TickType_t timeout)
{
    if (!s_started || rec == NULL) {
        return false;
    }
    return xQueueReceive(s_downlink_q, rec, timeout) == pdTRUE;
}

void ps_spib_set_flag(uint8_t flag, bool on)
{
    flag &= (uint8_t)(PS_SPIF_ESTOP_LATCHED | PS_SPIF_HUB_FAULT); /* sticky bits only */
    portENTER_CRITICAL(&s_lock);
    if (on) {
        s_sticky_flags |= flag;
    } else {
        s_sticky_flags &= (uint8_t)~flag;
    }
    portEXIT_CRITICAL(&s_lock);
}

void ps_spib_get_stats(ps_spib_stats_t *out)
{
    if (out == NULL) {
        return;
    }
    portENTER_CRITICAL(&s_lock);
    *out = s_stats;
    portEXIT_CRITICAL(&s_lock);
}
