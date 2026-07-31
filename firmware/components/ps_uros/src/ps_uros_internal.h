/* Shared internals between the ps_uros transport and session task. Not public:
 * apps use ps_uros.h. */
#pragma once

#include "ps_uros.h"

#ifndef PS_UROS_ENABLED
#define PS_UROS_ENABLED 0
#endif

#if PS_UROS_ENABLED

#include <uxr/client/profile/transport/custom/custom_transport.h>

esp_err_t ps_uros_transport_init(ps_can_handle_t can, uint8_t node_id, size_t ring_bytes);
void ps_uros_transport_log_stats(void);

bool ps_uros_open(struct uxrCustomTransport *transport);
bool ps_uros_close(struct uxrCustomTransport *transport);
size_t ps_uros_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len,
                     uint8_t *err);
size_t ps_uros_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout,
                    uint8_t *err);

#endif /* PS_UROS_ENABLED */
