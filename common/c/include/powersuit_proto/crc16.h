/* CRC16-CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no xorout).
 * Known-answer: "123456789" -> 0x29B1. Used by the SPI bridge framing. */
#ifndef POWERSUIT_PROTO_CRC16_H
#define POWERSUIT_PROTO_CRC16_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

uint16_t ps_crc16(const uint8_t *data, size_t len);
uint16_t ps_crc16_update(uint16_t crc, const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* POWERSUIT_PROTO_CRC16_H */
