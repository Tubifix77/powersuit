/* Powersuit CAN identifier layout — normative reference: docs/network-map.md §2.
 * 29-bit extended ID = class(3) | src(5) | dst(5) | type(8) | low(8), class in the
 * top bits so SAFETY wins arbitration. Pure C99, no dependencies; mirrored by
 * common/python/powersuit_proto/can_id.py and locked by shared tests. */
#ifndef POWERSUIT_PROTO_CAN_ID_H
#define POWERSUIT_PROTO_CAN_ID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Message classes (arbitration priority: lower value wins). */
enum {
    PS_CLS_SAFETY  = 0,
    PS_CLS_CONTROL = 1,
    PS_CLS_TELEM   = 2,
    PS_CLS_XRCE    = 3,
    PS_CLS_AUDIO   = 4,
    PS_CLS_MGMT    = 5,
};

/* Node registry. 0 is only valid as broadcast destination. */
enum {
    PS_NODE_BROADCAST = 0,
    PS_NODE_ARM_R     = 1,
    PS_NODE_ARM_L     = 2,
    PS_NODE_LEG_R     = 3,
    PS_NODE_LEG_L     = 4,
    PS_NODE_FLIGHT    = 5,
    PS_NODE_HELMET    = 6,
    PS_NODE_HUB       = 7,
    PS_NODE_ORCH      = 8,
};

/* Message types, grouped by class. */
enum {
    /* SAFETY */
    PS_T_HEARTBEAT   = 0x01,
    PS_T_ESTOP       = 0x02,
    PS_T_CLEAR_ESTOP = 0x03,
    PS_T_NODE_FAULT  = 0x04,
    /* CONTROL */
    PS_T_JOINT_CMD   = 0x10,
    PS_T_FLAP_CMD    = 0x11,
    PS_T_MODE_SET    = 0x12,
    PS_T_LED_PATTERN = 0x13,
    /* TELEM */
    PS_T_JOINT_STATE = 0x20,
    PS_T_IMU_QUAT    = 0x21,
    PS_T_IMU_ACC     = 0x22,
    PS_T_IMU_GYR     = 0x23,
    PS_T_FORCE       = 0x24,
    PS_T_BMS_SUMMARY = 0x25,
    PS_T_BMS_CELLS   = 0x26,
    PS_T_AERO_STATE  = 0x27,
    PS_T_FLAP_STATE  = 0x28,
    PS_T_ENV         = 0x29,
    PS_T_NODE_STATS  = 0x2A,
    /* XRCE */
    PS_T_XRCE_STREAM = 0x30,
    /* AUDIO */
    PS_T_AUDIO_DOWN  = 0x40,
    PS_T_AUDIO_UP    = 0x41,
    PS_T_AUDIO_SYNC  = 0x42,
    PS_T_AUDIO_CTL   = 0x43,
    /* MGMT */
    PS_T_TIME_SYNC   = 0x50,
    PS_T_FLOW_CTL    = 0x51,
    PS_T_STATS_REQ   = 0x52,
    PS_T_LOG         = 0x53,
    PS_T_VERSION     = 0x54,
};

typedef struct {
    uint8_t cls;   /* 0..7  */
    uint8_t src;   /* 0..31 */
    uint8_t dst;   /* 0..31, 0 = broadcast */
    uint8_t type;  /* 0..255 */
    uint8_t low;   /* per-class: seq (TELEM/AUDIO), counter (SAFETY), else 0 */
} ps_can_id_t;

#define PS_CAN_ID_MASK 0x1FFFFFFFu

static inline uint32_t ps_can_id_pack(uint8_t cls, uint8_t src, uint8_t dst,
                                      uint8_t type, uint8_t low)
{
    return (((uint32_t)cls & 0x7u) << 26) |
           (((uint32_t)src & 0x1Fu) << 21) |
           (((uint32_t)dst & 0x1Fu) << 16) |
           (((uint32_t)type) << 8) |
           ((uint32_t)low);
}

static inline void ps_can_id_unpack(uint32_t id, ps_can_id_t *out)
{
    out->cls  = (uint8_t)((id >> 26) & 0x7u);
    out->src  = (uint8_t)((id >> 21) & 0x1Fu);
    out->dst  = (uint8_t)((id >> 16) & 0x1Fu);
    out->type = (uint8_t)((id >> 8) & 0xFFu);
    out->low  = (uint8_t)(id & 0xFFu);
}

static inline uint8_t ps_can_id_cls(uint32_t id)  { return (uint8_t)((id >> 26) & 0x7u); }
static inline uint8_t ps_can_id_src(uint32_t id)  { return (uint8_t)((id >> 21) & 0x1Fu); }
static inline uint8_t ps_can_id_dst(uint32_t id)  { return (uint8_t)((id >> 16) & 0x1Fu); }
static inline uint8_t ps_can_id_type(uint32_t id) { return (uint8_t)((id >> 8) & 0xFFu); }
static inline uint8_t ps_can_id_low(uint32_t id)  { return (uint8_t)(id & 0xFFu); }

#ifdef __cplusplus
}
#endif

#endif /* POWERSUIT_PROTO_CAN_ID_H */
