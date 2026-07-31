/* Powersuit wire payload structs — normative reference: docs/network-map.md §3.
 * All payloads are little-endian, packed, ≤ 8 bytes (classic CAN). Both ESP32 targets
 * and all supported hosts are little-endian; test_wire asserts this at runtime.
 * Mirrored by common/python/powersuit_proto/wire.py. */
#ifndef POWERSUIT_PROTO_WIRE_H
#define POWERSUIT_PROTO_WIRE_H

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__GNUC__)
#define PS_PACKED __attribute__((packed))
#else
#error "powersuit_proto requires a GCC-compatible compiler (packed structs)"
#endif

/* --- Contract constants (docs/safety.md §2) --------------------------------- */
#define PS_HEARTBEAT_PERIOD_MS   10u
#define PS_HEARTBEAT_TIMEOUT_MS  50u
#define PS_REARM_WINDOW_MS       250u
#define PS_CMD_STALE_MS          200u
#define PS_ESTOP_REARM_MS        1000u
#define PS_ESTOP_REPEAT          3u
#define PS_CLEAR_ESTOP_MAGIC     0x52A4C13Au

/* Node safety states. */
enum {
    PS_STATE_BOOT        = 0,
    PS_STATE_STANDBY     = 1,
    PS_STATE_OPERATIONAL = 2,
    PS_STATE_PASSIVE     = 3,
    PS_STATE_ESTOP       = 4,
    PS_STATE_FAULT       = 5,
};

/* ESTOP causes. */
enum {
    PS_ESTOP_BMS_SHORT     = 1,
    PS_ESTOP_BMS_OVERVOLT  = 2,
    PS_ESTOP_BMS_UNDERVOLT = 3,
    PS_ESTOP_OPERATOR      = 4,
    PS_ESTOP_THERMAL       = 5,
    PS_ESTOP_COMM_LOSS     = 6,
    PS_ESTOP_SOFTWARE      = 7,
};

/* JOINT_CMD modes. */
enum {
    PS_JMODE_PASSIVE   = 0,
    PS_JMODE_POSITION  = 1,
    PS_JMODE_VELOCITY  = 2,
    PS_JMODE_TORQUE    = 3,
    PS_JMODE_IMPEDANCE = 4,
};

/* HEARTBEAT flag bits / BMS fault bits. */
#define PS_HB_ESTOP_LATCHED (1u << 0)
#define PS_HB_DEGRADED      (1u << 1)
#define PS_HB_CLOUD_UP      (1u << 2)

#define PS_BMSF_SHORT_LATCH   (1u << 0)
#define PS_BMSF_OV            (1u << 1)
#define PS_BMSF_UV            (1u << 2)
#define PS_BMSF_OT            (1u << 3)
#define PS_BMSF_UT            (1u << 4)
#define PS_BMSF_OC_CHARGE     (1u << 5)
#define PS_BMSF_OC_DISCHARGE  (1u << 6)
#define PS_BMSF_COMP_ARMED    (1u << 7)

/* --- SAFETY payloads --------------------------------------------------------- */
typedef struct PS_PACKED {
    uint16_t seq;
    uint8_t  flags;      /* PS_HB_* */
    uint8_t  src_state;  /* PS_STATE_* */
    uint32_t uptime_ms;
} ps_heartbeat_t;

typedef struct PS_PACKED {
    uint8_t  cause;       /* PS_ESTOP_* */
    uint8_t  origin_node;
    uint16_t seq;
    uint32_t uptime_ms;
} ps_estop_t;

typedef struct PS_PACKED {
    uint32_t magic;    /* PS_CLEAR_ESTOP_MAGIC */
    uint32_t counter;  /* strictly greater than last accepted */
} ps_clear_estop_t;

typedef struct PS_PACKED {
    uint8_t  fault_code;
    uint8_t  severity;   /* 0 info, 1 warn, 2 error, 3 critical */
    uint16_t detail;
    uint32_t uptime_ms;
} ps_node_fault_t;

/* --- CONTROL payloads --------------------------------------------------------- */
typedef struct PS_PACKED {
    uint8_t joint;      /* local joint index */
    uint8_t mode;       /* PS_JMODE_* */
    int16_t pos_crad;   /* 0.01 rad */
    int16_t vel_crad_s; /* 0.01 rad/s */
    int16_t eff_cNm;    /* 0.01 N*m */
} ps_joint_cmd_t;

typedef struct PS_PACKED {
    uint8_t  flap;
    uint8_t  rate_lim;  /* %/s, 0 = board default */
    int16_t  pos_pm;    /* permille, -1000..1000 */
    uint16_t flags;     /* bit0 brake_mode */
    uint16_t rsvd;
} ps_flap_cmd_t;

typedef struct PS_PACKED {
    uint8_t target_state; /* PS_STATE_* */
    uint8_t rsvd[7];
} ps_mode_set_t;

typedef struct PS_PACKED {
    uint8_t  pattern;
    uint8_t  brightness;
    uint8_t  r, g, b;
    uint8_t  speed;
    uint16_t rsvd;
} ps_led_pattern_t;

/* --- TELEM payloads ------------------------------------------------------------ */
typedef struct PS_PACKED {
    uint8_t joint;
    uint8_t flags;      /* bit0 saturated, bit1 passive */
    int16_t pos_crad;
    int16_t vel_crad_s;
    int16_t eff_cNm;
} ps_joint_state_t;

typedef struct PS_PACKED { int16_t qw, qx, qy, qz; } ps_imu_quat_t;            /* Q15 */
typedef struct PS_PACKED { int16_t ax, ay, az; uint16_t rsvd; } ps_imu_acc_t;  /* mg  */
typedef struct PS_PACKED { int16_t gx, gy, gz; uint16_t rsvd; } ps_imu_gyr_t;  /* 0.1 dps */
typedef struct PS_PACKED { int16_t ch[4]; } ps_force_t;                        /* cN  */

typedef struct PS_PACKED {
    uint16_t pack_cV;
    int16_t  current_cA;
    uint8_t  soc_pct;
    int8_t   temp_max_C;
    uint16_t fault_bits;  /* PS_BMSF_* */
} ps_bms_summary_t;

typedef struct PS_PACKED {
    uint8_t  group;
    uint8_t  rsvd;
    uint16_t mv[3];
} ps_bms_cells_t;

typedef struct PS_PACKED {
    uint16_t ias_cms;   /* indicated airspeed, cm/s */
    uint16_t q_pa;      /* dynamic pressure, Pa */
    int16_t  aoa_cdeg;  /* angle of attack, 0.01 deg */
    uint16_t flags;
} ps_aero_state_t;

typedef struct PS_PACKED {
    uint8_t  flap;
    uint8_t  flags;     /* bit0 at_limit, bit1 fault */
    int16_t  pos_pm;
    int16_t  target_pm;
    uint16_t rsvd;
} ps_flap_state_t;

typedef struct PS_PACKED {
    int16_t  temp_cC;
    uint16_t rh_pm;       /* relative humidity, permille */
    uint16_t press_dhPa;  /* pressure, 0.1 hPa */
    uint16_t rsvd;
} ps_env_t;

typedef struct PS_PACKED {
    uint8_t  cpu_pct;
    uint8_t  state;
    uint16_t rx_fps;
    uint16_t tx_fps;
    uint16_t err_cnt;
} ps_node_stats_t;

/* --- AUDIO payloads (frames themselves are raw ADPCM bytes) --------------------- */
typedef struct PS_PACKED {
    uint8_t  dir;        /* 0 down, 1 up */
    uint8_t  step_index;
    int16_t  predictor;
    uint16_t frame_seq;
    uint16_t rsvd;
} ps_audio_sync_t;

typedef struct PS_PACKED {
    uint8_t  dir;
    uint8_t  cmd;          /* 0 stop, 1 start, 2 rate */
    uint16_t sample_rate;  /* Hz */
    uint32_t rsvd;
} ps_audio_ctl_t;

/* --- MGMT payloads --------------------------------------------------------------- */
typedef struct PS_PACKED {
    uint32_t epoch_ms_lo;  /* lower 32 bits of Unix ms */
    uint16_t seq;
    uint16_t rsvd;
} ps_time_sync_t;

typedef struct PS_PACKED {
    uint8_t  plane;   /* class value */
    uint8_t  level;   /* 0 normal, 1 reduce, 2 pause */
    uint16_t rsvd;
    uint32_t rsvd2;
} ps_flow_ctl_t;

typedef struct PS_PACKED {
    uint8_t  major, minor, patch;
    uint8_t  node_state;
    uint32_t git_short;
} ps_version_t;

/* --- Size locks (classic CAN payload limit) ------------------------------------- */
#define PS_WIRE_SIZE_ASSERT(t) _Static_assert(sizeof(t) <= 8, #t " exceeds 8 bytes")
PS_WIRE_SIZE_ASSERT(ps_heartbeat_t);
PS_WIRE_SIZE_ASSERT(ps_estop_t);
PS_WIRE_SIZE_ASSERT(ps_clear_estop_t);
PS_WIRE_SIZE_ASSERT(ps_node_fault_t);
PS_WIRE_SIZE_ASSERT(ps_joint_cmd_t);
PS_WIRE_SIZE_ASSERT(ps_flap_cmd_t);
PS_WIRE_SIZE_ASSERT(ps_mode_set_t);
PS_WIRE_SIZE_ASSERT(ps_led_pattern_t);
PS_WIRE_SIZE_ASSERT(ps_joint_state_t);
PS_WIRE_SIZE_ASSERT(ps_imu_quat_t);
PS_WIRE_SIZE_ASSERT(ps_imu_acc_t);
PS_WIRE_SIZE_ASSERT(ps_imu_gyr_t);
PS_WIRE_SIZE_ASSERT(ps_force_t);
PS_WIRE_SIZE_ASSERT(ps_bms_summary_t);
PS_WIRE_SIZE_ASSERT(ps_bms_cells_t);
PS_WIRE_SIZE_ASSERT(ps_aero_state_t);
PS_WIRE_SIZE_ASSERT(ps_flap_state_t);
PS_WIRE_SIZE_ASSERT(ps_env_t);
PS_WIRE_SIZE_ASSERT(ps_node_stats_t);
PS_WIRE_SIZE_ASSERT(ps_audio_sync_t);
PS_WIRE_SIZE_ASSERT(ps_audio_ctl_t);
PS_WIRE_SIZE_ASSERT(ps_time_sync_t);
PS_WIRE_SIZE_ASSERT(ps_flow_ctl_t);
PS_WIRE_SIZE_ASSERT(ps_version_t);

/* Copy helpers: CAN/SPI buffers are byte-aligned; never cast, always memcpy. */
#define PS_WIRE_WRITE(buf, s)  memcpy((buf), &(s), sizeof(s))
#define PS_WIRE_READ(s, buf)   memcpy(&(s), (buf), sizeof(s))

#ifdef __cplusplus
}
#endif

#endif /* POWERSUIT_PROTO_WIRE_H */
