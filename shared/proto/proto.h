#ifndef PROTO_H
#define PROTO_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define PROTO_VERSION 2u
#define PROTO_SOF0 0xAAu
#define PROTO_SOF1 0x55u
#define PROTO_LEN_MAX 32u
#define PROTO_HDR_LEN 5u /* SOF0 SOF1 LEN SEQ CMD */
#define PROTO_FRAME_MAX 39u
#define PROTO_GAP_MS 5u
#define PROTO_ACK_FLAG 0x80u
#define PROTO_TELEM_LEN 22u
#define PROTO_DIAG_LEN 20u

enum {
    PROTO_CMD_PING = 0x01,
    PROTO_CMD_GET_STATUS = 0x02,
    PROTO_CMD_GET_DIAG = 0x03,
    PROTO_CMD_ENABLE = 0x10,
    PROTO_CMD_SET_VEL = 0x11,
    PROTO_CMD_MOVE_REL = 0x12,
    PROTO_CMD_STOP = 0x13,
    PROTO_CMD_ESTOP = 0x14,
    PROTO_CMD_CLEAR_FAULT = 0x15,
    PROTO_CMD_SET_LIMITS = 0x16,
    PROTO_CMD_SERVO_SET = 0x20,
    PROTO_CMD_SET_GAINS = 0x30,
    PROTO_CMD_SET_TELEM = 0x31,
    PROTO_CMD_OPENLOOP_DUTY = 0x40,
    PROTO_EVT_MOVE_DONE = 0xE0,
    PROTO_EVT_FAULT = 0xE1,
    PROTO_TELEM = 0xF0,
};

enum {
    PROTO_ST_OK,
    PROTO_ST_BAD_LEN,
    PROTO_ST_BAD_STATE,
    PROTO_ST_OUT_OF_RANGE,
    PROTO_ST_UNKNOWN_CMD,
};

enum {
    PROTO_STATE_BOOT,
    PROTO_STATE_IDLE,
    PROTO_STATE_READY,
    PROTO_STATE_RUNNING,
    PROTO_STATE_FAULT,
};

enum {
    PROTO_FAULT_WATCHDOG = 1u << 0,
    PROTO_FAULT_ESTOP = 1u << 1,
    PROTO_FAULT_STALL = 1u << 2, /* also a dead encoder */
};

/* CRC16-CCITT: poly 0x1021, init 0xFFFF, no reflection, xorout 0 */
static inline uint16_t proto_crc16_update(uint16_t crc, uint8_t b)
{
    crc ^= (uint16_t)(b << 8);
    for (int i = 0; i < 8; i++)
        crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    return crc;
}

static inline uint16_t proto_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; i++)
        crc = proto_crc16_update(crc, data[i]);
    return crc;
}

static inline void proto_put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static inline void proto_put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static inline void proto_put_i16(uint8_t *p, int16_t v) { proto_put_u16(p, (uint16_t)v); }
static inline void proto_put_i32(uint8_t *p, int32_t v) { proto_put_u32(p, (uint32_t)v); }

static inline void proto_put_f32(uint8_t *p, float v)
{
    uint32_t u;
    memcpy(&u, &v, sizeof u);
    proto_put_u32(p, u);
}

static inline uint16_t proto_get_u16(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

static inline uint32_t proto_get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static inline int16_t proto_get_i16(const uint8_t *p) { return (int16_t)proto_get_u16(p); }
static inline int32_t proto_get_i32(const uint8_t *p) { return (int32_t)proto_get_u32(p); }

static inline float proto_get_f32(const uint8_t *p)
{
    uint32_t u = proto_get_u32(p);
    float v;
    memcpy(&v, &u, sizeof v);
    return v;
}

typedef struct { /* spec §6.7 */
    uint32_t t_ms;
    uint8_t state, fault_bits;
    int32_t pos_um;
    int16_t vel_dmms, vel_ref_dmms;
    int32_t pos_ref_um;
    int16_t duty_permille;
    uint16_t missed_cycles;
} proto_telem_t;

typedef struct { /* spec §6.8 */
    uint16_t period_min_us, period_max_us, exec_max_us, estop_latency_max_us;
    uint16_t stack_free_ctrl, stack_free_rx, stack_free_tx;
    uint16_t err_crc, err_len, err_timeout;
} proto_diag_t;

static inline void proto_pack_telem(uint8_t *buf, const proto_telem_t *t)
{
    proto_put_u32(&buf[0], t->t_ms);
    buf[4] = t->state;
    buf[5] = t->fault_bits;
    proto_put_i32(&buf[6], t->pos_um);
    proto_put_i16(&buf[10], t->vel_dmms);
    proto_put_i16(&buf[12], t->vel_ref_dmms);
    proto_put_i32(&buf[14], t->pos_ref_um);
    proto_put_i16(&buf[18], t->duty_permille);
    proto_put_u16(&buf[20], t->missed_cycles);
}

static inline void proto_pack_diag(uint8_t *buf, const proto_diag_t *d)
{
    proto_put_u16(&buf[0], d->period_min_us);
    proto_put_u16(&buf[2], d->period_max_us);
    proto_put_u16(&buf[4], d->exec_max_us);
    proto_put_u16(&buf[6], d->estop_latency_max_us);
    proto_put_u16(&buf[8], d->stack_free_ctrl);
    proto_put_u16(&buf[10], d->stack_free_rx);
    proto_put_u16(&buf[12], d->stack_free_tx);
    proto_put_u16(&buf[14], d->err_crc);
    proto_put_u16(&buf[16], d->err_len);
    proto_put_u16(&buf[18], d->err_timeout);
}

#endif
