#ifndef FRAME_H
#define FRAME_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "proto.h"

typedef struct {
    uint8_t seq, cmd, len;
    uint8_t payload[PROTO_LEN_MAX];
} frame_t;

typedef struct {
    uint8_t state, idx;
    uint8_t buf[3 + PROTO_LEN_MAX]; /* LEN SEQ CMD PAYLOAD, the CRC input */
    uint16_t crc_rx;
    uint32_t last_ms;
    uint16_t err_crc, err_len, err_timeout;
} frame_parser_t;

void frame_parser_init(frame_parser_t *p);
/* Returns true and fills *out when a valid frame completes. A gap > PROTO_GAP_MS inside a
   frame drops it, counts err_timeout, then treats b as a new byte. */
bool frame_parse_byte(frame_parser_t *p, uint8_t b, uint32_t now_ms, frame_t *out);
/* buf must hold PROTO_FRAME_MAX bytes. len <= PROTO_LEN_MAX. Returns the frame length. */
size_t frame_build(uint8_t *buf, uint8_t seq, uint8_t cmd, const uint8_t *payload, uint8_t len);

#endif
