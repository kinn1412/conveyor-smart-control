#include "frame.h"

enum { ST_SOF0, ST_SOF1, ST_LEN, ST_SEQ, ST_CMD, ST_PAYLOAD, ST_CRC_L, ST_CRC_H };

void frame_parser_init(frame_parser_t *p)
{
    memset(p, 0, sizeof *p);
}

bool frame_parse_byte(frame_parser_t *p, uint8_t b, uint32_t now_ms, frame_t *out)
{
    if (p->state != ST_SOF0 && now_ms - p->last_ms > PROTO_GAP_MS) {
        p->err_timeout++;
        p->state = ST_SOF0;
    }
    p->last_ms = now_ms;

    switch (p->state) {
    case ST_SOF0:
        if (b == PROTO_SOF0)
            p->state = ST_SOF1;
        break;
    case ST_SOF1:
        if (b == PROTO_SOF1)
            p->state = ST_LEN;
        else if (b != PROTO_SOF0)
            p->state = ST_SOF0;
        break;
    case ST_LEN:
        if (b > PROTO_LEN_MAX) {
            p->err_len++;
            p->state = ST_SOF0;
            break;
        }
        p->buf[0] = b;
        p->state = ST_SEQ;
        break;
    case ST_SEQ:
        p->buf[1] = b;
        p->state = ST_CMD;
        break;
    case ST_CMD:
        p->buf[2] = b;
        p->idx = 3;
        p->state = p->buf[0] ? ST_PAYLOAD : ST_CRC_L;
        break;
    case ST_PAYLOAD:
        p->buf[p->idx++] = b;
        if (p->idx == 3 + p->buf[0])
            p->state = ST_CRC_L;
        break;
    case ST_CRC_L:
        p->crc_rx = b;
        p->state = ST_CRC_H;
        break;
    case ST_CRC_H:
        p->crc_rx |= (uint16_t)(b << 8);
        p->state = ST_SOF0;
        if (p->crc_rx != proto_crc16(p->buf, 3u + p->buf[0])) {
            p->err_crc++;
            break;
        }
        out->len = p->buf[0];
        out->seq = p->buf[1];
        out->cmd = p->buf[2];
        memcpy(out->payload, &p->buf[3], out->len);
        return true;
    }
    return false;
}

size_t frame_build(uint8_t *buf, uint8_t seq, uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    buf[0] = PROTO_SOF0;
    buf[1] = PROTO_SOF1;
    buf[2] = len;
    buf[3] = seq;
    buf[4] = cmd;
    if (len)
        memcpy(&buf[5], payload, len);
    proto_put_u16(&buf[5 + len], proto_crc16(&buf[2], 3u + len));
    return PROTO_HDR_LEN + len + 2u;
}
