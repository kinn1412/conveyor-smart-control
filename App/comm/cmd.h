#ifndef CMD_H
#define CMD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "frame.h"
#include "proto.h"

typedef struct { /* CmdQueue item */
    uint8_t cmd;    /* PROTO_CMD_* */
    uint8_t seq;
    union {
        uint8_t enable;
        int16_t vel_mms;
        int32_t dist_um;
        struct {
            uint16_t v_max_mms, a_max_mms2;
        } limits;
        int16_t angle_ddeg;
        struct {
            float kp_v, ki_v, kp_p, u_dz;
        } gains;
        uint8_t telem_hz;
        int16_t duty_permille;
    } arg;
} cmd_t;

typedef struct { /* snapshot filled by CommRxTask before each call */
    uint8_t state;
    bool openloop;   /* RUNNING in OPENLOOP mode */
    bool queue_full; /* CmdQueue has no free slot */
    proto_telem_t telem;
    proto_diag_t diag;
} cmd_view_t;

typedef struct { /* duplicate filter, REQ-P07 */
    bool valid;
    uint8_t last_seq, last_cmd;
    uint8_t reply_len;
    uint8_t reply[PROTO_FRAME_MAX];
} cmd_ctx_t;

typedef enum { CMD_ACT_NONE, CMD_ACT_QUEUE, CMD_ACT_ESTOP, CMD_ACT_DIAG_RESET } cmd_action_t;

void cmd_init(cmd_ctx_t *ctx);
/* State rule of spec §6.4. Also used by ControlTask to re-check dequeued commands. */
bool cmd_allowed(uint8_t cmd, uint8_t state, bool openloop);
/* Validates req, builds the reply frame in reply[PROTO_FRAME_MAX], fills *out for
   CMD_ACT_QUEUE. */
cmd_action_t cmd_handle(cmd_ctx_t *ctx, const frame_t *req, const cmd_view_t *view, cmd_t *out,
                        uint8_t *reply, size_t *reply_len);

#endif
