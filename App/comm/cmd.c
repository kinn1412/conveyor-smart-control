#include "cmd.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"

void cmd_init(cmd_ctx_t *ctx)
{
    memset(ctx, 0, sizeof *ctx);
}

static int payload_len(uint8_t cmd)
{
    switch (cmd) {
    case PROTO_CMD_PING:
    case PROTO_CMD_GET_STATUS:
    case PROTO_CMD_GET_DIAG:
    case PROTO_CMD_STOP:
    case PROTO_CMD_ESTOP:
    case PROTO_CMD_CLEAR_FAULT:
        return 0;
    case PROTO_CMD_ENABLE:
    case PROTO_CMD_SET_TELEM:
        return 1;
    case PROTO_CMD_SET_VEL:
    case PROTO_CMD_SERVO_SET:
    case PROTO_CMD_OPENLOOP_DUTY:
        return 2;
    case PROTO_CMD_MOVE_REL:
    case PROTO_CMD_SET_LIMITS:
        return 4;
    case PROTO_CMD_SET_GAINS:
        return 16;
    default:
        return -1;
    }
}

bool cmd_allowed(uint8_t cmd, uint8_t state, bool openloop)
{
    switch (cmd) {
    case PROTO_CMD_ENABLE:
    case PROTO_CMD_SET_LIMITS:
    case PROTO_CMD_SET_GAINS:
        return state == PROTO_STATE_IDLE || state == PROTO_STATE_READY;
    case PROTO_CMD_SET_VEL:
        return state == PROTO_STATE_READY || state == PROTO_STATE_RUNNING;
    case PROTO_CMD_MOVE_REL:
        return state == PROTO_STATE_READY;
    case PROTO_CMD_CLEAR_FAULT:
        return state == PROTO_STATE_FAULT;
    case PROTO_CMD_SERVO_SET:
        return state != PROTO_STATE_FAULT;
    case PROTO_CMD_OPENLOOP_DUTY:
        return state == PROTO_STATE_READY || (state == PROTO_STATE_RUNNING && openloop);
    default:
        return true;
    }
}

/* Decodes the payload into out->arg and checks the ranges of spec §6.4 */
static uint8_t parse_args(const frame_t *req, cmd_t *out)
{
    const uint8_t *p = req->payload;
    switch (req->cmd) {
    case PROTO_CMD_ENABLE:
        out->arg.enable = p[0];
        return p[0] <= 1 ? PROTO_ST_OK : PROTO_ST_OUT_OF_RANGE;
    case PROTO_CMD_SET_VEL:
        out->arg.vel_mms = proto_get_i16(p);
        return abs(out->arg.vel_mms) <= V_MAX_MMS ? PROTO_ST_OK : PROTO_ST_OUT_OF_RANGE;
    case PROTO_CMD_MOVE_REL:
        out->arg.dist_um = proto_get_i32(p);
        return out->arg.dist_um >= -MOVE_MAX_UM && out->arg.dist_um <= MOVE_MAX_UM
                   ? PROTO_ST_OK
                   : PROTO_ST_OUT_OF_RANGE;
    case PROTO_CMD_SET_LIMITS:
        out->arg.limits.v_max_mms = proto_get_u16(p);
        out->arg.limits.a_max_mms2 = proto_get_u16(p + 2);
        return out->arg.limits.v_max_mms >= 1 && out->arg.limits.v_max_mms <= V_MAX_MMS &&
                       out->arg.limits.a_max_mms2 >= A_MAX_MIN_MMS2 &&
                       out->arg.limits.a_max_mms2 <= A_MAX_MAX_MMS2
                   ? PROTO_ST_OK
                   : PROTO_ST_OUT_OF_RANGE;
    case PROTO_CMD_SERVO_SET: /* clamped by the actuator, never out of range */
        out->arg.angle_ddeg = proto_get_i16(p);
        return PROTO_ST_OK;
    case PROTO_CMD_SET_GAINS: {
        float g[4];
        for (int i = 0; i < 4; i++) {
            g[i] = proto_get_f32(p + 4 * i);
            if (!isfinite(g[i]) || g[i] < 0.0f)
                return PROTO_ST_OUT_OF_RANGE;
        }
        out->arg.gains.kp_v = g[0];
        out->arg.gains.ki_v = g[1];
        out->arg.gains.kp_p = g[2];
        out->arg.gains.u_dz = g[3];
        return PROTO_ST_OK;
    }
    case PROTO_CMD_SET_TELEM:
        out->arg.telem_hz = p[0];
        return (p[0] == 0 || p[0] == 10 || p[0] == 20 || p[0] == 50 || p[0] == 100)
                   ? PROTO_ST_OK
                   : PROTO_ST_OUT_OF_RANGE;
    case PROTO_CMD_OPENLOOP_DUTY:
        out->arg.duty_permille = proto_get_i16(p);
        return abs(out->arg.duty_permille) <= DUTY_MAX_PERMILLE ? PROTO_ST_OK
                                                                : PROTO_ST_OUT_OF_RANGE;
    default:
        return PROTO_ST_OK;
    }
}

cmd_action_t cmd_handle(cmd_ctx_t *ctx, const frame_t *req, const cmd_view_t *view, cmd_t *out,
                        uint8_t *reply, size_t *reply_len)
{
    bool exempt = req->cmd == PROTO_CMD_PING || req->cmd == PROTO_CMD_GET_STATUS ||
                  req->cmd == PROTO_CMD_GET_DIAG;

    if (!exempt && ctx->valid && req->seq == ctx->last_seq && req->cmd == ctx->last_cmd) {
        memcpy(reply, ctx->reply, ctx->reply_len);
        *reply_len = ctx->reply_len;
        return CMD_ACT_NONE;
    }

    uint8_t data[PROTO_LEN_MAX];
    uint8_t n = 1;
    uint8_t st;
    cmd_action_t act = CMD_ACT_NONE;
    int len = payload_len(req->cmd);

    if (len < 0)
        st = PROTO_ST_UNKNOWN_CMD;
    else if (req->len != len)
        st = PROTO_ST_BAD_LEN;
    else if (!cmd_allowed(req->cmd, view->state, view->openloop))
        st = PROTO_ST_BAD_STATE;
    else
        st = parse_args(req, out);

    if (st == PROTO_ST_OK) {
        switch (req->cmd) {
        case PROTO_CMD_PING:
            data[n++] = PROTO_VERSION;
            break;
        case PROTO_CMD_GET_STATUS:
            proto_pack_telem(&data[n], &view->telem);
            n += PROTO_TELEM_LEN;
            break;
        case PROTO_CMD_GET_DIAG:
            proto_pack_diag(&data[n], &view->diag);
            n += PROTO_DIAG_LEN;
            act = CMD_ACT_DIAG_RESET;
            break;
        case PROTO_CMD_ESTOP:
            act = CMD_ACT_ESTOP;
            break;
        default:
            if (view->queue_full) {
                st = PROTO_ST_BAD_STATE;
                break;
            }
            out->cmd = req->cmd;
            out->seq = req->seq;
            act = CMD_ACT_QUEUE;
            break;
        }
    }

    data[0] = st;
    *reply_len = frame_build(reply, req->seq, (uint8_t)(PROTO_ACK_FLAG | req->cmd), data, n);

    if (!exempt) {
        ctx->valid = true;
        ctx->last_seq = req->seq;
        ctx->last_cmd = req->cmd;
        ctx->reply_len = (uint8_t)*reply_len;
        memcpy(ctx->reply, reply, *reply_len);
    }
    return act;
}
