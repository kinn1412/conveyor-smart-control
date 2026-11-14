#include "conveyor.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"

static float clampf(float x, float lim)
{
    return fmaxf(-lim, fminf(lim, x));
}

/* Duty 0, integrator and trajectory reset */
static void go_ready(conveyor_t *c)
{
    c->state = PROTO_STATE_READY;
    c->mode = MODE_NONE;
    c->stopping = false;
    c->u_permille = 0.0f;
    c->v_cmd_mms = 0.0f;
    pid_reset(&c->pid);
    traj_init(&c->traj, c->traj.v_max_mms, c->traj.a_max_mms2);
}

static void enter_fault(conveyor_t *c, uint8_t bits, conv_out_t *out)
{
    go_ready(c);
    c->state = PROTO_STATE_FAULT;
    c->fault_bits |= bits;
    out->fault_new |= bits;
}

/* REQ-T03: ramp to 0 at a_max, then READY */
static void start_stop(conveyor_t *c)
{
    if (c->state != PROTO_STATE_RUNNING)
        return;
    if (c->mode == MODE_OPENLOOP) {
        go_ready(c);
        return;
    }
    c->mode = MODE_VEL;
    c->stopping = true;
    traj_ramp_to(&c->traj, 0.0f);
}

void conveyor_init(conveyor_t *c, uint16_t enc_cnt)
{
    memset(c, 0, sizeof *c);
    c->last_cnt = enc_cnt;
    lpf_init(&c->vel_lpf, VEL_LPF_ALPHA, 0.0f);
    pid_init(&c->pid, KP_V, KI_V, U_DZ_PERMILLE, (float)DUTY_MAX_PERMILLE);
    traj_init(&c->traj, (float)V_MAX_DEFAULT_MMS, (float)A_MAX_DEFAULT_MMS2);
    c->kp_p = KP_P;
    c->state = PROTO_STATE_IDLE;
}

void conveyor_apply(conveyor_t *c, const cmd_t *cmd)
{
    bool openloop = c->state == PROTO_STATE_RUNNING && c->mode == MODE_OPENLOOP;
    if (!cmd_allowed(cmd->cmd, c->state, openloop))
        return;

    switch (cmd->cmd) {
    case PROTO_CMD_ENABLE:
        if (cmd->arg.enable && c->state == PROTO_STATE_IDLE) {
            go_ready(c);
            safety_reset(&c->safety);
        } else if (!cmd->arg.enable) {
            go_ready(c);
            c->state = PROTO_STATE_IDLE;
        }
        break;
    case PROTO_CMD_SET_VEL: {
        float v = clampf((float)cmd->arg.vel_mms, c->traj.v_max_mms);
        if (openloop)
            c->traj.v_ref_mms = c->v_mms; /* bumpless switch */
        c->state = PROTO_STATE_RUNNING;
        c->mode = MODE_VEL;
        c->stopping = v == 0.0f;
        traj_ramp_to(&c->traj, v);
        break;
    }
    case PROTO_CMD_MOVE_REL:
        traj_move(&c->traj, c->pos_um, cmd->arg.dist_um);
        c->state = PROTO_STATE_RUNNING;
        c->mode = MODE_MOVE;
        c->stopping = false;
        c->settle_ms = 0;
        c->move_seq = cmd->seq;
        break;
    case PROTO_CMD_STOP:
        start_stop(c);
        break;
    case PROTO_CMD_CLEAR_FAULT: /* REQ-S05: position kept */
        c->fault_bits = 0;
        go_ready(c);
        c->state = PROTO_STATE_IDLE;
        break;
    case PROTO_CMD_SET_LIMITS:
        c->traj.v_max_mms = cmd->arg.limits.v_max_mms;
        c->traj.a_max_mms2 = cmd->arg.limits.a_max_mms2;
        break;
    case PROTO_CMD_SERVO_SET: {
        int16_t a = cmd->arg.angle_ddeg;
        a = a < SERVO_ANGLE_MIN_DDEG ? SERVO_ANGLE_MIN_DDEG
                                     : (a > SERVO_ANGLE_MAX_DDEG ? SERVO_ANGLE_MAX_DDEG : a);
        c->servo_us = (uint16_t)lroundf((float)SERVO_US_CENTER + (float)a * SERVO_US_PER_DDEG);
        break;
    }
    case PROTO_CMD_SET_GAINS:
        c->pid.kp = cmd->arg.gains.kp_v;
        c->pid.ki = cmd->arg.gains.ki_v;
        c->kp_p = cmd->arg.gains.kp_p;
        c->pid.u_dz = cmd->arg.gains.u_dz;
        break;
    case PROTO_CMD_SET_TELEM:
        c->telem_hz = cmd->arg.telem_hz;
        break;
    case PROTO_CMD_OPENLOOP_DUTY:
        c->state = PROTO_STATE_RUNNING;
        c->mode = MODE_OPENLOOP;
        c->stopping = false;
        c->ol_duty_permille = cmd->arg.duty_permille;
        break;
    default:
        break;
    }
}

void conveyor_step(conveyor_t *c, const conv_in_t *in, conv_out_t *out)
{
    memset(out, 0, sizeof *out);

    /* Measurement, REQ-M01..M03 */
    int32_t dcnt = (int16_t)(in->enc_cnt - c->last_cnt) * ENC_SIGN;
    c->last_cnt = in->enc_cnt;
    c->pos_cnt += dcnt;
    c->pos_um = (int32_t)llround((double)c->pos_cnt * UM_PER_CNT);
    float v_raw = (float)dcnt * (float)(UM_PER_CNT / 1000.0) / CTRL_TS_S;
    c->v_mms = lpf_step(&c->vel_lpf, v_raw);

    if (in->estop && !(c->fault_bits & PROTO_FAULT_ESTOP))
        enter_fault(c, PROTO_FAULT_ESTOP, out);

    safety_in_t si = {
        .u_permille = c->u_permille,
        .v_mms = c->v_mms,
        .ms_since_rx = in->ms_since_rx,
        .state = c->state,
    };
    uint8_t f = safety_check(&c->safety, &si);
    if (f == PROTO_FAULT_WATCHDOG && c->state == PROTO_STATE_RUNNING)
        start_stop(c); /* ramp stop first; FAULT once READY */
    else if (f)
        enter_fault(c, f, out);

    float u = 0.0f;
    if (c->state == PROTO_STATE_RUNNING) {
        switch (c->mode) {
        case MODE_VEL: {
            bool done = traj_step(&c->traj);
            c->v_cmd_mms = c->traj.v_ref_mms;
            u = pid_step(&c->pid, c->v_cmd_mms, c->v_mms);
            if (c->stopping && done) {
                go_ready(c);
                u = 0.0f;
            }
            break;
        }
        case MODE_MOVE: {
            bool done = traj_step(&c->traj);
            int32_t e_um = c->traj.p_ref_um - c->pos_um;
            c->v_cmd_mms = clampf(c->traj.v_ref_mms + c->kp_p * (float)e_um / 1000.0f,
                                  (float)V_MAX_MMS);
            u = pid_step(&c->pid, c->v_cmd_mms, c->v_mms);
            if (done && labs((long)e_um) < MOVE_DONE_ERR_UM && fabsf(c->v_mms) < V_ZERO_MMS)
                c->settle_ms = (uint16_t)(c->settle_ms + CTRL_PERIOD_MS);
            else
                c->settle_ms = 0;
            if (c->settle_ms >= MOVE_DONE_MS) {
                out->move_done = true;
                out->move_seq = c->move_seq;
                out->move_err_um = e_um;
                go_ready(c);
                u = 0.0f;
            }
            break;
        }
        case MODE_OPENLOOP:
            u = (float)c->ol_duty_permille;
            break;
        default:
            break;
        }
    }

    c->u_permille = u;
    c->duty_permille = out->duty_permille = (int16_t)(lroundf(u) * DIR_SIGN);
    out->servo_us = c->servo_us;
    c->servo_us = 0;
}

void conveyor_telem(const conveyor_t *c, uint32_t t_ms, proto_telem_t *t)
{
    bool moving = c->state == PROTO_STATE_RUNNING && c->mode == MODE_MOVE;
    *t = (proto_telem_t){
        .t_ms = t_ms,
        .state = c->state,
        .fault_bits = c->fault_bits,
        .pos_um = c->pos_um,
        .vel_dmms = (int16_t)lroundf(c->v_mms * 10.0f),
        .vel_ref_dmms = (int16_t)lroundf(c->v_cmd_mms * 10.0f),
        .pos_ref_um = moving ? c->traj.p_ref_um : c->pos_um,
        .duty_permille = c->duty_permille,
    };
}
