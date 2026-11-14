#ifndef CONVEYOR_H
#define CONVEYOR_H

#include <stdbool.h>
#include <stdint.h>

#include "cmd.h"
#include "lpf.h"
#include "pid.h"
#include "proto.h"
#include "safety.h"
#include "traj.h"

enum { MODE_NONE, MODE_VEL, MODE_MOVE, MODE_OPENLOOP };

typedef struct {
    uint16_t enc_cnt; /* raw TIM counter */
    uint32_t ms_since_rx;
    bool estop; /* g_estop */
} conv_in_t;

typedef struct {
    int16_t duty_permille; /* physical, DIR_SIGN applied */
    uint16_t servo_us;     /* 0 = no change */
    uint8_t fault_new;     /* bits latched this cycle -> EVT_FAULT */
    bool move_done;        /* -> EVT_MOVE_DONE */
    uint8_t move_seq;
    int32_t move_err_um;
} conv_out_t;

typedef struct {
    uint8_t state, mode, fault_bits;
    bool stopping; /* ramping to 0, then READY */
    uint16_t last_cnt;
    int32_t pos_cnt;
    int32_t pos_um;
    float v_mms, v_cmd_mms, u_permille;
    int16_t duty_permille;
    int16_t ol_duty_permille;
    lpf_t vel_lpf;
    pid_ctrl_t pid;
    traj_t traj;
    float kp_p;
    safety_t safety;
    uint16_t settle_ms;
    uint8_t move_seq;
    uint8_t telem_hz;
    uint16_t servo_us; /* pending servo write */
} conveyor_t;

void conveyor_init(conveyor_t *c, uint16_t enc_cnt); /* REQ-S03: IDLE, duty 0 */
void conveyor_apply(conveyor_t *c, const cmd_t *cmd); /* drops it if the state changed */
void conveyor_step(conveyor_t *c, const conv_in_t *in, conv_out_t *out);
void conveyor_telem(const conveyor_t *c, uint32_t t_ms, proto_telem_t *t);

#endif
