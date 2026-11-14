#include "pid.h"

#include <math.h>

#include "config.h"

void pid_init(pid_ctrl_t *c, float kp, float ki, float u_dz, float u_max)
{
    c->kp = kp;
    c->ki = ki;
    c->u_dz = u_dz;
    c->u_max = u_max;
    c->i = 0.0f;
}

void pid_reset(pid_ctrl_t *c)
{
    c->i = 0.0f;
}

float pid_step(pid_ctrl_t *c, float v_cmd_mms, float v_mms)
{
    if (v_cmd_mms == 0.0f && fabsf(v_mms) < V_ZERO_MMS) {
        c->i = 0.0f;
        return 0.0f;
    }

    float e = v_cmd_mms - v_mms;
    float ff = v_cmd_mms > 0.0f ? c->u_dz : (v_cmd_mms < 0.0f ? -c->u_dz : 0.0f);
    float u = c->kp * e + c->i + ff;

    /* Conditional anti-windup: hold I while saturated unless e pulls u back */
    if (!((u > c->u_max && e > 0.0f) || (u < -c->u_max && e < 0.0f)))
        c->i += c->ki * CTRL_TS_S * e;

    u = c->kp * e + c->i + ff;
    return fmaxf(-c->u_max, fminf(c->u_max, u));
}
