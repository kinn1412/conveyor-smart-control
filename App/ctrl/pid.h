#ifndef PID_H
#define PID_H

/* pid_ctrl_t, not pid_t: pid_t clashes with POSIX on the host */
typedef struct {
    float kp, ki, u_dz, u_max, i;
} pid_ctrl_t;

void pid_init(pid_ctrl_t *c, float kp, float ki, float u_dz, float u_max);
void pid_reset(pid_ctrl_t *c);
/* Velocity PI, REQ-C02. Returns duty in permille, clamped to +-u_max. */
float pid_step(pid_ctrl_t *c, float v_cmd_mms, float v_mms);

#endif
