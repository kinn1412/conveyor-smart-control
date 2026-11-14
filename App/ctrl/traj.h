#ifndef TRAJ_H
#define TRAJ_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float v_max_mms, a_max_mms2;
    int32_t p_ref_um; /* outputs, valid after traj_step */
    float v_ref_mms;

    uint8_t mode;
    int32_t p0_um, s_um;
    float dir, d_mm, vp_mms, t_a_s, t_c_s, t_end_s;
    uint32_t k;
    float v_target_mms;
} traj_t;

void traj_init(traj_t *t, float v_max_mms, float a_max_mms2);
/* Trapezoid (triangle when |s| < v_max^2/a_max) from v = 0, REQ-T01 */
void traj_move(traj_t *t, int32_t p0_um, int32_t s_um);
/* Ramp v_ref to v_mms at a_max, starting from the current v_ref */
void traj_ramp_to(traj_t *t, float v_mms);
/* Advances one CTRL_TS_S step. Returns true when the profile has finished. */
bool traj_step(traj_t *t);

#endif
