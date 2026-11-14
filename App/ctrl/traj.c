#include "traj.h"

#include <math.h>

#include "config.h"

enum { TRAJ_IDLE, TRAJ_MOVE, TRAJ_RAMP };

void traj_init(traj_t *t, float v_max_mms, float a_max_mms2)
{
    *t = (traj_t){.v_max_mms = v_max_mms, .a_max_mms2 = a_max_mms2};
}

void traj_move(traj_t *t, int32_t p0_um, int32_t s_um)
{
    float a = t->a_max_mms2;
    t->mode = TRAJ_MOVE;
    t->p0_um = p0_um;
    t->s_um = s_um;
    t->dir = s_um < 0 ? -1.0f : 1.0f;
    t->d_mm = fabsf((float)s_um) / 1000.0f;
    t->k = 0;
    t->p_ref_um = p0_um;
    t->v_ref_mms = 0.0f;

    if (t->d_mm < t->v_max_mms * t->v_max_mms / a) {
        t->vp_mms = sqrtf(t->d_mm * a);
        t->t_c_s = 0.0f;
    } else {
        t->vp_mms = t->v_max_mms;
        t->t_c_s = (t->d_mm - t->v_max_mms * t->v_max_mms / a) / t->v_max_mms;
    }
    t->t_a_s = t->vp_mms / a;
    t->t_end_s = 2.0f * t->t_a_s + t->t_c_s;
}

void traj_ramp_to(traj_t *t, float v_mms)
{
    t->mode = TRAJ_RAMP;
    t->v_target_mms = v_mms;
}

bool traj_step(traj_t *t)
{
    if (t->mode == TRAJ_RAMP) {
        float dv = t->a_max_mms2 * CTRL_TS_S;
        float e = t->v_target_mms - t->v_ref_mms;
        t->v_ref_mms = fabsf(e) <= dv ? t->v_target_mms : t->v_ref_mms + (e > 0 ? dv : -dv);
        return t->v_ref_mms == t->v_target_mms;
    }
    if (t->mode != TRAJ_MOVE)
        return true;

    float a = t->a_max_mms2;
    float tt = (float)++t->k * CTRL_TS_S;
    float x, v;
    if (tt >= t->t_end_s) {
        t->mode = TRAJ_IDLE;
        t->p_ref_um = t->p0_um + t->s_um; /* exact end point */
        t->v_ref_mms = 0.0f;
        return true;
    } else if (tt < t->t_a_s) {
        x = 0.5f * a * tt * tt;
        v = a * tt;
    } else if (tt < t->t_a_s + t->t_c_s) {
        x = 0.5f * a * t->t_a_s * t->t_a_s + t->vp_mms * (tt - t->t_a_s);
        v = t->vp_mms;
    } else {
        float td = t->t_end_s - tt;
        x = t->d_mm - 0.5f * a * td * td;
        v = a * td;
    }
    t->p_ref_um = t->p0_um + (int32_t)lroundf(t->dir * x * 1000.0f);
    t->v_ref_mms = t->dir * v;
    return false;
}
