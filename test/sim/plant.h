#ifndef PLANT_H
#define PLANT_H

/* Belt plant for SIL and conveyor tests: first order + deadzone + encoder quantization.
   Takes the physical duty and returns the raw counter, so DIR_SIGN / ENC_SIGN are undone here. */

#include <math.h>
#include <stdint.h>

#include "config.h"

typedef struct {
    double k_mms_per_permille, tau_s, dz_permille;
    double v_mms, pos_um;
} plant_t;

static inline void plant_step(plant_t *p, int16_t duty, double dt_s)
{
    double u = duty * DIR_SIGN;
    double target = 0.0;
    if (fabs(u) > p->dz_permille)
        target = p->k_mms_per_permille * (u - copysign(p->dz_permille, u));
    p->v_mms += (target - p->v_mms) * dt_s / p->tau_s;
    p->pos_um += p->v_mms * dt_s * 1000.0;
}

/* Runs one control period in 1 ms sub-steps */
static inline void plant_period(plant_t *p, int16_t duty)
{
    for (unsigned i = 0; i < CTRL_PERIOD_MS; i++)
        plant_step(p, duty, 0.001);
}

static inline uint16_t plant_cnt(const plant_t *p)
{
    return (uint16_t)((int32_t)floor(p->pos_um / UM_PER_CNT) * ENC_SIGN);
}

#endif
