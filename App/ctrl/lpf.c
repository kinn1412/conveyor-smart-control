#include "lpf.h"

void lpf_init(lpf_t *f, float alpha, float y0)
{
    f->alpha = alpha;
    f->y = y0;
}

float lpf_step(lpf_t *f, float x)
{
    f->y += f->alpha * (x - f->y);
    return f->y;
}
