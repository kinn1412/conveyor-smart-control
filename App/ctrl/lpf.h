#ifndef LPF_H
#define LPF_H

typedef struct {
    float alpha, y;
} lpf_t;

void lpf_init(lpf_t *f, float alpha, float y0);
float lpf_step(lpf_t *f, float x);

#endif
