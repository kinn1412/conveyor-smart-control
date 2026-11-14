#ifndef SAFETY_H
#define SAFETY_H

#include <stdint.h>

typedef struct {
    float u_permille, v_mms;
    uint32_t ms_since_rx;
    uint8_t state;
} safety_in_t;

typedef struct {
    uint16_t stall_ms;
} safety_t;

void safety_reset(safety_t *s);
/* Spec §8, checked in READY and RUNNING. Returns PROTO_FAULT_* bits. */
uint8_t safety_check(safety_t *s, const safety_in_t *in);

#endif
