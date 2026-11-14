#include "safety.h"

#include <math.h>

#include "config.h"
#include "proto.h"

void safety_reset(safety_t *s)
{
    s->stall_ms = 0;
}

uint8_t safety_check(safety_t *s, const safety_in_t *in)
{
    if (in->state != PROTO_STATE_READY && in->state != PROTO_STATE_RUNNING) {
        s->stall_ms = 0;
        return 0;
    }

    uint8_t f = 0;
    if (in->ms_since_rx > WATCHDOG_MS)
        f |= PROTO_FAULT_WATCHDOG;

    /* A held belt and a dead encoder look the same: high duty, no motion */
    s->stall_ms = (fabsf(in->u_permille) >= STALL_U_PERMILLE && fabsf(in->v_mms) < STALL_V_MMS)
                      ? (uint16_t)(s->stall_ms + CTRL_PERIOD_MS)
                      : 0;
    if (s->stall_ms >= STALL_MS)
        f |= PROTO_FAULT_STALL;
    return f;
}
