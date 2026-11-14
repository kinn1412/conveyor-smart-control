#ifndef TIMING_H
#define TIMING_H

#include <stdint.h>

#include "main.h"

void timing_init(void); /* enable the DWT cycle counter, REQ-F08 */

static inline uint32_t timing_cyc(void)
{
    return DWT->CYCCNT;
}

static inline uint32_t timing_cyc_to_us(uint32_t cyc)
{
    return cyc / (SystemCoreClock / 1000000u);
}

#endif
