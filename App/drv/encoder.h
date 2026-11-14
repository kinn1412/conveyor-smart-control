#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

void encoder_init(void);
uint16_t encoder_read(void); /* raw TIM4 counter */

#endif
