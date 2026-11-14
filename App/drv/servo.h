#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

void servo_init(void);          /* SERVO_US_CENTER */
void servo_set_us(uint16_t us); /* clamps to SERVO_US_MIN..MAX */

#endif
