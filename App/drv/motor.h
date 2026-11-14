#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

void motor_init(void);              /* PWM 0, DIR low, start TIM3 CH1 */
void motor_set(int16_t permille);   /* clamps to +-1000, sign -> DIR pin */
void motor_kill(void);              /* register write only, for fault handlers */

#endif
