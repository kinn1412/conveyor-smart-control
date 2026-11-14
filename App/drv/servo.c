#include "servo.h"

#include "config.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;

void servo_init(void)
{
    servo_set_us(SERVO_US_CENTER);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); /* also sets MOE on TIM1 */
}

void servo_set_us(uint16_t us)
{
    if (us < SERVO_US_MIN)
        us = SERVO_US_MIN;
    if (us > SERVO_US_MAX)
        us = SERVO_US_MAX;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, us); /* PSC 71: 1 tick = 1 us */
}
