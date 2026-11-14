#include "motor.h"

#include "config.h"
#include "main.h"

extern TIM_HandleTypeDef htim3;

void motor_init(void)
{
    motor_set(0);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}

void motor_set(int16_t permille)
{
    if (permille > DUTY_MAX_PERMILLE)
        permille = DUTY_MAX_PERMILLE;
    if (permille < -DUTY_MAX_PERMILLE)
        permille = -DUTY_MAX_PERMILLE;
    uint32_t mag = (uint32_t)(permille < 0 ? -permille : permille);
    HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, permille < 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    /* ARR 3599: 1000 permille -> CCR 3600 = 100 % */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,
                          mag * (__HAL_TIM_GET_AUTORELOAD(&htim3) + 1u) / 1000u);
}

void motor_kill(void)
{
    TIM3->CCR1 = 0;
}
