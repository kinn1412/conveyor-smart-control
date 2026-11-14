#include "encoder.h"

#include "main.h"

extern TIM_HandleTypeDef htim4;

void encoder_init(void)
{
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
}

uint16_t encoder_read(void)
{
    return (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
}
