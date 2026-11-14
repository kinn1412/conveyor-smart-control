#include "led.h"

#include "main.h"

void led_set(bool on)
{
    HAL_GPIO_WritePin(LEDR_GPIO_Port, LEDR_Pin, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
