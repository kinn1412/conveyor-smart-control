/* Driver bring-up without UART (APP_BRINGUP = 1 in config.h).
   Watch and write the bu_* variables with STM32CubeIDE Live Expressions.
   bu_auto = 1: fixed sequence, one step every BU_STEP_MS.
   bu_auto = 0: bu_duty_permille and bu_servo_us are applied as written. */
#include "bringup.h"

#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "encoder.h"
#include "led.h"
#include "motor.h"
#include "servo.h"
#include "timing.h"

#define BU_STEP_MS 2000u
#define BU_DUTY_PERMILLE 150

volatile uint8_t bu_auto = 1;
volatile uint8_t bu_step;
volatile int16_t bu_duty_permille;
volatile uint16_t bu_servo_us = SERVO_US_CENTER;
volatile uint16_t bu_enc_cnt;
volatile int32_t bu_enc_total; /* counts since boot, wrap handled */
volatile uint32_t bu_period_us;

static const struct {
    int16_t duty_permille;
    uint16_t servo_us;
} seq[] = {
    {0, SERVO_US_CENTER},  {BU_DUTY_PERMILLE, SERVO_US_CENTER},  {0, SERVO_US_CENTER},
    {-BU_DUTY_PERMILLE, SERVO_US_CENTER}, {0, SERVO_US_MIN}, {0, SERVO_US_MAX},
};

static StaticTask_t bu_tcb;
static StackType_t bu_stack[256];

static void bringup_task(void *arg)
{
    (void)arg;
    TickType_t wake = xTaskGetTickCount();
    uint32_t prev = timing_cyc(), ms = 0;
    uint16_t last = encoder_read();

    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(CTRL_PERIOD_MS));
        uint32_t now = timing_cyc();
        bu_period_us = timing_cyc_to_us(now - prev);
        prev = now;

        if (bu_auto) {
            bu_step = (uint8_t)((ms / BU_STEP_MS) % (sizeof seq / sizeof seq[0]));
            bu_duty_permille = seq[bu_step].duty_permille;
            bu_servo_us = seq[bu_step].servo_us;
        }
        int16_t d = bu_duty_permille; /* bring-up rule: |duty| <= 300 */
        if (d > BRINGUP_DUTY_MAX_PERMILLE)
            d = BRINGUP_DUTY_MAX_PERMILLE;
        if (d < -BRINGUP_DUTY_MAX_PERMILLE)
            d = -BRINGUP_DUTY_MAX_PERMILLE;
        motor_set(d);
        servo_set_us(bu_servo_us);

        uint16_t cnt = encoder_read();
        bu_enc_total += (int16_t)(cnt - last);
        last = cnt;
        bu_enc_cnt = cnt;

        led_set((ms / 250u) % 2u); /* 2 Hz blink: scheduler and clock alive */
        ms += CTRL_PERIOD_MS;
    }
}

void bringup_start(void)
{
    xTaskCreateStatic(bringup_task, "bringup", 256, NULL, tskIDLE_PRIORITY + 1, bu_stack, &bu_tcb);
}
