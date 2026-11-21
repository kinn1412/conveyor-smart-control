#ifndef CONFIG_H
#define CONFIG_H

/* Pure constants. No HAL / FreeRTOS includes: used by host tests and SIL. */

/* 1: driver bring-up without UART (App/app/bringup.c). 0: normal firmware. */
#define APP_BRINGUP 0

/* Timing */
#define CTRL_PERIOD_MS 10u
#define CTRL_TS_S 0.01f
#define CTRL_MISSED_US 15000u


/* Encoder and belt */
#define ENC_PPR 500 /* Omron E6B2-CWZ6C*/
#define ENC_CPR (ENC_PPR * 4)
#define ROLLER_D_MM 40.0 /* TBD: driven roller, belt included; encoder geared 1:1 to its shaft */
#define UM_PER_CNT (3.14159265358979 * ROLLER_D_MM * 1000.0 / ENC_CPR)
#define V_MAX_MMS 150 /* TBD: ~0.7 x no-load speed (224 mm/s if D = 40 mm) */
#define ENC_SIGN -1 /* set at bring-up (REQ-M04) */
#define DIR_SIGN 1

/* Limits */
#define DUTY_MAX_PERMILLE 1000
#define BRINGUP_DUTY_MAX_PERMILLE 300
#define MOVE_MAX_UM 10000000L
#define A_MAX_MIN_MMS2 10
#define A_MAX_MAX_MMS2 2000

/* Servo. TBD: mechanical calibration */
#define SERVO_US_MIN 1000u
#define SERVO_US_MAX 2000u
#define SERVO_US_CENTER 1500u
#define SERVO_ANGLE_MIN_DDEG (-900)
#define SERVO_ANGLE_MAX_DDEG 900
#define SERVO_US_PER_DDEG (500.0f / 900.0f)

/* Control defaults. TBD: plan phase 2-3 */
#define KP_V 2.5f   /* permille/(mm/s) */
#define KI_V 50.0f  /* permille/mm */
#define KP_P 2.0f   /* 1/s */
#define U_DZ_PERMILLE 0.0f
#define VEL_LPF_ALPHA 0.5f
#define V_ZERO_MMS 2.0f
#define V_MAX_DEFAULT_MMS 60
#define A_MAX_DEFAULT_MMS2 200

/* Move done (REQ-T02) */
#define MOVE_DONE_ERR_UM 500
#define MOVE_DONE_MS 100u

/* Safety (spec §8) */
#define STALL_CHECK 0 /* 0 only for bring-up while the encoder does not count */
#define WATCHDOG_MS 300u
#define STALL_U_PERMILLE 600.0f
#define STALL_V_MMS 5.0f
#define STALL_MS 500u

/* Tasks */
#define CTRL_STACK_WORDS 384u
#define RX_STACK_WORDS 256u
#define TX_STACK_WORDS 256u
#define CTRL_PRIO 3u /* added to tskIDLE_PRIORITY */
#define RX_PRIO 2u
#define TX_PRIO 1u
#define RX_STREAM_BYTES 256u
#define CMD_QUEUE_LEN 8u
#define TX_QUEUE_LEN 8u
#define RX_TIMEOUT_MS 5u
#define TX_DONE_TIMEOUT_MS 10u
#define CMD_APPLY_TIMEOUT_MS 15u /* > one control period; REQ-P09 */

#endif
