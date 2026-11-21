#include "tasks.h"

#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "stream_buffer.h"
#include "task.h"

#include "bringup.h"
#include "cmd.h"
#include "config.h"
#include "conveyor.h"
#include "encoder.h"
#include "frame.h"
#include "led.h"
#include "motor.h"
#include "servo.h"
#include "timing.h"
#include "uart_link.h"

typedef struct {
    uint8_t len;
    uint8_t buf[PROTO_FRAME_MAX];
} tx_item_t;

static StaticTask_t ctrl_tcb, rx_tcb, tx_tcb;
static StackType_t ctrl_stack[CTRL_STACK_WORDS], rx_stack[RX_STACK_WORDS], tx_stack[TX_STACK_WORDS];
static TaskHandle_t ctrl_h, rx_h, tx_h;

static StaticStreamBuffer_t rx_stream_cb;
static uint8_t rx_stream_buf[RX_STREAM_BYTES + 1];
static StreamBufferHandle_t rx_stream;

static StaticQueue_t cmd_q_cb, tx_q_cb;
static uint8_t cmd_q_buf[CMD_QUEUE_LEN * sizeof(cmd_t)];
static uint8_t tx_q_buf[TX_QUEUE_LEN * sizeof(tx_item_t)];
static QueueHandle_t cmd_q, tx_q;

/* Given by ControlTask after it applied queued commands: CommRxTask ACKs only then (REQ-P09) */
static StaticSemaphore_t applied_cb;
static SemaphoreHandle_t applied_sem;

static volatile uint32_t g_estop, last_rx_tick;

/* Owned by ControlTask, read by CommRxTask inside a critical section */
static conveyor_t conv;
static proto_telem_t snap_telem;
static uint8_t snap_state;
static bool snap_openloop;

/* Timing stats in us (REQ-F08), reset by GET_DIAG */
static uint16_t period_min_us = UINT16_MAX, period_max_us, exec_max_us, estop_latency_max_us;
static uint16_t missed_cycles;

static uint16_t sat_u16(uint32_t x)
{
    return x > UINT16_MAX ? UINT16_MAX : (uint16_t)x;
}

static void send_frame(uint8_t seq, uint8_t cmd, const uint8_t *payload, uint8_t len,
                       TickType_t wait)
{
    tx_item_t it;
    it.len = (uint8_t)frame_build(it.buf, seq, cmd, payload, len);
    xQueueSend(tx_q, &it, wait);
}

static void control_task(void *arg)
{
    (void)arg;
    TickType_t wake = xTaskGetTickCount();
    uint32_t prev_cyc = timing_cyc();
    bool first = true;
    uint8_t evt_seq = 0;
    uint32_t telem_div = 0;

    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(CTRL_PERIOD_MS));
        uint32_t t0 = timing_cyc();

        /* Consumed here; CommRxTask cannot preempt this task between read and clear */
        bool estop = g_estop != 0;
        g_estop = 0;
        if (estop)
            motor_set(0);

        uint32_t period_us = timing_cyc_to_us(t0 - prev_cyc);
        prev_cyc = t0;
        if (!first) {
            if (period_us > CTRL_MISSED_US)
                missed_cycles++;
            taskENTER_CRITICAL();
            if (period_us < period_min_us)
                period_min_us = sat_u16(period_us);
            if (period_us > period_max_us)
                period_max_us = sat_u16(period_us);
            taskEXIT_CRITICAL();
        }
        first = false;

        cmd_t c;
        bool applied = false;
        while (xQueueReceive(cmd_q, &c, 0) == pdTRUE) {
            conveyor_apply(&conv, &c);
            applied = true;
        }

        TickType_t now = xTaskGetTickCount();
        conv_in_t in = {
            .enc_cnt = encoder_read(),
            .ms_since_rx = (uint32_t)(now - last_rx_tick),
            .estop = estop,
        };
        conv_out_t out;
        conveyor_step(&conv, &in, &out);

        motor_set(out.duty_permille);
        if (out.servo_us)
            servo_set_us(out.servo_us);
        led_set(conv.state == PROTO_STATE_FAULT);

        if (out.fault_new)
            send_frame(evt_seq++, PROTO_EVT_FAULT, &conv.fault_bits, 1, 0);
        if (out.move_done) {
            uint8_t p[5];
            p[0] = out.move_seq;
            proto_put_i32(&p[1], out.move_err_um);
            send_frame(evt_seq++, PROTO_EVT_MOVE_DONE, p, sizeof p, 0);
        }

        proto_telem_t t;
        conveyor_telem(&conv, (uint32_t)now, &t);
        t.missed_cycles = missed_cycles;
        taskENTER_CRITICAL();
        snap_telem = t;
        snap_state = conv.state;
        snap_openloop = conv.state == PROTO_STATE_RUNNING && conv.mode == MODE_OPENLOOP;
        taskEXIT_CRITICAL();
        if (applied)
            xSemaphoreGive(applied_sem); /* the snapshot now shows the new state */

        if (conv.telem_hz && ++telem_div >= 1000u / CTRL_PERIOD_MS / conv.telem_hz) {
            uint8_t p[PROTO_TELEM_LEN];
            telem_div = 0;
            proto_pack_telem(p, &t);
            send_frame(evt_seq++, PROTO_TELEM, p, sizeof p, 0); /* REQ-P05: drop if full */
        }

        uint32_t exec_us = timing_cyc_to_us(timing_cyc() - t0);
        taskENTER_CRITICAL();
        if (exec_us > exec_max_us)
            exec_max_us = sat_u16(exec_us);
        taskEXIT_CRITICAL();
    }
}

static void on_rx(const uint8_t *data, size_t len)
{
    BaseType_t woken = pdFALSE;
    xStreamBufferSendFromISR(rx_stream, data, len, &woken);
    portYIELD_FROM_ISR(woken);
}

static void on_tx_done(void)
{
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(tx_h, &woken);
    portYIELD_FROM_ISR(woken);
}

static void handle_frame(cmd_ctx_t *ctx, const frame_parser_t *parser, const frame_t *req)
{
    uint32_t t_frame = timing_cyc();
    cmd_view_t v = {0};

    taskENTER_CRITICAL();
    v.state = snap_state;
    v.openloop = snap_openloop;
    v.telem = snap_telem;
    v.diag.period_min_us = period_min_us == UINT16_MAX ? 0 : period_min_us;
    v.diag.period_max_us = period_max_us;
    v.diag.exec_max_us = exec_max_us;
    v.diag.estop_latency_max_us = estop_latency_max_us;
    taskEXIT_CRITICAL();

    v.queue_full = uxQueueSpacesAvailable(cmd_q) == 0;
    if (req->cmd == PROTO_CMD_GET_DIAG) {
        v.diag.stack_free_ctrl = (uint16_t)uxTaskGetStackHighWaterMark(ctrl_h);
        v.diag.stack_free_rx = (uint16_t)uxTaskGetStackHighWaterMark(rx_h);
        v.diag.stack_free_tx = (uint16_t)uxTaskGetStackHighWaterMark(tx_h);
        v.diag.err_crc = parser->err_crc;
        v.diag.err_len = parser->err_len;
        v.diag.err_timeout = parser->err_timeout;
    }

    cmd_t c;
    tx_item_t it;
    size_t len;
    cmd_action_t act = cmd_handle(ctx, req, &v, &c, it.buf, &len);
    it.len = (uint8_t)len;
    last_rx_tick = xTaskGetTickCount(); /* REQ-P03 */

    switch (act) {
    case CMD_ACT_ESTOP: { /* REQ-S04: flag first, then duty 0 */
        g_estop = 1;
        motor_set(0);
        uint16_t lat = sat_u16(timing_cyc_to_us(timing_cyc() - t_frame));
        taskENTER_CRITICAL();
        if (lat > estop_latency_max_us)
            estop_latency_max_us = lat;
        taskEXIT_CRITICAL();
        break;
    }
    case CMD_ACT_QUEUE:
        xSemaphoreTake(applied_sem, 0); /* drop a give left by an earlier timeout */
        if (c.cmd == PROTO_CMD_STOP)
            xQueueSendToFront(cmd_q, &c, 0);
        else
            xQueueSend(cmd_q, &c, 0);
        /* REQ-P09: ACK after ControlTask applied it, so the next command sees the new state */
        xSemaphoreTake(applied_sem, pdMS_TO_TICKS(CMD_APPLY_TIMEOUT_MS));
        break;
    case CMD_ACT_DIAG_RESET:
        taskENTER_CRITICAL();
        period_min_us = UINT16_MAX;
        period_max_us = exec_max_us = estop_latency_max_us = 0;
        taskEXIT_CRITICAL();
        break;
    default:
        break;
    }

    xQueueSend(tx_q, &it, pdMS_TO_TICKS(TX_DONE_TIMEOUT_MS)); /* REQ-P04: wait for a slot */
}

static void comm_rx_task(void *arg)
{
    (void)arg;
    static frame_parser_t parser;
    static cmd_ctx_t ctx;
    frame_parser_init(&parser);
    cmd_init(&ctx);
    uart_link_start(on_rx, on_tx_done);

    for (;;) {
        uint8_t buf[32];
        size_t n = xStreamBufferReceive(rx_stream, buf, sizeof buf, pdMS_TO_TICKS(RX_TIMEOUT_MS));
        uint32_t now_ms = (uint32_t)xTaskGetTickCount();
        frame_t req;
        for (size_t i = 0; i < n; i++)
            if (frame_parse_byte(&parser, buf[i], now_ms, &req))
                handle_frame(&ctx, &parser, &req);
    }
}

static void tx_task(void *arg)
{
    (void)arg;
    tx_item_t it;
    for (;;) {
        xQueueReceive(tx_q, &it, portMAX_DELAY);
        uart_link_send(it.buf, it.len);
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(TX_DONE_TIMEOUT_MS));
    }
}

void app_init(void)
{
    timing_init();
    motor_init();
    encoder_init();
    servo_init();
    led_set(false);

#if APP_BRINGUP
    bringup_start();
    return;
#endif

    conveyor_init(&conv, encoder_read());
    snap_state = conv.state;

    rx_stream = xStreamBufferCreateStatic(sizeof rx_stream_buf, 1, rx_stream_buf, &rx_stream_cb);
    cmd_q = xQueueCreateStatic(CMD_QUEUE_LEN, sizeof(cmd_t), cmd_q_buf, &cmd_q_cb);
    tx_q = xQueueCreateStatic(TX_QUEUE_LEN, sizeof(tx_item_t), tx_q_buf, &tx_q_cb);
    applied_sem = xSemaphoreCreateBinaryStatic(&applied_cb);

    ctrl_h = xTaskCreateStatic(control_task, "ctrl", CTRL_STACK_WORDS, NULL,
                                  tskIDLE_PRIORITY + CTRL_PRIO, ctrl_stack, &ctrl_tcb);
    rx_h = xTaskCreateStatic(comm_rx_task, "rx", RX_STACK_WORDS, NULL,
                                tskIDLE_PRIORITY + RX_PRIO, rx_stack, &rx_tcb);
    tx_h = xTaskCreateStatic(tx_task, "tx", TX_STACK_WORDS, NULL, tskIDLE_PRIORITY + TX_PRIO,
                                tx_stack, &tx_tcb);
}

void app_fault(void)
{
    taskDISABLE_INTERRUPTS();
    motor_kill();
    led_set(true);
    for (;;) {
    }
}
