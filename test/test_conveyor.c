#include <math.h>
#include <stdlib.h>

#include "config.h"
#include "conveyor.h"
#include "plant.h"
#include "unity.h"

static conveyor_t c;
static conv_out_t out;
static plant_t pl;
static uint32_t ms_since_rx;
static bool estop;

void setUp(void)
{
    pl = (plant_t){.k_mms_per_permille = 0.24, .tau_s = 0.05, .dz_permille = 0.0};
    conveyor_init(&c, 0);
    ms_since_rx = 0;
    estop = false;
}
void tearDown(void) {}

static void step(void)
{
    conv_in_t in = {.enc_cnt = plant_cnt(&pl), .ms_since_rx = ms_since_rx, .estop = estop};
    conveyor_step(&c, &in, &out);
    plant_period(&pl, out.duty_permille);
}

static void run(int cycles)
{
    for (int i = 0; i < cycles; i++)
        step();
}

static void apply(uint8_t cmd, int32_t arg)
{
    cmd_t k = {.cmd = cmd, .seq = 42};
    switch (cmd) {
    case PROTO_CMD_ENABLE:
        k.arg.enable = (uint8_t)arg;
        break;
    case PROTO_CMD_SET_VEL:
        k.arg.vel_mms = (int16_t)arg;
        break;
    case PROTO_CMD_MOVE_REL:
        k.arg.dist_um = arg;
        break;
    case PROTO_CMD_SERVO_SET:
        k.arg.angle_ddeg = (int16_t)arg;
        break;
    case PROTO_CMD_OPENLOOP_DUTY:
        k.arg.duty_permille = (int16_t)arg;
        break;
    default:
        break;
    }
    conveyor_apply(&c, &k);
}

static void test_boot_idle_duty_zero(void)
{
    step();
    TEST_ASSERT_EQUAL(PROTO_STATE_IDLE, c.state);
    TEST_ASSERT_EQUAL(0, out.duty_permille);
}

static void test_encoder_wrap(void) /* REQ-M01 */
{
    conveyor_init(&c, 65530);
    conv_in_t in = {.enc_cnt = 4};
    conveyor_step(&c, &in, &out);
    TEST_ASSERT_EQUAL_INT32(10, c.pos_cnt);
    in.enc_cnt = 65534;
    conveyor_step(&c, &in, &out);
    TEST_ASSERT_EQUAL_INT32(4, c.pos_cnt);
}

static void test_position_one_rev(void) /* REQ-M02 */
{
    conv_in_t in = {.enc_cnt = ENC_CPR};
    conveyor_step(&c, &in, &out);
    TEST_ASSERT_INT32_WITHIN(1, (int32_t)lround(ENC_CPR * UM_PER_CNT), c.pos_um);
}

static void test_enable_disable(void)
{
    apply(PROTO_CMD_ENABLE, 1);
    TEST_ASSERT_EQUAL(PROTO_STATE_READY, c.state);
    apply(PROTO_CMD_ENABLE, 0);
    TEST_ASSERT_EQUAL(PROTO_STATE_IDLE, c.state);
}

static void test_motion_rejected_in_idle(void)
{
    apply(PROTO_CMD_SET_VEL, 50);
    apply(PROTO_CMD_OPENLOOP_DUTY, 300);
    run(10);
    TEST_ASSERT_EQUAL(PROTO_STATE_IDLE, c.state);
    TEST_ASSERT_EQUAL(0, out.duty_permille);
}

static void test_velocity_and_stop(void)
{
    apply(PROTO_CMD_ENABLE, 1);
    apply(PROTO_CMD_SET_VEL, 50);
    run(200);
    TEST_ASSERT_EQUAL(PROTO_STATE_RUNNING, c.state);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 50.0f, c.v_mms);
    TEST_ASSERT_TRUE(out.duty_permille > 0);

    apply(PROTO_CMD_STOP, 0);
    run(10);
    TEST_ASSERT_EQUAL(PROTO_STATE_RUNNING, c.state); /* still ramping: 50 / 2 = 25 cycles */
    run(20);
    TEST_ASSERT_EQUAL(PROTO_STATE_READY, c.state);
    TEST_ASSERT_EQUAL(0, out.duty_permille);
}

static void test_reverse(void)
{
    apply(PROTO_CMD_ENABLE, 1);
    apply(PROTO_CMD_SET_VEL, -40);
    run(200);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, -40.0f, c.v_mms);
    TEST_ASSERT_TRUE(c.pos_um < 0);
}

static void test_move_done(void)
{
    apply(PROTO_CMD_ENABLE, 1);
    apply(PROTO_CMD_MOVE_REL, 100000);
    bool done = false;
    int32_t max_track = 0;
    for (int i = 0; i < 1000 && !done; i++) {
        step();
        done = out.move_done;
        if (!done && abs(c.traj.p_ref_um - c.pos_um) > max_track)
            max_track = abs(c.traj.p_ref_um - c.pos_um);
    }
    TEST_ASSERT_TRUE(done);
    TEST_ASSERT_EQUAL(42, out.move_seq);
    TEST_ASSERT_TRUE(abs(out.move_err_um) < MOVE_DONE_ERR_UM);
    TEST_ASSERT_EQUAL(PROTO_STATE_READY, c.state);
    TEST_ASSERT_INT32_WITHIN(MOVE_DONE_ERR_UM, 100000, c.pos_um);
    TEST_ASSERT_TRUE(max_track < 10000); /* sanity only: PERF-02 is checked after tuning */
}

static void test_stop_during_move(void)
{
    apply(PROTO_CMD_ENABLE, 1);
    apply(PROTO_CMD_MOVE_REL, 200000);
    run(100);
    apply(PROTO_CMD_STOP, 0);
    run(60);
    TEST_ASSERT_EQUAL(PROTO_STATE_READY, c.state);
    TEST_ASSERT_TRUE(c.pos_um < 150000);
}

static void test_openloop_and_sign(void)
{
    apply(PROTO_CMD_ENABLE, 1);
    apply(PROTO_CMD_OPENLOOP_DUTY, -250);
    step();
    TEST_ASSERT_EQUAL(-250, out.duty_permille);
    apply(PROTO_CMD_OPENLOOP_DUTY, 300); /* allowed in RUNNING(OPENLOOP) */
    step();
    TEST_ASSERT_EQUAL(300, out.duty_permille);
    apply(PROTO_CMD_STOP, 0);
    step();
    TEST_ASSERT_EQUAL(PROTO_STATE_READY, c.state);
    TEST_ASSERT_EQUAL(0, out.duty_permille);
}

static void test_watchdog_ramps_then_faults(void)
{
    apply(PROTO_CMD_ENABLE, 1);
    apply(PROTO_CMD_SET_VEL, 50);
    run(100);
    ms_since_rx = WATCHDOG_MS + 1;
    step();
    TEST_ASSERT_EQUAL(PROTO_STATE_RUNNING, c.state);
    TEST_ASSERT_EQUAL(0, out.fault_new);
    int faults = 0;
    for (int i = 0; i < 50; i++) {
        step();
        faults += out.fault_new != 0;
    }
    TEST_ASSERT_EQUAL(PROTO_STATE_FAULT, c.state);
    TEST_ASSERT_EQUAL_HEX8(PROTO_FAULT_WATCHDOG, c.fault_bits);
    TEST_ASSERT_EQUAL(1, faults);
    TEST_ASSERT_EQUAL(0, out.duty_permille);
}

static void test_estop_and_clear(void)
{
    apply(PROTO_CMD_ENABLE, 1);
    apply(PROTO_CMD_SET_VEL, 50);
    run(100);
    int32_t pos = c.pos_um;
    estop = true;
    step();
    estop = false;
    TEST_ASSERT_EQUAL(PROTO_STATE_FAULT, c.state);
    TEST_ASSERT_EQUAL_HEX8(PROTO_FAULT_ESTOP, out.fault_new);
    TEST_ASSERT_EQUAL(0, out.duty_permille);

    apply(PROTO_CMD_SET_VEL, 50); /* rejected in FAULT */
    apply(PROTO_CMD_ENABLE, 1);
    run(50);
    TEST_ASSERT_EQUAL(PROTO_STATE_FAULT, c.state);

    apply(PROTO_CMD_CLEAR_FAULT, 0);
    TEST_ASSERT_EQUAL(PROTO_STATE_IDLE, c.state);
    TEST_ASSERT_EQUAL(0, c.fault_bits);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, c.pid.i);
    TEST_ASSERT_TRUE(c.pos_um > pos); /* position kept (belt coasted on) */
}

static void test_stall_fault(void)
{
    apply(PROTO_CMD_ENABLE, 1);
    apply(PROTO_CMD_OPENLOOP_DUTY, 700);
    pl.k_mms_per_permille = 0.0; /* belt held */
    run(STALL_MS / CTRL_PERIOD_MS + 2);
    TEST_ASSERT_EQUAL(PROTO_STATE_FAULT, c.state);
    TEST_ASSERT_EQUAL_HEX8(PROTO_FAULT_STALL, c.fault_bits);
}

static void test_dead_encoder_trips_stall(void)
{
    apply(PROTO_CMD_ENABLE, 1);
    apply(PROTO_CMD_SET_VEL, 50);
    int i;
    for (i = 0; i < 200 && c.state != PROTO_STATE_FAULT; i++) {
        conv_in_t in = {.enc_cnt = 0}; /* encoder unplugged: PI winds up to saturation */
        conveyor_step(&c, &in, &out);
        plant_period(&pl, out.duty_permille);
    }
    TEST_ASSERT_EQUAL(PROTO_STATE_FAULT, c.state);
    TEST_ASSERT_EQUAL_HEX8(PROTO_FAULT_STALL, c.fault_bits);
    TEST_ASSERT_TRUE(i < 100); /* within 1 s */
}

static void test_servo(void)
{
    int16_t ang[] = {0, 900, 2000, -900, -2000};
    uint16_t us[] = {1500, 2000, 2000, 1000, 1000};
    for (int i = 0; i < 5; i++) {
        apply(PROTO_CMD_SERVO_SET, ang[i]);
        step();
        TEST_ASSERT_EQUAL_UINT16(us[i], out.servo_us);
        step();
        TEST_ASSERT_EQUAL_UINT16(0, out.servo_us); /* written once */
    }
}

static void test_telemetry(void)
{
    apply(PROTO_CMD_ENABLE, 1);
    apply(PROTO_CMD_MOVE_REL, 50000);
    run(50);
    proto_telem_t t;
    conveyor_telem(&c, 1234, &t);
    TEST_ASSERT_EQUAL_UINT32(1234, t.t_ms);
    TEST_ASSERT_EQUAL(PROTO_STATE_RUNNING, t.state);
    TEST_ASSERT_EQUAL_INT32(c.traj.p_ref_um, t.pos_ref_um);
    TEST_ASSERT_EQUAL_INT32(c.pos_um, t.pos_um);
    TEST_ASSERT_INT_WITHIN(1, (int)lroundf(c.v_mms * 10.0f), t.vel_dmms);
    TEST_ASSERT_EQUAL_INT16(c.duty_permille, t.duty_permille);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_boot_idle_duty_zero);
    RUN_TEST(test_encoder_wrap);
    RUN_TEST(test_position_one_rev);
    RUN_TEST(test_enable_disable);
    RUN_TEST(test_motion_rejected_in_idle);
    RUN_TEST(test_velocity_and_stop);
    RUN_TEST(test_reverse);
    RUN_TEST(test_move_done);
    RUN_TEST(test_stop_during_move);
    RUN_TEST(test_openloop_and_sign);
    RUN_TEST(test_watchdog_ramps_then_faults);
    RUN_TEST(test_estop_and_clear);
    RUN_TEST(test_stall_fault);
    RUN_TEST(test_dead_encoder_trips_stall);
    RUN_TEST(test_servo);
    RUN_TEST(test_telemetry);
    return UNITY_END();
}
