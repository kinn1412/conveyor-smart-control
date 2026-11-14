#include <math.h>
#include <string.h>

#include "cmd.h"
#include "config.h"
#include "unity.h"

static cmd_ctx_t ctx;
static cmd_view_t view;
static cmd_t out;
static uint8_t reply[PROTO_FRAME_MAX];
static size_t reply_len;
static cmd_action_t act;
static uint8_t seq;

void setUp(void)
{
    cmd_init(&ctx);
    memset(&view, 0, sizeof view);
    memset(&out, 0, sizeof out);
    view.state = PROTO_STATE_READY;
}
void tearDown(void) {}

/* Sends one request with a fresh SEQ, returns the reply status */
static uint8_t send_seq(uint8_t s, uint8_t cmd, const uint8_t *pl, uint8_t len)
{
    frame_t req = {.seq = s, .cmd = cmd, .len = len};
    if (len)
        memcpy(req.payload, pl, len);
    act = cmd_handle(&ctx, &req, &view, &out, reply, &reply_len);
    TEST_ASSERT_EQUAL_HEX8(PROTO_ACK_FLAG | cmd, reply[4]);
    TEST_ASSERT_EQUAL(s, reply[3]);
    TEST_ASSERT_EQUAL_HEX16(proto_crc16(&reply[2], 3u + reply[2]),
                            proto_get_u16(&reply[5 + reply[2]]));
    return reply[5];
}

static uint8_t send(uint8_t cmd, const uint8_t *pl, uint8_t len)
{
    return send_seq(++seq, cmd, pl, len);
}

static uint8_t send_i16(uint8_t cmd, int16_t v)
{
    uint8_t pl[2];
    proto_put_i16(pl, v);
    return send(cmd, pl, 2);
}

static uint8_t send_u8(uint8_t cmd, uint8_t v) { return send(cmd, &v, 1); }

static void test_ping(void)
{
    view.state = PROTO_STATE_FAULT;
    TEST_ASSERT_EQUAL(PROTO_ST_OK, send(PROTO_CMD_PING, NULL, 0));
    TEST_ASSERT_EQUAL(2, reply[2]);
    TEST_ASSERT_EQUAL(PROTO_VERSION, reply[6]);
    TEST_ASSERT_EQUAL(CMD_ACT_NONE, act);
}

static void test_unknown_cmd(void)
{
    TEST_ASSERT_EQUAL(PROTO_ST_UNKNOWN_CMD, send(0x77, NULL, 0));
    TEST_ASSERT_EQUAL(1, reply[2]);
}

static void test_bad_len(void)
{
    uint8_t pl[4] = {0};
    TEST_ASSERT_EQUAL(PROTO_ST_BAD_LEN, send(PROTO_CMD_SET_VEL, pl, 1));
    TEST_ASSERT_EQUAL(PROTO_ST_BAD_LEN, send(PROTO_CMD_PING, pl, 1));
    TEST_ASSERT_EQUAL(PROTO_ST_BAD_LEN, send(PROTO_CMD_SET_GAINS, pl, 4));
    TEST_ASSERT_EQUAL(CMD_ACT_NONE, act);
}

static void test_get_status(void)
{
    view.telem.pos_um = -1234;
    view.telem.state = PROTO_STATE_READY;
    TEST_ASSERT_EQUAL(PROTO_ST_OK, send(PROTO_CMD_GET_STATUS, NULL, 0));
    TEST_ASSERT_EQUAL(1 + PROTO_TELEM_LEN, reply[2]);
    TEST_ASSERT_EQUAL_INT32(-1234, proto_get_i32(&reply[6 + 6]));
}

static void test_get_diag_resets(void)
{
    view.diag.exec_max_us = 321;
    TEST_ASSERT_EQUAL(PROTO_ST_OK, send(PROTO_CMD_GET_DIAG, NULL, 0));
    TEST_ASSERT_EQUAL(1 + PROTO_DIAG_LEN, reply[2]);
    TEST_ASSERT_EQUAL_UINT16(321, proto_get_u16(&reply[6 + 4]));
    TEST_ASSERT_EQUAL(CMD_ACT_DIAG_RESET, act);
}

static void test_estop_any_state(void)
{
    uint8_t states[] = {PROTO_STATE_IDLE, PROTO_STATE_READY, PROTO_STATE_RUNNING,
                        PROTO_STATE_FAULT};
    for (int i = 0; i < 4; i++) {
        view.state = states[i];
        TEST_ASSERT_EQUAL(PROTO_ST_OK, send(PROTO_CMD_ESTOP, NULL, 0));
        TEST_ASSERT_EQUAL(CMD_ACT_ESTOP, act);
    }
}

static void test_queued_args(void)
{
    TEST_ASSERT_EQUAL(PROTO_ST_OK, send_i16(PROTO_CMD_SET_VEL, -50));
    TEST_ASSERT_EQUAL(CMD_ACT_QUEUE, act);
    TEST_ASSERT_EQUAL_HEX8(PROTO_CMD_SET_VEL, out.cmd);
    TEST_ASSERT_EQUAL(seq, out.seq);
    TEST_ASSERT_EQUAL_INT16(-50, out.arg.vel_mms);

    uint8_t pl[16];
    proto_put_i32(pl, -200000);
    TEST_ASSERT_EQUAL(PROTO_ST_OK, send(PROTO_CMD_MOVE_REL, pl, 4));
    TEST_ASSERT_EQUAL_INT32(-200000, out.arg.dist_um);

    proto_put_u16(pl, 60);
    proto_put_u16(pl + 2, 200);
    TEST_ASSERT_EQUAL(PROTO_ST_OK, send(PROTO_CMD_SET_LIMITS, pl, 4));
    TEST_ASSERT_EQUAL(60, out.arg.limits.v_max_mms);
    TEST_ASSERT_EQUAL(200, out.arg.limits.a_max_mms2);

    float g[4] = {4.0f, 80.0f, 2.0f, 25.0f};
    for (int i = 0; i < 4; i++)
        proto_put_f32(pl + 4 * i, g[i]);
    TEST_ASSERT_EQUAL(PROTO_ST_OK, send(PROTO_CMD_SET_GAINS, pl, 16));
    TEST_ASSERT_EQUAL_FLOAT(4.0f, out.arg.gains.kp_v);
    TEST_ASSERT_EQUAL_FLOAT(80.0f, out.arg.gains.ki_v);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, out.arg.gains.kp_p);
    TEST_ASSERT_EQUAL_FLOAT(25.0f, out.arg.gains.u_dz);

    TEST_ASSERT_EQUAL(PROTO_ST_OK, send_i16(PROTO_CMD_SERVO_SET, -450));
    TEST_ASSERT_EQUAL_INT16(-450, out.arg.angle_ddeg);
    TEST_ASSERT_EQUAL(PROTO_ST_OK, send_u8(PROTO_CMD_SET_TELEM, 100));
    TEST_ASSERT_EQUAL(100, out.arg.telem_hz);
    TEST_ASSERT_EQUAL(PROTO_ST_OK, send_i16(PROTO_CMD_OPENLOOP_DUTY, -300));
    TEST_ASSERT_EQUAL_INT16(-300, out.arg.duty_permille);
    TEST_ASSERT_EQUAL(PROTO_ST_OK, send_u8(PROTO_CMD_ENABLE, 0));
    TEST_ASSERT_EQUAL(0, out.arg.enable);
    TEST_ASSERT_EQUAL(PROTO_ST_OK, send(PROTO_CMD_STOP, NULL, 0));
    TEST_ASSERT_EQUAL(CMD_ACT_QUEUE, act);
}

static void test_out_of_range(void)
{
    uint8_t pl[16];
    TEST_ASSERT_EQUAL(PROTO_ST_OK, send_i16(PROTO_CMD_SET_VEL, V_MAX_MMS));
    TEST_ASSERT_EQUAL(PROTO_ST_OUT_OF_RANGE, send_i16(PROTO_CMD_SET_VEL, V_MAX_MMS + 1));
    TEST_ASSERT_EQUAL(PROTO_ST_OUT_OF_RANGE, send_i16(PROTO_CMD_SET_VEL, -V_MAX_MMS - 1));
    TEST_ASSERT_EQUAL(CMD_ACT_NONE, act);

    proto_put_i32(pl, 10000000);
    TEST_ASSERT_EQUAL(PROTO_ST_OK, send(PROTO_CMD_MOVE_REL, pl, 4));
    proto_put_i32(pl, -10000001);
    TEST_ASSERT_EQUAL(PROTO_ST_OUT_OF_RANGE, send(PROTO_CMD_MOVE_REL, pl, 4));

    uint16_t lim[][2] = {{0, 200}, {V_MAX_MMS + 1, 200}, {60, 9}, {60, 2001}};
    for (int i = 0; i < 4; i++) {
        proto_put_u16(pl, lim[i][0]);
        proto_put_u16(pl + 2, lim[i][1]);
        TEST_ASSERT_EQUAL(PROTO_ST_OUT_OF_RANGE, send(PROTO_CMD_SET_LIMITS, pl, 4));
    }
    proto_put_u16(pl, 1);
    proto_put_u16(pl + 2, 10);
    TEST_ASSERT_EQUAL(PROTO_ST_OK, send(PROTO_CMD_SET_LIMITS, pl, 4));

    TEST_ASSERT_EQUAL(PROTO_ST_OUT_OF_RANGE, send_u8(PROTO_CMD_ENABLE, 2));
    TEST_ASSERT_EQUAL(PROTO_ST_OUT_OF_RANGE, send_u8(PROTO_CMD_SET_TELEM, 15));
    TEST_ASSERT_EQUAL(PROTO_ST_OK, send_u8(PROTO_CMD_SET_TELEM, 0));
    TEST_ASSERT_EQUAL(PROTO_ST_OUT_OF_RANGE, send_i16(PROTO_CMD_OPENLOOP_DUTY, 1001));
    TEST_ASSERT_EQUAL(PROTO_ST_OK, send_i16(PROTO_CMD_OPENLOOP_DUTY, -1000));

    float bad[] = {NAN, INFINITY, -1.0f};
    for (int i = 0; i < 3; i++) {
        memset(pl, 0, sizeof pl);
        proto_put_f32(pl + 4, bad[i]);
        TEST_ASSERT_EQUAL(PROTO_ST_OUT_OF_RANGE, send(PROTO_CMD_SET_GAINS, pl, 16));
    }
}

static void test_bad_state(void)
{
    uint8_t pl[16] = {0};
    view.state = PROTO_STATE_IDLE;
    TEST_ASSERT_EQUAL(PROTO_ST_BAD_STATE, send_i16(PROTO_CMD_SET_VEL, 10));
    TEST_ASSERT_EQUAL(PROTO_ST_BAD_STATE, send(PROTO_CMD_MOVE_REL, pl, 4));
    TEST_ASSERT_EQUAL(PROTO_ST_BAD_STATE, send_i16(PROTO_CMD_OPENLOOP_DUTY, 10));
    TEST_ASSERT_EQUAL(PROTO_ST_BAD_STATE, send(PROTO_CMD_CLEAR_FAULT, NULL, 0));

    view.state = PROTO_STATE_RUNNING;
    TEST_ASSERT_EQUAL(PROTO_ST_BAD_STATE, send(PROTO_CMD_MOVE_REL, pl, 4));
    TEST_ASSERT_EQUAL(PROTO_ST_BAD_STATE, send_u8(PROTO_CMD_ENABLE, 0));
    TEST_ASSERT_EQUAL(PROTO_ST_BAD_STATE, send(PROTO_CMD_SET_GAINS, pl, 16));
    TEST_ASSERT_EQUAL(PROTO_ST_BAD_STATE, send_i16(PROTO_CMD_OPENLOOP_DUTY, 10));
    view.openloop = true;
    TEST_ASSERT_EQUAL(PROTO_ST_OK, send_i16(PROTO_CMD_OPENLOOP_DUTY, 10));

    view.state = PROTO_STATE_FAULT;
    view.openloop = false;
    TEST_ASSERT_EQUAL(PROTO_ST_BAD_STATE, send_i16(PROTO_CMD_SERVO_SET, 0));
    TEST_ASSERT_EQUAL(PROTO_ST_BAD_STATE, send_u8(PROTO_CMD_ENABLE, 1));
    TEST_ASSERT_EQUAL(PROTO_ST_OK, send(PROTO_CMD_STOP, NULL, 0));
    TEST_ASSERT_EQUAL(PROTO_ST_OK, send_u8(PROTO_CMD_SET_TELEM, 10));
    TEST_ASSERT_EQUAL(PROTO_ST_OK, send(PROTO_CMD_CLEAR_FAULT, NULL, 0));
}

/* Spec §6.4 "Allowed in" column */
static void test_allowed_matrix(void)
{
    enum { I = 1 << PROTO_STATE_IDLE, R = 1 << PROTO_STATE_READY,
           U = 1 << PROTO_STATE_RUNNING, F = 1 << PROTO_STATE_FAULT };
    struct { uint8_t cmd; int mask; } rows[] = {
        {PROTO_CMD_PING, I | R | U | F},        {PROTO_CMD_GET_STATUS, I | R | U | F},
        {PROTO_CMD_GET_DIAG, I | R | U | F},    {PROTO_CMD_ENABLE, I | R},
        {PROTO_CMD_SET_VEL, R | U},             {PROTO_CMD_MOVE_REL, R},
        {PROTO_CMD_STOP, I | R | U | F},        {PROTO_CMD_ESTOP, I | R | U | F},
        {PROTO_CMD_CLEAR_FAULT, F},             {PROTO_CMD_SET_LIMITS, I | R},
        {PROTO_CMD_SERVO_SET, I | R | U},       {PROTO_CMD_SET_GAINS, I | R},
        {PROTO_CMD_SET_TELEM, I | R | U | F},   {PROTO_CMD_OPENLOOP_DUTY, R},
    };
    for (size_t i = 0; i < sizeof rows / sizeof rows[0]; i++)
        for (uint8_t s = PROTO_STATE_IDLE; s <= PROTO_STATE_FAULT; s++)
            TEST_ASSERT_EQUAL_MESSAGE((rows[i].mask >> s) & 1, cmd_allowed(rows[i].cmd, s, false),
                                      "cmd/state mismatch");
    TEST_ASSERT_TRUE(cmd_allowed(PROTO_CMD_OPENLOOP_DUTY, PROTO_STATE_RUNNING, true));
}

static void test_duplicate_resends_reply(void)
{
    view.state = PROTO_STATE_FAULT;
    TEST_ASSERT_EQUAL(PROTO_ST_OK, send_seq(6, PROTO_CMD_CLEAR_FAULT, NULL, 0));
    TEST_ASSERT_EQUAL(CMD_ACT_QUEUE, act);
    uint8_t first[PROTO_FRAME_MAX];
    size_t first_len = reply_len;
    memcpy(first, reply, reply_len);

    view.state = PROTO_STATE_IDLE; /* executed: the state changed */
    TEST_ASSERT_EQUAL(PROTO_ST_OK, send_seq(6, PROTO_CMD_CLEAR_FAULT, NULL, 0));
    TEST_ASSERT_EQUAL(CMD_ACT_NONE, act);
    TEST_ASSERT_EQUAL(first_len, reply_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(first, reply, reply_len);

    TEST_ASSERT_EQUAL(PROTO_ST_BAD_STATE, send_seq(7, PROTO_CMD_CLEAR_FAULT, NULL, 0));
}

static void test_exempt_commands_not_filtered(void)
{
    send_seq(9, PROTO_CMD_GET_DIAG, NULL, 0);
    send_seq(9, PROTO_CMD_GET_DIAG, NULL, 0);
    TEST_ASSERT_EQUAL(CMD_ACT_DIAG_RESET, act);

    /* an exempt command in between does not clear the filter */
    TEST_ASSERT_EQUAL(PROTO_ST_OK, send_seq(10, PROTO_CMD_STOP, NULL, 0));
    send_seq(11, PROTO_CMD_PING, NULL, 0);
    send_seq(10, PROTO_CMD_STOP, NULL, 0);
    TEST_ASSERT_EQUAL(CMD_ACT_NONE, act);
}

static void test_queue_full(void)
{
    view.queue_full = true;
    TEST_ASSERT_EQUAL(PROTO_ST_BAD_STATE, send_i16(PROTO_CMD_SET_VEL, 10));
    TEST_ASSERT_EQUAL(CMD_ACT_NONE, act);
    TEST_ASSERT_EQUAL(PROTO_ST_OK, send(PROTO_CMD_ESTOP, NULL, 0));
    TEST_ASSERT_EQUAL(PROTO_ST_OK, send(PROTO_CMD_PING, NULL, 0));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_ping);
    RUN_TEST(test_unknown_cmd);
    RUN_TEST(test_bad_len);
    RUN_TEST(test_get_status);
    RUN_TEST(test_get_diag_resets);
    RUN_TEST(test_estop_any_state);
    RUN_TEST(test_queued_args);
    RUN_TEST(test_out_of_range);
    RUN_TEST(test_bad_state);
    RUN_TEST(test_allowed_matrix);
    RUN_TEST(test_duplicate_resends_reply);
    RUN_TEST(test_exempt_commands_not_filtered);
    RUN_TEST(test_queue_full);
    return UNITY_END();
}
