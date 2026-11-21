#include "config.h"
#include "proto.h"
#include "safety.h"
#include "unity.h"

static safety_t s;
static safety_in_t in;

void setUp(void)
{
    safety_reset(&s);
    in = (safety_in_t){.state = PROTO_STATE_RUNNING};
}
void tearDown(void) {}

/* Number of cycles until `bit` is set, or -1 */
static int cycles_until(uint8_t bit, int max)
{
    for (int i = 1; i <= max; i++)
        if (safety_check(&s, &in) & bit)
            return i;
    return -1;
}

static void test_inactive_outside_ready_running(void)
{
    in.ms_since_rx = 10000;
    in.u_permille = 1000.0f;
    in.state = PROTO_STATE_IDLE;
    TEST_ASSERT_EQUAL(0, safety_check(&s, &in));
    in.state = PROTO_STATE_FAULT;
    TEST_ASSERT_EQUAL(0, safety_check(&s, &in));
}

static void test_watchdog(void)
{
    in.state = PROTO_STATE_READY;
    in.ms_since_rx = WATCHDOG_MS;
    TEST_ASSERT_EQUAL(0, safety_check(&s, &in));
    in.ms_since_rx = WATCHDOG_MS + 1;
    TEST_ASSERT_EQUAL(PROTO_FAULT_WATCHDOG, safety_check(&s, &in));
}

static void test_stall(void)
{
#if !STALL_CHECK
    TEST_IGNORE_MESSAGE("STALL_CHECK = 0 in config.h");
#endif
    in.u_permille = -STALL_U_PERMILLE;
    in.v_mms = 4.9f;
    TEST_ASSERT_EQUAL(STALL_MS / CTRL_PERIOD_MS, cycles_until(PROTO_FAULT_STALL, 100));
}

static void test_stall_timer_resets_on_motion(void)
{
#if !STALL_CHECK
    TEST_IGNORE_MESSAGE("STALL_CHECK = 0 in config.h");
#endif
    in.u_permille = 700.0f;
    for (int i = 0; i < 40; i++)
        safety_check(&s, &in);
    in.v_mms = 6.0f;
    safety_check(&s, &in);
    in.v_mms = 0.0f;
    TEST_ASSERT_EQUAL(STALL_MS / CTRL_PERIOD_MS, cycles_until(PROTO_FAULT_STALL, 100));
}

static void test_no_stall_below_threshold(void)
{
    in.u_permille = 599.0f;
    TEST_ASSERT_EQUAL(-1, cycles_until(PROTO_FAULT_STALL, 200));
}

static void test_reset_clears_timer(void)
{
#if !STALL_CHECK
    TEST_IGNORE_MESSAGE("STALL_CHECK = 0 in config.h");
#endif
    in.u_permille = 700.0f;
    for (int i = 0; i < 40; i++)
        safety_check(&s, &in);
    safety_reset(&s);
    TEST_ASSERT_EQUAL(STALL_MS / CTRL_PERIOD_MS, cycles_until(PROTO_FAULT_STALL, 100));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_inactive_outside_ready_running);
    RUN_TEST(test_watchdog);
    RUN_TEST(test_stall);
    RUN_TEST(test_stall_timer_resets_on_motion);
    RUN_TEST(test_no_stall_below_threshold);
    RUN_TEST(test_reset_clears_timer);
    return UNITY_END();
}
