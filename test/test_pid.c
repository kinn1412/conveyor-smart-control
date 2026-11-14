#include "config.h"
#include "pid.h"
#include "unity.h"

static pid_ctrl_t c;

void setUp(void) { pid_init(&c, 4.0f, 80.0f, 0.0f, 1000.0f); }
void tearDown(void) {}

static void test_sign(void)
{
    TEST_ASSERT_TRUE(pid_step(&c, 50.0f, 0.0f) > 0.0f);
    pid_reset(&c);
    TEST_ASSERT_TRUE(pid_step(&c, -50.0f, 0.0f) < 0.0f);
}

static void test_integrator_accumulates(void)
{
    pid_init(&c, 0.0f, 100.0f, 0.0f, 1000.0f);
    pid_step(&c, 10.0f, 9.0f); /* e = 1 */
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 100.0f * CTRL_TS_S, c.i);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 2.0f, pid_step(&c, 10.0f, 9.0f));
}

static void test_zero_speed_reset(void)
{
    pid_step(&c, 20.0f, 0.0f);
    TEST_ASSERT_TRUE(c.i > 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid_step(&c, 0.0f, 1.9f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, c.i);
}

static void test_zero_cmd_still_brakes_when_moving(void)
{
    TEST_ASSERT_TRUE(pid_step(&c, 0.0f, 10.0f) < 0.0f);
}

static void test_no_windup_when_saturated(void)
{
    for (int i = 0; i < 20; i++)
        pid_step(&c, 80.0f, 0.0f); /* kp*e = 320, I grows until u saturates */
    float i_sat = c.i;
    for (int i = 0; i < 100; i++)
        TEST_ASSERT_EQUAL_FLOAT(1000.0f, pid_step(&c, 80.0f, 0.0f));
    TEST_ASSERT_EQUAL_FLOAT(i_sat, c.i);
    TEST_ASSERT_TRUE(i_sat <= 1000.0f);
}

static void test_integrates_out_of_saturation(void)
{
    for (int i = 0; i < 100; i++)
        pid_step(&c, 80.0f, 0.0f);
    float i_sat = c.i;
    /* v overshoots: e < 0 while u is still saturated -> I must unwind */
    float u = pid_step(&c, 80.0f, 85.0f);
    TEST_ASSERT_TRUE(c.i < i_sat);
    TEST_ASSERT_TRUE(u <= 1000.0f);
}

static void test_clamp_negative(void)
{
    pid_init(&c, 100.0f, 0.0f, 0.0f, 1000.0f);
    TEST_ASSERT_EQUAL_FLOAT(-1000.0f, pid_step(&c, -80.0f, 0.0f));
}

static void test_deadzone_compensation(void)
{
    pid_init(&c, 0.0f, 0.0f, 80.0f, 1000.0f);
    TEST_ASSERT_EQUAL_FLOAT(80.0f, pid_step(&c, 10.0f, 10.0f));
    TEST_ASSERT_EQUAL_FLOAT(-80.0f, pid_step(&c, -10.0f, -10.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid_step(&c, 0.0f, 0.0f));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_sign);
    RUN_TEST(test_integrator_accumulates);
    RUN_TEST(test_zero_speed_reset);
    RUN_TEST(test_zero_cmd_still_brakes_when_moving);
    RUN_TEST(test_no_windup_when_saturated);
    RUN_TEST(test_integrates_out_of_saturation);
    RUN_TEST(test_clamp_negative);
    RUN_TEST(test_deadzone_compensation);
    return UNITY_END();
}
