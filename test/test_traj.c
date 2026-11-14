#include <math.h>

#include "config.h"
#include "traj.h"
#include "unity.h"

static traj_t t;

void setUp(void) { traj_init(&t, 60.0f, 200.0f); }
void tearDown(void) {}

typedef struct {
    int steps;
    float v_peak, dv_max;
    bool monotonic;
} run_t;

/* Runs a move to completion and checks the REQ-T01 invariants on the way */
static run_t run_move(int32_t p0, int32_t s)
{
    run_t r = {0, 0.0f, 0.0f, true};
    traj_move(&t, p0, s);
    float v_prev = 0.0f;
    int32_t p_prev = p0;
    while (!traj_step(&t)) {
        r.steps++;
        TEST_ASSERT_TRUE_MESSAGE(r.steps < 100000, "trajectory never finishes");
        r.v_peak = fmaxf(r.v_peak, fabsf(t.v_ref_mms));
        r.dv_max = fmaxf(r.dv_max, fabsf(t.v_ref_mms - v_prev));
        if (s >= 0 ? t.p_ref_um < p_prev : t.p_ref_um > p_prev)
            r.monotonic = false;
        v_prev = t.v_ref_mms;
        p_prev = t.p_ref_um;
    }
    r.dv_max = fmaxf(r.dv_max, fabsf(v_prev)); /* last step down to 0 */
    TEST_ASSERT_EQUAL_INT32(p0 + s, t.p_ref_um);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.v_ref_mms);
    TEST_ASSERT_TRUE(r.monotonic);
    TEST_ASSERT_TRUE(r.v_peak <= t.v_max_mms + 1e-3f);
    /* 0.1 %: float time resolution on 100 s moves */
    TEST_ASSERT_TRUE(r.dv_max <= t.a_max_mms2 * CTRL_TS_S * 1.001f);
    return r;
}

static void test_trapezoid(void)
{
    run_t r = run_move(0, 200000); /* 200 mm > v^2/a = 18 mm */
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 60.0f, r.v_peak);
    /* T = 2 * 0.3 + (200 - 18) / 60 = 3.633 s */
    TEST_ASSERT_INT_WITHIN(2, 363, r.steps);
}

static void test_triangle(void)
{
    run_t r = run_move(0, 5000); /* 5 mm < 18 mm */
    TEST_ASSERT_FLOAT_WITHIN(0.5f, sqrtf(5.0f * 200.0f), r.v_peak);
    TEST_ASSERT_TRUE(r.v_peak < 60.0f);
}

static void test_negative_with_offset(void)
{
    run_t r = run_move(1234567, -200000);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 60.0f, r.v_peak);
}

static void test_negative_velocity_sign(void)
{
    traj_move(&t, 0, -50000);
    for (int i = 0; i < 50; i++) {
        traj_step(&t);
        TEST_ASSERT_TRUE(t.v_ref_mms <= 0.0f);
    }
}

static void test_long_move_exact_end(void)
{
    traj_init(&t, 90.0f, 2000.0f);
    run_move(-5000000, 10000000);
    traj_init(&t, 90.0f, 10.0f);
    run_move(0, 1); /* 1 um */
}

static void test_zero_move(void)
{
    traj_move(&t, 42, 0);
    TEST_ASSERT_TRUE(traj_step(&t));
    TEST_ASSERT_EQUAL_INT32(42, t.p_ref_um);
}

static void test_ramp(void)
{
    traj_ramp_to(&t, 50.0f);
    int n = 0;
    while (!traj_step(&t))
        n++;
    TEST_ASSERT_EQUAL(24, n); /* 2 mm/s per step, 25th step reaches 50 */
    TEST_ASSERT_EQUAL_FLOAT(50.0f, t.v_ref_mms);

    traj_ramp_to(&t, -10.0f);
    traj_step(&t);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 48.0f, t.v_ref_mms);
    while (!traj_step(&t))
        ;
    TEST_ASSERT_EQUAL_FLOAT(-10.0f, t.v_ref_mms);
}

static void test_ramp_from_move_velocity(void)
{
    traj_move(&t, 0, 200000);
    for (int i = 0; i < 100; i++)
        traj_step(&t);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 60.0f, t.v_ref_mms);
    traj_ramp_to(&t, 0.0f);
    int n = 1;
    while (!traj_step(&t))
        n++;
    TEST_ASSERT_EQUAL(30, n);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_trapezoid);
    RUN_TEST(test_triangle);
    RUN_TEST(test_negative_with_offset);
    RUN_TEST(test_negative_velocity_sign);
    RUN_TEST(test_long_move_exact_end);
    RUN_TEST(test_zero_move);
    RUN_TEST(test_ramp);
    RUN_TEST(test_ramp_from_move_velocity);
    return UNITY_END();
}
