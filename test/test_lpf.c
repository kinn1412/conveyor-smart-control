#include "lpf.h"
#include "unity.h"

void setUp(void) {}
void tearDown(void) {}

static void test_step_response(void)
{
    lpf_t f;
    lpf_init(&f, 0.5f, 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.5f, lpf_step(&f, 1.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.75f, lpf_step(&f, 1.0f));
    for (int i = 0; i < 30; i++)
        lpf_step(&f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, f.y);
}

static void test_alpha_one_passes_through(void)
{
    lpf_t f;
    lpf_init(&f, 1.0f, 5.0f);
    TEST_ASSERT_EQUAL_FLOAT(-3.0f, lpf_step(&f, -3.0f));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_step_response);
    RUN_TEST(test_alpha_one_passes_through);
    return UNITY_END();
}
