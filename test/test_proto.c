#include "proto.h"
#include "unity.h"

void setUp(void) {}
void tearDown(void) {}

static void test_crc_check_value(void)
{
    TEST_ASSERT_EQUAL_HEX16(0x29B1, proto_crc16((const uint8_t *)"123456789", 9));
}

static void test_crc_empty_is_init(void)
{
    TEST_ASSERT_EQUAL_HEX16(0xFFFF, proto_crc16(NULL, 0));
}

static void test_le_u16_u32(void)
{
    uint8_t b[4];
    proto_put_u16(b, 0x1234);
    TEST_ASSERT_EQUAL_HEX8(0x34, b[0]);
    TEST_ASSERT_EQUAL_HEX8(0x12, b[1]);
    TEST_ASSERT_EQUAL_HEX16(0x1234, proto_get_u16(b));

    proto_put_u32(b, 0x12345678u);
    uint8_t exp[] = {0x78, 0x56, 0x34, 0x12};
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp, b, 4);
    TEST_ASSERT_EQUAL_HEX32(0x12345678u, proto_get_u32(b));
}

static void test_le_signed(void)
{
    uint8_t b[4];
    proto_put_i16(b, -2);
    TEST_ASSERT_EQUAL_HEX8(0xFE, b[0]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, b[1]);
    TEST_ASSERT_EQUAL_INT16(-2, proto_get_i16(b));

    proto_put_i32(b, -200000);
    TEST_ASSERT_EQUAL_INT32(-200000, proto_get_i32(b));
    proto_put_i16(b, INT16_MIN);
    TEST_ASSERT_EQUAL_INT16(INT16_MIN, proto_get_i16(b));
}

static void test_f32(void)
{
    uint8_t b[4];
    proto_put_f32(b, 1.5f);
    uint8_t exp[] = {0x00, 0x00, 0xC0, 0x3F};
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp, b, 4);
    TEST_ASSERT_EQUAL_FLOAT(1.5f, proto_get_f32(b));
    proto_put_f32(b, -80.25f);
    TEST_ASSERT_EQUAL_FLOAT(-80.25f, proto_get_f32(b));
}

static void test_pack_telem_offsets(void)
{
    proto_telem_t t = {
        .t_ms = 0x01020304u,
        .state = 3,
        .fault_bits = 0x14,
        .pos_um = -123456,
        .vel_dmms = -500,
        .vel_ref_dmms = 600,
        .pos_ref_um = 200000,
        .duty_permille = -1000,
        .missed_cycles = 65535,
    };
    uint8_t b[PROTO_TELEM_LEN];
    proto_pack_telem(b, &t);
    TEST_ASSERT_EQUAL_HEX32(0x01020304u, proto_get_u32(&b[0]));
    TEST_ASSERT_EQUAL_UINT8(3, b[4]);
    TEST_ASSERT_EQUAL_HEX8(0x14, b[5]);
    TEST_ASSERT_EQUAL_INT32(-123456, proto_get_i32(&b[6]));
    TEST_ASSERT_EQUAL_INT16(-500, proto_get_i16(&b[10]));
    TEST_ASSERT_EQUAL_INT16(600, proto_get_i16(&b[12]));
    TEST_ASSERT_EQUAL_INT32(200000, proto_get_i32(&b[14]));
    TEST_ASSERT_EQUAL_INT16(-1000, proto_get_i16(&b[18]));
    TEST_ASSERT_EQUAL_UINT16(65535, proto_get_u16(&b[20]));
}

static void test_pack_diag_offsets(void)
{
    proto_diag_t d = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    uint8_t b[PROTO_DIAG_LEN];
    proto_pack_diag(b, &d);
    for (int i = 0; i < 10; i++)
        TEST_ASSERT_EQUAL_UINT16(i + 1, proto_get_u16(&b[2 * i]));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_crc_check_value);
    RUN_TEST(test_crc_empty_is_init);
    RUN_TEST(test_le_u16_u32);
    RUN_TEST(test_le_signed);
    RUN_TEST(test_f32);
    RUN_TEST(test_pack_telem_offsets);
    RUN_TEST(test_pack_diag_offsets);
    return UNITY_END();
}
