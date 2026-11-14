#include <stdio.h>
#include <string.h>

#include "frame.h"
#include "unity.h"

static frame_parser_t p;
static frame_t out;

void setUp(void) { frame_parser_init(&p); }
void tearDown(void) {}

/* Feeds bytes 1 ms apart from t0. Returns the number of frames parsed. */
static int feed(const uint8_t *b, size_t n, uint32_t t0)
{
    int frames = 0;
    for (size_t i = 0; i < n; i++)
        frames += frame_parse_byte(&p, b[i], t0 + (uint32_t)i, &out);
    return frames;
}

static void test_build_ping(void)
{
    uint8_t buf[PROTO_FRAME_MAX];
    size_t n = frame_build(buf, 7, PROTO_CMD_PING, NULL, 0);
    uint8_t crc_in[] = {0x00, 0x07, 0x01};
    uint16_t crc = proto_crc16(crc_in, 3);
    uint8_t exp[] = {0xAA, 0x55, 0x00, 0x07, 0x01, (uint8_t)crc, (uint8_t)(crc >> 8)};
    TEST_ASSERT_EQUAL(7, n);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(exp, buf, 7);
}

static void test_roundtrip_max_payload(void)
{
    uint8_t pl[PROTO_LEN_MAX], buf[PROTO_FRAME_MAX];
    for (int i = 0; i < (int)PROTO_LEN_MAX; i++)
        pl[i] = (uint8_t)(0xAA - i); /* includes SOF-like bytes */
    size_t n = frame_build(buf, 200, 0x30, pl, PROTO_LEN_MAX);
    TEST_ASSERT_EQUAL(PROTO_FRAME_MAX, n);
    TEST_ASSERT_EQUAL(1, feed(buf, n, 0));
    TEST_ASSERT_EQUAL(200, out.seq);
    TEST_ASSERT_EQUAL_HEX8(0x30, out.cmd);
    TEST_ASSERT_EQUAL(PROTO_LEN_MAX, out.len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(pl, out.payload, PROTO_LEN_MAX);
}

static void test_bad_crc_dropped_then_recovers(void)
{
    uint8_t pl[2] = {1, 2}, buf[PROTO_FRAME_MAX];
    size_t n = frame_build(buf, 1, 0x11, pl, 2);
    buf[5] ^= 0x01;
    TEST_ASSERT_EQUAL(0, feed(buf, n, 0));
    TEST_ASSERT_EQUAL(1, p.err_crc);
    buf[5] ^= 0x01;
    TEST_ASSERT_EQUAL(1, feed(buf, n, 100));
}

static void test_len_too_big(void)
{
    uint8_t bad[] = {0xAA, 0x55, 33, 0, 1};
    TEST_ASSERT_EQUAL(0, feed(bad, sizeof bad, 0));
    TEST_ASSERT_EQUAL(1, p.err_len);
    uint8_t buf[PROTO_FRAME_MAX];
    size_t n = frame_build(buf, 2, PROTO_CMD_PING, NULL, 0);
    TEST_ASSERT_EQUAL(1, feed(buf, n, 100));
}

static void test_garbage_before_sof(void)
{
    uint8_t buf[64] = {0x00, 0xAA, 0x00, 0x55, 0xAA, 0xAA}; /* last 0xAA starts the frame */
    size_t n = frame_build(&buf[5], 9, PROTO_CMD_STOP, NULL, 0);
    TEST_ASSERT_EQUAL(1, feed(buf, 5 + n, 0));
    TEST_ASSERT_EQUAL(9, out.seq);
    TEST_ASSERT_EQUAL(0, p.err_crc + p.err_len + p.err_timeout);
}

static void test_truncated_frame_times_out(void)
{
    uint8_t pl[4] = {1, 2, 3, 4}, buf[PROTO_FRAME_MAX];
    size_t n = frame_build(buf, 3, PROTO_CMD_MOVE_REL, pl, 4);
    TEST_ASSERT_EQUAL(0, feed(buf, 6, 0)); /* cut after the first payload byte */
    /* next frame starts 6 ms after the last byte (t = 5) */
    TEST_ASSERT_EQUAL(1, feed(buf, n, 5 + PROTO_GAP_MS + 1));
    TEST_ASSERT_EQUAL(1, p.err_timeout);
    TEST_ASSERT_EQUAL(3, out.seq);
}

static void test_gap_of_5ms_is_allowed(void)
{
    uint8_t buf[PROTO_FRAME_MAX];
    size_t n = frame_build(buf, 4, PROTO_CMD_PING, NULL, 0);
    int frames = 0;
    for (size_t i = 0; i < n; i++)
        frames += frame_parse_byte(&p, buf[i], (uint32_t)i * PROTO_GAP_MS, &out);
    TEST_ASSERT_EQUAL(1, frames);
    TEST_ASSERT_EQUAL(0, p.err_timeout);
}

static void test_back_to_back_frames(void)
{
    uint8_t buf[2 * PROTO_FRAME_MAX];
    size_t n = frame_build(buf, 1, PROTO_CMD_PING, NULL, 0);
    n += frame_build(&buf[n], 2, PROTO_CMD_PING, NULL, 0);
    TEST_ASSERT_EQUAL(2, feed(buf, n, 0));
    TEST_ASSERT_EQUAL(2, out.seq);
}

/* Shared vectors, same list as tools/tests/test_proto.py */
static size_t build_vectors(uint8_t v[][PROTO_FRAME_MAX], size_t len[])
{
    uint8_t pl[PROTO_LEN_MAX];
    size_t k = 0;
    len[k] = frame_build(v[k], 0, PROTO_CMD_PING, NULL, 0), k++;
    proto_put_i16(pl, -50);
    len[k] = frame_build(v[k], 1, PROTO_CMD_SET_VEL, pl, 2), k++;
    proto_put_i32(pl, 200000);
    len[k] = frame_build(v[k], 2, PROTO_CMD_MOVE_REL, pl, 4), k++;
    proto_put_f32(&pl[0], 4.0f);
    proto_put_f32(&pl[4], 80.0f);
    proto_put_f32(&pl[8], 2.0f);
    proto_put_f32(&pl[12], 25.5f);
    len[k] = frame_build(v[k], 3, PROTO_CMD_SET_GAINS, pl, 16), k++;
    for (int i = 0; i < (int)PROTO_TELEM_LEN; i++)
        pl[i] = (uint8_t)i;
    len[k] = frame_build(v[k], 255, PROTO_TELEM, pl, PROTO_TELEM_LEN), k++;
    for (int i = 0; i < (int)PROTO_LEN_MAX; i++)
        pl[i] = (uint8_t)(0xA0 + i);
    len[k] = frame_build(v[k], 7, 0x99, pl, PROTO_LEN_MAX), k++;
    return k;
}

static void test_vectors_exchange(void)
{
    uint8_t v[8][PROTO_FRAME_MAX];
    size_t len[8];
    size_t k = build_vectors(v, len);

    FILE *f = fopen("vectors/c_frames.hex", "w");
    TEST_ASSERT_NOT_NULL(f);
    for (size_t i = 0; i < k; i++) {
        for (size_t j = 0; j < len[i]; j++)
            fprintf(f, "%02X", v[i][j]);
        fprintf(f, "\n");
    }
    fclose(f);

    f = fopen("vectors/py_frames.hex", "r");
    if (!f)
        TEST_IGNORE_MESSAGE("vectors/py_frames.hex missing: run make -C test vectors");
    char line[2 * PROTO_FRAME_MAX + 4];
    size_t i = 0;
    while (fgets(line, sizeof line, f)) {
        uint8_t b[PROTO_FRAME_MAX];
        size_t n = strcspn(line, "\r\n") / 2;
        for (size_t j = 0; j < n; j++) {
            unsigned x;
            sscanf(&line[2 * j], "%2X", &x);
            b[j] = (uint8_t)x;
        }
        TEST_ASSERT_TRUE(i < k);
        TEST_ASSERT_EQUAL(len[i], n);
        TEST_ASSERT_EQUAL_HEX8_ARRAY(v[i], b, n);
        TEST_ASSERT_EQUAL(1, feed(b, n, 1000 * (uint32_t)(i + 1)));
        i++;
    }
    fclose(f);
    TEST_ASSERT_EQUAL(k, i);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_build_ping);
    RUN_TEST(test_roundtrip_max_payload);
    RUN_TEST(test_bad_crc_dropped_then_recovers);
    RUN_TEST(test_len_too_big);
    RUN_TEST(test_garbage_before_sof);
    RUN_TEST(test_truncated_frame_times_out);
    RUN_TEST(test_gap_of_5ms_is_allowed);
    RUN_TEST(test_back_to_back_frames);
    RUN_TEST(test_vectors_exchange);
    return UNITY_END();
}
