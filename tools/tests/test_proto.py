"""Tests for tools/proto.py and the C <-> Python vector exchange (REQ-H01).
Run: python tools/tests/test_proto.py   (or make -C test vectors)"""
import os
import struct
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import proto as P  # noqa: E402

VEC_DIR = os.path.join(ROOT, "test", "vectors")


def vectors():
    """Same list as build_vectors() in test/test_frame.c"""
    return [
        P.encode(0, P.PING),
        P.encode(1, P.SET_VEL, P.pack(P.SET_VEL, -50)),
        P.encode(2, P.MOVE_REL, P.pack(P.MOVE_REL, 200000)),
        P.encode(3, P.SET_GAINS, P.pack(P.SET_GAINS, 4.0, 80.0, 2.0, 25.5)),
        P.encode(255, P.TELEM, bytes(range(22))),
        P.encode(7, 0x99, bytes(0xA0 + i for i in range(32))),
    ]


class TestProto(unittest.TestCase):
    def test_crc_check_value(self):
        self.assertEqual(P.crc16(b"123456789"), 0x29B1)

    def test_encode_ping(self):
        f = P.encode(7, P.PING)
        self.assertEqual(f[:5], b"\xAA\x55\x00\x07\x01")
        self.assertEqual(struct.unpack("<H", f[5:])[0], P.crc16(b"\x00\x07\x01"))

    def test_roundtrip(self):
        p = P.Parser()
        payload = bytes(range(0xAA, 0xAA - 32, -1))
        self.assertEqual(p.feed(P.encode(200, 0x30, payload), 0.0), [(200, 0x30, payload)])

    def test_bad_crc(self):
        p = P.Parser()
        f = bytearray(P.encode(1, P.SET_VEL, b"\x01\x02"))
        f[5] ^= 1
        self.assertEqual(p.feed(bytes(f), 0.0), [])
        self.assertEqual(p.err_crc, 1)
        self.assertEqual(len(p.feed(P.encode(2, P.PING), 1.0)), 1)

    def test_len_too_big(self):
        p = P.Parser()
        self.assertEqual(p.feed(b"\xAA\x55\x21\x00\x01", 0.0), [])
        self.assertEqual(p.err_len, 1)

    def test_garbage_and_timeout(self):
        p = P.Parser()
        f = P.encode(3, P.MOVE_REL, P.pack(P.MOVE_REL, 1))
        p.feed(b"\x00\xAA\x00\x55" + f[:6], 0.0)
        self.assertEqual(p.feed(f, 0.010), [(3, P.MOVE_REL, f[5:9])])
        self.assertEqual(p.err_timeout, 1)

    def test_telem_size(self):
        self.assertEqual(struct.calcsize(P.TELEM_FMT), P.TELEM_LEN)
        self.assertEqual(struct.calcsize(P.DIAG_FMT), 20)
        t = P.unpack_telem(bytes(range(22)))
        self.assertEqual(t["state"], 4)
        self.assertEqual(t["duty"], 18 | 19 << 8)
        self.assertEqual(t["missed_cycles"], 20 | 21 << 8)

    def test_vectors_exchange(self):
        vec = vectors()
        os.makedirs(VEC_DIR, exist_ok=True)
        with open(os.path.join(VEC_DIR, "py_frames.hex"), "w") as f:
            f.writelines(v.hex().upper() + "\n" for v in vec)
        path = os.path.join(VEC_DIR, "c_frames.hex")
        if not os.path.exists(path):
            self.skipTest("c_frames.hex missing: run make -C test first")
        with open(path) as f:
            c_frames = [bytes.fromhex(line.strip()) for line in f if line.strip()]
        self.assertEqual(c_frames, vec)
        p = P.Parser()
        for i, raw in enumerate(c_frames):
            self.assertEqual(len(p.feed(raw, float(i))), 1)


if __name__ == "__main__":
    unittest.main()
