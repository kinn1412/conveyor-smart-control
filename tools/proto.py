"""Protocol reference implementation (spec §6, REQ-H01). Mirrors shared/proto/."""
import binascii
import struct

PROTO_VERSION = 2
SOF = b"\xAA\x55"
LEN_MAX = 32
GAP_S = 0.005
ACK_FLAG = 0x80

PING, GET_STATUS, GET_DIAG = 0x01, 0x02, 0x03
ENABLE, SET_VEL, MOVE_REL, STOP, ESTOP, CLEAR_FAULT, SET_LIMITS = range(0x10, 0x17)
SERVO_SET, SET_GAINS, SET_TELEM, OPENLOOP_DUTY = 0x20, 0x30, 0x31, 0x40
EVT_MOVE_DONE, EVT_FAULT, TELEM = 0xE0, 0xE1, 0xF0
EXEMPT = {PING, GET_STATUS, GET_DIAG}  # not duplicate-filtered (REQ-P07)

# Payload formats of §6.4, little-endian
CMD_FMT = {
    PING: "", GET_STATUS: "", GET_DIAG: "", ENABLE: "<B", SET_VEL: "<h", MOVE_REL: "<i",
    STOP: "", ESTOP: "", CLEAR_FAULT: "", SET_LIMITS: "<HH", SERVO_SET: "<h",
    SET_GAINS: "<4f", SET_TELEM: "<B", OPENLOOP_DUTY: "<h",
}
CMD_NAME = {
    PING: "ping", GET_STATUS: "get-status", GET_DIAG: "get-diag", ENABLE: "enable",
    SET_VEL: "set-vel", MOVE_REL: "move-rel", STOP: "stop", ESTOP: "estop",
    CLEAR_FAULT: "clear-fault", SET_LIMITS: "set-limits", SERVO_SET: "servo-set",
    SET_GAINS: "set-gains", SET_TELEM: "set-telem", OPENLOOP_DUTY: "openloop-duty",
    EVT_MOVE_DONE: "EVT_MOVE_DONE", EVT_FAULT: "EVT_FAULT", TELEM: "TELEM",
}
STATUS = ["OK", "BAD_LEN", "BAD_STATE", "OUT_OF_RANGE", "UNKNOWN_CMD"]
STATES = ["BOOT", "IDLE", "READY", "RUNNING", "FAULT"]
FAULTS = ["WATCHDOG", "ESTOP", "STALL"]

TELEM_FMT = "<IBBihhihH"  # §6.7, 22 B
TELEM_LEN = 22
TELEM_FIELDS = ("t_ms", "state", "fault_bits", "pos_um", "vel_dmms", "vel_ref_dmms",
                "pos_ref_um", "duty", "missed_cycles")
DIAG_FMT = "<10H"  # §6.8, 20 B
DIAG_FIELDS = ("period_min_us", "period_max_us", "exec_max_us", "estop_latency_max_us",
               "stack_free_ctrl", "stack_free_rx", "stack_free_tx",
               "err_crc", "err_len", "err_timeout")


def crc16(data: bytes) -> int:
    return binascii.crc_hqx(data, 0xFFFF)


def encode(seq: int, cmd: int, payload: bytes = b"") -> bytes:
    if len(payload) > LEN_MAX:
        raise ValueError("payload too long")
    body = bytes([len(payload), seq & 0xFF, cmd]) + payload
    return SOF + body + struct.pack("<H", crc16(body))


def pack(cmd: int, *args) -> bytes:
    return struct.pack(CMD_FMT[cmd], *args)


def unpack_telem(payload: bytes) -> dict:
    return dict(zip(TELEM_FIELDS, struct.unpack(TELEM_FMT, payload)))


def unpack_diag(payload: bytes) -> dict:
    return dict(zip(DIAG_FIELDS, struct.unpack(DIAG_FMT, payload)))


def fault_names(bits: int) -> str:
    return "|".join(n for i, n in enumerate(FAULTS) if bits >> i & 1) or "-"


class Parser:
    """Byte-wise parser, same rules as frame.c (REQ-P01, P02)."""

    def __init__(self):
        self.buf = bytearray()
        self.last_t = 0.0
        self.err_crc = self.err_len = self.err_timeout = 0

    def feed(self, data: bytes, now: float) -> list:
        """Returns a list of (seq, cmd, payload)."""
        frames = []
        for b in data:
            if self.buf and now - self.last_t > GAP_S:
                self.err_timeout += 1
                self.buf.clear()
            self.last_t = now
            buf = self.buf
            if not buf:
                if b == SOF[0]:
                    buf.append(b)
                continue
            if len(buf) == 1:
                if b == SOF[1]:
                    buf.append(b)
                elif b != SOF[0]:
                    buf.clear()
                continue
            if len(buf) == 2 and b > LEN_MAX:
                self.err_len += 1
                buf.clear()
                continue
            buf.append(b)
            if len(buf) == 7 + buf[2]:
                body, crc = bytes(buf[2:-2]), struct.unpack("<H", buf[-2:])[0]
                buf.clear()
                if crc != crc16(body):
                    self.err_crc += 1
                else:
                    frames.append((body[1], body[2], body[3:]))
        return frames
