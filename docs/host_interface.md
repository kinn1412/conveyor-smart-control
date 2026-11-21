# Giao tiếp Host (H7) ↔ Actuator (STM32F103)

Protocol **v2**.

Actuator tự điều khiển belt (vận tốc, vị trí, 100 Hz) và servo. H7 chỉ ra lệnh **làm gì** và
**khi nào** (thời điểm gạt servo là việc của H7).

## 1. Đường truyền

UART **115200 8N1**, không flow control, mức 3.3 V. F103: PA9 = TX, PA10 = RX, chung GND.
Mọi số nguyên và float đều **little-endian**.

## 2. Frame

```
AA 55 | LEN | SEQ | CMD | PAYLOAD[LEN] | CRC_L CRC_H
```

- LEN: độ dài payload, 0..32 (frame tối đa 39 byte).
- SEQ: số thứ tự 0..255 do bên gửi đặt; reply giữ nguyên SEQ của request.
- CRC16-CCITT (poly 0x1021, init 0xFFFF) tính trên LEN..PAYLOAD. Kiểm: `"123456789"` → `0x29B1`.
- Reply: CMD = `0x80 | CMD`, byte đầu payload là **status**:
  `0` OK · `1` BAD_LEN · `2` BAD_STATE · `3` OUT_OF_RANGE · `4` UNKNOWN_CMD.

Ví dụ: PING (SEQ 1) `AA 55 00 01 01 8C EF` → reply `AA 55 02 01 81 00 02 13 0F` (OK, version 2).

## 3. Command (H7 → F103)

State: **I** IDLE, **R** READY, **U** RUNNING, **F** FAULT.

| CMD | Tên | Payload | State | Chức năng |
|---|---|---|---|---|
| 0x01 | PING | – | tất cả | Kiểm tra link, nuôi watchdog. Reply: u8 version (= 2) |
| 0x02 | GET_STATUS | – | tất cả | Đọc trạng thái. Reply: 22 B telemetry (§5) |
| 0x03 | GET_DIAG | – | tất cả | Đọc chẩn đoán timing/stack/lỗi (dùng khi test) |
| 0x10 | ENABLE | u8 0/1 | I, R | 1: IDLE → READY. 0: về IDLE |
| 0x11 | SET_VEL | i16 mm/s | R, U | Chạy belt ở vận tốc v (\|v\| ≤ 150, âm = lùi), tăng tốc theo a_max. 0 = dừng có ramp |
| 0x12 | MOVE_REL | i32 µm | R | Đi thêm quãng s (\|s\| ≤ 10 m) theo hình thang. Xong gửi EVT_MOVE_DONE |
| 0x13 | STOP | – | tất cả | Giảm tốc về 0 theo a_max rồi về READY |
| 0x14 | ESTOP | – | tất cả | Cắt motor ngay (≤ 2 ms), vào FAULT |
| 0x15 | CLEAR_FAULT | – | F | Xóa lỗi, về IDLE (giữ vị trí) |
| 0x16 | SET_LIMITS | u16 v_max mm/s, u16 a_max mm/s² | I, R | Giới hạn vận tốc (1..150) và gia tốc (10..2000). Mặc định 60, 200 |
| 0x20 | SERVO_SET | i16 góc ×0.1° | I, R, U | Đặt góc servo, tự kẹp trong ±90° |
| 0x30 | SET_GAINS | 4 × f32: Kp_v, Ki_v, Kp_p, u_dz | I, R | Đổi gain bộ điều khiển (≥ 0) |
| 0x31 | SET_TELEM | u8 Hz | tất cả | Bật telemetry 10/20/50/100 Hz, 0 = tắt |
| 0x40 | OPENLOOP_DUTY | i16 ‰ | R, U | Chạy hở với duty cố định (\|d\| ≤ 1000). Chỉ để test |

Ví dụ: ENABLE 1 `AA 55 01 02 10 01 46 8F` · SET_VEL 50 `AA 55 02 03 11 32 00 F7 D9` ·
MOVE_REL 200 mm `AA 55 04 04 12 40 0D 03 00 EE 0F` · SERVO_SET 45° `AA 55 02 05 20 C2 01 1B 0F` ·
ESTOP `AA 55 00 07 14 BE 07`.

## 4. State và lỗi

```
IDLE --ENABLE 1--> READY --SET_VEL / MOVE_REL--> RUNNING --STOP / xong move--> READY
bất kỳ --lỗi--> FAULT --CLEAR_FAULT--> IDLE
```

Giá trị state: BOOT 0, IDLE 1, READY 2, RUNNING 3, FAULT 4.

| Bit lỗi | Tên | Khi nào |
|---|---|---|
| 0 | WATCHDOG | READY/RUNNING mà 300 ms không nhận frame nào từ H7 |
| 1 | ESTOP | nhận lệnh ESTOP |
| 2 | STALL | duty ≥ 600 ‰ mà belt đứng yên 0.5 s (kẹt belt, hỏng encoder) |

## 5. F103 tự gửi (H7 không reply)

| CMD | Tên | Payload |
|---|---|---|
| 0xE0 | EVT_MOVE_DONE | u8 SEQ của MOVE_REL, i32 sai số cuối (µm) |
| 0xE1 | EVT_FAULT | u8 fault_bits |
| 0xF0 | TELEMETRY | 22 B, theo tần số SET_TELEM |

Telemetry 22 B (cũng là dữ liệu của GET_STATUS):

| Offset | Trường | Kiểu | Đơn vị |
|---|---|---|---|
| 0 | t_ms | u32 | ms từ lúc bật |
| 4 | state | u8 | §4 |
| 5 | fault_bits | u8 | §4 |
| 6 | pos | i32 | µm |
| 10 | vel | i16 | 0.1 mm/s |
| 12 | vel_ref | i16 | 0.1 mm/s |
| 14 | pos_ref | i32 | µm |
| 18 | duty | i16 | ‰ |
| 20 | missed_cycles | u16 | lần |

Event có thể mất nếu hàng đợi đầy: dùng `state` / `fault_bits` trong telemetry làm nguồn chính.

## 6. Quy tắc H7 phải theo

1. **Watchdog**: khi READY/RUNNING, gửi frame ít nhất mỗi 300 ms. Nên PING mỗi 100 ms.
   Telemetry từ F103 không tính.
2. **Một lệnh một lúc**: gửi, chờ reply rồi mới gửi lệnh tiếp. Reply OK của lệnh điều khiển chỉ
   về sau khi lệnh đã được áp dụng, nên `ENABLE 1` → OK → `SET_VEL` gửi liền được.
3. **Retry**: 50 ms không có reply → gửi lại **đúng frame cũ, giữ SEQ**, tối đa 3 lần. F103 nhận
   ra frame lặp và không chạy lại lệnh. Lệnh **mới** phải dùng SEQ mới (tăng SEQ mỗi frame).
4. **F103 reset** (t_ms giảm, hoặc state tự về IDLE): vị trí về 0, telemetry tắt, gain/limits về
   mặc định → chạy lại trình tự khởi động.
5. **Dừng giao tiếp**: gửi STOP rồi ENABLE 0 trước, để không bị lỗi WATCHDOG.

## 7. Trình tự mẫu

```
Khởi động:  PING (kiểm version = 2) → SET_TELEM 50 → [SET_LIMITS] → [SET_GAINS] → ENABLE 1
            → PING mỗi 100 ms
Phân loại:  SET_VEL 80 → theo dõi pos trong telemetry → tới vị trí gạt: SERVO_SET 450
            → SERVO_SET 0 → … → STOP
Đi quãng:   MOVE_REL 200000 → chờ EVT_MOVE_DONE (hoặc state về READY)
Lỗi:        thấy state = FAULT → xử lý nguyên nhân → CLEAR_FAULT → ENABLE 1
Khẩn cấp:   ESTOP (mọi state) → CLEAR_FAULT → ENABLE 1
```

## 8. API C dùng cho H7 (`shared/proto/`)

C11 thuần, không phụ thuộc HAL/RTOS. Chép `proto.h`, `frame.h`, `frame.c` vào project H7.

| API | Chức năng |
|---|---|
| `frame_build(buf, seq, cmd, payload, len)` | Đóng gói frame vào `buf` (39 B), trả về độ dài |
| `frame_parser_init(&p)` | Khởi tạo parser, một lần cho mỗi UART |
| `frame_parse_byte(&p, byte, now_ms, &frame)` | Nạp từng byte nhận được; trả `true` khi có frame hợp lệ trong `frame` (`seq`, `cmd`, `len`, `payload`) |
| `proto_put_u16/i16/u32/i32/f32(p, v)` | Ghi số vào payload (little-endian) |
| `proto_get_u16/i16/u32/i32/f32(p)` | Đọc số từ payload |
| `proto_crc16(data, len)` | Tính CRC riêng (nếu cần) |
| `PROTO_CMD_*`, `PROTO_ST_*`, `PROTO_STATE_*`, `PROTO_FAULT_*`, `PROTO_ACK_FLAG`, `PROTO_TELEM` | Hằng số |

Gửi SET_VEL 50:
```c
uint8_t pl[2], buf[PROTO_FRAME_MAX];
proto_put_i16(pl, 50);
uart_send(buf, frame_build(buf, seq++, PROTO_CMD_SET_VEL, pl, 2));
```

Nhận:
```c
frame_t f;
if (frame_parse_byte(&parser, byte, now_ms, &f)) {
    if (f.cmd == (PROTO_ACK_FLAG | PROTO_CMD_SET_VEL) && f.seq == sent_seq)
        status = f.payload[0];                      /* PROTO_ST_OK, ... */
    else if (f.cmd == PROTO_TELEM)
        pos_um = proto_get_i32(&f.payload[6]);      /* offset theo §5 */
    else if (f.cmd == PROTO_EVT_FAULT)
        faults = f.payload[0];
}
```

Lưu ý: parser bỏ frame nếu hai byte cách nhau > 5 ms. `now_ms` phải là **thời điểm byte tới**;
nếu nhận bằng DMA rồi xử lý sau, truyền thời điểm nhận khối dữ liệu, không phải lúc xử lý.

## 9. Kiểm tra code H7

- `test/vectors/c_frames.hex`: frame do code F103 sinh ra (mỗi dòng một frame hex). Encoder của
  H7 phải ra đúng các byte này, parser phải nhận được (`make -C test vectors` để tạo lại).
- Tham khảo Python: `tools/proto.py` (encode/decode), `tools/host.py` (retry, heartbeat).
