# Design — Conveyor Actuator

Status: implemented (2026-11-14), pending user review. Requirements: `spec.md`. Tasks: `plan.md`.
The headers are the API reference; this file records the decisions and how the parts fit.

---

## 1. Decisions

| # | Topic | Decision | Reason |
|---|---|---|---|
| D1 | CRC16 | `static inline` in `shared/proto/proto.h` | One header to copy into the H7 project |
| D2 | Parser 5 ms gap | `now_ms` passed with every byte; parser compares with the previous byte | Parser stays pure C, no extra poll call |
| D3 | State check for motion commands | CommRxTask checks against a state snapshot and ACKs; ControlTask re-checks with `cmd_allowed()` when it dequeues and drops the command if the state changed | Matches spec §3.2 (CommRxTask sends ACK) |
| D4 | `cmd.c` shape | One pure function `cmd_handle()` returns an action; duplicate filter lives in `cmd_ctx_t` | Host-testable, one entry point |
| D5 | Number type in `ctrl/` | `float`; `double` only to convert counts to um | Gains are f32 on the wire; float loses um precision past 16 M counts |
| D6 | Velocity ramp | Inside `traj.c` (`traj_move`, `traj_ramp_to`, shared `traj_step`) | VEL, MOVE and STOP use one reference generator |
| D7 | `app/` split | `conveyor.c`, `safety.c` pure C; only `tasks.c` calls FreeRTOS and `drv/` | Unit tests and SIL run the real state machine |
| D8 | Position accumulation (REQ-M01) | `encoder_read()` returns raw 16-bit count; delta, sign and sum in `conveyor.c` | Testable on host |
| D9 | GET_STATUS / GET_DIAG data | ControlTask writes snapshots; CommRxTask copies them inside `taskENTER_CRITICAL` | Small structs, short critical section |
| D10 | CubeMX integration | See §5 | No `.ioc` edits beyond what CubeMX generates |
| D11 | C unit tests | Unity v2.6.0, vendored in `test/unity/` | Standard embedded C framework |
| D12 | C/Python shared vectors | `test_frame` writes `test/vectors/c_frames.hex` and checks `py_frames.hex`; `tools/tests/test_proto.py` does the reverse | Each side checks the other's real output |
| D13 | Fault handlers | HardFault, `configASSERT`, stack overflow call `app_fault()`: TIM3 CCR1/CCR2 = 0 by register, LED on, halt | Motors stop even when HAL state is broken |
| D14 | Debug output | No printf. Telemetry and `GET_DIAG` only. USART2 (PA2/PA3) configured, unused | No blocking I/O in tasks |
| D15 | Formatting | `.clang-format`: LLVM base, indent 4, column 100, Linux braces | |
| D16 | Python tools | `proto.py`, `host.py`, `analyze.py`, `hwtest.py`; tests with stdlib `unittest` (no pytest dependency) | |
| D17 | UART ISR and FreeRTOS | `uart_link.c` takes callbacks; the callbacks in `tasks.c` call `...FromISR` | Keeps "only `app/` calls FreeRTOS" |
| D18 | Direction signs | `DIR_SIGN`, `ENC_SIGN` in `config.h`, applied in `conveyor.c`; `motor_set()` writes the physical duty | Telemetry `duty` shows what the driver gets |
| D19 | `g_estop` | Event flag: CommRxTask sets it and writes duty 0; ControlTask consumes it at the start of its cycle and latches FAULT ESTOP | ControlTask has the highest priority, so read-and-clear cannot race |
| D20 | One motor (spec v1.1) | SLIP and ENCODER faults removed; a dead encoder shows up as STALL (PI saturates, v = 0) | Both faults need two encoders |
| D21 | Watchdog in RUNNING | Start the STOP ramp; FAULT is latched on the first cycle in READY | Spec §8 "ramp stop, then FAULT" |
| D22 | STOP in OPENLOOP | Duty 0 at once, READY | No velocity reference to ramp in open loop |
| D23 | SET_VEL 0 | Same as STOP (ramp to 0, then READY) | Spec §6.4 "0 = ramp stop" |
| D24 | Status LED | `drv/led.c`, on while in FAULT | Spec §2.3 PC13 status |
| D25 | Host SEQ | `host.py` skips a SEQ that equals the last executed non-exempt command | After 256 frames a new command could look like a retry (REQ-P07) |
| D26 | Hardware tests | `tools/hwtest.py` over the protocol, no separate test firmware | One firmware image; tests are repeatable from the PC |

## 2. Open items adopted (spec text still to be updated by the user)

| # | Item | Adopted |
|---|---|---|
| Q1 | Spec §6.7 `state` has no numeric values | BOOT 0, IDLE 1, READY 2, RUNNING 3, FAULT 4 (`proto.h`, `proto.py`). Needs a line in spec §7 |
| Q2 | CmdQueue full: REQ-P04 still needs a reply | Reply BAD_STATE |
| Q3 | "STOP handled at once" (§3.2) | STOP goes to the front of CmdQueue |
| Q4 | SET_VEL above `v_max` from SET_LIMITS | Accepted; ramp target clamped to `v_max` |
| Q5 | SET_GAINS with NaN, inf or negative values | OUT_OF_RANGE |
| Q6 | SET_LIMITS `v_max = 0` | OUT_OF_RANGE (`1 <= v_max <= V_MAX`) |

---

## 3. Layout and dependencies

```
Core/, Drivers/    CubeMX (STM32CubeIDE project at the repo root, .ioc, .cproject)
App/               source folder in .cproject
  config.h         constants (pure)
  app/  tasks.c/.h          FreeRTOS objects, tasks, ISR callbacks, app_fault   (FreeRTOS + drv)
        conveyor.c/.h       state machine, measurement, control step             (pure C)
        safety.c/.h         fault detection                                      (pure C)
  ctrl/ pid, traj, lpf                                                           (pure C)
  comm/ cmd                                                                      (pure C)
  drv/  motor (TIM3 CH1 + DIR1), encoder (TIM4), servo, uart_link, timing, led  (HAL / CMSIS)
shared/proto/  proto.h (constants, CRC, LE helpers, telemetry/diag pack), frame.c/.h
test/          Makefile, unity/, test_*.c, sim/ (plant.h, sim.c), vectors/
tools/         proto.py, host.py, analyze.py, hwtest.py, tests/test_proto.py
```

```
tasks.c  --> conveyor, cmd, frame, drv/*, FreeRTOS
conveyor --> safety, pid, traj, lpf, cmd (cmd_t, cmd_allowed), proto.h, config.h
cmd      --> frame, proto.h, config.h
ctrl/*   --> config.h, <math.h>
drv/*    --> HAL (main.h), config.h
```

`make -C test` fails if `shared/proto/`, `ctrl/`, `comm/`, `config.h`, `conveyor.*` or `safety.*`
include a HAL, CMSIS, FreeRTOS or `drv/` header.

## 4. Runtime data flow

```
USART1 RX DMA (64 B circular) --HT/TC/IDLE--> uart_link --on_rx()--> RxStream (256 B)
CommRxTask: frame_parse_byte(now_ms) -> cmd_handle(snapshot)
   QUEUE      -> CmdQueue (STOP to the front)
   ESTOP      -> g_estop = 1, motor_set(0), latency stat
   DIAG_RESET -> clear min/max stats
   reply      -> TxQueue (waits up to 10 ms for a slot, REQ-P04); last_rx_tick = now
ControlTask (10 ms): consume g_estop; period stats; drain CmdQueue -> conveyor_apply()
   encoder_read -> conveyor_step() -> motor_set, servo_set_us, led_set
   EVT_FAULT / EVT_MOVE_DONE / telemetry -> TxQueue (timeout 0); snapshots; exec stat
TxTask: TxQueue -> uart_link_send() -> wait on_tx_done notification (10 ms max)
```

Control step (`conveyor_step`): measure (REQ-M01..M03) -> ESTOP -> safety (spec §8) -> mode:
VEL (ramp + PI), MOVE (trapezoid + `v_ff + Kp_p e`, clamp `V_MAX`, done check REQ-T02),
OPENLOOP (duty). Duty = round(u) x `DIR_SIGN`.

## 5. CubeMX / STM32CubeIDE integration

The CubeMX project is at the repo root (toolchain STM32CubeIDE, FW_F1 V1.8.7, FreeRTOS 10.3.1).

Generated and integrated (2026-11-14):
- `.cproject` (Debug and Release): source folders `App`, `shared`; include paths `../App`,
  `../App/app`, `../App/ctrl`, `../App/comm`, `../App/drv`, `../shared/proto`.
  CubeMX regeneration keeps them; check after each regeneration.
- `main.c`: `#include "tasks.h"`; `app_init()` in `RTOS_THREADS`; `StartDefaultTask` calls
  `osThreadExit()`; `Error_Handler` calls `app_fault()`.
- `freertos.c`: stack overflow and malloc failed hooks call `app_fault()`.
- `FreeRTOSConfig.h` `USER CODE 1`: `configASSERT` calls `app_fault()`.
- `stm32f1xx_it.c`: `HardFault_Handler` calls `app_fault()`.
- Labels used by `drv/`: `DIR1` (PB12), `LEDR` (PC13). `DIR2` (PB13) is configured but unused.
- NVIC USART1 / DMA1 Ch4 / Ch5 = 5, `configUSE_TIMERS` = 1 (CubeMX default): both accepted.
- Checked with the STM32CubeIDE GCC 14.3: all sources compile with `-Wall -Wextra`; link
  needs `htim4` (below). Size at -O0: flash 43.7 KB, RAM 15.1 KB.

Still missing in the `.ioc` (user applies in CubeMX):

| Item | Setting |
|---|---|
| TIM4 | Combined Channels: Encoder Mode TI1 and TI2; Period 65535; IC1/IC2 Filter 15; PB6/PB7 GPIO pull-up |

## 6. Tests and tools

| Command | Covers |
|---|---|
| `make -C test` | include check; Unity tests for proto, frame, cmd, lpf, pid, traj, safety, conveyor |
| `make -C test vectors` | C <-> Python frame vectors (REQ-H01) |
| `make -C test sim SCN="step-vel 50"` / `SCN="move 200"` | SIL -> `test/sim/out.csv`; plant args `K tau dz` after the scenario |
| `python tools/tests/test_proto.py` | `proto.py` |
| `python tools/host.py --port COM5 <cmd>` | every §6.4 command, `shell`, `log`, scenarios `sysid`, `step-vel`, `move`, `disturbance` |
| `python tools/analyze.py run.csv [--sim out.csv]` | t_r, t_s, POT, e_ss or tracking max/RMS and final error; plots |
| `python tools/analyze.py --fit runs/sysid_+*.csv runs/sysid_-*.csv` | K, tau, u_dz per direction |
| `python tools/hwtest.py --port COM5 <test>` | `pwm`, `encoder`, `motor`, `servo`, `link`, `watchdog`, `estop`, `diag` |

On Windows with MSYS2, `make` is `mingw32-make`. `host.py --port socket://host:port` also works.

## 7. Gain calculation (plan 3.2)

TODO(user): plant K, tau, u_dz from plan 2.5; tau_cl choice; Kp_v, Ki_v; Ts check; Kp_p.
Placeholders in `config.h`: Kp_v 2.5, Ki_v 50, Kp_p 2, u_dz 0 (assume K ~ 0.24 mm/s/permille, tau ~ 50 ms).
