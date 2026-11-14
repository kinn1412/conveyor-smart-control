# Conveyor Actuator Firmware

STM32F103C8T6 + FreeRTOS firmware for a belt conveyor: one DC motor (JGB37-520, 12 V, 107 rpm,
MDD10A driver) drives the belt, and an MG996R servo moves the sorting gate. A host (PC Python during tuning,
STM32H7 later) sends commands over UART. The firmware controls belt velocity and position, runs
trapezoidal moves, positions the servo, and enforces safety.

Course project for Automatic Control: the deliverables are control performance (POT, t_s, e_ss,
tracking error) and the report.

## Status

- Firmware, unit tests, SIL and Python tools are written. Host tests pass.
- STM32CubeIDE project at the repo root with FreeRTOS, USART1 + DMA, PWM, servo and LED; the `.ioc`
  still misses the TIM4 encoder ([docs/design.md §5](docs/design.md)). Everything else compiles
  with the CubeIDE GCC.
- TBD values in `App/config.h`: `GEAR_RATIO` (1:90 assumed), `ROLLER_D_MM`, gains, `u_dz`, servo limits (spec §11).

## Layout

```
bsw_conveyor_control.ioc, Core/, Drivers/   CubeMX / STM32CubeIDE project
App/                 application code (source folder in .cproject)
  config.h           constants: geometry, limits, default gains, task settings
  app/               tasks.c (FreeRTOS), conveyor.c (state machine, control step), safety.c
  ctrl/              pid.c, traj.c, lpf.c            pure C
  comm/              cmd.c                           pure C
  drv/               motor, encoder, servo, uart_link, timing, led   (HAL)
shared/proto/        proto.h, frame.c/h              pure C, shared with the H7 project
test/                Unity unit tests, sim/ (SIL), vectors/
tools/               proto.py, host.py, analyze.py, hwtest.py
```

`ctrl/`, `comm/`, `shared/proto/`, `conveyor.*` and `safety.*` do not include HAL, CMSIS or
FreeRTOS headers, so they build and run on the PC. `make -C test` enforces this.

## Requirements

- STM32CubeMX 6.x, STM32CubeIDE, STM32CubeF1 V1.8.7
- Host tests: `gcc`, GNU make (on Windows/MSYS2: `mingw32-make`)
- Python 3.10+: `pip install -r tools/requirements.txt`

## Build and flash

1. Open `bsw_conveyor_control.ioc` and apply the settings in [docs/design.md §5](docs/design.md),
   then regenerate the code. `App/` and `shared/` are already source folders in `.cproject`.
2. In STM32CubeIDE: File > Import > Existing Projects into Workspace > this folder.
3. Build (Debug), then Run/Debug with the ST-Link (SWD). Or from the command line:

```
STM32_Programmer_CLI -c port=SWD -w Debug/bsw_conveyor_control.elf -v -rst
```

## Tests on the PC

```
make -C test                         # include check + unit tests
make -C test vectors                 # C <-> Python frame vectors
make -C test sim SCN="step-vel 50"   # SIL -> test/sim/out.csv
make -C test sim SCN="move 200"      # plant parameters can follow: "move 200 0.12 0.05 60" (K tau dz)
python tools/tests/test_proto.py
```

## Hardware tests

Driver bring-up without UART: set `APP_BRINGUP 1` in `App/config.h`, flash, and watch the `bu_*`
variables in STM32CubeIDE Live Expressions (see `App/app/bringup.c`). Set it back to 0 for the
normal firmware and the Python tests below.

`hwtest.py` runs each check over the protocol and prints PASS/FAIL. Motor tests limit duty to
300 permille (bring-up rule).

```
python tools/hwtest.py --port COM5 pwm [--motor-disconnected]    # duty for the multimeter
python tools/hwtest.py --port COM5 encoder                       # hand-turn 1 rev = ENC_CPR +- 4
python tools/hwtest.py --port COM5 motor                         # DIR_SIGN / ENC_SIGN check
python tools/hwtest.py --port COM5 servo [--dwell 1]             # angle sweep
python tools/hwtest.py --port COM5 link                          # 1000 requests, losses, duplicates
python tools/hwtest.py --port COM5 watchdog
python tools/hwtest.py --port COM5 estop                         # latency <= 2 ms
python tools/hwtest.py --port COM5 diag [--time 1800]            # timing, stacks, frame errors
```

## Running the conveyor

Single commands (`python tools/host.py --help` lists all):

```
python tools/host.py --port COM5 ping
python tools/host.py --port COM5 get-status
python tools/host.py --port COM5 clear-fault
```

The actuator faults if it gets no frame for 300 ms in READY or RUNNING. A single command exits
right away, so use `shell` for anything that moves the belt. It sends a PING every 100 ms:

```
python tools/host.py --port COM5 shell
> enable 1
> set-vel 50
> stop
> move-rel 200
> servo-set 30
> quit
```

## Experiments

Each scenario sets telemetry to 100 Hz and writes one CSV per run to `runs/`.

```
python tools/host.py --port COM5 sysid                       # open-loop steps 300/500/700, both directions
python tools/analyze.py --fit runs/sysid_+*.csv runs/sysid_-*.csv   # K, tau, u_dz
python tools/analyze.py --dz runs/sysid_dz_pos.csv runs/sysid_dz_neg.csv

python tools/host.py --port COM5 step-vel --v 50 --gains 4 80 2 0    # PERF-01
python tools/host.py --port COM5 move --dist 200                     # PERF-02
python tools/host.py --port COM5 disturbance --v 50                  # PERF-03, add load by hand

python tools/analyze.py runs/step_vel_50_*.csv --sim test/sim/out.csv
```

`analyze.py` prints t_r, t_s, POT and e_ss for velocity runs, or tracking error max/RMS and final
error for moves, with the mean and standard deviation over runs. It then plots the runs with the
SIL overlay (`--save fig.png` writes the plot to a file).

## Protocol summary

UART 115200 8N1. Frame `AA 55 LEN SEQ CMD PAYLOAD[LEN] CRC_L CRC_H`, CRC16-CCITT (0x1021, init
0xFFFF) over LEN..PAYLOAD, little-endian fields. Every valid frame gets one ACK (`0x80 | CMD`, same
SEQ, status byte first). Full command table, telemetry and diagnostics layout: spec §6.
`tools/proto.py` is the Python reference implementation.
