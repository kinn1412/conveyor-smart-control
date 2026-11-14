# Host unit tests, include check, SIL. Run from the repo root: make -C test [sim|vectors|clean]
CC ?= gcc
PY ?= python
ROOT = ..
APP = $(ROOT)/App
PROTO = $(ROOT)/shared/proto
BUILD = build
EXE = $(if $(filter Windows_NT,$(OS)),.exe,)

INC = -I$(PROTO) -I$(APP) -I$(APP)/ctrl -I$(APP)/comm -I$(APP)/app -Iunity -Isim
CFLAGS = -std=c11 -Wall -Wextra -Wconversion -funsigned-char -g $(INC)

LIB_SRC = $(PROTO)/frame.c $(APP)/ctrl/lpf.c $(APP)/ctrl/pid.c $(APP)/ctrl/traj.c \
          $(APP)/comm/cmd.c $(APP)/app/safety.c $(APP)/app/conveyor.c
TESTS = test_proto test_frame test_cmd test_lpf test_pid test_traj test_safety test_conveyor

# Files that must stay free of HAL / CMSIS / FreeRTOS / drv
PURE = $(PROTO) $(APP)/ctrl $(APP)/comm $(APP)/config.h \
       $(APP)/app/conveyor.c $(APP)/app/conveyor.h $(APP)/app/safety.c $(APP)/app/safety.h

SCN ?= step-vel 50

.PHONY: all check_includes sim vectors clean $(TESTS:%=run_%)

all: check_includes $(TESTS:%=run_%)

check_includes:
	@if grep -rnE '^[[:space:]]*#[[:space:]]*include[[:space:]]*[<"](stm32|main\.h|cmsis|core_cm|FreeRTOS\.h|task\.h|queue\.h|stream_buffer\.h|semphr\.h|motor\.h|encoder\.h|servo\.h|uart_link\.h|timing\.h|led\.h)' $(PURE); \
	then echo "include check FAILED"; exit 1; else echo "include check OK"; fi

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/unity.o: unity/unity.c | $(BUILD)
	$(CC) -std=c11 -g -c -o $@ $<

$(BUILD)/%$(EXE): %.c $(LIB_SRC) $(BUILD)/unity.o sim/plant.h
	$(CC) $(CFLAGS) -o $@ $(filter %.c %.o,$^) -lm

$(TESTS:%=run_%): run_%: $(BUILD)/%$(EXE)
	./$<

$(BUILD)/sim$(EXE): sim/sim.c $(LIB_SRC) sim/plant.h | $(BUILD)
	$(CC) $(CFLAGS) -o $@ $(filter %.c,$^) -lm

sim: $(BUILD)/sim$(EXE)
	./$< $(SCN) > sim/out.csv
	@echo "wrote test/sim/out.csv ($(SCN))"

# C <-> Python vector exchange (REQ-H01): each side writes its frames and checks the other's
vectors: $(BUILD)/test_frame$(EXE)
	./$<
	$(PY) $(ROOT)/tools/tests/test_proto.py
	./$<

clean:
	rm -rf $(BUILD) vectors/*.hex sim/out.csv
