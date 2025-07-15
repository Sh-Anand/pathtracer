ROOT_DIR := $(realpath ..)

INC_DIR := include
SRC_DIR := src

LLVM_CFLAGS += --sysroot=$(RISCV_32)/riscv32-unknown-elf --gcc-toolchain=$(RISCV_32) --target=riscv32-unknown-elf
LLVM_CFLAGS += -Xclang -target-feature -Xclang +vortex -mllvm -vortex-branch-divergence=0
LLVM_CFLAGS += -DLLVM_VORTEX

CC  = $(LLVM_VORTEX)/bin/clang
AR  = $(LLVM_VORTEX)/bin/llvm-ar
DP  = $(LLVM_VORTEX)/bin/llvm-objdump
CP  = $(LLVM_VORTEX)/bin/llvm-objcopy

CFLAGS += -march=rv32imf -mabi=ilp32f
CFLAGS += -O3 -mcmodel=medany -fno-exceptions -nostartfiles -fdata-sections -ffunction-sections
CFLAGS += -I$(INC_DIR)
CFLAGS += -DXLEN_32
CFLAGS += $(LLVM_CFLAGS)

PROJECT ?= libvortex

SRCS = $(SRC_DIR)/vx_start.S $(SRC_DIR)/vx_syscalls.c $(SRC_DIR)/vx_print.S $(SRC_DIR)/tinyprintf.c $(SRC_DIR)/vx_print.c $(SRC_DIR)/vx_spawn.c $(SRC_DIR)/vx_serial.S $(SRC_DIR)/vx_perf.c

OBJS = $(addsuffix .o, $(notdir $(SRCS)))

all: $(PROJECT).a $(PROJECT).dump

$(PROJECT).dump: $(PROJECT).a
	$(DP) -D $(PROJECT).a > $(PROJECT).dump

%.S.o: $(SRC_DIR)/%.S
	$(CC) $(CFLAGS) -c $< -o $@

%.c.o: $(SRC_DIR)/%.c
	$(CC) $(CFLAGS) -c $< -o $@

$(PROJECT).a: $(OBJS)
	$(AR) rcs $@ $^

.depend: $(SRCS)
	$(CC) $(CFLAGS) -MM $^ > .depend;

clean:
	rm -rf *.a *.o *.dump .depend
