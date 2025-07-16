CC := $(LLVM)/bin/clang++
OBJDUMP := $(LLVM)/bin/llvm-objdump
OBJCOPY := $(LLVM)/bin/llvm-objcopy

STARTUP_ADDR ?= 0x80000000

CFLAGS := --target=riscv32-unknown-elf --sysroot=$(RISCV_32)/riscv32-unknown-elf --gcc-toolchain=$(RISCV_32)
CFLAGS += -march=rv32imf -mabi=ilp32f -O3 -std=c++17
CFLAGS += -Xclang -target-feature -Xclang +vortex
CFLAGS += -mcmodel=medany -fno-rtti -fno-exceptions -fdata-sections -ffunction-sections -mllvm -inline-threshold=262144
CFLAGS += -I$(PT_SRC_DIR) -I$(RT_LIB_DIR)/include/
CFLAGS += -DLLVM_VORTEX

LDFLAGS := -nostartfiles -Wl,-Bstatic,--gc-sections,-T,$(RT_LIB_DIR)/linker/link32.ld,--defsym=STARTUP_ADDR=$(STARTUP_ADDR) $(RT_LIB)

OBJDUMP_FLAGS := -D --section=.init --section=.text

PT_OBJ := $(PT).elf
TARGET_LIB_DEP := $(RT_LIB)

baked := $(baked).cpp
PT_TARGET := $(PT_TARGET).cpp

all: lib$(target) $(PT).dump $(PT).bin $(PT).elf
