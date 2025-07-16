CFLAGS += -march=rv32imf -mabi=ilp32f

LLVM ?= /home/shashank/radiance/radiance-llvm/build/

LLVM_CFLAGS += --sysroot=$(RISCV_32)/riscv32-unknown-elf --gcc-toolchain=$(RISCV_32) --target=riscv32-unknown-elf
LLVM_CFLAGS += -Xclang -target-feature -Xclang +vortex -mllvm -vortex-branch-divergence=0
#LLVM_CFLAGS += -I$(RISCV_SYSROOT)/include/c++/9.2.0/$(RISCV_PREFIX) 
#LLVM_CFLAGS += -I$(RISCV_SYSROOT)/include/c++/9.2.0
#LLVM_CFLAGS += -Wl,-L$(RISCV_TOOLCHAIN_PATH)/lib/gcc/$(RISCV_PREFIX)/9.2.0
#LLVM_CFLAGS += --rtlib=libgcc

#CC  = $(LLVM_VORTEX)/bin/clang $(LLVM_CFLAGS)
#CXX = $(LLVM_VORTEX)/bin/clang++ $(LLVM_CFLAGS)
#DP  = $(LLVM_VORTEX)/bin/llvm-objdump
#CP  = $(LLVM_VORTEX)/bin/llvm-objcopy

# CC  = $(RISCV_TOOLCHAIN_PATH)/bin/$(RISCV_PREFIX)-gcc
# CXX = $(RISCV_TOOLCHAIN_PATH)/bin/$(RISCV_PREFIX)-g++
AR  = $(RISCV_32)/bin/riscv32-unknown-elf-gcc-ar
# DP  = $(RISCV_TOOLCHAIN_PATH)/bin/$(RISCV_PREFIX)-objdump
# CP  = $(RISCV_TOOLCHAIN_PATH)/bin/$(RISCV_PREFIX)-objcopy

MU_CC  = $(LLVM)/bin/clang $(LLVM_CFLAGS)
MU_CXX = $(LLVM)/bin/clang++ $(LLVM_CFLAGS)
MU_DP  = $(LLVM)/bin/llvm-objdump
MU_CP  = $(LLVM)/bin/llvm-objcopy

CFLAGS += -O3 -mcmodel=medany -fno-exceptions -nostartfiles -fdata-sections -ffunction-sections
CFLAGS += -I./include
CFLAGS += -DXLEN_$(XLEN)

VX_CFLAGS := $(CFLAGS)
VX_CFLAGS += -mllvm -inline-threshold=262144
VX_CFLAGS += -I$(VORTEX_KN_PATH)/include
VX_CFLAGS += -DNDEBUG -DLLVM_VORTEX

MU_CFLAGS := $(VX_CFLAGS)
MU_CFLAGS += -fuse-ld=lld

PROJECT = libmuon

SRCS = ./src/vx_start.S ./src/vx_syscalls.c ./src/tinyprintf.c ./src/vx_spawn.c

OBJS := $(addsuffix .o, $(notdir $(SRCS)))

all: $(PROJECT).a $(PROJECT).dump

$(PROJECT).dump: $(PROJECT).a
	$(MU_DP) -D $(PROJECT).a > $(PROJECT).dump

%.S.o: src/%.S
	$(MU_CC) $(MU_CFLAGS) -c $< -o $@

%.cpp.o: src/%.cpp include/vx_spawn.h
	$(MU_CXX) $(MU_CFLAGS) -c $< -o $@

%.c.o: src/%.c include/vx_spawn.h
	$(MU_CC) $(MU_CFLAGS) -c $< -o $@

$(PROJECT).a: $(OBJS)
	$(AR) rcs $@ $^

.depend: $(SRCS)
	$(MU_CC) $(MU_CFLAGS) -MM $^ > .depend;

clean:
	rm -rf *.a *.o *.dump .depend
