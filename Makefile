# Scene / bake parameters
scene ?= gltf/bunny.glb
w ?= 800
h ?= 600
l ?= 2     # samples / area-light
m ?= 10    # max ray depth
d ?= 1     # debug flag
o ?= 1     # accumulate
R ?= 0     # ReSTIR-GI
S ?= 0     # serializer (0=text,1=binary)
f ?=       # optional explicit output filename
target ?= riscv# or "cuda"

scene_name  = $(basename $(notdir $(scene)))
ext := .cpp
ifeq ($(target),cuda)
  ext       := .cu
endif
baked := $(CURDIR)/bakes/$(scene_name)_$(w)_$(h)$(ext)

LLVM_VORTEX ?= $(HOME)/opt/llvm-vortex
RISCV_32    ?= $(HOME)/opt/riscv32

STARTUP_ADDR ?= 0x80000000

# Source directories
PT_SRC_DIR  ?= src
LIB_DIR ?= lib
PT_TARGET := $(PT_SRC_DIR)/target/pathtracer$(ext)

# Build directories
BUILD_DIR ?= build
VORTEX_LIB_DIR ?= lib
TARGET_DIR := $(BUILD_DIR)/$(scene_name)_$(w)_$(h)_$(target)
APP_EXE := $(BUILD_DIR)/baker
BAKED_OBJ := $(TARGET_DIR)/baked_data.o
PT := $(TARGET_DIR)/pathtracer

.PHONY: all clean libvortex
ifeq ($(target),cuda)
all: $(PT).dump $(PT)
else
all: libvortex $(PT).dump $(PT).bin $(PT).elf
endif

# Build the vortex library for RISC-V target
libvortex: $(VORTEX_LIB_DIR)/libvortex.a

$(VORTEX_LIB_DIR)/libvortex.a:
	@echo "[Make] Building libvortex for RISC-V target"
	@$(MAKE) -C $(LIB_DIR) RISCV_32=$(RISCV_32)

clean:
	@rm -rf $(BUILD_DIR) bakes
	@$(MAKE) -C $(LIB_DIR) clean

$(BUILD_DIR)/build.ninja: CMakeLists.txt
	@echo "[CMake] Configuring (only if first run) ..."
	@cmake -S . -B $(BUILD_DIR) -G Ninja
	@touch $@

$(APP_EXE): $(BUILD_DIR)/build.ninja
	@ninja -C $(BUILD_DIR)

$(baked): $(APP_EXE)
	@mkdir -p $(@D)
	@echo "[Bake]  Cooking scene -> $@"
	@$(APP_EXE) -l $(l) -m $(m) -d $(d) -o $(o) -R $(R) -r $(w) $(h) -S $(S) -f $@ $(scene)


RISCV_CC := $(LLVM_VORTEX)/bin/clang++
RISCV_OBJDUMP := $(LLVM_VORTEX)/bin/llvm-objdump
RISCV_OBJCOPY := $(LLVM_VORTEX)/bin/llvm-objcopy
RISCV_CFLAGS := --target=riscv32-unknown-elf --sysroot=$(RISCV_32)/riscv32-unknown-elf --gcc-toolchain=$(RISCV_32)
RISCV_CFLAGS += -march=rv32imaf -mabi=ilp32f -O3 -std=c++17
RISCV_CFLAGS += -Xclang -target-feature -Xclang +vortex
RISCV_CFLAGS += -mcmodel=medany -fno-rtti -fno-exceptions -fdata-sections -ffunction-sections -mllvm -inline-threshold=262144
RISCV_CFLAGS += -I$(PT_SRC_DIR) -I$(LIB_DIR)/include/
RISCV_CFLAGS += -DLLVM_VORTEX
RISCV_LDFLAGS := -nostartfiles -Wl,-Bstatic,--gc-sections,-T,$(VORTEX_LIB_DIR)/linker/link32.ld,--defsym=STARTUP_ADDR=$(STARTUP_ADDR) $(VORTEX_LIB_DIR)/libvortex.a
RISCV_OBJDUMP_FLAGS = -D --section=.text

CUDA_CC := nvcc
CUDA_OBJDUMP := cuobjdump 
CUDA_FLAGS := -std=c++17 -rdc=true -I$(PT_SRC_DIR)
CUDA_LDFLAGS :=
CUDA_OBJDUMP_FLAGS = --dump-sass

CC := $(RISCV_CC)
OBJDUMP := $(RISCV_OBJDUMP)
CFLAGS := $(RISCV_CFLAGS)
LDFLAGS := $(RISCV_LDFLAGS)
OBJDUMP_FLAGS := $(RISCV_OBJDUMP_FLAGS)
PT_OBJ := $(PT).elf

ifeq ($(target),cuda)
CC := $(CUDA_CC)
OBJDUMP := $(CUDA_OBJDUMP)
CFLAGS := $(CUDA_FLAGS)
LDFLAGS := $(CUDA_LDFLAGS)
OBJDUMP_FLAGS := $(CUDA_OBJDUMP_FLAGS)
PT_OBJ := $(PT)
endif

# Build baked object
$(BAKED_OBJ): $(baked)
	@mkdir -p $(@D)
	@echo "[Compile] $(target) baked data"
	@$(CC) $(CFLAGS) -c $< -o $@

# Link final binary
$(PT_OBJ): $(BAKED_OBJ) $(if $(filter riscv,$(target)),$(VORTEX_LIB_DIR)/libvortex.a)
	@echo "[Link] $(target) pathtracer"
	@$(CC) $(CFLAGS) $(PT_TARGET) $< $(LDFLAGS) -o $@

$(PT).bin: $(PT).elf
	@echo "[Objcopy] $(target) pathtracer.bin"
	@$(RISCV_OBJCOPY) -O binary $< $@

# objdump
$(PT).dump: $(PT_OBJ)
	@echo "[Objdump] $(target) pathtracer"
	@$(OBJDUMP) $(OBJDUMP_FLAGS) $< > $@