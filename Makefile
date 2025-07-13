# Scene / bake parameters
scene ?= gltf/bunny.glb
w ?= 32
h ?= 32
l ?= 2# samples / area-light
m ?= 10# max ray depth
d ?= 1# debug flag
o ?= 1# accumulate
R ?= 0# ReSTIR-GI
S ?= 0# serializer (0=text,1=binary)
f ?=  # optional explicit output filename
target ?= riscv# or "cuda"

# Project configuration
PROJECT_NAME := pathtracer
scene_name := $(basename $(notdir $(scene)))
ext := .cpp
ifeq ($(target),cuda)
  ext := .cu
endif
baked = $(CURDIR)/bakes/$(scene_name)_$(w)_$(h)_$(R)$(ext)

# Tool paths
LLVM_VORTEX ?= $(HOME)/opt/llvm-vortex
RISCV_32 ?= $(HOME)/opt/riscv32
STARTUP_ADDR ?= 0x80000000

# Source directories
PT_SRC_DIR ?= src
LIB_DIR ?= lib
PT_TARGET := $(PT_SRC_DIR)/target/$(PROJECT_NAME)$(ext)

# Build directories
BUILD_DIR ?= build
VORTEX_LIB_DIR ?= lib
TARGET_DIR = $(BUILD_DIR)/$(scene_name)_$(w)_$(h)_$(R)_$(target)
APP_EXE := $(BUILD_DIR)/baker
BAKED_OBJ := $(TARGET_DIR)/baked_data.o
PT := $(TARGET_DIR)/$(PROJECT_NAME)

# SSH parameters
SSH_HOST ?= a5.millennium.berkeley.edu
SSH_USER ?= shashank.anand

# Run sim parameters
SSH ?= 0
GPU_SIM_DIR ?= /home/eecs/shashank.anand/vortex/build/sim/rtlsim
GPU_SIM ?= $(GPU_SIM_DIR)/rtlsim
GPU_SIM_BIN ?= $(PT).bin
POST_PROCESS_SCRIPT ?= scripts/pathtrace_postprocess.py
STDOUT_SENTINEL ?= "RAYTRACING_COMPLETE"
STDOUT_SKIP_STR ?= "\#0: "
RENDER_PFM ?= $(TARGET_DIR)/render.pfm
DEBUG ?= 0

# RISC-V toolchain
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
RISCV_OBJDUMP_FLAGS := -D --section=.text

# CUDA toolchain
CUDA_CC := nvcc
CUDA_OBJDUMP := cuobjdump 
CUDA_FLAGS := -std=c++17 -rdc=true -I$(PT_SRC_DIR)
CUDA_LDFLAGS :=
CUDA_OBJDUMP_FLAGS := --dump-sass

# Platform-specific fixes
ifeq ($(shell test -f /etc/fedora-release && grep -q "Fedora.*release 42" /etc/fedora-release && echo yes),yes)
NVCC_CCBIN := /usr/bin/g++-14
endif

# Target-specific toolchain selection
CC := $(RISCV_CC)
OBJDUMP := $(RISCV_OBJDUMP)
CFLAGS := $(RISCV_CFLAGS)
LDFLAGS := $(RISCV_LDFLAGS)
OBJDUMP_FLAGS := $(RISCV_OBJDUMP_FLAGS)
PT_OBJ := $(PT).elf

ifeq ($(target),cuda)
CC := NVCC_CCBIN=$(NVCC_CCBIN) $(CUDA_CC)
OBJDUMP := $(CUDA_OBJDUMP)
CFLAGS := $(CUDA_FLAGS)
LDFLAGS := $(CUDA_LDFLAGS)
OBJDUMP_FLAGS := $(CUDA_OBJDUMP_FLAGS)
PT_OBJ := $(PT)
endif

.PHONY: all relink libvortex clean clean-link clean-libvortex run-sim run-sim-ssh

ifeq ($(target),cuda)
all: $(PT).dump $(PT)
else
all: libvortex $(PT).dump $(PT).bin $(PT).elf
endif

# Build targets
libvortex: $(VORTEX_LIB_DIR)/libvortex.a

$(VORTEX_LIB_DIR)/libvortex.a:
	@echo "[Make] Building libvortex for RISC-V target"
	@$(MAKE) -C $(LIB_DIR) RISCV_32=$(RISCV_32)

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

$(BAKED_OBJ): $(baked)
	@mkdir -p $(@D)
	@echo "[Compile] $(target) baked data"
	@$(CC) $(CFLAGS) -c $< -o $@

$(PT_OBJ): $(BAKED_OBJ) $(if $(filter riscv,$(target)),$(VORTEX_LIB_DIR)/libvortex.a)
	@echo "[Link] $(target) $(PROJECT_NAME)"
	@$(CC) $(CFLAGS) $(PT_TARGET) $< $(LDFLAGS) -o $@

$(PT).bin: $(PT).elf
	@echo "[Objcopy] $(target) $(PROJECT_NAME).bin"
	@$(RISCV_OBJCOPY) -O binary $< $@

$(PT).dump: $(PT_OBJ)
	@echo "[Objdump] $(target) $(PROJECT_NAME)"
	@$(OBJDUMP) $(OBJDUMP_FLAGS) $< > $@

# Simulation targets
run-sim: $(PT).bin
	@python3 scripts/run_sim.py \
		--ssh $(SSH) \
		--user $(SSH_USER) \
		--host $(SSH_HOST) \
		--sim $(GPU_SIM) \
		--bin $(GPU_SIM_BIN) \
		--build_folder $(TARGET_DIR) \
		--post $(POST_PROCESS_SCRIPT) \
		--sentinel $(STDOUT_SENTINEL) \
		--skip $(STDOUT_SKIP_STR) \
		--outfile $(RENDER_PFM) \
		--debug $(DEBUG)

run-sim-ssh:
	@GPU_SIM_BIN=$(GPU_SIM_DIR)/$(PROJECT_NAME).bin; \
	scp $(PT).bin $(SSH_USER)@$(SSH_HOST):$$GPU_SIM_BIN; \
	$(MAKE) SSH=1 GPU_SIM_BIN=$$GPU_SIM_BIN run-sim

# Utility targets
relink: clean-link
	@$(MAKE) all

clean-libvortex:
	@$(MAKE) -C $(LIB_DIR) clean

clean-link:
	@rm -f $(PT)*

clean:
	@rm -rf $(BUILD_DIR) bakes
	@$(MAKE) -C $(LIB_DIR) clean
