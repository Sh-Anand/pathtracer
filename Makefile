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

# Tool-chain roots
LLVM_VORTEX ?= $(HOME)/opt/llvm-vortex
RISCV_32    ?= $(HOME)/opt/riscv32

# Derived names
scene_name  = $(basename $(notdir $(scene)))

ext := .cpp
ifeq ($(target),cuda)
  ext       := .cu
endif

baked := $(CURDIR)/bakes/$(scene_name)_$(w)_$(h)$(ext)

# Build directories
BUILD_DIR ?= build
TARGET_DIR := $(BUILD_DIR)/$(scene_name)_$(w)_$(h)_$(target)
APP_EXE := $(BUILD_DIR)/baker
BAKED_OBJ := $(TARGET_DIR)/baked_data.o
FINAL_BIN := $(TARGET_DIR)/pathtracer

# Source directories
PT_SRC_DIR  ?= src
PT_TARGET_CPP = $(PT_SRC_DIR)/target/pathtracer.cpp
PT_TARGET_CU  = $(PT_SRC_DIR)/target/pathtracer.cu

.PHONY: all clean
all: $(FINAL_BIN)

clean:
	@rm -rf $(BUILD_DIR) bakes

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

# Set up tool-chain variables
RISCV_CC := $(LLVM_VORTEX)/bin/clang++
RISCV_FLAGS := --target=riscv32-unknown-elf --sysroot=$(RISCV_32)/riscv32-unknown-elf --gcc-toolchain=$(RISCV_32)
RISCV_FLAGS += -march=rv32imaf -mabi=ilp32f -O3
RISCV_FLAGS += -Xclang -target-feature -Xclang +vortex
RISCV_FLAGS += -fno-rtti -fno-exceptions -fdata-sections -ffunction-sections -mllvm -inline-threshold=262144
RISCV_FLAGS += -I $(PT_SRC_DIR)

CUDA_CC := nvcc
CUDA_FLAGS := -std=c++17 -rdc=true -I $(PT_SRC_DIR)

CC := $(RISCV_CC)
CFLAGS := $(RISCV_FLAGS)
PT_TARGET := $(PT_TARGET_CPP)

ifeq ($(target),cuda)
CC := $(CUDA_CC)
CFLAGS := $(CUDA_FLAGS)
PT_TARGET := $(PT_TARGET_CU)
endif

# Build baked object
$(BAKED_OBJ): $(baked)
	@mkdir -p $(@D)
	@echo "[Compile] $(target) baked data"
	@$(CC) $(CFLAGS) -c $< -o $@

# Link final binary
$(FINAL_BIN): $(BAKED_OBJ)
	@mkdir -p $(@D)
	@echo "[Link] $(target) pathtracer"
	@$(CC) $(CFLAGS) $^ $(PT_TARGET) -o $@