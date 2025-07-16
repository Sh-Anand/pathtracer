CC := nvcc
OBJDUMP := cuobjdump
OBJCOPY :=
CFLAGS := -std=c++17 -rdc=true -I$(PT_SRC_DIR)
LDFLAGS :=
OBJDUMP_FLAGS := --dump-sass

# Fedora 42 stupidity
ifeq ($(shell test -f /etc/fedora-release && grep -q "Fedora.*release 42" /etc/fedora-release && echo yes),yes)
NVCC_CCBIN := /usr/bin/g++-14
CC := NVCC_CCBIN=$(NVCC_CCBIN) $(CC)
endif

baked := $(baked).cu
PT_TARGET := $(PT_TARGET).cu

# run params
PT_OBJ := $(PT)
GPU_SIM_BIN := $(PT)
RENDER_PFM ?= $(TARGET_DIR)/render.pfm
RUN_CMD := $(GPU_SIM_BIN) $(R) $(RENDER_PFM)
TARGET_LIB_DEP :=

all: $(PT).dump $(PT)