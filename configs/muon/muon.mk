RT_LIB_DIR ?= $(LIB_DIR)/$(target)/baremetal
RT_LIB := $(RT_LIB_DIR)/lib$(target).a

LLVM ?= $(MUON_32)

CFLAGS += -march=rv32im_zfinx -mabi=ilp32 
CFLAGS += --target=riscv32-unknown-elf --sysroot=$(LLVM)/riscv32-unknown-elf -I$(LLVM)/include 

include configs/riscv.mk

CFLAGS += -L$(LLVM)/lib/riscv32-unknown-elf -L$(LLVM)/lib/clang/18/lib/riscv32-unknown-elf -lm -lc -lclang_rt.builtins
CFLAGS += -nostartfiles -nostdlib
CFLAGS += -DMUON

GPU_SIM_DIR ?= /home/shashank/radiance/cyclotron
CYCLOTRON_CONFIG = configs/muon/cyclotron.toml
GPU_SIM ?= scripts/cyclotron_wrapper.sh
GPU_SIM_BIN ?= $(PT).elf
POST_PROCESS_SCRIPT ?= scripts/pathtrace_postprocess.py
STDOUT_SENTINEL ?= "RAYTRACING_COMPLETE"
STDOUT_SKIP_STR ?= ""
RENDER_PFM ?= $(TARGET_DIR)/render.pfm
DEBUG ?= 0
RUN_CMD = 	sed -i 's|elf = ".*"|elf = "$(PT).elf"|' $(CYCLOTRON_CONFIG); \
		python3 scripts/run_sim.py \
		--ssh $(SSH) \
		--user $(SSH_USER) \
		--host $(SSH_HOST) \
		--sim $(GPU_SIM) \
		--bin "$(GPU_SIM_DIR) $(CYCLOTRON_CONFIG)" \
		--build_folder $(TARGET_DIR) \
		--post $(POST_PROCESS_SCRIPT) \
		--sentinel $(STDOUT_SENTINEL) \
		--skip $(STDOUT_SKIP_STR) \
		--outfile $(RENDER_PFM) \
		--debug $(DEBUG)