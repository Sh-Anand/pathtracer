LLVM ?= $(VORTEX_32)

CFLAGS += -march=rv32imf -mabi=ilp32f
CFLAGS += --target=riscv32-unknown-elf --sysroot=$(RISCV_32)/riscv32-unknown-elf --gcc-toolchain=$(RISCV_32)

include configs/riscv.mk

CFLAGS += -DVORTEX

GPU_SIM_DIR ?= /home/eecs/shashank.anand/vortex/build/sim/rtlsim
GPU_SIM ?= $(GPU_SIM_DIR)/rtlsim
GPU_SIM_BIN ?= $(PT).bin
POST_PROCESS_SCRIPT ?= scripts/pathtrace_postprocess.py
STDOUT_SENTINEL ?= "RAYTRACING_COMPLETE"
STDOUT_SKIP_STR ?= "\#0: "
RENDER_PFM ?= $(TARGET_DIR)/render.pfm
DEBUG ?= 0
RUN_CMD = 	python3 scripts/run_sim.py \
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