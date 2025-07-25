LLVM ?= $(MUON)

CFLAGS += -march=rv32im_zfinx -mabi=ilp32 
CFLAGS += --target=riscv32-unknown-elf --sysroot=$(MUON)/riscv32-unknown-elf -I$(MUON)/include 

include configs/riscv.mk

CFLAGS += -L$(MUON)/lib/riscv32-unknown-elf -lm
CFLAGS += -nostartfiles -nostdlib
CFLAGS += -DMUON

GPU_SIM_DIR ?= /home/shashank/radiance/cyclotron
CYCLOTRON_CONFIG = configs/muon/cyclotron.toml
RUN_CMD += sed -i 's|elf = ".*"|elf = "$(PT).elf"|' $(CYCLOTRON_CONFIG);
RUN_CMD += RUST_BACKTRACE=1 cargo run --manifest-path $(GPU_SIM_DIR)/Cargo.toml ${CYCLOTRON_CONFIG}