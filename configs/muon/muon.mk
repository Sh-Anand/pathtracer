LLVM ?= /home/shashank/radiance/radiance-llvm/build

CFLAGS += -march=rv32im_zfinx -mabi=ilp32

include configs/riscv.mk

CFLAGS += -nostartfiles -nostdlib
CFLAGS += -DMUON

GPU_SIM_DIR ?= /home/shashank/radiance/cyclotron
CYCLOTRON_CONFIG = configs/muon/cyclotron.toml
RUN_CMD += sed -i 's|elf = ".*"|elf = "$(PT).elf"|' $(CYCLOTRON_CONFIG);
RUN_CMD += RUST_BACKTRACE=1 cargo run --manifest-path $(GPU_SIM_DIR)/Cargo.toml ${CYCLOTRON_CONFIG}