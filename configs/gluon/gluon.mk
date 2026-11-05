# gluon command processor: nothing is baked.  The host loads the scene, uploads
# it and launches the kernel through gluon's driver.
GLUON_ROOT ?= $(abspath $(CURDIR)/../../..)
LLVM ?= $(MUON_32)
TRIPLE := riscv32-unknown-elf
LIBMUON := $(GLUON_ROOT)/sw/libmuon/gluon

GLUON_BUILD := $(BUILD_DIR)/gluon
KERNEL_ELF := $(GLUON_BUILD)/kernel.elf
HOST_BUILD := $(GLUON_BUILD)/host
HOST_EXE := $(HOST_BUILD)/pathtracer_gluon

# same flags as gluon's sw/test kernels, plus the pathtracer's C++ and inlining flags
KERNEL_CFLAGS := --target=$(TRIPLE) -march=rv32im_zfinx -mabi=ilp32 --sysroot=$(LLVM)/$(TRIPLE)
KERNEL_CFLAGS += -O3 -std=c++17 -fno-rtti -fno-exceptions -fno-builtin-memset -fno-builtin-memcpy
KERNEL_CFLAGS += -ffunction-sections -fdata-sections -fPIC -fno-unwind-tables -fno-asynchronous-unwind-tables
KERNEL_CFLAGS += -Xclang -target-feature -Xclang +vortex -mllvm -inline-threshold=262144
KERNEL_CFLAGS += -DGLUON -I$(PT_SRC_DIR) -I$(LIBMUON)/include -isystem $(LLVM)/include/$(TRIPLE)
KERNEL_LDFLAGS := -fuse-ld=lld -nostdlib -Wl,-melf32lriscv -Wl,-pie -L$(LLVM)/lib/$(TRIPLE)
KERNEL_LIBS := $(LIBMUON)/libmuon.a -lm -lc $(LLVM)/lib/clang/18/lib/$(TRIPLE)/libclang_rt.builtins.a

all: $(KERNEL_ELF) $(HOST_EXE)

$(KERNEL_ELF): src/target/gluon/kernel.cpp src/target/gluon/scene.h $(wildcard src/target/*.h src/scene/*.h src/util/*.h) $(LIBMUON)/libmuon.a
	@mkdir -p $(@D)
	@echo "[Link] gluon kernel"
	@$(LLVM)/bin/clang++ $(KERNEL_CFLAGS) $< $(KERNEL_LDFLAGS) $(KERNEL_LIBS) -o $@

$(LIBMUON)/libmuon.a:
	@$(MAKE) -C $(LIBMUON)

$(HOST_BUILD)/build.ninja: CMakeLists.txt
	@cmake -S . -B $(HOST_BUILD) -G Ninja -DCMAKE_BUILD_TYPE=Release -DGLUON_ROOT=$(GLUON_ROOT)

# always ask ninja: it tracks the host's sources, gluon's driver included
.PHONY: $(HOST_EXE)
$(HOST_EXE): $(HOST_BUILD)/build.ninja
	@ninja -C $(HOST_BUILD) pathtracer_gluon
