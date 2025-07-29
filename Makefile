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
#[cuda, muon, vortex]
target ?= muon

# Project configuration
PROJECT_NAME := pathtracer
scene_name := $(basename $(notdir $(scene)))
baked = $(CURDIR)/bakes/$(scene_name)_$(w)_$(h)_$(R)

# Source directories
PT_SRC_DIR ?= src
LIB_DIR ?= lib
PT_TARGET := $(PT_SRC_DIR)/target/$(target)/$(PROJECT_NAME)
RT_LIB_DIR ?= $(LIB_DIR)/$(target)

# Build directories
BUILD_DIR ?= build
TARGET_DIR = $(BUILD_DIR)/$(scene_name)_$(w)_$(h)_$(R)_$(target)
APP_EXE := $(BUILD_DIR)/baker
BAKED_OBJ := $(CURDIR)/bakes/$(scene_name)_$(w)_$(h)_$(R)_$(target).o
PT := $(TARGET_DIR)/$(PROJECT_NAME)

# SSH parameters
SSH_HOST ?= a5.millennium.berkeley.edu
SSH_USER ?= shashank.anand

# Run sim parameters
SSH ?= 0

# Runtime library
RT_LIB := $(RT_LIB_DIR)/lib$(target).a

.PHONY: all relink runtime clean clean-link clean-lib run-sim run-sim-ssh

include configs/$(target)/$(target).mk

# Build targets
lib$(target): $(RT_LIB)

$(RT_LIB):
	@$(MAKE) -C $(RT_LIB_DIR)

$(BUILD_DIR)/build.ninja: CMakeLists.txt
	@cmake -S . -B $(BUILD_DIR) -G Ninja
	@touch $@

$(APP_EXE): $(BUILD_DIR)/build.ninja
	@ninja -C $(BUILD_DIR)

$(baked): | $(APP_EXE)
	@mkdir -p $(@D)
	@echo "[Bake]  Cooking scene -> $@"
	@$(APP_EXE) -l $(l) -m $(m) -d $(d) -o $(o) -R $(R) -r $(w) $(h) -S $(S) -f $@ $(scene)

$(BAKED_OBJ): | $(baked)
	@echo "[Compile] $(target) baked data"
	@$(CC) $(CFLAGS) -c $(baked) -o $@

$(PT_OBJ): $(BAKED_OBJ) | $(TARGET_LIB_DEP)
	@mkdir -p $(@D)
	@echo "[Link] $(target) $(PROJECT_NAME)"
	@$(CC) $(CFLAGS) $(PT_TARGET) $< $(LDFLAGS) -o $@

$(PT).dump: $(PT_OBJ)
	@echo "[Objdump] $(target) $(PROJECT_NAME)"
	@$(OBJDUMP) $(OBJDUMP_FLAGS) $< > $@

$(PT).bin: $(PT).dump
	@echo "[Objcopy] $(target) $(PROJECT_NAME).bin"
	@$(OBJCOPY) -O binary $(PT).elf $@

# run targets
run: $(GPU_SIM_BIN)
	$(RUN_CMD)

run-ssh: $(GPU_SIM_BIN)
	$(subst --ssh $(SSH),--ssh 1,$(RUN_CMD))

# Utility targets
relink: clean-link
	@$(MAKE) all

clean-bakes:
	@rm -rf bakes

clean-build:
	@rm -rf $(BUILD_DIR)

clean-lib:
	@$(MAKE) -C $(RT_LIB_DIR) clean

clean-lib-all:
	@for dir in $(LIB_DIR)/*/; do \
		if [ -d "$$dir" ] && [ -f "$$dir/Makefile" ]; then \
			$(MAKE) -C "$$dir" clean; \
		fi; \
	done

clean-link:
	@rm -f $(PT)*

clean: clean-build clean-lib

clean-all: clean-bakes clean-build clean-lib-all
