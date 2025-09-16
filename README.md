# Pathtracer – Vortex Branch

This branch retargets the CUDA path tracer to run on the Muon/Vortex RISC-V GPU architecture. Instead of launching kernels directly on a desktop GPU, a host-side **baker** application serializes a scene into static data, and backend-specific firmware replays the path tracer within a simulator (or accelerator hardware). This document explains the prerequisites, build pipeline, and simulator workflow needed to render scenes on this branch.

---

## Overview

- `Makefile` orchestrates baking scenes, compiling accelerator binaries for each backend (`cuda`, `muon`, `vortex`), and running the resulting program inside a simulator.
- `src/host/application.cpp` builds the BVH and writes out baked data (`*.cpp`/`*.cu`) that embed the entire scene plus renderer state.
- `src/target/{cuda,muon,vortex}/` provide small entry points that call into the shared device code in `src/target/pathtracer.*`.
- `lib/{muon,vortex}/` contain runtime support libraries (startup code, syscalls, printf, etc.) that are linked into the accelerator program.
- `scripts/` houses helpers for streaming simulator logs and converting them into images.
- Generated artifacts live in `bakes/` and `build/`; both are ignored by Git so you can freely experiment.

---

## Prerequisites

### Host Environment

- CMake ≥ 3.14 and Ninja (used to build the baker executable).
- A C++17 compiler (Clang/GCC/MSVC).
- Assimp development headers. If Assimp is not installed, CMake automatically fetches and builds Assimp v5.3.1.
- Python 3.8+ for the helper scripts (`scripts/run_sim.py`, `scripts/pathtrace_postprocess.py`).

### Backend Toolchains

| Target (`make target=…`) | Required environment variables | Purpose |
| --- | --- | --- |
| `cuda` | CUDA toolkit with `nvcc` and a host GPU | Validates the serialized scene locally on NVIDIA hardware. |
| `muon` | `MUON_32` → root of the Muon LLVM toolchain | Produces an ELF for the Cyclotron Muon simulator. |
| `vortex` | `VORTEX_32` → LLVM toolchain root<br>`RISCV_32` → RISC-V GNU toolchain/sysroot | Produces binaries/bitstreams for the Vortex RTL simulator. |

Export the toolchain paths before using a backend, for example:

```bash
export VORTEX_32=$HOME/toolchains/vortex-llvm
export RISCV_32=$HOME/toolchains/riscv32-gcc
export MUON_32=$HOME/toolchains/muon-llvm   # only needed for target=muon
```

Simulators may run locally or over SSH. Adjust these environment variables as needed:

- `SSH_USER`, `SSH_HOST` – remote login when `SSH=1`.
- `GPU_SIM_DIR` / `GPU_SIM` – location of the simulator executable (Cyclotron wrapper for Muon, `rtlsim` for Vortex).
- `GPU_SIM_BIN` – which artifact to send to the simulator (`.elf` or `.bin`). Defaults are set per backend in `configs/<target>/<target>.mk`.

---

## Repository Layout

| Path | Description |
| --- | --- |
| `CMakeLists.txt` | Configures and builds the host-side baker, auto-fetching Assimp when necessary. |
| `Makefile` | Main entry point for baking, compiling, running, and cleaning. |
| `configs/` | Toolchain and simulator settings per backend (`cuda`, `muon`, `vortex`). |
| `lib/` | Runtime libraries for Muon/Vortex targets, plus shared linker scripts. |
| `scripts/` | Simulator wrappers and post-processing utilities. |
| `src/host/` | Baker source and supporting modules (BVH construction, serialization). |
| `src/target/pathtracer.*` | Shared renderer that each backend invokes. |
| `src/target/{cuda,muon,vortex}/` | Minimal mains and launch glue for each platform. |
| `bakes/` | Auto-generated scene dumps created by the baker. |
| `build/` | Per-scene build outputs (ELF/BIN/DUMP images, simulator logs, rendered PFM). |

---

## Baking & Building

The Makefile exposes the scene configuration via variables (defaults shown below):

| Variable | Description | Default |
| --- | --- | --- |
| `scene` | Path to the glTF/DAE scene file | `gltf/bunny.glb` |
| `w`, `h` | Output resolution | `32`, `32` |
| `l` | Samples per area light | `2` |
| `m` | Maximum ray depth | `10` |
| `o` | Accumulate indirect lighting (`1` = on) | `1` |
| `R` | Enable ReSTIR-GI (`1` = on) | `0` |
| `S` | Serializer type (`0` text, `1` binary) | `0` |
| `f` | Optional explicit bake filename | auto-generated in `bakes/` |
| `target` | Backend (`cuda`, `muon`, `vortex`) | `muon` |

Example: bake a 64×64 bunny scene with ReSTIR and compile for the Vortex simulator.

```bash
make target=vortex scene=gltf/bunny.glb w=64 h=64 l=4 m=8 R=1
```

The Stanford bunny renders quickly and comfortably fits in the RTL simulators, so it is the default scene. Larger assets such as `Chess.glb` are still supported but are best reserved for CUDA validation.

This performs:

1. **Runtime library** – builds `lib/vortex/libvortex.a` if it is missing.
2. **Baker** – configures CMake (generating `build/baker`) and compiles the host application.
3. **Bake** – serializes the scene into `bakes/bunny_64_64_1_vortex.cpp` (or `.cu` for CUDA).
4. **Link** – compiles the baked data with the backend runtime into `build/bunny_64_64_1_vortex_vortex/pathtracer.elf`, and derives `.bin`/`.dump` images as required by the simulator.

Subsequent builds reuse cached artifacts per `(scene, w, h, R, target)` tuple.

### Cleaning

```bash
make clean-build            # remove the CMake build directory
make clean-link             # delete generated pathtracer binaries for the active tuple
make clean-bakes            # remove serialized scene files
make clean-lib target=vortex  # rebuild lib/vortex on the next run
make clean-all              # nuke bakes, build artifacts, and runtime libs
```

---

## Running Simulators

After a successful build you can stream simulator output and capture the rendered image via `make run` (local) or `make run-ssh` (remote). Both targets delegate to `scripts/run_sim.py`.

```bash
# Local Vortex RTL simulation (override GPU_SIM_DIR if needed)
make target=vortex run GPU_SIM_DIR=$HOME/vortex/build/sim/rtlsim

# Remote Muon simulation over SSH
make target=muon run-ssh SSH=1 SSH_USER=myuser SSH_HOST=remote.host \
     GPU_SIM_DIR=/opt/cyclotron
```

`run_sim.py` steps through the following:

1. Optionally copies the compiled binary to the remote host (unless `--no-scp` is specified).
2. Launches the simulator, streaming stdout into `build/<scene>_<w>_<h>_<R>_<target>/sim_stdout.log` until it encounters the sentinel string `RAYTRACING_COMPLETE`.
3. Invokes `scripts/pathtrace_postprocess.py` to parse the `PIXEL_BUFFER_*` output block and write a floating-point `render.pfm` image to the build folder.

Tune the backend-specific variables in `configs/<target>/<target>.mk` if your simulator command, sentinel string, or output expectations differ.

---

## Make Targets

| Command | Description |
| --- | --- |
| `make target=<backend>` | Bake the configured scene and build the accelerator binary for `cuda`, `muon`, or `vortex`. |
| `make target=<backend> run` | Launch the simulator locally using the artifacts from the configured tuple. |
| `make target=<backend> run-ssh SSH=1` | Launch the simulator on a remote host via SSH (requires `SSH_USER`/`SSH_HOST`). |
| `make clean-build` | Remove the CMake build tree that produces `baker`. |
| `make clean-bakes` | Delete generated scene dumps in `bakes/`. |
| `make clean-link` | Remove compiled accelerator binaries for the current tuple. |
| `make clean-lib target=<backend>` | Rebuild the corresponding runtime library on the next invocation. |
| `make clean-all` | Wipe bakes, build artifacts, and runtime libraries. |

All targets inherit the Make variables described earlier (`scene`, `w`, `h`, `R`, etc.). Override them directly on the command line when invoking the desired target.

---

## Backend Environment Variables

### `target=cuda`

- `PATH` must expose `nvcc` (CUDA toolkit). Set `CUDA_HOME`/`PATH` according to your installation.
- Optional: `DEBUG=1` to enable verbose simulator logging from `run_sim.py`.

### `target=muon`

- `MUON_32` – root of the Muon LLVM toolchain (provides `clang++`, `lld`, and the sysroot).
- `GPU_SIM_DIR` – directory containing the Cyclotron simulator (defaults to `/home/shashank/radiance/cyclotron`).
- `GPU_SIM` – simulator executable or wrapper (defaults to `scripts/cyclotron_wrapper.sh`).
- `GPU_SIM_BIN` – binary passed to the simulator (`$(PT).elf` by default).
- `SSH_USER`, `SSH_HOST` – required when invoking `run-ssh` with `SSH=1`.
- Optional: `SSH=1`, `DEBUG=1`, `POST_PROCESS_SCRIPT`, `STDOUT_SENTINEL`, `STDOUT_SKIP_STR` to customize log handling.

### `target=vortex`

- `VORTEX_32` – root of the Vortex LLVM toolchain.
- `RISCV_32` – root of the RISC-V GNU toolchain/sysroot used for linking.
- `GPU_SIM_DIR` – directory containing the Vortex simulator (`/home/eecs/shashank.anand/vortex/build/sim/rtlsim` by default).
- `GPU_SIM` – simulator binary (defaults to `$(GPU_SIM_DIR)/rtlsim`).
- `GPU_SIM_BIN` – binary passed to the simulator (`$(PT).bin` by default).
- `SSH_USER`, `SSH_HOST` – required when `run-ssh` is used with `SSH=1`.
- Optional: `SSH=1`, `DEBUG=1`, `POST_PROCESS_SCRIPT`, `STDOUT_SENTINEL`, `STDOUT_SKIP_STR` as with the Muon backend.
